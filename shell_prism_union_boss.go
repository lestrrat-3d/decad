package decad

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/r3"
)

const prismUnionBossShellStations = 16

// prismUnionBossShellSource is a plate and a strictly nested boss in one
// horizontal scene. The boss's held XY basis is an exact similarity over its
// stored dyadic coefficients; its scale is bracketed before any offset is
// mapped into the boss's local rectangle.
type prismUnionBossShellSource struct {
	plate, boss                                  prismPayload
	plateRect, bossRect                          prismshell.BossRect
	plateBottom, interfaceZ, bossBottom, bossTop float64
	unitLo, unitHi                               float64
	outer                                        [4]int
	topGroup                                     int
}

func (b *Body) shellPrismUnionBoss(ctx context.Context, fp facetedPayload,
	removed []*Face, sense ShellSense, t, tDelta float64) (*Body, bool, error) {
	if len(removed) != 1 || sense != Inward || tDelta != 0 || !(t > 0) ||
		proofbound.IsNonFinite(t) || fp.PrismUnionFillet == nil || fp.Xform != r3.Identity() {
		return nil, false, nil
	}
	for _, pair := range [][2]*Body{{bodyHandle(fp.PrismUnionFillet.First), bodyHandle(fp.PrismUnionFillet.Second)},
		{bodyHandle(fp.PrismUnionFillet.Second), bodyHandle(fp.PrismUnionFillet.First)}} {
		source, ok := prismUnionBossShellAuthenticate(b, fp, pair[0], pair[1], removed[0], t)
		if !ok {
			continue
		}
		out, err := buildPrismUnionBossShell(ctx, b, fp, source, t)
		if err != nil {
			return nil, true, err
		}
		body, err := commitModifyResult(ctx, b, out)
		return body, true, err
	}
	return nil, false, nil
}

func prismUnionBossShellAuthenticate(result *Body, fp facetedPayload, plateBody, bossBody *Body,
	opening *Face, t float64) (prismUnionBossShellSource, bool) {
	var src prismUnionBossShellSource
	if plateBody == nil || bossBody == nil || opening == nil || opening.n.Body != bodyNode(result) ||
		len(fp.Verts) != len(fp.VertexBound) || len(fp.Tris) != len(fp.Src) ||
		len(fp.Tris) != len(fp.FaceOf) || fp.VolSymDiff < 0 ||
		proofbound.IsNonFinite(fp.VolSymDiff) {
		return src, false
	}
	var ok bool
	src.plate, ok = plateBody.n.Payload.(prismPayload)
	if !ok || src.plate.SectionDelta != 0 || src.plate.Z0Delta != 0 || src.plate.Z1Delta != 0 ||
		src.plate.SurfaceResult || src.plate.Xform != r3.Identity() ||
		src.plate.Frame.U() != (r3.Vec{X: 1}) || src.plate.Frame.V() != (r3.Vec{Y: 1}) ||
		src.plate.Frame.N() != (r3.Vec{Z: 1}) {
		return src, false
	}
	src.boss, ok = bossBody.n.Payload.(prismPayload)
	if !ok || src.boss.SectionDelta != 0 || src.boss.Z0Delta != 0 || src.boss.Z1Delta != 0 ||
		src.boss.SurfaceResult || src.boss.Frame.U() != (r3.Vec{X: 1}) ||
		src.boss.Frame.V() != (r3.Vec{Y: 1}) || src.boss.Frame.N() != (r3.Vec{Z: 1}) {
		return src, false
	}
	src.plateRect, ok = prismshell.RecordedBossRect(src.plate.Profile)
	if !ok {
		return src, false
	}
	src.bossRect, ok = prismshell.RecordedBossRect(src.boss.Profile)
	if !ok {
		return src, false
	}
	src.unitLo, src.unitHi, ok = prismshell.PrismUnionBossShellSimilarity(src.boss.Xform)
	if !ok || !prismshell.BossRectArithmeticExact(src.plateRect, t) ||
		!prismshell.BossRectArithmeticExact(src.bossRect, 0) {
		return src, false
	}
	plateOrigin := src.plate.Frame.Origin()
	bossOrigin := src.boss.Frame.Origin()
	if plateOrigin.X != 0 || plateOrigin.Y != 0 || bossOrigin.X != 0 || bossOrigin.Y != 0 ||
		src.boss.Xform.Translation().Z != 0 {
		return src, false
	}
	src.plateBottom = plateOrigin.Z + src.plate.Z0
	src.interfaceZ = plateOrigin.Z + src.plate.Z1
	src.bossBottom = bossOrigin.Z + src.boss.Z0
	src.bossTop = bossOrigin.Z + src.boss.Z1
	for _, reading := range [][3]float64{{src.plateBottom, plateOrigin.Z, src.plate.Z0},
		{src.interfaceZ, plateOrigin.Z, src.plate.Z1},
		{src.bossBottom, bossOrigin.Z, src.boss.Z0},
		{src.bossTop, bossOrigin.Z, src.boss.Z1}} {
		if proofbound.ExactSumRound(reading[0], reading[1], reading[2]) != 0 {
			return src, false
		}
	}
	if proofbound.ExactSumRound(src.plateBottom+t, src.plateBottom, t) != 0 ||
		proofbound.ExactSumRound(src.bossBottom-src.plateBottom-t,
			src.bossBottom, -src.plateBottom, -t) != 0 ||
		proofbound.ExactSumRound(src.interfaceZ-src.plateBottom,
			src.interfaceZ, -src.plateBottom) != 0 ||
		proofbound.ExactSumRound(src.bossTop-src.interfaceZ,
			src.bossTop, -src.interfaceZ) != 0 {
		return src, false
	}
	if !prismshell.PrismUnionBossShellLevels(src.plateBottom, src.bossBottom,
		src.interfaceZ, src.bossTop, t) ||
		!prismshell.PrismUnionBossShellRoom(src.bossRect.Width(), src.unitLo, t) ||
		!prismshell.PrismUnionBossShellRoom(src.bossRect.Height(), src.unitLo, t) {
		return src, false
	}
	// The entire boss, enlarged by its source-point bound, must lie strictly
	// inside the plate's inward wall. The root band then cannot meet an outer
	// side or the lower floor.
	plateInner := src.plateRect.Inset(t)
	var corners [4]r3.Vec
	for i, uv := range prismshell.PrismUnionBossShellCorners(src.bossRect) {
		p := src.boss.Point(uv[0], uv[1], src.boss.Z1)
		beta := src.boss.PointBound(proofbound.ExactScalar(uv[0]),
			proofbound.ExactScalar(uv[1]), proofbound.ExactScalar(src.boss.Z1))
		if !proofbound.FiniteVec(p) || proofbound.IsNonFinite(beta) ||
			!prismshell.PrismUnionBossShellInside(p.X, beta, plateInner.U0, plateInner.U1) ||
			!prismshell.PrismUnionBossShellInside(p.Y, beta, plateInner.V0, plateInner.V1) ||
			p.Z != src.bossTop {
			return src, false
		}
		corners[i] = p
	}
	faces := result.Faces()
	openingIdx := -1
	for i, f := range faces {
		if f == opening {
			openingIdx = i
			break
		}
	}
	if openingIdx < 0 || !opening.isPlanar() || len(opening.n.Origins) != 1 ||
		opening.n.Origins[0] != refOf(CapEnd(bossBody)) {
		return src, false
	}
	var topVerts map[int]struct{}
	var topTris [][3]int
	for fi, faceIdx := range fp.FaceOf {
		if faceIdx != openingIdx {
			continue
		}
		if topVerts == nil {
			topVerts = map[int]struct{}{}
			src.topGroup = fp.Src[fi]
		}
		if fp.Src[fi] != src.topGroup {
			return src, false
		}
		topTris = append(topTris, fp.Tris[fi])
		for _, vi := range fp.Tris[fi] {
			topVerts[vi] = struct{}{}
		}
	}
	if len(topVerts) != 4 || len(topTris) != 2 || src.topGroup < 0 || src.topGroup >= len(fp.Groups) ||
		len(fp.Groups[src.topGroup].Origins) != 1 || !fp.Groups[src.topGroup].Planar ||
		fp.Groups[src.topGroup].Origins[0] != refOf(CapEnd(bossBody)) {
		return src, false
	}
	for fi, group := range fp.Src {
		if group == src.topGroup && fp.FaceOf[fi] != openingIdx {
			return src, false
		}
	}
	for i, group := range fp.Groups {
		if i == src.topGroup {
			continue
		}
		if slices.Contains(group.Origins, refOf(CapEnd(bossBody))) {
			return src, false
		}
	}
	for i, p := range corners {
		found := -1
		for vi := range topVerts {
			if fp.Verts[vi] == p {
				found = vi
				break
			}
		}
		if found < 0 {
			return src, false
		}
		src.outer[i] = found
	}
	if !prismshell.PrismUnionBossShellCompleteCap(src.outer, topTris) {
		return src, false
	}
	return src, true
}

func buildPrismUnionBossShell(ctx context.Context, source *Body, fp facetedPayload,
	src prismUnionBossShellSource, t float64) (*Body, error) {
	ref := source.n.Doc.NextProducer
	mesh, err := prismshell.PrismUnionBossShellMeshOf(ctx, ref, prismshell.BossMeshInput{
		Verts: fp.Verts, VertexBound: fp.VertexBound, Groups: fp.Groups,
		Tris: fp.Tris, Src: fp.Src, Boss: src.boss,
		PlateRect: src.plateRect, BossRect: src.bossRect,
		PlateBottom: src.plateBottom, InterfaceZ: src.interfaceZ,
		BossBottom: src.bossBottom, BossTop: src.bossTop,
		UnitLo: src.unitLo, UnitHi: src.unitHi,
		Outer: src.outer, TopGroup: src.topGroup,
		Stations: prismUnionBossShellStations,
	}, t)
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	data, err := revolvemesh.RequireRevolveFacetAreas(budget, mesh.Verts, mesh.Tris, 0)
	if err != nil {
		return nil, err
	}
	if err := revolvemesh.RevolveContactAudit(budget, data, mesh.Tris, 0); err != nil {
		return nil, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, mesh.Verts, mesh.Tris)
	if err != nil {
		return nil, err
	}
	trueVolume := prismshell.PrismUnionBossShellTrueVolume(src.plateRect, src.bossRect,
		src.plateBottom, src.interfaceZ, src.bossBottom, src.bossTop,
		src.unitLo, src.unitHi, t)
	if trueVolume.Lo == nil || trueVolume.Lo.Sign() <= 0 ||
		len(audit.CompVol) != 1 || audit.CompVol[0].Sign() <= 0 {
		return nil, fmt.Errorf("%w: boss shell has no proven positive occupied volume", ErrUnsupported)
	}
	curve := prismshell.PrismUnionBossShellCurveBound(t, prismUnionBossShellStations)
	// All source-preserving exterior facets keep their Boolean proof. The new
	// cavity is a family of nested convex rectangles. The root's true
	// quarter-circle surface stays within curve plus the station lift of a
	// chorded surface matched at the same rational curve parameter. This
	// matching lets Z move near the horizontal tangent;
	// a same-Z radial gap would be larger. Every homotopy section remains a
	// strictly nested rectangle. Its swept root area is below 4 P t, where P
	// is the boss perimeter; multiplying by the displacement bounds the
	// occupied-volume change without using a same-Z horizontal gap, which
	// becomes large near the quarter-circle's horizontal tangent. The lower
	// cavity is exact; the upper cavity uses a fixed inset.
	width := proofbound.ProductUpper(src.bossRect.Width(), src.unitHi)
	height := proofbound.ProductUpper(src.bossRect.Height(), src.unitHi)
	perimeter := proofbound.ProductUpper(2, proofbound.AbsSumUpper(width, height))
	station := mesh.MaxVertexError
	rootGap := proofbound.AbsSumUpper(curve, proofbound.ProductUpper(4, station))
	upperGap := proofbound.ProductUpper(4, station)
	if !(rootGap < t/16) {
		return nil, fmt.Errorf("%w: boss shell root stations are too coarse for a bounded cavity", ErrUnsupported)
	}
	rootVol := proofbound.ProductUpper(
		proofbound.ProductUpper(4, proofbound.ProductUpper(perimeter, t)), rootGap)
	upperVol := proofbound.ProductUpper(src.bossTop-src.interfaceZ,
		proofbound.AbsSumUpper(proofbound.ProductUpper(perimeter, upperGap),
			proofbound.ProductUpper(4, proofbound.ProductUpper(upperGap, upperGap))))
	capArea := proofbound.AbsSumUpper(
		proofbound.ProductUpper(width, height),
		proofbound.ProductUpper(src.plateRect.Inset(t).Width(), src.plateRect.Inset(t).Height()))
	levelVol := proofbound.ProductUpper(proofbound.ProductUpper(4, capArea), station)
	volumeBound := proofbound.AbsSumUpper(fp.VolSymDiff, rootVol, upperVol, levelVol)
	heldVolume := audit.CompVol[0]
	if proofbound.IntervalAbsUpper(proofbound.IntervalSub(trueVolume,
		proofbound.PointInterval(heldVolume))).Cmp(proofarith.FloatRat(volumeBound)) > 0 {
		return nil, fmt.Errorf("%w: boss shell held volume exceeds its occupied-volume allowance", ErrUnsupported)
	}
	for i := mesh.RootStart; i < mesh.BossPointEnd; i++ {
		mesh.VertexBound[i] = proofbound.AbsSumUpper(mesh.VertexBound[i], curve)
	}
	bound := facetproof.FacetBoundMax(mesh.Tris, mesh.VertexBound)
	// The root patch's curved area is no larger than the boss perimeter times
	// sqrt(2) times a quarter-circle arc. Bounding both true and held root
	// patches by twice that value covers their area difference independently
	// of any cancellation across the four sides.
	rootAreaSlack := proofbound.ProductUpper(
		proofbound.ProductUpper(4, perimeter), proofbound.ProductUpper(t, proofbound.RatFloatUp(proofbound.PiUpper)))
	payload := facetedPayload{
		Verts: mesh.Verts, Tris: mesh.Tris, Src: mesh.Src, Groups: mesh.Groups,
		VertexBound: mesh.VertexBound, MeshBound: bound, VolSymDiff: volumeBound,
		AreaSlack: proofbound.AbsSumUpper(fp.AreaSlack, rootAreaSlack),
		DPair:     facetedrecord.SourceDiameterUpper(mesh.Verts, bound),
		Xform:     r3.Identity(), ContactAudited: true,
	}
	return bodyResult(facetedrecord.BuildWithProof(ctx, envOf(documentNode(documentHandle(source.n.Doc)), ref), payload, audit, measurementToInternal(Measurement{}), nil))
}
