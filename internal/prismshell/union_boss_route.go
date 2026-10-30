package prismshell

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

const prismUnionBossShellStations = 16

// prismUnionBossShellSource is a plate and a strictly nested boss in one
// horizontal scene. The boss's held XY basis is an exact similarity over its
// stored dyadic coefficients; its scale is bracketed before any offset is
// mapped into the boss's local rectangle.
type prismUnionBossShellSource struct {
	plate, boss                                  prismrecord.Payload
	plateRect, bossRect                          BossRect
	plateBottom, interfaceZ, bossBottom, bossTop float64
	unitLo, unitHi                               float64
	outer                                        [4]int
	topGroup                                     int
}

// PrismUnionBossShell shells the plate-and-boss union result inward through
// its single removed boss cap, or declines when the receiver is not that
// authenticated construction.
func PrismUnionBossShell(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload,
	removed []*topo.Face, inward bool, t, tDelta float64) (*topo.Body, bool, error) {
	if len(removed) != 1 || !inward || tDelta != 0 || !(t > 0) ||
		proofbound.IsNonFinite(t) || fp.PrismUnionFillet == nil || fp.Xform != r3.Identity() {
		return nil, false, nil
	}
	for _, pair := range [][2]*topo.Body{{fp.PrismUnionFillet.First, fp.PrismUnionFillet.Second},
		{fp.PrismUnionFillet.Second, fp.PrismUnionFillet.First}} {
		source, ok := prismUnionBossShellAuthenticate(b, fp, pair[0], pair[1], removed[0], t)
		if !ok {
			continue
		}
		payload, audit, err := PrismUnionBossShellPayload(ctx, env.Ref, BossMeshInput{
			Verts: fp.Verts, VertexBound: fp.VertexBound, Groups: fp.Groups,
			Tris: fp.Tris, Src: fp.Src, Boss: source.boss,
			PlateRect: source.plateRect, BossRect: source.bossRect,
			PlateBottom: source.plateBottom, InterfaceZ: source.interfaceZ,
			BossBottom: source.bossBottom, BossTop: source.bossTop,
			UnitLo: source.unitLo, UnitHi: source.unitHi,
			Outer: source.outer, TopGroup: source.topGroup,
			Stations: prismUnionBossShellStations,
		}, fp.VolSymDiff, fp.AreaSlack, t)
		if err != nil {
			return nil, true, err
		}
		out, err := facetedrecord.BuildWithProof(ctx, env, payload, audit, measurement.Measurement{}, nil)
		return out, true, err
	}
	return nil, false, nil
}

func prismUnionBossShellAuthenticate(result *topo.Body, fp facetedrecord.Payload, plateBody, bossBody *topo.Body,
	opening *topo.Face, t float64) (prismUnionBossShellSource, bool) {
	var src prismUnionBossShellSource
	if plateBody == nil || bossBody == nil || opening == nil || opening.Body != result ||
		len(fp.Verts) != len(fp.VertexBound) || len(fp.Tris) != len(fp.Src) ||
		len(fp.Tris) != len(fp.FaceOf) || fp.VolSymDiff < 0 ||
		proofbound.IsNonFinite(fp.VolSymDiff) {
		return src, false
	}
	var ok bool
	src.plate, ok = plateBody.Payload.(prismrecord.Payload)
	if !ok || src.plate.SectionDelta != 0 || src.plate.Z0Delta != 0 || src.plate.Z1Delta != 0 ||
		src.plate.SurfaceResult || src.plate.Xform != r3.Identity() ||
		src.plate.Frame.U() != (r3.Vec{X: 1}) || src.plate.Frame.V() != (r3.Vec{Y: 1}) ||
		src.plate.Frame.N() != (r3.Vec{Z: 1}) {
		return src, false
	}
	src.boss, ok = bossBody.Payload.(prismrecord.Payload)
	if !ok || src.boss.SectionDelta != 0 || src.boss.Z0Delta != 0 || src.boss.Z1Delta != 0 ||
		src.boss.SurfaceResult || src.boss.Frame.U() != (r3.Vec{X: 1}) ||
		src.boss.Frame.V() != (r3.Vec{Y: 1}) || src.boss.Frame.N() != (r3.Vec{Z: 1}) {
		return src, false
	}
	src.plateRect, ok = RecordedBossRect(src.plate.Profile)
	if !ok {
		return src, false
	}
	src.bossRect, ok = RecordedBossRect(src.boss.Profile)
	if !ok {
		return src, false
	}
	src.unitLo, src.unitHi, ok = PrismUnionBossShellSimilarity(src.boss.Xform)
	if !ok || !BossRectArithmeticExact(src.plateRect, t) ||
		!BossRectArithmeticExact(src.bossRect, 0) {
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
	if !PrismUnionBossShellLevels(src.plateBottom, src.bossBottom,
		src.interfaceZ, src.bossTop, t) ||
		!PrismUnionBossShellRoom(src.bossRect.Width(), src.unitLo, t) ||
		!PrismUnionBossShellRoom(src.bossRect.Height(), src.unitLo, t) {
		return src, false
	}
	// The entire boss, enlarged by its source-point bound, must lie strictly
	// inside the plate's inward wall. The root band then cannot meet an outer
	// side or the lower floor.
	plateInner := src.plateRect.Inset(t)
	var corners [4]r3.Vec
	for i, uv := range PrismUnionBossShellCorners(src.bossRect) {
		p := src.boss.Point(uv[0], uv[1], src.boss.Z1)
		beta := src.boss.PointBound(proofbound.ExactScalar(uv[0]),
			proofbound.ExactScalar(uv[1]), proofbound.ExactScalar(src.boss.Z1))
		if !proofbound.FiniteVec(p) || proofbound.IsNonFinite(beta) ||
			!PrismUnionBossShellInside(p.X, beta, plateInner.U0, plateInner.U1) ||
			!PrismUnionBossShellInside(p.Y, beta, plateInner.V0, plateInner.V1) ||
			p.Z != src.bossTop {
			return src, false
		}
		corners[i] = p
	}
	faces := topobuild.Faces(result)
	openingIdx := -1
	for i, f := range faces {
		if f == opening {
			openingIdx = i
			break
		}
	}
	// A mesh result face holds its operand face's own roles in Origins and any
	// inherited Boolean roles apart in Upstream (prism-boolean §11), so a boss
	// whose top inherited a role still shows exactly its own cap here.
	if openingIdx < 0 || opening.Surface.Kind() != surfacegeom.KindPlane && !opening.HeldPlanar || len(opening.Origins) != 1 ||
		opening.Origins[0] != capEndRef(bossBody) {
		return src, false
	}
	src.outer, src.topGroup, ok = PrismUnionBossShellCapOwners(
		fp, openingIdx, capEndRef(bossBody), corners)
	if !ok {
		return src, false
	}
	return src, true
}

func capEndRef(b *topo.Body) provenance.Ref {
	return provenance.Ref{Producer: b.Origin.Producer, Role: provenance.RoleCapEnd}
}
