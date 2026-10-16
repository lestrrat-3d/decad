package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/triangulation"
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
	if !prismUnionBossShellSimilarity(&src) || !prismshell.BossRectArithmeticExact(src.plateRect, t) ||
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
	if !prismUnionBossShellLevels(src, t) ||
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

func prismUnionBossShellLevels(src prismUnionBossShellSource, t float64) bool {
	pt := func(v float64) *big.Rat { return proofarith.FloatRat(v) }
	return new(big.Rat).Add(pt(src.plateBottom), pt(t)).Cmp(pt(src.bossBottom)) < 0 &&
		pt(src.bossBottom).Cmp(pt(src.interfaceZ)) < 0 &&
		pt(src.interfaceZ).Cmp(pt(src.bossTop)) < 0 &&
		new(big.Rat).Sub(pt(src.interfaceZ), pt(src.bossBottom)).Cmp(pt(t)) == 0
}

func prismUnionBossShellSimilarity(src *prismUnionBossShellSource) bool {
	basis := src.boss.Xform.Basis()
	c, s := basis.EX.X, basis.EX.Y
	if basis.EX.Z != 0 || basis.EY != (r3.Vec{X: -s, Y: c}) ||
		basis.EZ != (r3.Vec{Z: 1}) || !proofbound.FiniteVec(basis.EX) ||
		!proofbound.FiniteVec(src.boss.Xform.Translation()) {
		return false
	}
	length2 := new(big.Rat).Add(
		new(big.Rat).Mul(proofarith.FloatRat(c), proofarith.FloatRat(c)),
		new(big.Rat).Mul(proofarith.FloatRat(s), proofarith.FloatRat(s)))
	src.unitLo, src.unitHi = proofbound.RatSqrtDown(length2), proofbound.RatSqrtUp(length2)
	return src.unitLo > 0.99 && src.unitHi < 1.01
}

type prismUnionBossShellMesh struct {
	verts          []r3.Vec
	tris           [][3]int
	src            []int
	groups         []facetGroup
	beta           []float64
	maxVertexError float64
	rootStart      int
	bossPointEnd   int
}

func buildPrismUnionBossShell(ctx context.Context, source *Body, fp facetedPayload,
	src prismUnionBossShellSource, t float64) (*Body, error) {
	ref := source.n.Doc.NextProducer
	mesh, err := prismUnionBossShellMeshOf(ctx, ref, fp, src, t)
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	data, err := revolvemesh.RequireRevolveFacetAreas(budget, mesh.verts, mesh.tris, 0)
	if err != nil {
		return nil, err
	}
	if err := revolvemesh.RevolveContactAudit(budget, data, mesh.tris, 0); err != nil {
		return nil, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, mesh.verts, mesh.tris)
	if err != nil {
		return nil, err
	}
	trueVolume := prismUnionBossShellTrueVolume(src, t)
	if trueVolume.Lo == nil || trueVolume.Lo.Sign() <= 0 ||
		len(audit.CompVol) != 1 || audit.CompVol[0].Sign() <= 0 {
		return nil, fmt.Errorf("%w: boss shell has no proven positive occupied volume", ErrUnsupported)
	}
	curve := prismUnionBossShellCurveBound(t)
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
	station := mesh.maxVertexError
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
	for i := mesh.rootStart; i < mesh.bossPointEnd; i++ {
		mesh.beta[i] = proofbound.AbsSumUpper(mesh.beta[i], curve)
	}
	bound := facetproof.FacetBoundMax(mesh.tris, mesh.beta)
	// The root patch's curved area is no larger than the boss perimeter times
	// sqrt(2) times a quarter-circle arc. Bounding both true and held root
	// patches by twice that value covers their area difference independently
	// of any cancellation across the four sides.
	rootAreaSlack := proofbound.ProductUpper(
		proofbound.ProductUpper(4, perimeter), proofbound.ProductUpper(t, proofbound.RatFloatUp(proofbound.PiUpper)))
	payload := facetedPayload{
		Verts: mesh.verts, Tris: mesh.tris, Src: mesh.src, Groups: mesh.groups,
		VertexBound: mesh.beta, MeshBound: bound, VolSymDiff: volumeBound,
		AreaSlack: proofbound.AbsSumUpper(fp.AreaSlack, rootAreaSlack),
		DPair:     facetedrecord.SourceDiameterUpper(mesh.verts, bound),
		Xform:     r3.Identity(), ContactAudited: true,
	}
	return bodyResult(facetedrecord.BuildWithProof(ctx, envOf(documentNode(documentHandle(source.n.Doc)), ref), payload, audit, measurementToInternal(Measurement{}), nil))
}

func prismUnionBossShellMeshOf(ctx context.Context, ref producerID, fp facetedPayload,
	src prismUnionBossShellSource, t float64) (prismUnionBossShellMesh, error) {
	var out prismUnionBossShellMesh
	out.verts = append(out.verts, fp.Verts...)
	out.beta = append(out.beta, fp.VertexBound...)
	out.groups = append(out.groups, fp.Groups...)
	addGroup := func(role string, planar bool) int {
		i := len(out.groups)
		out.groups = append(out.groups, facetGroup{Origins: []provenance.Ref{{Producer: ref, Role: role}}, Planar: planar})
		return i
	}
	// The source's top cap is replaced by the opening's narrow horizontal rim.
	for i, tri := range fp.Tris {
		if fp.Src[i] == src.topGroup {
			continue
		}
		out.tris = append(out.tris, tri)
		out.src = append(out.src, fp.Src[i])
	}
	add := func(v r3.Vec, beta float64) int {
		i := len(out.verts)
		out.verts = append(out.verts, v)
		out.beta = append(out.beta, beta)
		out.maxVertexError = max(out.maxVertexError, beta)
		return i
	}
	tri := func(a, b, c, group int) {
		out.tris = append(out.tris, [3]int{a, b, c})
		out.src = append(out.src, group)
	}
	quad := func(bottom, top [4]int, groups [4]int) {
		for i := range 4 {
			j := (i + 1) % 4
			tri(bottom[i], top[i], top[j], groups[i])
			tri(bottom[i], top[j], bottom[j], groups[i])
		}
	}
	sideGroups := func(role string, planar bool) [4]int {
		var groups [4]int
		for i := range groups {
			groups[i] = addGroup(fmt.Sprintf("%s(%d)", role, i), planar)
		}
		return groups
	}
	point := func(corner int, qWorld, qError, zWorld, zError float64) (int, error) {
		uv := prismshell.PrismUnionBossShellCorners(src.bossRect)[corner]
		sx, sy := 1.0, 1.0
		if corner == 1 || corner == 2 {
			sx = -1
		}
		if corner >= 2 {
			sy = -1
		}
		qLocal := proofbound.DivUpper(qWorld, src.unitLo)
		qLoRat := new(big.Rat).Quo(new(big.Rat).Sub(
			proofarith.FloatRat(qWorld), proofarith.FloatRat(qError)), proofarith.FloatRat(src.unitHi))
		qLo := proofbound.RatFloatDown(qLoRat)
		qBound := proofbound.ProvenUpRound(qLocal - qLo)
		localZ := zWorld - src.boss.Frame.Origin().Z
		uLocal, vLocal := uv[0]+sx*qLocal, uv[1]+sy*qLocal
		uBound := proofbound.AbsSumUpper(qBound, proofbound.ExactSumRound(uLocal, uv[0], sx*qLocal))
		vBound := proofbound.AbsSumUpper(qBound, proofbound.ExactSumRound(vLocal, uv[1], sy*qLocal))
		zBound := proofbound.AbsSumUpper(zError,
			proofbound.ExactSumRound(localZ, zWorld, -src.boss.Frame.Origin().Z))
		v := src.boss.Point(uLocal, vLocal, localZ)
		beta := src.boss.PointBound(
			proofbound.MeasuredScalar(uLocal, uBound),
			proofbound.MeasuredScalar(vLocal, vBound),
			proofbound.MeasuredScalar(localZ, zBound))
		if !proofbound.FiniteVec(v) || proofbound.IsNonFinite(beta) || math.Abs(v.Z-zWorld) > beta {
			return 0, fmt.Errorf("%w: boss shell station has no bounded horizontal lift", ErrUnsupported)
		}
		if (zWorld == src.bossBottom || zWorld == src.interfaceZ || zWorld == src.bossTop) &&
			v.Z != zWorld {
			return 0, fmt.Errorf("%w: boss shell rim or ledge moved off its source plane", ErrUnsupported)
		}
		return add(v, beta), nil
	}
	out.rootStart = len(out.verts)
	var upperTop, upperBase [4]int
	for i := range 4 {
		var err error
		upperTop[i], err = point(i, t, 0, src.bossTop, 0)
		if err != nil {
			return out, err
		}
		upperBase[i], err = point(i, t, 0, src.interfaceZ, 0)
		if err != nil {
			return out, err
		}
	}
	for i := range 4 {
		j := (i + 1) % 4
		g := addGroup(fmt.Sprintf("shellRim(%d)", i), true)
		tri(src.outer[i], src.outer[j], upperTop[j], g)
		tri(src.outer[i], upperTop[j], upperTop[i], g)
	}
	quad(upperBase, upperTop, sideGroups("shellUpperWall", true))
	prev := upperBase
	rootGroups := sideGroups("shellRootBand", false)
	for k := prismUnionBossShellStations - 1; k >= 0; k-- {
		w := big.NewRat(int64(k), prismUnionBossShellStations)
		w2 := new(big.Rat).Mul(w, w)
		den := new(big.Rat).Add(big.NewRat(1, 1), w2)
		qRat := new(big.Rat).Quo(new(big.Rat).Mul(proofarith.FloatRat(t),
			new(big.Rat).Mul(big.NewRat(2, 1), w2)), den)
		sRat := new(big.Rat).Quo(new(big.Rat).Mul(proofarith.FloatRat(t),
			new(big.Rat).Mul(big.NewRat(2, 1), w)), den)
		zRat := new(big.Rat).Add(proofarith.FloatRat(src.bossBottom), sRat)
		q := proofbound.RatFloatUp(qRat)
		z, _ := zRat.Float64()
		qError := proofarith.RationalFloatError(qRat, q)
		zError := proofarith.RationalFloatError(zRat, z)
		if k == 0 {
			q = 0
			z = src.bossBottom
			qError, zError = 0, 0
		}
		var ring [4]int
		for i := range 4 {
			var err error
			ring[i], err = point(i, q, qError, z, zError)
			if err != nil {
				return out, err
			}
		}
		quad(ring, prev, rootGroups)
		prev = ring
	}
	root := prev
	out.bossPointEnd = len(out.verts)
	plateInner := src.plateRect.Inset(t)
	plateCorners := prismshell.PrismUnionBossShellCorners(plateInner)
	var ledgeOuter, floor [4]int
	for i, uv := range plateCorners {
		ledgeOuter[i] = add(r3.Vec{X: uv[0], Y: uv[1], Z: src.bossBottom}, 0)
		floor[i] = add(r3.Vec{X: uv[0], Y: uv[1], Z: src.plateBottom + t}, 0)
	}
	pts := make([]triangulation.Point2, len(out.verts))
	for i, v := range out.verts {
		pts[i] = triangulation.Point2{U: v.X, V: v.Y}
	}
	ledge, err := triangulation.Triangulate(ctx, pts, [][]int{
		{ledgeOuter[0], ledgeOuter[1], ledgeOuter[2], ledgeOuter[3]},
		{root[3], root[2], root[1], root[0]},
	})
	if err != nil {
		return out, fmt.Errorf("%w: boss shell ledge triangulation: %v", ErrUnsupported, err)
	}
	ledgeGroup := addGroup("shellLedge", true)
	for _, tr := range ledge {
		tri(tr[0], tr[2], tr[1], ledgeGroup)
	}
	quad(floor, ledgeOuter, sideGroups("shellLowerWall", true))
	floorGroup := addGroup("shellFloor", true)
	tri(floor[0], floor[1], floor[2], floorGroup)
	tri(floor[0], floor[2], floor[3], floorGroup)
	return out, nil
}

func prismUnionBossShellCurveBound(t float64) float64 {
	// The planar quarter circle (q(w), s(w)) has second-derivative norm at
	// most 4t on 0 <= w <= 1. Linear interpolation at matching w across a
	// cell of width 1/N differs by at most 4t/(8N²) = t/(2N²). Insetting
	// both rectangle coordinates stretches this gap by at most sqrt(2),
	// rounded safely to 3/2.
	n := float64(prismUnionBossShellStations)
	return proofbound.DivUpper(proofbound.ProductUpper(1.5, t), 2*n*n)
}

func prismUnionBossShellTrueVolume(src prismUnionBossShellSource, t float64) proofbound.RatInterval {
	point := func(v float64) proofbound.RatInterval { return proofbound.PointInterval(proofarith.FloatRat(v)) }
	add, sub, mul := proofbound.IntervalAdd, proofbound.IntervalSub, proofbound.IntervalMul
	scale := proofbound.Interval(proofarith.FloatRat(src.unitLo), proofarith.FloatRat(src.unitHi))
	w := mul(point(src.bossRect.Width()), scale)
	h := mul(point(src.bossRect.Height()), scale)
	tv := point(t)
	t2, t3 := mul(tv, tv), mul(mul(tv, tv), tv)
	plateArea := mul(point(src.plateRect.Width()), point(src.plateRect.Height()))
	lowerRect := src.plateRect.Inset(t)
	lowerArea := mul(point(lowerRect.Width()), point(lowerRect.Height()))
	outer := add(mul(plateArea, point(src.interfaceZ-src.plateBottom)),
		mul(mul(w, h), point(src.bossTop-src.interfaceZ)))
	lower := mul(lowerArea, point(src.bossBottom-src.plateBottom-t))
	upper := mul(mul(sub(w, mul(point(2), tv)), sub(h, mul(point(2), tv))),
		point(src.bossTop-src.interfaceZ))
	pi := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
	widths := add(w, h)
	band := add(sub(mul(mul(w, h), tv), mul(mul(point(2), widths), t2)),
		add(mul(mul(mul(widths, pi), t2), point(0.5)),
			sub(mul(t3, proofbound.Interval(big.NewRat(20, 3), big.NewRat(20, 3))),
				mul(mul(point(2), pi), t3))))
	return sub(outer, add(add(lower, upper), band))
}
