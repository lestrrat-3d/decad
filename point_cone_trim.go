package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// A point cone trim uses the original point-loft fan as its radial domain.
// Chained trims always start from that same authenticated fan rather than a
// previously rounded Boolean mesh.
type pointConeTrimRecord struct {
	base  facetedPayload
	lower *pointConeLimit
	upper *pointConeLimit
}

type pointConeLimit struct {
	apex, far, slope *big.Rat
	surface          Cone
	origins          []FeatureRef
	denoted          *surfacenormal.Revolved
	reversed         bool
}

type pointConeEdge struct{ a, b int }

type pointConeSide struct{ u, v, group int }

type pointConeSample struct {
	q, alpha           float64
	qError, pointError float64
}

func pointConeFacetGroup(cone *pointConeLimit, flip bool) facetGroup {
	denoted := cone.denoted
	if flip {
		// A straight revolve wall's denotation stores its oriented meridian
		// runs. Its Allow method reads that orientation directly.
		denotedCopy := *denoted
		denotedCopy.Runs = append([][2]proofbound.RatInterval(nil), denoted.Runs...)
		for i := range denotedCopy.Runs {
			denotedCopy.Runs[i][0] = proofbound.IntervalNeg(denotedCopy.Runs[i][0])
			denotedCopy.Runs[i][1] = proofbound.IntervalNeg(denotedCopy.Runs[i][1])
		}
		denoted = &denotedCopy
	}
	return facetGroup{origins: cone.origins, surface: cone.surface,
		denoted: denoted, reversed: cone.reversed != flip}
}

func orderedPointConeEdge(a, b int) pointConeEdge {
	if a > b {
		a, b = b, a
	}
	return pointConeEdge{a, b}
}

// exactPointCone recognizes the unplaced, full, straight-meridian cone whose
// axis is world X. Every parameter is read from the authenticated meridian;
// the rounded public Cone tag is used only for surface reporting.
func exactPointCone(b *Body) (*pointConeLimit, bool) {
	rp, ok := b.payload.(revolvePayload)
	if !ok || !rp.full || rp.surfaceResult || rp.sectionDelta != 0 || rp.xform != r3.Identity() ||
		len(rp.profile.Holes) != 0 || len(rp.profile.Outer.Segments) != 3 ||
		rp.frame.Origin() != (r3.Vec{}) || rp.frame.U() != (r3.Vec{X: 1}) ||
		rp.frame.V() != (r3.Vec{Y: 1}) || rp.ax.AU != 0 || rp.ax.AV != 0 ||
		rp.ax.DU != 1 || rp.ax.DV != 0 || rp.ax.AUBound != 0 || rp.ax.AVBound != 0 ||
		rp.ax.DUBound != 0 || rp.ax.DVBound != 0 {
		return nil, false
	}
	var apex, far, radius float64
	far = math.Inf(1)
	axisCount, rimCount := 0, 0
	for _, segment := range rp.profile.Outer.Segments {
		line, ok := segment.(lineSeg)
		if !ok {
			return nil, false
		}
		for _, pt := range [...]Point2{line.Start, line.End} {
			if pt.V == 0 {
				axisCount++
				apex = max(apex, pt.U)
				far = min(far, pt.U)
			} else if pt.V > 0 {
				rimCount++
				radius = max(radius, pt.V)
				far = min(far, pt.U)
			} else {
				return nil, false
			}
		}
	}
	if axisCount != 4 || rimCount != 2 || !proofbound.FiniteVec(r3.Vec{X: apex, Y: far, Z: radius}) ||
		apex <= 0 || far >= 0 || radius <= 0 {
		return nil, false
	}
	for _, segment := range rp.profile.Outer.Segments {
		line, ok := segment.(lineSeg)
		if !ok {
			return nil, false
		}
		for _, pt := range [...]Point2{line.Start, line.End} {
			if pt.V == 0 && pt.U != apex && pt.U != far || pt.V > 0 &&
				(pt.U != far || pt.V != radius) {
				return nil, false
			}
		}
	}
	var surface Cone
	var origins []FeatureRef
	var denoted *surfacenormal.Revolved
	var reversed bool
	for _, face := range b.Faces() {
		if cone, ok := face.Surface().(Cone); ok {
			surface, origins = cone, face.Origins()
			denoted, reversed = face.denoted, face.reversed
			break
		}
	}
	if len(origins) == 0 || denoted == nil {
		return nil, false
	}
	a := proofarith.FloatRat(apex)
	k := new(big.Rat).Quo(new(big.Rat).Sub(a, proofarith.FloatRat(far)), proofarith.FloatRat(radius))
	return &pointConeLimit{apex: a, far: proofarith.FloatRat(far), slope: k,
		surface: surface, origins: origins, denoted: denoted, reversed: reversed}, true
}

// tryPointConeBoolean is an accept-only structural arm. It does not change
// the mesh Boolean fallback for any pair outside this exact source class.
func tryPointConeBoolean(ctx context.Context, op meshbool.OperationKind, d *Document,
	ref producerID, target, tool *Body) (*Body, bool, error) {
	if op != meshbool.OpCut && op != meshbool.OpIntersect {
		return nil, false, nil
	}
	cone, ok := exactPointCone(tool)
	if !ok {
		return nil, false, nil
	}
	fp, ok := target.payload.(facetedPayload)
	if !ok || fp.xform != r3.Identity() {
		return nil, false, nil
	}
	record := pointConeTrimRecord{base: fp}
	if fp.pointCone != nil {
		record = *fp.pointCone
	} else if fp.pointSection == nil || fp.pointSection.apex != (r3.Vec{}) {
		return nil, false, nil
	}
	if !pointConeSourceAboveFarCap(record.base, cone) {
		return nil, false, nil
	}
	if op == meshbool.OpCut {
		if record.lower != nil && !pointConeLimitLater(record.lower, cone, record.base) {
			return nil, false, nil
		}
		record.lower = cone
	} else {
		if record.upper != nil && !pointConeLimitEarlier(record.upper, cone, record.base) {
			return nil, false, nil
		}
		record.upper = cone
	}
	if record.lower != nil && record.upper != nil &&
		!pointConeLimitOrdered(record.lower, record.upper, record.base) {
		return nil, false, nil
	}
	body, err := buildPointConeTrim(ctx, d, ref, record)
	return body, true, err
}

// The cone's far plane is not represented in the radial trim mesh. Admit
// this arm only when the entire authenticated fan clears that plane.
func pointConeSourceAboveFarCap(base facetedPayload, cone *pointConeLimit) bool {
	if !proofbound.FiniteVec(r3.Vec{X: base.meshBound}) || base.meshBound < 0 {
		return false
	}
	for _, p := range base.verts {
		if !proofbound.FiniteVec(p) {
			return false
		}
		lo := new(big.Rat).Sub(proofarith.FloatRat(p.X), proofarith.FloatRat(base.meshBound))
		if lo.Cmp(cone.far) <= 0 {
			return false
		}
	}
	return true
}

// The larger cone's radial crossing lies farther from the point-loft apex.
// This comparison uses a deliberately wide exact-rational enclosure of the
// source fan, so nearly coincident or crossing cones fall back to mesh Boolean.
func pointConeLimitLater(old, next *pointConeLimit, base facetedPayload) bool {
	return pointConeLimitOrdered(old, next, base)
}

func pointConeLimitEarlier(old, next *pointConeLimit, base facetedPayload) bool {
	return pointConeLimitOrdered(next, old, base)
}

func pointConeLimitOrdered(earlier, later *pointConeLimit, base facetedPayload) bool {
	xMin, xMax := math.Inf(1), math.Inf(-1)
	yMin, yMax := math.Inf(1), math.Inf(-1)
	zMin, zMax := math.Inf(1), math.Inf(-1)
	rhoMax := new(big.Rat)
	for _, p := range base.verts[1:] {
		xMin, xMax = min(xMin, p.X), max(xMax, p.X)
		yMin, yMax = min(yMin, p.Y), max(yMax, p.Y)
		zMin, zMax = min(zMin, p.Z), max(zMax, p.Z)
		y, z := proofarith.FloatRat(p.Y), proofarith.FloatRat(p.Z)
		r2 := new(big.Rat).Add(new(big.Rat).Mul(y, y), new(big.Rat).Mul(z, z))
		r := proofarith.FloatRat(proofbound.RatSqrtUp(r2))
		if r.Cmp(rhoMax) > 0 {
			rhoMax = r
		}
	}
	minAbs := func(lo, hi float64) float64 {
		if lo <= 0 && hi >= 0 {
			return 0
		}
		return min(math.Abs(lo), math.Abs(hi))
	}
	rhoMin := proofarith.FloatRat(max(minAbs(yMin, yMax), minAbs(zMin, zMax)))
	// aLater/gLater > aEarlier/gEarlier exactly when this affine
	// expression in x and rho is positive over the whole far cap.
	dA := new(big.Rat).Sub(later.apex, earlier.apex)
	dR := new(big.Rat).Sub(new(big.Rat).Mul(later.apex, earlier.slope),
		new(big.Rat).Mul(earlier.apex, later.slope))
	x := proofarith.FloatRat(xMin)
	if dA.Sign() < 0 {
		x = proofarith.FloatRat(xMax)
	}
	rho := rhoMin
	if dR.Sign() < 0 {
		rho = rhoMax
	}
	margin := new(big.Rat).Add(new(big.Rat).Mul(dA, x), new(big.Rat).Mul(dR, rho))
	move := new(big.Rat).Add(new(big.Rat).Abs(dA), new(big.Rat).Abs(dR))
	move.Mul(move, proofarith.FloatRat(base.meshBound))
	return margin.Cmp(move) > 0
}

func buildPointConeTrim(ctx context.Context, d *Document, ref producerID,
	record pointConeTrimRecord) (*Body, error) {
	if record.upper != nil {
		return buildPointConeInsideBand(ctx, d, ref, record)
	}
	if record.lower == nil {
		return nil, fmt.Errorf("%w: a point-cone trim has no radial limit", ErrUnsupported)
	}
	return buildPointConeOuterBand(ctx, d, ref, record)
}

// buildPointConeInsideBand keeps the material inside the upper cone. A prior
// toe Cut supplies the lower cone; otherwise the common loft apex is its lower
// boundary. The upper cap is split where the cone reaches the far profile.
func buildPointConeInsideBand(ctx context.Context, d *Document, ref producerID,
	record pointConeTrimRecord) (*Body, error) {
	base, upper, lower := record.base, record.upper, record.lower
	far, caps, sides, farRound, err := pointConeSubdivide(ctx, base, 4)
	if err != nil {
		return nil, err
	}
	capArea, reach := pointConeCapArea(far, caps), pointConeReachUpper(far)
	upperSamples := make([]pointConeSample, len(far))
	upperNodeRound, upperQError := 0.0, 0.0
	for i, p := range far {
		upperSamples[i], err = pointConeFactor(p, upper)
		if err != nil {
			return nil, err
		}
		upperNodeRound = max(upperNodeRound, upperSamples[i].pointError)
		upperQError = max(upperQError, upperSamples[i].qError)
	}
	upperCapBound, upperLip, err := pointConeCapBound(far, caps, upper)
	if err != nil {
		return nil, err
	}
	upperCapBound = proofbound.AbsSumUpper(upperCapBound,
		proofbound.ProductUpper(reach, proofbound.ProductUpper(upperQError,
			proofbound.ProductUpper(upperLip, upperLip))))
	lowerCapBound, lowerLip, lowerNodeRound := 0.0, 0.0, 0.0
	if lower != nil {
		lowerCapBound, lowerLip, err = pointConeCapBound(far, caps, lower)
		if err != nil {
			return nil, err
		}
	}
	outside, inside, _, allSides, far, upperSamples, contactRound, err :=
		pointConeSplitAtContact(far, caps, sides, upperSamples)
	if err != nil {
		return nil, err
	}
	farRound = proofbound.AbsSumUpper(farRound, contactRound)
	allCaps := make([][3]int, 0, len(outside)+len(inside))
	allCaps = append(allCaps, outside...)
	allCaps = append(allCaps, inside...)
	lowerSamples := make([]pointConeSample, len(far))
	if lower != nil {
		lowerQError := 0.0
		for i, p := range far {
			lowerSamples[i], err = pointConeFactor(p, lower)
			if err != nil {
				return nil, err
			}
			if lowerSamples[i].q-lowerSamples[i].qError <= 1 {
				return nil, fmt.Errorf("%w: the lower cone touches the point-loft far cap", ErrUnsupported)
			}
			lowerNodeRound = max(lowerNodeRound, lowerSamples[i].pointError)
			lowerQError = max(lowerQError, lowerSamples[i].qError)
		}
		lowerCapBound = proofbound.AbsSumUpper(lowerCapBound,
			proofbound.ProductUpper(reach, proofbound.ProductUpper(lowerQError,
				proofbound.ProductUpper(lowerLip, lowerLip))))
	}
	n := len(far)
	verts := make([]r3.Vec, 0, 2*n+1)
	for i, p := range far {
		if upperSamples[i].q <= 1 {
			verts = append(verts, p)
		} else {
			verts = append(verts, p.Scale(upperSamples[i].alpha))
		}
	}
	if lower != nil {
		for i, p := range far {
			verts = append(verts, p.Scale(lowerSamples[i].alpha))
		}
	} else {
		verts = append(verts, r3.Vec{})
	}
	groups := append([]facetGroup(nil), base.groups...)
	farGroup := len(groups) - 1
	upperGroup := len(groups)
	groups = append(groups, pointConeFacetGroup(upper, false))
	lowerGroup := -1
	if lower != nil {
		lowerGroup = len(groups)
		groups = append(groups, pointConeFacetGroup(lower, true))
	}
	for i := range farGroup {
		groups[i].surface = NURBSSurface{}
	}
	tris := make([][3]int, 0, 2*len(allCaps)+2*len(allSides))
	src := make([]int, 0, cap(tris))
	add := func(t [3]int, group int) {
		tris = append(tris, t)
		src = append(src, group)
	}
	for _, t := range outside {
		add(t, upperGroup)
	}
	for _, t := range inside {
		add(t, farGroup)
	}
	if lower != nil {
		for _, t := range allCaps {
			add([3]int{t[2] + n, t[1] + n, t[0] + n}, lowerGroup)
		}
		for _, side := range allSides {
			u, v := side.u, side.v
			add([3]int{u, u + n, v + n}, side.group)
			add([3]int{u, v + n, v}, side.group)
		}
	} else {
		apex := len(verts) - 1
		for _, side := range allSides {
			add([3]int{apex, side.v, side.u}, side.group)
		}
	}
	verts, tris = pointConeCompact(verts, tris)
	if tessellation.OrientationSign(verts, tris, r3.Vec{}) < 0 {
		for i := range tris {
			tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
		}
	}
	lip := proofbound.AbsSumUpper(1, upperLip, lowerLip)
	capBound := proofbound.AbsSumUpper(upperCapBound, lowerCapBound)
	roundEnvelope := proofbound.AbsSumUpper(proofbound.ProductUpper(farRound, lip),
		upperNodeRound, lowerNodeRound)
	bound := proofbound.AbsSumUpper(proofbound.ProductUpper(base.meshBound, lip),
		capBound, roundEnvelope)
	areaUpper, err := proofbound.PerturbedAreaUpperContext(ctx, verts, tris, roundEnvelope)
	if err != nil {
		return nil, err
	}
	// Moving an interpolated cone cap to its radial image can enlarge its
	// swept area by the square of the crossing map's Lipschitz bound.
	capAreaFactor := max(1, proofbound.ProductUpper(upperLip, upperLip),
		proofbound.ProductUpper(lowerLip, lowerLip))
	volumeGap := proofbound.AbsSumUpper(base.volSymDiff,
		proofbound.ProductUpper(proofbound.ProductUpper(capArea, capBound), capAreaFactor),
		proofbound.SweptVolumeAllow(roundEnvelope, areaUpper))
	baseArea, err := proofbound.PerturbedAreaUpperContext(ctx, base.verts, base.tris, base.meshBound)
	if err != nil {
		return nil, err
	}
	areaSlack := proofbound.AbsSumUpper(base.areaSlack, baseArea, areaUpper,
		proofbound.ProductUpper(proofbound.AbsSumUpper(1,
			proofbound.ProductUpper(upperLip, upperLip),
			proofbound.ProductUpper(lowerLip, lowerLip)),
			proofbound.AbsSumUpper(capArea, base.areaSlack)))
	if proofbound.IsNonFinite(bound) || proofbound.IsNonFinite(volumeGap) ||
		proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: the point-cone intersection has no finite mesh proof", ErrUnsupported)
	}
	vertexBound := make([]float64, len(verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	pp := facetedPayload{
		verts: verts, tris: tris, src: src, groups: groups, vertexBound: vertexBound,
		meshBound: bound, volSymDiff: volumeGap, areaSlack: areaSlack,
		dPair: base.dPair, xform: r3.Identity(), pointCone: &record,
	}
	return buildFacetedBody(ctx, d, ref, pp)
}

// buildPointConeOuterBand constructs a near cone cap and the original far cap
// over the same triangulated radial domain. The cone must cross every ray
// strictly before the far cap; contact curves use a separate clipping arm.
func buildPointConeOuterBand(ctx context.Context, d *Document, ref producerID,
	record pointConeTrimRecord) (*Body, error) {
	base, cone := record.base, record.lower
	if !pointConeHasOutsideWitness(base, cone) {
		return nil, fmt.Errorf("%w: the point loft has no certified material outside the cone", ErrUnsupported)
	}
	far, caps, sides, farRound, err := pointConeSubdivide(ctx, base, 3)
	if err != nil {
		return nil, err
	}
	if len(far) == 0 || len(caps) == 0 {
		return nil, fmt.Errorf("%w: the point loft has no far cap", ErrDegenerate)
	}
	fullCapArea := pointConeCapArea(far, caps)
	fullReach := pointConeReachUpper(far)
	samples := make([]pointConeSample, len(far))
	maxNodeRound, maxQError := 0.0, 0.0
	for i, p := range far {
		sample, err := pointConeFactor(p, cone)
		if err != nil {
			return nil, err
		}
		samples[i] = sample
		maxNodeRound = max(maxNodeRound, sample.pointError)
		maxQError = max(maxQError, sample.qError)
	}
	capBound, lip, err := pointConeCapBound(far, caps, cone)
	if err != nil {
		return nil, err
	}
	capBound = proofbound.AbsSumUpper(capBound,
		proofbound.ProductUpper(fullReach, proofbound.ProductUpper(maxQError,
			proofbound.ProductUpper(lip, lip))))
	clippedCaps, _, clippedSides, _, clippedFar, clippedSamples, contactRound, err :=
		pointConeSplitAtContact(far, caps, sides, samples)
	if err != nil {
		return nil, err
	}
	far, caps, sides, samples = clippedFar, clippedCaps, clippedSides, clippedSamples
	farRound = proofbound.AbsSumUpper(farRound, contactRound)
	verts := append([]r3.Vec(nil), far...)
	for i, p := range far {
		if samples[i].q <= 1 {
			verts = append(verts, p)
		} else {
			verts = append(verts, p.Scale(samples[i].alpha))
		}
	}
	groups := append([]facetGroup(nil), base.groups...)
	capGroup := len(groups) - 1
	coneGroup := len(groups)
	groups = append(groups, pointConeFacetGroup(cone, true))
	for i := range capGroup {
		groups[i].surface = NURBSSurface{}
	}
	tris := make([][3]int, 0, len(caps)*2+len(sides)*2)
	src := make([]int, 0, cap(tris))
	add := func(t [3]int, group int) {
		tris = append(tris, t)
		src = append(src, group)
	}
	n := len(far)
	near := func(i int) int {
		if samples[i].q <= 1 {
			return i
		}
		return i + n
	}
	for _, t := range caps {
		add(t, capGroup)
		add([3]int{near(t[2]), near(t[1]), near(t[0])}, coneGroup)
	}
	for _, side := range sides {
		u, v := side.u, side.v
		if near(u) != u {
			add([3]int{u, near(u), near(v)}, side.group)
		}
		if near(v) != v {
			add([3]int{u, near(v), v}, side.group)
		}
	}
	verts, tris = pointConeCompact(verts, tris)
	if tessellation.OrientationSign(verts, tris, r3.Vec{}) < 0 {
		for i := range tris {
			tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	roundEnvelope := proofbound.AbsSumUpper(
		proofbound.ProductUpper(farRound, proofbound.AbsSumUpper(1, lip)), maxNodeRound)
	bound := proofbound.AbsSumUpper(
		proofbound.ProductUpper(base.meshBound, proofbound.AbsSumUpper(1, lip)),
		capBound, roundEnvelope)
	capArea := fullCapArea
	areaUpper, err := proofbound.PerturbedAreaUpperContext(ctx, verts, tris, roundEnvelope)
	if err != nil {
		return nil, err
	}
	capAreaFactor := max(1, proofbound.ProductUpper(lip, lip))
	volumeGap := proofbound.AbsSumUpper(base.volSymDiff,
		proofbound.ProductUpper(proofbound.ProductUpper(capArea, capBound), capAreaFactor),
		proofbound.SweptVolumeAllow(roundEnvelope, areaUpper))
	baseArea, err := proofbound.PerturbedAreaUpperContext(ctx, base.verts, base.tris, base.meshBound)
	if err != nil {
		return nil, err
	}
	areaSlack := proofbound.AbsSumUpper(base.areaSlack, baseArea,
		proofbound.ProductUpper(proofbound.ProductUpper(lip, lip),
			proofbound.AbsSumUpper(capArea, base.areaSlack)), areaUpper)
	if proofbound.IsNonFinite(bound) || proofbound.IsNonFinite(volumeGap) ||
		proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: the point-cone trim has no finite mesh proof", ErrUnsupported)
	}
	vertexBound := make([]float64, len(verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	pp := facetedPayload{
		verts: verts, tris: tris, src: src, groups: groups, vertexBound: vertexBound,
		meshBound: bound, volSymDiff: volumeGap, areaSlack: areaSlack,
		dPair: base.dPair, xform: r3.Identity(), pointCone: &record,
	}
	return buildFacetedBody(ctx, d, ref, pp)
}

func pointConeHasOutsideWitness(base facetedPayload, cone *pointConeLimit) bool {
	move := new(big.Rat).Quo(new(big.Rat).Mul(
		new(big.Rat).Add(big.NewRat(1, 1), cone.slope),
		proofarith.FloatRat(base.meshBound)), cone.apex)
	for _, p := range base.verts[1:] {
		sample, err := pointConeFactor(p, cone)
		if err != nil {
			continue
		}
		margin := new(big.Rat).Sub(proofarith.FloatRat(sample.q), big.NewRat(1, 1))
		margin.Sub(margin, proofarith.FloatRat(sample.qError))
		if margin.Cmp(move) > 0 {
			return true
		}
	}
	return false
}

func pointConeSubdivide(ctx context.Context, base facetedPayload, rounds int) (
	[]r3.Vec, [][3]int, []pointConeSide, float64, error) {
	if base.pointSection == nil || len(base.verts) < 4 {
		return nil, nil, nil, 0, fmt.Errorf("%w: the point-cone source has no fan", ErrUnsupported)
	}
	far := append([]r3.Vec(nil), base.verts[1:]...)
	boundaryGroups := map[pointConeEdge]int{}
	for i, tri := range base.tris {
		if tri[0] == 0 {
			boundaryGroups[orderedPointConeEdge(tri[1]-1, tri[2]-1)] = base.src[i]
		}
	}
	var caps [][3]int
	var sides []pointConeSide
	for i, tri := range base.tris {
		if tri[0] == 0 || tri[1] == 0 || tri[2] == 0 {
			continue
		}
		t := [3]int{tri[0] - 1, tri[1] - 1, tri[2] - 1}
		caps = append(caps, t)
		for j := range 3 {
			u, v := t[j], t[(j+1)%3]
			if group, ok := boundaryGroups[orderedPointConeEdge(u, v)]; ok {
				sides = append(sides, pointConeSide{u, v, group})
			}
		}
		_ = i
	}
	totalRound := 0.0
	for range rounds {
		levelRound := 0.0
		midpoints := map[pointConeEdge]int{}
		midpoint := func(u, v int) int {
			key := orderedPointConeEdge(u, v)
			if index, ok := midpoints[key]; ok {
				return index
			}
			p, q := far[u], far[v]
			held := p.Scale(0.5).Add(q.Scale(0.5))
			exact := [3]*big.Rat{
				new(big.Rat).Mul(new(big.Rat).Add(proofarith.FloatRat(p.X), proofarith.FloatRat(q.X)), big.NewRat(1, 2)),
				new(big.Rat).Mul(new(big.Rat).Add(proofarith.FloatRat(p.Y), proofarith.FloatRat(q.Y)), big.NewRat(1, 2)),
				new(big.Rat).Mul(new(big.Rat).Add(proofarith.FloatRat(p.Z), proofarith.FloatRat(q.Z)), big.NewRat(1, 2)),
			}
			levelRound = max(levelRound, proofbound.Radius3D(max(
				proofbound.RatAbsDiff(exact[0], held.X),
				proofbound.RatAbsDiff(exact[1], held.Y),
				proofbound.RatAbsDiff(exact[2], held.Z))))
			index := len(far)
			far = append(far, held)
			midpoints[key] = index
			return index
		}
		next := make([][3]int, 0, len(caps)*4)
		for _, t := range caps {
			if err := ctx.Err(); err != nil {
				return nil, nil, nil, 0, err
			}
			a, b, c := t[0], t[1], t[2]
			ab, bc, ca := midpoint(a, b), midpoint(b, c), midpoint(c, a)
			next = append(next, [3]int{a, ab, ca}, [3]int{ab, b, bc},
				[3]int{ca, bc, c}, [3]int{ab, bc, ca})
		}
		newSides := make([]pointConeSide, 0, len(sides)*2)
		for _, side := range sides {
			m := midpoints[orderedPointConeEdge(side.u, side.v)]
			newSides = append(newSides, pointConeSide{side.u, m, side.group},
				pointConeSide{m, side.v, side.group})
		}
		caps, sides = next, newSides
		totalRound = proofbound.AbsSumUpper(totalRound, levelRound)
	}
	return far, caps, sides, totalRound, nil
}

func pointConeFactor(p r3.Vec, cone *pointConeLimit) (pointConeSample, error) {
	if !proofbound.FiniteVec(p) {
		return pointConeSample{}, fmt.Errorf("%w: a point-cone cap vertex is non-finite", ErrNotFinite)
	}
	x, y, z := proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y), proofarith.FloatRat(p.Z)
	rho2 := new(big.Rat).Add(new(big.Rat).Mul(y, y), new(big.Rat).Mul(z, z))
	rho, ok := proofbound.IntervalSqrt(proofbound.PointInterval(rho2))
	if !ok {
		return pointConeSample{}, fmt.Errorf("%w: the cone radius has no finite enclosure", ErrUnsupported)
	}
	g := proofbound.IntervalAdd(proofbound.PointInterval(x),
		proofbound.IntervalScale(rho, cone.slope))
	if g.Lo.Sign() <= 0 {
		return pointConeSample{}, fmt.Errorf("%w: a point-cone ray is not forward", ErrUnsupported)
	}
	alphaIv, ok := proofbound.IntervalQuo(proofbound.PointInterval(cone.apex), g)
	if !ok {
		return pointConeSample{}, fmt.Errorf("%w: a point-cone ray has no crossing", ErrUnsupported)
	}
	qIv, ok := proofbound.IntervalQuo(g, proofbound.PointInterval(cone.apex))
	if !ok {
		return pointConeSample{}, fmt.Errorf("%w: a point-cone ray has no inverse crossing", ErrUnsupported)
	}
	a, _ := cone.apex.Float64()
	k, _ := cone.slope.Float64()
	q := (p.X + k*math.Hypot(p.Y, p.Z)) / a
	if !proofbound.FiniteVec(r3.Vec{X: q}) || q <= 0 {
		return pointConeSample{}, fmt.Errorf("%w: the held point-cone ray has no finite crossing", ErrUnsupported)
	}
	alpha := 1 / q
	if !proofbound.FiniteVec(r3.Vec{X: alpha}) {
		return pointConeSample{}, fmt.Errorf("%w: the held point-cone crossing is non-finite", ErrUnsupported)
	}
	held := p.Scale(alpha)
	bound := proofbound.Radius3D(max(
		proofbound.IntervalFloatError(proofbound.IntervalScale(alphaIv, x), held.X),
		proofbound.IntervalFloatError(proofbound.IntervalScale(alphaIv, y), held.Y),
		proofbound.IntervalFloatError(proofbound.IntervalScale(alphaIv, z), held.Z)))
	return pointConeSample{q: q, alpha: alpha,
		qError: proofbound.IntervalFloatError(qIv, q), pointError: bound}, nil
}

func pointConeSplitAtContact(far []r3.Vec, caps [][3]int, sides []pointConeSide,
	samples []pointConeSample) ([][3]int, [][3]int, []pointConeSide, []pointConeSide,
	[]r3.Vec, []pointConeSample, float64, error) {
	contact := map[pointConeEdge]int{}
	maxRound := 0.0
	crossingFailure := false
	crossing := func(u, v int) int {
		if samples[u].q == 1 {
			return u
		}
		if samples[v].q == 1 {
			return v
		}
		key := orderedPointConeEdge(u, v)
		if index, ok := contact[key]; ok {
			return index
		}
		qu, qv := samples[u].q, samples[v].q
		t := (1 - qu) / (qv - qu)
		if !(t > 0 && t < 1) {
			crossingFailure = true
			return u
		}
		p := far[u].Scale(1 - t).Add(far[v].Scale(t))
		tr := new(big.Rat).Quo(new(big.Rat).Sub(big.NewRat(1, 1), proofarith.FloatRat(qu)),
			new(big.Rat).Sub(proofarith.FloatRat(qv), proofarith.FloatRat(qu)))
		oneminust := new(big.Rat).Sub(big.NewRat(1, 1), tr)
		exact := [3]*big.Rat{}
		for axis, pair := range [...][2]float64{{far[u].X, far[v].X}, {far[u].Y, far[v].Y}, {far[u].Z, far[v].Z}} {
			exact[axis] = new(big.Rat).Add(new(big.Rat).Mul(oneminust, proofarith.FloatRat(pair[0])),
				new(big.Rat).Mul(tr, proofarith.FloatRat(pair[1])))
		}
		maxRound = max(maxRound, proofbound.Radius3D(max(
			proofbound.RatAbsDiff(exact[0], p.X),
			proofbound.RatAbsDiff(exact[1], p.Y),
			proofbound.RatAbsDiff(exact[2], p.Z))))
		index := len(far)
		far = append(far, p)
		samples = append(samples, pointConeSample{q: 1, alpha: 1})
		contact[key] = index
		return index
	}
	outside := make([][3]int, 0, len(caps))
	inside := make([][3]int, 0, len(caps))
	for _, tri := range caps {
		for _, arm := range []struct {
			keepOutside bool
			output      *[][3]int
		}{{true, &outside}, {false, &inside}} {
			if arm.keepOutside && samples[tri[0]].q <= 1 && samples[tri[1]].q <= 1 && samples[tri[2]].q <= 1 {
				continue
			}
			if !arm.keepOutside && samples[tri[0]].q >= 1 && samples[tri[1]].q >= 1 && samples[tri[2]].q >= 1 {
				continue
			}
			poly := []int{tri[0], tri[1], tri[2]}
			out := make([]int, 0, 4)
			for j := range poly {
				u, v := poly[j], poly[(j+1)%len(poly)]
				uIn, vIn := samples[u].q >= 1, samples[v].q >= 1
				if !arm.keepOutside {
					uIn, vIn = samples[u].q <= 1, samples[v].q <= 1
				}
				if uIn {
					out = append(out, u)
				}
				if uIn != vIn {
					out = append(out, crossing(u, v))
				}
			}
			poly = poly[:0]
			for _, index := range out {
				if len(poly) == 0 || poly[len(poly)-1] != index {
					poly = append(poly, index)
				}
			}
			if len(poly) > 1 && poly[0] == poly[len(poly)-1] {
				poly = poly[:len(poly)-1]
			}
			for j := 1; j+1 < len(poly); j++ {
				*arm.output = append(*arm.output, [3]int{poly[0], poly[j], poly[j+1]})
			}
		}
	}
	outsideSides := make([]pointConeSide, 0, len(sides))
	allSides := make([]pointConeSide, 0, len(sides)*2)
	for _, side := range sides {
		u, v := side.u, side.v
		qu, qv := samples[u].q, samples[v].q
		if qu == 1 || qv == 1 || (qu > 1) == (qv > 1) {
			allSides = append(allSides, side)
			if qu > 1 || qv > 1 {
				outsideSides = append(outsideSides, side)
			}
			continue
		}
		m := crossing(u, v)
		first, second := pointConeSide{u, m, side.group}, pointConeSide{m, v, side.group}
		allSides = append(allSides, first, second)
		if qu > 1 {
			outsideSides = append(outsideSides, first)
		} else {
			outsideSides = append(outsideSides, second)
		}
	}
	if crossingFailure {
		return nil, nil, nil, nil, nil, nil, 0,
			fmt.Errorf("%w: a point-cone contact rounds onto a cap vertex", ErrUnsupported)
	}
	return outside, inside, outsideSides, allSides, far, samples, maxRound, nil
}

func pointConeCompact(verts []r3.Vec, tris [][3]int) ([]r3.Vec, [][3]int) {
	index := make(map[int]int, len(verts))
	out := make([]r3.Vec, 0, len(verts))
	for i := range tris {
		for j, old := range tris[i] {
			fresh, ok := index[old]
			if !ok {
				fresh = len(out)
				index[old] = fresh
				out = append(out, verts[old])
			}
			tris[i][j] = fresh
		}
	}
	return out, tris
}

func pointConeReachUpper(far []r3.Vec) float64 {
	reach := 0.0
	for _, p := range far {
		reach = max(reach, proofbound.DvLenUpper(proofbound.HeldDelta(p, r3.Vec{})))
	}
	return reach
}

func pointConeCapArea(far []r3.Vec, caps [][3]int) float64 {
	return proofbound.PerturbedAreaUpper(far, caps, 0)
}

func pointConeCapBound(far []r3.Vec, caps [][3]int,
	cone *pointConeLimit) (float64, float64, error) {
	capBound, lip := 0.0, 0.0
	for _, tri := range caps {
		cellBound, _, cellLip, err := pointConeTriangleBound(
			[3]r3.Vec{far[tri[0]], far[tri[1]], far[tri[2]]}, cone)
		if err != nil {
			return 0, 0, err
		}
		capBound = max(capBound, cellBound)
		lip = max(lip, cellLip)
	}
	return capBound, lip, nil
}

func pointConeTriangleBound(tri [3]r3.Vec,
	cone *pointConeLimit) (float64, float64, float64, error) {
	xMin, yMin, yMax, zMin, zMax := math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
	reach, h := 0.0, 0.0
	for i, p := range tri {
		xMin = min(xMin, p.X)
		yMin, yMax = min(yMin, p.Y), max(yMax, p.Y)
		zMin, zMax = min(zMin, p.Z), max(zMax, p.Z)
		reach = max(reach, proofbound.DvLenUpper(proofbound.HeldDelta(p, r3.Vec{})))
		h = max(h, proofbound.DvLenUpper(proofbound.HeldDelta(p, tri[(i+1)%3])))
	}
	minAbs := func(lo, hi float64) float64 {
		if lo <= 0 && hi >= 0 {
			return 0
		}
		return min(math.Abs(lo), math.Abs(hi))
	}
	rhoMin := max(minAbs(yMin, yMax), minAbs(zMin, zMax))
	if rhoMin <= 0 {
		return 0, 0, 0, fmt.Errorf("%w: the point-cone cap reaches the axis", ErrUnsupported)
	}
	gMin := new(big.Rat).Add(proofarith.FloatRat(xMin),
		new(big.Rat).Mul(cone.slope, proofarith.FloatRat(rhoMin)))
	if gMin.Sign() <= 0 {
		return 0, 0, 0, fmt.Errorf("%w: the point-cone cap has no positive ray denominator", ErrUnsupported)
	}
	// The crossing map is F(p)=p*a/g(p), where g=x+k*rho. Its Hessian
	// includes both the curvature of rho and the reciprocal/product terms.
	// For triangle diameter h, the interpolation departure is at most
	// h²/2 times sup ||H_F||. The scalar alpha bound also covers the
	// piecewise contact decision at g=a.
	a, k := cone.apex, cone.slope
	onePlusK := new(big.Rat).Add(big.NewRat(1, 1), k)
	h2 := new(big.Rat).Mul(proofarith.FloatRat(h), proofarith.FloatRat(h))
	g2 := new(big.Rat).Mul(gMin, gMin)
	g3 := new(big.Rat).Mul(g2, gMin)
	reciprocal := new(big.Rat).Quo(
		new(big.Rat).Mul(new(big.Rat).Mul(a, onePlusK), new(big.Rat).Mul(onePlusK, h2)), g3)
	radial := new(big.Rat).Quo(
		new(big.Rat).Mul(new(big.Rat).Mul(a, k), h2),
		new(big.Rat).Mul(big.NewRat(2, 1),
			new(big.Rat).Mul(proofarith.FloatRat(rhoMin), g2)))
	alphaBound := proofbound.AbsSumUpper(proofbound.RatFloatUp(reciprocal), proofbound.RatFloatUp(radial))
	product := new(big.Rat).Quo(
		new(big.Rat).Mul(new(big.Rat).Mul(a, onePlusK), h2), g2)
	capBound := proofbound.AbsSumUpper(
		proofbound.ProductUpper(reach, alphaBound), proofbound.RatFloatUp(product))
	alphaMax := proofbound.RatFloatUp(new(big.Rat).Quo(cone.apex, gMin))
	lipNumer := new(big.Rat).Mul(cone.apex, new(big.Rat).Add(big.NewRat(1, 1), cone.slope))
	lipDenom := new(big.Rat).Mul(gMin, gMin)
	lip := proofbound.AbsSumUpper(alphaMax,
		proofbound.ProductUpper(reach, proofbound.RatFloatUp(new(big.Rat).Quo(lipNumer, lipDenom))))
	return capBound, alphaBound, lip, nil
}
