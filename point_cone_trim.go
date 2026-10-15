package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
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
	surface          surfacegeom.Cone
	origins          []provenance.Ref
	denoted          *surfacenormal.Revolved
	reversed         bool

	consts *pointcone.Consts // built on first use by constants
}

func (cone *pointConeLimit) constants() *pointcone.Consts {
	if cone.consts == nil {
		cone.consts = pointcone.ConstantsOf(cone.apex, cone.slope)
	}
	return cone.consts
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

// exactPointCone recognizes the unplaced, full, straight-meridian cone whose
// axis is world X. Every parameter is read from the authenticated meridian;
// the rounded public Cone tag is used only for surface reporting.
func exactPointCone(b *Body) (*pointConeLimit, bool) {
	rp, ok := b.n.Payload.(revolvePayload)
	if !ok || !rp.Full || rp.SurfaceResult || rp.SectionDelta != 0 || rp.Xform != r3.Identity() ||
		len(rp.Profile.Holes) != 0 || len(rp.Profile.Outer.Segments) != 3 ||
		rp.Frame.Origin() != (r3.Vec{}) || rp.Frame.U() != (r3.Vec{X: 1}) ||
		rp.Frame.V() != (r3.Vec{Y: 1}) || rp.Ax.AU != 0 || rp.Ax.AV != 0 ||
		rp.Ax.DU != 1 || rp.Ax.DV != 0 || rp.Ax.AUBound != 0 || rp.Ax.AVBound != 0 ||
		rp.Ax.DUBound != 0 || rp.Ax.DVBound != 0 {
		return nil, false
	}
	var apex, far, radius float64
	far = math.Inf(1)
	axisCount, rimCount := 0, 0
	for _, segment := range rp.Profile.Outer.Segments {
		line, ok := segment.(lineSeg)
		if !ok {
			return nil, false
		}
		for _, pt := range [...]planePoint{line.Start, line.End} {
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
	for _, segment := range rp.Profile.Outer.Segments {
		line, ok := segment.(lineSeg)
		if !ok {
			return nil, false
		}
		for _, pt := range [...]planePoint{line.Start, line.End} {
			if pt.V == 0 && pt.U != apex && pt.U != far || pt.V > 0 &&
				(pt.U != far || pt.V != radius) {
				return nil, false
			}
		}
	}
	var surface surfacegeom.Cone
	var origins []provenance.Ref
	var denoted *surfacenormal.Revolved
	var reversed bool
	for _, face := range b.Faces() {
		if cone, ok := face.n.Surface.(surfacegeom.Cone); ok {
			surface, origins = cone, face.originRefs()
			denoted, reversed = face.n.Denoted, face.n.Reversed
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
	fp, ok := target.n.Payload.(facetedPayload)
	if !ok || fp.xform != r3.Identity() {
		return nil, false, nil
	}
	record := pointConeTrimRecord{base: fp}
	if fp.pointCone != nil {
		record = *fp.pointCone
	} else if fp.pointSection == nil || fp.pointSection.apex != (r3.Vec{}) {
		return nil, false, nil
	}
	if !pointcone.AboveFarCap(record.base.verts, record.base.meshBound, cone.far) {
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
	return pointcone.Ordered(earlier.apex, earlier.slope, later.apex, later.slope,
		base.verts, base.meshBound)
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
	capArea, reach := pointcone.CapArea(far, caps), pointcone.ReachUpper(far)
	upperNodeRound, upperQError := 0.0, 0.0
	upperSamples, upperErrs := pointConeFactors(far, upper,
		orderedwork.EachWorkers(len(far), pointConeFactorParallelMin))
	for i := range far {
		if upperErrs[i] != nil {
			return nil, upperErrs[i]
		}
		upperNodeRound = max(upperNodeRound, upperSamples[i].PointError)
		upperQError = max(upperQError, upperSamples[i].QError)
	}
	capGeom := pointcone.CapGeometry(far, caps)
	upperCapBound, upperLip, err := pointcone.CapBound(capGeom, upper.constants())
	if err != nil {
		return nil, err
	}
	upperCapBound = proofbound.AbsSumUpper(upperCapBound,
		proofbound.ProductUpper(reach, proofbound.ProductUpper(upperQError,
			proofbound.ProductUpper(upperLip, upperLip))))
	lowerCapBound, lowerLip, lowerNodeRound := 0.0, 0.0, 0.0
	if lower != nil {
		lowerCapBound, lowerLip, err = pointcone.CapBound(capGeom, lower.constants())
		if err != nil {
			return nil, err
		}
	}
	outside, inside, _, allSides, far, upperSamples, contactRound, err :=
		pointcone.SplitAtContact(far, caps, sides, upperSamples)
	if err != nil {
		return nil, err
	}
	farRound = proofbound.AbsSumUpper(farRound, contactRound)
	allCaps := make([][3]int, 0, len(outside)+len(inside))
	allCaps = append(allCaps, outside...)
	allCaps = append(allCaps, inside...)
	lowerSamples := make([]pointcone.Sample, len(far))
	if lower != nil {
		lowerQError := 0.0
		var lowerErrs []error
		lowerSamples, lowerErrs = pointConeFactors(far, lower,
			orderedwork.EachWorkers(len(far), pointConeFactorParallelMin))
		for i := range far {
			if lowerErrs[i] != nil {
				return nil, lowerErrs[i]
			}
			if lowerSamples[i].Q-lowerSamples[i].QError <= 1 {
				return nil, fmt.Errorf("%w: the lower cone touches the point-loft far cap", ErrUnsupported)
			}
			lowerNodeRound = max(lowerNodeRound, lowerSamples[i].PointError)
			lowerQError = max(lowerQError, lowerSamples[i].QError)
		}
		lowerCapBound = proofbound.AbsSumUpper(lowerCapBound,
			proofbound.ProductUpper(reach, proofbound.ProductUpper(lowerQError,
				proofbound.ProductUpper(lowerLip, lowerLip))))
	}
	n := len(far)
	verts := make([]r3.Vec, 0, 2*n+1)
	for i, p := range far {
		if upperSamples[i].Q <= 1 {
			verts = append(verts, p)
		} else {
			verts = append(verts, p.Scale(upperSamples[i].Alpha))
		}
	}
	if lower != nil {
		for i, p := range far {
			verts = append(verts, p.Scale(lowerSamples[i].Alpha))
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
		groups[i].surface = surfacegeom.NURBSSurface{}
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
			u, v := side.U, side.V
			add([3]int{u, u + n, v + n}, side.Group)
			add([3]int{u, v + n, v}, side.Group)
		}
	} else {
		apex := len(verts) - 1
		for _, side := range allSides {
			add([3]int{apex, side.V, side.U}, side.Group)
		}
	}
	verts, tris = pointcone.Compact(verts, tris)
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
	fullCapArea := pointcone.CapArea(far, caps)
	fullReach := pointcone.ReachUpper(far)
	maxNodeRound, maxQError := 0.0, 0.0
	samples, sampleErrs := pointConeFactors(far, cone,
		orderedwork.EachWorkers(len(far), pointConeFactorParallelMin))
	for i := range far {
		if sampleErrs[i] != nil {
			return nil, sampleErrs[i]
		}
		sample := samples[i]
		maxNodeRound = max(maxNodeRound, sample.PointError)
		maxQError = max(maxQError, sample.QError)
	}
	capBound, lip, err := pointcone.CapBound(pointcone.CapGeometry(far, caps), cone.constants())
	if err != nil {
		return nil, err
	}
	capBound = proofbound.AbsSumUpper(capBound,
		proofbound.ProductUpper(fullReach, proofbound.ProductUpper(maxQError,
			proofbound.ProductUpper(lip, lip))))
	clippedCaps, _, clippedSides, _, clippedFar, clippedSamples, contactRound, err :=
		pointcone.SplitAtContact(far, caps, sides, samples)
	if err != nil {
		return nil, err
	}
	far, caps, sides, samples = clippedFar, clippedCaps, clippedSides, clippedSamples
	farRound = proofbound.AbsSumUpper(farRound, contactRound)
	verts := append([]r3.Vec(nil), far...)
	for i, p := range far {
		if samples[i].Q <= 1 {
			verts = append(verts, p)
		} else {
			verts = append(verts, p.Scale(samples[i].Alpha))
		}
	}
	groups := append([]facetGroup(nil), base.groups...)
	capGroup := len(groups) - 1
	coneGroup := len(groups)
	groups = append(groups, pointConeFacetGroup(cone, true))
	for i := range capGroup {
		groups[i].surface = surfacegeom.NURBSSurface{}
	}
	tris := make([][3]int, 0, len(caps)*2+len(sides)*2)
	src := make([]int, 0, cap(tris))
	add := func(t [3]int, group int) {
		tris = append(tris, t)
		src = append(src, group)
	}
	n := len(far)
	near := func(i int) int {
		if samples[i].Q <= 1 {
			return i
		}
		return i + n
	}
	for _, t := range caps {
		add(t, capGroup)
		add([3]int{near(t[2]), near(t[1]), near(t[0])}, coneGroup)
	}
	for _, side := range sides {
		u, v := side.U, side.V
		if near(u) != u {
			add([3]int{u, near(u), near(v)}, side.Group)
		}
		if near(v) != v {
			add([3]int{u, near(v), v}, side.Group)
		}
	}
	verts, tris = pointcone.Compact(verts, tris)
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
		margin := new(big.Rat).Sub(proofarith.FloatRat(sample.Q), big.NewRat(1, 1))
		margin.Sub(margin, proofarith.FloatRat(sample.QError))
		if margin.Cmp(move) > 0 {
			return true
		}
	}
	return false
}

func pointConeSubdivide(ctx context.Context, base facetedPayload, rounds int) (
	[]r3.Vec, [][3]int, []pointcone.Side, float64, error) {
	if base.pointSection == nil {
		return nil, nil, nil, 0, fmt.Errorf("%w: the point-cone source has no fan", ErrUnsupported)
	}
	return pointcone.Subdivide(ctx, base.verts, base.tris, base.src, rounds)
}

// pointConeFactor dispatches a held far-cap point to the Dyadic crossing for a
// positive apex and slope, and to the big.Rat form otherwise.
func pointConeFactor(p r3.Vec, cone *pointConeLimit) (pointcone.Sample, error) {
	if cone.apex.Sign() <= 0 || cone.slope.Sign() <= 0 {
		return pointcone.FactorRat(p, cone.apex, cone.slope)
	}
	return pointcone.FactorDyadic(p, cone.constants())
}

// pointConeFactorParallelMin is the point count from which pointConeFactors
// callers ask for more than one worker.
const pointConeFactorParallelMin = 1024

// pointConeFactors is pointConeFactor over every point: samples[i] and errs[i]
// are point i's. With workers below 2, or a non-positive apex or slope (the
// big.Rat form), it runs in point order on the calling goroutine and stops
// after the first error, leaving later slots zero. Otherwise the cone
// constants are resolved first and the points are spread over workers; the
// lowest non-nil errs index is then the serial first error, and slots past it
// may or may not be filled.
func pointConeFactors(points []r3.Vec, cone *pointConeLimit, workers int) ([]pointcone.Sample, []error) {
	samples := make([]pointcone.Sample, len(points))
	errs := make([]error, len(points))
	if cone.apex.Sign() <= 0 || cone.slope.Sign() <= 0 {
		workers = 1
	}
	if workers < 2 {
		for i, p := range points {
			samples[i], errs[i] = pointConeFactor(p, cone)
			if errs[i] != nil {
				break
			}
		}
		return samples, errs
	}
	consts := cone.constants()
	// The per-index error lives in errs, so Each's own result is unused.
	_, _ = orderedwork.Each(len(points), workers, 256, nil, func(i int) error {
		samples[i], errs[i] = pointcone.FactorDyadic(points[i], consts)
		return errs[i]
	})
	return samples, errs
}
