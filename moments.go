package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the mass-property engine of docs/evaluator-design.md §4:
// decad integrating its OWN records. sketch decides topology and
// admissibility; once a region is recorded, its areas and moments are decad's
// job, computed by closed-form boundary integrals (Green's theorem) per
// segment kind — exact for line, circle and arc walks, and ErrUnsupported for
// the free-form kinds.

// Area returns the recorded region's net area — the outer loop minus its
// holes — as a [Measurement] of Kind Area (mm²): a computed quantity carries
// its Exactness and Bound (docs/api-design.md §6). The closed forms here are
// exact, so the measurement reads Exact with a zero bound; each boundary
// segment contributes its Green's-theorem integral in walk order, so a hole's
// clockwise walk subtracts without a special case.
//
// A region whose boundary contains a free-form segment kind (ellipse,
// elliptical arc, conic, spline, closed spline, fit spline, NURBS) is
// [ErrUnsupported] (docs/evaluator-design.md §11) — never approximated.
// A malformed or open record is [ErrDegenerate]. A circle radius of the wrong
// kind is [ErrUnitKind], a negative radius is [ErrNegativeMagnitude], and a
// non-finite field or arithmetic result is [ErrNotFinite]. No measurement is
// returned on error.
func (r ProfileRecord) Area() (Measurement, error) {
	ig, err := r.integrals()
	if err != nil {
		return Measurement{}, err
	}
	return Measurement{
		Value:     units.SquareMillimeters(ig.area),
		Exactness: Exact,
		Bound:     units.SquareMillimeters(0),
	}, nil
}

// Centroid returns the recorded region's centroid from its exact first
// moments, as a [VecMeasurement] — a computed coordinate is a measurement
// (docs/api-design.md §6). The Value is PLANE-LOCAL: (u, v, 0) in the
// region's own plane coordinates, millimetres (§5.2), not a world position —
// lift it through the profile's PlaneRecord to place it in space. The closed
// forms are exact, so it reads Exact with a zero bound.
//
// A region whose net area is zero has no centroid and is [ErrDegenerate].
// Record validation and arithmetic errors match [ProfileRecord.Area].
func (r ProfileRecord) Centroid() (VecMeasurement, error) {
	ig, err := r.integrals()
	if err != nil {
		return VecMeasurement{}, err
	}
	if ig.area == 0 {
		return VecMeasurement{}, fmt.Errorf(`%w: a region with zero net area has no centroid`, ErrDegenerate)
	}
	u, v := ig.mu/ig.area, ig.mv/ig.area
	if !finiteMomentValues(u, v) {
		return VecMeasurement{}, fmt.Errorf(`%w: the region centroid is not finite`, ErrNotFinite)
	}
	return VecMeasurement{
		Value:     r3.NewVec(u, v, 0),
		Exactness: Exact,
		Bound:     units.Millimeters(0),
	}, nil
}

// SecondMoments is a recorded region's second moments of area about the
// plane origin, in the plane's own (u, v): every field is a Measurement of
// Kind SecondMomentOfArea (mm⁴), Exact with a zero bound — the closed forms
// admit no approximation. They are what a revolve's solid centroid is
// computed from (docs/evaluator-design.md §4/§6); to re-reference them to
// another axis, use the parallel-axis theorem with the region's Area and
// Centroid.
type SecondMoments struct {
	// UU is ∫u² dA, VV is ∫v² dA, UV is the mixed ∫uv dA.
	UU Measurement
	UV Measurement
	VV Measurement
}

// SecondMoments returns the region's exact second moments of area about the
// plane origin. The staging matches Area: a free-form boundary kind is
// [ErrUnsupported], and malformed or non-finite records are rejected before a
// measurement is constructed.
func (r ProfileRecord) SecondMoments() (SecondMoments, error) {
	ig, err := r.integrals()
	if err != nil {
		return SecondMoments{}, err
	}
	exact := func(x float64) Measurement {
		return Measurement{Value: units.QuarticMillimeters(x), Exactness: Exact, Bound: units.QuarticMillimeters(0)}
	}
	return SecondMoments{UU: exact(ig.muu), UV: exact(ig.muv), VV: exact(ig.mvv)}, nil
}

// regionIntegrals accumulates the boundary integrals of one region: the net
// signed area, the first moments ∫u dA and ∫v dA, and the second moments
// ∫u² dA, ∫uv dA and ∫v² dA.
type regionIntegrals struct {
	area float64
	mu   float64 // ∫u dA
	mv   float64 // ∫v dA
	muu  float64 // ∫u² dA
	muv  float64 // ∫uv dA
	mvv  float64 // ∫v² dA
}

func finiteMomentValues(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

func (ig regionIntegrals) isFinite() bool {
	return finiteMomentValues(ig.area, ig.mu, ig.mv, ig.muu, ig.muv, ig.mvv)
}

// integrals walks the outer loop and every hole in recorded walk order and
// sums each segment's closed-form contribution. Walk order carries the sign:
// the outer loop is counter-clockwise (positive), holes are clockwise
// (negative), so the sum IS the net region integral.
func (r ProfileRecord) integrals() (regionIntegrals, error) {
	loops := append([]LoopRecord{r.Outer}, r.Holes...)
	var ig regionIntegrals
	for loopIndex, loop := range loops {
		if err := validateMomentLoop(loop); err != nil {
			return regionIntegrals{}, fmt.Errorf(`decad: profile loop %d is invalid: %w`, loopIndex, err)
		}
		var loopIntegrals regionIntegrals
		for segmentIndex, seg := range loop.Segments {
			if err := loopIntegrals.add(seg); err != nil {
				return regionIntegrals{}, err
			}
			if !loopIntegrals.isFinite() {
				return regionIntegrals{}, fmt.Errorf(`%w: mass-property integration overflowed at loop %d segment %d`, ErrNotFinite, loopIndex, segmentIndex)
			}
		}
		if loopIndex == 0 && loopIntegrals.area <= 0 {
			return regionIntegrals{}, fmt.Errorf(`%w: the profile outer loop must run counter-clockwise`, ErrDegenerate)
		}
		if loopIndex > 0 && loopIntegrals.area >= 0 {
			return regionIntegrals{}, fmt.Errorf(`%w: profile hole %d must run clockwise`, ErrDegenerate, loopIndex-1)
		}
		ig.area += loopIntegrals.area
		ig.mu += loopIntegrals.mu
		ig.mv += loopIntegrals.mv
		ig.muu += loopIntegrals.muu
		ig.muv += loopIntegrals.muv
		ig.mvv += loopIntegrals.mvv
		if !ig.isFinite() {
			return regionIntegrals{}, fmt.Errorf(`%w: mass-property integration overflowed while combining profile loop %d`, ErrNotFinite, loopIndex)
		}
	}
	if err := validateMomentArrangement(loops); err != nil {
		return regionIntegrals{}, err
	}
	if ig.area <= 0 {
		return regionIntegrals{}, fmt.Errorf(`%w: the recorded region encloses no positive net area`, ErrDegenerate)
	}
	return ig, nil
}

// validateMomentArrangement applies the package's independent line/arc
// boundary audit after each loop's fields, closure and winding have passed.
// Caller-built and decoded records carry no upstream arrangement certificate,
// so crossing, outside or nested holes are malformed input here. Boundary
// tangency remains admissible for mass properties; consumers that require a
// manifold boundary apply their own stricter gate.
func validateMomentArrangement(loops []LoopRecord) error {
	segs, err := buildMomentSegEntries(loops)
	if err != nil {
		return err
	}
	for i := range segs {
		for j := i + 1; j < len(segs); j++ {
			if momentSegmentsCross(segs[i], segs[j]) {
				return fmt.Errorf(`%w: the recorded profile boundaries cross`, ErrDegenerate)
			}
		}
	}
	plain := make([]segEntry, len(segs))
	for i, seg := range segs {
		plain[i] = seg.segEntry
	}
	if !momentNestingValid(plain, len(loops)) {
		return fmt.Errorf(`%w: each profile hole must lie inside the outer loop and outside every other hole`, ErrDegenerate)
	}
	return nil
}

type momentSegEntry struct {
	segEntry
	bounds momentWalk
}

func buildMomentSegEntries(loops []LoopRecord) ([]momentSegEntry, error) {
	var entries []momentSegEntry
	for loopIndex, loop := range loops {
		for segmentIndex, segment := range loop.Segments {
			bounds, err := validateMomentSegment(segment)
			if err != nil {
				return nil, err
			}
			entries = append(entries, momentSegEntry{
				segEntry: segEntry{
					loop: loopIndex,
					idx:  segmentIndex,
					n:    len(loop.Segments),
					w:    bounds.segmentWalk,
				},
				bounds: bounds,
			})
		}
	}
	return entries, nil
}

// momentSegmentsCross reports a proven transversal crossing or a
// positive-length overlap on one circular carrier. Point tangencies are valid
// for mass properties, so the shared modify audit's stricter
// crossing-or-contact result cannot be used here.
func momentSegmentsCross(a, b momentSegEntry) bool {
	shared := momentSharedEndpointBounds(a, b)
	switch {
	case !a.w.circular && !b.w.circular:
		return momentLinesCross(a.w, b.w, adjacent(a.segEntry, b.segEntry))
	case a.w.circular && b.w.circular:
		return momentArcsCross(a.w, b.w, shared)
	case a.w.circular:
		return momentLineArcCross(b.w, a.w, shared)
	default:
		return momentLineArcCross(a.w, b.w, shared)
	}
}

type momentPointBounds struct {
	u, v momentRatInterval
}

func momentSharedEndpointBounds(a, b momentSegEntry) []momentPointBounds {
	if !adjacent(a.segEntry, b.segEntry) {
		return nil
	}
	var shared []momentPointBounds
	if (a.idx+1)%a.n == b.idx {
		shared = append(shared, momentEndpointBoundsHull(a.bounds, false, b.bounds, true))
	}
	if (b.idx+1)%b.n == a.idx {
		shared = append(shared, momentEndpointBoundsHull(a.bounds, true, b.bounds, false))
	}
	return shared
}

func momentEndpointBoundsHull(a momentWalk, aStart bool, b momentWalk, bStart bool) momentPointBounds {
	aU, aV := a.endUInterval, a.endVInterval
	if aStart {
		aU, aV = a.startUInterval, a.startVInterval
	}
	bU, bV := b.endUInterval, b.endVInterval
	if bStart {
		bU, bV = b.startUInterval, b.startVInterval
	}
	return momentPointBounds{
		u: momentIntervalHull(aU, bU),
		v: momentIntervalHull(aV, bV),
	}
}

func momentIntervalHull(a, b momentRatInterval) momentRatInterval {
	lo, hi := a.lo, a.hi
	if b.lo.Cmp(lo) < 0 {
		lo = b.lo
	}
	if b.hi.Cmp(hi) > 0 {
		hi = b.hi
	}
	return momentRatInterval{lo: lo, hi: hi}
}

// momentLinesCross uses exact rational orientations of the recorded float64
// coordinates. A proper crossing remains visible at every coordinate scale,
// and a positive-length collinear overlap is rejected while point contact is
// permitted. Adjacent non-collinear lines can meet only at their accepted
// junction; rounding may move that one intersection just inside both walks.
func momentLinesCross(a, b segmentWalk, areAdjacent bool) bool {
	a0 := momentExactPoint(a.startU, a.startV)
	a1 := momentExactPoint(a.endU, a.endV)
	b0 := momentExactPoint(b.startU, b.startV)
	b1 := momentExactPoint(b.endU, b.endV)
	ab0 := momentOrientation(a0, a1, b0)
	ab1 := momentOrientation(a0, a1, b1)
	ba0 := momentOrientation(b0, b1, a0)
	ba1 := momentOrientation(b0, b1, a1)
	if ab0 == 0 && ab1 == 0 && ba0 == 0 && ba1 == 0 {
		return momentCollinearOverlap(a0, a1, b0, b1)
	}
	if ab0 == 0 || ab1 == 0 || ba0 == 0 || ba1 == 0 {
		return false
	}
	if areAdjacent {
		return false
	}
	return ab0 != ab1 && ba0 != ba1
}

type momentExactPoint2 struct {
	u *big.Rat
	v *big.Rat
}

func momentExactPoint(u, v float64) momentExactPoint2 {
	return momentExactPoint2{u: ratOf(u), v: ratOf(v)}
}

func momentOrientation(a, b, c momentExactPoint2) int {
	bu := new(big.Rat).Sub(b.u, a.u)
	bv := new(big.Rat).Sub(b.v, a.v)
	cu := new(big.Rat).Sub(c.u, a.u)
	cv := new(big.Rat).Sub(c.v, a.v)
	left := new(big.Rat).Mul(bu, cv)
	right := new(big.Rat).Mul(bv, cu)
	return new(big.Rat).Sub(left, right).Sign()
}

func momentCollinearOverlap(a0, a1, b0, b1 momentExactPoint2) bool {
	if a0.u.Cmp(a1.u) != 0 || b0.u.Cmp(b1.u) != 0 {
		return momentIntervalsOverlap(a0.u, a1.u, b0.u, b1.u)
	}
	return momentIntervalsOverlap(a0.v, a1.v, b0.v, b1.v)
}

func momentIntervalsOverlap(a0, a1, b0, b1 *big.Rat) bool {
	aLo, aHi := a0, a1
	if aLo.Cmp(aHi) > 0 {
		aLo, aHi = aHi, aLo
	}
	bLo, bHi := b0, b1
	if bLo.Cmp(bHi) > 0 {
		bLo, bHi = bHi, bLo
	}
	lo := aLo
	if bLo.Cmp(lo) > 0 {
		lo = bLo
	}
	hi := aHi
	if bHi.Cmp(hi) < 0 {
		hi = bHi
	}
	return lo.Cmp(hi) < 0
}

func momentLineArcCross(line, arc segmentWalk, shared []momentPointBounds) bool {
	carrier := momentExactLine{
		u:  ratOf(line.startU),
		v:  ratOf(line.startV),
		du: new(big.Rat).Sub(ratOf(line.endU), ratOf(line.startU)),
		dv: new(big.Rat).Sub(ratOf(line.endV), ratOf(line.startV)),
	}
	usedShared := make([]bool, len(shared))
	for _, root := range momentCircleRoots(carrier, arc) {
		if root.cmp(big.NewRat(0, 1)) <= 0 || root.cmp(big.NewRat(1, 1)) >= 0 {
			continue
		}
		if !momentArcRootInterior(arc, carrier, root) {
			continue
		}
		if root.consumeSharedPoint(carrier, shared, usedShared) {
			continue
		}
		return true
	}
	return false
}

type momentExactLine struct {
	u, v   *big.Rat
	du, dv *big.Rat
}

type momentRoot struct {
	p     ratPoly
	chain []ratPoly
	iv    ratIv
}

// cmp compares the isolated algebraic root with an exact rational value.
// The Sturm count decides which side holds the root without approximating it.
func (r momentRoot) cmp(value *big.Rat) int {
	if value.Cmp(r.iv.lo) <= 0 {
		return 1
	}
	if value.Cmp(r.iv.hi) >= 0 {
		return -1
	}
	if rpEval(r.p, value).Sign() == 0 {
		return 0
	}
	if sturmCount(r.chain, r.iv.lo, value) > 0 {
		return -1
	}
	return 1
}

// linearSign returns the exact sign of constant+slope*t at this root.
func (r momentRoot) linearSign(constant, slope *big.Rat) int {
	if slope.Sign() == 0 {
		return constant.Sign()
	}
	zero := new(big.Rat).Neg(constant)
	zero.Quo(zero, slope)
	return slope.Sign() * r.cmp(zero)
}

func (r momentRoot) inPointBounds(line momentExactLine, bounds *momentPointBounds) bool {
	if bounds == nil {
		return false
	}
	return r.linearInInterval(line.u, line.du, bounds.u) &&
		r.linearInInterval(line.v, line.dv, bounds.v)
}

func (r momentRoot) consumeSharedPoint(line momentExactLine, bounds []momentPointBounds, used []bool) bool {
	for i := range bounds {
		if used[i] || !r.inPointBounds(line, &bounds[i]) {
			continue
		}
		used[i] = true
		return true
	}
	return false
}

func (r momentRoot) linearInInterval(constant, slope *big.Rat, interval momentRatInterval) bool {
	aboveLo := new(big.Rat).Sub(constant, interval.lo)
	belowHi := new(big.Rat).Sub(constant, interval.hi)
	return r.linearSign(aboveLo, slope) >= 0 &&
		r.linearSign(belowHi, slope) <= 0
}

// momentCircleRoots isolates the two transversal intersections of an exact
// affine line with a circle. The squared direction and discriminant stay
// rational, so neither a tiny segment nor a tiny circle can disappear through
// binary64 underflow. Tangency returns no roots because boundary tangency is
// admissible for mass properties.
func momentCircleRoots(line momentExactLine, circle segmentWalk) []momentRoot {
	fu := new(big.Rat).Sub(line.u, ratOf(circle.cU))
	fv := new(big.Rat).Sub(line.v, ratOf(circle.cV))
	aa := new(big.Rat).Add(
		new(big.Rat).Mul(line.du, line.du),
		new(big.Rat).Mul(line.dv, line.dv),
	)
	if aa.Sign() == 0 {
		return nil
	}
	bb := new(big.Rat).Add(
		new(big.Rat).Mul(fu, line.du),
		new(big.Rat).Mul(fv, line.dv),
	)
	bb.Mul(bb, big.NewRat(2, 1))
	cc := new(big.Rat).Add(
		new(big.Rat).Mul(fu, fu),
		new(big.Rat).Mul(fv, fv),
	)
	cc.Sub(cc, new(big.Rat).Mul(ratOf(circle.radius), ratOf(circle.radius)))
	discriminant := new(big.Rat).Mul(bb, bb)
	fourAC := new(big.Rat).Mul(aa, cc)
	fourAC.Mul(fourAC, big.NewRat(4, 1))
	discriminant.Sub(discriminant, fourAC)
	if discriminant.Sign() <= 0 {
		return nil
	}

	p := ratPoly{cc, bb, aa}
	chain := sturmChain(p)
	bound := rpRootBound(p)
	vertex := new(big.Rat).Neg(bb)
	vertex.Quo(vertex, new(big.Rat).Mul(big.NewRat(2, 1), aa))
	return []momentRoot{
		{p: p, chain: chain, iv: ratIv{lo: new(big.Rat).Neg(bound), hi: vertex}},
		{p: p, chain: chain, iv: ratIv{lo: vertex, hi: bound}},
	}
}

func momentArcsCross(a, b segmentWalk, shared []momentPointBounds) bool {
	if a.cU == b.cU && a.cV == b.cV {
		if a.radius != b.radius {
			return false
		}
		return momentCircularRangesOverlap(a, b)
	}

	// The common chord lies on the exact radical axis. Parameterizing that
	// rational line reduces circle/circle intersections to the same certified
	// quadratic roots used above, without forming an underflowing center
	// distance or NaN intersection coordinates.
	du := new(big.Rat).Sub(ratOf(b.cU), ratOf(a.cU))
	dv := new(big.Rat).Sub(ratOf(b.cV), ratOf(a.cV))
	rhs := new(big.Rat).Sub(
		new(big.Rat).Mul(ratOf(a.radius), ratOf(a.radius)),
		new(big.Rat).Mul(ratOf(b.radius), ratOf(b.radius)),
	)
	bCenter2 := new(big.Rat).Add(
		new(big.Rat).Mul(ratOf(b.cU), ratOf(b.cU)),
		new(big.Rat).Mul(ratOf(b.cV), ratOf(b.cV)),
	)
	aCenter2 := new(big.Rat).Add(
		new(big.Rat).Mul(ratOf(a.cU), ratOf(a.cU)),
		new(big.Rat).Mul(ratOf(a.cV), ratOf(a.cV)),
	)
	rhs.Add(rhs, new(big.Rat).Sub(bCenter2, aCenter2))
	rhs.Quo(rhs, big.NewRat(2, 1))

	carrier := momentExactLine{
		u:  new(big.Rat),
		v:  new(big.Rat),
		du: new(big.Rat).Neg(dv),
		dv: new(big.Rat).Set(du),
	}
	if du.Sign() != 0 {
		carrier.u.Quo(rhs, du)
	} else {
		carrier.v.Quo(rhs, dv)
	}
	usedShared := make([]bool, len(shared))
	for _, root := range momentCircleRoots(carrier, a) {
		if !momentArcRootInterior(a, carrier, root) || !momentArcRootInterior(b, carrier, root) {
			continue
		}
		if root.consumeSharedPoint(carrier, shared, usedShared) {
			continue
		}
		return true
	}
	return false
}

func momentCircularRangesOverlap(a, b segmentWalk) bool {
	aLo, aHi := math.Min(a.th0, a.th1), math.Max(a.th0, a.th1)
	bLo, bHi := math.Min(b.th0, b.th1), math.Max(b.th0, b.th1)
	period := 2 * math.Pi
	for shift := -2; shift <= 2; shift++ {
		lo := math.Max(aLo, bLo+float64(shift)*period)
		hi := math.Min(aHi, bHi+float64(shift)*period)
		if lo < hi {
			return true
		}
	}
	return false
}

func momentArcRootInterior(arc segmentWalk, line momentExactLine, root momentRoot) bool {
	if arc.closed {
		return true
	}
	centerU, centerV := ratOf(arc.cU), ratOf(arc.cV)
	startU := new(big.Rat).Sub(ratOf(arc.startU), centerU)
	startV := new(big.Rat).Sub(ratOf(arc.startV), centerV)
	endU := new(big.Rat).Sub(ratOf(arc.endU), centerU)
	endV := new(big.Rat).Sub(ratOf(arc.endV), centerV)
	baseU := new(big.Rat).Sub(line.u, centerU)
	baseV := new(big.Rat).Sub(line.v, centerV)

	startConstant := new(big.Rat).Sub(
		new(big.Rat).Mul(startU, baseV),
		new(big.Rat).Mul(startV, baseU),
	)
	startSlope := new(big.Rat).Sub(
		new(big.Rat).Mul(startU, line.dv),
		new(big.Rat).Mul(startV, line.du),
	)
	endConstant := new(big.Rat).Sub(
		new(big.Rat).Mul(baseU, endV),
		new(big.Rat).Mul(baseV, endU),
	)
	endSlope := new(big.Rat).Sub(
		new(big.Rat).Mul(line.du, endV),
		new(big.Rat).Mul(line.dv, endU),
	)
	startSign := root.linearSign(startConstant, startSlope)
	endSign := root.linearSign(endConstant, endSlope)
	if arc.th1 < arc.th0 {
		startSign = -startSign
		endSign = -endSign
	}
	if math.Abs(arc.th1-arc.th0) <= math.Pi {
		return startSign > 0 && endSign > 0
	}
	return startSign > 0 || endSign > 0
}

// momentNestingValid uses several boundary probes per segment. A probe proven
// outside the outer loop or inside another hole rejects the record. An
// undecidable probe can be a permitted tangency, so it never admits or rejects
// by itself; proven transversal crossings were already rejected above.
func momentNestingValid(segs []segEntry, nLoops int) bool {
	if nLoops <= 1 {
		return true
	}
	bounds := make([][]surveyElem, nLoops)
	probes := make([][][2]float64, nLoops)
	for _, seg := range segs {
		if elem, ok := elemOf(seg.w); ok {
			bounds[seg.loop] = append(bounds[seg.loop], elem)
		}
		probes[seg.loop] = append(probes[seg.loop], momentSegmentProbes(seg.w)...)
	}
	if _, _, _, _, ok := sectionBBox(segs); !ok {
		return false
	}
	for hole := 1; hole < nLoops; hole++ {
		provenInside := false
		for _, probe := range probes[hole] {
			tolerance := momentContainmentTolerance(bounds[0], probe[0], probe[1])
			inside, decided := loopContains(bounds[0], probe[0], probe[1], tolerance)
			if decided && !inside {
				return false
			}
			provenInside = provenInside || decided && inside
		}
		if !provenInside {
			return false
		}
	}
	for a := 1; a < nLoops; a++ {
		for b := a + 1; b < nLoops; b++ {
			aOutsideB := false
			for _, probe := range probes[a] {
				tolerance := momentContainmentTolerance(bounds[b], probe[0], probe[1])
				inside, decided := loopContains(bounds[b], probe[0], probe[1], tolerance)
				if decided && inside {
					return false
				}
				aOutsideB = aOutsideB || decided && !inside
			}
			bOutsideA := false
			for _, probe := range probes[b] {
				tolerance := momentContainmentTolerance(bounds[a], probe[0], probe[1])
				inside, decided := loopContains(bounds[a], probe[0], probe[1], tolerance)
				if decided && inside {
					return false
				}
				bOutsideA = bOutsideA || decided && !inside
			}
			if !aOutsideB || !bOutsideA {
				return false
			}
		}
	}
	return true
}

func momentSegmentProbes(w segmentWalk) [][2]float64 {
	midU, midV := (w.startU+w.endU)/2, (w.startV+w.endV)/2
	if w.circular {
		midAngle := (w.th0 + w.th1) / 2
		midU = w.cU + w.radius*math.Cos(midAngle)
		midV = w.cV + w.radius*math.Sin(midAngle)
	}
	return [][2]float64{{w.startU, w.startV}, {midU, midV}}
}

// momentContainmentTolerance bounds only local floating evaluation around the
// probe. It does not turn the section's contact floor or absolute translation
// into an admission margin, so every representable positive gap remains
// classifiable once it is wider than local rounding.
func momentContainmentTolerance(boundary []surveyElem, probeU, probeV float64) float64 {
	scale := 0.0
	grow := func(values ...float64) {
		for _, value := range values {
			scale = math.Max(scale, math.Abs(value))
		}
	}
	for _, elem := range boundary {
		if elem.kind == surveyLine {
			grow(
				elem.ax-probeU, elem.ay-probeV,
				elem.bx-probeU, elem.by-probeV,
			)
			continue
		}
		grow(elem.qx-probeU, elem.qy-probeV, elem.rr)
	}
	return momentULPAllowance(scale)
}

type momentWalk struct {
	segmentWalk
	startUInterval momentRatInterval
	startVInterval momentRatInterval
	endUInterval   momentRatInterval
	endVInterval   momentRatInterval
}

type momentRatInterval struct {
	lo, hi *big.Rat
}

func (iv momentRatInterval) overlaps(other momentRatInterval) bool {
	return iv.lo.Cmp(other.hi) <= 0 && other.lo.Cmp(iv.hi) <= 0
}

func momentBoundedInterval(value float64, bound *big.Rat) momentRatInterval {
	center := ratOf(value)
	return momentRatInterval{
		lo: new(big.Rat).Sub(center, bound),
		hi: new(big.Rat).Add(center, bound),
	}
}

// momentLineCoordinateInterval bounds the exact interpolation result around
// the binary64 value that lerp2 actually produced. The bound is derived from
// that evaluation, not from the absolute source-coordinate scale.
func momentLineCoordinateInterval(start, end, t, value float64) momentRatInterval {
	exact := new(big.Rat).Sub(ratOf(end), ratOf(start))
	exact.Mul(exact, ratOf(t))
	exact.Add(exact, ratOf(start))
	errorBound := new(big.Rat).Sub(ratOf(value), exact)
	errorBound.Abs(errorBound)
	return momentBoundedInterval(value, errorBound)
}

// validateMomentLoop checks the part of ProfileRecord's structural contract
// the closed-form integrator relies on: every supported segment has finite,
// usable walk geometry and the directed walks join into one closed boundary.
func validateMomentLoop(loop LoopRecord) error {
	if len(loop.Segments) == 0 {
		return fmt.Errorf(`%w: a recorded loop holds no segments`, ErrDegenerate)
	}
	walks := make([]momentWalk, len(loop.Segments))
	for i, seg := range loop.Segments {
		walk, err := validateMomentSegment(seg)
		if err != nil {
			return fmt.Errorf(`segment %d: %w`, i, err)
		}
		walks[i] = walk
	}
	if len(walks) == 1 && walks[0].closed {
		if momentEndpointsJoin(walks[0], walks[0]) {
			return nil
		}
		return fmt.Errorf(
			`%w: the closed segment ends at (%g, %g), not its start (%g, %g)`,
			ErrDegenerate, walks[0].endU, walks[0].endV, walks[0].startU, walks[0].startV,
		)
	}
	for i, walk := range walks {
		if walk.closed {
			return fmt.Errorf(`%w: a closed segment cannot share a loop with another segment`, ErrDegenerate)
		}
		next := walks[(i+1)%len(walks)]
		if !momentEndpointsJoin(walk, next) {
			return fmt.Errorf(
				`%w: segment %d ends at (%g, %g), not segment %d's start (%g, %g)`,
				ErrDegenerate, i, walk.endU, walk.endV, (i+1)%len(walks), next.startU, next.startV,
			)
		}
	}
	return nil
}

func validateMomentSegment(seg CurveSegment) (momentWalk, error) {
	seg, err := normalizeSegment(seg)
	if err != nil {
		return momentWalk{}, err
	}
	if seg == nil {
		return momentWalk{}, errNilSegment
	}
	switch seg := seg.(type) {
	case LineSeg:
		if !finiteMomentValues(seg.Start.U, seg.Start.V, seg.End.U, seg.End.V, seg.TStart, seg.TEnd) {
			return momentWalk{}, fmt.Errorf(`%w: a line segment field is not finite`, ErrNotFinite)
		}
		if err := validateMomentRange(seg.TStart, seg.TEnd); err != nil {
			return momentWalk{}, err
		}
	case CircleSeg:
		if _, err := magnitudeIn(seg.Radius, units.Length, units.Millimeter, "a circle segment's radius"); err != nil {
			return momentWalk{}, err
		}
		if !finiteMomentValues(seg.Center.U, seg.Center.V, seg.TStart, seg.TEnd) {
			return momentWalk{}, fmt.Errorf(`%w: a circle segment field is not finite`, ErrNotFinite)
		}
		if err := validateMomentRange(seg.TStart, seg.TEnd); err != nil {
			return momentWalk{}, err
		}
	case ArcSeg:
		if !finiteMomentValues(
			seg.Center.U, seg.Center.V,
			seg.Start.U, seg.Start.V,
			seg.End.U, seg.End.V,
			seg.TStart, seg.TEnd,
		) {
			return momentWalk{}, fmt.Errorf(`%w: an arc segment field is not finite`, ErrNotFinite)
		}
		if err := validateMomentRange(seg.TStart, seg.TEnd); err != nil {
			return momentWalk{}, err
		}
		startRadius := math.Hypot(seg.Start.U-seg.Center.U, seg.Start.V-seg.Center.V)
		endRadius := math.Hypot(seg.End.U-seg.Center.U, seg.End.V-seg.Center.V)
		if !momentCoordinateJoins(startRadius, endRadius) {
			return momentWalk{}, fmt.Errorf(
				`%w: an arc segment's pinned start and end radii differ (%g and %g)`,
				ErrDegenerate, startRadius, endRadius,
			)
		}
	default:
		return momentWalk{}, fmt.Errorf(`%w: this evaluator computes mass properties over line, arc and circle profile segments only; the profile has a %T segment`, ErrUnsupported, seg)
	}

	walk, err := walkOf(seg)
	if err != nil {
		return momentWalk{}, err
	}
	if line, ok := seg.(LineSeg); ok {
		walk.startU, walk.startV = momentLineEndpoint(line, line.TStart)
		walk.endU, walk.endV = momentLineEndpoint(line, line.TEnd)
	}
	if !finiteMomentValues(
		walk.startU, walk.startV, walk.endU, walk.endV,
		walk.tanInU, walk.tanInV, walk.tanOutU, walk.tanOutV,
		walk.length, walk.cU, walk.cV, walk.radius, walk.th0, walk.th1,
	) {
		return momentWalk{}, fmt.Errorf(`%w: a segment's derived walk is not finite`, ErrNotFinite)
	}
	if walk.circular && walk.radius < 0 {
		return momentWalk{}, fmt.Errorf(`%w: a circular segment radius must be non-negative`, ErrNegativeMagnitude)
	}
	if walk.length <= 0 {
		return momentWalk{}, fmt.Errorf(`%w: a zero-length segment contributes no boundary`, ErrDegenerate)
	}
	out := momentWalk{
		segmentWalk:    walk,
		startUInterval: momentBoundedInterval(walk.startU, new(big.Rat)),
		startVInterval: momentBoundedInterval(walk.startV, new(big.Rat)),
		endUInterval:   momentBoundedInterval(walk.endU, new(big.Rat)),
		endVInterval:   momentBoundedInterval(walk.endV, new(big.Rat)),
	}
	switch seg := seg.(type) {
	case LineSeg:
		out.startUInterval = momentLineCoordinateInterval(seg.Start.U, seg.End.U, seg.TStart, walk.startU)
		out.startVInterval = momentLineCoordinateInterval(seg.Start.V, seg.End.V, seg.TStart, walk.startV)
		out.endUInterval = momentLineCoordinateInterval(seg.Start.U, seg.End.U, seg.TEnd, walk.endU)
		out.endVInterval = momentLineCoordinateInterval(seg.Start.V, seg.End.V, seg.TEnd, walk.endV)
	case CircleSeg, ArcSeg:
		allowance := ratOf(momentEndpointAllowance(walk.radius))
		out.startUInterval = momentBoundedInterval(walk.startU, allowance)
		out.startVInterval = momentBoundedInterval(walk.startV, allowance)
		out.endUInterval = momentBoundedInterval(walk.endU, allowance)
		out.endVInterval = momentBoundedInterval(walk.endV, allowance)
	}
	return out, nil
}

func momentLineEndpoint(line LineSeg, t float64) (float64, float64) {
	switch t {
	case 0:
		return line.Start.U, line.Start.V
	case 1:
		return line.End.U, line.End.V
	default:
		u, v := lerp2(line.Start, line.End, t)
		return u, v
	}
}

func momentEndpointAllowance(radius float64) float64 {
	// Circular endpoints compose angle, trigonometric and multiply-add steps.
	// Their allowance remains local to the radius and never includes model
	// translation. Line endpoints use their exact evaluation bounds above.
	return momentULPAllowance(radius)
}

func validateMomentRange(start, end float64) error {
	if start < 0 || start > 1 || end < 0 || end > 1 {
		return fmt.Errorf(`%w: a segment range must stay within [0, 1]`, ErrDegenerate)
	}
	if start == end {
		return fmt.Errorf(`%w: a zero-length segment range contributes no boundary`, ErrDegenerate)
	}
	return nil
}

// momentEndpointsJoin requires the two proven endpoint-evaluation intervals to
// overlap. It is intentionally unrelated to the model's geometric scale
// tolerance.
func momentEndpointsJoin(a, b momentWalk) bool {
	return a.endUInterval.overlaps(b.startUInterval) &&
		a.endVInterval.overlaps(b.startVInterval)
}

func momentCoordinateJoins(a, b float64) bool {
	scale := math.Max(math.Abs(a), math.Abs(b))
	return math.Abs(a-b) <= momentULPAllowance(scale)
}

func momentULPAllowance(scale float64) float64 {
	scale = math.Abs(scale)
	if scale == 0 {
		return 0
	}
	ulp := scale - math.Nextafter(scale, 0)
	// The local arc and ray formulas compose subtraction, hypot, angle,
	// trigonometric and multiply-add steps. This budget covers that bounded
	// chain without admitting a model-translation term.
	return 1024 * ulp
}

// add accumulates one segment's boundary-integral contribution, in the
// segment's recorded walk direction (TStart→TEnd, which TStart > TEnd runs
// against the curve's natural sense — the sign falls out of the integral
// limits, no special case).
func (ig *regionIntegrals) add(seg CurveSegment) error {
	// The codec accepts pointer variants and normalizes them to values
	// (record.go); the integrals read segments on the same terms, so a
	// *LineSeg integrates exactly like its value and a nil pointer is
	// rejected rather than misread as an unsupported kind.
	seg, err := normalizeSegment(seg)
	if err != nil {
		return err
	}
	switch seg := seg.(type) {
	case LineSeg:
		ig.addLine(seg)
		return nil
	case CircleSeg:
		r, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return fmt.Errorf(`decad: a circle segment's radius is not a length: %w`, err)
		}
		// The record carries the walk twice — the range order, and CCW —
		// and the two must agree (a reversed walk swaps the range AND flips
		// CCW, seam §2). A record that contradicts itself is malformed, not
		// a judgement call.
		if seg.CCW != (seg.TStart < seg.TEnd) {
			return fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, ErrDegenerate)
		}
		// The arrangement's normalized t is the angle 2π·t from +u
		// (geom.BoundaryEdge); the recorded range order is the walk.
		ig.addCircular(seg.Center, r, 2*math.Pi*seg.TStart, 2*math.Pi*seg.TEnd)
		return nil
	case ArcSeg:
		// geom.Arc's derived readings, computed on the record's own pinned
		// points: radius from Center→Start, angles about the center, the
		// sweep CCW-wrapped into (0, 2π].
		radius := math.Hypot(seg.Start.U-seg.Center.U, seg.Start.V-seg.Center.V)
		a0 := math.Atan2(seg.Start.V-seg.Center.V, seg.Start.U-seg.Center.U)
		a1 := math.Atan2(seg.End.V-seg.Center.V, seg.End.U-seg.Center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		// normalized t maps to angle = a0 + t·sweep; the range order is the walk.
		ig.addCircular(seg.Center, radius, a0+seg.TStart*sweep, a0+seg.TEnd*sweep)
		return nil
	default:
		return fmt.Errorf(`%w: this evaluator computes mass properties over line, arc and circle profile segments only; the profile has a %T segment`, ErrUnsupported, seg)
	}
}

// addLine accumulates the straight chord from the walk's start point to its
// end point. The recorded range picks the walked piece of the entity's own
// Start→End parameterization.
func (ig *regionIntegrals) addLine(seg LineSeg) {
	u0, v0 := lerp2(seg.Start, seg.End, seg.TStart)
	u1, v1 := lerp2(seg.Start, seg.End, seg.TEnd)
	// A     = ½ ∮ (u dv − v du)
	// ∫u dA = ½ ∮ u² dv
	// ∫v dA = −½ ∮ v² du
	ig.area += 0.5 * (u0*v1 - u1*v0)
	ig.mu += (v1 - v0) * (u0*u0 + u0*u1 + u1*u1) / 6
	ig.mv += -(u1 - u0) * (v0*v0 + v0*v1 + v1*v1) / 6

	// ∫u² dA = ⅓ ∮ u³ dv;  ∫v² dA = −⅓ ∮ v³ du — the cubic sums are the
	// exact ∫₀¹ of the lerp cubed.
	ig.muu += (v1 - v0) * (u0*u0*u0 + u0*u0*u1 + u0*u1*u1 + u1*u1*u1) / 12
	ig.mvv += -(u1 - u0) * (v0*v0*v0 + v0*v0*v1 + v0*v1*v1 + v1*v1*v1) / 12

	// ∫uv dA = ½ ∮ u²v dv: expand u(t)² v(t) and integrate the polynomial
	// exactly, term by term.
	du, dv := u1-u0, v1-v0
	intU2V := v0*(u0*u0+u0*du+du*du/3) + dv*(u0*u0/2+2*u0*du/3+du*du/4)
	ig.muv += 0.5 * dv * intU2V
}

// addCircular accumulates a circular path about center c with radius r, from
// angle th0 to th1 in the walk direction (th1 < th0 walks clockwise). The
// antiderivatives are exact, so a whole period, a fragment, and a reversed
// walk are all the same formula.
func (ig *regionIntegrals) addCircular(c Point2, r, th0, th1 float64) {
	sin0, cos0 := math.Sincos(th0)
	sin1, cos1 := math.Sincos(th1)
	dth := th1 - th0

	// A = ½ ∫ (u v′ − v u′) dθ = ½ [r²·θ + c_u·r·sin θ + c_v·r·cos θ]
	ig.area += 0.5 * (r*r*dth + c.U*r*(sin1-sin0) - c.V*r*(cos1-cos0))

	// ∫u dA = ½ ∮ u² dv, with u = c_u + r cos θ, dv = r cos θ dθ:
	//   ½ r [c_u²·sin θ + 2 c_u r (θ/2 + sin 2θ/4) + r²(sin θ − sin³θ/3)]
	intCos := sin1 - sin0
	intCos2 := dth/2 + (math.Sin(2*th1)-math.Sin(2*th0))/4
	intCos3 := (sin1 - sin1*sin1*sin1/3) - (sin0 - sin0*sin0*sin0/3)
	ig.mu += 0.5 * r * (c.U*c.U*intCos + 2*c.U*r*intCos2 + r*r*intCos3)

	// ∫v dA = −½ ∮ v² du, with v = c_v + r sin θ, du = −r sin θ dθ:
	//   ½ r [c_v²·(−cos θ)′... i.e. ½ r ∫ v² sin θ dθ]
	intSin := cos0 - cos1
	intSin2 := dth/2 - (math.Sin(2*th1)-math.Sin(2*th0))/4
	intSin3 := (cos0 - cos0*cos0*cos0/3) - (cos1 - cos1*cos1*cos1/3)
	ig.mv += 0.5 * r * (c.V*c.V*intSin + 2*c.V*r*intSin2 + r*r*intSin3)

	// The quartic antiderivatives for the second moments:
	//   ∫cos⁴θ dθ = 3θ/8 + sin 2θ/4 + sin 4θ/32
	//   ∫sin⁴θ dθ = 3θ/8 − sin 2θ/4 + sin 4θ/32
	intCos4 := 3*dth/8 + (math.Sin(2*th1)-math.Sin(2*th0))/4 + (math.Sin(4*th1)-math.Sin(4*th0))/32
	intSin4 := 3*dth/8 - (math.Sin(2*th1)-math.Sin(2*th0))/4 + (math.Sin(4*th1)-math.Sin(4*th0))/32

	// ∫u² dA = ⅓ ∮ u³ dv, dv = r cos θ dθ, u³ expanded about the center.
	ig.muu += r / 3 * (c.U*c.U*c.U*intCos + 3*c.U*c.U*r*intCos2 + 3*c.U*r*r*intCos3 + r*r*r*intCos4)

	// ∫v² dA = −⅓ ∮ v³ du, du = −r sin θ dθ, v³ expanded about the center.
	ig.mvv += r / 3 * (c.V*c.V*c.V*intSin + 3*c.V*c.V*r*intSin2 + 3*c.V*r*r*intSin3 + r*r*r*intSin4)

	// ∫uv dA = ½ ∮ u²v dv: u²v = c_v·u² + r sin θ·u², with
	//   ∫sin θ cos θ dθ = sin²θ/2, ∫sin θ cos²θ dθ = −cos³θ/3,
	//   ∫sin θ cos³θ dθ = −cos⁴θ/4.
	intSC := (sin1*sin1 - sin0*sin0) / 2
	intSC2 := (cos0*cos0*cos0 - cos1*cos1*cos1) / 3
	intSC3 := (cos0*cos0*cos0*cos0 - cos1*cos1*cos1*cos1) / 4
	ig.muv += 0.5 * r * (c.V*(c.U*c.U*intCos+2*c.U*r*intCos2+r*r*intCos3) +
		r*(c.U*c.U*intSC+2*c.U*r*intSC2+r*r*intSC3))
}

// lerp2 returns the point at parameter t on the segment start→end.
func lerp2(start, end Point2, t float64) (float64, float64) {
	return start.U + t*(end.U-start.U), start.V + t*(end.V-start.V)
}
