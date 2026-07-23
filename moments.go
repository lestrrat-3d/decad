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
// segment kind. Exactly representable line results retain zero bounds;
// circular evaluations carry outward bounds. Free-form kinds are unsupported.

// Area returns the recorded region's net area — the outer loop minus its
// holes — as a [Measurement] of Kind Area (mm²): a computed quantity carries
// its Exactness and Bound (docs/api-design.md §6). Each boundary segment
// contributes its Green's-theorem integral in walk order, so a hole's clockwise
// walk subtracts without a special case. Line contributions are accumulated
// against exact rationals; circular contributions carry a proven float
// evaluation bound.
//
// A region whose boundary contains a free-form segment kind (ellipse,
// elliptical arc, conic, spline, closed spline, fit spline, NURBS) is
// [ErrUnsupported] (docs/evaluator-design.md §11) — never approximated.
// A malformed or open record is [ErrDegenerate]. A circle radius of the wrong
// kind is [ErrUnitKind], a negative radius is [ErrNegativeMagnitude], and a
// non-finite field or arithmetic result is [ErrNotFinite]. No measurement is
// returned on error.
func (r ProfileRecord) Area() (Measurement, error) {
	ig, err := r.integralsTo(momentAreaOrder)
	if err != nil {
		return Measurement{}, err
	}
	return Measurement{
		Value:     units.SquareMillimeters(ig.area),
		Exactness: exactnessOf(ig.areaBound),
		Bound:     units.SquareMillimeters(ig.areaBound),
	}, nil
}

// Centroid returns the recorded region's centroid from its bounded first
// moments, as a [VecMeasurement] — a computed coordinate is a measurement
// (docs/api-design.md §6). The Value is PLANE-LOCAL: (u, v, 0) in the
// region's own plane coordinates, millimetres (§5.2), not a world position —
// lift it through the profile's PlaneRecord to place it in space.
//
// A region whose net area is zero has no centroid and is [ErrDegenerate].
// Record validation and arithmetic errors match [ProfileRecord.Area].
func (r ProfileRecord) Centroid() (VecMeasurement, error) {
	ig, err := r.integralsTo(momentFirstOrder)
	if err != nil {
		return VecMeasurement{}, err
	}
	if ig.area == 0 && ig.areaBound == 0 {
		return VecMeasurement{}, fmt.Errorf(`%w: a region with zero net area has no centroid`, ErrDegenerate)
	}
	if math.Abs(ig.area) <= ig.areaBound {
		return VecMeasurement{}, fmt.Errorf(`%w: the evaluator cannot prove this region's net area stays away from zero`, ErrUnsupported)
	}
	u := boundedQuotient(ig.mu, ig.muBound, ig.area, ig.areaBound)
	v := boundedQuotient(ig.mv, ig.mvBound, ig.area, ig.areaBound)
	bound := radius2D(u.bound, v.bound)
	geometryBound := radius2D(
		upRound(math.Abs(u.value)+ig.coordUpper),
		upRound(math.Abs(v.value)+ig.coordUpper),
	)
	bound = math.Min(bound, geometryBound)
	return VecMeasurement{
		Value:     r3.NewVec(u.value, v.value, 0),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// SecondMoments is a recorded region's second moments of area about the
// plane origin, in the plane's own (u, v): every field is a Measurement of
// Kind SecondMomentOfArea (mm⁴), with the closed-form evaluation's proven
// rounding bound. They are what a revolve's solid centroid is computed from
// (docs/evaluator-design.md §4/§6); to re-reference them to another axis, use
// the parallel-axis theorem with the region's Area and Centroid.
type SecondMoments struct {
	// UU is ∫u² dA, VV is ∫v² dA, UV is the mixed ∫uv dA.
	UU Measurement
	UV Measurement
	VV Measurement
}

// SecondMoments returns the region's bounded second moments of area about the
// plane origin. The staging matches Area: a free-form boundary kind is
// [ErrUnsupported], and malformed or non-finite records are rejected before a
// measurement is constructed.
func (r ProfileRecord) SecondMoments() (SecondMoments, error) {
	ig, err := r.integralsTo(momentSecondOrder)
	if err != nil {
		return SecondMoments{}, err
	}
	measured := func(x, bound float64) Measurement {
		return Measurement{
			Value:     units.QuarticMillimeters(x),
			Exactness: exactnessOf(bound),
			Bound:     units.QuarticMillimeters(bound),
		}
	}
	return SecondMoments{
		UU: measured(ig.muu, ig.muuBound),
		UV: measured(ig.muv, ig.muvBound),
		VV: measured(ig.mvv, ig.mvvBound),
	}, nil
}

// regionIntegrals accumulates the boundary integrals of one region: the net
// signed area, the first moments ∫u dA and ∫v dA, and the second moments
// ∫u² dA, ∫uv dA and ∫v² dA.
type regionIntegrals struct {
	coordUpper float64 // max |u|+|v| over the recorded material
	area       float64
	areaBound  float64
	mu         float64 // ∫u dA
	muBound    float64
	mv         float64 // ∫v dA
	mvBound    float64
	muu        float64 // ∫u² dA
	muuBound   float64
	muv        float64 // ∫uv dA
	muvBound   float64
	mvv        float64 // ∫v² dA
	mvvBound   float64
}

// boundedScalar is a held float64 and a proven absolute error bound. The
// arithmetic helpers below propagate input intervals and add the exact
// round-to-nearest error of the held operation, measured with big.Rat. This
// keeps a result Exact only when both its inputs and its final float are exact.
type boundedScalar struct {
	value float64
	bound float64
}

func exactScalar(value float64) boundedScalar {
	return boundedScalar{value: value}
}

func measuredScalar(value, bound float64) boundedScalar {
	return boundedScalar{value: value, bound: bound}
}

func floatRat(value float64) *big.Rat {
	r := new(big.Rat)
	if r.SetFloat64(value) == nil {
		return nil
	}
	return r
}

// rationalFloatError returns |exact-held| rounded upward.
func rationalFloatError(exact *big.Rat, held float64) float64 {
	heldRat := floatRat(held)
	if exact == nil || heldRat == nil {
		return math.Inf(1)
	}
	d := new(big.Rat).Sub(exact, heldRat)
	d.Abs(d)
	out, exactFloat := d.Float64()
	if !exactFloat {
		out = math.Nextafter(out, math.Inf(1))
	}
	return out
}

func addRoundError(a, b, held float64) float64 {
	ra, rb := floatRat(a), floatRat(b)
	if ra == nil || rb == nil {
		return math.Inf(1)
	}
	return rationalFloatError(new(big.Rat).Add(ra, rb), held)
}

func mulRoundError(a, b, held float64) float64 {
	ra, rb := floatRat(a), floatRat(b)
	if ra == nil || rb == nil {
		return math.Inf(1)
	}
	return rationalFloatError(new(big.Rat).Mul(ra, rb), held)
}

func divRoundError(a, b, held float64) float64 {
	ra, rb := floatRat(a), floatRat(b)
	if ra == nil || rb == nil || rb.Sign() == 0 {
		return math.Inf(1)
	}
	return rationalFloatError(new(big.Rat).Quo(ra, rb), held)
}

func boundedAdd(a, b boundedScalar) boundedScalar {
	value := a.value + b.value
	bound := absSumUpper(a.bound, b.bound, addRoundError(a.value, b.value, value))
	return measuredScalar(value, bound)
}

func boundedSub(a, b boundedScalar) boundedScalar {
	return boundedAdd(a, measuredScalar(-b.value, b.bound))
}

func boundedAbs(a boundedScalar) boundedScalar {
	a.value = math.Abs(a.value)
	return a
}

func boundedMul(a, b boundedScalar) boundedScalar {
	value := a.value * b.value
	bound := absSumUpper(
		productUpper(math.Abs(a.value), b.bound),
		productUpper(math.Abs(b.value), a.bound),
		productUpper(a.bound, b.bound),
		mulRoundError(a.value, b.value, value),
	)
	return measuredScalar(value, bound)
}

func boundedQuotient(num float64, numBound float64, den float64, denBound float64) boundedScalar {
	value := num / den
	clearance := math.Nextafter(math.Abs(den)-denBound, math.Inf(-1))
	if clearance <= 0 {
		return measuredScalar(value, math.Inf(1))
	}
	centralRound := divRoundError(num, den, value)
	centralUpper := absSumUpper(value, centralRound)
	numerator := absSumUpper(numBound, productUpper(centralUpper, denBound))
	bound := upRound(numerator / clearance)
	return measuredScalar(value, absSumUpper(bound, centralRound))
}

func boundedDiv(a, b boundedScalar) boundedScalar {
	return boundedQuotient(a.value, a.bound, b.value, b.bound)
}

func boundedSin(x boundedScalar) boundedScalar {
	value := math.Sin(x.value)
	return measuredScalar(value, conservativeValueError(value, 1))
}

func boundedCos(x boundedScalar) boundedScalar {
	value := math.Cos(x.value)
	return measuredScalar(value, conservativeValueError(value, 1))
}

// radius2D turns independent coordinate bounds into a plane-distance bound.
// sqrt2Up is √2 rounded upward, so the square containing both coordinate
// intervals is enclosed without relying on a rounded square root.
func radius2D(x, y float64) float64 {
	const sqrt2Up = 1.4142135623730952
	if x <= 0 && y <= 0 {
		return 0
	}
	return productUpper(math.Max(x, y), sqrt2Up)
}

// analyticRoundBound is the analytic evaluator's basic-arithmetic roundoff
// budget. Each caller supplies an absolute-term envelope and evaluates fewer
// than 128 additions, multiplications or divisions, so 256·u·scale dominates
// their round-to-nearest error without cancellation shrinking the bound.
//
// Go deliberately gives Sin, Cos, Atan2 and Hypot no public ulp contract. Their
// results never use this helper. conservativeValueError instead applies the
// triangle inequality to a mathematical magnitude envelope, so those results
// remain bounded without assuming an undocumented library accuracy.
func analyticRoundBound(scale float64) float64 {
	if scale <= 0 {
		return 0
	}
	if isNonFinite(scale) {
		return math.Inf(1)
	}
	return productUpper(productUpper(256, unitRoundoff), scale)
}

// conservativeValueError proves |held-true| from |true| <= trueAbsUpper:
// |held-true| <= |held|+|true|. It is intentionally wider than an ulp estimate,
// but it is portable across every conforming implementation of Go's math
// package and remains finite for finite geometry.
func conservativeValueError(held, trueAbsUpper float64) float64 {
	if isNonFinite(held) || isNonFinite(trueAbsUpper) {
		return math.Inf(1)
	}
	return absSumUpper(held, math.Max(0, trueAbsUpper))
}

func absSumUpper(values ...float64) float64 {
	total := 0.0
	for _, value := range values {
		total = upRound(total + math.Abs(value))
	}
	return total
}

func productUpper(a, b float64) float64 {
	if a <= 0 || b <= 0 {
		return 0
	}
	return upRound(a * b)
}

func twoPiUpper() float64 {
	return productUpper(2, math.Nextafter(math.Pi, math.Inf(1)))
}

// circularSweepUpper bounds |(t1-t0)·2π| for both CircleSeg and ArcSeg.
// ArcSeg's underlying sweep is at most 2π; CircleSeg uses exactly that scale.
func circularSweepUpper(t0, t1 float64) float64 {
	return productUpper(absSumUpper(t0, t1), twoPiUpper())
}

func arcRadiusUpper(seg ArcSeg) float64 {
	// The exact coordinate differences can each be no larger than the sum of
	// their input magnitudes, and hypot is no larger than the L1 norm.
	return absSumUpper(seg.Start.U, seg.Center.U, seg.Start.V, seg.Center.V)
}

type ratInterval struct {
	lo *big.Rat
	hi *big.Rat
}

var (
	piLower = mustRatDecimal("3.141592653589793238462643383279502884197169399375105820974944592307816406286")
	piUpper = mustRatDecimal("3.141592653589793238462643383279502884197169399375105820974944592307816406287")
)

func mustRatDecimal(value string) *big.Rat {
	out, ok := new(big.Rat).SetString(value)
	if !ok {
		panic("decad: invalid in-tree rational constant")
	}
	return out
}

func interval(lo, hi *big.Rat) ratInterval {
	return ratInterval{lo: new(big.Rat).Set(lo), hi: new(big.Rat).Set(hi)}
}

func intervalAdd(a, b ratInterval) ratInterval {
	return interval(new(big.Rat).Add(a.lo, b.lo), new(big.Rat).Add(a.hi, b.hi))
}

func intervalNeg(a ratInterval) ratInterval {
	return interval(new(big.Rat).Neg(a.hi), new(big.Rat).Neg(a.lo))
}

func intervalSub(a, b ratInterval) ratInterval {
	return intervalAdd(a, intervalNeg(b))
}

func intervalScale(a ratInterval, scale *big.Rat) ratInterval {
	if scale.Sign() < 0 {
		return interval(
			new(big.Rat).Mul(a.hi, scale),
			new(big.Rat).Mul(a.lo, scale),
		)
	}
	return interval(
		new(big.Rat).Mul(a.lo, scale),
		new(big.Rat).Mul(a.hi, scale),
	)
}

func pointInterval(value *big.Rat) ratInterval {
	return interval(value, value)
}

func intervalFloatError(a ratInterval, held float64) float64 {
	return math.Max(
		rationalFloatError(a.lo, held),
		rationalFloatError(a.hi, held),
	)
}

// atanSmallInterval bounds atan(x) for |x| <= 1/2 with an exact rational
// alternating series. Sixty-four terms leave a remainder below 2^-128, far
// below float64 resolution; the first omitted term is the rigorous remainder.
func atanSmallInterval(x *big.Rat) ratInterval {
	if x.Sign() < 0 {
		return intervalNeg(atanSmallInterval(new(big.Rat).Neg(x)))
	}
	x2 := new(big.Rat).Mul(x, x)
	power := new(big.Rat).Set(x)
	sum := new(big.Rat)
	for n := range 64 {
		term := new(big.Rat).Quo(power, big.NewRat(int64(2*n+1), 1))
		if n%2 == 0 {
			sum.Add(sum, term)
		} else {
			sum.Sub(sum, term)
		}
		power.Mul(power, x2)
	}
	remainder := new(big.Rat).Quo(power, big.NewRat(129, 1))
	// The last included term is negative, so the next positive term bounds
	// the exact value above the partial sum.
	return interval(sum, new(big.Rat).Add(sum, remainder))
}

func atanPositiveInterval(x *big.Rat) ratInterval {
	if x.Cmp(big.NewRat(1, 2)) <= 0 {
		return atanSmallInterval(x)
	}
	// atan(x) = π/4 + atan((x-1)/(x+1)); for x in (1/2,1], the transformed
	// argument lies in [-1/3,0), inside the fast alternating-series range.
	q := new(big.Rat).Quo(
		new(big.Rat).Sub(x, big.NewRat(1, 1)),
		new(big.Rat).Add(x, big.NewRat(1, 1)),
	)
	quarterPi := intervalScale(interval(piLower, piUpper), big.NewRat(1, 4))
	return intervalAdd(quarterPi, atanSmallInterval(q))
}

func atan2Interval(y, x *big.Rat, negativeZeroY bool) ratInterval {
	zero := new(big.Rat)
	if x.Sign() == 0 {
		halfPi := intervalScale(interval(piLower, piUpper), big.NewRat(1, 2))
		if y.Sign() < 0 {
			return intervalNeg(halfPi)
		}
		return halfPi
	}
	if y.Sign() == 0 {
		if x.Sign() < 0 {
			if negativeZeroY {
				return intervalNeg(interval(piLower, piUpper))
			}
			return interval(piLower, piUpper)
		}
		return pointInterval(zero)
	}
	ax, ay := new(big.Rat).Abs(x), new(big.Rat).Abs(y)
	var base ratInterval
	if ay.Cmp(ax) <= 0 {
		base = atanPositiveInterval(new(big.Rat).Quo(ay, ax))
	} else {
		halfPi := intervalScale(interval(piLower, piUpper), big.NewRat(1, 2))
		base = intervalSub(halfPi, atanPositiveInterval(new(big.Rat).Quo(ax, ay)))
	}
	switch {
	case x.Sign() > 0 && y.Sign() > 0:
		return base
	case x.Sign() > 0 && y.Sign() < 0:
		return intervalNeg(base)
	case x.Sign() < 0 && y.Sign() > 0:
		return intervalSub(interval(piLower, piUpper), base)
	default:
		return intervalAdd(intervalNeg(interval(piLower, piUpper)), base)
	}
}

func exactCoordinateDelta(a, b float64) *big.Rat {
	return new(big.Rat).Sub(floatRat(a), floatRat(b))
}

func circularAreaInterval(seg CurveSegment) (ratInterval, bool) {
	switch seg := seg.(type) {
	case CircleSeg:
		dt := exactCoordinateDelta(seg.TEnd, seg.TStart)
		if !dt.IsInt() {
			return ratInterval{}, false
		}
		radius, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return ratInterval{}, false
		}
		r := floatRat(radius)
		scale := new(big.Rat).Mul(dt, new(big.Rat).Mul(r, r))
		// An integer number of turns has equal endpoint sine/cosine terms,
		// leaving exactly dt·π·r².
		return intervalScale(interval(piLower, piUpper), scale), true
	case ArcSeg:
		forward := seg.TStart == 0 && seg.TEnd == 1
		reverse := seg.TStart == 1 && seg.TEnd == 0
		if !forward && !reverse {
			return ratInterval{}, false
		}
		dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
		dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
		dx1 := exactCoordinateDelta(seg.End.U, seg.Center.U)
		dy1 := exactCoordinateDelta(seg.End.V, seg.Center.V)
		r21 := new(big.Rat).Add(
			new(big.Rat).Mul(dx1, dx1),
			new(big.Rat).Mul(dy1, dy1),
		)
		r2 := new(big.Rat).Add(
			new(big.Rat).Mul(dx0, dx0),
			new(big.Rat).Mul(dy0, dy0),
		)
		if r2.Cmp(r21) != 0 {
			return ratInterval{}, false
		}
		heldDY0 := seg.Start.V - seg.Center.V
		heldDY1 := seg.End.V - seg.Center.V
		a0 := atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
		a1 := atan2Interval(dy1, dx1, heldDY1 == 0 && math.Signbit(heldDY1))
		sweep := intervalSub(a1, a0)
		heldA0 := math.Atan2(seg.Start.V-seg.Center.V, seg.Start.U-seg.Center.U)
		heldA1 := math.Atan2(seg.End.V-seg.Center.V, seg.End.U-seg.Center.U)
		if heldA1-heldA0 <= 0 {
			sweep = intervalAdd(sweep, intervalScale(interval(piLower, piUpper), big.NewRat(2, 1)))
		}
		sign := big.NewRat(1, 1)
		dx, dy := new(big.Rat).Sub(dx1, dx0), new(big.Rat).Sub(dy1, dy0)
		if reverse {
			sign.Neg(sign)
			dx.Neg(dx)
			dy.Neg(dy)
		}
		sector := intervalScale(sweep, new(big.Rat).Mul(sign, r2))
		centerTerm := new(big.Rat).Sub(
			new(big.Rat).Mul(floatRat(seg.Center.U), dy),
			new(big.Rat).Mul(floatRat(seg.Center.V), dx),
		)
		return intervalScale(
			intervalAdd(sector, pointInterval(centerTerm)),
			big.NewRat(1, 2),
		), true
	default:
		return ratInterval{}, false
	}
}

func accumulateMoment(value, bound *float64, term, termBound float64) {
	next := *value + term
	*bound = absSumUpper(*bound, termBound, addRoundError(*value, term, next))
	*value = next
}

type momentIntegralOrder uint8

const (
	momentAreaOrder momentIntegralOrder = iota
	momentFirstOrder
	momentSecondOrder
)

func finiteMomentValues(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

func (ig regionIntegrals) isFinite(order momentIntegralOrder) bool {
	switch order {
	case momentAreaOrder:
		return finiteMomentValues(ig.area)
	case momentFirstOrder:
		return finiteMomentValues(ig.area, ig.mu, ig.mv)
	default:
		return finiteMomentValues(ig.area, ig.mu, ig.mv, ig.muu, ig.muv, ig.mvv)
	}
}

// integrals walks the outer loop and every hole in recorded walk order and
// sums each segment's closed-form contribution. Walk order carries the sign:
// the outer loop is counter-clockwise (positive), holes are clockwise
// (negative), so the sum IS the net region integral.
func (r ProfileRecord) integrals() (regionIntegrals, error) {
	return r.integralsBudget(nil)
}

func (r ProfileRecord) integralsBudget(budget *workBudget) (regionIntegrals, error) {
	record, anchor, err := validateMomentFieldsBudget(budget, r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecordBudget(record, anchor, momentSecondOrder, budget)
}

func (r ProfileRecord) integralsTo(order momentIntegralOrder) (regionIntegrals, error) {
	record, anchor, err := validateMomentRecord(r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecord(record, anchor, order)
}

// evaluatorIntegrals supplies only the mass properties an evaluator needs.
// Public measurement methods use integralsTo and retain full topology and
// finiteness checks; evaluator construction must not let unused higher-order
// overflow prevent clearance verification from running.
func (r ProfileRecord) evaluatorIntegrals(order momentIntegralOrder) (regionIntegrals, error) {
	record, anchor, err := validateMomentFields(r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecord(record, anchor, order)
}

func (r ProfileRecord) evaluatorIntegralsUnchecked(order momentIntegralOrder) (regionIntegrals, error) {
	record, anchor, err := validateMomentFields(r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecordUnchecked(record, anchor, order)
}

func integrateMomentRecord(record ProfileRecord, anchor Point2, order momentIntegralOrder) (regionIntegrals, error) {
	return integrateMomentRecordBudget(record, anchor, order, nil)
}

func integrateMomentRecordBudget(record ProfileRecord, anchor Point2, order momentIntegralOrder, budget *workBudget) (regionIntegrals, error) {
	return integrateMomentRecordMode(record, anchor, order, true, budget)
}

func integrateMomentRecordUnchecked(record ProfileRecord, anchor Point2, order momentIntegralOrder) (regionIntegrals, error) {
	return integrateMomentRecordMode(record, anchor, order, false, nil)
}

func integrateMomentRecordMode(record ProfileRecord, anchor Point2, order momentIntegralOrder, checkFinite bool, budget *workBudget) (regionIntegrals, error) {
	var ig regionIntegrals
	for loopIndex, loop := range append([]LoopRecord{record.Outer}, record.Holes...) {
		if err := wallBudgetStep(budget); err != nil {
			return regionIntegrals{}, err
		}
		for segmentIndex, segment := range loop.Segments {
			if err := wallBudgetStep(budget); err != nil {
				return regionIntegrals{}, err
			}
			if err := ig.add(segment); err != nil {
				return regionIntegrals{}, err
			}
			if checkFinite && !finiteMomentValues(ig.area, ig.mu, ig.mv, ig.muu, ig.muv, ig.mvv) {
				return regionIntegrals{}, fmt.Errorf(`%w: mass-property integration overflowed at loop %d segment %d`, ErrNotFinite, loopIndex, segmentIndex)
			}
		}
	}
	if ig.area <= 0 {
		return regionIntegrals{}, fmt.Errorf(`%w: the recorded region encloses no positive net area`, ErrDegenerate)
	}
	if checkFinite && !finiteMomentValues(ig.area, ig.mu, ig.mv, ig.muu, ig.muv, ig.mvv) {
		return regionIntegrals{}, fmt.Errorf(`%w: mass-property integration overflowed while restoring the profile origin`, ErrNotFinite)
	}
	return ig, nil
}

func shiftMomentSegment(segment CurveSegment, anchor Point2) (CurveSegment, error) {
	segment, err := normalizeSegment(segment)
	if err != nil {
		return nil, err
	}
	shift := func(point Point2) Point2 {
		return Point2{U: point.U - anchor.U, V: point.V - anchor.V}
	}
	switch segment := segment.(type) {
	case LineSeg:
		segment.Start = shift(segment.Start)
		segment.End = shift(segment.End)
		return segment, nil
	case CircleSeg:
		segment.Center = shift(segment.Center)
		return segment, nil
	case ArcSeg:
		segment.Center = shift(segment.Center)
		segment.Start = shift(segment.Start)
		segment.End = shift(segment.End)
		return segment, nil
	default:
		return nil, fmt.Errorf(`%w: this evaluator computes mass properties over line, arc and circle profile segments only; the profile has a %T segment`, ErrUnsupported, segment)
	}
}

func translateMomentIntegrals(ig regionIntegrals, anchor Point2, order momentIntegralOrder) regionIntegrals {
	if order == momentAreaOrder {
		return ig
	}
	mu, mv := ig.mu, ig.mv
	if order == momentSecondOrder {
		ig.muu += 2*anchor.U*mu + anchor.U*anchor.U*ig.area
		ig.muv += anchor.V*mu + anchor.U*mv + anchor.U*anchor.V*ig.area
		ig.mvv += 2*anchor.V*mv + anchor.V*anchor.V*ig.area
	}
	ig.mu += anchor.U * ig.area
	ig.mv += anchor.V * ig.area
	return ig
}

// add accumulates one segment's boundary-integral contribution, in the
// segment's recorded walk direction. Circular contributions carry an outward
// evaluation bound.
func (ig *regionIntegrals) add(segment CurveSegment) error {
	segment, err := normalizeSegment(segment)
	if err != nil {
		return err
	}
	switch segment := segment.(type) {
	case LineSeg:
		ig.addLine(segment)
		return nil
	case CircleSeg:
		if segment.Radius.Kind() != units.Length {
			return fmt.Errorf(`%w: a circle segment's radius must be a %s, got %s`, ErrUnitKind, units.Length, segment.Radius.Kind())
		}
		radius, err := segment.Radius.In(units.Millimeter)
		if err != nil {
			return fmt.Errorf(`%w: a circle segment's radius is not representable: %s`, ErrNotFinite, err)
		}
		if segment.CCW != (segment.TStart < segment.TEnd) {
			return fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, ErrDegenerate)
		}
		areaProof, haveAreaProof := circularAreaInterval(segment)
		// The arrangement's normalized t is the angle 2π·t from +u
		// (geom.BoundaryEdge); the recorded range order is the walk.
		ig.addCircular(
			segment.Center,
			radius,
			2*math.Pi*segment.TStart,
			2*math.Pi*segment.TEnd,
			math.Abs(radius),
			circularSweepUpper(segment.TStart, segment.TEnd),
			areaProof,
			haveAreaProof,
		)
		return nil
	case ArcSeg:
		radius := math.Hypot(segment.Start.U-segment.Center.U, segment.Start.V-segment.Center.V)
		a0 := math.Atan2(segment.Start.V-segment.Center.V, segment.Start.U-segment.Center.U)
		a1 := math.Atan2(segment.End.V-segment.Center.V, segment.End.U-segment.Center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		areaProof, haveAreaProof := circularAreaInterval(segment)
		// normalized t maps to angle = a0 + t·sweep; the range order is the walk.
		ig.addCircular(
			segment.Center,
			radius,
			a0+segment.TStart*sweep,
			a0+segment.TEnd*sweep,
			arcRadiusUpper(segment),
			circularSweepUpper(segment.TStart, segment.TEnd),
			areaProof,
			haveAreaProof,
		)
		return nil
	default:
		return fmt.Errorf(`%w: this evaluator computes mass properties over line, arc and circle profile segments only; the profile has a %T segment`, ErrUnsupported, segment)
	}
}

// addFor preserves the evaluator helper's order-aware call shape. The bounded
// implementation computes all moments together, so the requested order does
// not change the accumulated result.
func (ig *regionIntegrals) addFor(segment CurveSegment, _ momentIntegralOrder) error {
	return ig.add(segment)
}

// addLine accumulates the straight chord from the walk's start point to its
// end point. The recorded range picks the walked piece of the entity's own
// Start→End parameterization.
func (ig *regionIntegrals) addLine(seg LineSeg) {
	u0, v0 := lerp2(seg.Start, seg.End, seg.TStart)
	u1, v1 := lerp2(seg.Start, seg.End, seg.TEnd)
	exact := exactLineMoments(seg)
	_, _, coordUpper := lineWalkBounds(seg, math.Hypot(u1-u0, v1-v0))
	ig.coordUpper = math.Max(ig.coordUpper, coordUpper)

	// A     = ½ ∮ (u dv − v du)
	// ∫u dA = ½ ∮ u² dv
	// ∫v dA = −½ ∮ v² du
	area := 0.5 * (u0*v1 - u1*v0)
	mu := (v1 - v0) * (u0*u0 + u0*u1 + u1*u1) / 6
	mv := -(u1 - u0) * (v0*v0 + v0*v1 + v1*v1) / 6

	// ∫u² dA = ⅓ ∮ u³ dv;  ∫v² dA = −⅓ ∮ v³ du — the cubic sums are the
	// exact ∫₀¹ of the lerp cubed.
	muu := (v1 - v0) * (u0*u0*u0 + u0*u0*u1 + u0*u1*u1 + u1*u1*u1) / 12
	mvv := -(u1 - u0) * (v0*v0*v0 + v0*v0*v1 + v0*v1*v1 + v1*v1*v1) / 12

	du, dv := u1-u0, v1-v0
	intU2V := v0*(u0*u0+u0*du+du*du/3) + dv*(u0*u0/2+2*u0*du/3+du*du/4)
	muv := 0.5 * dv * intU2V

	accumulateMoment(&ig.area, &ig.areaBound, area, rationalFloatError(exact.area, area))
	accumulateMoment(&ig.mu, &ig.muBound, mu, rationalFloatError(exact.mu, mu))
	accumulateMoment(&ig.mv, &ig.mvBound, mv, rationalFloatError(exact.mv, mv))
	accumulateMoment(&ig.muu, &ig.muuBound, muu, rationalFloatError(exact.muu, muu))
	accumulateMoment(&ig.muv, &ig.muvBound, muv, rationalFloatError(exact.muv, muv))
	accumulateMoment(&ig.mvv, &ig.mvvBound, mvv, rationalFloatError(exact.mvv, mvv))
}

type exactMoments struct {
	area *big.Rat
	mu   *big.Rat
	mv   *big.Rat
	muu  *big.Rat
	muv  *big.Rat
	mvv  *big.Rat
}

func ratAdd(values ...*big.Rat) *big.Rat {
	out := new(big.Rat)
	for _, value := range values {
		out.Add(out, value)
	}
	return out
}

func ratMul(values ...*big.Rat) *big.Rat {
	out := big.NewRat(1, 1)
	for _, value := range values {
		out.Mul(out, value)
	}
	return out
}

func ratScale(value *big.Rat, num, den int64) *big.Rat {
	return new(big.Rat).Mul(value, big.NewRat(num, den))
}

func ratLerp(start, end, t float64) *big.Rat {
	rs, re, rt := floatRat(start), floatRat(end), floatRat(t)
	if rs == nil || re == nil || rt == nil {
		return nil
	}
	return new(big.Rat).Add(rs, new(big.Rat).Mul(rt, new(big.Rat).Sub(re, rs)))
}

// exactLineMoments evaluates the polynomial line formulas over exact rationals.
// The public values retain the existing float evaluation; the rational result
// proves whether its rounding is exact and, when it is not, the precise error.
func exactLineMoments(seg LineSeg) exactMoments {
	u0 := ratLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0 := ratLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1 := ratLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1 := ratLerp(seg.Start.V, seg.End.V, seg.TEnd)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil {
		return exactMoments{}
	}
	du := new(big.Rat).Sub(u1, u0)
	dv := new(big.Rat).Sub(v1, v0)

	u0sq, u1sq := ratMul(u0, u0), ratMul(u1, u1)
	v0sq, v1sq := ratMul(v0, v0), ratMul(v1, v1)
	area := ratScale(new(big.Rat).Sub(ratMul(u0, v1), ratMul(u1, v0)), 1, 2)
	mu := ratScale(ratMul(dv, ratAdd(u0sq, ratMul(u0, u1), u1sq)), 1, 6)
	mv := ratScale(ratMul(du, ratAdd(v0sq, ratMul(v0, v1), v1sq)), -1, 6)

	muu := ratScale(ratMul(dv, ratAdd(
		ratMul(u0, u0, u0),
		ratMul(u0, u0, u1),
		ratMul(u0, u1, u1),
		ratMul(u1, u1, u1),
	)), 1, 12)
	mvv := ratScale(ratMul(du, ratAdd(
		ratMul(v0, v0, v0),
		ratMul(v0, v0, v1),
		ratMul(v0, v1, v1),
		ratMul(v1, v1, v1),
	)), -1, 12)

	duSq := ratMul(du, du)
	u2v0 := ratMul(v0, ratAdd(u0sq, ratMul(u0, du), ratScale(duSq, 1, 3)))
	u2dv := ratMul(dv, ratAdd(
		ratScale(u0sq, 1, 2),
		ratScale(ratMul(u0, du), 2, 3),
		ratScale(duSq, 1, 4),
	))
	muv := ratScale(ratMul(dv, ratAdd(u2v0, u2dv)), 1, 2)
	return exactMoments{area: area, mu: mu, mv: mv, muu: muu, muv: muv, mvv: mvv}
}

// addCircular accumulates a circular path about center c with radius r, from
// angle th0 to th1 in the walk direction (th1 < th0 walks clockwise). The
// antiderivatives use the same formula for a whole period, a fragment, and a
// reversed walk; numerical evaluation carries an outward error bound.
func (ig *regionIntegrals) addCircular(
	c Point2,
	r, th0, th1, radiusUpper, sweepUpper float64,
	areaProof ratInterval,
	haveAreaProof bool,
) {
	sin0, cos0 := math.Sincos(th0)
	sin1, cos1 := math.Sincos(th1)
	dth := th1 - th0
	absR, absU, absV, absDth := radiusUpper, math.Abs(c.U), math.Abs(c.V), sweepUpper
	ig.coordUpper = math.Max(ig.coordUpper, absSumUpper(c.U, c.V, absR, absR))

	// A = ½ ∫ (u v′ − v u′) dθ = ½ [r²·θ + c_u·r·sin θ + c_v·r·cos θ]
	area := 0.5 * (r*r*dth + c.U*r*(sin1-sin0) - c.V*r*(cos1-cos0))
	areaScale := 0.5 * (absR*absR*absDth + 2*absU*absR + 2*absV*absR)

	intCos := sin1 - sin0
	intCos2 := dth/2 + (math.Sin(2*th1)-math.Sin(2*th0))/4
	intCos3 := (sin1 - sin1*sin1*sin1/3) - (sin0 - sin0*sin0*sin0/3)
	mu := 0.5 * r * (c.U*c.U*intCos + 2*c.U*r*intCos2 + r*r*intCos3)

	intSin := cos0 - cos1
	intSin2 := dth/2 - (math.Sin(2*th1)-math.Sin(2*th0))/4
	intSin3 := (cos0 - cos0*cos0*cos0/3) - (cos1 - cos1*cos1*cos1/3)
	mv := 0.5 * r * (c.V*c.V*intSin + 2*c.V*r*intSin2 + r*r*intSin3)

	int2Scale := absDth/2 + 0.5
	int3Scale := 4.0 / 3
	muScale := 0.5 * absR * (2*absU*absU + 2*absU*absR*int2Scale + absR*absR*int3Scale)
	mvScale := 0.5 * absR * (2*absV*absV + 2*absV*absR*int2Scale + absR*absR*int3Scale)

	intCos4 := 3*dth/8 + (math.Sin(2*th1)-math.Sin(2*th0))/4 + (math.Sin(4*th1)-math.Sin(4*th0))/32
	intSin4 := 3*dth/8 - (math.Sin(2*th1)-math.Sin(2*th0))/4 + (math.Sin(4*th1)-math.Sin(4*th0))/32
	int4Scale := 3*absDth/8 + 0.5 + 1.0/16

	// ∫u² dA = ⅓ ∮ u³ dv, dv = r cos θ dθ, u³ expanded about the center.
	muu := r / 3 * (c.U*c.U*c.U*intCos + 3*c.U*c.U*r*intCos2 + 3*c.U*r*r*intCos3 + r*r*r*intCos4)
	muuScale := absR / 3 * (2*absU*absU*absU + 3*absU*absU*absR*int2Scale +
		3*absU*absR*absR*int3Scale + absR*absR*absR*int4Scale)

	// ∫v² dA = −⅓ ∮ v³ du, du = −r sin θ dθ, v³ expanded about the center.
	mvv := r / 3 * (c.V*c.V*c.V*intSin + 3*c.V*c.V*r*intSin2 + 3*c.V*r*r*intSin3 + r*r*r*intSin4)
	mvvScale := absR / 3 * (2*absV*absV*absV + 3*absV*absV*absR*int2Scale +
		3*absV*absR*absR*int3Scale + absR*absR*absR*int4Scale)

	intSC := (sin1*sin1 - sin0*sin0) / 2
	intSC2 := (cos0*cos0*cos0 - cos1*cos1*cos1) / 3
	intSC3 := (cos0*cos0*cos0*cos0 - cos1*cos1*cos1*cos1) / 4
	muv := 0.5 * r * (c.V*(c.U*c.U*intCos+2*c.U*r*intCos2+r*r*intCos3) +
		r*(c.U*c.U*intSC+2*c.U*r*intSC2+r*r*intSC3))
	muvScale := 0.5 * absR * (absV*(2*absU*absU+2*absU*absR*int2Scale+absR*absR*int3Scale) +
		absR*(0.5*absU*absU+4*absU*absR/3+absR*absR/4))
	areaScale = productUpper(2, areaScale)
	muScale = productUpper(2, muScale)
	mvScale = productUpper(2, mvScale)
	muuScale = productUpper(2, muuScale)
	muvScale = productUpper(2, muvScale)
	mvvScale = productUpper(2, mvvScale)

	areaBound := conservativeValueError(area, areaScale)
	if haveAreaProof {
		areaBound = math.Min(areaBound, intervalFloatError(areaProof, area))
	}
	accumulateMoment(&ig.area, &ig.areaBound, area, areaBound)
	accumulateMoment(&ig.mu, &ig.muBound, mu, conservativeValueError(mu, muScale))
	accumulateMoment(&ig.mv, &ig.mvBound, mv, conservativeValueError(mv, mvScale))
	accumulateMoment(&ig.muu, &ig.muuBound, muu, conservativeValueError(muu, muuScale))
	accumulateMoment(&ig.muv, &ig.muvBound, muv, conservativeValueError(muv, muvScale))
	accumulateMoment(&ig.mvv, &ig.mvvBound, mvv, conservativeValueError(mvv, mvvScale))
}

// lerp2 returns the point at parameter t on the segment start→end.
func lerp2(start, end Point2, t float64) (float64, float64) {
	return start.U + t*(end.U-start.U), start.V + t*(end.V-start.V)
}
