package decad

import (
	"context"
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
// segment kind. Line and Tier A free-form walks (docs/spline-design.md Table F)
// integrate to exact rationals, so a region built only from them is published
// as its own rational rounded ONCE and retains a zero bound wherever that
// rational is representable; circular evaluations have no exact rational and
// carry outward bounds instead. Every other free-form kind is unsupported.

// Area returns the recorded region's net area — the outer loop minus its
// holes — as a [Measurement] of Kind Area (mm²): a computed quantity carries
// its Exactness and Bound (docs/api-design.md §6). Each boundary segment
// contributes its Green's-theorem integral in walk order, so a hole's clockwise
// walk subtracts without a special case.
//
// Line and Tier A free-form contributions — a spline, a closed spline, and a
// NURBS whose weights are all equal (docs/spline-design.md Table F) — integrate
// to exact rationals. A region built only from them is reported as the whole
// region's rational rounded ONCE, so its bound is that single rounding: zero,
// hence Exact, exactly when the rational is representable in float64, and never
// unconditionally. A circular contribution has no exact rational and carries a
// proven float evaluation bound, which the whole region then inherits.
//
// A region whose exact area is strictly positive is measured even where no
// float64 holds it: a section scaled far enough down reports the zero its
// rational rounds to, with that rounding as the bound — Approximate, never a
// refusal. A region whose area is genuinely zero or negative is [ErrDegenerate].
//
// The remaining free-form kinds are [ErrUnsupported] (docs/spline-design.md
// Table R) — never approximated: an ellipse, a conic and a rational NURBS have
// no exact rational moments yet, an elliptical arc's record is self-inconsistent,
// and a fit spline's curve is sketch's own interpolation solve. So is a Tier A
// boundary whose exact integration would exceed this evaluator's fixed work
// budget (Table R row R7). A malformed or open record is [ErrDegenerate].
// A circle radius of the wrong kind is [ErrUnitKind], a negative radius is
// [ErrNegativeMagnitude], and a non-finite field or arithmetic result is
// [ErrNotFinite]. No measurement is returned on error.
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
// A region whose exact area and first moments are all rational — a boundary of
// line and Tier A free-form walks only — has its centroid taken over those
// rationals and each coordinate rounded ONCE, so the reported bound is that
// single rounding: zero, hence Exact, exactly when the quotient is
// representable. The region's exact area is already proven strictly positive
// there (see requirePositiveArea), so the quotient exists however small the
// area's own float image is — a section scaled far enough down for its area to
// underflow still reports its centroid.
//
// Only where some contribution has NO exact rational — a circular walk, whose
// integral carries π — is the centroid divided in bounded floats. A region whose
// net area is zero has no centroid and is [ErrDegenerate], and one whose net
// area that float division cannot prove stays away from zero is
// [ErrUnsupported]: the division has no bounded result. Record validation and
// arithmetic errors match [ProfileRecord.Area].
func (r ProfileRecord) Centroid() (VecMeasurement, error) {
	ig, err := r.integralsTo(momentFirstOrder)
	if err != nil {
		return VecMeasurement{}, err
	}
	if exact, ok := ig.exactCentroid(); ok {
		return exact, nil
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

// exactCentroid divides the region's exact first moments by its exact area over
// rationals and rounds each quotient ONCE, which is the same single-rounding
// rule publishExact applies to the moments themselves (docs/spline-design.md
// §3). It reports whether the region has such an accumulator at all.
//
// It is what keeps the answer from being refused when it is already in hand.
// requirePositiveArea has PROVEN the exact area strictly positive before this
// runs, so the centroid exists; the float guards below it read the published
// area, whose float image can be zero for a region whose exact area underflows,
// and would report ErrUnsupported for a quotient that is perfectly
// representable. The float path stays for the regions this one cannot serve:
// those whose accumulator a circular contribution retired.
func (ig regionIntegrals) exactCentroid() (VecMeasurement, bool) {
	if ig.exactDead || !ig.exact.complete() || ig.exact.area.Sign() <= 0 {
		return VecMeasurement{}, false
	}
	u := new(big.Rat).Quo(ig.exact.mu, ig.exact.area)
	v := new(big.Rat).Quo(ig.exact.mv, ig.exact.area)
	uHeld, _ := u.Float64()
	vHeld, _ := v.Float64()
	if isNonFinite(uHeld) || isNonFinite(vHeld) {
		// No float64 holds this centroid, so there is no single rounding to
		// publish; the bounded path answers, or refuses, on its own terms.
		return VecMeasurement{}, false
	}
	bound := radius2D(rationalFloatError(u, uHeld), rationalFloatError(v, vHeld))
	return VecMeasurement{
		Value:     r3.NewVec(uHeld, vHeld, 0),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, true
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
// plane origin. The staging matches [ProfileRecord.Area]: a Tier A free-form
// boundary is integrated exactly and rounded once, every other free-form kind
// is [ErrUnsupported], and malformed or non-finite records are rejected before
// a measurement is constructed.
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

	// exact is the WHOLE region's moments as exact rationals — every line and
	// Tier A free-form contribution added into it, and the anchor
	// re-referencing applied over rationals too. It stays alive only while
	// every contribution so far has had an exact rational, and when it does the
	// published float is that region-level rational rounded ONCE
	// (docs/spline-design.md §3/§5.2). Rounding per segment instead would make
	// the held float a sum of roundings, so a multi-segment region would miss
	// the single-rounding property a one-segment region has.
	exact exactMoments
	// exactDead records that some contribution had no exact rational — a
	// circular walk's integral carries π and trig terms — so the region's own
	// sum is not exact and the per-segment float accumulation with its own
	// proven bounds is what gets published.
	exactDead bool
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

// circularAreaInterval brackets one circular walk's exact area contribution
// about the walk anchor. The segment holds the RECORDED coordinates and the
// anchor is subtracted here over rationals: every radial term is a difference
// the shift cancels out of exactly, and only the centre term carries it, so
// this bracket stays a proof about the recorded arc rather than about a
// float-shifted copy of it.
func circularAreaInterval(seg CurveSegment, anchor Point2) (ratInterval, bool) {
	anchorU, anchorV := floatRat(anchor.U), floatRat(anchor.V)
	if anchorU == nil || anchorV == nil {
		return ratInterval{}, false
	}
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
		// Arc endpoints may retain solver drift within the accepted radius join
		// tolerance. The integrated path uses the start radius and endpoint angles,
		// so the area interval charges the radial mismatch before it is published.
		r2 := new(big.Rat).Add(
			new(big.Rat).Mul(dx0, dx0),
			new(big.Rat).Mul(dy0, dy0),
		)
		endR2 := new(big.Rat).Add(
			new(big.Rat).Mul(dx1, dx1),
			new(big.Rat).Mul(dy1, dy1),
		)
		// The held angles read the same float-shifted coordinates the float
		// evaluation does, so this bracket brackets THAT walk's sweep branch.
		heldCenter := shiftPoint(seg.Center, anchor)
		heldStart := shiftPoint(seg.Start, anchor)
		heldEnd := shiftPoint(seg.End, anchor)
		heldDY0 := heldStart.V - heldCenter.V
		heldDY1 := heldEnd.V - heldCenter.V
		a0 := atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
		a1 := atan2Interval(dy1, dx1, heldDY1 == 0 && math.Signbit(heldDY1))
		sweep := intervalSub(a1, a0)
		heldA0 := math.Atan2(heldStart.V-heldCenter.V, heldStart.U-heldCenter.U)
		heldA1 := math.Atan2(heldEnd.V-heldCenter.V, heldEnd.U-heldCenter.U)
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
		// Only the centre carries the anchor: every radial term above is a
		// difference the shift cancels out of.
		centerU := new(big.Rat).Sub(floatRat(seg.Center.U), anchorU)
		centerV := new(big.Rat).Sub(floatRat(seg.Center.V), anchorV)
		centerTerm := new(big.Rat).Sub(
			new(big.Rat).Mul(centerU, dy),
			new(big.Rat).Mul(centerV, dx),
		)
		areaProof := intervalAdd(sector, pointInterval(centerTerm))
		if endR2.Cmp(r2) != 0 {
			if endR2.Sign() == 0 {
				return ratInterval{}, false
			}
			radialGap := new(big.Rat).Sub(endR2, r2)
			radialGap.Abs(radialGap)
			radialRatioUpper := new(big.Rat).Quo(radialGap, endR2)
			endpointScale := new(big.Rat).Add(
				new(big.Rat).Mul(
					new(big.Rat).Abs(centerU),
					new(big.Rat).Abs(dy1),
				),
				new(big.Rat).Mul(
					new(big.Rat).Abs(centerV),
					new(big.Rat).Abs(dx1),
				),
			)
			correction := new(big.Rat).Mul(radialRatioUpper, endpointScale)
			correctionFloat, exact := correction.Float64()
			if !exact {
				correctionFloat = math.Nextafter(correctionFloat, math.Inf(1))
			}
			correctionFloat = absSumUpper(
				correctionFloat,
				analyticRoundBound(absSumUpper(
					heldCenter.U,
					heldCenter.V,
					heldEnd.U-heldCenter.U,
					heldEnd.V-heldCenter.V,
				)),
			)
			correction = floatRat(correctionFloat)
			areaProof = intervalAdd(
				areaProof,
				intervalScale(interval(big.NewRat(-1, 1), big.NewRat(1, 1)), correction),
			)
		}
		return intervalScale(
			areaProof,
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

func (r ProfileRecord) integralsBudget(budget *workBudget) (regionIntegrals, error) {
	if err := wallBudgetErr(budget); err != nil {
		return regionIntegrals{}, err
	}
	pre, err := validateMomentFieldsBudget(budget, r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecordBudget(pre, momentSecondOrder, budget)
}

func (r ProfileRecord) integralsTo(order momentIntegralOrder) (regionIntegrals, error) {
	pre, err := validateMomentRecord(r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecord(pre, order)
}

// evaluatorIntegrals supplies only the mass properties an evaluator needs.
// Public measurement methods use integralsTo and retain full topology and
// finiteness checks; evaluator construction must not let unused higher-order
// overflow prevent clearance verification from running.
func (r ProfileRecord) evaluatorIntegrals(order momentIntegralOrder) (regionIntegrals, error) {
	pre, err := validateMomentFields(r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecord(pre, order)
}

func (r ProfileRecord) evaluatorIntegralsContext(ctx context.Context, order momentIntegralOrder) (regionIntegrals, error) {
	pre, err := validateMomentFieldsContext(ctx, r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecordModeContext(ctx, pre, order, true)
}

func (r ProfileRecord) evaluatorIntegralsUncheckedContext(ctx context.Context, order momentIntegralOrder) (regionIntegrals, error) {
	pre, err := validateMomentFieldsContext(ctx, r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecordUncheckedContext(ctx, pre, order)
}

func integrateMomentRecord(pre momentPreflight, order momentIntegralOrder) (regionIntegrals, error) {
	return integrateMomentRecordBudget(pre, order, nil)
}

func integrateMomentRecordBudget(pre momentPreflight, order momentIntegralOrder, budget *workBudget) (regionIntegrals, error) {
	return integrateMomentRecordMode(pre, order, true, budget)
}

func integrateMomentRecordMode(pre momentPreflight, order momentIntegralOrder, checkFinite bool, budget *workBudget) (regionIntegrals, error) {
	return integrateMomentRecordWithPoll(func() error { return wallBudgetStep(budget) }, pre, order, checkFinite)
}

func integrateMomentRecordUncheckedContext(ctx context.Context, pre momentPreflight, order momentIntegralOrder) (regionIntegrals, error) {
	return integrateMomentRecordModeContext(ctx, pre, order, false)
}

func integrateMomentRecordModeContext(ctx context.Context, pre momentPreflight, order momentIntegralOrder, checkFinite bool) (regionIntegrals, error) {
	return integrateMomentRecordWithPoll(ctx.Err, pre, order, checkFinite)
}

func integrateMomentRecordWithPoll(poll func() error, pre momentPreflight, order momentIntegralOrder, checkFinite bool) (regionIntegrals, error) {
	var ig regionIntegrals
	for loopIndex, loop := range append([]LoopRecord{pre.record.Outer}, pre.record.Holes...) {
		if poll != nil {
			if err := poll(); err != nil {
				return regionIntegrals{}, err
			}
		}
		for segmentIndex, segment := range loop.Segments {
			if poll != nil {
				if err := poll(); err != nil {
					return regionIntegrals{}, err
				}
			}
			if err := ig.addFor(segment, pre.planAt(loopIndex, segmentIndex), pre.anchor, order); err != nil {
				return regionIntegrals{}, err
			}
			if checkFinite && !ig.isFinite(order) {
				return regionIntegrals{}, fmt.Errorf(`%w: mass-property integration overflowed at loop %d segment %d`, ErrNotFinite, loopIndex, segmentIndex)
			}
		}
	}
	ig = translateMomentIntegrals(ig, pre.anchor, order)
	ig.publishExact()
	if err := ig.requirePositiveArea(); err != nil {
		return regionIntegrals{}, err
	}
	if checkFinite && !ig.isFinite(order) {
		return regionIntegrals{}, fmt.Errorf(`%w: mass-property integration overflowed while restoring the profile origin`, ErrNotFinite)
	}
	return ig, nil
}

// requirePositiveArea refuses a region whose net area is not positive.
//
// The region's own EXACT rational decides wherever there is one, because a
// strictly positive area can have a float64 image of zero: a valid section
// scaled far enough down has an exact area of s²·A > 0 that underflows, and a
// gate reading the float accumulator would refuse the region rather than publish
// the bounded zero the accumulator already holds — value 0 with the rational
// rounded up as its bound, hence Approximate, which is the honest reading of a
// positive area no float64 can hold. Every exactly integrated boundary is
// covered: the line path's closed forms and the Tier A free-form chains alike.
// Only where a contribution has no exact rational at all — a circular walk,
// whose integral carries π — does the float sum decide, as it always has.
func (ig *regionIntegrals) requirePositiveArea() error {
	if !ig.exactDead && ig.exact.complete() {
		if ig.exact.area.Sign() > 0 {
			return nil
		}
		return fmt.Errorf(`%w: the recorded region encloses no positive net area`, ErrDegenerate)
	}
	if ig.area <= 0 {
		return fmt.Errorf(`%w: the recorded region encloses no positive net area`, ErrDegenerate)
	}
	return nil
}

// shiftPoint re-references one recorded coordinate to the walk anchor in
// float64. It conditions the FLOAT evaluation only: every exact rational
// subtracts the anchor over rationals instead, because fl(p−anchor) rounds and
// an exact result taken over rounded coordinates is the exact answer for a
// different region (see regionIntegrals.add).
func shiftPoint(point, anchor Point2) Point2 {
	return Point2{U: point.U - anchor.U, V: point.V - anchor.V}
}

// shiftPoints translates a control-point slice into a fresh slice, leaving the
// caller's recorded segment untouched.
func shiftPoints(points []Point2, shift func(Point2) Point2) []Point2 {
	out := make([]Point2, len(points))
	for i, point := range points {
		out[i] = shift(point)
	}
	return out
}

func translateMomentIntegrals(ig regionIntegrals, anchor Point2, order momentIntegralOrder) regionIntegrals {
	if !ig.exactDead {
		ig.exact = translateExactMoments(ig.exact, anchor, order)
	}
	if order == momentAreaOrder {
		return ig
	}
	area := measuredScalar(ig.area, ig.areaBound)
	mu := measuredScalar(ig.mu, ig.muBound)
	mv := measuredScalar(ig.mv, ig.mvBound)
	if order == momentSecondOrder {
		two := exactScalar(2)
		anchorU := exactScalar(anchor.U)
		anchorV := exactScalar(anchor.V)
		muu := boundedAdd(
			measuredScalar(ig.muu, ig.muuBound),
			boundedAdd(
				boundedMul(boundedMul(two, anchorU), mu),
				boundedMul(boundedMul(anchorU, anchorU), area),
			),
		)
		muv := boundedAdd(
			measuredScalar(ig.muv, ig.muvBound),
			boundedAdd(
				boundedMul(anchorV, mu),
				boundedAdd(
					boundedMul(anchorU, mv),
					boundedMul(boundedMul(anchorU, anchorV), area),
				),
			),
		)
		mvv := boundedAdd(
			measuredScalar(ig.mvv, ig.mvvBound),
			boundedAdd(
				boundedMul(boundedMul(two, anchorV), mv),
				boundedMul(boundedMul(anchorV, anchorV), area),
			),
		)
		ig.muu, ig.muuBound = muu.value, muu.bound
		ig.muv, ig.muvBound = muv.value, muv.bound
		ig.mvv, ig.mvvBound = mvv.value, mvv.bound
	}
	mu = boundedAdd(mu, boundedMul(exactScalar(anchor.U), area))
	mv = boundedAdd(mv, boundedMul(exactScalar(anchor.V), area))
	ig.mu, ig.muBound = mu.value, mu.bound
	ig.mv, ig.mvBound = mv.value, mv.bound
	ig.coordUpper = math.Max(ig.coordUpper, absSumUpper(anchor.U, anchor.V))
	return ig
}

// add accumulates one segment's boundary-integral contribution, in the
// segment's recorded walk direction. Circular contributions carry an outward
// evaluation bound.
//
// Every contribution is re-referenced to the walk anchor, and the two
// arithmetics do it differently ON PURPOSE. The FLOAT evaluation reads
// anchor-shifted float coordinates, which is what keeps it conditioned. Every
// EXACT rational — a line's closed form, an arc's proven area interval, a
// converted free-form chain — subtracts the anchor over rationals instead,
// from the recorded coordinates themselves. Shifting in float first would round
// the geometry before the exact pipeline ever saw it, so the rational would be
// the exact answer for a region the caller did not record, and publishExact
// would round an already representable value and report Exact with a zero bound
// for it.
// A free-form segment arrives with the chain the record-level preflight already
// converted and charged (moments_validate.go), so this pass converts nothing and
// charges nothing.
func (ig *regionIntegrals) add(segment CurveSegment, plan freeformPlan, anchor Point2) error {
	segment, err := normalizeSegment(segment)
	if err != nil {
		return err
	}
	switch segment := segment.(type) {
	case LineSeg:
		ig.addLine(segment, anchor)
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
		areaProof, haveAreaProof := circularAreaInterval(segment, anchor)
		segment.Center = shiftPoint(segment.Center, anchor)
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
		areaProof, haveAreaProof := circularAreaInterval(segment, anchor)
		segment.Center = shiftPoint(segment.Center, anchor)
		segment.Start = shiftPoint(segment.Start, anchor)
		segment.End = shiftPoint(segment.End, anchor)
		radius := math.Hypot(segment.Start.U-segment.Center.U, segment.Start.V-segment.Center.V)
		a0 := math.Atan2(segment.Start.V-segment.Center.V, segment.Start.U-segment.Center.U)
		a1 := math.Atan2(segment.End.V-segment.Center.V, segment.End.U-segment.Center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
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
		// A free-form kind with no converted chain is one the preflight could
		// not convert, so this evaluator has no integral for it.
		if !isFreeformSegment(segment) || len(plan.spans) == 0 {
			return fmt.Errorf(`%w: this evaluator computes mass properties over line, arc, circle and Tier A free-form profile segments only; the profile has a %T segment`, ErrUnsupported, segment)
		}
		// The chain was converted from the RECORDED control points, so shifting
		// it here over rationals keeps the spans the recorded curve.
		if err := shiftFreeformSpans(plan.spans, anchor); err != nil {
			return err
		}
		ig.addFreeform(plan.spans, plan.reversed)
		return nil
	}
}

// addAnalytic accumulates one line, circle or arc segment about the given
// anchor. It is how the section audits take a loop's own signed area: their
// loops are proven walkable before any area is asked for — walkOf refuses every
// free-form kind — so no converted chain is involved and no work is charged.
func (ig *regionIntegrals) addAnalytic(segment CurveSegment, anchor Point2) error {
	return ig.add(segment, freeformPlan{}, anchor)
}

// addFor preserves the evaluator helper's order-aware call shape. The bounded
// implementation computes all moments together, so the requested order does
// not change the accumulated result.
func (ig *regionIntegrals) addFor(segment CurveSegment, plan freeformPlan, anchor Point2, _ momentIntegralOrder) error {
	return ig.add(segment, plan, anchor)
}

// addLine accumulates the straight chord from the walk's start point to its
// end point. The recorded range picks the walked piece of the entity's own
// Start→End parameterization.
func (ig *regionIntegrals) addLine(seg LineSeg, anchor Point2) {
	exact := exactLineMoments(seg, anchor)
	seg.Start = shiftPoint(seg.Start, anchor)
	seg.End = shiftPoint(seg.End, anchor)
	u0, v0 := lerp2(seg.Start, seg.End, seg.TStart)
	u1, v1 := lerp2(seg.Start, seg.End, seg.TEnd)
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
	ig.addExact(exact)
}

type exactMoments struct {
	area *big.Rat
	mu   *big.Rat
	mv   *big.Rat
	muu  *big.Rat
	muv  *big.Rat
	mvv  *big.Rat
}

func newExactMoments() exactMoments {
	return exactMoments{
		area: new(big.Rat),
		mu:   new(big.Rat),
		mv:   new(big.Rat),
		muu:  new(big.Rat),
		muv:  new(big.Rat),
		mvv:  new(big.Rat),
	}
}

// complete reports whether every moment has an exact rational. A contributor
// that could not build one leaves a nil field, which is the signal to retire
// the region-level accumulator rather than publish a partial sum.
func (m exactMoments) complete() bool {
	return m.area != nil && m.mu != nil && m.mv != nil &&
		m.muu != nil && m.muv != nil && m.mvv != nil
}

func (m exactMoments) fields() [6]*big.Rat {
	return [6]*big.Rat{m.area, m.mu, m.mv, m.muu, m.muv, m.mvv}
}

// heldFields pairs each accumulated float moment with its bound, in the same
// order exactMoments.fields uses, so a rational and its published float are
// never matched up by hand at a call site.
func (ig *regionIntegrals) heldFields() [6]struct{ value, bound *float64 } {
	return [6]struct{ value, bound *float64 }{
		{&ig.area, &ig.areaBound},
		{&ig.mu, &ig.muBound},
		{&ig.mv, &ig.mvBound},
		{&ig.muu, &ig.muuBound},
		{&ig.muv, &ig.muvBound},
		{&ig.mvv, &ig.mvvBound},
	}
}

// addExact folds one segment's exact contribution into the region-level
// rational accumulator.
func (ig *regionIntegrals) addExact(exact exactMoments) {
	if ig.exactDead {
		return
	}
	if !exact.complete() {
		ig.dropExact()
		return
	}
	if !ig.exact.complete() {
		ig.exact = newExactMoments()
	}
	running := ig.exact.fields()
	for i, term := range exact.fields() {
		running[i].Add(running[i], term)
	}
}

// dropExact retires the region-level rational accumulator for good: once one
// contribution has no exact rational, the region's own sum has none either.
func (ig *regionIntegrals) dropExact() {
	ig.exactDead = true
	ig.exact = exactMoments{}
}

// publishExact replaces the per-segment float accumulation with the region's
// own exact rational rounded ONCE, and the bound with that single rounding —
// zero, hence Exact, exactly when the rational is representable
// (docs/spline-design.md §3). It is a no-op once any contribution lacked an
// exact rational.
//
// Each field is published INDEPENDENTLY, and it must be: a field's value and
// bound are self-contained — the value is fl(exact) and the bound |exact −
// value| over rationals — so a rational no float64 can hold costs only its own
// field its single rounding. Publishing the six together instead abandons every
// one of them, so a second moment overflowing at coordinates near 1e78 mm denies
// Area the rational already in hand and leaves it the SUM of its per-segment
// roundings, past the half ulp §3 promises unconditionally. The mixed result is
// sound because every consumer reads each field through its own (value, bound)
// pair, and all cross-field composition — a revolve's axisMoments, the cup mass
// properties, Centroid's bounded-quotient fallback — is interval arithmetic,
// which asks only that each input interval encloses the truth.
func (ig *regionIntegrals) publishExact() {
	if ig.exactDead || !ig.exact.complete() {
		return
	}
	exact := ig.exact.fields()
	for i, field := range ig.heldFields() {
		held, _ := exact[i].Float64()
		if isNonFinite(held) {
			// No float64 holds this moment, so it has no single rounding to
			// publish; its own float accumulation and proven bound stand, and a
			// non-finite accumulation is refused by the order's finiteness check
			// rather than reported.
			continue
		}
		*field.value = held
		*field.bound = rationalFloatError(exact[i], held)
	}
}

// translateExactMoments re-references the exact accumulator from the walk
// anchor back to the profile origin, mirroring translateMomentIntegrals step
// for step but over rationals — the anchor coordinates are floats, hence exact
// rationals, and the shift is only sums and products, so nothing rounds here.
func translateExactMoments(exact exactMoments, anchor Point2, order momentIntegralOrder) exactMoments {
	if order == momentAreaOrder || !exact.complete() {
		return exact
	}
	anchorU, anchorV := floatRat(anchor.U), floatRat(anchor.V)
	if anchorU == nil || anchorV == nil {
		return exactMoments{}
	}
	if order == momentSecondOrder {
		// Second-order terms read the PRE-shift first moments, so they are
		// re-referenced before mu and mv are.
		exact.muu = ratAdd(
			exact.muu,
			ratMul(big.NewRat(2, 1), anchorU, exact.mu),
			ratMul(anchorU, anchorU, exact.area),
		)
		exact.muv = ratAdd(
			exact.muv,
			ratMul(anchorV, exact.mu),
			ratMul(anchorU, exact.mv),
			ratMul(anchorU, anchorV, exact.area),
		)
		exact.mvv = ratAdd(
			exact.mvv,
			ratMul(big.NewRat(2, 1), anchorV, exact.mv),
			ratMul(anchorV, anchorV, exact.area),
		)
	}
	exact.mu = ratAdd(exact.mu, ratMul(anchorU, exact.area))
	exact.mv = ratAdd(exact.mv, ratMul(anchorV, exact.area))
	return exact
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

// exactLineMoments evaluates the polynomial line formulas over exact rationals,
// about the walk anchor. The public values retain the existing float
// evaluation; the rational result proves whether its rounding is exact and,
// when it is not, the precise error.
//
// The segment handed in holds the RECORDED coordinates and the anchor is
// subtracted here, over rationals. A lerp is affine, so lerping then
// subtracting is identical to subtracting then lerping — but only in exact
// arithmetic: fl(p−anchor) rounds, and the rational taken over those rounded
// coordinates would be a different chord's exact area.
func exactLineMoments(seg LineSeg, anchor Point2) exactMoments {
	u0 := ratLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0 := ratLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1 := ratLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1 := ratLerp(seg.Start.V, seg.End.V, seg.TEnd)
	anchorU, anchorV := floatRat(anchor.U), floatRat(anchor.V)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil || anchorU == nil || anchorV == nil {
		return exactMoments{}
	}
	u0.Sub(u0, anchorU)
	u1.Sub(u1, anchorU)
	v0.Sub(v0, anchorV)
	v1.Sub(v1, anchorV)
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
	// A circular integral's exact value carries π and trig terms, so it has no
	// exact rational and the region's rational sum ends here.
	ig.dropExact()
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
