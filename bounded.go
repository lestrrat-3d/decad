package decad

import (
	"math"
	"math/big"
)

// This file is the package's bounded-scalar vocabulary: a float64 value
// carried beside a proven upper bound on its own error, and the arithmetic
// that composes the two together.
//
// Every operation here charges its OWN rounding on top of the operand bounds
// it composes, so a boundedScalar's bound is never smaller than the error
// actually committed to reach it. The three admission readers (admitAbove,
// admitBelow, admitMagnitudeAbove) are three-valued for the same reason: an
// interval straddling the threshold answers survStraddle rather than picking
// the side its held value happens to sit on.
//
// moments.go and every reading built on it consume this vocabulary; bounds.go
// owns the mechanism-specific allowances that feed it.

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

// boundedNeg flips a held value's sign. IEEE negation is exact, so the proven
// bound rides along unchanged.
func boundedNeg(a boundedScalar) boundedScalar {
	return measuredScalar(-a.value, a.bound)
}

// boundedMin encloses min(a, b), which is [min(a.lo, b.lo), min(a.hi, b.hi)]
// and NEVER the interval of whichever HELD value compared smaller. Two held
// floats cannot decide which QUANTITY is smaller while their proven intervals
// overlap, so publishing the selected endpoint's own bound throws away the case
// where the discarded one was the smaller — the enclosure then excludes a
// minimum it was supposed to contain, however narrowly.
//
// The published held value is the smaller of the two, which always lies inside
// that enclosure, and the bound reaches whichever end sits further from it.
// Both ends come from boundedEnds, so a zero-bound pair keeps a zero bound and
// stays exact.
func boundedMin(a, b boundedScalar) boundedScalar {
	value := math.Min(a.value, b.value)
	aLo, aHi := boundedEnds(a)
	bLo, bHi := boundedEnds(b)
	lo, hi := math.Min(aLo, bLo), math.Min(aHi, bHi)
	if isNonFinite(value) || isNonFinite(lo) || isNonFinite(hi) {
		return measuredScalar(value, math.Inf(1))
	}
	return measuredScalar(value, math.Max(upRound(value-lo), upRound(hi-value)))
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

// survAdmission is the three-valued reading of a bounded quantity against a
// threshold: the answer a HELD float cannot give, because the held float is not
// the quantity. It is the single owner of that reading — the survey kernel, the
// revolve's own wedge resolution and the candidate generators all ask it rather
// than comparing a `.value` field against a constant — so a cell that cannot
// decide is visible as a state instead of silently taking one branch.
type survAdmission int

const (
	// survReject: the whole proven interval fails the test.
	survReject survAdmission = iota
	// survAdmit: the whole proven interval passes it.
	survAdmit
	// survStraddle: the interval contains the threshold, so the held value
	// decides nothing. What a cell does with this is the cell's own business
	// and is documented where it asks — generate the candidate anyway (a
	// superfluous candidate is re-checked against the whole boundary before it
	// can reach a reading), or refuse.
	survStraddle
)

// boundedEnds is the proven interval of a bounded scalar, stepped outward so
// the two ends' own rounding can never pull them inside the interval they
// stand for. A non-finite bound answers the whole line, which every reading
// below turns into survStraddle.
func boundedEnds(q boundedScalar) (float64, float64) {
	if isNonFinite(q.value) || isNonFinite(q.bound) {
		return math.Inf(-1), math.Inf(1)
	}
	if q.bound == 0 {
		return q.value, q.value
	}
	return math.Nextafter(q.value-q.bound, math.Inf(-1)),
		math.Nextafter(q.value+q.bound, math.Inf(1))
}

// admitAbove reads `q > t`.
func admitAbove(q boundedScalar, t float64) survAdmission {
	lo, hi := boundedEnds(q)
	switch {
	case lo > t:
		return survAdmit
	case hi <= t:
		return survReject
	default:
		return survStraddle
	}
}

// admitBelow reads `q < t`.
func admitBelow(q boundedScalar, t float64) survAdmission {
	lo, hi := boundedEnds(q)
	switch {
	case hi < t:
		return survAdmit
	case lo >= t:
		return survReject
	default:
		return survStraddle
	}
}

// admitMagnitudeAbove reads `|q| > t` for a non-negative t: the degeneracy
// question every closed-form solve asks of its own denominator. The magnitude's
// own range over the interval is what decides it — an interval spanning zero
// reaches magnitude zero, whatever its ends read.
func admitMagnitudeAbove(q boundedScalar, t float64) survAdmission {
	lo, hi := boundedEnds(q)
	upper := math.Max(math.Abs(lo), math.Abs(hi))
	lower := 0.0
	if lo > 0 {
		lower = lo
	} else if hi < 0 {
		lower = -hi
	}
	switch {
	case lower > t:
		return survAdmit
	case upper <= t:
		return survReject
	default:
		return survStraddle
	}
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
// Go deliberately gives Sin, Cos, Atan2 and Hypot no public ulp contract, so a
// result computed through them never trusts this helper's roundoff budget on
// its own. Where no certified rational bracket exists for the result,
// conservativeValueError's magnitude envelope is what stands; where one does
// (circularAreaInterval, circularLengthInterval, circularFirstMomentInterval —
// each reducing the trig terms to exact rationals, or to a proven
// atan2Interval/ratSqrtDown/ratSqrtUp/turnSinCosInterval bracket over exact
// rationals or the shared fixed-point grid, so no libm accuracy is assumed
// either), the caller takes math.Min of the two, which can only shrink the
// published bound.
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
// package and remains finite for finite geometry. It is the bound a circular
// reading falls back to wherever no certified rational bracket admits it (a
// trimmed CircleSeg fragment used as a first moment, an ArcSeg fragment whose
// two endpoints round to different radii); every reading a bracket does admit
// takes math.Min against this value, never a replacement of it.
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

// productUpper is a PROVEN upper bound on a·b for two non-negative bounds.
//
// An operand at or below zero is an ABSENT term and answers an honest 0 — the
// only way a product legitimately vanishes, and the only zero exactnessOf may
// read as a claim of exactness. Two POSITIVE operands can never answer 0: the
// true product is positive, so a rounded +0 is float64's own underflow flush,
// and bounds.go's provenUpRound replaces it with the smallest subnormal, a
// correct finite upper bound on anything that flushed. Its own `a > 0 && b > 0`
// arm is the proof of positivity provenUpRound requires; no caller has to
// repeat it.
//
// A +Inf operand is a REFUSAL and not a magnitude, so it wins over a zero
// rather than being annihilated by it: an unbounded factor times an absent one
// bounds nothing, and answering 0 there would republish a refusal as a proven
// zero.
func productUpper(a, b float64) float64 {
	if math.IsInf(a, 1) || math.IsInf(b, 1) {
		return math.Inf(1)
	}
	if a <= 0 || b <= 0 {
		return 0
	}
	return provenUpRound(a * b)
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
