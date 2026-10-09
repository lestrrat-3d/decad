package proofbound

import (
	"math"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// This file is the package's bounded-scalar vocabulary: a float64 value
// carried beside a proven upper bound on its own error, and the arithmetic
// that composes the two together.
//
// The rounding primitives live in internal/proof/rounding.go. Every bounded
// operation here charges its OWN rounding on top of the operand bounds
// it composes, so a BoundedScalar's bound is never smaller than the error
// actually committed to reach it. The three admission readers (AdmitAbove,
// AdmitBelow, AdmitMagnitudeAbove) are three-valued for the same reason: an
// interval straddling the threshold answers SurvStraddle rather than picking
// the side its held value happens to sit on.
//
// moments.go and every reading built on it consume this vocabulary; bounds.go
// owns the mechanism-specific allowances that feed it.

// BoundedScalar is a held float64 and a proven absolute error bound. The
// arithmetic helpers below propagate input intervals and add the exact
// round-to-nearest error of the held operation, measured with big.Rat. This
// keeps a result Exact only when both its inputs and its final float are exact.
type BoundedScalar struct {
	Value float64
	Bound float64
}

func ExactScalar(value float64) BoundedScalar {
	return BoundedScalar{Value: value}
}

func MeasuredScalar(value, bound float64) BoundedScalar {
	return BoundedScalar{Value: value, Bound: bound}
}

func BoundedAdd(a, b BoundedScalar) BoundedScalar {
	value := a.Value + b.Value
	bound := AbsSumUpper(a.Bound, b.Bound, proofarith.AddRoundError(a.Value, b.Value, value))
	return MeasuredScalar(value, bound)
}

func BoundedSub(a, b BoundedScalar) BoundedScalar {
	return BoundedAdd(a, MeasuredScalar(-b.Value, b.Bound))
}

func BoundedAbs(a BoundedScalar) BoundedScalar {
	a.Value = math.Abs(a.Value)
	return a
}

// BoundedNeg flips a held value's sign. IEEE negation is exact, so the proven
// bound rides along unchanged.
func BoundedNeg(a BoundedScalar) BoundedScalar {
	return MeasuredScalar(-a.Value, a.Bound)
}

// BoundedMin encloses min(a, b), which is [min(a.lo, b.lo), min(a.hi, b.hi)]
// and NEVER the interval of whichever HELD value compared smaller. Two held
// floats cannot decide which QUANTITY is smaller while their proven intervals
// overlap, so publishing the selected endpoint's own bound throws away the case
// where the discarded one was the smaller — the enclosure then excludes a
// minimum it was supposed to contain, however narrowly.
//
// The published held value is the smaller of the two, which always lies inside
// that enclosure, and the bound reaches whichever end sits further from it.
// Both ends come from BoundedEnds, so a zero-bound pair keeps a zero bound and
// stays exact.
func BoundedMin(a, b BoundedScalar) BoundedScalar {
	value := math.Min(a.Value, b.Value)
	aLo, aHi := BoundedEnds(a)
	bLo, bHi := BoundedEnds(b)
	lo, hi := math.Min(aLo, bLo), math.Min(aHi, bHi)
	if IsNonFinite(value) || IsNonFinite(lo) || IsNonFinite(hi) {
		return MeasuredScalar(value, math.Inf(1))
	}
	return MeasuredScalar(value, math.Max(UpRound(value-lo), UpRound(hi-value)))
}

func BoundedMul(a, b BoundedScalar) BoundedScalar {
	value := a.Value * b.Value
	bound := AbsSumUpper(
		ProductUpper(math.Abs(a.Value), b.Bound),
		ProductUpper(math.Abs(b.Value), a.Bound),
		ProductUpper(a.Bound, b.Bound),
		proofarith.MulRoundError(a.Value, b.Value, value),
	)
	return MeasuredScalar(value, bound)
}

func BoundedQuotient(num float64, numBound float64, den float64, denBound float64) BoundedScalar {
	value := num / den
	clearance := math.Nextafter(math.Abs(den)-denBound, math.Inf(-1))
	if clearance <= 0 {
		return MeasuredScalar(value, math.Inf(1))
	}
	centralRound := proofarith.DivRoundError(num, den, value)
	centralUpper := AbsSumUpper(value, centralRound)
	numerator := AbsSumUpper(numBound, ProductUpper(centralUpper, denBound))
	bound := UpRound(numerator / clearance)
	return MeasuredScalar(value, AbsSumUpper(bound, centralRound))
}

func BoundedDiv(a, b BoundedScalar) BoundedScalar {
	return BoundedQuotient(a.Value, a.Bound, b.Value, b.Bound)
}

// BoundedStretch keeps a's held value and widens its bound to cover the true
// quantity multiplied by an unknown factor f in [1 − r, 1 + r]: with t the
// true unscaled value, |f·t − a.Value| ≤ |f − 1|·|t| + |t − a.Value| ≤
// r·(|a.Value| + a.Bound) + a.Bound. r ≤ 0 returns a unchanged, so an exact
// factor keeps the bound bit for bit.
func BoundedStretch(a BoundedScalar, r float64) BoundedScalar {
	if r <= 0 {
		return a
	}
	return MeasuredScalar(a.Value, AbsSumUpper(a.Bound, ProductUpper(r, AbsSumUpper(a.Value, a.Bound))))
}

// SurvAdmission is the three-valued reading of a bounded quantity against a
// threshold: the answer a HELD float cannot give, because the held float is not
// the quantity. It is the single owner of that reading — the survey kernel, the
// revolve's own wedge resolution and the candidate generators all ask it rather
// than comparing a `.value` field against a constant — so a cell that cannot
// decide is visible as a state instead of silently taking one branch.
type SurvAdmission int

const (
	// SurvReject: the whole proven interval fails the test.
	SurvReject SurvAdmission = iota
	// SurvAdmit: the whole proven interval passes it.
	SurvAdmit
	// SurvStraddle: the interval contains the threshold, so the held value
	// decides nothing. What a cell does with this is the cell's own business
	// and is documented where it asks — generate the candidate anyway (a
	// superfluous candidate is re-checked against the whole boundary before it
	// can reach a reading), or refuse.
	SurvStraddle
)

// BoundedEnds is the proven interval of a bounded scalar, stepped outward so
// the two ends' own rounding can never pull them inside the interval they
// stand for. A non-finite bound answers the whole line, which every reading
// below turns into SurvStraddle.
func BoundedEnds(q BoundedScalar) (float64, float64) {
	if IsNonFinite(q.Value) || IsNonFinite(q.Bound) {
		return math.Inf(-1), math.Inf(1)
	}
	if q.Bound == 0 {
		return q.Value, q.Value
	}
	return math.Nextafter(q.Value-q.Bound, math.Inf(-1)),
		math.Nextafter(q.Value+q.Bound, math.Inf(1))
}

// AdmitAbove reads `q > t`.
func AdmitAbove(q BoundedScalar, t float64) SurvAdmission {
	lo, hi := BoundedEnds(q)
	switch {
	case lo > t:
		return SurvAdmit
	case hi <= t:
		return SurvReject
	default:
		return SurvStraddle
	}
}

// AdmitBelow reads `q < t`.
func AdmitBelow(q BoundedScalar, t float64) SurvAdmission {
	lo, hi := BoundedEnds(q)
	switch {
	case hi < t:
		return SurvAdmit
	case lo >= t:
		return SurvReject
	default:
		return SurvStraddle
	}
}

// AdmitMagnitudeAbove reads `|q| > t` for a non-negative t: the degeneracy
// question every closed-form solve asks of its own denominator. The magnitude's
// own range over the interval is what decides it — an interval spanning zero
// reaches magnitude zero, whatever its ends read.
func AdmitMagnitudeAbove(q BoundedScalar, t float64) SurvAdmission {
	lo, hi := BoundedEnds(q)
	upper := math.Max(math.Abs(lo), math.Abs(hi))
	lower := 0.0
	if lo > 0 {
		lower = lo
	} else if hi < 0 {
		lower = -hi
	}
	switch {
	case lower > t:
		return SurvAdmit
	case upper <= t:
		return SurvReject
	default:
		return SurvStraddle
	}
}

func BoundedCos(x BoundedScalar) BoundedScalar {
	value := math.Cos(x.Value)
	return MeasuredScalar(value, ConservativeValueError(value, 1))
}

// Radius2D turns independent coordinate bounds into a plane-distance bound.
// sqrt2Up is √2 rounded upward, so the square containing both coordinate
// intervals is enclosed without relying on a rounded square root.
func Radius2D(x, y float64) float64 {
	const sqrt2Up = 1.4142135623730952
	if x <= 0 && y <= 0 {
		return 0
	}
	return ProductUpper(math.Max(x, y), sqrt2Up)
}

// AnalyticRoundBound is the analytic evaluator's basic-arithmetic roundoff
// budget. Each caller supplies an absolute-term envelope and evaluates fewer
// than 128 additions, multiplications or divisions, so 256·u·scale dominates
// their round-to-nearest error without cancellation shrinking the bound.
//
// Go deliberately gives Sin, Cos, Atan2 and Hypot no public ulp contract, so a
// result computed through them never trusts this helper's roundoff budget on
// its own. Where no certified rational bracket exists for the result,
// ConservativeValueError's magnitude envelope is what stands; where one does
// (circularAreaInterval, circularLengthInterval, circularFirstMomentInterval —
// each reducing the trig terms to exact rationals, or to a proven
// Atan2Interval/RatSqrtDown/RatSqrtUp/TurnSinCosInterval bracket over exact
// rationals or the shared fixed-point grid, so no libm accuracy is assumed
// either), the caller takes math.Min of the two, which can only shrink the
// published bound. The cap-chamfer Cone patch's brackets
// (internal/capband's conePatchFluxInterval for its volume flux and
// phaseSumInterval for its first moments) REPLACE the
// envelope wherever they build, since each encloses the whole closed form
// over exact rationals with radSinCosInterval's certified trig factors. The
// envelope stands only in the flux's non-finite fallback; the moments'
// non-finite fallback publishes an infinite bound.
func AnalyticRoundBound(scale float64) float64 {
	if scale <= 0 {
		return 0
	}
	if IsNonFinite(scale) {
		return math.Inf(1)
	}
	return ProductUpper(ProductUpper(256, UnitRoundoff), scale)
}

// ConservativeValueError proves |held-true| from |true| <= trueAbsUpper:
// |held-true| <= |held|+|true|. It is intentionally wider than an ulp estimate,
// but it is portable across every conforming implementation of Go's math
// package and remains finite for finite geometry. It is the bound a circular
// reading falls back to wherever no certified rational bracket admits it (a
// trimmed CircleSeg fragment used as a first moment, an ArcSeg fragment whose
// two endpoints round to different radii); every reading a bracket does admit
// takes math.Min against this value, never a replacement of it.
func ConservativeValueError(held, trueAbsUpper float64) float64 {
	if IsNonFinite(held) || IsNonFinite(trueAbsUpper) {
		return math.Inf(1)
	}
	return AbsSumUpper(held, math.Max(0, trueAbsUpper))
}

func AbsSumUpper(values ...float64) float64 {
	total := 0.0
	for _, value := range values {
		total = UpRound(total + math.Abs(value))
	}
	return total
}

// ProductUpper is a PROVEN upper bound on a·b for two non-negative bounds.
//
// An operand at or below zero is an ABSENT term and answers an honest 0 — the
// only way a product legitimately vanishes, and the only zero exactnessOf may
// read as a claim of exactness. Two POSITIVE operands can never answer 0: the
// true product is positive, so a rounded +0 is float64's own underflow flush,
// and bounds.go's ProvenUpRound replaces it with the smallest subnormal, a
// correct finite upper bound on anything that flushed. Its own `a > 0 && b > 0`
// arm is the proof of positivity ProvenUpRound requires; no caller has to
// repeat it.
//
// A +Inf operand is a REFUSAL and not a magnitude, so it wins over a zero
// rather than being annihilated by it: an unbounded factor times an absent one
// bounds nothing, and answering 0 there would republish a refusal as a proven
// zero.
func ProductUpper(a, b float64) float64 {
	if math.IsInf(a, 1) || math.IsInf(b, 1) {
		return math.Inf(1)
	}
	if a <= 0 || b <= 0 {
		return 0
	}
	return ProvenUpRound(a * b)
}

func TwoPiUpper() float64 {
	return ProductUpper(2, math.Nextafter(math.Pi, math.Inf(1)))
}

// CircularSweepUpper bounds |(t1-t0)·2π| for both CircleSeg and ArcSeg.
// ArcSeg's underlying sweep is at most 2π; CircleSeg uses exactly that scale.
func CircularSweepUpper(t0, t1 float64) float64 {
	return ProductUpper(AbsSumUpper(t0, t1), TwoPiUpper())
}
