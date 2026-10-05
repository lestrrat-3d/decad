package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// These names keep the root package's existing internal callers stable while
// the arithmetic implementation lives in internal/proof.
func floatRat(value float64) *big.Rat { return proof.FloatRat(value) }

func rationalFloatError(exact *big.Rat, held float64) float64 {
	return proof.RationalFloatError(exact, held)
}

func addRoundError(a, b, held float64) float64  { return proof.AddRoundError(a, b, held) }
func mulRoundError(a, b, held float64) float64  { return proof.MulRoundError(a, b, held) }
func exactFloatSquare(root, value float64) bool { return proof.ExactFloatSquare(root, value) }
func divRoundError(a, b, held float64) float64  { return proof.DivRoundError(a, b, held) }

func toProofInterval(a ratInterval) proof.RatInterval {
	return proof.RatInterval{Lo: a.lo, Hi: a.hi}
}

func fromProofInterval(a proof.RatInterval) ratInterval {
	return ratInterval{lo: a.Lo, hi: a.Hi}
}

func interval(lo, hi *big.Rat) ratInterval {
	return fromProofInterval(proof.Interval(lo, hi))
}

func intervalOwned(lo, hi *big.Rat) ratInterval {
	return fromProofInterval(proof.OwnedInterval(lo, hi))
}

func pointInterval(value *big.Rat) ratInterval {
	return fromProofInterval(proof.PointInterval(value))
}

func intervalAdd(a, b ratInterval) ratInterval {
	return fromProofInterval(proof.AddInterval(toProofInterval(a), toProofInterval(b)))
}

func intervalNeg(a ratInterval) ratInterval {
	return fromProofInterval(proof.NegInterval(toProofInterval(a)))
}

func intervalSub(a, b ratInterval) ratInterval {
	return fromProofInterval(proof.SubInterval(toProofInterval(a), toProofInterval(b)))
}

func intervalScale(a ratInterval, scale *big.Rat) ratInterval {
	return fromProofInterval(proof.ScaleInterval(toProofInterval(a), scale))
}

func intervalMul(a, b ratInterval) ratInterval {
	return fromProofInterval(proof.MulInterval(toProofInterval(a), toProofInterval(b)))
}

func intervalFloatError(a ratInterval, held float64) float64 {
	return proof.IntervalFloatError(toProofInterval(a), held)
}

// These wrappers keep existing root-package consumers on the exact proof core.
type dyadic = proof.Dyadic
type dyV3 = proof.DyV3

func dyZero() dyadic                                      { return proof.DyZero() }
func dyOf(f float64) (dyadic, bool)                       { return proof.DyOf(f) }
func dyOfFiniteInto(f float64, mant *big.Int) dyadic      { return proof.DyOfFiniteInto(f, mant) }
func mustDyOf(f float64) dyadic                           { return proof.MustDyOf(f) }
func dyAdd(a, b dyadic) dyadic                            { return proof.DyAdd(a, b) }
func dySubScalar(a, b dyadic) dyadic                      { return proof.DySubScalar(a, b) }
func dyMul(a, b dyadic) dyadic                            { return proof.DyMul(a, b) }
func dyCmp(a, b dyadic) int                               { return proof.DyCmp(a, b) }
func dyAbs(d dyadic) dyadic                               { return proof.DyAbs(d) }
func dyNeg(d dyadic) dyadic                               { return proof.DyNeg(d) }
func dyOfRat(r *big.Rat) (dyadic, bool)                   { return proof.DyOfRat(r) }
func dyVec(v r3.Vec) dyV3                                 { return proof.DyVec(v) }
func dvSub(a, b dyV3) dyV3                                { return proof.DvSub(a, b) }
func dvAdd(a, b dyV3) dyV3                                { return proof.DvAdd(a, b) }
func dvCross(a, b dyV3) dyV3                              { return proof.DvCross(a, b) }
func dvDot(a, b dyV3) dyadic                              { return proof.DvDot(a, b) }
func dvIsZero(a dyV3) bool                                { return proof.DvIsZero(a) }
func dySquareAtMost(f float64, d dyadic) bool             { return proof.DySquareAtMost(f, d) }
func dySquareEquals(f float64, d dyadic) bool             { return proof.DySquareEquals(f, d) }
func dySqrtDown(d dyadic) float64                         { return proof.DySqrtDown(d) }
func dySqrtUp(d dyadic) float64                           { return proof.DySqrtUp(d) }
func dyInt(v int64) dyadic                                { return proof.DyInt(v) }
func dyShift(d dyadic, n int) dyadic                      { return proof.DyShift(d, n) }
func dyFloatDown(d dyadic) float64                        { return proof.DyFloatDown(d) }
func dyFloatUp(d dyadic) float64                          { return proof.DyFloatUp(d) }
func dyadicFloatError(exact dyadic, held float64) float64 { return proof.DyadicFloatError(exact, held) }
func dyRoundedFloatError(exact dyadic, held float64) float64 {
	return proof.DyRoundedFloatError(exact, held)
}
func dyLerp(start, end, t float64) (dyadic, bool) { return proof.DyLerp(start, end, t) }
func dyL1Upper(values ...dyadic) float64          { return proof.DyL1Upper(values...) }
