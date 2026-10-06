package proofbound

import (
	"math"
	"math/big"
)

// Moved from the root package's spline_extreme.go.

// RatFloatDown and RatFloatUp are proven outward roundings of an exact
// rational to float64: the largest float64 at or below q, and the smallest
// at or above it. big.Rat.Float64() already rounds to nearest, which can
// land on EITHER side of q, so each checks its own direction against the
// float converted back to a rational and only steps once via Nextafter when
// that check fails — an exactly representable q comes back UNCHANGED, which
// is what keeps a candidate whose true value is exact from picking up a
// spurious width it never earned.
//
// A q past the float64 range has no such rounding: Float64 saturates to an
// infinity, which each returns unchanged rather than stepping to a finite
// neighbour it cannot prove. That infinity is a REFUSAL, never a bound, so no
// caller may fold it — freeformExtremeFloats is the one conversion the fold
// runs, and it is what turns the saturation into errFreeformExtremeUnrepresentable.
func RatFloatDown(q *big.Rat) float64 {
	f, _ := q.Float64()
	if IsNonFinite(f) {
		return f
	}
	if fr, ok := RatOf(f); ok && fr.Cmp(q) <= 0 {
		return f
	}
	return math.Nextafter(f, math.Inf(-1))
}

func RatFloatUp(q *big.Rat) float64 {
	f, _ := q.Float64()
	if IsNonFinite(f) {
		return f
	}
	if fr, ok := RatOf(f); ok && fr.Cmp(q) >= 0 {
		return f
	}
	return math.Nextafter(f, math.Inf(1))
}
