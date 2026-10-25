package proofbound

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// LazyIntervalSqrt is IntervalSqrt reading each endpoint in lowest terms.
func LazyIntervalSqrt(a proofarith.LazyInterval) (proofarith.LazyInterval, bool) {
	if a.Hi.Sign() < 0 {
		return proofarith.LazyInterval{}, false
	}
	lo := 0.0
	if a.Lo.Sign() > 0 {
		lo = RatSqrtDown(a.Lo.Rat())
	}
	hi := RatSqrtUp(a.Hi.Rat())
	rlo, rhi := proofarith.FloatRat(lo), proofarith.FloatRat(hi)
	if rlo == nil || rhi == nil {
		return proofarith.LazyInterval{}, false
	}
	return proofarith.LazyInterval{Lo: proofarith.LazyOf(rlo), Hi: proofarith.LazyOf(rhi)}, true
}

// LazyRadSinCosSpan is RadSinCosSpan: width = Hi-Lo normalised, a negative
// width refuses, RadSinCosInterval(Lo.Rat()) (the memo sees the same key as
// before), both enclosures widened lazily by trigWidening's upward-rounded
// dyadic width, exactly as RadSinCosSpan does.
func LazyRadSinCosSpan(x proofarith.LazyInterval) (proofarith.LazyInterval, proofarith.LazyInterval, bool) {
	diff := proofarith.LazySub(x.Hi, x.Lo)
	if diff.Sign() < 0 {
		return proofarith.LazyInterval{}, proofarith.LazyInterval{}, false
	}
	sin, cos, ok := RadSinCosInterval(x.Lo.Rat())
	if !ok {
		return proofarith.LazyInterval{}, proofarith.LazyInterval{}, false
	}
	width := proofarith.LazyOf(trigWidening(diff.Rat()))
	return proofarith.LazyIntervalWiden(proofarith.LazyIntervalOf(ToProofInterval(sin)), width),
		proofarith.LazyIntervalWiden(proofarith.LazyIntervalOf(ToProofInterval(cos)), width), true
}

// LazyRatFloatUp is RatFloatUp(x.Rat()): f from x.Float64(); exact or
// RatOf(f) >= x (LazyCmp) returns f, else NextUp(f); out of range it
// returns RatFloatUp(x.Rat()).
func LazyRatFloatUp(x proofarith.LazyRat) float64 {
	f, exact, ok := x.Float64()
	if !ok {
		return RatFloatUp(x.Rat())
	}
	if exact || proofarith.LazyCmp(proofarith.LazyOf(proofarith.FloatRat(f)), x) >= 0 {
		return f
	}
	return proofarith.NextUp(f)
}

// RatDistanceUp is RatFloatUp(|a-b|) without reducing a-b to lowest terms:
// LazySub forms the exact difference, LazyNeg makes it non-negative, and
// LazyRatFloatUp rounds it up. a and b are only read.
func RatDistanceUp(a, b *big.Rat) float64 {
	d := proofarith.LazySub(proofarith.LazyOf(a), proofarith.LazyOf(b))
	if d.Sign() < 0 {
		d = proofarith.LazyNeg(d)
	}
	return LazyRatFloatUp(d)
}

// LazyIntervalWidthUp is RatFloatUp of the interval's width Hi-Lo.
func LazyIntervalWidthUp(a proofarith.LazyInterval) float64 {
	return LazyRatFloatUp(proofarith.LazySub(a.Hi, a.Lo))
}

// LazyIntervalAbsUpperUp is RatFloatUp(IntervalAbsUpper(a)): |Lo| or |Hi|
// chosen by LazyCmp exactly as IntervalAbsUpper chooses (Lo when >=).
func LazyIntervalAbsUpperUp(a proofarith.LazyInterval) float64 {
	lo, hi := a.Lo, a.Hi
	if lo.Sign() < 0 {
		lo = proofarith.LazyNeg(lo)
	}
	if hi.Sign() < 0 {
		hi = proofarith.LazyNeg(hi)
	}
	if proofarith.LazyCmp(lo, hi) >= 0 {
		return LazyRatFloatUp(lo)
	}
	return LazyRatFloatUp(hi)
}

// LazyIntervalFloatError is IntervalFloatError: math.Max of LazyFloatError of
// both endpoints.
func LazyIntervalFloatError(a proofarith.LazyInterval, held float64) float64 {
	return math.Max(proofarith.LazyFloatError(a.Lo, held), proofarith.LazyFloatError(a.Hi, held))
}
