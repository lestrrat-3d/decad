package proof

import (
	"math"
	"math/big"
)

// Directed float64 roundings of a quotient of
// two Dyadics, without forming a big.Rat: no GCD, one integer QuoRem.
//
// dyQuoRound rounds n/d (n >= 0, d > 0) to a float64 either to nearest-even
// (up=false; what big.Rat.Float64 answers, and xhpFractionFloat64's algorithm)
// or toward +Inf (up=true; the smallest float64 at or above n/d, which is what
// proofbound.RatFloatUp answers). ok is false where the result would leave the
// normal float64 range; the caller then takes the big.Rat path, so the fast
// path never has to reproduce subnormal or overflow rounding.
func dyQuoRound(n, d Dyadic, up bool) (float64, bool, bool) {
	if n.IsZero() {
		return 0, true, true
	}
	var numerator, denominator big.Int
	a := n.MantInto(&numerator)
	b := d.MantInto(&denominator)
	exp := a.BitLen() - b.BitLen()
	if shift := 54 - exp; shift > 0 {
		a.Lsh(a, uint(shift))
	} else if shift < 0 {
		b.Lsh(b, uint(-shift))
	}
	var q, r big.Int
	q.QuoRem(a, b, &r)
	mant := q.Uint64()
	sticky := r.Sign() != 0
	if mant>>54 == 1 {
		if mant&1 != 0 {
			sticky = true
		}
		mant >>= 1
		exp++
	}
	// mant has 54 bits; its value is mant·2^(exp-54) scaled by 2^(n.exp-d.exp).
	exact := !sticky && mant&1 == 0
	if up {
		mant >>= 1
		if !exact {
			mant++
			if mant >= 1<<53 {
				mant >>= 1
				exp++
			}
		}
	} else {
		if mant&1 != 0 && (sticky || mant&2 != 0) {
			mant++
			if mant >= 1<<54 {
				mant >>= 1
				exp++
			}
		}
		mant >>= 1
	}
	// Leading bit of the 53-bit mant sits at 2^52; the value is mant·2^scale.
	scale := exp - 53 + n.exp - d.exp
	if top := scale + 52; top < -1022 || top > 1023 {
		return 0, false, false
	}
	if scale >= -dyPow2Bias && scale <= dyPow2Bias {
		return float64(mant) * dyPow2[scale+dyPow2Bias], exact, true
	}
	return math.Ldexp(float64(mant), scale), exact, true
}

// DyQuoFloatUp is proofbound.RatFloatUp over the exact quotient n/d, for
// n >= 0 and d > 0: the smallest float64 at or above n/d, bit for bit. Outside
// the normal range it takes the big.Rat path itself.
func DyQuoFloatUp(n, d Dyadic) float64 {
	if n.Sign() >= 0 && d.Sign() > 0 {
		if f, _, ok := dyQuoRound(n, d, true); ok {
			return f
		}
	}
	q := new(big.Rat).Quo(n.Rat(), d.Rat())
	f, _ := q.Float64()
	if math.IsNaN(f) || math.IsInf(f, 0) {
		return f
	}
	if fr := FloatRat(f); fr != nil && fr.Cmp(q) >= 0 {
		return f
	}
	return math.Nextafter(f, math.Inf(1))
}

// DyQuoFloatError is RationalFloatError(n/d, held) for d > 0: |n/d − held|
// rounded to nearest and stepped one ulp up when that rounding was inexact.
func DyQuoFloatError(n, d Dyadic, held float64) float64 {
	heldDy, ok := DyOf(held)
	if !ok || d.Sign() <= 0 {
		return math.Inf(1)
	}
	num := DyAbs(DySubScalar(n, DyMul(heldDy, d)))
	if f, exact, ok := dyQuoRound(num, d, false); ok {
		if !exact {
			f = NextUp(f)
		}
		return f
	}
	return RationalFloatError(new(big.Rat).Quo(n.Rat(), d.Rat()), held)
}
