package proof

import (
	"encoding/binary"
	"math"
	"math/big"
)

// This file holds two helpers that let a caller skip exact work without
// changing an answer.
//
//   - Outward float enclosures. FloatBounds brackets a Dyadic between two
//     floats, and DotSubEnclosure brackets x·y − c from those brackets. Each
//     float operation rounds to nearest and then steps one ulp outward, so the
//     exact value always lies inside the result. An enclosure that decides a
//     sign decides the exact sign. Anything else defers to exact arithmetic.
//   - Canonical keys. DvPrimitive and Dyadic.AppendKey give a value a form
//     that is equal exactly when the values are, so a map lookup can stand in
//     for a scan of exact comparisons.

// FloatBounds returns floats lo ≤ d ≤ hi. A float conversion that is not exact
// rounds to the nearest float, so the exact value lies strictly between that
// float's neighbours. That also holds when the conversion overflows to an
// infinity: the infinity's neighbour toward zero is MaxFloat64.
func FloatBounds(d Dyadic) (float64, float64) {
	f, exact := d.Float64()
	if exact {
		return f, f
	}
	return math.Nextafter(f, math.Inf(-1)), math.Nextafter(f, math.Inf(1))
}

// FloatBox3 is an exact vector's outward float box: Lo is at or below each
// exact component and Hi at or above it.
type FloatBox3 struct {
	Lo, Hi [3]float64
}

// DvFloatBox returns v's outward float box.
func DvFloatBox(v DyV3) FloatBox3 {
	var out FloatBox3
	for axis := range 3 {
		out.Lo[axis], out.Hi[axis] = FloatBounds(v[axis])
	}
	return out
}

// The directed operations below round one float operation to nearest and then
// step the result one ulp outward. The exact result lies within half an ulp of
// the rounded one, so the step lands on the far side of it. An overflow to an
// infinity steps to MaxFloat64 toward zero, which the exact result also
// passes. The explicit float64 conversion of a product stops the compiler from
// fusing it into a later add, which would round differently.
func floatDown(x float64) float64       { return math.Nextafter(x, math.Inf(-1)) }
func floatUp(x float64) float64         { return math.Nextafter(x, math.Inf(1)) }
func floatMulDown(a, b float64) float64 { return floatDown(float64(a * b)) }
func floatMulUp(a, b float64) float64   { return floatUp(float64(a * b)) }

// DotSubEnclosure returns floats lo ≤ x·y − c ≤ hi for every pair of vectors
// in the boxes x and y and every c in [cLo, cHi]. Each product lies between
// its least and greatest corner product. A NaN, from an infinity times zero or
// opposed infinities, makes an end NaN, which every ordered comparison treats
// as undecided.
func DotSubEnclosure(x, y FloatBox3, cLo, cHi float64) (float64, float64) {
	lo, hi := 0.0, 0.0
	for axis := range 3 {
		a, b, c, d := x.Lo[axis], x.Hi[axis], y.Lo[axis], y.Hi[axis]
		plo := math.Min(math.Min(floatMulDown(a, c), floatMulDown(a, d)),
			math.Min(floatMulDown(b, c), floatMulDown(b, d)))
		phi := math.Max(math.Max(floatMulUp(a, c), floatMulUp(a, d)),
			math.Max(floatMulUp(b, c), floatMulUp(b, d)))
		lo, hi = floatDown(lo+plo), floatUp(hi+phi)
	}
	return floatDown(lo - cHi), floatUp(hi - cLo)
}

// DvPrimitive returns the primitive integer vector along a nonzero v: v times
// the positive rational that makes its components integers with no common
// factor. Two nonzero vectors share it exactly when one is a positive multiple
// of the other, so it names v's direction, sign kept. A zero v returns zero.
func DvPrimitive(v DyV3) DyV3 {
	exp, found := 0, false
	for _, c := range v {
		if c.IsZero() {
			continue
		}
		if !found || c.exp < exp {
			exp = c.exp
		}
		found = true
	}
	if !found {
		return DyV3{}
	}
	var ints [3]*big.Int
	gcd := new(big.Int)
	for i, c := range v {
		if c.IsZero() {
			continue
		}
		ints[i] = new(big.Int).Lsh(c.mant, uint(c.exp-exp))
		if gcd.Sign() == 0 {
			gcd.Abs(ints[i])
			continue
		}
		gcd.GCD(nil, nil, gcd, new(big.Int).Abs(ints[i]))
	}
	var out DyV3
	for i, k := range ints {
		if k == nil {
			continue
		}
		if gcd.BitLen() > 1 {
			k.Quo(k, gcd)
		}
		out[i] = Dyadic{mant: k}.norm()
	}
	return out
}

// AppendKey appends a byte encoding of d to b. Two encodings are equal exactly
// when the values are: the encoding reads the reduced form (norm), whose
// fields are equal exactly when the values are, and it prefixes the
// mantissa's length so no two encodings run together.
func (d Dyadic) AppendKey(b []byte) []byte {
	if d.IsZero() {
		return append(b, 0)
	}
	mant, exp := d.mant, d.exp
	if shift := mant.TrailingZeroBits(); shift > 0 {
		mant = new(big.Int).Rsh(mant, shift)
		exp += int(shift)
	}
	sign := byte(1)
	if mant.Sign() < 0 {
		sign = 2
	}
	b = append(b, sign)
	b = binary.AppendVarint(b, int64(exp))
	words := mant.Bytes()
	b = binary.AppendUvarint(b, uint64(len(words)))
	return append(b, words...)
}
