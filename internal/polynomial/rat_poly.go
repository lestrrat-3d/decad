package polynomial

import (
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This package holds the shared exact polynomial and root-isolation machinery
// used by clearance, mesh boolean, and spline proofs. The P4/P8 stationarity
// polynomials of docs/clearance-design.md §4/§5 are isolated by Sturm
// sequences built over math/big.Rat on float coefficients taken exactly. Every real root lands
// in an interval that cannot lie, and a bracket on a critical VALUE follows
// from the isolated parameter interval plus a proven Lipschitz bound — the
// same adaptive-exactness discipline as the boolean's sign tests
// (evaluator §9). The chain's repeated consumer is a sign query, not a
// value, so each member is converted once to big.Int coefficients over a
// positive common denominator and evaluated for sign by a homogenised
// integer Horner (IpSign) — an exact, sign-preserving rescaling, never a
// rounding. A certified bracket is a proof, not a hope.

// RatPoly is a dense univariate polynomial over big.Rat; index i carries the
// coefficient of x^i.
type RatPoly []*big.Rat

// MustRatOf lifts a float whose caller has already proved finite.
func MustRatOf(f float64) *big.Rat {
	r, ok := proofbound.RatOf(f)
	if !ok {
		panic("decad: exact rational lift requires a finite float")
	}
	return r
}

func RatPolyOf(coeffs ...float64) (RatPoly, bool) {
	out := make(RatPoly, len(coeffs))
	for i, coeff := range coeffs {
		var ok bool
		out[i], ok = proofbound.RatOf(coeff)
		if !ok {
			return nil, false
		}
	}
	return out, true
}

func RpTrim(p RatPoly) RatPoly {
	n := len(p)
	for n > 0 && p[n-1].Sign() == 0 {
		n--
	}
	return p[:n]
}

func RpDeg(p RatPoly) int { return len(RpTrim(p)) - 1 }

func RpAdd(a, b RatPoly) RatPoly {
	n := max(len(a), len(b))
	out := make(RatPoly, n)
	for i := range out {
		out[i] = new(big.Rat)
		if i < len(a) {
			out[i].Add(out[i], a[i])
		}
		if i < len(b) {
			out[i].Add(out[i], b[i])
		}
	}
	return out
}

func RpNeg(a RatPoly) RatPoly {
	out := make(RatPoly, len(a))
	for i := range a {
		out[i] = new(big.Rat).Neg(a[i])
	}
	return out
}

func RpSub(a, b RatPoly) RatPoly { return RpAdd(a, RpNeg(b)) }

func RpMul(a, b RatPoly) RatPoly {
	a, b = RpTrim(a), RpTrim(b)
	if len(a) == 0 || len(b) == 0 {
		return RatPoly{}
	}
	out := make(RatPoly, len(a)+len(b)-1)
	for i := range out {
		out[i] = new(big.Rat)
	}
	tmp := new(big.Rat)
	for i, ai := range a {
		if ai.Sign() == 0 {
			continue
		}
		for j, bj := range b {
			tmp.Mul(ai, bj)
			out[i+j].Add(out[i+j], tmp)
		}
	}
	return out
}

func RpScale(a RatPoly, s *big.Rat) RatPoly {
	out := make(RatPoly, len(a))
	for i := range a {
		out[i] = new(big.Rat).Mul(a[i], s)
	}
	return out
}

func RpDeriv(a RatPoly) RatPoly {
	if len(a) <= 1 {
		return RatPoly{}
	}
	out := make(RatPoly, len(a)-1)
	for i := 1; i < len(a); i++ {
		out[i-1] = new(big.Rat).Mul(a[i], new(big.Rat).SetInt64(int64(i)))
	}
	return out
}

func RpEval(a RatPoly, x *big.Rat) *big.Rat {
	out := new(big.Rat)
	for _, c := range slices.Backward(a) {
		out.Mul(out, x)
		out.Add(out, c)
	}
	return out
}

// RpRem is the polynomial remainder of a by b (deg b >= 0).
func RpRem(a, b RatPoly) RatPoly {
	a, b = RpTrim(a), RpTrim(b)
	if len(b) == 0 {
		return a
	}
	rem := make(RatPoly, len(a))
	for i := range a {
		rem[i] = new(big.Rat).Set(a[i])
	}
	lead := b[len(b)-1]
	tmp := new(big.Rat)
	for len(rem) >= len(b) {
		rem = RpTrim(rem)
		if len(rem) < len(b) {
			break
		}
		q := new(big.Rat).Quo(rem[len(rem)-1], lead)
		shift := len(rem) - len(b)
		for j := range b {
			tmp.Mul(q, b[j])
			rem[shift+j].Sub(rem[shift+j], tmp)
		}
		rem = rem[:len(rem)-1]
	}
	return RpTrim(rem)
}

// RpNormalize divides by the leading coefficient's absolute value, taming
// coefficient blowup in the Euclidean sequences without moving any root.
func RpNormalize(a RatPoly) RatPoly {
	a = RpTrim(a)
	if len(a) == 0 {
		return a
	}
	lead := new(big.Rat).Abs(a[len(a)-1])
	inv := new(big.Rat).Inv(lead)
	return RpScale(a, inv)
}

// RpGCD is the monic polynomial GCD.
func RpGCD(a, b RatPoly) RatPoly {
	a, b = RpTrim(a), RpTrim(b)
	for len(b) > 0 {
		a, b = b, RpNormalize(RpRem(a, b))
	}
	return a
}

// RpSquareFree strips repeated roots: p / gcd(p, p').
func RpSquareFree(p RatPoly) RatPoly {
	p = RpTrim(p)
	if len(p) <= 2 {
		return p
	}
	g := RpGCD(p, RpDeriv(p))
	if RpDeg(g) < 1 {
		return p
	}
	return RpQuo(p, g)
}

// RpQuo is exact polynomial division (remainder known zero).
func RpQuo(a, b RatPoly) RatPoly {
	a, b = RpTrim(a), RpTrim(b)
	if len(b) == 0 || len(a) < len(b) {
		return RatPoly{}
	}
	out := make(RatPoly, len(a)-len(b)+1)
	rem := make(RatPoly, len(a))
	for i := range a {
		rem[i] = new(big.Rat).Set(a[i])
	}
	lead := b[len(b)-1]
	tmp := new(big.Rat)
	for i := range slices.Backward(out) {
		q := new(big.Rat).Quo(rem[i+len(b)-1], lead)
		out[i] = q
		for j := range b {
			tmp.Mul(q, b[j])
			rem[i+j].Sub(rem[i+j], tmp)
		}
	}
	return RpTrim(out)
}
