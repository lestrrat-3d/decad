package polynomial

import (
	"context"
	"math"
	"math/big"
)

// CsPoly is a trigonometric polynomial in the canonical form A(c) + s·B(c)
// with c = cosθ, s = sinθ (every s² reduced to 1 − c²): closed under
// products and d/dθ, and convertible to a plain polynomial in t = tan(θ/2).
type CsPoly struct{ A, B RatPoly }

func CsConst(f float64) (CsPoly, bool) {
	a, ok := RatPolyOf(f)
	return CsPoly{A: a}, ok
}

// CsLin builds k0 + kc·cosθ + ks·sinθ from float coefficients taken exactly.
func CsLin(k0, kc, ks float64) (CsPoly, bool) {
	a, ok := RatPolyOf(k0, kc)
	if !ok {
		return CsPoly{}, false
	}
	b, ok := RatPolyOf(ks)
	return CsPoly{A: a, B: b}, ok
}

func CsAdd(x, y CsPoly) CsPoly { return CsPoly{A: RpAdd(x.A, y.A), B: RpAdd(x.B, y.B)} }

func CsSub(x, y CsPoly) CsPoly { return CsPoly{A: RpSub(x.A, y.A), B: RpSub(x.B, y.B)} }

// CsMul multiplies with the s² = 1 − c² reduction.
func CsMul(x, y CsPoly) CsPoly {
	oneMinusC2 := RatPoly{big.NewRat(1, 1), new(big.Rat), big.NewRat(-1, 1)}
	a := RpAdd(RpMul(x.A, y.A), RpMul(oneMinusC2, RpMul(x.B, y.B)))
	b := RpAdd(RpMul(x.A, y.B), RpMul(x.B, y.A))
	return CsPoly{A: a, B: b}
}

// CsDerivTheta is d/dθ: (A + sB)' = [cB − (1−c²)B'] − s·A'.
func CsDerivTheta(x CsPoly) CsPoly {
	oneMinusC2 := RatPoly{big.NewRat(1, 1), new(big.Rat), big.NewRat(-1, 1)}
	cTimesB := RpMul(RatPoly{new(big.Rat), big.NewRat(1, 1)}, x.B)
	a := RpSub(cTimesB, RpMul(oneMinusC2, RpDeriv(x.B)))
	return CsPoly{A: a, B: RpNeg(RpDeriv(x.A))}
}

func CsIsZero(x CsPoly) bool {
	return len(RpTrim(x.A)) == 0 && len(RpTrim(x.B)) == 0
}

// OmPow is (1 − c²)^k.
func OmPow(k int) RatPoly {
	out := RatPoly{big.NewRat(1, 1)}
	base := RatPoly{big.NewRat(1, 1), new(big.Rat), big.NewRat(-1, 1)}
	for range k {
		out = RpMul(out, base)
	}
	return out
}

// CsQuarterShift maps θ → θ + π/2 exactly: cos → −sin, sin → cos.
func CsQuarterShift(x CsPoly) CsPoly {
	var a, b RatPoly
	cPoly := RatPoly{new(big.Rat), big.NewRat(1, 1)}
	for i, ai := range x.A {
		if ai.Sign() == 0 {
			continue
		}
		if i%2 == 0 {
			a = RpAdd(a, RpScale(OmPow(i/2), ai))
			continue
		}
		neg := new(big.Rat).Neg(ai)
		b = RpAdd(b, RpScale(OmPow((i-1)/2), neg))
	}
	for j, bj := range x.B {
		if bj.Sign() == 0 {
			continue
		}
		if j%2 == 0 {
			a = RpAdd(a, RpMul(cPoly, RpScale(OmPow(j/2), bj)))
			continue
		}
		neg := new(big.Rat).Neg(bj)
		b = RpAdd(b, RpMul(cPoly, RpScale(OmPow((j-1)/2), neg)))
	}
	return CsPoly{A: a, B: b}
}

// CsToT substitutes the half-angle chart t = tan(θ/2), clearing the
// (1 + t²)^K denominator: root sets over θ ∈ (−π, π) map bijectively.
func CsToT(x CsPoly) RatPoly {
	a, b := RpTrim(x.A), RpTrim(x.B)
	k := max(len(a)-1, len(b))
	num := RatPoly{big.NewRat(1, 1), new(big.Rat), big.NewRat(-1, 1)} // 1 − t²
	den := RatPoly{big.NewRat(1, 1), new(big.Rat), big.NewRat(1, 1)}  // 1 + t²
	twoT := RatPoly{new(big.Rat), big.NewRat(2, 1)}
	powNum := func(n int) RatPoly {
		out := RatPoly{big.NewRat(1, 1)}
		for range n {
			out = RpMul(out, num)
		}
		return out
	}
	powDen := func(n int) RatPoly {
		out := RatPoly{big.NewRat(1, 1)}
		for range n {
			out = RpMul(out, den)
		}
		return out
	}
	var out RatPoly
	for i, ai := range a {
		if ai.Sign() == 0 {
			continue
		}
		out = RpAdd(out, RpScale(RpMul(powNum(i), powDen(k-i)), ai))
	}
	for j, bj := range b {
		if bj.Sign() == 0 {
			continue
		}
		term := RpMul(twoT, RpMul(powNum(j), powDen(k-1-j)))
		out = RpAdd(out, RpScale(term, bj))
	}
	return RpTrim(out)
}

// CritBracket is one certified enclosure of a stationary point: the critical
// parameter lies in [thLo, thHi], and the objective's value there in
// [lo, hi].
type CritBracket struct {
	ThLo, ThHi float64
	Lo, Hi     float64
}

func (b CritBracket) Mid() float64 { return (b.ThLo + b.ThHi) / 2 }

// TrigStationaryBracketsContext isolates every zero of the stationarity polynomial
// f over the full circle and returns certified enclosures of the objective g
// at each of them: g is 1-Lipschitz-composed with a parameterization whose
// speed is at most lip, and slack absorbs floating evaluation noise. The
// second result is false when f is identically zero — a constant objective,
// the caller's closed-form path.
func TrigStationaryBracketsContext(ctx context.Context, f CsPoly, g func(float64) float64, lip, slack float64) ([]CritBracket, bool, error) {
	if CsIsZero(f) {
		return nil, false, nil
	}
	var out []CritBracket
	// Two quarter-turn charts cover the circle: t = tan(θ/2) misses θ = ±π,
	// which the shifted chart holds interior.
	for _, chart := range []struct {
		shift float64
		poly  CsPoly
	}{{shift: 0, poly: f}, {shift: math.Pi / 2, poly: CsQuarterShift(f)}} {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		p := RpSquareFree(RpTrim(CsToT(chart.poly)))
		if RpDeg(p) < 1 {
			continue
		}
		chain, err := SturmChainIntContext(ctx, p)
		if err != nil {
			return nil, false, err
		}
		ivs, err := RpIsolateRootsContext(ctx, p, chain)
		if err != nil {
			return nil, false, err
		}
		for _, iv := range ivs {
			iv, err = RpRefineRootContext(ctx, chain, iv, func(lo, hi float64) bool {
				return 2*math.Atan(hi)-2*math.Atan(lo) <= 1e-11
			})
			if err != nil {
				return nil, false, err
			}
			tLo, _ := iv.Lo.Float64()
			tHi, _ := iv.Hi.Float64()
			thLo := chart.shift + 2*math.Atan(tLo)
			thHi := chart.shift + 2*math.Atan(tHi)
			v := g((thLo + thHi) / 2)
			half := lip*(thHi-thLo)/2 + slack
			out = append(out, CritBracket{ThLo: thLo, ThHi: thHi, Lo: v - half, Hi: v + half})
		}
	}
	return out, true, nil
}
