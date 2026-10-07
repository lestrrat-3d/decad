package polynomial

import (
	"context"
	"math/big"
)

// SturmChainContext builds the Sturm sequence of a square-free polynomial,
// polling ctx once per chain member. One remainder step divides big.Rat
// polynomials whose coefficient height grows along the chain, so the build of
// a high-degree stationarity polynomial's chain is milliseconds of work; a
// caller that polled only around the whole build would keep running that long
// after its context was cancelled. Polling per member bounds the wait by a
// single remainder step instead.
func SturmChainContext(ctx context.Context, p RatPoly) ([]RatPoly, error) {
	chain := []RatPoly{RpTrim(p), RpTrim(RpDeriv(p))}
	for {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		last := chain[len(chain)-1]
		if len(last) == 0 {
			return chain[:len(chain)-1], nil
		}
		if RpDeg(last) == 0 {
			return chain, nil
		}
		rem := RpRem(chain[len(chain)-2], last)
		chain = append(chain, RpNormalize(RpNeg(rem)))
	}
}

// IntPoly is a dense univariate polynomial over big.Int; index i carries the
// coefficient of x^i. It holds one chain member's RatPoly numerators taken
// over a single positive common denominator (ClearDenoms), so the repeated
// sign query below pays that rescaling once rather than on every gcd
// normalisation a big.Rat Mul/Add performs.
type IntPoly []*big.Int

// SturmChainInt is a Sturm chain (SturmChainContext's return) converted member
// by member through ClearDenoms; it is read only for sign.
type SturmChainInt []IntPoly

// ClearDenoms rescales p by the least common denominator D of its
// coefficients, returning D·p's numerators as big.Int. D is always positive,
// so the rescaling changes no sign p ever reports.
func ClearDenoms(p RatPoly) IntPoly {
	den := big.NewInt(1)
	for _, c := range p {
		d := c.Denom()
		g := new(big.Int).GCD(nil, nil, den, d)
		den.Mul(den, new(big.Int).Quo(d, g))
	}
	out := make(IntPoly, len(p))
	for i, c := range p {
		out[i] = new(big.Int).Mul(c.Num(), new(big.Int).Quo(den, c.Denom()))
	}
	return out
}

// NewSturmChainInt converts every member of a built Sturm chain through
// ClearDenoms.
func NewSturmChainInt(chain []RatPoly) SturmChainInt {
	out := make(SturmChainInt, len(chain))
	for i, p := range chain {
		out[i] = ClearDenoms(p)
	}
	return out
}

// SturmChainIntContext builds a square-free polynomial's Sturm chain and
// converts it for sign queries. Every caller that hands a chain to
// RpIsolateRootsContext builds it here, so the build carries the same
// cancellation the isolation and refinement loops already have; the
// conversion itself is a rescaling per coefficient, orders of magnitude
// cheaper than the build it follows.
func SturmChainIntContext(ctx context.Context, p RatPoly) (SturmChainInt, error) {
	chain, err := SturmChainContext(ctx, p)
	if err != nil {
		return nil, err
	}
	return NewSturmChainInt(chain), nil
}

// IpSign is the sign of p(num/den) for den > 0, by homogenised integer
// Horner: sum_i c_i·num^i·den^(n−i), whose sign is p(num/den)'s because
// den^n > 0.
func IpSign(p IntPoly, num, den *big.Int, acc, dpow, tmp *big.Int) int {
	if len(p) == 0 {
		return 0
	}
	acc.Set(p[len(p)-1])
	dpow.SetInt64(1)
	for i := len(p) - 2; i >= 0; i-- {
		dpow.Mul(dpow, den)
		acc.Mul(acc, num)
		if p[i].Sign() != 0 {
			tmp.Mul(p[i], dpow)
			acc.Add(acc, tmp)
		}
	}
	return acc.Sign()
}

// SturmVarAt counts sign changes of the chain at x, reading each member's
// sign from cleared-denominator big.Int coefficients (IpSign) rather than a
// full big.Rat evaluation; the sign vector is identical either way
// (TestSturmVarAtMatchesRationalHornerSigns).
func SturmVarAt(chain SturmChainInt, x *big.Rat) int {
	acc, dpow, tmp := new(big.Int), new(big.Int), new(big.Int)
	num, den := x.Num(), x.Denom()
	vars, prev := 0, 0
	for _, p := range chain {
		s := IpSign(p, num, den, acc, dpow, tmp)
		if s == 0 {
			continue
		}
		if prev != 0 && s != prev {
			vars++
		}
		prev = s
	}
	return vars
}

// SturmCount counts distinct real roots in (lo, hi].
func SturmCount(chain SturmChainInt, lo, hi *big.Rat) int {
	return SturmVarAt(chain, lo) - SturmVarAt(chain, hi)
}

// RatIv is a rational interval.
type RatIv struct{ Lo, Hi *big.Rat }

// RpRootBound is a Cauchy bound: every real root lies in (-B, B).
func RpRootBound(p RatPoly) *big.Rat {
	p = RpTrim(p)
	b := new(big.Rat).SetInt64(1)
	lead := new(big.Rat).Abs(p[len(p)-1])
	tmp := new(big.Rat)
	for i := range len(p) - 1 {
		tmp.Abs(p[i])
		tmp.Quo(tmp, lead)
		if tmp.Cmp(b) > 0 {
			b.Set(tmp)
		}
	}
	return b.Add(b, new(big.Rat).SetInt64(1))
}

// rpIsolateRoots isolates every real root of a square-free polynomial into
// disjoint rational intervals, each holding exactly one root. chain is p's
// own Sturm chain (SturmChainIntContext(ctx, p)) — callers that also refine
// the isolated intervals build it once and pass it to both.
func RpIsolateRootsContext(ctx context.Context, p RatPoly, chain SturmChainInt) ([]RatIv, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	p = RpTrim(p)
	if RpDeg(p) < 1 {
		return nil, nil
	}
	bound := RpRootBound(p)
	var out []RatIv
	half := big.NewRat(1, 2)
	type job struct {
		iv    RatIv
		depth int
	}
	stack := []job{{iv: RatIv{Lo: new(big.Rat).Neg(bound), Hi: bound}}}
	work := 0
	for len(stack) > 0 {
		work++
		if work%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		j := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		n := SturmCount(chain, j.iv.Lo, j.iv.Hi)
		switch {
		case n == 0:
			continue
		case n == 1:
			out = append(out, j.iv)
		case j.depth > 256:
			// Unreachable for a square-free polynomial; the honest wide
			// interval stands rather than a wrong count.
			out = append(out, j.iv)
		default:
			mid := new(big.Rat).Add(j.iv.Lo, j.iv.Hi)
			mid.Mul(mid, half)
			stack = append(stack,
				job{iv: RatIv{Lo: j.iv.Lo, Hi: mid}, depth: j.depth + 1},
				job{iv: RatIv{Lo: mid, Hi: j.iv.Hi}, depth: j.depth + 1})
		}
	}
	return out, nil
}

// rpRefineRoot narrows an isolating interval by bisection until the mapped
// width predicate holds or the fixed depth budget runs out (clearance §5:
// deterministic, and on exhaustion the honest wide interval stands). varLo
// caches the variation count at the interval's low end — SturmCount(lo, mid)
// = SturmVarAt(lo) − SturmVarAt(mid), so each iteration needs only the new
// count at mid, not a fresh count at lo.
func RpRefineRootContext(ctx context.Context, chain SturmChainInt, iv RatIv, narrow func(lo, hi float64) bool) (RatIv, error) {
	half := big.NewRat(1, 2)
	varLo := -1
	for i := range 128 {
		if i%32 == 0 {
			if err := ctx.Err(); err != nil {
				return RatIv{}, err
			}
		}
		lo, _ := iv.Lo.Float64()
		hi, _ := iv.Hi.Float64()
		if narrow(lo, hi) {
			break
		}
		if varLo < 0 {
			varLo = SturmVarAt(chain, iv.Lo)
		}
		mid := new(big.Rat).Add(iv.Lo, iv.Hi)
		mid.Mul(mid, half)
		varMid := SturmVarAt(chain, mid)
		if varLo-varMid > 0 {
			iv = RatIv{Lo: iv.Lo, Hi: mid}
			continue
		}
		iv = RatIv{Lo: mid, Hi: iv.Hi}
		varLo = varMid
	}
	return iv, nil
}
