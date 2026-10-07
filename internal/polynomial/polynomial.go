package polynomial

import (
	"context"
	"math"
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
