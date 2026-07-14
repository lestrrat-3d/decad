package decad

import (
	"math"
	"math/big"
	"slices"
)

// This file is the certified-bracket machinery of docs/clearance-design.md
// §4/§5: the P4/P8 cells' stationarity polynomials and the torus ray-cast
// quartic are isolated by Sturm sequences over math/big.Rat on the
// evaluator's own float coefficients taken exactly, so every real root lands
// in an interval that cannot lie, and a bracket on a critical VALUE follows
// from the isolated parameter interval plus a proven Lipschitz bound — the
// same adaptive-exactness discipline as the boolean's sign tests
// (evaluator §9). A certified bracket is a proof, not a hope.

// ratPoly is a dense univariate polynomial over big.Rat; index i carries the
// coefficient of x^i.
type ratPoly []*big.Rat

func ratOf(f float64) *big.Rat {
	r := new(big.Rat)
	if r.SetFloat64(f) == nil {
		// A non-finite coefficient cannot happen for the finite geometry the
		// evaluator holds; a zero keeps the polynomial well-formed anyway.
		return new(big.Rat)
	}
	return r
}

func rpTrim(p ratPoly) ratPoly {
	n := len(p)
	for n > 0 && p[n-1].Sign() == 0 {
		n--
	}
	return p[:n]
}

func rpDeg(p ratPoly) int { return len(rpTrim(p)) - 1 }

func rpAdd(a, b ratPoly) ratPoly {
	n := max(len(a), len(b))
	out := make(ratPoly, n)
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

func rpNeg(a ratPoly) ratPoly {
	out := make(ratPoly, len(a))
	for i := range a {
		out[i] = new(big.Rat).Neg(a[i])
	}
	return out
}

func rpSub(a, b ratPoly) ratPoly { return rpAdd(a, rpNeg(b)) }

func rpMul(a, b ratPoly) ratPoly {
	a, b = rpTrim(a), rpTrim(b)
	if len(a) == 0 || len(b) == 0 {
		return ratPoly{}
	}
	out := make(ratPoly, len(a)+len(b)-1)
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

func rpScale(a ratPoly, s *big.Rat) ratPoly {
	out := make(ratPoly, len(a))
	for i := range a {
		out[i] = new(big.Rat).Mul(a[i], s)
	}
	return out
}

func rpDeriv(a ratPoly) ratPoly {
	if len(a) <= 1 {
		return ratPoly{}
	}
	out := make(ratPoly, len(a)-1)
	for i := 1; i < len(a); i++ {
		out[i-1] = new(big.Rat).Mul(a[i], new(big.Rat).SetInt64(int64(i)))
	}
	return out
}

func rpEval(a ratPoly, x *big.Rat) *big.Rat {
	out := new(big.Rat)
	for _, c := range slices.Backward(a) {
		out.Mul(out, x)
		out.Add(out, c)
	}
	return out
}

// rpRem is the polynomial remainder of a by b (deg b >= 0).
func rpRem(a, b ratPoly) ratPoly {
	a, b = rpTrim(a), rpTrim(b)
	if len(b) == 0 {
		return a
	}
	rem := make(ratPoly, len(a))
	for i := range a {
		rem[i] = new(big.Rat).Set(a[i])
	}
	lead := b[len(b)-1]
	tmp := new(big.Rat)
	for len(rem) >= len(b) {
		rem = rpTrim(rem)
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
	return rpTrim(rem)
}

// rpNormalize divides by the leading coefficient's absolute value, taming
// coefficient blowup in the Euclidean sequences without moving any root.
func rpNormalize(a ratPoly) ratPoly {
	a = rpTrim(a)
	if len(a) == 0 {
		return a
	}
	lead := new(big.Rat).Abs(a[len(a)-1])
	inv := new(big.Rat).Inv(lead)
	return rpScale(a, inv)
}

// rpGCD is the monic polynomial GCD.
func rpGCD(a, b ratPoly) ratPoly {
	a, b = rpTrim(a), rpTrim(b)
	for len(b) > 0 {
		a, b = b, rpNormalize(rpRem(a, b))
	}
	return a
}

// rpSquareFree strips repeated roots: p / gcd(p, p').
func rpSquareFree(p ratPoly) ratPoly {
	p = rpTrim(p)
	if len(p) <= 2 {
		return p
	}
	g := rpGCD(p, rpDeriv(p))
	if rpDeg(g) < 1 {
		return p
	}
	return rpQuo(p, g)
}

// rpQuo is exact polynomial division (remainder known zero).
func rpQuo(a, b ratPoly) ratPoly {
	a, b = rpTrim(a), rpTrim(b)
	if len(b) == 0 || len(a) < len(b) {
		return ratPoly{}
	}
	out := make(ratPoly, len(a)-len(b)+1)
	rem := make(ratPoly, len(a))
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
	return rpTrim(out)
}

// sturmChain builds the Sturm sequence of a square-free polynomial.
func sturmChain(p ratPoly) []ratPoly {
	chain := []ratPoly{rpTrim(p), rpTrim(rpDeriv(p))}
	for {
		last := chain[len(chain)-1]
		if len(last) == 0 {
			return chain[:len(chain)-1]
		}
		if rpDeg(last) == 0 {
			return chain
		}
		rem := rpRem(chain[len(chain)-2], last)
		chain = append(chain, rpNormalize(rpNeg(rem)))
	}
}

// sturmVarAt counts sign changes of the chain at x.
func sturmVarAt(chain []ratPoly, x *big.Rat) int {
	vars, prev := 0, 0
	for _, p := range chain {
		s := rpEval(p, x).Sign()
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

// sturmCount counts distinct real roots in (lo, hi].
func sturmCount(chain []ratPoly, lo, hi *big.Rat) int {
	return sturmVarAt(chain, lo) - sturmVarAt(chain, hi)
}

// ratIv is a rational interval.
type ratIv struct{ lo, hi *big.Rat }

// rpRootBound is a Cauchy bound: every real root lies in (-B, B).
func rpRootBound(p ratPoly) *big.Rat {
	p = rpTrim(p)
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
// disjoint rational intervals, each holding exactly one root.
func rpIsolateRoots(p ratPoly) []ratIv {
	p = rpTrim(p)
	if rpDeg(p) < 1 {
		return nil
	}
	chain := sturmChain(p)
	bound := rpRootBound(p)
	var out []ratIv
	half := big.NewRat(1, 2)
	type job struct {
		iv    ratIv
		depth int
	}
	stack := []job{{iv: ratIv{lo: new(big.Rat).Neg(bound), hi: bound}}}
	for len(stack) > 0 {
		j := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		n := sturmCount(chain, j.iv.lo, j.iv.hi)
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
			mid := new(big.Rat).Add(j.iv.lo, j.iv.hi)
			mid.Mul(mid, half)
			stack = append(stack,
				job{iv: ratIv{lo: j.iv.lo, hi: mid}, depth: j.depth + 1},
				job{iv: ratIv{lo: mid, hi: j.iv.hi}, depth: j.depth + 1})
		}
	}
	return out
}

// rpRefineRoot narrows an isolating interval by bisection until the mapped
// width predicate holds or the fixed depth budget runs out (clearance §5:
// deterministic, and on exhaustion the honest wide interval stands).
func rpRefineRoot(chain []ratPoly, iv ratIv, narrow func(lo, hi float64) bool) ratIv {
	half := big.NewRat(1, 2)
	for range 128 {
		lo, _ := iv.lo.Float64()
		hi, _ := iv.hi.Float64()
		if narrow(lo, hi) {
			break
		}
		mid := new(big.Rat).Add(iv.lo, iv.hi)
		mid.Mul(mid, half)
		if sturmCount(chain, iv.lo, mid) > 0 {
			iv = ratIv{lo: iv.lo, hi: mid}
			continue
		}
		iv = ratIv{lo: mid, hi: iv.hi}
	}
	return iv
}

// csPoly is a trigonometric polynomial in the canonical form A(c) + s·B(c)
// with c = cosθ, s = sinθ (every s² reduced to 1 − c²): closed under
// products and d/dθ, and convertible to a plain polynomial in t = tan(θ/2).
type csPoly struct{ a, b ratPoly }

func csConst(f float64) csPoly { return csPoly{a: ratPoly{ratOf(f)}} }

// csLin builds k0 + kc·cosθ + ks·sinθ from float coefficients taken exactly.
func csLin(k0, kc, ks float64) csPoly {
	return csPoly{a: ratPoly{ratOf(k0), ratOf(kc)}, b: ratPoly{ratOf(ks)}}
}

func csAdd(x, y csPoly) csPoly { return csPoly{a: rpAdd(x.a, y.a), b: rpAdd(x.b, y.b)} }

func csSub(x, y csPoly) csPoly { return csPoly{a: rpSub(x.a, y.a), b: rpSub(x.b, y.b)} }

// csMul multiplies with the s² = 1 − c² reduction.
func csMul(x, y csPoly) csPoly {
	oneMinusC2 := ratPoly{big.NewRat(1, 1), new(big.Rat), big.NewRat(-1, 1)}
	a := rpAdd(rpMul(x.a, y.a), rpMul(oneMinusC2, rpMul(x.b, y.b)))
	b := rpAdd(rpMul(x.a, y.b), rpMul(x.b, y.a))
	return csPoly{a: a, b: b}
}

// csDerivTheta is d/dθ: (A + sB)' = [cB − (1−c²)B'] − s·A'.
func csDerivTheta(x csPoly) csPoly {
	oneMinusC2 := ratPoly{big.NewRat(1, 1), new(big.Rat), big.NewRat(-1, 1)}
	cTimesB := rpMul(ratPoly{new(big.Rat), big.NewRat(1, 1)}, x.b)
	a := rpSub(cTimesB, rpMul(oneMinusC2, rpDeriv(x.b)))
	return csPoly{a: a, b: rpNeg(rpDeriv(x.a))}
}

func csIsZero(x csPoly) bool {
	return len(rpTrim(x.a)) == 0 && len(rpTrim(x.b)) == 0
}

// omPow is (1 − c²)^k.
func omPow(k int) ratPoly {
	out := ratPoly{big.NewRat(1, 1)}
	base := ratPoly{big.NewRat(1, 1), new(big.Rat), big.NewRat(-1, 1)}
	for range k {
		out = rpMul(out, base)
	}
	return out
}

// csQuarterShift maps θ → θ + π/2 exactly: cos → −sin, sin → cos.
func csQuarterShift(x csPoly) csPoly {
	var a, b ratPoly
	cPoly := ratPoly{new(big.Rat), big.NewRat(1, 1)}
	for i, ai := range x.a {
		if ai.Sign() == 0 {
			continue
		}
		if i%2 == 0 {
			a = rpAdd(a, rpScale(omPow(i/2), ai))
			continue
		}
		neg := new(big.Rat).Neg(ai)
		b = rpAdd(b, rpScale(omPow((i-1)/2), neg))
	}
	for j, bj := range x.b {
		if bj.Sign() == 0 {
			continue
		}
		if j%2 == 0 {
			a = rpAdd(a, rpMul(cPoly, rpScale(omPow(j/2), bj)))
			continue
		}
		neg := new(big.Rat).Neg(bj)
		b = rpAdd(b, rpMul(cPoly, rpScale(omPow((j-1)/2), neg)))
	}
	return csPoly{a: a, b: b}
}

// csToT substitutes the half-angle chart t = tan(θ/2), clearing the
// (1 + t²)^K denominator: root sets over θ ∈ (−π, π) map bijectively.
func csToT(x csPoly) ratPoly {
	a, b := rpTrim(x.a), rpTrim(x.b)
	k := max(len(a)-1, len(b))
	num := ratPoly{big.NewRat(1, 1), new(big.Rat), big.NewRat(-1, 1)} // 1 − t²
	den := ratPoly{big.NewRat(1, 1), new(big.Rat), big.NewRat(1, 1)}  // 1 + t²
	twoT := ratPoly{new(big.Rat), big.NewRat(2, 1)}
	powNum := func(n int) ratPoly {
		out := ratPoly{big.NewRat(1, 1)}
		for range n {
			out = rpMul(out, num)
		}
		return out
	}
	powDen := func(n int) ratPoly {
		out := ratPoly{big.NewRat(1, 1)}
		for range n {
			out = rpMul(out, den)
		}
		return out
	}
	var out ratPoly
	for i, ai := range a {
		if ai.Sign() == 0 {
			continue
		}
		out = rpAdd(out, rpScale(rpMul(powNum(i), powDen(k-i)), ai))
	}
	for j, bj := range b {
		if bj.Sign() == 0 {
			continue
		}
		term := rpMul(twoT, rpMul(powNum(j), powDen(k-1-j)))
		out = rpAdd(out, rpScale(term, bj))
	}
	return rpTrim(out)
}

// critBracket is one certified enclosure of a stationary point: the critical
// parameter lies in [thLo, thHi], and the objective's value there in
// [lo, hi].
type critBracket struct {
	thLo, thHi float64
	lo, hi     float64
}

func (b critBracket) mid() float64 { return (b.thLo + b.thHi) / 2 }

// trigStationaryBrackets isolates every zero of the stationarity polynomial
// f over the full circle and returns certified enclosures of the objective g
// at each of them: g is 1-Lipschitz-composed with a parameterization whose
// speed is at most lip, and slack absorbs floating evaluation noise. The
// second result is false when f is identically zero — a constant objective,
// the caller's closed-form path.
func trigStationaryBrackets(f csPoly, g func(float64) float64, lip, slack float64) ([]critBracket, bool) {
	if csIsZero(f) {
		return nil, false
	}
	var out []critBracket
	// Two quarter-turn charts cover the circle: t = tan(θ/2) misses θ = ±π,
	// which the shifted chart holds interior.
	for _, chart := range []struct {
		shift float64
		poly  csPoly
	}{{shift: 0, poly: f}, {shift: math.Pi / 2, poly: csQuarterShift(f)}} {
		p := rpSquareFree(rpTrim(csToT(chart.poly)))
		if rpDeg(p) < 1 {
			continue
		}
		chain := sturmChain(p)
		for _, iv := range rpIsolateRoots(p) {
			iv = rpRefineRoot(chain, iv, func(lo, hi float64) bool {
				return 2*math.Atan(hi)-2*math.Atan(lo) <= 1e-11
			})
			tLo, _ := iv.lo.Float64()
			tHi, _ := iv.hi.Float64()
			thLo := chart.shift + 2*math.Atan(tLo)
			thHi := chart.shift + 2*math.Atan(tHi)
			v := g((thLo + thHi) / 2)
			half := lip*(thHi-thLo)/2 + slack
			out = append(out, critBracket{thLo: thLo, thHi: thHi, lo: v - half, hi: v + half})
		}
	}
	return out, true
}

// circleParam is an evaluated circle parameterization: center + r(u·cosθ +
// v·sinθ) with u, v unit and orthogonal.
type circleParam struct {
	c, u, v [3]float64
	r       float64
}

func (cp circleParam) at(th float64) [3]float64 {
	s, c := math.Sincos(th)
	var out [3]float64
	for i := range out {
		out[i] = cp.c[i] + cp.r*(cp.u[i]*c+cp.v[i]*s)
	}
	return out
}

// lineCircleBrackets encloses every critical value of the distance from the
// circle to the infinite line (a, d̂): the P4 cell. Returns the brackets, or
// constant=true when the distance is constant over the circle.
func lineCircleBrackets(cp circleParam, a, d [3]float64, slack float64) ([]critBracket, bool) {
	m := [3]float64{cp.c[0] - a[0], cp.c[1] - a[1], cp.c[2] - a[2]}
	dot := func(x, y [3]float64) float64 { return x[0]*y[0] + x[1]*y[1] + x[2]*y[2] }
	// D(θ) = |Q − a|² − ((Q − a)·d̂)², a degree-2 trig polynomial.
	base := csAdd(csConst(dot(m, m)+cp.r*cp.r), csLin(0, 2*cp.r*dot(m, cp.u), 2*cp.r*dot(m, cp.v)))
	t := csLin(dot(m, d), cp.r*dot(cp.u, d), cp.r*dot(cp.v, d))
	dist2 := csSub(base, csMul(t, t))
	g := func(th float64) float64 {
		q := cp.at(th)
		rel := [3]float64{q[0] - a[0], q[1] - a[1], q[2] - a[2]}
		along := dot(rel, d)
		return math.Sqrt(math.Max(0, dot(rel, rel)-along*along))
	}
	return trigStationaryBrackets(csDerivTheta(dist2), g, cp.r, slack)
}

// circleCircleBrackets encloses every critical value of the distance from
// circle 1 to circle 2 (unit normal n2): the P8 cell. The squared
// stationarity s·(w')² = r2²·(s')² covers every smooth critical point;
// callers guard the ρ = 0 kink separately (a circle meeting the other's
// axis), where the distance is not differentiable.
func circleCircleBrackets(c1 circleParam, c2 circleParam, n2 [3]float64, slack float64) ([]critBracket, bool) {
	m := [3]float64{c1.c[0] - c2.c[0], c1.c[1] - c2.c[1], c1.c[2] - c2.c[2]}
	dot := func(x, y [3]float64) float64 { return x[0]*y[0] + x[1]*y[1] + x[2]*y[2] }
	w := csAdd(csConst(dot(m, m)+c1.r*c1.r), csLin(0, 2*c1.r*dot(m, c1.u), 2*c1.r*dot(m, c1.v)))
	h := csLin(dot(m, n2), c1.r*dot(c1.u, n2), c1.r*dot(c1.v, n2))
	sp := csSub(w, csMul(h, h))
	wd := csDerivTheta(w)
	sd := csDerivTheta(sp)
	f := csSub(csMul(sp, csMul(wd, wd)), csMul(csConst(c2.r*c2.r), csMul(sd, sd)))
	g := func(th float64) float64 {
		p := c1.at(th)
		rel := [3]float64{p[0] - c2.c[0], p[1] - c2.c[1], p[2] - c2.c[2]}
		hh := dot(rel, n2)
		rho := math.Sqrt(math.Max(0, dot(rel, rel)-hh*hh))
		return math.Hypot(hh, rho-c2.r)
	}
	return trigStationaryBrackets(f, g, c1.r, slack)
}
