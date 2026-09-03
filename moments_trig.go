package decad

import "math/big"

// This file is moments.go's certified sine/cosine primitive: a proven
// enclosure of sin(2*pi*t) and cos(2*pi*t) for an exact rational TURN t,
// used by circularAreaInterval and circularFirstMomentInterval to bracket a
// CircleSeg fragment whose recorded range is not a whole number of turns —
// the case those two functions' whole-turn fast paths do not cover, and the
// only reason a trimmed circular walk ever fell back to
// conservativeValueError's body-scale envelope instead of a bracket
// proportional to the actual error.
//
// The construction works entirely in TURN space rather than angle space, in
// four steps, each its own proof obligation:
//
//  1. Period reduction, exact: f = t - floor(t) over rationals, f in [0, 1).
//     sin/cos of 2*pi*t are unchanged by this reduction.
//  2. Octant reduction, exact: k = floor(8f) (an integer division of
//     rationals), residual s = f - k/8 in [0, 1/8], complement
//     sc = 1/8 - s in [0, 1/8]. The reduction never touches pi — octant
//     boundaries are the rational turns k/8 — so no transcendental
//     comparison and no tolerance enters here.
//  3. First-octant evaluation: for a residual turn u in [0, 1/8] the angle
//     x = 2*pi*u lies in [0, pi/4]. x is enclosed on the fixed-point grid
//     2^-fixedBits by rounding 2*piLower*u down and 2*piUpper*u up
//     (fixedFloor/fixedCeil), and trigFixedSeries evaluates the alternating
//     Maclaurin series at each endpoint with an explicit truncation budget.
//     sin is increasing and cos is decreasing on [0, pi/4], so the interval's
//     own endpoints decide the enclosure.
//  4. Octant symmetry, exact: the first-octant pair is mapped back through
//     the eight sign/swap cases (turnSinCosInterval's own switch), using s
//     for even k and the complement sc for odd k — odd octants are measured
//     down from the next multiple of pi/2, so they are again a first-octant
//     evaluation.
//
// Arithmetic is fixed-point big.Int, not big.Rat: a big.Rat pays a GCD
// normalization per operation, which is what makes a naive rational series
// far slower at the 2000-bit numerators this scale needs. The fixed-point
// series is the same proof — every step is still an exact integer operation
// with an explicit, charged truncation (trigFixedSeries's own comment) — at a
// large constant-factor speedup, and TestTurnSinCosIntervalEnclosesMathSincos
// / BenchmarkTurnSinCosInterval are this file's own measurement of both the
// resulting enclosure width and the per-call cost, rather than a number
// asserted here: this primitive runs once or twice per fractional-turn
// CircleSeg endpoint, never in a per-segment or per-vertex loop, so it is not
// the cost bottleneck of the bracket it serves regardless of the exact
// figure.

// trigFixedBits is the fixed-point scale exponent P: the package's shared
// fixed-point grid, every value on it an integer count of 2^-trigFixedBits.
// This file's trigFixedSeries and moments.go's atanSmallInterval both
// evaluate their series on it.
const trigFixedBits = 200

// trigSeriesTerms is the number of alternating Maclaurin terms evaluated per
// endpoint. On [0, pi/4] the terms strictly decrease (x^2 <= (pi/4)^2 < 2
// bounds the ratio of consecutive terms below 1 from the first term on), so
// the first OMITTED term is a rigorous remainder bound (the standard
// alternating-series tail bound). trigFixedSeries computes that held-back
// term itself rather than trusting a hand-derived estimate of its size — see
// its own comment — but the term count still has to be large enough that the
// held-back term is actually small: at x = pi/4, x^(2n+1)/(2n+1)! first
// drops below 2^-trigFixedBits (~6.22e-61) between n=21 and n=22 (that
// crossing checked independently in Python at 53-bit float precision, ample
// for confirming an order of magnitude). 24 terms — first omitted degree
// 2*24+1 = 49 — clears it with several more orders of margin
// ((pi/4)^49/49! ~ 1.2e-68).
const trigSeriesTerms = 24

// trigFixedOne is 1 in fixed-point representation, i.e. 2^trigFixedBits.
var trigFixedOne = new(big.Int).Lsh(big.NewInt(1), trigFixedBits)

// fixedFloor returns floor(q * 2^trigFixedBits) as a big.Int. big.Int's
// QuoRem truncates toward zero, so a negative remainder (num and den have the
// same sign convention: den is always positive for a big.Rat, so the
// remainder's sign matches q's) means the truncated quotient overshot the
// true floor by one and is corrected down.
func fixedFloor(q *big.Rat) *big.Int {
	num := new(big.Int).Mul(q.Num(), trigFixedOne)
	quo, rem := new(big.Int), new(big.Int)
	quo.QuoRem(num, q.Denom(), rem)
	if rem.Sign() < 0 {
		quo.Sub(quo, big.NewInt(1))
	}
	return quo
}

// fixedCeil is fixedFloor's outward-rounding mirror.
func fixedCeil(q *big.Rat) *big.Int {
	num := new(big.Int).Mul(q.Num(), trigFixedOne)
	quo, rem := new(big.Int), new(big.Int)
	quo.QuoRem(num, q.Denom(), rem)
	if rem.Sign() > 0 {
		quo.Add(quo, big.NewInt(1))
	}
	return quo
}

// fixedToRat converts a fixed-point integer back to an exact big.Rat.
func fixedToRat(v *big.Int) *big.Rat {
	return new(big.Rat).SetFrac(v, trigFixedOne)
}

// trigFixedSeries evaluates fixed-point sin(x), cos(x) for X representing
// x*2^trigFixedBits with x in [0, pi/4], returning both together with an
// error budget in units of 2^-trigFixedBits that bounds |returned-true| from
// ABOVE for both results.
//
// The budget is not a hand-derived estimate of the series remainder — it is
// computed FROM the remainder itself. series returns, alongside its partial
// sum, the magnitude of the first OMITTED term: one further step of the same
// recurrence, carried out but never folded into the sum. Because term
// magnitudes strictly decrease on [0, pi/4] (trigSeriesTerms's own comment),
// the alternating-series remainder theorem bounds the true infinite tail of
// EITHER series by that one held-back term — a fact about THIS computation's
// own inputs, not an assumption about how small it is.
//
// The held-back term is itself only approximate — it passed through the same
// truncating steps as every summed term — so it needs its own margin, and so
// does the partial sum. Both are bounded by the SAME per-step accounting.
// Every big.Int operation is exact except: x2's one Rsh (a fixed <1-unit
// error shared by every step, since x2 is computed once and reused) and,
// per step, one Rsh (the product's fixed-point rescale) and one Quo (the
// exact-integer divisor) — two more <1-unit truncations. Tracking e_k, the
// worst-case error (in units of 2^-trigFixedBits) in term index k: e_0 = 0
// (term 0 is the untruncated input, x or 1), and since every value in this
// series — x, x2, and every term — has magnitude <=1, multiplying a
// <=1-magnitude quantity with error e by another with its own error adds at
// most that error (not a multiple of it): e_(k+1) <= e_k*1 (x2's <=1
// magnitude carries term_k's error through) + 1*e_x2 (term_k's <=1 magnitude
// carries x2's fixed error through, e_x2<=1) + 1 (the Rsh's own new
// truncation) + 1 (the Quo's own new truncation) = e_k + 3. So e_k <= 3k.
//
// The partial sum folds in terms 0..trigSeriesTerms-1 through exact
// Add/Sub, so its own error is at most their sum: sum_(k=0)^(n-1) 3k =
// 3n(n-1)/2. The held-back term (index n = trigSeriesTerms) carries error
// e_n <= 3n on top of its own computed magnitude. Both figures are folded
// into one margin, 3n(n-1)/2 + 3n = 3n(n+1)/2, added to the held-back term's
// own value — the single quantity that actually dominates this budget, being
// many orders above the margin (trigSeriesTerms's own comment).
func trigFixedSeries(x *big.Int) (sin, cos, errUnits *big.Int) {
	x2 := new(big.Int).Mul(x, x)
	x2.Rsh(x2, trigFixedBits) // one truncation, one unit low

	// series evaluates the alternating Maclaurin sum starting at `first`
	// (x for sin, 1 for cos) with successive terms divided by k*(k+1) —
	// (2)(3), (4)(5), … for sin; (1)(2), (3)(4), … for cos — which is the
	// fixed-point form of dividing by the next two factorial steps. Every
	// iteration updates `term` to the NEXT term after folding the current one
	// into the sum, so once the loop exits (trigSeriesTerms terms summed,
	// indices 0..trigSeriesTerms-1) the variable `term` already holds term
	// index trigSeriesTerms — the first OMITTED one — which is returned
	// alongside the sum as the series' own remainder proof.
	series := func(first *big.Int, k0 int64) (sum, remainder *big.Int) {
		sum = new(big.Int)
		term := new(big.Int).Set(first)
		k := k0
		for n := range trigSeriesTerms {
			if n%2 == 0 {
				sum.Add(sum, term)
			} else {
				sum.Sub(sum, term)
			}
			term.Mul(term, x2)
			term.Rsh(term, trigFixedBits)
			term.Quo(term, big.NewInt(k*(k+1)))
			k += 2
		}
		return sum, term
	}
	var sinRem, cosRem *big.Int
	sin, sinRem = series(new(big.Int).Set(x), 2)
	cos, cosRem = series(new(big.Int).Set(trigFixedOne), 1)

	rem := sinRem
	if cosRem.CmpAbs(rem) > 0 {
		rem = cosRem
	}
	const n = trigSeriesTerms
	margin := big.NewInt(3 * n * (n + 1) / 2)
	errUnits = new(big.Int).Add(new(big.Int).Abs(rem), margin)
	return sin, cos, errUnits
}

// turnFirstOctantSinCos brackets sin(2*pi*u), cos(2*pi*u) for a rational turn
// u in [0, 1/8] (angle in [0, pi/4]), by evaluating trigFixedSeries at each
// outward-rounded endpoint of the enclosed angle and taking the monotone
// endpoints: sin increasing, cos decreasing on [0, pi/4].
func turnFirstOctantSinCos(u *big.Rat) (ratInterval, ratInterval) {
	twoU := new(big.Rat).Mul(big.NewRat(2, 1), u)
	lo := fixedFloor(new(big.Rat).Mul(piLower, twoU))
	hi := fixedCeil(new(big.Rat).Mul(piUpper, twoU))
	if lo.Sign() < 0 {
		lo = new(big.Int)
	}
	sLo, cLo, eLo := trigFixedSeries(lo)
	sHi, cHi, eHi := trigFixedSeries(hi)
	e := eLo
	if eHi.Cmp(e) > 0 {
		e = eHi
	}
	sinIv := interval(fixedToRat(new(big.Int).Sub(sLo, e)), fixedToRat(new(big.Int).Add(sHi, e)))
	cosIv := interval(fixedToRat(new(big.Int).Sub(cHi, e)), fixedToRat(new(big.Int).Add(cLo, e)))
	return sinIv, cosIv
}

// turnSinCosInterval returns proven enclosures of sin(2*pi*t) and
// cos(2*pi*t) for an exact rational turn t: period reduction into [0, 1),
// exact octant reduction (step 2 of the file comment), a first-octant
// evaluation of the residual and its complement, and the octant symmetry
// table below. Interval arithmetic is inclusion-monotonic throughout, so the
// returned intervals enclose the true values whatever the platform's own
// libm does — nothing here depends on Go's Sin/Cos.
func turnSinCosInterval(t *big.Rat) (ratInterval, ratInterval) {
	f := new(big.Rat).Set(t)
	fl := new(big.Int).Quo(f.Num(), f.Denom())
	if new(big.Rat).SetInt(fl).Cmp(f) > 0 {
		fl.Sub(fl, big.NewInt(1))
	}
	f.Sub(f, new(big.Rat).SetInt(fl))

	eight := big.NewRat(8, 1)
	scaled := new(big.Rat).Mul(f, eight)
	kBig := new(big.Int).Quo(scaled.Num(), scaled.Denom())
	k := kBig.Int64()
	s := new(big.Rat).Sub(f, new(big.Rat).SetFrac64(k, 8))
	sc := new(big.Rat).Sub(big.NewRat(1, 8), s)

	// Even octants read the residual's own pair, odd octants the complement's;
	// only the pair the switch below returns is evaluated.
	var sn, cs, snC, csC ratInterval
	if k%2 == 0 {
		sn, cs = turnFirstOctantSinCos(s)
	} else {
		snC, csC = turnFirstOctantSinCos(sc)
	}
	// Octant symmetry: within octant k the true angle is k*pi/4 + x for even
	// k (x = 2*pi*s) or (k+1)*pi/4 - x' for odd k (x' = 2*pi*sc), so odd
	// octants read off the COMPLEMENT pair with sin/cos swapped.
	switch k {
	case 0:
		return sn, cs
	case 1:
		return csC, snC
	case 2:
		return cs, intervalNeg(sn)
	case 3:
		return snC, intervalNeg(csC)
	case 4:
		return intervalNeg(sn), intervalNeg(cs)
	case 5:
		return intervalNeg(csC), intervalNeg(snC)
	case 6:
		return intervalNeg(cs), sn
	default: // 7
		return intervalNeg(snC), csC
	}
}
