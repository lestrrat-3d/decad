package decad

import "math/big"

// Exact Bernstein arithmetic over one shared integer denominator. Every
// function here returns the same rational as its big.Rat formulation; the
// common denominator only removes the per-operation GCD of big.Rat.norm.

// spiralIntegerRow returns fresh integer numerators over the positive least
// common denominator of a. nums[i]/den == a[i] exactly.
func spiralIntegerRow(a []*big.Rat) ([]*big.Int, *big.Int) {
	den := big.NewInt(1)
	var gcd big.Int
	for _, c := range a {
		d := c.Denom()
		gcd.GCD(nil, nil, den, d)
		den.Quo(den, &gcd)
		den.Mul(den, d)
	}
	nums := make([]*big.Int, len(a))
	var scale big.Int
	for i, c := range a {
		scale.Quo(den, c.Denom())
		nums[i] = new(big.Int).Mul(c.Num(), &scale)
	}
	return nums, den
}

// spiralBigBinomials returns C(n,0..n) as exact integers.
func spiralBigBinomials(n int) []*big.Int {
	out := make([]*big.Int, n+1)
	out[0] = big.NewInt(1)
	for i := 1; i <= n; i++ {
		out[i] = new(big.Int).Mul(out[i-1], big.NewInt(int64(n-i+1)))
		out[i].Quo(out[i], big.NewInt(int64(i)))
	}
	return out
}

// spiralScaledBernstein folds the binomial weights into integer numerators:
// scaled[i] = nums[i]*C(n,i), so that
// Σ scaled[i] p^i (q-p)^(n-i) / (den q^n) is the Bernstein value at p/q.
func spiralScaledBernstein(coeff []*big.Rat) ([]*big.Int, *big.Int) {
	nums, den := spiralIntegerRow(coeff)
	for i, binom := range spiralBigBinomials(len(coeff) - 1) {
		nums[i].Mul(nums[i], binom)
	}
	return nums, den
}

// spiralScaledNumerator evaluates Σ scaled[i] p^i (q-p)^(n-i).
func spiralScaledNumerator(scaled []*big.Int, p, q *big.Int) *big.Int {
	n := len(scaled) - 1
	qp := new(big.Int).Sub(q, p)
	pPow := make([]*big.Int, n+1)
	qpPow := make([]*big.Int, n+1)
	pPow[0], qpPow[0] = big.NewInt(1), big.NewInt(1)
	for i := 1; i <= n; i++ {
		pPow[i] = new(big.Int).Mul(pPow[i-1], p)
		qpPow[i] = new(big.Int).Mul(qpPow[i-1], qp)
	}
	sum := new(big.Int)
	var term big.Int
	for i, c := range scaled {
		term.Mul(c, pPow[i])
		term.Mul(&term, qpPow[n-i])
		sum.Add(sum, &term)
	}
	return sum
}

func spiralBernValue(coeff []*big.Rat, parameter *big.Rat) *big.Rat {
	scaled, den := spiralScaledBernstein(coeff)
	p, q := parameter.Num(), parameter.Denom()
	num := spiralScaledNumerator(scaled, p, q)
	qn := new(big.Int).Exp(q, big.NewInt(int64(len(coeff)-1)), nil)
	return new(big.Rat).SetFrac(num, qn.Mul(qn, den))
}

// spiralBernSign is spiralBernValue(coeff, parameter).Sign() without the
// final normalisation. den and q^n are positive, so the numerator's sign is
// the value's sign.
func spiralBernSign(coeff []*big.Rat, parameter *big.Rat) int {
	scaled, _ := spiralScaledBernstein(coeff)
	return spiralScaledNumerator(scaled, parameter.Num(), parameter.Denom()).Sign()
}

func spiralIsolateRootN(coeff []*big.Rat, steps int) spiralRootCrossing {
	scaled, _ := spiralScaledBernstein(coeff)
	// lo = a/2^s, hi = (a+1)/2^s; the midpoint (2a+1)/2^(s+1) is the same
	// dyadic rational the big.Rat bisection forms.
	a := new(big.Int)
	q := big.NewInt(1)
	mid := new(big.Int)
	for range steps {
		a.Lsh(a, 1)
		q.Lsh(q, 1)
		mid.Add(a, big.NewInt(1))
		if spiralScaledNumerator(scaled, mid, q).Sign() < 0 {
			a.Set(mid)
		}
	}
	lo := new(big.Rat).SetFrac(a, q)
	hi := new(big.Rat).SetFrac(new(big.Int).Add(a, big.NewInt(1)), q)
	return spiralRootCrossing{span: 0, lo: lo, hi: hi}
}

// spiralBernSplit is de Casteljau subdivision at p/q over integers. After
// level ℓ updates every entry is scaled by den*q^ℓ.
func spiralBernSplit(a []*big.Rat, at *big.Rat) ([]*big.Rat, []*big.Rat) {
	n := len(a) - 1
	left, right := make([]*big.Rat, n+1), make([]*big.Rat, n+1)
	work, den := spiralIntegerRow(a)
	p, q := at.Num(), at.Denom()
	qp := new(big.Int).Sub(q, p)
	scale := new(big.Int).Set(den)
	var t1, t2 big.Int
	for level := 0; level <= n; level++ {
		left[level] = new(big.Rat).SetFrac(work[0], scale)
		right[n-level] = new(big.Rat).SetFrac(work[n-level], scale)
		for i := range n - level {
			t1.Mul(qp, work[i])
			t2.Mul(p, work[i+1])
			work[i].Add(&t1, &t2)
		}
		scale.Mul(scale, q)
	}
	return left, right
}

// spiralMul multiplies in the scaled Bernstein basis, where the product is a
// plain convolution: c̃_k = Σ ã_i b̃_j with ã_i = C(n,i) a_i, and the
// Bernstein coefficient is c̃_k / C(n+m,k).
func spiralMul(a, b []*big.Rat) []*big.Rat {
	n, m := len(a)-1, len(b)-1
	an, ad := spiralScaledBernstein(a)
	bn, bd := spiralScaledBernstein(b)
	den := new(big.Int).Mul(ad, bd)
	binom := spiralBigBinomials(n + m)
	out := make([]*big.Rat, n+m+1)
	var term, outDen big.Int
	for k := range out {
		sum := new(big.Int)
		for i := max(0, k-m); i <= min(n, k); i++ {
			term.Mul(an[i], bn[k-i])
			sum.Add(sum, &term)
		}
		outDen.Mul(den, binom[k])
		out[k] = new(big.Rat).SetFrac(sum, &outDen)
	}
	return out
}
