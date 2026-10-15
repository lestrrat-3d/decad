package polynomial

import (
	"context"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// Exact Bernstein arithmetic uses one shared integer denominator. The
// common denominator avoids normalizing a big.Rat after each operation.

// bernsteinUnit is the read-only interval end 1. It is never written.
var bernsteinUnit = big.NewRat(1, 1)

func ratOne() *big.Rat              { return big.NewRat(1, 1) }
func ratAdd(a, b *big.Rat) *big.Rat { return proofarith.AddRat(new(big.Rat), a, b) }
func ratSub(a, b *big.Rat) *big.Rat { return proofarith.SubRat(new(big.Rat), a, b) }
func ratMul(a, b *big.Rat) *big.Rat { return proofarith.MulRat(new(big.Rat), a, b) }
func ratDiv(a, b *big.Rat) *big.Rat { return proofarith.QuoRat(new(big.Rat), a, b) }

// bernsteinIntegerRow returns fresh numerators over their positive least
// common denominator. nums[i]/den equals coeff[i] exactly.
func bernsteinIntegerRow(coeff []*big.Rat) ([]*big.Int, *big.Int) {
	den := big.NewInt(1)
	var gcd big.Int
	for _, c := range coeff {
		gcd.GCD(nil, nil, den, c.Denom())
		den.Quo(den, &gcd)
		den.Mul(den, c.Denom())
	}
	nums := make([]*big.Int, len(coeff))
	var scale big.Int
	for i, c := range coeff {
		scale.Quo(den, c.Denom())
		nums[i] = new(big.Int).Mul(c.Num(), &scale)
	}
	return nums, den
}

// bernsteinBinomialTable holds the binomial rows for n in 0..32. Read only.
// These are constants of n alone, computed once at package initialisation;
// no caller may write into a returned big.Int.
var bernsteinBinomialTable = func() [][]*big.Int {
	table := make([][]*big.Int, 33)
	for n := range table {
		table[n] = bernsteinBinomialsFresh(n)
	}
	return table
}()

// bernsteinBinomials returns the binomial row C(n, 0..n). The result may be a
// shared table row: callers must only read it.
func bernsteinBinomials(n int) []*big.Int {
	if n >= 0 && n < len(bernsteinBinomialTable) {
		return bernsteinBinomialTable[n]
	}
	return bernsteinBinomialsFresh(n)
}

func bernsteinBinomialsFresh(n int) []*big.Int {
	out := make([]*big.Int, n+1)
	out[0] = big.NewInt(1)
	for i := 1; i <= n; i++ {
		out[i] = new(big.Int).Mul(out[i-1], big.NewInt(int64(n-i+1)))
		out[i].Quo(out[i], big.NewInt(int64(i)))
	}
	return out
}

// bernsteinScaledCoefficients folds the binomial weights into integer
// numerators, so a value at p/q has numerator Σ scaled[i] p^i (q-p)^(n-i).
func bernsteinScaledCoefficients(coeff []*big.Rat) ([]*big.Int, *big.Int) {
	nums, den := bernsteinIntegerRow(coeff)
	for i, binom := range bernsteinBinomials(len(coeff) - 1) {
		nums[i].Mul(nums[i], binom)
	}
	return nums, den
}

func bernsteinScaledNumerator(scaled []*big.Int, p, q *big.Int) *big.Int {
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

// BernsteinSign is the sign of a Bernstein row at a rational parameter.
// The omitted denominator is positive, so the integer numerator has the
// same sign as the exact value.
func BernsteinSign(coeff []*big.Rat, parameter *big.Rat) int {
	scaled, _ := bernsteinScaledCoefficients(coeff)
	return bernsteinScaledNumerator(scaled, new(big.Int).Set(parameter.Num()), new(big.Int).Set(parameter.Denom())).Sign()
}

// BernsteinValue is the exact value of a Bernstein row at a rational
// parameter.
func BernsteinValue(coeff []*big.Rat, parameter *big.Rat) *big.Rat {
	scaled, den := bernsteinScaledCoefficients(coeff)
	p, q := new(big.Int).Set(parameter.Num()), new(big.Int).Set(parameter.Denom())
	num := bernsteinScaledNumerator(scaled, p, q)
	qn := new(big.Int).Exp(q, big.NewInt(int64(len(coeff)-1)), nil)
	return proofarith.RatFrac(new(big.Rat), num, qn.Mul(qn, den))
}

// BernsteinIsolateRoot bisects the one sign change of a Bernstein row steps
// times and returns the dyadic bracket lo, hi.
func BernsteinIsolateRoot(coeff []*big.Rat, steps int) (lo, hi *big.Rat) {
	scaled, _ := bernsteinScaledCoefficients(coeff)
	// lo = a/2^s, hi = (a+1)/2^s; the midpoint (2a+1)/2^(s+1) is the same
	// dyadic rational the big.Rat bisection forms.
	a := new(big.Int)
	q := big.NewInt(1)
	mid := new(big.Int)
	for range steps {
		a.Lsh(a, 1)
		q.Lsh(q, 1)
		mid.Add(a, big.NewInt(1))
		if bernsteinScaledNumerator(scaled, mid, q).Sign() < 0 {
			a.Set(mid)
		}
	}
	lo = proofarith.RatFrac(new(big.Rat), a, q)
	hi = proofarith.RatFrac(new(big.Rat), new(big.Int).Add(a, big.NewInt(1)), q)
	return lo, hi
}

// BernsteinSplit is de Casteljau subdivision at p/q over integers. After
// level ℓ updates every entry is scaled by den*q^ℓ.
func BernsteinSplit(a []*big.Rat, at *big.Rat) ([]*big.Rat, []*big.Rat) {
	n := len(a) - 1
	left, right := make([]*big.Rat, n+1), make([]*big.Rat, n+1)
	work, den := bernsteinIntegerRow(a)
	p, q := new(big.Int).Set(at.Num()), new(big.Int).Set(at.Denom())
	qp := new(big.Int).Sub(q, p)
	scale := new(big.Int).Set(den)
	var t1, t2 big.Int
	for level := 0; level <= n; level++ {
		left[level] = proofarith.RatFrac(new(big.Rat), work[0], scale)
		right[n-level] = proofarith.RatFrac(new(big.Rat), work[n-level], scale)
		for i := range n - level {
			t1.Mul(qp, work[i])
			t2.Mul(p, work[i+1])
			work[i].Add(&t1, &t2)
		}
		scale.Mul(scale, q)
	}
	return left, right
}

// BernsteinMul multiplies in the scaled Bernstein basis, where the product is a
// plain convolution: c̃_k = Σ ã_i b̃_j with ã_i = C(n,i) a_i, and the
// Bernstein coefficient is c̃_k / C(n+m,k).
func BernsteinMul(a, b []*big.Rat) []*big.Rat {
	n, m := len(a)-1, len(b)-1
	an, ad := bernsteinScaledCoefficients(a)
	bn, bd := bernsteinScaledCoefficients(b)
	den := new(big.Int).Mul(ad, bd)
	binom := bernsteinBinomials(n + m)
	out := make([]*big.Rat, n+m+1)
	var term, outDen big.Int
	for k := range out {
		sum := new(big.Int)
		for i := max(0, k-m); i <= min(n, k); i++ {
			term.Mul(an[i], bn[k-i])
			sum.Add(sum, &term)
		}
		outDen.Mul(den, binom[k])
		out[k] = proofarith.RatFrac(new(big.Rat), sum, &outDen)
	}
	return out
}

// BernsteinRestrict is the Bernstein row of a on the sub-interval [lo, hi].
// It runs both de Casteljau splits over one shared integer denominator: the
// right half of the split at lo stays unreduced over den·q^n, and only the
// kept half of the second split is reduced.
func BernsteinRestrict(a []*big.Rat, lo, hi *big.Rat) []*big.Rat {
	// Restricting to [0, 1] is the identity. The coefficients are already in
	// lowest terms, so a copy has the same numerators and denominators the
	// general path would normalise them to.
	if lo.Sign() == 0 && hi.Cmp(bernsteinUnit) == 0 {
		out := make([]*big.Rat, len(a))
		for i, c := range a {
			out[i] = new(big.Rat).Set(c)
		}
		return out
	}
	n := len(a) - 1
	work, den := bernsteinIntegerRow(a)
	p, q := new(big.Int).Set(lo.Num()), new(big.Int).Set(lo.Denom())
	qp := new(big.Int).Sub(q, p)
	// right[n-level] = work[n-level] / (den*q^level) after level updates;
	// scaling by q^(n-level) puts every entry over den*q^n.
	qPow := make([]*big.Int, n+1)
	qPow[0] = big.NewInt(1)
	for i := 1; i <= n; i++ {
		qPow[i] = new(big.Int).Mul(qPow[i-1], q)
	}
	right := make([]*big.Int, n+1)
	var t1, t2 big.Int
	for level := 0; level <= n; level++ {
		right[n-level] = new(big.Int).Mul(work[n-level], qPow[n-level])
		for i := range n - level {
			t1.Mul(qp, work[i])
			t2.Mul(p, work[i+1])
			work[i].Add(&t1, &t2)
		}
	}
	t := ratDiv(ratSub(hi, lo), ratSub(ratOne(), lo))
	pp, qq := new(big.Int).Set(t.Num()), new(big.Int).Set(t.Denom())
	qpp := new(big.Int).Sub(qq, pp)
	scale := new(big.Int).Mul(den, qPow[n])
	left := make([]*big.Rat, n+1)
	for level := 0; level <= n; level++ {
		left[level] = proofarith.RatFrac(new(big.Rat), right[0], scale)
		for i := range n - level {
			t1.Mul(qpp, right[i])
			t2.Mul(pp, right[i+1])
			right[i].Add(&t1, &t2)
		}
		scale.Mul(scale, qq)
	}
	return left
}

// BernsteinAdd is the coefficient-wise sum of two rows of equal length.
func BernsteinAdd(a, b []*big.Rat) []*big.Rat {
	out := make([]*big.Rat, len(a))
	for i := range a {
		out[i] = ratAdd(a[i], b[i])
	}
	return out
}

// BernsteinSub is the coefficient-wise difference of two rows of equal length.
func BernsteinSub(a, b []*big.Rat) []*big.Rat {
	out := make([]*big.Rat, len(a))
	for i := range a {
		out[i] = ratSub(a[i], b[i])
	}
	return out
}

// BernsteinScale multiplies every coefficient of a by k.
func BernsteinScale(a []*big.Rat, k *big.Rat) []*big.Rat {
	out := make([]*big.Rat, len(a))
	for i := range a {
		out[i] = ratMul(a[i], k)
	}
	return out
}

// BernsteinDeriv is the Bernstein derivative n*(a[i+1]-a[i]).
func BernsteinDeriv(a []*big.Rat) []*big.Rat {
	out := make([]*big.Rat, len(a)-1)
	for i := range out {
		out[i] = ratMul(big.NewRat(int64(len(a)-1), 1), ratSub(a[i+1], a[i]))
	}
	return out
}

func bernsteinHalve(coeff []*big.Rat) ([]*big.Rat, []*big.Rat) {
	left := make([]*big.Rat, len(coeff))
	right := make([]*big.Rat, len(coeff))
	work := append([]*big.Rat(nil), coeff...)
	left[0], right[len(right)-1] = work[0], work[len(work)-1]
	for row := 1; row < len(coeff); row++ {
		next := make([]*big.Rat, len(work)-1)
		for i := range next {
			next[i] = ratMul(ratAdd(work[i], work[i+1]), big.NewRat(1, 2))
		}
		left[row], right[len(right)-1-row] = next[0], next[len(next)-1]
		work = next
	}
	return left, right
}

// BernsteinSignByHalving reports whether every coefficient of the row has the
// given sign, bisecting the row up to depth times when its own coefficients
// do not already decide it. A cancelled ctx returns its error.
func BernsteinSignByHalving(ctx context.Context, coeff []*big.Rat, sign, depth int) (bool, error) {
	if err := ctx.Err(); err != nil {
		return false, err
	}
	all := true
	for _, value := range coeff {
		if value.Sign() != sign {
			all = false
			break
		}
	}
	if all {
		return true, nil
	}
	if depth == 0 {
		return false, nil
	}
	left, right := bernsteinHalve(coeff)
	ok, err := BernsteinSignByHalving(ctx, left, sign, depth-1)
	if err != nil || !ok {
		return ok, err
	}
	return BernsteinSignByHalving(ctx, right, sign, depth-1)
}
