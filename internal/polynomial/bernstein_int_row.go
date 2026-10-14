package polynomial

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// BernsteinIntRow is a Bernstein coefficient row held as integer numerators over
// one positive denominator: coefficient i is nums[i]/den. Chaining Add, Sub,
// Scale, Deriv and Mul on it defers every GCD to Rats, which converts each
// coefficient once with proof.RatFrac (a shift for a power-of-two denominator,
// big.Rat.SetFrac otherwise). The rational returned for a
// coefficient is the canonical big.Rat of the same exact value, so a chain
// computed here equals the same chain computed with big.Rat operations.
type BernsteinIntRow struct {
	nums []*big.Int
	den  *big.Int
}

// BernsteinIntRowOf returns a as an integer-numerator row.
func BernsteinIntRowOf(a []*big.Rat) BernsteinIntRow {
	nums, den := bernsteinIntegerRow(a)
	return BernsteinIntRow{nums: nums, den: den}
}

// Rats returns the canonical big.Rat of every coefficient.
func (r BernsteinIntRow) Rats() []*big.Rat {
	out := make([]*big.Rat, len(r.nums))
	for i, n := range r.nums {
		out[i] = proofarith.RatFrac(new(big.Rat), n, r.den)
	}
	return out
}

// commonScale returns the factors fa, fb that bring a and b to the common
// denominator lcm(a.den, b.den) = a.den*fa = b.den*fb.
func (r BernsteinIntRow) commonScale(o BernsteinIntRow) (fa, fb, den *big.Int) {
	var g big.Int
	g.GCD(nil, nil, r.den, o.den)
	fa = new(big.Int).Quo(o.den, &g)
	fb = new(big.Int).Quo(r.den, &g)
	den = new(big.Int).Mul(r.den, fa)
	return fa, fb, den
}

// Add is the coefficient-wise sum of r and o.
func (r BernsteinIntRow) Add(o BernsteinIntRow) BernsteinIntRow { return r.combine(o, 1) }

// Sub is the coefficient-wise difference of r and o.
func (r BernsteinIntRow) Sub(o BernsteinIntRow) BernsteinIntRow { return r.combine(o, -1) }

func (r BernsteinIntRow) combine(o BernsteinIntRow, sign int) BernsteinIntRow {
	fa, fb, den := r.commonScale(o)
	nums := make([]*big.Int, len(r.nums))
	var t big.Int
	for i := range nums {
		nums[i] = new(big.Int).Mul(r.nums[i], fa)
		t.Mul(o.nums[i], fb)
		if sign > 0 {
			nums[i].Add(nums[i], &t)
		} else {
			nums[i].Sub(nums[i], &t)
		}
	}
	return BernsteinIntRow{nums: nums, den: den}
}

// Scale multiplies every coefficient by k.
func (r BernsteinIntRow) Scale(k *big.Rat) BernsteinIntRow {
	nums := make([]*big.Int, len(r.nums))
	for i, n := range r.nums {
		nums[i] = new(big.Int).Mul(n, k.Num())
	}
	return BernsteinIntRow{nums: nums, den: new(big.Int).Mul(r.den, k.Denom())}
}

// Deriv is the Bernstein derivative n*(a[i+1]-a[i]).
func (r BernsteinIntRow) Deriv() BernsteinIntRow {
	n := big.NewInt(int64(len(r.nums) - 1))
	nums := make([]*big.Int, len(r.nums)-1)
	for i := range nums {
		nums[i] = new(big.Int).Sub(r.nums[i+1], r.nums[i])
		nums[i].Mul(nums[i], n)
	}
	return BernsteinIntRow{nums: nums, den: new(big.Int).Set(r.den)}
}

// Mul multiplies in the scaled basis, where the product is a convolution,
// and returns the coefficients over den(a)*den(b)*lcm(C(n+m,k)). The result
// is divided by the common factor of its numerators and denominator.
func (r BernsteinIntRow) Mul(o BernsteinIntRow) BernsteinIntRow {
	n, m := len(r.nums)-1, len(o.nums)-1
	binom := bernsteinBinomials(n + m)
	lcm := bernsteinBinomialLCM(n + m)
	an, bn := make([]*big.Int, n+1), make([]*big.Int, m+1)
	an0, bn0 := bernsteinBinomials(n), bernsteinBinomials(m)
	for i := range an {
		an[i] = new(big.Int).Mul(r.nums[i], an0[i])
	}
	for j := range bn {
		bn[j] = new(big.Int).Mul(o.nums[j], bn0[j])
	}
	nums := make([]*big.Int, n+m+1)
	var term, weight big.Int
	for k := range nums {
		sum := new(big.Int)
		for i := max(0, k-m); i <= min(n, k); i++ {
			term.Mul(an[i], bn[k-i])
			sum.Add(sum, &term)
		}
		weight.Quo(lcm, binom[k])
		nums[k] = sum.Mul(sum, &weight)
	}
	den := new(big.Int).Mul(r.den, o.den)
	den.Mul(den, lcm)
	return BernsteinIntRow{nums: nums, den: den}.reduced()
}

// reduced divides the numerators and denominator by their common factor.
func (r BernsteinIntRow) reduced() BernsteinIntRow {
	g := new(big.Int).Set(r.den)
	for _, n := range r.nums {
		if g.IsInt64() && g.Int64() == 1 {
			return r
		}
		g.GCD(nil, nil, g, new(big.Int).Abs(n))
	}
	if g.Sign() == 0 || (g.IsInt64() && g.Int64() == 1) {
		return r
	}
	nums := make([]*big.Int, len(r.nums))
	for i, n := range r.nums {
		nums[i] = new(big.Int).Quo(n, g)
	}
	return BernsteinIntRow{nums: nums, den: new(big.Int).Quo(r.den, g)}
}

// bernsteinBinomialLCMTable holds lcm(C(n, 0..n)) for n in 0..32. Read only.
// These are constants of n alone, computed once at package initialisation;
// no caller may write into a returned big.Int.
var bernsteinBinomialLCMTable = func() []*big.Int {
	table := make([]*big.Int, len(bernsteinBinomialTable))
	for n := range table {
		table[n] = bernsteinBinomialLCMFresh(n)
	}
	return table
}()

// bernsteinBinomialLCM returns lcm(C(n, 0..n)). The result may be a shared
// table value: callers must only read it.
func bernsteinBinomialLCM(n int) *big.Int {
	if n >= 0 && n < len(bernsteinBinomialLCMTable) {
		return bernsteinBinomialLCMTable[n]
	}
	return bernsteinBinomialLCMFresh(n)
}

func bernsteinBinomialLCMFresh(n int) *big.Int {
	lcm := big.NewInt(1)
	var g big.Int
	for _, c := range bernsteinBinomialsFresh(n) {
		g.GCD(nil, nil, lcm, c)
		lcm.Quo(lcm, &g)
		lcm.Mul(lcm, c)
	}
	return lcm
}
