package decad

import "math/big"

// bernsteinIntRow is a Bernstein coefficient row held as integer numerators over
// one positive denominator: coefficient i is nums[i]/den. Chaining add, sub,
// scale, deriv and mul on it defers every GCD to rats, which converts each
// coefficient once with big.Rat.SetFrac. The rational returned for a
// coefficient is the canonical big.Rat of the same exact value, so a chain
// computed here equals the same chain computed with big.Rat operations.
type bernsteinIntRow struct {
	nums []*big.Int
	den  *big.Int
}

func bernsteinIntRowOf(a []*big.Rat) bernsteinIntRow {
	nums, den := bernsteinIntegerRow(a)
	return bernsteinIntRow{nums: nums, den: den}
}

func (r bernsteinIntRow) rats() []*big.Rat {
	out := make([]*big.Rat, len(r.nums))
	for i, n := range r.nums {
		out[i] = new(big.Rat).SetFrac(n, r.den)
	}
	return out
}

// commonScale returns the factors fa, fb that bring a and b to the common
// denominator lcm(a.den, b.den) = a.den*fa = b.den*fb.
func (r bernsteinIntRow) commonScale(o bernsteinIntRow) (fa, fb, den *big.Int) {
	var g big.Int
	g.GCD(nil, nil, r.den, o.den)
	fa = new(big.Int).Quo(o.den, &g)
	fb = new(big.Int).Quo(r.den, &g)
	den = new(big.Int).Mul(r.den, fa)
	return fa, fb, den
}

func (r bernsteinIntRow) add(o bernsteinIntRow) bernsteinIntRow { return r.combine(o, 1) }
func (r bernsteinIntRow) sub(o bernsteinIntRow) bernsteinIntRow { return r.combine(o, -1) }

func (r bernsteinIntRow) combine(o bernsteinIntRow, sign int) bernsteinIntRow {
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
	return bernsteinIntRow{nums: nums, den: den}
}

// scale multiplies every coefficient by k.
func (r bernsteinIntRow) scale(k *big.Rat) bernsteinIntRow {
	nums := make([]*big.Int, len(r.nums))
	for i, n := range r.nums {
		nums[i] = new(big.Int).Mul(n, k.Num())
	}
	return bernsteinIntRow{nums: nums, den: new(big.Int).Mul(r.den, k.Denom())}
}

// deriv is the Bernstein derivative n*(a[i+1]-a[i]).
func (r bernsteinIntRow) deriv() bernsteinIntRow {
	n := big.NewInt(int64(len(r.nums) - 1))
	nums := make([]*big.Int, len(r.nums)-1)
	for i := range nums {
		nums[i] = new(big.Int).Sub(r.nums[i+1], r.nums[i])
		nums[i].Mul(nums[i], n)
	}
	return bernsteinIntRow{nums: nums, den: new(big.Int).Set(r.den)}
}

// mul multiplies in the scaled basis, where the product is a convolution,
// and returns the coefficients over den(a)*den(b)*lcm(C(n+m,k)). The result
// is divided by the common factor of its numerators and denominator.
func (r bernsteinIntRow) mul(o bernsteinIntRow) bernsteinIntRow {
	n, m := len(r.nums)-1, len(o.nums)-1
	binom := bernsteinBinomials(n + m)
	lcm := big.NewInt(1)
	var g big.Int
	for _, c := range binom {
		g.GCD(nil, nil, lcm, c)
		lcm.Quo(lcm, &g)
		lcm.Mul(lcm, c)
	}
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
	return bernsteinIntRow{nums: nums, den: den}.reduced()
}

// reduced divides the numerators and denominator by their common factor.
func (r bernsteinIntRow) reduced() bernsteinIntRow {
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
	return bernsteinIntRow{nums: nums, den: new(big.Int).Quo(r.den, g)}
}
