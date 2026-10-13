package decad

import "math/big"

// spiralIntRow is a Bernstein coefficient row held as integer numerators over
// one positive denominator: coefficient i is nums[i]/den. Chaining add, sub,
// scale, deriv and mul on it defers every GCD to rats, which converts each
// coefficient once with big.Rat.SetFrac. The rational returned for a
// coefficient is the canonical big.Rat of the same exact value, so a chain
// computed here equals the same chain computed with big.Rat operations.
type spiralIntRow struct {
	nums []*big.Int
	den  *big.Int
}

func spiralIntRowOf(a []*big.Rat) spiralIntRow {
	nums, den := spiralIntegerRow(a)
	return spiralIntRow{nums: nums, den: den}
}

func (r spiralIntRow) rats() []*big.Rat {
	out := make([]*big.Rat, len(r.nums))
	for i, n := range r.nums {
		out[i] = new(big.Rat).SetFrac(n, r.den)
	}
	return out
}

// commonScale returns the factors fa, fb that bring a and b to the common
// denominator lcm(a.den, b.den) = a.den*fa = b.den*fb.
func (r spiralIntRow) commonScale(o spiralIntRow) (fa, fb, den *big.Int) {
	var g big.Int
	g.GCD(nil, nil, r.den, o.den)
	fa = new(big.Int).Quo(o.den, &g)
	fb = new(big.Int).Quo(r.den, &g)
	den = new(big.Int).Mul(r.den, fa)
	return fa, fb, den
}

func (r spiralIntRow) add(o spiralIntRow) spiralIntRow { return r.combine(o, 1) }
func (r spiralIntRow) sub(o spiralIntRow) spiralIntRow { return r.combine(o, -1) }

func (r spiralIntRow) combine(o spiralIntRow, sign int) spiralIntRow {
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
	return spiralIntRow{nums: nums, den: den}
}

// scale multiplies every coefficient by k.
func (r spiralIntRow) scale(k *big.Rat) spiralIntRow {
	nums := make([]*big.Int, len(r.nums))
	for i, n := range r.nums {
		nums[i] = new(big.Int).Mul(n, k.Num())
	}
	return spiralIntRow{nums: nums, den: new(big.Int).Mul(r.den, k.Denom())}
}

// deriv is the Bernstein derivative n*(a[i+1]-a[i]).
func (r spiralIntRow) deriv() spiralIntRow {
	n := big.NewInt(int64(len(r.nums) - 1))
	nums := make([]*big.Int, len(r.nums)-1)
	for i := range nums {
		nums[i] = new(big.Int).Sub(r.nums[i+1], r.nums[i])
		nums[i].Mul(nums[i], n)
	}
	return spiralIntRow{nums: nums, den: new(big.Int).Set(r.den)}
}

// mul multiplies in the scaled basis, where the product is a convolution,
// and returns the coefficients over den(a)*den(b)*lcm(C(n+m,k)). The result
// is divided by the common factor of its numerators and denominator.
func (r spiralIntRow) mul(o spiralIntRow) spiralIntRow {
	n, m := len(r.nums)-1, len(o.nums)-1
	binom := spiralBigBinomials(n + m)
	lcm := big.NewInt(1)
	var g big.Int
	for _, c := range binom {
		g.GCD(nil, nil, lcm, c)
		lcm.Quo(lcm, &g)
		lcm.Mul(lcm, c)
	}
	an, bn := make([]*big.Int, n+1), make([]*big.Int, m+1)
	an0, bn0 := spiralBigBinomials(n), spiralBigBinomials(m)
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
	return spiralIntRow{nums: nums, den: den}.reduced()
}

// reduced divides the numerators and denominator by their common factor.
func (r spiralIntRow) reduced() spiralIntRow {
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
	return spiralIntRow{nums: nums, den: new(big.Int).Quo(r.den, g)}
}
