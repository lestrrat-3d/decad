package decad

import "math/big"

// Exact Bernstein arithmetic uses one shared integer denominator. The
// common denominator avoids normalizing a big.Rat after each operation.

// bernsteinIntegerRow returns fresh numerators over their positive least
// common denominator. nums[i]/den equals coeff[i] exactly.
func bernsteinIntegerRow(coeff []*big.Rat) ([]*big.Int, *big.Int) {
	den := big.NewInt(1)
	var gcd big.Int
	for _, c := range coeff {
		d := c.Denom()
		gcd.GCD(nil, nil, den, d)
		den.Quo(den, &gcd)
		den.Mul(den, d)
	}
	nums := make([]*big.Int, len(coeff))
	var scale big.Int
	for i, c := range coeff {
		scale.Quo(den, c.Denom())
		nums[i] = new(big.Int).Mul(c.Num(), &scale)
	}
	return nums, den
}

func bernsteinBinomials(n int) []*big.Int {
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

// bernsteinExactSign is the sign of a Bernstein row at a rational parameter.
// The omitted denominator is positive, so the integer numerator has the
// same sign as the exact value.
func bernsteinExactSign(coeff []*big.Rat, parameter *big.Rat) int {
	scaled, _ := bernsteinScaledCoefficients(coeff)
	return bernsteinScaledNumerator(scaled, parameter.Num(), parameter.Denom()).Sign()
}
