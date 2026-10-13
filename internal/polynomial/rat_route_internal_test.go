package polynomial

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"
)

func requireRatsEqual(t *testing.T, want, got []*big.Rat) {
	t.Helper()
	require.Len(t, got, len(want))
	for i := range want {
		require.Zero(t, want[i].Num().Cmp(got[i].Num()), "numerator %d: want %s, got %s", i, want[i], got[i])
		require.Zero(t, want[i].Denom().Cmp(got[i].Denom()), "denominator %d: want %s, got %s", i, want[i], got[i])
	}
}

// randCoeff is a dyadic or non-dyadic coefficient of either sign, or zero.
func randCoeff(rng *rand.Rand) *big.Rat {
	n := big.NewInt(rng.Int64N(1<<40) - 1<<39)
	switch rng.IntN(4) {
	case 0:
		return new(big.Rat).SetInt(n)
	case 1:
		return new(big.Rat).SetFrac(n, new(big.Int).Lsh(big.NewInt(1), uint(rng.IntN(70))))
	default:
		return new(big.Rat).SetFrac(n, big.NewInt(rng.Int64N(1<<20)+1))
	}
}

func randRow(rng *rand.Rand) []*big.Rat {
	row := make([]*big.Rat, 2+rng.IntN(8))
	for i := range row {
		row[i] = randCoeff(rng)
	}
	return row
}

// randUnit is a parameter in [0, 1], dyadic or not.
func randUnit(rng *rand.Rand) *big.Rat {
	if rng.IntN(2) == 0 {
		return big.NewRat(rng.Int64N(1<<20), 1<<20)
	}
	q := rng.Int64N(1<<20) + 1
	return big.NewRat(rng.Int64N(q+1), q)
}

func legacyValue(coeff []*big.Rat, parameter *big.Rat) *big.Rat {
	scaled, den := bernsteinScaledCoefficients(coeff)
	p, q := parameter.Num(), parameter.Denom()
	num := bernsteinScaledNumerator(scaled, p, q)
	qn := new(big.Int).Exp(q, big.NewInt(int64(len(coeff)-1)), nil)
	return new(big.Rat).SetFrac(num, qn.Mul(qn, den))
}

func legacyIsolateRoot(coeff []*big.Rat, steps int) (*big.Rat, *big.Rat) {
	scaled, _ := bernsteinScaledCoefficients(coeff)
	a, q, mid := new(big.Int), big.NewInt(1), new(big.Int)
	for range steps {
		a.Lsh(a, 1)
		q.Lsh(q, 1)
		mid.Add(a, big.NewInt(1))
		if bernsteinScaledNumerator(scaled, mid, q).Sign() < 0 {
			a.Set(mid)
		}
	}
	return new(big.Rat).SetFrac(a, q), new(big.Rat).SetFrac(new(big.Int).Add(a, big.NewInt(1)), q)
}

func legacySplit(a []*big.Rat, at *big.Rat) ([]*big.Rat, []*big.Rat) {
	n := len(a) - 1
	left, right := make([]*big.Rat, n+1), make([]*big.Rat, n+1)
	work, den := bernsteinIntegerRow(a)
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

func legacyMul(a, b []*big.Rat) []*big.Rat {
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
		out[k] = new(big.Rat).SetFrac(sum, &outDen)
	}
	return out
}

func legacyRestrict(a []*big.Rat, lo, hi *big.Rat) []*big.Rat {
	n := len(a) - 1
	work, den := bernsteinIntegerRow(a)
	p, q := lo.Num(), lo.Denom()
	qp := new(big.Int).Sub(q, p)
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
	t := new(big.Rat).Quo(new(big.Rat).Sub(hi, lo), new(big.Rat).Sub(big.NewRat(1, 1), lo))
	pp, qq := t.Num(), t.Denom()
	qpp := new(big.Int).Sub(qq, pp)
	scale := new(big.Int).Mul(den, qPow[n])
	left := make([]*big.Rat, n+1)
	for level := 0; level <= n; level++ {
		left[level] = new(big.Rat).SetFrac(right[0], scale)
		for i := range n - level {
			t1.Mul(qpp, right[i])
			t2.Mul(pp, right[i+1])
			right[i].Add(&t1, &t2)
		}
		scale.Mul(scale, qq)
	}
	return left
}

func legacyDeriv(a []*big.Rat) []*big.Rat {
	out := make([]*big.Rat, len(a)-1)
	for i := range out {
		out[i] = new(big.Rat).Mul(big.NewRat(int64(len(a)-1), 1), new(big.Rat).Sub(a[i+1], a[i]))
	}
	return out
}

func TestExactRatRoutesMatchLegacy(t *testing.T) {
	rng := rand.New(rand.NewPCG(47, 53))
	for range 300 {
		a, b := randRow(rng), randRow(rng)
		at := randUnit(rng)

		requireRatsEqual(t, []*big.Rat{legacyValue(a, at)}, []*big.Rat{BernsteinValue(a, at)})

		steps := 1 + rng.IntN(40)
		wantLo, wantHi := legacyIsolateRoot(a, steps)
		gotLo, gotHi := BernsteinIsolateRoot(a, steps)
		requireRatsEqual(t, []*big.Rat{wantLo, wantHi}, []*big.Rat{gotLo, gotHi})

		wantL, wantR := legacySplit(a, at)
		gotL, gotR := BernsteinSplit(a, at)
		requireRatsEqual(t, wantL, gotL)
		requireRatsEqual(t, wantR, gotR)

		requireRatsEqual(t, legacyMul(a, b), BernsteinMul(a, b))

		lo, hi := randUnit(rng), randUnit(rng)
		if lo.Cmp(hi) > 0 {
			lo, hi = hi, lo
		}
		if lo.Cmp(hi) == 0 || lo.Cmp(big.NewRat(1, 1)) == 0 {
			hi = big.NewRat(1, 1)
			lo = big.NewRat(rng.Int64N(1<<10), 1<<11)
		}
		requireRatsEqual(t, legacyRestrict(a, lo, hi), BernsteinRestrict(a, lo, hi))

		requireRatsEqual(t, legacyDeriv(a), BernsteinDeriv(a))

		nums, den := bernsteinIntegerRow(a)
		want := make([]*big.Rat, len(nums))
		for i, n := range nums {
			want[i] = new(big.Rat).SetFrac(n, den)
		}
		requireRatsEqual(t, want, BernsteinIntRow{nums: nums, den: den}.Rats())
		// A power-of-two denominator takes the shift route.
		pow := new(big.Int).Lsh(big.NewInt(1), uint(rng.IntN(80)))
		for i, n := range nums {
			want[i] = new(big.Rat).SetFrac(n, pow)
		}
		requireRatsEqual(t, want, BernsteinIntRow{nums: nums, den: pow}.Rats())
	}
}
