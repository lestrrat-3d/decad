package polynomial_test

import (
	"math/big"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/stretchr/testify/require"
)

// legacyBernsteinRestrict is the two-split form BernsteinRestrict had
// before it shared one integer denominator.
func legacyBernsteinRestrict(a []*big.Rat, lo, hi *big.Rat) []*big.Rat {
	_, right := polynomial.BernsteinSplit(a, lo)
	left, _ := polynomial.BernsteinSplit(right, new(big.Rat).Quo(new(big.Rat).Sub(hi, lo), new(big.Rat).Sub(big.NewRat(1, 1), lo)))
	return left
}

func randomTestRat(rng *rand.Rand) *big.Rat {
	var r *big.Rat
	switch rng.Intn(3) {
	case 0:
		r = big.NewRat(rng.Int63n(2001)-1000, 1)
	case 1:
		r = big.NewRat(rng.Int63n(2001)-1000, int64(1)<<uint(rng.Intn(20)))
	default:
		r = big.NewRat(rng.Int63n(2000001)-1000000, rng.Int63n(1000000)+1)
	}
	return r
}

func randomUnitRat(rng *rand.Rand) *big.Rat {
	switch rng.Intn(4) {
	case 0:
		return big.NewRat(rng.Int63n(1000), 1000)
	default:
		den := rng.Int63n(1000000) + 1
		return big.NewRat(rng.Int63n(den+1), den)
	}
}

func TestBernsteinRestrictMatchesSplitForm(t *testing.T) {
	rng := rand.New(rand.NewSource(11))
	for iter := range 300 {
		a := make([]*big.Rat, 2+rng.Intn(9))
		for i := range a {
			a[i] = randomTestRat(rng)
		}
		lo, hi := randomUnitRat(rng), randomUnitRat(rng)
		switch iter % 10 {
		case 0:
			lo, hi = new(big.Rat), big.NewRat(1, 1)
		case 1:
			lo = new(big.Rat)
		case 2:
			hi = big.NewRat(1, 1)
		}
		if lo.Cmp(hi) > 0 {
			lo, hi = hi, lo
		}
		if lo.Cmp(hi) == 0 || lo.Cmp(big.NewRat(1, 1)) == 0 {
			continue
		}
		want := legacyBernsteinRestrict(a, lo, hi)
		got := polynomial.BernsteinRestrict(a, lo, hi)
		require.Len(t, got, len(want))
		for i := range want {
			require.Zero(t, want[i].Cmp(got[i]), "iter %d coefficient %d", iter, i)
			require.Zero(t, want[i].Num().Cmp(got[i].Num()))
			require.Zero(t, want[i].Denom().Cmp(got[i].Denom()))
		}
	}
}
