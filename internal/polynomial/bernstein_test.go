package polynomial_test

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/stretchr/testify/require"
)

// The *Ref functions are the big.Rat formulations the integer primitives
// replaced. The differential tests compare against them value for value.

// Products of Bernstein polynomials retain exact rational coefficients.
// The largest degree used here is 23, whose binomial weights fit int64.
func binomRef(n, k int) int64 {
	if k < 0 || k > n {
		return 0
	}
	if k > n-k {
		k = n - k
	}
	out := int64(1)
	for i := 1; i <= k; i++ {
		out = out * int64(n-k+i) / int64(i)
	}
	return out
}

func ratZero() *big.Rat             { return new(big.Rat) }
func ratOne() *big.Rat              { return big.NewRat(1, 1) }
func ratAdd(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
func ratSub(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
func ratMul(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }

func spiralBernValueRef(coeff []*big.Rat, parameter *big.Rat) *big.Rat {
	work := append([]*big.Rat(nil), coeff...)
	for width := len(work); width > 1; width-- {
		for i := 0; i+1 < width; i++ {
			work[i] = ratAdd(ratMul(ratSub(ratOne(), parameter), work[i]),
				ratMul(parameter, work[i+1]))
		}
	}
	return work[0]
}

func spiralIsolateRootNRef(coeff []*big.Rat, steps int) (lo, hi *big.Rat) {
	lo, hi = ratZero(), ratOne()
	for range steps {
		mid := ratMul(ratAdd(lo, hi), big.NewRat(1, 2))
		if spiralBernValueRef(coeff, mid).Sign() < 0 {
			lo = mid
		} else {
			hi = mid
		}
	}
	return lo, hi
}

func spiralBernSplitRef(a []*big.Rat, at *big.Rat) ([]*big.Rat, []*big.Rat) {
	n := len(a) - 1
	left, right := make([]*big.Rat, n+1), make([]*big.Rat, n+1)
	work := append([]*big.Rat(nil), a...)
	for level := 0; level <= n; level++ {
		left[level], right[n-level] = work[0], work[n-level]
		for i := range n - level {
			work[i] = ratAdd(ratMul(ratSub(ratOne(), at), work[i]), ratMul(at, work[i+1]))
		}
	}
	return left, right
}

// Products of Bernstein polynomials retain exact rational coefficients.
// The largest degree used here is 23, whose binomial weights fit int64.
func spiralMulRef(a, b []*big.Rat) []*big.Rat {
	n, m := len(a)-1, len(b)-1
	out := make([]*big.Rat, n+m+1)
	for k := range out {
		out[k] = ratZero()
		for i := max(0, k-m); i <= min(n, k); i++ {
			j := k - i
			weight := big.NewRat(binomRef(n, i)*binomRef(m, j), binomRef(n+m, k))
			out[k] = ratAdd(out[k], ratMul(weight, ratMul(a[i], b[j])))
		}
	}
	return out
}

func randomRatRow(rng *rand.Rand, n int) []*big.Rat {
	out := make([]*big.Rat, n)
	for i := range out {
		// Float-derived rationals have 2^-k denominators; mix in odd ones too.
		num := big.NewInt(rng.Int64N(1<<40) - 1<<39)
		num.Mul(num, big.NewInt(rng.Int64N(1<<30)+1))
		den := new(big.Int).Lsh(big.NewInt(1), uint(rng.IntN(60)))
		if rng.IntN(3) == 0 {
			den.Mul(den, big.NewInt(rng.Int64N(1000)+1))
		}
		out[i] = new(big.Rat).SetFrac(num, den)
	}
	return out
}

func randomParameter(rng *rand.Rand) *big.Rat {
	switch rng.IntN(4) {
	case 0:
		return big.NewRat(rng.Int64N(1<<20), 1<<20)
	case 1:
		return new(big.Rat).SetFrac(big.NewInt(rng.Int64N(1<<62)), new(big.Int).Lsh(big.NewInt(1), 64))
	case 2:
		return big.NewRat(rng.Int64N(997), 997)
	default:
		return big.NewRat(rng.Int64N(3)-1, 1)
	}
}

func requireSameRows(t *testing.T, want, got []*big.Rat) {
	t.Helper()
	require.Len(t, got, len(want))
	for i := range want {
		require.Equal(t, 0, want[i].Cmp(got[i]), "index %d", i)
		require.Equal(t, want[i].Num().String(), got[i].Num().String())
		require.Equal(t, want[i].Denom().String(), got[i].Denom().String())
	}
}

func TestBernsteinMatchesReference(t *testing.T) {
	rng := rand.New(rand.NewPCG(7, 11))
	for round := range 72 {
		n := rng.IntN(13) + 1
		a := randomRatRow(rng, n+1)
		m := rng.IntN(13) + 1
		b := randomRatRow(rng, m+1)
		s := randomParameter(rng)
		requireSameRows(t, spiralMulRef(a, b), polynomial.BernsteinMul(a, b))
		product := polynomial.BernsteinMul(a, b)
		pl, pr := spiralBernSplitRef(product, s)
		gl2, gr2 := polynomial.BernsteinSplit(product, s)
		requireSameRows(t, pl, gl2)
		requireSameRows(t, pr, gr2)
		requireSameRows(t, []*big.Rat{spiralBernValueRef(product, s)}, []*big.Rat{polynomial.BernsteinValue(product, s)})
		pLo, pHi := spiralIsolateRootNRef(product, 20)
		gLo, gHi := polynomial.BernsteinIsolateRoot(product, 20)
		requireSameRows(t, []*big.Rat{pLo, pHi}, []*big.Rat{gLo, gHi})
		want, got := spiralBernValueRef(a, s), polynomial.BernsteinValue(a, s)
		requireSameRows(t, []*big.Rat{want}, []*big.Rat{got})
		require.Equal(t, want.Sign(), polynomial.BernsteinSign(a, s))
		wl, wr := spiralBernSplitRef(a, s)
		gl, gr := polynomial.BernsteinSplit(a, s)
		requireSameRows(t, wl, gl)
		requireSameRows(t, wr, gr)
		steps := []int{0, 1, 5, 64, 80, 128}[round%6]
		wantLo, wantHi := spiralIsolateRootNRef(a, steps)
		gotLo, gotHi := polynomial.BernsteinIsolateRoot(a, steps)
		requireSameRows(t, []*big.Rat{wantLo, wantHi}, []*big.Rat{gotLo, gotHi})
	}
}
