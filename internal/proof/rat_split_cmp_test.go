package proof_test

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// TestCmpRatDyadicMatchesBigRat compares CmpRat with big.Rat.Cmp over dyadic
// pairs that probe the equal-exponent branch and the bit-length branch: zero,
// negatives, equal values in different representations, neighbours one unit
// in the last place apart, shifts of +-1100 bits and numerators at a bit-length
// boundary (2^k-1 against 2^k).
func TestCmpRatDyadicMatchesBigRat(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(5, 9))
	dyadic := func(n *big.Int, e int) *big.Rat {
		if e >= 0 {
			return new(big.Rat).SetFrac(n, new(big.Int).Lsh(big.NewInt(1), uint(e)))
		}
		return new(big.Rat).SetInt(new(big.Int).Lsh(n, uint(-e)))
	}
	signed := func(n *big.Int) *big.Int {
		if rng.IntN(2) == 0 {
			n.Neg(n)
		}
		return n
	}
	exps := []int{0, 1, 2, 3, 63, 64, 65, 1100, -1100, -1, -64}
	exp := func() int {
		if rng.IntN(3) == 0 {
			return exps[rng.IntN(len(exps))]
		}
		return rng.IntN(13) - 6
	}
	pow2 := func(k int) *big.Int { return new(big.Int).Lsh(big.NewInt(1), uint(k)) }

	var pairs [][2]*big.Rat
	add := func(a, b *big.Rat) { pairs = append(pairs, [2]*big.Rat{a, b}) }
	for range 20000 {
		n := signed(randBits(rng, 1+rng.IntN(120)))
		e := exp()
		a := dyadic(n, e)
		switch rng.IntN(6) {
		case 0:
			add(a, new(big.Rat))
		case 1:
			// Equal value, different representation.
			s := rng.IntN(70)
			add(a, dyadic(new(big.Int).Lsh(n, uint(s)), e+s))
		case 2:
			// Neighbour one unit in the last place away.
			add(a, dyadic(new(big.Int).Add(n, big.NewInt(1)), e))
			add(a, dyadic(new(big.Int).Sub(n, big.NewInt(1)), e))
		case 3:
			// Bit-length boundary after alignment: 2^k-1 against 2^k.
			k := 1 + rng.IntN(130)
			s := rng.IntN(5) - 2
			lo := new(big.Int).Sub(pow2(k), big.NewInt(1))
			hi := pow2(k)
			if rng.IntN(2) == 0 {
				lo.Neg(lo)
				hi.Neg(hi)
			}
			add(dyadic(lo, e), dyadic(hi, e+s))
			add(dyadic(hi, e), dyadic(lo, e+s))
			add(dyadic(lo, e), dyadic(new(big.Int).Lsh(hi, 1), e+1))
		case 4:
			add(a, dyadic(signed(randBits(rng, 1+rng.IntN(120))), e))
		default:
			add(a, dyadic(signed(randBits(rng, 1+rng.IntN(120))), exp()))
		}
	}
	for _, p := range pairs {
		a, b := p[0], p[1]
		wantAB, wantBA := a.Cmp(b), b.Cmp(a)
		aBefore, bBefore := a.String(), b.String()
		if proof.CmpRat(a, b) != wantAB || proof.CmpRat(b, a) != wantBA {
			require.Failf(t, "CmpRat differs from big.Rat.Cmp", "a=%s b=%s want=%d", aBefore, bBefore, wantAB)
		}
		require.Equal(t, aBefore, a.String())
		require.Equal(t, bBefore, b.String())
	}
}
