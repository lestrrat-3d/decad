package proof_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// written is n/d with both integers stored as given, unreduced and with any
// sign, the forms only a write through Num or Denom builds.
func written(n, d int64) *big.Rat {
	r := big.NewRat(1, 7)
	r.Num().SetInt64(n)
	r.Denom().SetInt64(d)
	return r
}

func ratTextPool() []*big.Rat {
	huge, _ := new(big.Rat).SetString("340282366920938463463374607431768211457/18446744073709551629")
	hugeNext, _ := new(big.Rat).SetString("340282366920938463463374607431768211459/18446744073709551629")
	return []*big.Rat{
		new(big.Rat),             // zero value: no stored denominator
		big.NewRat(0, 9),         // zero with a stored denominator
		new(big.Rat).SetInt64(3), // integer, no stored denominator
		big.NewRat(3, 1),         // integer, stored denominator 1
		big.NewRat(-3, 1),
		big.NewRat(1, 2),
		big.NewRat(-1, 2),
		big.NewRat(3, 2),
		big.NewRat(1, 3),
		written(3, 9),  // unreduced 1/3
		written(6, 18), // unreduced 1/3, another form
		written(9, 3),  // unreduced 3
		written(3, -1), // IsInt with a negative denominator: text "3"
		written(1, -2),
		huge,
		hugeNext,
	}
}

// TestRatTextEqualMatchesRatString checks every pair of the pool, plus each
// against an equal-valued fresh copy, against the texts themselves, and
// that equal texts hash equally.
func TestRatTextEqualMatchesRatString(t *testing.T) {
	t.Parallel()
	pool := ratTextPool()
	pool = append(pool, ratTextPool()...) // equal content behind new pointers
	for i, a := range pool {
		for j, b := range pool {
			want := a.RatString() == b.RatString()
			require.Equal(t, want, proof.RatTextEqual(a, b), "pair %d %d (%s, %s)", i, j, a.RatString(), b.RatString())
			if want {
				require.Equal(t, proof.RatTextHash(proof.HashSeed, a), proof.RatTextHash(proof.HashSeed, b), "pair %d %d", i, j)
			}
		}
	}
}

// TestRatTextEqualNeedsEachLeg pins one pair per comparison RatTextEqual
// makes, each equal in every other respect.
func TestRatTextEqualNeedsEachLeg(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		a, b *big.Rat
	}{
		{"integer against fraction", big.NewRat(1, 1), written(1, 2)},
		{"fraction against integer", written(1, 2), big.NewRat(1, 1)},
		{"numerator", big.NewRat(1, 2), big.NewRat(3, 2)},
		{"denominator", big.NewRat(1, 2), big.NewRat(1, 3)},
	} {
		require.NotEqual(t, tc.a.RatString(), tc.b.RatString(), tc.name)
		require.False(t, proof.RatTextEqual(tc.a, tc.b), tc.name)
	}
}

// TestIntHashSeparatesWordSplits checks that the word count and sign enter
// the hash, so integers that share their words hash apart.
func TestIntHashSeparatesWordSplits(t *testing.T) {
	t.Parallel()
	two := new(big.Int).Lsh(big.NewInt(5), 64)
	two.Add(two, big.NewInt(7))
	pairA := proof.IntHash(proof.IntHash(proof.HashSeed, big.NewInt(7)), big.NewInt(5))
	pairB := proof.IntHash(proof.IntHash(proof.HashSeed, two), new(big.Int))
	require.NotEqual(t, pairA, pairB)
	require.NotEqual(t, proof.IntHash(proof.HashSeed, big.NewInt(3)), proof.IntHash(proof.HashSeed, big.NewInt(-3)))
}
