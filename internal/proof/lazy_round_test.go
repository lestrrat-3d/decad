package proof_test

import (
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func TestLazyRound(t *testing.T) {
	const bits = 128
	rng := rand.New(rand.NewPCG(1, 2))
	// randInt returns a random integer with exactly n bits.
	randInt := func(n int) *big.Int {
		words := make([]big.Word, (n+63)/64)
		for i := range words {
			words[i] = big.Word(rng.Uint64())
		}
		v := new(big.Int).SetBits(words)
		v.Rsh(v, uint(len(words)*64-n))
		return v.SetBit(v, n-1, 1)
	}
	var cases []*big.Rat
	for range 200 {
		n, d := randInt(64+rng.IntN(5900)), randInt(64+rng.IntN(5900))
		if rng.IntN(2) == 0 {
			n.Neg(n)
		}
		cases = append(cases, new(big.Rat).SetFrac(n, d))
	}
	cases = append(cases, big.NewRat(0, 1), big.NewRat(3, 7), big.NewRat(-5, 1),
		new(big.Rat).SetFrac(new(big.Int).Lsh(big.NewInt(1), 5000), big.NewInt(1)),
		new(big.Rat).SetFrac(big.NewInt(-1), new(big.Int).Lsh(big.NewInt(1), 5000)))
	limit := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), bits-1))
	for idx, x := range cases {
		lx := proofarith.LazyOf(x)
		down, up := proofarith.LazyRoundDown(lx, bits), proofarith.LazyRoundUp(lx, bits)
		require.LessOrEqual(t, proofarith.LazyCmp(down, lx), 0, "case %d down of", idx)
		require.GreaterOrEqual(t, proofarith.LazyCmp(up, lx), 0, "case %d up of", idx)
		require.Equal(t, x.Sign(), down.Sign(), "case %d down sign of", idx)
		require.Equal(t, x.Sign(), up.Sign(), "case %d up sign of", idx)
		n, d := x.Num(), x.Denom()
		if n.Sign() == 0 || n.BitLen()+d.BitLen() <= 4*bits {
			require.Zero(t, down.Rat().Cmp(x), "case %d small value moved down", idx)
			require.Zero(t, up.Rat().Cmp(x), "case %d small value moved up", idx)
			continue
		}
		gap := new(big.Rat).Sub(up.Rat(), down.Rat())
		rel := new(big.Rat).Quo(gap, new(big.Rat).Abs(x))
		require.Negative(t, rel.Cmp(new(big.Rat).Mul(big.NewRat(4, 1), limit)), "case %d relative gap of", idx)
		for _, end := range []proofarith.LazyRat{down, up} {
			// The significand is the numerator without its trailing zeros.
			en, _ := end.Parts()
			significand := en.BitLen() - int(en.TrailingZeroBits())
			require.LessOrEqual(t, significand, bits+2, "case %d numerator size", idx)
		}
	}
	// A value on the grid is unchanged, and the operand is not written.
	grid := new(big.Rat).SetFrac(new(big.Int).Lsh(big.NewInt(12345), 700), new(big.Int).Lsh(big.NewInt(1), 900))
	lg := proofarith.LazyOf(grid)
	require.Zero(t, proofarith.LazyRoundDown(lg, bits).Rat().Cmp(grid))
	require.Zero(t, proofarith.LazyRoundUp(lg, bits).Rat().Cmp(grid))
	iv := proofarith.LazyIntervalRoundOut(proofarith.LazyInterval{Lo: proofarith.LazyOf(cases[0]), Hi: proofarith.LazyOf(cases[1])}, bits)
	require.LessOrEqual(t, proofarith.LazyCmp(iv.Lo, proofarith.LazyOf(cases[0])), 0)
	require.GreaterOrEqual(t, proofarith.LazyCmp(iv.Hi, proofarith.LazyOf(cases[1])), 0)
}
