package proof_test

import (
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// lazyEndsInterval is lazyRandInterval, with point intervals and intervals
// whose ends are equal added to the mix.
func lazyEndsInterval(rng *rand.Rand) proofarith.LazyInterval {
	_, iv := lazyRandInterval(rng)
	if rng.IntN(8) == 0 {
		return proofarith.LazyPoint(lazyRandRat(rng))
	}
	return iv
}

func TestLazyIntervalEndsMatchFullIntervalOps(t *testing.T) {
	rng := rand.New(rand.NewPCG(71, 73))
	patterns := map[[4]int]struct{}{}
	for range 20000 {
		a, b := lazyEndsInterval(rng), lazyEndsInterval(rng)
		patterns[[4]int{a.Lo.Sign(), a.Hi.Sign(), b.Lo.Sign(), b.Hi.Sign()}] = struct{}{}

		mul := proofarith.LazyIntervalMul(a, b)
		require.Zero(t, mul.Lo.Rat().Cmp(proofarith.LazyIntervalMulLo(a, b).Rat()))
		require.Zero(t, mul.Hi.Rat().Cmp(proofarith.LazyIntervalMulHi(a, b).Rat()))

		sq := proofarith.LazyIntervalSquare(a)
		require.Zero(t, sq.Lo.Rat().Cmp(proofarith.LazyIntervalSquareLo(a).Rat()))
		require.Zero(t, sq.Hi.Rat().Cmp(proofarith.LazyIntervalSquareHi(a).Rat()))
	}
	// Every sign pattern a product branches on must have been drawn: the
	// nine (a, b) pairs of {negative, straddling, non-negative}.
	require.GreaterOrEqual(t, len(patterns), 9)
}
