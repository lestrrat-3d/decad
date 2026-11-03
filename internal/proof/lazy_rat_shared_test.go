package proof_test

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// TestLazyZeroSharedIntegersStayUnwritten pins the contract that lets every
// zero LazyRat share one numerator: no operation writes an operand's
// integers, so the shared zero reads as 0/1 after being fed to each one.
func TestLazyZeroSharedIntegersStayUnwritten(t *testing.T) {
	zero := proofarith.LazyIntervalMul(proofarith.LazyPoint(big.NewRat(0, 1)), proofarith.LazyPoint(big.NewRat(3, 1))).Lo
	other := proofarith.LazyOf(big.NewRat(-7, 12))
	odd := proofarith.LazyOf(big.NewRat(8, 24))
	zi := proofarith.LazyInterval{Lo: zero, Hi: zero}
	xi := proofarith.LazyInterval{Lo: other, Hi: odd}
	n, d := zero.Parts()
	for range 3 {
		_ = proofarith.LazyAdd(zero, other)
		_ = proofarith.LazySub(zero, other)
		_ = proofarith.LazySub(other, zero)
		_ = proofarith.LazyMul(zero, other)
		_ = proofarith.LazyNeg(zero)
		_ = proofarith.LazyCmp(zero, other)
		_, _ = proofarith.LazyQuo(zero, other), zero.Rat()
		_ = proofarith.LazyIntervalAdd(zi, xi)
		_ = proofarith.LazyIntervalSub(zi, xi)
		_ = proofarith.LazyIntervalSub(xi, zi)
		_ = proofarith.LazyIntervalMul(xi, xi)
		_ = proofarith.LazyIntervalSquare(xi)
		_, _ = proofarith.LazyIntervalQuo(zi, xi)
		_, _, _ = zero.Float64()
		_ = proofarith.LazyFloatError(zero, 0.5)
		// A fresh zero from a straddling square shares the same integers.
		sq := proofarith.LazyIntervalSquare(proofarith.LazyInterval{Lo: proofarith.LazyOf(big.NewRat(-2, 3)), Hi: proofarith.LazyOf(big.NewRat(1, 2))})
		sn, sd := sq.Lo.Parts()
		require.Zero(t, sn.Sign())
		require.Zero(t, sd.Cmp(big.NewInt(1)))
	}
	require.Zero(t, n.Sign())
	require.Zero(t, d.Cmp(big.NewInt(1)))
	require.Zero(t, zero.Rat().Sign())
	n2, d2 := proofarith.LazyIntervalMul(zi, xi).Lo.Parts()
	require.Zero(t, n2.Sign())
	require.Zero(t, d2.Cmp(big.NewInt(1)))
}
