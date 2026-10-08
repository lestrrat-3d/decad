package proofbound_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// requireEncloses asserts lo ≤ want ≤ hi for a float reference within its
// own last-ulp error: math.Log and math.Asinh promise no correct rounding, so
// the reference is widened by four ulps before it is compared.
func requireEncloses(t *testing.T, iv proofbound.RatInterval, want float64, msg string) {
	t.Helper()
	slack := 4 * proofbound.UlpOf(want)
	lo := proofarith.FloatRat(want - slack)
	hi := proofarith.FloatRat(want + slack)
	require.LessOrEqual(t, iv.Lo.Cmp(hi), 0, msg)
	require.GreaterOrEqual(t, iv.Hi.Cmp(lo), 0, msg)
}

func requireNarrow(t *testing.T, iv proofbound.RatInterval, msg string) {
	t.Helper()
	width := new(big.Rat).Sub(iv.Hi, iv.Lo)
	require.GreaterOrEqual(t, width.Sign(), 0, msg)
	limit := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 150))
	require.Negative(t, width.Cmp(limit), msg)
}

func TestLnIntervalEnclosesTheLogarithm(t *testing.T) {
	t.Run("ln 1 is exactly zero", func(t *testing.T) {
		got, ok := proofbound.LnInterval(proofbound.PointInterval(big.NewRat(1, 1)))
		require.True(t, ok)
		require.Zero(t, got.Lo.Sign())
		require.Zero(t, got.Hi.Sign())
	})
	t.Run("ln 2 matches the held constant", func(t *testing.T) {
		got, ok := proofbound.LnInterval(proofbound.PointInterval(big.NewRat(2, 1)))
		require.True(t, ok)
		requireEncloses(t, got, math.Ln2, "ln 2")
		requireNarrow(t, got, "ln 2")
		ln2 := proofbound.Ln2Interval()
		require.Zero(t, got.Lo.Cmp(ln2.Lo))
		require.Zero(t, got.Hi.Cmp(ln2.Hi))
	})
	t.Run("a non-positive argument is refused", func(t *testing.T) {
		_, ok := proofbound.LnInterval(proofbound.Interval(big.NewRat(-1, 1), big.NewRat(1, 1)))
		require.False(t, ok)
		_, ok = proofbound.LnInterval(proofbound.PointInterval(new(big.Rat)))
		require.False(t, ok)
	})
	t.Run("random rationals", func(t *testing.T) {
		r := rand.New(rand.NewPCG(7, 11))
		for range 200 {
			x := math.Exp(r.Float64()*80 - 40)
			got, ok := proofbound.LnInterval(proofbound.PointInterval(proofarith.FloatRat(x)))
			require.True(t, ok)
			requireEncloses(t, got, math.Log(x), "ln")
			requireNarrow(t, got, "ln")
		}
	})
}

func TestAsinhIntervalEnclosesTheInverseSine(t *testing.T) {
	t.Run("asinh 0 is exactly zero", func(t *testing.T) {
		got := proofbound.AsinhInterval(proofbound.PointInterval(new(big.Rat)))
		require.Zero(t, got.Lo.Sign())
		require.Zero(t, got.Hi.Sign())
	})
	t.Run("asinh is odd", func(t *testing.T) {
		pos := proofbound.AsinhInterval(proofbound.PointInterval(big.NewRat(3, 7)))
		neg := proofbound.AsinhInterval(proofbound.PointInterval(big.NewRat(-3, 7)))
		require.Zero(t, pos.Lo.Cmp(new(big.Rat).Neg(neg.Hi)))
		require.Zero(t, pos.Hi.Cmp(new(big.Rat).Neg(neg.Lo)))
	})
	t.Run("random rationals", func(t *testing.T) {
		r := rand.New(rand.NewPCG(3, 5))
		for range 200 {
			x := (r.Float64()*2 - 1) * math.Exp(r.Float64()*20-10)
			got := proofbound.AsinhInterval(proofbound.PointInterval(proofarith.FloatRat(x)))
			requireEncloses(t, got, math.Asinh(x), "asinh")
			requireNarrow(t, got, "asinh")
		}
	})
}

func TestSqrtFixedEnclosesTheRoot(t *testing.T) {
	t.Run("a perfect square answers exactly", func(t *testing.T) {
		got, ok := proofbound.SqrtFixed(big.NewRat(9, 4))
		require.True(t, ok)
		require.Zero(t, got.Lo.Cmp(big.NewRat(3, 2)))
		require.Zero(t, got.Hi.Cmp(big.NewRat(3, 2)))
	})
	t.Run("an irrational root is bracketed", func(t *testing.T) {
		got, ok := proofbound.SqrtFixed(big.NewRat(2, 1))
		require.True(t, ok)
		lo2 := new(big.Rat).Mul(got.Lo, got.Lo)
		hi2 := new(big.Rat).Mul(got.Hi, got.Hi)
		require.Negative(t, lo2.Cmp(big.NewRat(2, 1)))
		require.Positive(t, hi2.Cmp(big.NewRat(2, 1)))
		requireNarrow(t, got, "sqrt 2")
	})
	t.Run("a negative argument is refused", func(t *testing.T) {
		_, ok := proofbound.SqrtFixed(big.NewRat(-1, 3))
		require.False(t, ok)
	})
}
