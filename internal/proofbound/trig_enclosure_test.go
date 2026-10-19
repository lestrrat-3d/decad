package proofbound_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestAtanSmallIntervalContainsExactSeries(t *testing.T) {
	for _, x := range []float64{-0.5, -1.0 / 3, -1e-9, 0, 1e-9, 1.0 / 3, 0.5} {
		r := new(big.Rat).SetFloat64(x)
		got := proofbound.AtanSmallInterval(r)
		want := exactAtanSeries(r)
		require.LessOrEqual(t, got.Lo.Cmp(want.Lo), 0)
		require.GreaterOrEqual(t, got.Hi.Cmp(want.Hi), 0)
	}
}

func TestTurnSinCosIntervalEnclosesMathSincos(t *testing.T) {
	turns := []float64{0, 1.0 / 12, 1.0 / 8, 0.25, 0.5, 0.75, 1}
	for i := 1; i <= 64; i++ {
		turns = append(turns, float64(i)/64)
	}
	for _, turn := range turns {
		sin, cos := proofbound.TurnSinCosInterval(new(big.Rat).SetFloat64(turn))
		wantSin, wantCos := math.Sincos(2 * math.Pi * turn)
		requireIntervalContainsFloat(t, sin, wantSin)
		requireIntervalContainsFloat(t, cos, wantCos)
	}
}

func exactAtanSeries(x *big.Rat) proofbound.RatInterval {
	if x.Sign() < 0 {
		return proofbound.IntervalNeg(exactAtanSeries(new(big.Rat).Neg(x)))
	}
	x2 := new(big.Rat).Mul(x, x)
	power := new(big.Rat).Set(x)
	sum := new(big.Rat)
	for n := range 64 {
		term := new(big.Rat).Quo(power, big.NewRat(int64(2*n+1), 1))
		if n%2 == 0 {
			sum.Add(sum, term)
		} else {
			sum.Sub(sum, term)
		}
		power.Mul(power, x2)
	}
	return proofbound.Interval(sum, new(big.Rat).Add(sum, new(big.Rat).Quo(power, big.NewRat(129, 1))))
}

func requireIntervalContainsFloat(t *testing.T, iv proofbound.RatInterval, value float64) {
	t.Helper()
	got := new(big.Rat).SetFloat64(value)
	ulp := math.Nextafter(value, math.Inf(1)) - value
	slack := new(big.Rat).SetFloat64(math.Abs(ulp)*2 + 1e-15)
	require.LessOrEqual(t, iv.Lo.Cmp(new(big.Rat).Add(got, slack)), 0)
	require.GreaterOrEqual(t, iv.Hi.Cmp(new(big.Rat).Sub(got, slack)), 0)
}
