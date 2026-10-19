package momentline_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentline"
	"github.com/stretchr/testify/require"
)

func TestRatLerpEndpointsMatchGeneralFormula(t *testing.T) {
	for _, start := range []float64{0, 1, -1, 1e-300, 1e300, math.MaxFloat64} {
		for _, end := range []float64{0, 1, -1, 1e-300, math.MaxFloat64} {
			for _, at := range []float64{0, 1} {
				got := momentline.RatLerp(start, end, at)
				want := new(big.Rat).Add(new(big.Rat).SetFloat64(start), new(big.Rat).Mul(
					new(big.Rat).SetFloat64(at), new(big.Rat).Sub(new(big.Rat).SetFloat64(end), new(big.Rat).SetFloat64(start))))
				require.NotNil(t, got)
				require.Zero(t, got.Cmp(want))
			}
		}
	}
	require.Nil(t, momentline.RatLerp(3, math.NaN(), 0))
	require.Nil(t, momentline.RatLerp(math.Inf(1), 7, 1))
}

func TestRatLerpEndpointReturnsFreshRational(t *testing.T) {
	a := momentline.RatLerp(2, 5, 0)
	require.NotNil(t, a)
	a.Sub(a, big.NewRat(1, 1))
	b := momentline.RatLerp(2, 5, 0)
	require.Equal(t, big.NewRat(2, 1), b)
}
