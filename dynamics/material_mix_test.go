package dynamics

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestBodyFrictionMeanBoundsExactProduct(t *testing.T) {
	for _, tc := range []struct {
		name string
		a, b float64
	}{
		{name: "exact root", a: .25, b: 1},
		{name: "unequal nonexact root", a: .2, b: .3},
		{name: "smallest positive", a: math.SmallestNonzeroFloat64, b: math.SmallestNonzeroFloat64},
		{name: "largest finite", a: math.MaxFloat64, b: math.MaxFloat64},
		{name: "wide range", a: math.SmallestNonzeroFloat64, b: math.MaxFloat64},
		{name: "one zero", a: 0, b: math.MaxFloat64},
	} {
		t.Run(tc.name, func(t *testing.T) {
			mu, ok := mixBodyFriction(units.Scalar(tc.a), units.Scalar(tc.b))
			require.True(t, ok)
			product := new(big.Rat).Mul(ratFloat(tc.a), ratFloat(tc.b))
			lowerSquared := new(big.Rat).Mul(mu.lower, mu.lower)
			upperSquared := new(big.Rat).Mul(mu.upper, mu.upper)
			require.LessOrEqual(t, lowerSquared.Cmp(product), 0)
			require.GreaterOrEqual(t, upperSquared.Cmp(product), 0)
			require.LessOrEqual(t, mu.lower.Cmp(exactBase(mu.nominal)), 0)
			require.GreaterOrEqual(t, mu.upper.Cmp(exactBase(mu.nominal)), 0)
			require.True(t, finite(mu.nominal.Base()))
			if mu.lower.Cmp(mu.upper) != 0 {
				lower, _ := mu.lower.Float64()
				upper, _ := mu.upper.Float64()
				require.Equal(t, math.Nextafter(lower, math.Inf(1)), upper)
			}
		})
	}
}
