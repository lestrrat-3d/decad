package retainedblend

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestAzimuthStepRefusesNonNumbers pins azimuthStep on inputs that are not a
// usable radius or budget: each gives 0, which columnCount refuses, never the
// coarsest step π.
func TestAzimuthStepRefusesNonNumbers(t *testing.T) {
	t.Parallel()
	// 2·sin²(π/8) = 1 − cos(π/4): a budget of rho·(1 − cos(π/4)) allows Δφ = π/2.
	quarter := 1 - math.Sqrt2/2
	tests := []struct {
		name        string
		budget, rho float64
		want        float64
		eps         float64
	}{
		{name: "NaN radius", budget: 1, rho: math.NaN(), want: 0},
		{name: "NaN budget", budget: math.NaN(), rho: 1, want: 0},
		{name: "negative radius", budget: 1, rho: -1, want: 0},
		{name: "infinite radius", budget: 1, rho: math.Inf(1), want: 0},
		{name: "infinite budget", budget: math.Inf(1), rho: 1, want: math.Pi},
		{name: "zero budget", budget: 0, rho: 1, want: 0},
		{name: "negative budget", budget: -1, rho: 1, want: 0},
		{name: "budget past the radius", budget: 3, rho: 1, want: math.Pi},
		{name: "quarter turn", budget: 3 * quarter, rho: 3, want: math.Pi / 2, eps: 1e-4},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			got := azimuthStep(tc.budget, tc.rho)
			if tc.eps == 0 {
				require.Equal(t, tc.want, got)
				return
			}
			require.InEpsilon(t, tc.want, got, tc.eps)
		})
	}
}
