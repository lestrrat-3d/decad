package mirrorjoin_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/mirrorjoin"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPlaneDeltaAdmitted(t *testing.T) {
	t.Parallel()
	origin := r3.NewVec(0, 0, 20)
	for name, tc := range map[string]struct {
		delta float64
		want  bool
	}{
		"exact":                {0, true},
		"unit conversion":      {2.9e-16, true},
		"within the allowance": {1e-15, true},
		"stated displacement":  {1e-6, false},
		"just past":            {64 * 0x1p-52 * 20, false},
		"negative":             {-1e-16, false},
		"infinite":             {math.Inf(1), false},
		"NaN":                  {math.NaN(), false},
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, mirrorjoin.PlaneDeltaAdmitted(tc.delta, origin))
		})
	}
}
