package smoothloft

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/stretchr/testify/require"
)

// TestTwistedSelectCountsFallsBackPastTheCeiling checks that the finest
// target is used while its table fits the triangle ceiling, that each coarser
// target is tried in turn when the finer table does not, and that the loft is
// refused when none fits.
func TestTwistedSelectCountsFallsBackPastTheCeiling(t *testing.T) {
	patches := twistBenchPatches(8)
	row := make([]twistBounds, len(patches))
	for i := range patches {
		row[i] = patches[i].overCommon().bounds()
	}
	bounds := [][]twistBounds{row}

	var nus, nts [][]int
	var triangles []int
	for _, target := range twistTargets {
		nu, nt, err := twistCounts(t.Context(), bounds, target)
		require.NoError(t, err)
		nus, nts = append(nus, nu), append(nts, nt)
		triangles = append(triangles, twistTriangleCount(nu, nt))
	}
	require.Greater(t, triangles[0], triangles[1], "premise: the 0.02 mm table is larger than the 0.03 mm table")
	require.Greater(t, triangles[1], triangles[2], "premise: the 0.03 mm table is larger than the 0.049 mm table")

	for _, tc := range []struct {
		name    string
		ceiling int
		want    int // index of the selected target; -1 for a refusal
	}{
		{"finest fits", triangles[0], 0},
		{"finest one over", triangles[0] - 1, 1},
		{"middle fits", triangles[1], 1},
		{"middle one over", triangles[1] - 1, 2},
		{"coarsest fits", triangles[2], 2},
		{"coarsest one over", triangles[2] - 1, -1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			nu, nt, err := twistSelectCounts(t.Context(), bounds, tc.ceiling)
			if tc.want < 0 {
				require.ErrorIs(t, err, decaderr.ErrUnsupported)
				require.ErrorContains(t, err, "needs more than")
				return
			}
			require.NoError(t, err)
			require.Equal(t, nus[tc.want], nu)
			require.Equal(t, nts[tc.want], nt)
		})
	}

	t.Run("cancelled", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, _, err := twistSelectCounts(ctx, bounds, triangles[0])
		require.ErrorIs(t, err, context.Canceled)
	})
}
