package revolvemesh_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/stretchr/testify/require"
)

func TestSectorCapFanRequiresAdjacentCompleteSourceRing(t *testing.T) {
	positions := map[int]int{10: 0, 11: 1, 12: 2, 13: 3}
	cell, fan, err := revolvemesh.SectorCapFan([3]int{2, 11, 10}, 7, 2, 7, positions, 4)
	require.NoError(t, err)
	require.True(t, fan)
	require.Equal(t, 0, cell)
	for _, tc := range []struct {
		tri   [3]int
		group int
	}{
		{tri: [3]int{2, 10, 12}, group: 7},
		{tri: [3]int{2, 10, 11}, group: 8},
		{tri: [3]int{2, 10, 99}, group: 7},
		{tri: [3]int{2, 2, 10}, group: 7},
	} {
		_, _, err := revolvemesh.SectorCapFan(tc.tri, tc.group, 2, 7, positions, 4)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	}
}

func TestSectorRingMaxStep(t *testing.T) {
	ring := func(n int, step float64) []float64 {
		angles := make([]float64, n)
		for k := range angles {
			angles[k] = step * float64(k)
		}
		return angles
	}
	// The last station sits at 5.85, so the wrap step 2*pi-5.85 is the widest.
	step, ok := revolvemesh.SectorRingMaxStep(ring(16, 0.39))
	require.True(t, ok)
	require.Greater(t, step, 0.43)
	require.Less(t, step, 0.44)

	_, ok = revolvemesh.SectorRingMaxStep(ring(15, 0.41))
	require.False(t, ok)

	repeated := ring(16, 0.39)
	repeated[3] = repeated[2]
	_, ok = revolvemesh.SectorRingMaxStep(repeated)
	require.False(t, ok)

	notFinite := ring(16, 0.39)
	notFinite[0] = math.NaN()
	_, ok = revolvemesh.SectorRingMaxStep(notFinite)
	require.False(t, ok)

	_, ok = revolvemesh.SectorRingMaxStep(ring(revolvemesh.SectorBoreMaxStations+1, 0.001))
	require.False(t, ok)
}

func TestSectorBorePatches(t *testing.T) {
	src := []int{5, 7, 5, 7, 9}
	patch := func(axes ...int) loftmesh.ProjectedPatch {
		return loftmesh.ProjectedPatch{Triangles: axes, Axes: [2]int{1, 2}}
	}
	full := revolvemesh.SectorBorePatches(src, 5, 4, false, 7, 8, -1, 6)
	require.Equal(t, []loftmesh.ProjectedPatch{
		patch(0, 2),
		patch(5, 6, 11, 12),
		patch(17, 18, 23, 24),
		patch(7, 8, 13, 14),
		patch(19, 20, 25, 26),
	}, full)

	band := revolvemesh.SectorBorePatches(src, 5, 4, true, 7, 8, -1, 6)
	require.Equal(t, []loftmesh.ProjectedPatch{full[0], full[3], full[4]}, band)
}
