package filletband_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// The blind-pocket shell's floor band (internal/stackedshell) reads the
// pocket's rounded offset as fillet walks. Their offset loop must collapse
// back to the source square, and each of the four corners must be a sphere.
func TestBlindPocketFloorBandHasFourSphereOctants(t *testing.T) {
	t.Parallel()
	budget := proofbound.NewWorkBudget(t.Context())
	pocket := prismshell.BossRect{U0: 3, V0: 3, U1: 7, V1: 7}.Profile()
	rounded, err := offset2d.OffsetProfile(budget, pocket, -1, 1, offset2d.ShellTol)
	require.NoError(t, err)
	delta, err := offset2d.OffsetSectionDelta(budget, pocket, -1, 1, 0, offset2d.ShellTol)
	require.NoError(t, err)
	require.Zero(t, delta)
	read, err := breprecord.FilletLoopOf(budget, rounded.Outer, 1, "pocket floor", freeform.NewFreeformWork())
	require.NoError(t, err)
	joins, err := filletband.OffsetJoins(budget, read.Walks, 1, 0, offset2d.ShellTol)
	require.NoError(t, err)
	collapsed, err := filletband.OffsetLoop(budget, read.Walks, joins, 1, offset2d.ShellTol)
	require.NoError(t, err)
	require.Equal(t, pocket.Outer.Segments, collapsed)
	spheres := 0
	for _, walk := range read.Walks {
		if filletband.SphereWalk(walk, 1) {
			spheres++
		}
	}
	require.Equal(t, 4, spheres)
}
