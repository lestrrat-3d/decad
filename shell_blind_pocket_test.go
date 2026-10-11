package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestBlindPocketFloorBandHasFourSphereOctants(t *testing.T) {
	t.Parallel()
	budget := proofbound.NewWorkBudget(t.Context())
	pocket := bossRect{3, 3, 7, 7}.profile()
	rounded, err := offsetProfile(budget, pocket, -1, 1)
	require.NoError(t, err)
	delta, err := offsetSectionDelta(budget, pocket, -1, 1, 0)
	require.NoError(t, err)
	require.Zero(t, delta)
	read, err := filletLoopOf(budget, rounded.Outer, 1, "pocket floor", freeform.NewFreeformWork())
	require.NoError(t, err)
	joins, err := filletband.OffsetJoins(budget, read.walks, 1, 0, shellTol)
	require.NoError(t, err)
	collapsed, err := filletband.OffsetLoop(budget, read.walks, joins, 1, shellTol)
	require.NoError(t, err)
	require.Equal(t, pocket.Outer.Segments, collapsed)
	spheres := 0
	for _, walk := range read.walks {
		if filletband.SphereWalk(walk, 1) {
			spheres++
		}
	}
	require.Equal(t, 4, spheres)
}

// A closed mesh alone cannot check the analytic mass formula. Compare its
// exact signed volume with the body's interval and the published occupied-
// volume allowance at two chord tolerances.
func TestBlindPocketShellMeshVolumeProof(t *testing.T) {
	t.Parallel()
	doc := New()
	outer := internalBoxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10)
	tool := internalBoxBodyAtZ(t, doc, 3, 3, 7, 7, 6, 4)
	pocket, err := Cut(t.Context(), outer, tool)
	require.NoError(t, err)
	shell, err := pocket.Shell(t.Context(), Faces(FaceCreatedBy(CapEnd(pocket))).Exactly(1), units.Millimeters(1))
	require.NoError(t, err)
	volume, err := shell.Volume()
	require.NoError(t, err)
	for _, chord := range []float64{0.2, 0.05} {
		mesh, err := tessellateContext(t.Context(), shell, units.Millimeters(chord), VerifyAll)
		require.NoError(t, err)
		require.True(t, mesh.symDiffOK)
		delta := new(big.Rat).Sub(internalMeshVolumeRat(mesh), new(big.Rat).SetFloat64(volume.Value.Base()))
		delta.Abs(delta)
		held, _ := delta.Float64()
		require.LessOrEqual(t, held, proofbound.AbsSumUpper(mesh.volSymDiff, volume.Bound.Base()),
			"chord %.3f signed mesh volume falls outside the occupied-volume proof", chord)
	}
}
