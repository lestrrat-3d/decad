package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Compare the mesh's exact signed tetrahedron sum with the round boss's
// analytic volume interval and the occupied-volume bound at two chord sizes.
func TestRoundBossShellMeshVolumeProof(t *testing.T) {
	t.Parallel()
	stack := internalRoundBoss(t)
	shell, err := stack.Shell(t.Context(), Faces(FaceCreatedBy(CapEnd(stack))).Exactly(1), units.Millimeters(1))
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
