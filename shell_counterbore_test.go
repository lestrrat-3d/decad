package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The exact signed tetrahedron sum must fit the independently integrated
// shell-volume interval plus the mesh's published occupied-volume allowance.
func TestCounterboreShellMeshVolumeProof(t *testing.T) {
	doc := New()
	plate := internalBoxBody(t, doc, -10, -10, 10, 10, 10)
	through := internalCircleBody(t, doc, 0, 3, 0, Distance{D: units.Millimeters(10), Dir: Along})
	part, err := Cut(t.Context(), plate, through)
	require.NoError(t, err)
	wide := internalCircleBody(t, doc, 0, 5, 7, Distance{D: units.Millimeters(3), Dir: Along})
	part, err = Cut(t.Context(), part, wide)
	require.NoError(t, err)
	shell, err := part.Shell(t.Context(), Faces(FaceCreatedBy(CapEnd(part))).Exactly(1), units.Millimeters(1))
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
