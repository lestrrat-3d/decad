package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCurvedRetainedHeelCapPatchSharesLiveBlankCircles(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	patch, ok, err := revolveLoftFacesHeelRetainedPatchesOf(t.Context(), proof,
		big.NewRat(1, 50))
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, patch.shell.capRings, 2)
	require.NotEmpty(t, patch.mesh.tris)
	require.Equal(t, len(patch.mesh.tris), len(patch.mesh.src))
	require.Equal(t, proof.heelFace.Origins(), patch.mesh.groups[0].origins)
	require.Less(t, patch.chord, 0.1)
	require.Less(t, patch.patchBound, 0.1)
	for _, rows := range patch.rows {
		require.Greater(t, len(rows), 3)
	}
	t.Logf("source heel cap: %d triangles, chord %.6g mm, root curve %.6g mm, total %.6g mm",
		len(patch.mesh.tris), patch.chord, patch.curveBound, patch.patchBound)
}
