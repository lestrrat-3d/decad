package meshbool

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// These tests need the private local charge and the exact-to-held gap reader.
func TestStitchedRoundingVolumeUsesTheSmallerProvenCharge(t *testing.T) {
	m := &StitchedMesh{Round: 0.01, PreArea: 100}
	global := proofbound.SweptVolumeAllow(m.Round, m.PreArea)
	require.Equal(t, global, m.RoundingVolumeBound())
	local := global / 100
	m.localRoundVolume = &local
	require.Equal(t, local, m.RoundingVolumeBound())
	local = global * 2
	require.Equal(t, global, m.RoundingVolumeBound())
}

func TestLocalRoundingUsesEveryWeldedCorner(t *testing.T) {
	// Exact vertices 0 and 3 map to held vertex 0. Its larger gap occurs
	// second, so skipping another contributor would undercharge its facets.
	gaps := []*big.Rat{big.NewRat(1, 1000), big.NewRat(0, 1), big.NewRat(0, 1), big.NewRat(1, 10)}
	remap := []int{0, 1, 2, 0}
	radii, err := heldVertexBoundsFromGaps(t.Context(), gaps, make([]float64, len(gaps)), remap, 3)
	require.NoError(t, err)
	want := proofbound.AbsSumUpper(0, proofbound.Radius3D(proofbound.ProvenUpRound(0.1)))
	require.Equal(t, want, radii[0])
	require.Zero(t, radii[1])
	require.Zero(t, radii[2])
	// Inherited geometry error is a separate charge, never a rounding radius.
	bounds, err := heldVertexBoundsFromGaps(t.Context(), gaps, []float64{5, 6, 7, 8}, remap, 3)
	require.NoError(t, err)
	require.Greater(t, bounds[0], radii[0]+7)

	verts := []r3.Vec{{}, {X: 1}, {Y: 1}}
	_, volume, err := proofbound.PerturbedAreaAndRoundingVolumeContext(t.Context(), verts, [][3]int{{0, 1, 2}}, want, radii)
	require.NoError(t, err)
	require.Greater(t, volume, 0.05, "the larger welded corner's gap must be charged")
}
