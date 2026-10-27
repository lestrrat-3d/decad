package proofbound_test

import (
	"context"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRoundingVolumeChargesCollapsedFacet(t *testing.T) {
	// The source triangle (0,0,0), (1,0,0), (0,1,0) moves to these
	// collinear held corners. Along its straight homotopy, z=t and the
	// projected area is (1-t)/2. It sweeps volume integral_0^1 (1-t)/2 = 1/4.
	verts := []r3.Vec{{Z: 1}, {X: 1, Z: 1}, {X: 0.5, Z: 1}}
	tris := [][3]int{{0, 1, 2}}
	_, volume, err := proofbound.PerturbedAreaAndRoundingVolumeContext(
		t.Context(), verts, tris, 1.5, []float64{1, 1, 1.5})
	require.NoError(t, err)
	require.True(t, volume >= 0.25 && !math.IsInf(volume, 1), "swept volume bound %g", volume)
}

func TestRoundingVolumeDoesNotChargeUnmovedFacets(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {X: 10}, {X: 11}, {X: 10, Y: 1}}
	radii := []float64{0.001, 0, 0, 0, 0, 0}
	tris := [][3]int{{0, 1, 2}}
	_, one, err := proofbound.PerturbedAreaAndRoundingVolumeContext(t.Context(), verts, tris, radii[0], radii)
	require.NoError(t, err)
	for range 128 {
		tris = append(tris, [3]int{3, 4, 5})
	}
	area, many, err := proofbound.PerturbedAreaAndRoundingVolumeContext(t.Context(), verts, tris, radii[0], radii)
	require.NoError(t, err)
	requireBits(t, one, many, "unmoved facets must add no rounding charge")
	require.Positive(t, many)
	require.Less(t, many, proofbound.SweptVolumeAllow(radii[0], area)/100)

	_, zero, err := proofbound.PerturbedAreaAndRoundingVolumeContext(t.Context(), verts, tris, 0, make([]float64, len(verts)))
	require.NoError(t, err)
	require.Zero(t, zero)
}

func TestRoundingVolumeSharesTheGlobalAreaProof(t *testing.T) {
	for _, n := range []int{7, 3000} {
		verts, tris := randomMesh(rand.New(rand.NewPCG(31, 32)), n, n)
		const delta = 3e-4
		radii := make([]float64, len(verts))
		for i := range radii {
			radii[i] = delta
		}
		area, volume, err := proofbound.PerturbedAreaAndRoundingVolumeContext(t.Context(), verts, tris, delta, radii)
		require.NoError(t, err)
		want, err := proofbound.PerturbedAreaUpperContext(t.Context(), verts, tris, delta)
		require.NoError(t, err)
		requireBits(t, want, area, "global area")
		// Every triangle translates normally by at most delta. Even one
		// unchanged triangle's area times delta must fit the total bound.
		tri := tris[0]
		heldArea := proofbound.PerturbedTriangleAreaUpper(verts[tri[0]], verts[tri[1]], verts[tri[2]], 0)
		require.GreaterOrEqual(t, volume, delta*heldArea)
		require.False(t, math.IsInf(volume, 1))
	}
}

func TestRoundingVolumeRefusesInvalidClaims(t *testing.T) {
	for _, tc := range []struct {
		name   string
		global float64
		radii  []float64
		verts  []r3.Vec
	}{
		{"missing radius", 1, []float64{1, 1}, []r3.Vec{{}, {X: 1}, {Y: 1}}},
		{"negative radius", 1, []float64{1, -1, 1}, []r3.Vec{{}, {X: 1}, {Y: 1}}},
		{"nan radius", 1, []float64{1, math.NaN(), 1}, []r3.Vec{{}, {X: 1}, {Y: 1}}},
		{"infinite radius", 1, []float64{1, math.Inf(1), 1}, []r3.Vec{{}, {X: 1}, {Y: 1}}},
		{"negative global", -1, []float64{1, 1, 1}, []r3.Vec{{}, {X: 1}, {Y: 1}}},
		{"nan global", math.NaN(), []float64{1, 1, 1}, []r3.Vec{{}, {X: 1}, {Y: 1}}},
		{"infinite global", math.Inf(1), []float64{1, 1, 1}, []r3.Vec{{}, {X: 1}, {Y: 1}}},
		{"nonfinite vertex", 1, []float64{1, 1, 1}, []r3.Vec{{}, {X: math.Inf(1)}, {Y: 1}}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			area, volume, err := proofbound.PerturbedAreaAndRoundingVolumeContext(t.Context(), tc.verts, [][3]int{{0, 1, 2}}, tc.global, tc.radii)
			require.NoError(t, err)
			require.True(t, math.IsInf(area, 1))
			require.True(t, math.IsInf(volume, 1))
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, _, err := proofbound.PerturbedAreaAndRoundingVolumeContext(ctx, nil, nil, 0, nil)
	require.ErrorIs(t, err, context.Canceled)
}

func TestRoundingVolumePreservesSubnormalMovement(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}}
	_, volume, err := proofbound.PerturbedAreaAndRoundingVolumeContext(
		t.Context(), verts, [][3]int{{0, 1, 2}}, math.SmallestNonzeroFloat64,
		[]float64{math.SmallestNonzeroFloat64, 0, 0})
	require.NoError(t, err)
	require.Positive(t, volume)
}
