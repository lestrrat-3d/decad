package decad

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// TestCupPayloadForTracksEachSourceEndDisplacement covers both opening ends and
// shell senses. The top end is computed while the bottom is stated exactly, so
// a floor derived from the latter must not inherit the former's displacement.
func TestCupPayloadForTracksEachSourceEndDisplacement(t *testing.T) {
	t.Parallel()
	const (
		thickness      = 0.1
		thicknessDelta = 0.03125
	)
	pp := prismPayload{
		z0:      0,
		z1:      1e12,
		z1Delta: 0.25,
	}
	floorDelta := func(from, sourceDelta, by float64) float64 {
		to := from + by
		return absSumUpper(sourceDelta, thicknessDelta, addRoundError(from, by, to))
	}
	topFloorDelta := floorDelta(pp.z0, pp.z0Delta, thickness)
	bottomInFloorDelta := floorDelta(pp.z1, pp.z1Delta, -thickness)
	bottomOutFloorDelta := floorDelta(pp.z1, pp.z1Delta, thickness)

	tests := []struct {
		name                            string
		sense                           float64
		removedEnd                      bool
		openDelta, outerDelta, cavDelta float64
	}{
		{
			name: "top inward", sense: 1, removedEnd: true,
			openDelta: pp.z1Delta, outerDelta: pp.z0Delta, cavDelta: topFloorDelta,
		},
		{
			name: "top outward", sense: -1, removedEnd: true,
			openDelta: pp.z1Delta, outerDelta: topFloorDelta, cavDelta: pp.z0Delta,
		},
		{
			name: "bottom inward", sense: 1,
			openDelta: pp.z0Delta, outerDelta: pp.z1Delta, cavDelta: bottomInFloorDelta,
		},
		{
			name: "bottom outward", sense: -1,
			openDelta: pp.z0Delta, outerDelta: bottomOutFloorDelta, cavDelta: pp.z1Delta,
		},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			cp := cupPayloadFor(pp, ProfileRecord{}, tt.sense, thickness, thicknessDelta, tt.removedEnd)
			require.Equal(t, tt.openDelta, cp.openScalar().bound)
			require.Equal(t, tt.outerDelta, cp.outerScalar().bound)
			require.Equal(t, tt.cavDelta, cp.cavityScalar().bound)

			requireCupPrismLevelBounds(t, cp.outerPrism(), cp.outerScalar(), cp.openScalar())
			requireCupPrismLevelBounds(t, cp.cavityPrism(), cp.cavityScalar(), cp.openScalar())
		})
	}
}

func requireCupPrismLevelBounds(t *testing.T, prism prismPayload, a, b boundedScalar) {
	t.Helper()
	if a.value <= b.value {
		require.Equal(t, a.bound, prism.z0Delta)
		require.Equal(t, b.bound, prism.z1Delta)
		return
	}
	require.Equal(t, b.bound, prism.z0Delta)
	require.Equal(t, a.bound, prism.z1Delta)
}
