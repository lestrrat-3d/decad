package circularbounds_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/stretchr/testify/require"
)

func TestCircularBoreRingBoundsShiftedStationChord(t *testing.T) {
	const count = 256
	const radius = 5.0
	const phase = -0.01
	angles := make([]float64, count)
	for i := range angles {
		angles[i] = phase + 2*math.Pi*float64(i)/count
	}
	points, bound, err := circularbounds.BoreRing(t.Context(), radius, angles)
	require.NoError(t, err)
	require.Len(t, points, count)
	require.Greater(t, bound, 0.0)
	for i, point := range points {
		next := points[(i+1)%count]
		midU, midV := (point.U+next.U)/2, (point.V+next.V)/2
		require.LessOrEqual(t, radius-math.Hypot(midU, midV), bound)
	}
	// A repeated phase leaves a zero-width chord and cannot certify a ring.
	angles[1] = angles[0]
	_, _, err = circularbounds.BoreRing(t.Context(), radius, angles)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

func TestCircularBoreRingAcceptsCertifiedWiderStepAndBoundsEveryChord(t *testing.T) {
	const count = 64
	const radius = 1.0
	angles := make([]float64, count)
	for i := range angles {
		angles[i] = -0.03 + 2*math.Pi*float64(i)/count
	}
	_, _, err := circularbounds.BoreRing(t.Context(), radius, angles)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	points, bound, err := circularbounds.BoreRingAtStep(t.Context(), radius, angles, 0.1)
	require.NoError(t, err)
	require.Less(t, bound, 0.1)
	for i, point := range points {
		next := points[(i+1)%count]
		midU, midV := (point.U+next.U)/2, (point.V+next.V)/2
		require.LessOrEqual(t, radius-math.Hypot(midU, midV), bound)
	}
	_, _, err = circularbounds.BoreRingAtStep(t.Context(), radius, angles, 0.08)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}
