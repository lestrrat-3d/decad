package decad

import (
	"math"
	"testing"

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
	points, bound, err := certifiedCircularBoreRing(t.Context(), radius, angles)
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
	_, _, err = certifiedCircularBoreRing(t.Context(), radius, angles)
	require.ErrorIs(t, err, ErrUnsupported)
}
