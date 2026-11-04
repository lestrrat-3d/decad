package revolvemesh_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/stretchr/testify/require"
)

// TestSectorRingMaxStepStationCap holds the station cap to the joins a fitted
// route builds: a ring of 2,368 stations, the count a 32-tooth straight bevel
// gear holds at the fitted route's starting sag, has its largest step read, and a
// ring one station past the cap does not.
func TestSectorRingMaxStepStationCap(t *testing.T) {
	t.Parallel()
	ring := func(n int) []float64 {
		angles := make([]float64, n)
		for i := range angles {
			angles[i] = 2 * math.Pi * float64(i) / float64(n)
		}
		return angles
	}
	for _, n := range []int{2_304, 2_368, revolvemesh.SectorBoreMaxStations} {
		step, ok := revolvemesh.SectorRingMaxStep(ring(n))
		require.True(t, ok, "%d stations", n)
		require.InDelta(t, 2*math.Pi/float64(n), step, 1e-12, "%d stations", n)
	}
	_, ok := revolvemesh.SectorRingMaxStep(ring(revolvemesh.SectorBoreMaxStations + 1))
	require.False(t, ok)
}
