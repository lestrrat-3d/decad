package tessellation_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/stretchr/testify/require"
)

// TestCircularStationRoundsEachProduct pins every station of a circle to the
// value with each product rounded before its sum. A fused multiply-add rounds
// once instead, and arm64 fuses an unconverted a + b*c while amd64 does not.
// The fixture is the export golden's bore (radius 5 about (20, 15)), and its
// stations must include some whose fused value differs, or the pin could not
// tell the two apart. On amd64 nothing fuses, so the check bites on arm64.
func TestCircularStationRoundsEachProduct(t *testing.T) {
	t.Parallel()
	w := survey2d.SegmentWalk{CU: 20, CV: 15, Radius: 5, Th0: 0, Th1: 2 * math.Pi}
	const n = 64
	fusedDiffers := 0
	for k := 1; k < n; k++ {
		th := w.Th0 + float64(float64(k)*((w.Th1-w.Th0)/n))
		cos, sin := math.Cos(th), math.Sin(th)
		want := sectionrecord.Point2{U: w.CU + float64(w.Radius*cos), V: w.CV + float64(w.Radius*sin)}
		require.Equal(t, want, tessellation.CircularStation(w, k, n), "station %d", k)
		if math.FMA(w.Radius, cos, w.CU) != want.U || math.FMA(w.Radius, sin, w.CV) != want.V {
			fusedDiffers++
		}
	}
	require.Positive(t, fusedDiffers, "the fixture tells fused from unfused rounding")
}
