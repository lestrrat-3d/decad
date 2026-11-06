package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// TestLoftMirrorJoinCapChamfer chamfers both outer cap loops of a rounded
// twisted gear loft joined to its mirror image, the construction of a
// herringbone gear. Each band removes the triangle of legs d along the cap
// outline it gives up, to within the lean of its short side, so the removed
// held volume is d/2 times the cap area lost; the middle ring stays. The short
// half turns its walls far enough that the side ring at depth d turns past the
// inset loop in projection.
func TestLoftMirrorJoinCapChamfer(t *testing.T) {
	const d = 0.1
	twist := 14.5 * math.Pi / 180
	for _, tc := range []struct {
		name string
		half float64
	}{{"half 1.5", 1.5}, {"half 0.5", 0.5}} {
		joined := roundedMirrorJoin(t, 5, tc.half, twist)
		held := heldVolume(t, joined)
		capsBefore, _ := capAreaTotal(t, joined)
		chamfered, err := decad.Chamfer(t.Context(), joined, outerCapLoops(joined), units.Millimeters(d))
		require.NoError(t, err, tc.name)
		after, err := chamfered.Volume()
		require.NoError(t, err, tc.name)
		capsAfter, _ := capAreaTotal(t, chamfered)
		shrink := capsBefore - capsAfter
		require.Positive(t, shrink, tc.name)
		require.InEpsilon(t, d/2*shrink, held-after.Value.Base(), 0.03, tc.name)

		mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err, tc.name)
		require.True(t, mesh.BoundaryVerified(), tc.name)
		require.True(t, mesh.VolumeVerified(), tc.name)
		require.Equal(t, 2, loftMirrorChamferEuler(mesh), "the chamfered shell must close like a sphere: %s", tc.name)
		top := 2 * tc.half
		for _, p := range mesh.Vertices() {
			on := false
			for _, level := range []float64{0, d, tc.half, top - d, top} {
				on = on || math.Abs(p.Z-level) <= 1e-12
			}
			require.True(t, on, "%s: vertex z=%g is off the cap, band and middle levels", tc.name, p.Z)
		}
	}
}
