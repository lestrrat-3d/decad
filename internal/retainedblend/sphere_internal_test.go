package retainedblend

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/stretchr/testify/require"
)

// TestCrossing pins the grazing gate of the plane-sphere rim
// (agent-docs/retained-edge-blend-design.md §4.5): a mesh Boolean cannot place a
// plane within 1e-7 of a sphere's tangent plane, so the gate is shown on the
// rim's numbers directly.
func TestCrossing(t *testing.T) {
	t.Parallel()
	const radius = 2.0
	require.NoError(t, crossing(3, radius), "a 60° crossing")
	for _, rr := range []float64{0, -1, (0.5 * minSine * radius) * (0.5 * minSine * radius)} {
		err := crossing(rr, radius)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "the plane meets the sphere at a grazing angle")
	}
}
