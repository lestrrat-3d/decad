package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestFacetedRetainedBlendSlantedEndBound builds the notched round's back root with the
// ear's outer face, the end plane where the root's arc ends, moved toward the round's
// edge, so the round meets the plane at a smaller and smaller sine s (0.71 down to
// 0.53). A cut point on that plane lies within the held bound β of the true wall and
// within the true wall's slant of the true boundary, which is at least β/s away
// along the wall (retained-edge-blend §4.3, the slant of the crossing bound). The
// mesh's published bound is the largest vertex bound, so it must reach β/s for the
// largest held β on the round. Drop the slant term from the crossing and the rows
// with s ≤ 0.6 fall short of it.
func TestFacetedRetainedBlendSlantedEndBound(t *testing.T) {
	t.Parallel()

	for _, x0 := range []float64{0.6, 0.4, 0.3} {
		t.Run(fmt.Sprintf("ear at x = %v", x0), func(t *testing.T) {
			t.Parallel()
			c := notchedRoots[1]
			c.ear[0] = x0
			offset := notchShortR - x0
			sine := math.Sqrt(1 - (offset/c.radius)*(offset/c.radius))
			body := notchedBody(t, c.ear)
			held, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)

			out, err := decad.Fillet(t.Context(), body, c.seed(), units.Millimeters(1), decad.WithTangentChain())
			require.NoError(t, err)

			mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			require.GreaterOrEqual(t, revolveMeshBound(t, mesh), revolveMeshBound(t, held)/sine,
				"the bound reaches the held bound over the slant's sine %.4f", sine)
		})
	}
}
