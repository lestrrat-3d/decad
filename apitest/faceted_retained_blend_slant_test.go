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

			out, err := decad.Fillet(body, c.seed(), units.Millimeters(1)).TangentChain().Do(t.Context())
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

// TestFacetedRetainedBlendArcSlantedEndBound is the arc route's counterpart of the test
// above: earBody with the rib widened so its side faces y = ±half, the arc's end planes,
// meet the round (r 3) at sine √(1 − (half/3)²), 0.60 and 0.50 for half 2.4 and 2.6. The
// mesh bound must reach the held bound over the sine. Drop the slant from the arc
// route's end planes (arc_band.go) and the rows at half 2.4 with size 0.3 and at half 2.6
// fall short of it.
func TestFacetedRetainedBlendArcSlantedEndBound(t *testing.T) {
	t.Parallel()

	for _, half := range []float64{2.4, 2.6} {
		for _, size := range []float64{1, 0.3} {
			t.Run(fmt.Sprintf("half %v size %v", half, size), func(t *testing.T) {
				t.Parallel()
				sine := math.Sqrt(1 - (half/3)*(half/3))
				sc := newRetentionScene()
				bar := xCylinderAt(t, sc.doc, 0, 0, 3, -15, 15)
				box := placed(t, rectBox(t, sc.w, sc.doc, 0, -half, earLen, half, 4), -5)
				body, err := decad.Union(bar, box).Do(t.Context())
				require.NoError(t, err)
				held, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
				require.NoError(t, err)

				out, err := decad.Fillet(body, earArc(t, body), units.Millimeters(size)).Do(t.Context())
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
}
