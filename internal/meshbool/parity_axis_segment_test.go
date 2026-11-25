package meshbool_test

// CI: isolated-tests

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/stretchr/testify/require"
)

// TestParityAxisSegmentFilterMatchesTheUnfilteredScan requires the box filter,
// with its axis-parallel segment rejection, to return what the unfiltered scan
// and the indexed scan return, error text included, on prisms whose caps
// project to axis-parallel segments.
func TestParityAxisSegmentFilterMatchesTheUnfilteredScan(t *testing.T) {
	rng := rand.New(rand.NewPCG(3, 4))
	const height = 1.5
	for _, n := range []int{6, 24, 80} {
		verts, tris := parityStarPrism(n, height, rng)
		subset := make([]int, len(tris))
		for i := range subset {
			subset[i] = i
		}
		filtered := meshbool.NewParityMesh(verts, tris)
		indexed := meshbool.NewParityMesh(verts, tris)
		plain := meshbool.NewParityMesh(verts, tris)
		plain.Unfiltered = true

		flagged := 0
		for _, ray := range meshbool.AxisRays {
			for ti := range tris {
				if filtered.BuildFacetBox(ray.Axis, ray.U, ray.V, ti).AxisSegment {
					flagged++
				}
			}
		}
		require.Positive(t, flagged, "n=%d: the prism must have axis-parallel segment facets", n)

		points := parityPrismPoints(verts, height, 3000, rng)
		var bad []int
		for i, x := range points {
			wantIn, wantOn, wantErr := meshbool.MeshParityPreparedContext(t.Context(), x, plain, subset)
			gotIn, gotOn, gotErr := meshbool.MeshParityPreparedContext(t.Context(), x, filtered, subset)
			allIn, allOn, allErr := meshbool.MeshParityAllContext(t.Context(), x, indexed)
			if gotIn != wantIn || gotOn != wantOn || errorText(gotErr) != errorText(wantErr) ||
				allIn != wantIn || allOn != wantOn || errorText(allErr) != errorText(wantErr) {
				bad = append(bad, i)
			}
		}
		require.Empty(t, bad[:min(len(bad), 5)], "n=%d: %d of %d points differ; first indices", n, len(bad), len(points))
	}
}
