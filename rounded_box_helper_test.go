package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// roundedBoxAt is boxAt turned 20 degrees about the body diagonal through its
// centre and back by two separate Transforms, so the float corners of its
// faces round (agent-docs/coplanar-contact-design.md §11.3). Its faces read a
// positive bound, and a coplanar contact with them keeps the gap-tolerance
// retry. A turn about a cardinal axis keeps the corners exact and does not.
func roundedBoxAt(t *testing.T, w *sketch.World, doc *Document, x, y, z, sx, sy, sz float64) *Body {
	t.Helper()
	box := boxAt(t, w, doc, 0, 0, 0, sx, sy, sz)
	centre := r3.Vec{X: sx / 2, Y: sy / 2, Z: sz / 2}
	for _, deg := range []float64{20, -20} {
		turn, err := r3.RotationAround(centre, r3.Vec{X: 1, Y: 1, Z: 1}, units.Degrees(deg))
		require.NoError(t, err)
		box, err = Transform(t.Context(), box, turn)
		require.NoError(t, err)
	}
	move, err := r3.Translation(r3.Vec{X: x, Y: y, Z: z})
	require.NoError(t, err)
	placed, err := Transform(t.Context(), box, move)
	require.NoError(t, err)
	return placed
}

// requireRoundedContactFace asserts the tessellation of b reads a positive
// face bound on its face nearest the plane z: the fixture's contact face.
func requireRoundedContactFace(t *testing.T, b *Body, z float64) {
	t.Helper()
	m, _, err := pairMeshes{}.TessellateOperand(t.Context(), &b.n, 0.01)
	require.NoError(t, err)
	best, bestDist := -1, math.Inf(1)
	for i, tri := range m.Triangles {
		d := 0.0
		for _, v := range tri {
			d = max(d, math.Abs(m.Vertices[v].Z-z))
		}
		if d < bestDist {
			best, bestDist = i, d
		}
	}
	require.GreaterOrEqual(t, best, 0)
	require.Less(t, bestDist, 1e-6, "a facet lies on the contact plane")
	require.Positive(t, m.FaceBound[m.Source[best]], "the rounded contact face reads a positive bound")
}

// TestRoundedBoxContactFaceReadsABound pins the fixture itself: the rounded
// box's contact face reads a positive face bound in its tessellation, and the
// same box unrounded reads none.
func TestRoundedBoxContactFaceReadsABound(t *testing.T) {
	w := sketch.NewWorld()
	doc := New()
	requireRoundedContactFace(t, roundedBoxAt(t, w, doc, 0.5, 0, 10, 10, 10, 10), 10)
	plain := boxAt(t, w, doc, 0.5, 0, 10, 10, 10, 10)
	m, _, err := pairMeshes{}.TessellateOperand(t.Context(), &plain.n, 0.01)
	require.NoError(t, err)
	for f, bound := range m.FaceBound {
		require.Zero(t, bound, "face %p of the exact box", f)
	}
}

// TestCoplanarUnionDropsTheOccupancyRecord pins the occupancy guard of
// agent-docs/coplanar-contact-design.md §6 end to end: the Union of two tracked
// boxes that meet face to face states no occupancy record, because the
// proximity re-run could defer an admitted pair no mesh refusal backs, while a
// crossing pair keeps its record. The flush result is still the exact volume.
func TestCoplanarUnionDropsTheOccupancyRecord(t *testing.T) {
	w := sketch.NewWorld()
	doc := New()
	flush, err := Union(t.Context(), boxAt(t, w, doc, 0, 0, 0, 10, 10, 10), boxAt(t, w, doc, 0.5, 0, 10, 10, 10, 10))
	require.NoError(t, err)
	payload, ok := flush.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.True(t, payload.OccupancyRecord() == nil, "an admitted coplanar pair publishes no record")
	v, err := flush.Volume()
	require.NoError(t, err)
	require.Equal(t, 2000.0, v.Value.Base())
	require.Zero(t, v.Bound.Base())

	crossing, err := Union(t.Context(), boxAt(t, w, doc, 0, 0, 0, 10, 10, 10), boxAt(t, w, doc, 5, 3, 4, 10, 10, 10))
	require.NoError(t, err)
	payload, ok = crossing.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.NotNil(t, payload.OccupancyRecord(), "a crossing pair keeps its record")
}
