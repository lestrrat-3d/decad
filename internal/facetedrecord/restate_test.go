package facetedrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// restateFixture is a two-facet held mesh over a two-face body: facet 0 on
// face 0, facet 1 on face 1, with exact dyadic per-vertex bounds.
func restateFixture() (*topo.Body, []*topo.Face, facetedrecord.Payload) {
	faces := []*topo.Face{{}, {}}
	body := &topo.Body{Lumps: []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: faces}}}}}
	fp := facetedrecord.Payload{
		Verts:       []r3.Vec{{}, {X: 1}, {X: 1, Y: 1}, {Y: 1}},
		Tris:        [][3]int{{0, 1, 2}, {0, 2, 3}},
		FaceOf:      []int{0, 1},
		VertexBound: []float64{0.125, 0.5, 0.25, 0.0625},
		MeshBound:   0.5,
		AreaSlack:   1,
		VolSymDiff:  2,
	}
	return body, faces, fp
}

func TestRestateHeldMesh(t *testing.T) {
	t.Parallel()

	t.Run("RestatesFacetsAndBounds", func(t *testing.T) {
		t.Parallel()
		body, faces, fp := restateFixture()
		m, err := facetedrecord.Restate(t.Context(), body, fp, 0.5)
		require.NoError(t, err)
		require.Equal(t, fp.Verts, m.Vertices)
		require.Equal(t, fp.Tris, m.Triangles)
		require.Len(t, m.Source, 2)
		require.True(t, m.Source[0] == faces[0] && m.Source[1] == faces[1], "each facet keeps its live face")
		require.Equal(t, fp.VertexBound, m.VertexBound)
		require.InDelta(t, 0.5, m.FaceBound[faces[0]], 0)
		require.InDelta(t, 0.25, m.FaceBound[faces[1]], 0)
		require.InDelta(t, 0.5, m.Bound, 0)
		require.InDelta(t, 1, m.AreaSlack, 0)
		require.InDelta(t, 2, m.VolSymDiff, 0)
		require.True(t, m.SymDiffOK)
	})

	t.Run("FinerToleranceRefuses", func(t *testing.T) {
		t.Parallel()
		body, _, fp := restateFixture()
		_, err := facetedrecord.Restate(t.Context(), body, fp, 0.25)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "requested tolerance 0.25 mm is below the faceted body's minimum mesh bound 0.5 mm")
	})

	t.Run("MeshBoundNotTheFacetMaximumRefuses", func(t *testing.T) {
		t.Parallel()
		body, _, fp := restateFixture()
		fp.MeshBound = 0.75
		_, err := facetedrecord.Restate(t.Context(), body, fp, 1)
		require.ErrorIs(t, err, decaderr.ErrBooleanFailed)
	})
}
