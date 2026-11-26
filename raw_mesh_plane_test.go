package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRawMeshPlaneOnBooleanFragments(t *testing.T) {
	t.Parallel()
	ctx, doc := t.Context(), New()
	mesh := func(at r3.Vec) *MeshBody {
		t.Helper()
		v := []r3.Vec{at, at.Add(r3.Vec{X: 4}), at.Add(r3.Vec{Y: 3}), at.Add(r3.Vec{Z: 5})}
		m, err := NewMeshBody(ctx, v, [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}})
		require.NoError(t, err)
		return m
	}
	target := mesh(r3.Vec{})
	solid, err := SolidFromMesh(doc, target).Do(ctx)
	require.NoError(t, err)
	var original *topo.MeshPlaneSlab
	for _, face := range solid.Faces() {
		if _, ok := face.n.Surface.(surfacegeom.Faceted); ok {
			original = face.n.MeshPlaneSlab()
			require.NotNil(t, original)
			require.Less(t, original.Distance, 1e-12)
		} else {
			require.Nil(t, face.n.MeshPlaneSlab())
		}
	}
	require.NotNil(t, original)
	tool, err := SolidFromMesh(doc, mesh(r3.Vec{X: 1, Y: .5, Z: .25})).Do(ctx)
	require.NoError(t, err)
	result, err := Cut(solid, tool).Do(ctx)
	require.NoError(t, err)
	var kept bool
	for _, face := range result.Faces() {
		kept = kept || face.n.MeshPlaneSlab() == original
	}
	require.True(t, kept, "the cut retains the untouched part of the original oblique source plane")
	translation, err := r3.Translation(r3.Vec{X: 2})
	require.NoError(t, err)
	moved, err := Transform(result, translation).Do(ctx)
	require.NoError(t, err)
	for _, face := range moved.Faces() {
		require.Nil(t, face.n.MeshPlaneSlab())
	}
	bounded, err := solidFromMeshBodyBoundedAt(ctx, doc, target, doc.producerAt(t.Context()), .001)
	require.NoError(t, err)
	for _, face := range bounded.Faces() {
		require.Nil(t, face.n.MeshPlaneSlab())
	}
	derived, err := solidFromMeshBody(ctx, doc, target)
	require.NoError(t, err)
	for _, face := range derived.Faces() {
		require.Nil(t, face.n.MeshPlaneSlab())
	}

	source := new(topo.Face)
	source.SetMeshPlaneSlab(original)
	clone := *source
	require.Nil(t, clone.MeshPlaneSlab())
	other := new(topo.Face)
	other.SetMeshPlaneSlab(new(topo.MeshPlaneSlab))
	unaffected := facetedrecord.Group{MeshSlab: original}
	unaffected.RestrictMeshPlaneSlab(source, false)
	require.Same(t, original, unaffected.MeshSlab)
	for _, tc := range []struct {
		face    *topo.Face
		changed bool
	}{
		{source, true}, {new(topo.Face), false}, {other, false},
	} {
		g := unaffected
		g.RestrictMeshPlaneSlab(tc.face, tc.changed)
		g.RestrictMeshPlaneSlab(source, false)
		require.Nil(t, g.MeshSlab, "one affected or unrelated fragment invalidates the group")
	}
}
