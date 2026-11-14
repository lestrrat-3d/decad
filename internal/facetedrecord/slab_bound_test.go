package facetedrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestSlabFacetBound(t *testing.T) {
	t.Parallel()
	// Three triangles stacked along z: one crossing z = 0, one wholly above the
	// slab, one wholly inside it but not crossing.
	verts := []r3.Vec{
		r3.NewVec(0, 0, -1), r3.NewVec(1, 0, 1), r3.NewVec(0, 1, 1), // crosses
		r3.NewVec(0, 0, 5), r3.NewVec(1, 0, 5), r3.NewVec(0, 1, 5), // far above
		r3.NewVec(0, 0, 0.05), r3.NewVec(1, 0, 0.05), r3.NewVec(0, 1, 0.05), // inside the slab
	}
	tris := [][3]int{{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}
	origin, normal := r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1)

	t.Run("far facet is not charged", func(t *testing.T) {
		t.Parallel()
		beta := []float64{0.01, 0.01, 0.01, 0.9, 0.9, 0.9, 0, 0, 0}
		got, ok := facetedrecord.SlabFacetBound(verts, tris, beta, origin, normal, 0.1)
		require.True(t, ok)
		require.Equal(t, 0.01, got)
	})
	t.Run("larger facet inside the slab is charged", func(t *testing.T) {
		t.Parallel()
		beta := []float64{0.01, 0.01, 0.01, 0.9, 0.9, 0.9, 0, 0.5, 0}
		got, ok := facetedrecord.SlabFacetBound(verts, tris, beta, origin, normal, 0.1)
		require.True(t, ok)
		require.Equal(t, 0.5, got, "one corner of a facet in the slab states the facet's bound")
	})
	t.Run("narrow slab leaves that facet out", func(t *testing.T) {
		t.Parallel()
		beta := []float64{0.01, 0.01, 0.01, 0.9, 0.9, 0.9, 0, 0.5, 0}
		// The crossing facet lies on both sides, so it stays in any slab.
		got, ok := facetedrecord.SlabFacetBound(verts, tris, beta, origin, normal, 0.01)
		require.True(t, ok)
		require.Equal(t, 0.01, got)
	})
	t.Run("no record", func(t *testing.T) {
		t.Parallel()
		_, ok := facetedrecord.SlabFacetBound(verts, tris, nil, origin, normal, 0.1)
		require.False(t, ok)
	})
}
