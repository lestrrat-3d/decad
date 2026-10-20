package meshbool_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestSetSlabBounds(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	prep := func(t *testing.T, verts []r3.Vec, beta float64) *meshbool.BoolMesh {
		t.Helper()
		bm, err := meshbool.PrepContext(t.Context(), verts, [][3]int{{0, 1, 2}}, []int{0})
		require.NoError(t, err)
		bm.SetVertexBounds([]float64{beta, beta, beta})
		return bm
	}
	always := func(int) (r3.Frame, bool) { return frame, true }

	t.Run("a facet in the plane states a thin slab", func(t *testing.T) {
		t.Parallel()
		bm := prep(t, []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(10, 0, 0), r3.NewVec(0, 10, 0)}, 0.003)
		bm.SetSlabBounds(always)
		require.Zero(t, bm.SlabBound[0])
		require.Equal(t, 0.003, bm.FacetBound[0])
	})

	t.Run("a tilted facet charges its corner distance and its slope", func(t *testing.T) {
		t.Parallel()
		const beta, rise = 0.003, 1e-6
		bm := prep(t, []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(10, 0, rise), r3.NewVec(0, 10, 0)}, beta)
		bm.SetSlabBounds(always)
		// The facet normal is (-rise*10, 0, 100): slope sin = rise*10/100.
		want := rise + beta*(rise*10/math.Hypot(rise*10, 100))
		require.GreaterOrEqual(t, bm.SlabBound[0], want)
		require.LessOrEqual(t, bm.SlabBound[0], want*(1+1e-9))
	})

	t.Run("a steep facet keeps the displacement bound", func(t *testing.T) {
		t.Parallel()
		bm := prep(t, []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(10, 0, 0), r3.NewVec(0, 0, 10)}, 0.003)
		bm.SetSlabBounds(always)
		require.Equal(t, 0.003, bm.SlabBound[0])
	})

	t.Run("a facet with no plane keeps the displacement bound", func(t *testing.T) {
		t.Parallel()
		bm := prep(t, []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(10, 0, 0), r3.NewVec(0, 10, 0)}, 0.003)
		bm.SetSlabBounds(func(int) (r3.Frame, bool) { return r3.Frame{}, false })
		require.Equal(t, 0.003, bm.SlabBound[0])
	})
}
