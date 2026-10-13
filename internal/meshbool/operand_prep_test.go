package meshbool_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPrepContext(t *testing.T) {
	verts := []r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 2, Y: 0, Z: 0}, {X: 0, Y: 3, Z: 0}, {X: 0, Y: 0, Z: 4}}
	tris := [][3]int{{0, 2, 1}, {0, 1, 3}}

	bm, err := meshbool.PrepContext(t.Context(), verts, tris, []int{7, 9})
	require.NoError(t, err)
	require.Equal(t, []int{7, 9}, bm.Src)
	require.Len(t, bm.Xverts, 4)
	require.Len(t, bm.FnormsReady, 2)
	require.Equal(t, [2]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 2, Y: 3, Z: 0}}, bm.Boxes[0])
	require.Equal(t, 0, bm.Owner[[2]int{0, 2}])
	require.Equal(t, 1, bm.Owner[[2]int{1, 3}])
	require.Len(t, bm.Owner, 6)
	// Facet 0 winds (0,2,1), so its exact normal points along -z.
	require.Equal(t, -1, bm.Norms[0].Z.Sign())
	require.Equal(t, 0, bm.Norms[0].X.Sign())

	t.Run("non-finite vertex", func(t *testing.T) {
		bad := []r3.Vec{{X: math.NaN()}, {X: 1}, {Y: 1}}
		_, err := meshbool.PrepContext(t.Context(), bad, [][3]int{{0, 1, 2}}, []int{0})
		require.ErrorIs(t, err, decaderr.ErrBooleanFailed)
		require.ErrorContains(t, err, "a mesh vertex is not finite")
	})
	t.Run("collapsed facet", func(t *testing.T) {
		line := []r3.Vec{{}, {X: 1}, {X: 2}}
		_, err := meshbool.PrepContext(t.Context(), line, [][3]int{{0, 1, 2}}, []int{0})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "collapsed facet")
	})
	t.Run("cancelled", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err := meshbool.PrepContext(ctx, verts, tris, []int{0, 0})
		require.ErrorIs(t, err, context.Canceled)
	})
}

func TestSetVertexBounds(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
	bm, err := meshbool.PrepContext(t.Context(), verts, [][3]int{{0, 2, 1}, {0, 1, 3}}, []int{0, 0})
	require.NoError(t, err)
	bm.SetVertexBounds([]float64{0.5, 0.25, 0.125, 2})
	require.Equal(t, []float64{0.5, 0.25, 0.125, 2}, bm.VertexBound)
	require.Equal(t, []float64{0.5, 2}, bm.FacetBound)
	require.NotNil(t, bm.Parity)
}

func TestSourceIDs(t *testing.T) {
	a, b := new(int), new(int)
	ids, err := meshbool.SourceIDs(t.Context(), []*int{a, b, a}, map[*int]int{a: 3, b: 5})
	require.NoError(t, err)
	require.Equal(t, []int{3, 5, 3}, ids)

	_, err = meshbool.SourceIDs(t.Context(), []*int{a, b}, map[*int]int{a: 3})
	require.ErrorIs(t, err, decaderr.ErrBooleanFailed)
	require.ErrorContains(t, err, "a facet's source is not an operand face")

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = meshbool.SourceIDs(ctx, []*int{a}, map[*int]int{a: 3})
	require.ErrorIs(t, err, context.Canceled)
}

func TestOperandSymDiff(t *testing.T) {
	got, err := meshbool.OperandSymDiff(true, true, 17)
	require.NoError(t, err)
	require.Equal(t, 17.0, got)

	_, err = meshbool.OperandSymDiff(true, false, 17)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "carries no proof of the volume")

	// The audit refusal wins when both proofs are missing.
	_, err = meshbool.OperandSymDiff(false, false, 17)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "ran no facet-contact audit")
}
