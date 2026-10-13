package loftcylinder_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestRingEdgesInside drives the edge loop with no teeth: an inner ring, a
// ring that may reach the bore, and a ring outside the blank that has no
// strict blank-inside witness.
func TestRingEdgesInside(t *testing.T) {
	t.Parallel()
	square := func(half float64) [][2]proofarith.Xpt {
		corners := []r3.Vec{
			{X: half, Y: -half, Z: 1}, {X: half, Y: half, Z: 1},
			{X: -half, Y: half, Z: 1}, {X: -half, Y: -half, Z: 1},
		}
		edges := make([][2]proofarith.Xpt, len(corners))
		for i, p := range corners {
			edges[i] = [2]proofarith.Xpt{proofarith.XptOf(p), proofarith.XptOf(corners[(i+1)%len(corners)])}
		}
		return edges
	}
	t.Run("inner ring", func(t *testing.T) {
		t.Parallel()
		got, err := loftcylinder.RingEdgesInside(t.Context(), square(1), nil, 2, 0.5)
		require.NoError(t, err)
		require.Equal(t, 4, got.Inner)
		require.Zero(t, got.Outer)
		require.Zero(t, got.Seam)
	})
	t.Run("reaches the bore", func(t *testing.T) {
		t.Parallel()
		_, err := loftcylinder.RingEdgesInside(t.Context(), square(1), nil, 2, 1.5)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "ring may reach the centered bore")
	})
	t.Run("outside the blank", func(t *testing.T) {
		t.Parallel()
		got, err := loftcylinder.RingEdgesInside(t.Context(), square(3), nil, 2, 0.5)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "ring has no strict blank-inside witness")
		require.Equal(t, 4, got.Outer)
	})
}
