package loftcylinder_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestWedgeGate(t *testing.T) {
	tris := [][3]int{{0, 1, 2}}
	verts := func(y1, z1 float64) []r3.Vec {
		return []r3.Vec{
			{X: 1, Y: 0, Z: 0}, {X: 1, Y: y1, Z: z1}, {X: 0, Y: 1, Z: 0},
			{X: 1, Y: 0, Z: 1}, {X: 1, Y: -0.01, Z: 1}, {X: 0, Y: 1, Z: 1},
		}
	}
	t.Run("small excursion", func(t *testing.T) {
		departure, ok := loftcylinder.WedgeGate(verts(-0.01, 0), tris, 2, 0, 1)
		require.True(t, ok)
		// The departure is the float 0.01 rounded up from an exact rational.
		require.GreaterOrEqual(t, departure, 0.01)
		require.InDelta(t, 0.01, departure, 1e-15)
	})
	t.Run("large excursion", func(t *testing.T) {
		_, ok := loftcylinder.WedgeGate(verts(-0.5, 0), tris, 2, 0, 1)
		require.False(t, ok)
	})
	t.Run("vertex off both caps", func(t *testing.T) {
		_, ok := loftcylinder.WedgeGate(verts(-0.01, 0.5), tris, 2, 0, 1)
		require.False(t, ok)
	})
}
