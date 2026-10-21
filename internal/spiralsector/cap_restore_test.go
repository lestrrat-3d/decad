package spiralsector_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRestoreCaps(t *testing.T) {
	t.Parallel()

	// A closed tetrahedron whose apex was moved 0.03 in Y by the proxy.
	moved := []r3.Vec{{}, {X: 1}, {Y: 1}, {X: 0.25, Y: 0.28, Z: 1}}
	source := []r3.Vec{{}, {X: 1}, {Y: 1}, {X: 0.25, Y: 0.25, Z: 1}}
	tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}}
	bound := []float64{0.01, 0.01, 0.01, 0.04}
	sourceBound := []float64{0.01, 0.01, 0.01, 0.01}

	t.Run("restores the moved vertex and charges the move", func(t *testing.T) {
		t.Parallel()
		home := spiralsector.NewCapHome(moved, source, sourceBound)
		require.Equal(t, 1, home.Len())
		got, ok, err := spiralsector.RestoreCaps(t.Context(), home, moved, tris, bound)
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, source, got.Verts)
		require.Equal(t, 0.28, moved[3].Y, "the held vertices are not modified")
		require.Positive(t, got.VolumeCharge)
		require.Positive(t, got.AreaCharge)
		// The apex moved 0.03 over a base of area 0.5, so the volume changed by
		// a third of that product. The charge must cover the change.
		require.GreaterOrEqual(t, got.VolumeCharge, 0.03*0.5/3)
	})

	t.Run("leaves a mesh without moved vertices alone", func(t *testing.T) {
		t.Parallel()
		home := spiralsector.NewCapHome(moved, source, sourceBound)
		_, ok, err := spiralsector.RestoreCaps(t.Context(), home, source, tris, sourceBound)
		require.NoError(t, err)
		require.False(t, ok)
	})

	t.Run("refuses a vertex bound below the source bound", func(t *testing.T) {
		t.Parallel()
		home := spiralsector.NewCapHome(moved, source, []float64{0.01, 0.01, 0.01, 0.05})
		_, ok, err := spiralsector.RestoreCaps(t.Context(), home, moved, tris, bound)
		require.NoError(t, err)
		require.False(t, ok)
	})

	t.Run("refuses two moved vertices at one position", func(t *testing.T) {
		t.Parallel()
		home := spiralsector.NewCapHome(moved, source, sourceBound)
		home.Append(spiralsector.NewCapHome(moved, source, sourceBound))
		_, ok, err := spiralsector.RestoreCaps(t.Context(), home, moved, tris, bound)
		require.NoError(t, err)
		require.False(t, ok)
	})

	t.Run("pairs only slices of one length", func(t *testing.T) {
		t.Parallel()
		require.Zero(t, spiralsector.NewCapHome(moved, source[:3], sourceBound).Len())
	})
}
