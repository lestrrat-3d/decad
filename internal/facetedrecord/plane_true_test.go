package facetedrecord

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfaceplace"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestPlacedPlane proves a placed plane tag exact only when the placement carries the plane onto its float image
// exactly (agent-docs/coplanar-contact-design.md §17.1).
func TestPlacedPlane(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 6), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	tag := surfacegeom.Plane{Frame: frame}
	exact := Group{Surface: tag, Retained: true}
	place := func(t *testing.T, g Group, delta r3.Transform) *surfacegeom.Plane {
		t.Helper()
		moved, err := surfaceplace.TransformSurface(g.Surface, delta)
		require.NoError(t, err)
		return placedPlane(g, moved, delta)
	}
	about := func(t *testing.T, axis r3.Vec, deg float64) r3.Transform {
		t.Helper()
		turn, err := r3.Rotation(axis, units.Degrees(deg))
		require.NoError(t, err)
		return turn
	}
	t.Run("a turn about the plane's normal keeps it exact", func(t *testing.T) {
		t.Parallel()
		got := place(t, exact, about(t, r3.NewVec(0, 0, 1), 70))
		require.NotNil(t, got)
		// A second placement reads the first one's claim.
		again := exact
		again.Surface, again.NormalBound, again.PlacedPlane = *got, 1e-15, got
		require.NotNil(t, place(t, again, about(t, r3.NewVec(0, 0, 1), 25)))
	})
	// Rejected variants: each would claim a plane the true piece need not lie in.
	t.Run("a turn that tilts the plane claims nothing", func(t *testing.T) {
		t.Parallel()
		require.Nil(t, place(t, exact, about(t, r3.NewVec(1, 0, 0), 31)))
	})
	t.Run("a move below the floats' resolution claims nothing", func(t *testing.T) {
		t.Parallel()
		lift, err := r3.Translation(r3.NewVec(0, 0, 1e-30))
		require.NoError(t, err)
		moved, err := surfaceplace.TransformSurface(tag, lift)
		require.NoError(t, err)
		require.Equal(t, tag, moved, "the move leaves the tag's floats unchanged")
		require.Nil(t, placedPlane(exact, moved, lift))
	})
	t.Run("a tag that states a departure claims nothing", func(t *testing.T) {
		t.Parallel()
		g := exact
		g.NormalBound = 1e-15
		require.Nil(t, place(t, g, about(t, r3.NewVec(0, 0, 1), 70)))
	})
	t.Run("a stale claim claims nothing", func(t *testing.T) {
		t.Parallel()
		// The claim names another tag: something moved the tag without proving the image again.
		other, err := r3.NewFrame(r3.NewVec(0, 0, 7), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		g := exact
		g.NormalBound, g.PlacedPlane = 1e-15, &surfacegeom.Plane{Frame: other}
		require.Nil(t, place(t, g, about(t, r3.NewVec(0, 0, 1), 70)))
	})
	t.Run("a displaced tag claims nothing", func(t *testing.T) {
		t.Parallel()
		g := exact
		g.AxialDelta, g.HasAxialDelta = 1e-6, true
		require.Nil(t, place(t, g, about(t, r3.NewVec(0, 0, 1), 70)))
	})
	t.Run("a tag the generic Boolean did not retain claims nothing", func(t *testing.T) {
		t.Parallel()
		g := exact
		g.Retained = false
		require.Nil(t, place(t, g, about(t, r3.NewVec(0, 0, 1), 70)))
	})
}
