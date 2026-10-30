package facetedrecord_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfaceplace"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func placedTagFixtures(t *testing.T) (surfacegeom.Plane, surfacegeom.Cylinder, []r3.Vec) {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 20), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	cyl := surfacegeom.Cylinder{Origin: r3.NewVec(20, 0, 10), Axis: r3.NewVec(0, 1, 0), Radius: units.Millimeters(4)}
	verts := []r3.Vec{r3.NewVec(16, 0, 10), r3.NewVec(24, 20, 10), r3.NewVec(40, 20, 20)}
	return surfacegeom.Plane{Frame: frame}, cyl, verts
}

func requireDeparture(t *testing.T, tag surfacegeom.Surface, delta r3.Transform, verts []r3.Vec) (float64, bool) {
	t.Helper()
	moved, err := surfaceplace.TransformSurface(tag, delta)
	require.NoError(t, err)
	return facetedrecord.PlacedTagDeparture(tag, moved, delta, verts, 1e-9)
}

func TestPlacedTagDeparture(t *testing.T) {
	t.Parallel()
	plane, cyl, verts := placedTagFixtures(t)
	shift, err := r3.Translation(r3.NewVec(7, -3, 100))
	require.NoError(t, err)
	turn, err := r3.RotationAround(r3.NewVec(20, 10, 0), r3.NewVec(0, 0, 1), units.Degrees(20))
	require.NoError(t, err)

	t.Run("an exact translation leaves a plane's bound at zero", func(t *testing.T) {
		t.Parallel()
		add, ok := requireDeparture(t, plane, shift, verts)
		require.True(t, ok)
		require.Zero(t, add)
	})

	t.Run("an exact translation leaves a curved tag's bound at zero", func(t *testing.T) {
		t.Parallel()
		sphere := surfacegeom.Sphere{Center: r3.NewVec(20, 0, 10), Radius: units.Millimeters(4)}
		torus := surfacegeom.Torus{Center: r3.NewVec(20, 0, 10), Axis: r3.NewVec(0, 0, 1),
			Major: units.Millimeters(6), Minor: units.Millimeters(1)}
		for _, tag := range []surfacegeom.Surface{cyl, sphere, torus} {
			add, ok := requireDeparture(t, tag, shift, verts)
			require.True(t, ok)
			require.Zero(t, add, "%T moved exactly states no departure", tag)
		}
	})

	t.Run("an inexact translation charges the rounding of the origin", func(t *testing.T) {
		t.Parallel()
		tag := surfacegeom.Cylinder{Origin: r3.NewVec(0.1, 0, 0), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(4)}
		s := r3.NewVec(0.7, 0, 0)
		exact := new(big.Rat).Add(new(big.Rat).SetFloat64(tag.Origin.X), new(big.Rat).SetFloat64(s.X))
		require.NotZero(t, exact.Cmp(new(big.Rat).SetFloat64(tag.Origin.X+s.X)),
			"the fixture's sum is not a float, so the moved origin rounds")
		inexact, err := r3.Translation(s)
		require.NoError(t, err)
		add, ok := requireDeparture(t, tag, inexact, verts)
		require.True(t, ok)
		require.Positive(t, add)
		require.Less(t, add, 1e-12, "the charge is the origin's rounding, far below the trivial 2")
	})

	t.Run("a rotation charges at least the orthonormality defect", func(t *testing.T) {
		t.Parallel()
		linear, err := massmoment.PlacementRotation(turn)
		require.NoError(t, err)
		charge, err := massmoment.MapChargeOf(linear)
		require.NoError(t, err)
		require.Positive(t, charge.Stretch, "a 20 degree rotation is not exactly orthonormal in floats")
		add, ok := requireDeparture(t, cyl, turn, verts)
		require.True(t, ok)
		require.GreaterOrEqual(t, add, 4*charge.Stretch)
		require.Less(t, add, 1e-9, "the charge is rounding-sized, far below the trivial 2")
		planeAdd, ok := requireDeparture(t, plane, turn, verts)
		require.True(t, ok)
		require.Positive(t, planeAdd, "a rotated plane is a float image of the exact one")
	})

	t.Run("a cone keeps no stated bound", func(t *testing.T) {
		t.Parallel()
		cone := surfacegeom.Cone{Origin: r3.NewVec(0, 0, 0), Axis: r3.NewVec(0, 0, 1),
			Radius: units.Millimeters(1), HalfAngle: units.Degrees(10)}
		_, ok := requireDeparture(t, cone, shift, verts)
		require.False(t, ok)
	})

	t.Run("a displacement past the radius keeps no stated bound", func(t *testing.T) {
		t.Parallel()
		far := []r3.Vec{r3.NewVec(1e16, 0, 0)}
		_, ok := requireDeparture(t, cyl, turn, far)
		require.False(t, ok)
	})
}
