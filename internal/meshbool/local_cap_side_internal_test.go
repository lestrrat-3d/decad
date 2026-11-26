package meshbool

// CI: isolated-tests

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRuledTagSeamsAndCrossings(t *testing.T) {
	plane := func(u, v r3.Vec) *topo.Face {
		f, err := r3.NewFrame(r3.Vec{}, u, v)
		require.NoError(t, err)
		return &topo.Face{RetainedTag: true, Surface: surfacegeom.Plane{Frame: f}}
	}
	cylinder := func(origin, axis r3.Vec, r float64) *topo.Face {
		return &topo.Face{RetainedTag: true, Surface: surfacegeom.Cylinder{Origin: origin, Axis: axis,
			Radius: units.Millimeters(r)}}
	}
	read := func(f *topo.Face) ruledTag {
		tag, ok := exactRuledTag(f)
		require.True(t, ok)
		return tag
	}
	up := [3]*big.Rat{big.NewRat(0, 1), big.NewRat(0, 1), big.NewRat(1, 1)}

	wall := read(plane(r3.Vec{X: 1}, r3.Vec{Z: 1}))         // the plane y = 0, which holds the z direction
	side := read(plane(r3.Vec{Y: 1}, r3.Vec{Z: 1}))         // the plane x = 0
	floor := read(plane(r3.Vec{X: 1}, r3.Vec{Y: 1}))        // the plane z = 0
	upright := read(cylinder(r3.Vec{}, r3.Vec{Z: 1}, 1))    // an upright cylinder
	beside := read(cylinder(r3.Vec{X: 3}, r3.Vec{Z: 1}, 2)) // a parallel one
	lying := read(cylinder(r3.Vec{}, r3.Vec{X: 1}, 1))      // one along x

	require.True(t, wall.crosses(up))
	require.True(t, upright.crosses(up))
	require.False(t, floor.crosses(up), "a plane parallel to the cap plane never crosses it")
	require.False(t, lying.crosses(up), "a cylinder along the cap plane has rulings parallel to it")

	d, ok := wall.seam(side)
	require.True(t, ok)
	require.Zero(t, d[0].Sign())
	require.Zero(t, d[1].Sign())
	require.NotZero(t, d[2].Sign(), "two upright planes meet along z")
	_, ok = upright.seam(wall)
	require.True(t, ok, "an upright plane holds the upright cylinder's axis direction")
	_, ok = upright.seam(beside)
	require.True(t, ok, "parallel cylinders meet in lines along their axes")
	_, ok = upright.seam(lying)
	require.False(t, ok, "crossing axes meet in curves")
	d, ok = lying.seam(wall)
	require.True(t, ok)
	require.Zero(t, ratDot(d, up).Sign(), "the plane y = 0 meets the cylinder along x, which never crosses z = 0")
	_, ok = upright.seam(read(cylinder(r3.Vec{}, r3.Vec{Z: 1}, 1)))
	require.False(t, ok, "one surface twice has no seam lines")
	_, ok = upright.seam(read(plane(r3.Vec{X: 1}, r3.Vec{Y: 1, Z: 1})))
	require.False(t, ok, "a tilted plane meets the upright cylinder in an ellipse")
}

func TestExactRuledTagRefusesBoundedTags(t *testing.T) {
	f, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	for name, face := range map[string]*topo.Face{
		"not retained": {Surface: surfacegeom.Plane{Frame: f}},
		"departed":     {RetainedTag: true, TagDeparture: 1e-15, Surface: surfacegeom.Plane{Frame: f}},
		"displaced":    {RetainedTag: true, AxialDelta: 1e-3, Surface: surfacegeom.Plane{Frame: f}},
		"extent":       {RetainedTag: true, ExtentDelta: 1e-3, Surface: surfacegeom.Plane{Frame: f}},
		"cone":         {RetainedTag: true, Surface: surfacegeom.Cone{Axis: r3.Vec{Z: 1}, Radius: units.Millimeters(1)}},
		"zero radius":  {RetainedTag: true, Surface: surfacegeom.Cylinder{Axis: r3.Vec{Z: 1}}},
		"charged view": {RetainedTag: true, ViewCharge: true, Surface: surfacegeom.Plane{Frame: f}},
		"flat patch":   {RetainedTag: true, FlatPatch: true, Surface: surfacegeom.Plane{Frame: f}},
		"nil face":     nil,
	} {
		_, ok := exactRuledTag(face)
		require.False(t, ok, name)
	}
}
