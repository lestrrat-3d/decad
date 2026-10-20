package facetedrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSourceTag(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	plane := surfacegeom.Plane{Frame: frame}
	cyl := surfacegeom.Cylinder{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(4)}

	t.Run("a built plane is copied and flipped for a tool", func(t *testing.T) {
		t.Parallel()
		g, ok := facetedrecord.SourceTag(&topo.Face{Surface: plane}, false)
		require.True(t, ok)
		require.True(t, g.Retained)
		require.False(t, g.Reversed)
		g, ok = facetedrecord.SourceTag(&topo.Face{Surface: plane}, true)
		require.True(t, ok)
		require.True(t, g.Reversed)
		g, ok = facetedrecord.SourceTag(&topo.Face{Surface: cyl, Reversed: true}, true)
		require.True(t, ok)
		require.False(t, g.Reversed, "a hole wall that a tool flips once more is outward again")
	})
	t.Run("a stated tilt or a denoted patch declines", func(t *testing.T) {
		t.Parallel()
		_, ok := facetedrecord.SourceTag(&topo.Face{Surface: plane, NormalBound: 1e-9}, false)
		require.False(t, ok, "a built face with a tilt is a ruled stand-in")
		_, ok = facetedrecord.SourceTag(&topo.Face{Surface: plane, Denoted: &surfacenormal.Revolved{}}, false)
		require.False(t, ok)
		_, ok = facetedrecord.SourceTag(&topo.Face{Surface: surfacegeom.Faceted{}}, false)
		require.False(t, ok)
	})
	t.Run("a retained face passes its tag departure on, not its held term", func(t *testing.T) {
		t.Parallel()
		g, ok := facetedrecord.SourceTag(&topo.Face{Surface: cyl, RetainedTag: true,
			TagDeparture: 0.25, NormalBound: 0.75}, false)
		require.True(t, ok)
		require.Equal(t, 0.25, g.NormalBound)
	})
}

func TestRetainedNormalBound(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)

	bound, ok := facetedrecord.RetainedNormalBound(surfacegeom.Plane{Frame: frame}, 0, 3)
	require.True(t, ok)
	require.Zero(t, bound, "a plane adds no held term however far the facets sit")

	cyl := surfacegeom.Cylinder{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(10)}
	bound, ok = facetedrecord.RetainedNormalBound(cyl, 0.5, 0.1)
	require.True(t, ok)
	require.GreaterOrEqual(t, bound, 0.5+2*0.1/10)
	require.Less(t, bound, 0.5+2*0.1/10+1e-12)

	_, ok = facetedrecord.RetainedNormalBound(cyl, 0, 5)
	require.False(t, ok, "2·δ/R at 1 states no usable direction")
	_, ok = facetedrecord.RetainedNormalBound(cyl, 0, 4.99)
	require.True(t, ok)

	sphere := surfacegeom.Sphere{Radius: units.Millimeters(2)}
	_, ok = facetedrecord.RetainedNormalBound(sphere, 0, 1)
	require.False(t, ok)
	_, ok = facetedrecord.RetainedNormalBound(surfacegeom.Cone{}, 0, 0)
	require.False(t, ok, "cones wait for increment 2")
}
