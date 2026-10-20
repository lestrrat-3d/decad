package facetedrecord_test

import (
	"math"
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
	t.Run("a stated tilt on a built face declines", func(t *testing.T) {
		t.Parallel()
		_, ok := facetedrecord.SourceTag(&topo.Face{Surface: plane, NormalBound: 1e-9}, false)
		require.False(t, ok, "a built face with a tilt is a ruled stand-in")
		_, ok = facetedrecord.SourceTag(&topo.Face{Surface: surfacegeom.Faceted{}}, false)
		require.False(t, ok)
	})
	t.Run("a denoted patch is carried with the tilt the operand states", func(t *testing.T) {
		t.Parallel()
		denoted := &surfacenormal.Revolved{}
		g, ok := facetedrecord.SourceTag(&topo.Face{Surface: cyl, Denoted: denoted, NormalBound: 0.5}, false)
		require.True(t, ok)
		require.Same(t, denoted, g.Denoted)
		require.Equal(t, 0.5, g.NormalBound)
	})
	t.Run("a retained face passes its tag departure on, not its held term", func(t *testing.T) {
		t.Parallel()
		g, ok := facetedrecord.SourceTag(&topo.Face{Surface: cyl, RetainedTag: true,
			TagDeparture: 0.25, NormalBound: 0.75}, false)
		require.True(t, ok)
		require.Equal(t, 0.25, g.NormalBound)
	})
}

func TestHeldNormalTerm(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)

	term, ok := facetedrecord.HeldNormalTerm(surfacegeom.Plane{Frame: frame}, 3, 0)
	require.True(t, ok)
	require.Zero(t, term, "a plane adds no held term however far the facets sit")

	cyl := surfacegeom.Cylinder{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(10)}
	term, ok = facetedrecord.HeldNormalTerm(cyl, 0.1, 0)
	require.True(t, ok)
	require.GreaterOrEqual(t, term, 2*0.1/10)
	require.Less(t, term, 2*0.1/10+1e-12)

	_, ok = facetedrecord.HeldNormalTerm(cyl, 5, 0)
	require.False(t, ok, "2·δ/R at 1 states no usable direction")
	_, ok = facetedrecord.HeldNormalTerm(cyl, 4.99, 0)
	require.True(t, ok)

	sphere := surfacegeom.Sphere{Radius: units.Millimeters(2)}
	_, ok = facetedrecord.HeldNormalTerm(sphere, 1, 0)
	require.False(t, ok)

	cone := surfacegeom.Cone{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(5), HalfAngle: units.Degrees(30)}
	term, ok = facetedrecord.HeldNormalTerm(cone, 0.1, 5)
	require.True(t, ok)
	require.GreaterOrEqual(t, term, math.Cos(math.Pi/6)*2*0.1/5)
	_, ok = facetedrecord.HeldNormalTerm(cone, 0.1, 0)
	require.False(t, ok, "a cone patch that may reach its axis has no normal bound")
	_, ok = facetedrecord.HeldNormalTerm(cone, 0.1, -1)
	require.False(t, ok)

	torus := surfacegeom.Torus{Axis: r3.NewVec(0, 0, 1), Major: units.Millimeters(10), Minor: units.Millimeters(2)}
	term, ok = facetedrecord.HeldNormalTerm(torus, 0.01, 0)
	require.True(t, ok)
	require.GreaterOrEqual(t, term, 2*0.01*(1+2*10.0/8)/2)
	horn := surfacegeom.Torus{Axis: r3.NewVec(0, 0, 1), Major: units.Millimeters(2), Minor: units.Millimeters(2)}
	_, ok = facetedrecord.HeldNormalTerm(horn, 0.01, 0)
	require.False(t, ok, "a horn torus reaches its axis")
}

func TestConeRhoLower(t *testing.T) {
	t.Parallel()
	cone := surfacegeom.Cone{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(5), HalfAngle: units.Degrees(45)}
	smin := facetedrecord.ConeAxialMin(cone, r3.NewVec(5, 0, 2), r3.NewVec(5, 1, 3), r3.NewVec(5, 0, 4))
	require.Equal(t, 2.0, smin)
	rho := facetedrecord.ConeRhoLower(cone, smin, 0.1)
	// Radius 5 at s=0 growing at slope 1: the patch starts at s=2−0.1.
	require.Less(t, rho, 5+1.9)
	require.Greater(t, rho, 5+1.9-1e-9)
	require.LessOrEqual(t, facetedrecord.ConeRhoLower(cone, -6, 0.1), 0.0,
		"a patch that reaches back to the apex has no bound")
}
