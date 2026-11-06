package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// requireBoredLoft cuts a centered through-bore of radius r from body, which
// runs from z = 0 to height, and checks that the result is still a loft: the
// bore removes exactly a cylinder of volume πr²·height, the Cut's own caps
// remain selectable, and the tessellation is verified.
func requireBoredLoft(t *testing.T, body *decad.Body, height, r float64, msg string) *decad.Body {
	t.Helper()
	before, err := body.Volume()
	require.NoError(t, err, msg)
	tool := circleBodyAtZ(t, body.Document(), 0, r, -1, height+2)
	bored, err := decad.Cut(t.Context(), body, tool)
	require.NoError(t, err, msg)
	after, err := bored.Volume()
	require.NoError(t, err, msg)
	require.InDelta(t, math.Pi*r*r*height, before.Value.Base()-after.Value.Base(),
		before.Bound.Base()+after.Bound.Base(), msg)
	for _, role := range []decad.FeatureRef{decad.CapStart(bored), decad.CapEnd(bored)} {
		_, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(bored)
		require.NoError(t, err, msg)
	}
	walls, err := decad.Faces(decad.Cylindrical()).SelectFaces(bored)
	require.NoError(t, err, msg)
	require.Len(t, walls, 1, msg)
	cylinder, ok := walls[0].Surface().(decad.Cylinder)
	require.True(t, ok, msg)
	require.InDelta(t, r, cylinder.Radius.Base(), 1e-12, msg)
	requireVerifiedSolid(t, bored, msg)
	return bored
}

// TestLoftBoreTwistedSections cuts a centered through-bore from lofts whose two
// sections differ: a rounded gear outline lofted to the same outline turned
// about its axis, and a square lofted to a turned, smaller square. Each stays a
// loft whose sections share the new hole.
func TestLoftBoreTwistedSections(t *testing.T) {
	const height = 3.0
	twist := 14.5 * math.Pi / 180
	g := newTwistedGear(5, 20*math.Pi/180, 0.05)
	body, bottom, top := g.loft(t, height, twist, 0)
	rounded, err := decad.Fillet(t.Context(), body, rungQuery(bottom, top, height).Exactly(len(bottom)),
		units.Millimeters(0.25))
	require.NoError(t, err)
	bored := requireBoredLoft(t, rounded, height, 0.5, "gear")
	require.Equal(t, len(bottom), filletRoleCount(bored), "the rounded roots stay")

	w := sketch.NewWorld()
	sk0, p0, _ := squareSection(t, w, w.XY(), 10, 0, 0)
	sk1, p1, _ := squareSection(t, w, splitTopPlane(t, w, height), 7, -30*math.Pi/180, 0)
	square, err := decad.Loft(t.Context(), decad.New(), sk0, p0, sk1, p1)
	require.NoError(t, err)
	requireBoredLoft(t, square, height, 1, "square")
}
