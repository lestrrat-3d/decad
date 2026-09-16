package decadtest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// manifoldUAxis is the sketch plane's own u axis, the revolve axis for the
// torus fixture below.
var manifoldUAxis = decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}

// TestIsManifoldAcceptsBlock proves IsManifold accepts the easy case: a plain
// extruded block, six planar faces each with one loop of four edges.
func TestIsManifoldAcceptsBlock(t *testing.T) {
	t.Parallel()

	doc := decad.New()
	body := decadtest.NewBlock(t, doc, 0, 0, 10, 6, units.Millimeters(2))

	kinds := map[decad.SurfaceKind]int{}
	for _, f := range body.Faces() {
		kinds[f.Surface().Kind()]++
	}
	require.Equal(t, map[decad.SurfaceKind]int{decad.KindPlane: 6}, kinds)

	decadtest.IsManifold(t, body)
}

// TestIsManifoldAcceptsHolePlate proves IsManifold accepts a plate with a
// circular through hole. The hole's wall is one seamless cylindrical face
// carrying TWO loops (the top and bottom circle rims), not the "exactly one
// outer loop" an earlier, wrong design draft would have required. This is
// one of the two awkward cases the wrong rule would have rejected.
func TestIsManifoldAcceptsHolePlate(t *testing.T) {
	t.Parallel()

	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)

	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)

	var wall *decad.Face
	for _, f := range body.Faces() {
		if f.Surface().Kind() == decad.KindCylinder {
			wall = f
		}
	}
	require.NotNil(t, wall)
	require.Len(t, wall.Loops(), 2)

	decadtest.IsManifold(t, body)
}

// TestIsManifoldAcceptsTorus proves IsManifold accepts a full-revolve
// torus: one toroidal face with NO loops at all, since a closed surface of
// revolution needs no boundary. This is the second of the two awkward
// cases the wrong "exactly one outer loop" rule would have rejected.
func TestIsManifoldAcceptsTorus(t *testing.T) {
	t.Parallel()

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 10)
	s.Fix(center)
	s.CreateCircle(center, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Revolve(s, s.Profiles()[0], manifoldUAxis, decad.FullRevolution{})
	require.NoError(t, err)

	require.Len(t, body.Faces(), 1)
	torus, ok := body.Faces()[0].Surface().(decad.Torus)
	require.True(t, ok)
	require.Empty(t, body.Faces()[0].Loops())
	require.True(t, torus.Major.Equal(units.Millimeters(10), 1e-9))

	decadtest.IsManifold(t, body)
}

// TestIsManifoldRejectsNilBody shows IsManifold fail when body is nil.
func TestIsManifoldRejectsNilBody(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.IsManifold(tb, nil)
	})
	require.Contains(t, out, "body must not be nil")
}
