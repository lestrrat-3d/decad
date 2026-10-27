package prismcells

// An internal test: the opposed-pocket scenes the public API draws reach
// CurvedCrossingsCut only after other checks that also refuse a crossing of
// two circles, and no public scene leaves a sketch-reported crossing without
// its arranged vertex, so only this reading shows those two legs deciding.

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// curvedScene arranges operand A's shapes and operand B's shapes in one
// sketch and tags every entity with its operand.
func curvedScene(t *testing.T, a, b func(*sketch.Sketch) []sketch.Entity) ([]*sketch.Profile, map[sketch.Entity]Origin) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	tags := map[sketch.Entity]Origin{}
	for _, e := range a(s) {
		tags[e] = Origin{Hole: -1}
	}
	for _, e := range b(s) {
		tags[e] = Origin{IsB: true, Hole: -1}
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s.Profiles(), tags
}

func sceneRect(x0, y0, x1, y1 float64) func(*sketch.Sketch) []sketch.Entity {
	return func(s *sketch.Sketch) []sketch.Entity {
		rect := s.CreateRectangle(x0, y0, x1, y1)
		s.Fix(rect.A)
		s.Fix(rect.C)
		return []sketch.Entity{rect.AB, rect.BC, rect.CD, rect.DA}
	}
}

// sceneCircle is a circle of radius r about (x, 0).
func sceneCircle(x, r float64) func(*sketch.Sketch) []sketch.Entity {
	return func(s *sketch.Sketch) []sketch.Entity {
		centre := s.CreatePoint(x, 0)
		s.Fix(centre)
		return []sketch.Entity{s.CreateCircle(centre, r)}
	}
}

// TestCurvedCrossingsCut reads each leg on a scene sketch arranges: a
// rectangle crossing a circle, and two nested circles, pass; a rectangle
// whose edges touch a circle, and two crossing circles, refuse. The same
// crossing with no arranged cells has no vertex to stand for sketch's
// reported points, and refuses.
//
// Shown red: dropping the two-curve leg passes crossing-circles, and
// dropping the vertex match passes no-vertex.
func TestCurvedCrossingsCut(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		a, b    func(*sketch.Sketch) []sketch.Entity
		noCells bool
		want    bool
	}{
		{name: "line-crosses-circle", a: sceneRect(-6, -3, 0, 3), b: sceneCircle(1, 3), want: true},
		{name: "nested-circles", a: sceneCircle(-2, 3), b: sceneCircle(-2, 1), want: true},
		// The circle crosses x = 0 and touches y = ±2 at (-1, ±2).
		{name: "tangent-line", a: sceneRect(-6, -2, 0, 2), b: sceneCircle(-1, 2)},
		{name: "crossing-circles", a: sceneCircle(-2, 2), b: sceneCircle(0, 2)},
		{name: "no-vertex", a: sceneRect(-6, -3, 0, 3), b: sceneCircle(1, 3), noCells: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			profiles, tags := curvedScene(t, tc.a, tc.b)
			require.NotEmpty(t, profiles)
			if tc.noCells {
				profiles = nil
			}
			got, err := CurvedCrossingsCut(proofbound.NewWorkBudget(t.Context()), tags, profiles)
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}
