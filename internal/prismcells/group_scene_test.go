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
// dropping the vertex match passes no-vertex. Dropping the tangent check
// leaves tangent-line refused, since its uncut tangency has no vertex for
// the vertex match either; apitest's slot-tangent-edge shows that check
// deciding.
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

// sceneBosses is several circles of radius 3 about (x, 0), one per x.
func sceneBosses(xs ...float64) func(*sketch.Sketch) []sketch.Entity {
	return func(s *sketch.Sketch) []sketch.Entity {
		var out []sketch.Entity
		for _, x := range xs {
			out = append(out, sceneCircle(x, 3)(s)...)
		}
		return out
	}
}

// TestRingContact reads the holed cell of a 20 mm square floor round
// radius-3 bosses: bosses apart from each other and from the outline pass. A boss
// tangent inside the outline refuses: the arrangement leaves the tangency
// uncut and returns a valid cell with whole rings. Two tangent bosses refuse:
// the arrangement cuts one circle at the tangency and returns one hole ring
// through that point twice.
//
// Shown red: dropping RegionContact from RingContact passes tangent-outline,
// and dropping the repeated-vertex reading passes tangent-bosses.
func TestRingContact(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name  string
		b     func(*sketch.Sketch) []sketch.Entity
		holes int
		want  bool
	}{
		{name: "one-apart", b: sceneBosses(0), holes: 1},
		{name: "two-apart", b: sceneBosses(-4, 4), holes: 2},
		// The circle about (7, 0) touches x = 10 at (10, 0).
		{name: "tangent-outline", b: sceneBosses(7), holes: 1, want: true},
		// The two circles touch at the origin.
		{name: "tangent-bosses", b: sceneBosses(-3, 3), want: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			profiles, _ := curvedScene(t, sceneRect(-10, -10, 10, 10), tc.b)
			var floor *sketch.Profile
			for _, p := range profiles {
				if len(p.Holes) == 0 {
					continue
				}
				require.Nil(t, floor, "one cell carries the holes")
				floor = p
			}
			require.NotNil(t, floor)
			if tc.holes != 0 {
				require.True(t, floor.Valid)
				require.Len(t, floor.Holes, tc.holes)
			}
			got, err := RingContact(proofbound.NewWorkBudget(t.Context()), floor)
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}
