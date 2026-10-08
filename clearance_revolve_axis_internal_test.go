package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// revolveAxisGap builds b's clearance carriers and returns their axis gap
// (clearance.RevolveCarrierResult.AxisGap).
func revolveAxisGap(t *testing.T, b *Body) float64 {
	t.Helper()
	rp, ok := b.payload.(revolvePayload)
	require.True(t, ok)
	in, ok, err := revolveCarrierInput(proofbound.NewWorkBudget(t.Context()), rp)
	require.NoError(t, err)
	require.True(t, ok)
	return clearance.BuildRevolveCarriers(in).AxisGap
}

// TestRevolveCarrierAxisGapExactRevolvesStayZero is the exact half of the
// carriers' axis gap: an integer profile about a coordinate axis re-expresses
// exactly, so every carrier — cylinder, plane, cone, sphere, torus, and a
// partial sweep's caps — matches its record and the gap is exactly zero,
// which leaves bodyGeom.delta, and every Clearance row such a body publishes,
// exactly as it reads without the charge. The torus below the axis resolves
// the axis direction to (−1, 0), whose held rotation β = −fl(π) is not π; a
// whole circle pairs with its record at any phase, so the comparison takes the
// exactly unit direction itself instead.
//
// Shown to fail: comparing a closed circle through the held β's enclosure
// (dropping revolveGapMeter.unitDirection) turns torus-below red.
func TestRevolveCarrierAxisGapExactRevolvesStayZero(t *testing.T) {
	t.Parallel()
	uAxis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}
	draw := func(t *testing.T, f func(s *sketch.Sketch)) (*sketch.Sketch, *sketch.Profile) {
		t.Helper()
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		f(s)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		require.Len(t, s.Profiles(), 1)
		return s, s.Profiles()[0]
	}
	rect := func(s *sketch.Sketch) {
		r := s.CreateRectangle(0, 5, 10, 15)
		s.Fix(r.A)
	}
	ball := func(s *sketch.Sketch) {
		a, b, c := s.CreatePoint(-3, 0), s.CreatePoint(3, 0), s.CreatePoint(0, 0)
		s.Fix(a)
		s.Fix(b)
		s.Fix(c)
		s.CreateLine(a, b)
		s.CreateArc(c, b, a)
	}
	torus := func(v float64) func(s *sketch.Sketch) {
		return func(s *sketch.Sketch) {
			c := s.CreatePoint(0, v)
			s.Fix(c)
			s.CreateCircle(c, 2)
		}
	}
	cone := func(s *sketch.Sketch) {
		a, b, c := s.CreatePoint(0, 0), s.CreatePoint(4, 0), s.CreatePoint(0, 3)
		s.Fix(a)
		s.Fix(b)
		s.Fix(c)
		s.CreateLine(a, b)
		s.CreateLine(b, c)
		s.CreateLine(c, a)
	}
	quarter := AngleExtent{A: units.Degrees(90), Dir: Along}
	cases := []struct {
		name   string
		draw   func(s *sketch.Sketch)
		extent AngularExtent
	}{
		{"rect-full", rect, FullRevolution{}},
		{"rect-partial", rect, quarter},
		{"ball-full", ball, FullRevolution{}},
		{"ball-partial", ball, quarter},
		{"torus", torus(5), FullRevolution{}},
		{"torus-below", torus(-5), FullRevolution{}},
		{"cone-full", cone, FullRevolution{}},
		{"cone-partial", cone, quarter},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, p := draw(t, tc.draw)
			b, err := New().Revolve(s, p, uAxis, tc.extent)
			require.NoError(t, err)
			require.Zero(t, revolveAxisGap(t, b))
		})
	}
}
