package decad

import (
	"fmt"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// liftOriginSketch draws on an XY-parallel sketch plane at o.
func liftOriginSketch(t *testing.T, o r3.Vec, draw func(s *sketch.Sketch)) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	f, err := r3.NewFrame(o, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	draw(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// liftOriginBodies builds one body of each carrier family on an XY-parallel
// sketch plane at o, every plane-local u shifted by shift: a box (prism plane
// carriers), a disc (a cylinder carrier) and a full-turn revolve (revolve
// carriers). A brep record builds its carriers through the same prism
// carrier constructors, and the boolean that makes one off the world origin
// carries a section displacement, which refuses it a model, so it has no
// row of its own here.
func liftOriginBodies(t *testing.T, o r3.Vec, shift float64) map[string]*Body {
	t.Helper()
	extrude := func(doc *Document, h float64, draw func(s *sketch.Sketch)) *Body {
		s, p := liftOriginSketch(t, o, draw)
		b, err := doc.Extrude(s, p, Distance{D: units.Millimeters(h), Dir: Along})
		require.NoError(t, err)
		return b
	}
	rect := func(u0, v0, u1, v1 float64) func(s *sketch.Sketch) {
		return func(s *sketch.Sketch) {
			r := s.CreateRectangle(u0+shift, v0, u1+shift, v1)
			s.Fix(r.A)
		}
	}
	out := map[string]*Body{}
	out["box"] = extrude(New(), 1, rect(0, 0, 2, 1))
	out["disc"] = extrude(New(), 1, func(s *sketch.Sketch) {
		c := s.CreatePoint(3+shift, 3)
		s.Fix(c)
		s.CreateCircle(c, 2)
	})
	s, p := liftOriginSketch(t, o, rect(0, 1, 3, 2))
	rev, err := New().Revolve(s, p, SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}, FullRevolution{})
	require.NoError(t, err)
	out["revolve"] = rev
	return out
}

// TestBodyGeomLiftDeltaReadsTheFrameOrigin pins the point term of the
// clearance kernel's per-body carrier displacement (bodyGeom.carrierDelta) on
// an XY-parallel sketch plane away from the world origin. At an INTEGER origin
// with integer coordinates every carrier lift is exact, so the box, disc and
// revolve models all keep a carrier displacement of exactly zero. At a far
// origin with coordinates shifted by 0.1 every carrier anchor's origin.X + u
// sum rounds by up to half an ulp at 10⁶, so each model's displacement must be
// positive and at least the rounding its own carriers recorded.
//
// Shown to fail: replacing the exact per-point measurement with a magnitude
// charge (proofbound.RigidRoundAllow at the coordinate and origin envelope) in
// clearance.PrismCarrierFrame.point and BuildRevolveCarriers turns every
// integer-origin subtest red; dropping the measurement (recording no
// LiftRound) turns every far-origin subtest red.
func TestBodyGeomLiftDeltaReadsTheFrameOrigin(t *testing.T) {
	t.Parallel()
	t.Run("integer origin", func(t *testing.T) {
		t.Parallel()
		for name, b := range liftOriginBodies(t, r3.NewVec(10, 20, 30), 0) {
			t.Run(name, func(t *testing.T) {
				g, ok := newBodyGeom(b)
				require.True(t, ok)
				require.Zero(t, carrierLiftRound(g.faces), "an exact lift records no rounding")
				require.Zero(t, g.carrierDelta, "an exact lift must keep the model's carrier displacement zero")
			})
		}
	})
	t.Run("far origin", func(t *testing.T) {
		t.Parallel()
		for name, b := range liftOriginBodies(t, r3.NewVec(1e6+0.1, 0, 0), 0.1) {
			t.Run(name, func(t *testing.T) {
				g, ok := newBodyGeom(b)
				require.True(t, ok)
				lift := carrierLiftRound(g.faces)
				require.Positive(t, lift, fmt.Sprintf("the %s carriers' far lift rounds", name))
				require.GreaterOrEqual(t, g.carrierDelta, lift)
				require.GreaterOrEqual(t, g.delta, g.carrierDelta)
			})
		}
	})
}
