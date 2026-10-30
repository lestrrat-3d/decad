package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

const crossingCylindersReason = `no retained blend class reads selected edge[0]: two retained cylinders whose axes are not parallel: ` +
	`the blend is a canal surface (retained-edge-blend SR1)`

// turnedShaftWithCrossPin is a radius-10 shaft, 80 mm long on the x axis,
// joined to a radius-3 pin along z at x = 40: a mesh Boolean result whose pin
// roots lie between two retained cylinders with crossing axes, each with its
// material inside.
func turnedShaftWithCrossPin(t *testing.T) *decad.Body {
	t.Helper()
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 80, 10)
	s.Fix(rect.A)
	s.Fix(rect.C)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	shaft, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], axis, decad.FullRevolution{})
	require.NoError(t, err)

	plane, err := w.CreateOffsetPlane(w.XY(), -15)
	require.NoError(t, err)
	hs, err := w.CreateSketch(plane)
	require.NoError(t, err)
	centre := hs.CreatePoint(40, 0)
	hs.Fix(centre)
	hs.CreateCircle(centre, 3)
	_, err = hs.Solve(t.Context())
	require.NoError(t, err)
	pin, err := decad.Extrude(t.Context(), doc, hs, hs.Profiles()[0], decad.Distance{D: units.Millimeters(45), Dir: decad.Along})
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), shaft, pin)
	require.NoError(t, err)
	return joined
}

// crossHoleMouths selects the edges of body whose end points both lie on the
// pin's wall (radius 3 about its axis) and on the shaft wall (radius 10).
func crossHoleMouths(t *testing.T, body *decad.Body) *decad.EdgeQuery {
	t.Helper()
	near := func(a, b float64) bool { d := a - b; return d < 0.05 && d > -0.05 }
	onBoth := func(v r3.Vec) bool {
		return near(hypot(v.X-40, v.Y), 3) && near(hypot(v.Y, v.Z), 10)
	}
	var q *decad.EdgeQuery
	n := 0
	for _, e := range body.Edges() {
		s, f := e.Start().Position().Value, e.End().Position().Value
		if !onBoth(s) || !onBoth(f) {
			continue
		}
		preds := []decad.EdgePredicate{decad.EndpointAt(s), decad.EndpointAt(f)}
		if q == nil {
			q = decad.Edges(preds...)
		} else {
			q = q.Or(preds...)
		}
		n++
	}
	require.Positive(t, n, `the mouth edges are on the Boolean result`)
	return q.Exactly(n)
}

func hypot(a, b float64) float64 {
	return r3.NewVec(a, b, 0).Len()
}

func TestBlendRefusalNamesCrossingCylinders(t *testing.T) {
	t.Parallel()
	// Fillet builds a cross hole's mouths and a pin's or boss's roots
	// (retained_saddle_fillet_test.go); Chamfer builds only the mouths
	// (retained_saddle_chamfer_test.go). A root's chamfer is a canal surface no
	// route builds.
	t.Run(`chamfer of cross-pin roots`, func(t *testing.T) {
		t.Parallel()
		body := turnedShaftWithCrossPin(t)
		_, err := decad.Chamfer(t.Context(), body, crossHoleMouths(t, body), units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		requireReasonLeads(t, err, `this evaluator chamfers a straight prism, its one-line Sweep reduction, or a revolve only`)
		requireReasonLeads(t, err, crossingCylindersReason)
		require.Contains(t, err.Error(), `revolve only; no retained blend class reads`,
			`the existing prefix is followed by the named condition`)
	})

	t.Run(`chamfer of a boss root on a rod`, func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		rod := diskBody(t, doc, 0, 0, 15)
		boss, err := decad.Boss(t.Context(), rod, decad.BossSpec{Face: decad.Faces(decad.Cylindrical()),
			At: r3.NewVec(15, 0, 10), Diameter: units.Millimeters(10), Height: units.Millimeters(8)})
		require.NoError(t, err)
		_, err = decad.Chamfer(t.Context(), boss, decad.Edges(decad.Concave()).Exactly(1), units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		requireReasonLeads(t, err, `this evaluator chamfers a straight prism, its one-line Sweep reduction, or a revolve only`)
		requireReasonLeads(t, err, crossingCylindersReason)
	})
}
