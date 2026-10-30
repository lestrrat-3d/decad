package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// sb7Plate extrudes the 80x60x10 plate at the origin and unions the boss
// outline drawn by draw on the z=10 plane, h tall.
func sb7Plate(t *testing.T, h float64, draw func(s *sketch.Sketch, fix func(x, y float64) *sketch.Point)) *decad.Body {
	t.Helper()
	doc := decad.New()
	w := sketch.NewWorld()
	ps, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := ps.CreateRectangle(0, 0, 80, 60)
	ps.Fix(rect.A)
	ps.Fix(rect.C)
	_, err = ps.Solve(t.Context())
	require.NoError(t, err)
	plate, err := decad.Extrude(t.Context(), doc, ps, ps.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	plane, err := w.CreateOffsetPlane(w.XY(), 10)
	require.NoError(t, err)
	bs, err := w.CreateSketch(plane)
	require.NoError(t, err)
	draw(bs, func(x, y float64) *sketch.Point {
		p := bs.CreatePoint(x, y)
		bs.Fix(p)
		return p
	})
	_, err = bs.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, bs.Profiles(), 1)
	boss, err := decad.Extrude(t.Context(), doc, bs, bs.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	u, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	return u
}

// sb7RoundedBoss is a 30x30 boss with radius-5 corners at (25..55, 15..45),
// 10 mm tall on the plate.
func sb7RoundedBoss(t *testing.T) *decad.Body {
	return sb7Plate(t, 10, func(s *sketch.Sketch, fix func(x, y float64) *sketch.Point) {
		a0, a1 := fix(30, 15), fix(50, 15)
		b0, b1 := fix(55, 20), fix(55, 40)
		c0, c1 := fix(50, 45), fix(30, 45)
		d0, d1 := fix(25, 40), fix(25, 20)
		s.CreateLine(a0, a1)
		s.CreateArc(fix(50, 20), a1, b0)
		s.CreateLine(b0, b1)
		s.CreateArc(fix(50, 40), b1, c0)
		s.CreateLine(c0, c1)
		s.CreateArc(fix(30, 40), c1, d0)
		s.CreateLine(d0, d1)
		s.CreateArc(fix(30, 20), d1, a0)
	})
}

func sb7FrontTopEdge() *decad.EdgeQuery {
	return decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(30, 15, 20)), decad.EndpointAt(r3.NewVec(50, 15, 20))).Exactly(1)
}

func TestSB7TangentJunctionNamesChain(t *testing.T) {
	t.Parallel()
	u := sb7RoundedBoss(t)
	_, err := decad.Fillet(t.Context(), u, sb7FrontTopEdge(), units.Millimeters(1.5))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	requireReasonLeads(t, err, `the selected edge ends at the tangent junction 30,15,20 `)
	require.Contains(t, err.Error(), `whose continuation is not selected; WithTangentChain selects the whole chain (brep-modify SB7)`)

	// The named workaround builds: plate + boss minus the rim fillet strip,
	// 4 straight sides of 20 mm and 4 quarter arcs of radius 5 whose
	// centroid line sits at 5 - c from the arc centre.
	const r = 1.5
	area := (1 - math.Pi/4) * r * r
	cen := r * (10 - 3*math.Pi) / (3 * (4 - math.Pi))
	v0 := 48000 + (900-(4-math.Pi)*25)*10
	want := v0 - area*(80+2*math.Pi*(5-cen))
	got, err := decad.Fillet(t.Context(), u, sb7FrontTopEdge(), units.Millimeters(r), decad.WithTangentChain())
	require.NoError(t, err)
	vol, err := got.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, vol.Value.Base(), vol.Bound.Base()+1e-6)
}

// TestSB7NonTangentJunctionKeepsCurvedFaceText selects the top edge of a boss
// whose left wall is an arc joined to the edge at an angle: the junction is
// not tangent, so the curved-face reason stays.
func TestSB7NonTangentJunctionKeepsCurvedFaceText(t *testing.T) {
	t.Parallel()
	u := sb7Plate(t, 10, func(s *sketch.Sketch, fix func(x, y float64) *sketch.Point) {
		p0, p1, p2, p3 := fix(30, 10), fix(40, 10), fix(40, 20), fix(30, 20)
		s.CreateLine(p0, p1)
		s.CreateLine(p1, p2)
		s.CreateLine(p2, p3)
		s.CreateArc(fix(27, 15), p3, p0)
	})
	sel := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(30, 20, 20)), decad.EndpointAt(r3.NewVec(40, 20, 20))).Exactly(1)
	_, err := decad.Fillet(t.Context(), u, sel, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), `is a curved face, not a plane across the axis (brep-modify SB7)`)
	require.NotContains(t, err.Error(), `WithTangentChain`)
}
