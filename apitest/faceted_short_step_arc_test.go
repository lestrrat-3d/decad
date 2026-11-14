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

// TestFacetedShortStepArc cuts a window with 1 mm rounded corners through a
// revolved drum's back plate at a slant. The mesh Boolean puts rim vertices
// where the target's triangle edges cross each corner round, so the held
// chain of a round's rim has steps far below the angular floor of 4·bound/r.
// Rule E7 skips those steps instead of refusing the arc
// (agent-docs/faceted-source-retention-design.md §10.1), so every rim edge of
// the window loop reads as an Arc3 or a Line3 and Chamfer finds a retained
// blend class for the loop.
func TestFacetedShortStepArc(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	doc := decad.New()
	const h = 84.0

	meridian := shortStepSketch(t, r3.Vec{Z: h}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	pts := [][2]float64{
		{4.9, -11.5}, {48.5, -11.5}, {49, -12.0}, {49, -21.4}, {57.5, -21.4},
		{58, -21.9}, {58, -27.9}, {57.5, -28.4}, {46.5, -28.4}, {46, -27.9}, {46, -15.5},
		{4.9, -15.5}, {4.4, -15.0}, {4.4, -12.0},
	}
	first := meridian.CreatePoint(pts[0][0], pts[0][1])
	prev := first
	for _, p := range pts[1:] {
		next := meridian.CreatePoint(p[0], p[1])
		meridian.CreateLine(prev, next)
		prev = next
	}
	meridian.CreateLine(prev, first)
	drum, err := decad.Revolve(ctx, doc, meridian, shortStepProfile(t, meridian),
		decad.ConstructionAxis{Origin: r3.Vec{Z: h}, Dir: r3.Vec{Y: 1}}, decad.FullRevolution{})
	require.NoError(t, err)

	angle := 60 * math.Pi / 180
	u := r3.Vec{X: math.Cos(angle), Z: math.Sin(angle)}
	ws := shortStepSketch(t, r3.Vec{Z: h, Y: -16.5}, u, r3.Vec{Y: 1}.Cross(u))
	shortStepWindow(t, ws, 14, 24, 40*math.Pi/180, 1.0)
	window, err := decad.Extrude(ctx, doc, ws, shortStepProfile(t, ws),
		decad.Distance{D: units.Millimeters(6), Dir: decad.Along})
	require.NoError(t, err)
	body, err := decad.Cut(ctx, drum, window)
	require.NoError(t, err)

	// The window loops on the plate's two faces, y = -11.5 and y = -15.5:
	// 4 lines and 4 rounds, each round one edge.
	var sel *decad.EdgeQuery
	loops, arcs := 0, 0
	for _, f := range body.Faces() {
		for _, loop := range f.Loops() {
			es := loop.Edges()
			if !shortStepWindowLoop(es) {
				continue
			}
			loops++
			for _, e := range es {
				s, en := e.Start().Position().Value, e.End().Position().Value
				var preds []decad.EdgePredicate
				switch e.Curve().(type) {
				case decad.Arc3:
					arcs++
					preds = []decad.EdgePredicate{decad.EndpointAt(s), decad.Circular()}
				case decad.Line3:
					preds = []decad.EdgePredicate{decad.EndpointAt(s), decad.ParallelTo(en.Sub(s))}
				default:
					require.Failf(t, "rim edge is not exact", "the window loop holds a %T", e.Curve())
				}
				if sel == nil {
					sel = decad.Edges(preds...)
				} else {
					sel = sel.Or(preds...)
				}
			}
		}
	}
	require.Equal(t, 2, loops, "the window has a loop on each plate face")
	require.Equal(t, 8, arcs, "each loop holds four rounds")

	out, err := decad.Chamfer(ctx, body, sel.Exactly(16), units.Millimeters(0.5))
	require.NoError(t, err)
	require.True(t, out.IsSolid())
}

// shortStepWindowLoop reports whether es is an 8-edge loop lying on the
// plate's faces y = -11.5 or y = -15.5.
func shortStepWindowLoop(es []*decad.Edge) bool {
	if len(es) != 8 {
		return false
	}
	for _, e := range es {
		y := e.Start().Position().Value.Y
		if math.Abs(y+11.5) > 1e-6 && math.Abs(y+15.5) > 1e-6 {
			return false
		}
	}
	return true
}

func shortStepSketch(t *testing.T, origin, u, v r3.Vec) *sketch.Sketch {
	t.Helper()
	f, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	w := sketch.NewWorld()
	p, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(p)
	require.NoError(t, err)
	return s
}

func shortStepProfile(t *testing.T, s *sketch.Sketch) *sketch.Profile {
	t.Helper()
	_, err := s.Solve(t.Context())
	require.NoError(t, err)
	var sel *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == 0 {
			require.Nil(t, sel)
			sel = p
		}
	}
	require.NotNil(t, sel)
	return sel
}

// shortStepWindow draws an annular sector of inner radius rIn, outer radius
// rOut and half-angle half about the sketch origin, its four corners rounded.
func shortStepWindow(t *testing.T, s *sketch.Sketch, rIn, rOut, half, round float64) {
	t.Helper()
	cos, sin := math.Cos(half), math.Sin(half)
	p1, p2 := s.CreatePoint(rIn*cos, -rIn*sin), s.CreatePoint(rIn*cos, rIn*sin)
	p3, p4 := s.CreatePoint(rOut*cos, -rOut*sin), s.CreatePoint(rOut*cos, rOut*sin)
	inner, outer := s.CreateLine(p1, p2), s.CreateLine(p3, p4)
	right, left := s.CreateLine(p1, p3), s.CreateLine(p2, p4)
	x1 := s.CreatePoint(10, 0)
	ref := s.CreateLine(s.Origin(), x1)
	ref.SetConstruction(true)
	deg := half * 180 / math.Pi
	s.AddConstraint(
		sketch.NewHorizontal(ref), sketch.NewDistance(s.Origin(), x1, 10),
		sketch.NewVertical(inner), sketch.NewDistancePointLine(s.Origin(), inner, rIn*cos),
		sketch.NewVertical(outer), sketch.NewDistancePointLine(s.Origin(), outer, rOut*cos),
		sketch.NewPointOnLine(s.Origin(), right), sketch.NewPointOnLine(s.Origin(), left),
		sketch.NewAngle(ref, right, -deg), sketch.NewAngle(ref, left, deg),
	)
	for _, c := range []struct {
		a, b *sketch.Line
		at   *sketch.Point
	}{{inner, right, p1}, {left, inner, p2}, {right, outer, p3}, {outer, left, p4}} {
		_, err := sketch.CreateFillet(s, c.a, c.b, c.at, round)
		require.NoError(t, err)
	}
}

// TestFacetedShortStepArcAtCapRound unions a box ear onto a block whose end
// faces are rounded, after a cavity cut. The ear's back root runs from the
// floor plane onto the cap's r 2 round. Its held chain has steps below the
// angular floor, and the edge is still the Arc3 of the plane and the round.
func TestFacetedShortStepArcAtCapRound(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
	const floor, top, left, front, back = 100.0, 190.0, -20.0, -41.7, 39.2
	block := shortStepPrism(t, doc, r3.Vec{X: left}, r3.Vec{Y: 1}, r3.Vec{Z: 1},
		[][2]float64{{front, floor}, {back, floor}, {back, top}, {front, top}}, []float64{3, 3, 3, 3}, 51)
	caps := decad.Edges(decad.OuterLoopOf(decad.CapStart(block))).Or(decad.OuterLoopOf(decad.CapEnd(block)))
	block, err := decad.Fillet(ctx, block, caps, units.Millimeters(2))
	require.NoError(t, err)
	cavity := shortStepPrism(t, doc, r3.Vec{X: left + 9.8}, r3.Vec{Y: 1}, r3.Vec{Z: 1},
		[][2]float64{{front + 9.8, floor + 5}, {back - 9.8, floor + 5}, {back - 9.8, top + 1}, {front + 9.8, top + 1}},
		[]float64{0, 3, 0, 0}, 31)
	tray, err := decad.Cut(ctx, block, cavity)
	require.NoError(t, err)
	ear := shortStepBox(t, doc, -19, -11, -40, -24, floor-8, floor+1)
	out, err := decad.Union(ctx, tray, ear)
	require.NoError(t, err)

	var found *decad.Edge
	for _, f := range out.Faces() {
		for _, l := range f.Loops() {
			for _, e := range l.Edges() {
				s, en := e.Start().Position().Value, e.End().Position().Value
				if s.Sub(r3.Vec{X: -18, Y: -24, Z: floor}).Len() < 1e-6 && math.Abs(en.X+19) < 1e-6 &&
					math.Abs(en.Y+24) < 1e-6 && en.Z > floor && en.Z < floor+0.5 {
					found = e
				}
			}
		}
	}
	require.NotNil(t, found, "the ear's back root meets the cap round at x = -19")
	arc, ok := found.Curve().(decad.Arc3)
	require.Truef(t, ok, "the junction edge is %T, not an Arc3", found.Curve())
	require.InDelta(t, 2, arc.Radius.Base(), 1e-9)
	require.InDelta(t, 0, arc.Center.Sub(r3.Vec{X: -18, Y: -24, Z: floor + 2}).Len(), 1e-9)
}

func shortStepPrism(t *testing.T, doc *decad.Document, o, u, v r3.Vec, xy [][2]float64, radii []float64, length float64) *decad.Body {
	s := shortStepSketch(t, o, u, v)
	n := len(xy)
	pts := make([]*sketch.Point, n)
	for i, p := range xy {
		pts[i] = s.CreatePoint(p[0], p[1])
	}
	lines := make([]*sketch.Line, n)
	for i := range xy {
		j := (i + 1) % n
		lines[i] = s.CreateLine(pts[i], pts[j])
		if xy[i][0] == xy[j][0] {
			s.AddConstraint(sketch.NewVertical(lines[i]), sketch.NewDistancePointLine(s.Origin(), lines[i], math.Abs(xy[i][0])))
		} else {
			s.AddConstraint(sketch.NewHorizontal(lines[i]), sketch.NewDistancePointLine(s.Origin(), lines[i], math.Abs(xy[i][1])))
		}
	}
	for i := range xy {
		if radii[i] == 0 {
			continue
		}
		_, err := sketch.CreateFillet(s, lines[(i+n-1)%n], lines[i], pts[i], radii[i])
		require.NoError(t, err)
	}
	b, err := decad.Extrude(t.Context(), doc, s, shortStepProfile(t, s), decad.Distance{D: units.Millimeters(length), Dir: decad.Along})
	require.NoError(t, err)
	return b
}

func shortStepBox(t *testing.T, doc *decad.Document, x0, x1, y0, y1, z0, z1 float64) *decad.Body {
	s := shortStepSketch(t, r3.Vec{Z: z0}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	r := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(r.A)
	b, err := decad.Extrude(t.Context(), doc, s, shortStepProfile(t, s), decad.Distance{D: units.Millimeters(z1 - z0), Dir: decad.Along})
	require.NoError(t, err)
	return b
}
