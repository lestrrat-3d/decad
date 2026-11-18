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

// This file covers a straight edge that ends on an oblique end plane
// (agent-docs/modify-design.md Part E §6.1, "An oblique end plane"): the
// chamfer band of an outline drawn with tangent constraints states a rounding
// tilt on its flat patches, a mesh Boolean keeps their Plane tags with that
// tilt and an extent (agent-docs/faceted-source-retention-design.md §3.2, A2,
// Claim F), and the edge where a ramp meets the top face ends on two of them.

// roundedPlate is a 20 × 60 plate, 17 tall, whose four corners are rounded by
// r = 4.5, both caps' outlines chamfered by 0.5, cut down to 8 beyond a 30°
// ramp. solved draws the outline with tangent and radius constraints the way a
// model draws it, so the solve leaves each line–arc join tangent only to
// rounding; otherwise every point is fixed where it belongs and every join is
// exactly tangent.
func roundedPlate(t *testing.T, solved bool) *decad.Body {
	t.Helper()
	w, doc := sketch.NewWorld(), decad.New()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	const half, y0, y1, r = rampArmHalf, -5.0, 55.0, 4.5
	var fixed []*sketch.Point
	pt := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		fixed = append(fixed, p)
		return p
	}
	// The centres and the tangent points, counter-clockwise from the bottom right.
	c := []*sketch.Point{pt(half-r, y0+r), pt(half-r, y1-r), pt(-half+r, y1-r), pt(-half+r, y0+r)}
	p := []*sketch.Point{
		pt(half, y0+r), pt(half, y1-r), pt(half-r, y1), pt(-half+r, y1),
		pt(-half, y1-r), pt(-half, y0+r), pt(-half+r, y0), pt(half-r, y0),
	}
	right, top := s.CreateLine(p[0], p[1]), s.CreateLine(p[2], p[3])
	left, bottom := s.CreateLine(p[4], p[5]), s.CreateLine(p[6], p[7])
	arcs := []*sketch.Arc{s.CreateArc(c[1], p[1], p[2]), s.CreateArc(c[2], p[3], p[4]), s.CreateArc(c[3], p[5], p[6]),
		s.CreateArc(c[0], p[7], p[0])}
	if solved {
		o := s.Origin()
		s.AddConstraint(sketch.NewVertical(right), sketch.NewVertical(left), sketch.NewHorizontal(top),
			sketch.NewHorizontal(bottom), sketch.NewHorizontalDistance(o, p[0], half),
			sketch.NewHorizontalDistance(p[4], o, half), sketch.NewVerticalDistance(o, p[2], y1),
			sketch.NewVerticalDistance(p[6], o, -y0))
		legs := [][2]*sketch.Line{{right, top}, {top, left}, {left, bottom}, {bottom, right}}
		for i, a := range arcs {
			s.AddConstraint(sketch.NewRadius(a, r), sketch.NewTangent(legs[i][0], a), sketch.NewTangent(legs[i][1], a))
		}
	} else {
		for _, q := range fixed {
			s.Fix(q)
		}
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profs := s.Profiles()
	require.Len(t, profs, 1)
	body, err := decad.Extrude(t.Context(), doc, s, profs[0],
		decad.Distance{D: units.Millimeters(rampArmTop), Dir: decad.Along})
	require.NoError(t, err)
	loops := decad.Edges(decad.OuterLoopOf(decad.CapStart(body))).Or(decad.OuterLoopOf(decad.CapEnd(body)))
	body, err = decad.Chamfer(t.Context(), body, loops, units.Millimeters(rampArmSize))
	require.NoError(t, err)
	return rampCut(t, w, doc, body)
}

// galleryArm is the crank handle arm of the decad-gallery repository's
// construction in miniature: a 20 wide outline whose sides run tangent into a
// round pin end at y = 55 and, at the hub end, into r = 4.5 corner arcs that
// an R = 22 end arc closes, every join set by constraints; both caps' outlines
// chamfered by 0.5 and the arm cut down to 8 beyond a 30° ramp.
func galleryArm(t *testing.T) *decad.Body {
	t.Helper()
	w, doc := sketch.NewWorld(), decad.New()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	const half, side, cornerR, endR = rampArmHalf, -10.0, 4.5, 22.0
	o := s.Origin()
	top := s.CreatePoint(0, rampArmPin)
	s.AddConstraint(sketch.NewVerticalPoints(o, top), sketch.NewVerticalDistance(o, top, rampArmPin))
	r2, l2 := s.CreatePoint(half, rampArmPin), s.CreatePoint(-half, rampArmPin)
	sR, sL := s.CreatePoint(half, side), s.CreatePoint(-half, side)
	pin := s.CreateArc(top, r2, l2)
	right, left := s.CreateLine(sR, r2), s.CreateLine(l2, sL)
	s.AddConstraint(sketch.NewRadius(pin, half), sketch.NewTangent(right, pin), sketch.NewTangent(left, pin),
		sketch.NewVertical(right), sketch.NewVertical(left),
		sketch.NewVerticalDistance(o, sR, side), sketch.NewVerticalDistance(o, sL, side))
	cR := r3.Vec{X: half - cornerR, Y: side}
	cB := r3.Vec{Y: side + math.Sqrt((endR-cornerR)*(endR-cornerR)-cR.X*cR.X)}
	pR := cB.Add(cR.Sub(cB).Scale(endR / cR.Sub(cB).Len()))
	cRp, cLp, cBp := s.CreatePoint(cR.X, cR.Y), s.CreatePoint(-cR.X, cR.Y), s.CreatePoint(cB.X, cB.Y)
	pRp, pLp := s.CreatePoint(pR.X, pR.Y), s.CreatePoint(-pR.X, pR.Y)
	arcR, arcL := s.CreateArc(cRp, pRp, sR), s.CreateArc(cLp, sL, pLp)
	arcB := s.CreateArc(cBp, pLp, pRp)
	s.AddConstraint(sketch.NewRadius(arcR, cornerR), sketch.NewTangent(right, arcR),
		sketch.NewRadius(arcL, cornerR), sketch.NewTangent(left, arcL),
		sketch.NewRadius(arcB, endR), sketch.NewTangentCircles(arcB, arcR, true), sketch.NewTangentCircles(arcB, arcL, true))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profs := s.Profiles()
	require.Len(t, profs, 1)
	body, err := decad.Extrude(t.Context(), doc, s, profs[0],
		decad.Distance{D: units.Millimeters(rampArmTop), Dir: decad.Along})
	require.NoError(t, err)
	loops := decad.Edges(decad.OuterLoopOf(decad.CapStart(body))).Or(decad.OuterLoopOf(decad.CapEnd(body)))
	body, err = decad.Chamfer(t.Context(), body, loops, units.Millimeters(rampArmSize))
	require.NoError(t, err)
	return rampCut(t, w, doc, body)
}

// requireTiltedEndPlanes requires the cap chamfer's flat patches beside the
// ramp's top edge to have kept their Plane tags through the cut.
func requireTiltedEndPlanes(t *testing.T, b *decad.Body) {
	t.Helper()
	e := rampArmTopEdge(t, b)
	for _, v := range []*decad.Vertex{e.Start(), e.End()} {
		found := false
		for _, f := range b.Faces() {
			p, ok := f.Surface().(decad.Plane)
			if !ok || math.Abs(math.Abs(p.Frame.N().X)-math.Sqrt2/2) > 1e-9 {
				continue
			}
			d, err := f.DistanceToPoint(t.Context(), v.Position().Value, units.Millimeters(0.1))
			require.NoError(t, err)
			found = found || d.Value.Mag() <= d.Bound.Mag()+1e-9
		}
		require.True(t, found, "a 45° Plane face holds the top edge's end %v", v.Position().Value)
	}
}

func TestChamferEdgeEndsOnTiltedPlane(t *testing.T) {
	t.Parallel()
	const size = rampArmSize
	// The section between the top and the ramp is a triangle of legs 0.5 at the
	// 150° interior angle, area size²·sin(150°)/2, its far corner size·sin(30°)
	// below the top. Each end plane x ± z = ±26.5 meets a row at depth h at
	// x = ±(9.5 + h), so a row runs 19 + 2h and the edge removes
	// A·(19 + 2·h̄), h̄ = size·sin(30°)/3.
	area := size * size * math.Sin(5*math.Pi/6) / 2
	want := area * (2*(rampArmHalf-size) + 2*size*math.Sin(rampArmAngle)/3)

	topChamfer := func(t *testing.T, b *decad.Body) *decad.Body {
		t.Helper()
		out, err := decad.Chamfer(t.Context(), b, namedEdges([]*decad.Edge{rampArmTopEdge(t, b)}),
			units.Millimeters(size))
		require.NoError(t, err)
		v0, err := b.Volume()
		require.NoError(t, err)
		v1, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, v0, v1, want)
		requireVerifiedMesh(t, out)
		return out
	}

	t.Run("a plate drawn with tangent constraints", func(t *testing.T) {
		t.Parallel()
		solved := roundedPlate(t, true)
		requireTiltedEndPlanes(t, solved)
		out := topChamfer(t, solved)

		// The exact plate's end planes state no tilt, and the two remove the same
		// volume to within their published bounds.
		exact := roundedPlate(t, false)
		exactOut := topChamfer(t, exact)
		v, err := out.Volume()
		require.NoError(t, err)
		ve, err := exactOut.Volume()
		require.NoError(t, err)
		require.InDelta(t, removedVolume(t, exact, exactOut), removedVolume(t, solved, out), v.Bound.Mag()+ve.Bound.Mag())
	})

	t.Run("the gallery's arm, its top edge then its outline", func(t *testing.T) {
		t.Parallel()
		arm := galleryArm(t)
		requireTiltedEndPlanes(t, arm)
		out := topChamfer(t, arm)
		_, err := decad.Chamfer(t.Context(), out, namedEdges(rampArmOutlineEdges(t, out)), units.Millimeters(size))
		require.NoError(t, err)
	})

	t.Run("the gallery's arm, its outline and top edge in one call", func(t *testing.T) {
		t.Parallel()
		arm := galleryArm(t)
		out, err := decad.Chamfer(t.Context(), arm,
			namedEdges(append(rampArmOutlineEdges(t, arm), rampArmTopEdge(t, arm))), units.Millimeters(size))
		require.NoError(t, err)
		requireVerifiedMesh(t, out)
	})
}
