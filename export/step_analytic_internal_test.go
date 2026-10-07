package export

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// arcPrisms are the two arc-bearing prisms the analytic arm's orientation
// tests read: a half disc of radius 10 swept 5 mm (a convex partial wall and
// two caps whose loops are an arc and a line), and a 20×10 plate swept 4 mm
// with a radius-3 semicircular notch in its top edge (a concave partial
// wall).
func arcPrisms(t *testing.T) map[string]*decad.Body {
	t.Helper()
	doc := decad.New()
	w := sketch.NewWorld()
	extrude := func(s *sketch.Sketch, h float64) *decad.Body {
		_, err := s.Solve(t.Context())
		require.NoError(t, err)
		profiles := s.Profiles()
		require.Len(t, profiles, 1)
		body, err := doc.Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
		require.NoError(t, err)
		return body
	}
	fixed := func(s *sketch.Sketch, u, v float64) *sketch.Point {
		p := s.CreatePoint(u, v)
		s.Fix(p)
		return p
	}

	half, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c, a, b := fixed(half, 0, 0), fixed(half, 10, 0), fixed(half, -10, 0)
	half.CreateArc(c, a, b)
	half.CreateLine(b, a)

	notch, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	p0, p1, p2, p3 := fixed(notch, 0, 0), fixed(notch, 20, 0), fixed(notch, 20, 10), fixed(notch, 13, 10)
	q0, q1, nc := fixed(notch, 7, 10), fixed(notch, 0, 10), fixed(notch, 10, 10)
	notch.CreateLine(p0, p1)
	notch.CreateLine(p1, p2)
	notch.CreateLine(p2, p3)
	notch.CreateArc(nc, q0, p3)
	notch.CreateLine(q0, q1)
	notch.CreateLine(q1, p0)

	return map[string]*decad.Body{"half disc": extrude(half, 5), "notched plate": extrude(notch, 4)}
}

// vectorArea is ½∮ r × dr over the loop, each arc sampled finely along its own
// sweep: the test's own oracle for which way a loop turns. For a planar loop
// it is the area times the normal the loop runs counter-clockwise about; for
// a loop on a cylinder it is the integral of that normal over the patch.
func vectorArea(t *testing.T, loop *decad.Loop) r3.Vec {
	t.Helper()
	var pts []r3.Vec
	for _, ce := range loop.CoEdges() {
		edge := ce.Edge()
		arc, ok := edge.Curve().(decad.Arc3)
		if !ok {
			pts = append(pts, ce.Start().Position().Value)
			continue
		}
		sweep, ok := arcSweep(edge, arc)
		require.True(t, ok)
		u, ok := edge.Start().Position().Value.Sub(arc.Center).Normalize()
		require.True(t, ok)
		v := arc.Axis.Cross(u)
		r, err := arc.Radius.In(units.Millimeter)
		require.NoError(t, err)
		const n = 512
		samples := make([]r3.Vec, n)
		for i := range n {
			s, c := math.Sincos(sweep * float64(i) / n)
			samples[i] = arc.Center.Add(u.Scale(r * c)).Add(v.Scale(r * s))
		}
		if !ce.IsForward() {
			end := edge.End().Position().Value
			samples = append([]r3.Vec{end}, reverseVecs(samples[1:])...)
		}
		pts = append(pts, samples...)
	}
	sum := r3.Vec{}
	for i, p := range pts {
		sum = sum.Add(p.Cross(pts[(i+1)%len(pts)]))
	}
	return sum.Scale(0.5)
}

// patchMidRadial is the unit radial direction at the middle of the loop's
// first arc: where a partial cylinder wall's patch faces.
func patchMidRadial(t *testing.T, loop *decad.Loop) r3.Vec {
	t.Helper()
	for _, ce := range loop.CoEdges() {
		arc, ok := ce.Edge().Curve().(decad.Arc3)
		if !ok {
			continue
		}
		sweep, ok := arcSweep(ce.Edge(), arc)
		require.True(t, ok)
		u, ok := ce.Edge().Start().Position().Value.Sub(arc.Center).Normalize()
		require.True(t, ok)
		s, c := math.Sincos(sweep / 2)
		return u.Scale(c).Add(arc.Axis.Cross(u).Scale(s))
	}
	t.Fatal("a partial wall loop carries an arc")
	return r3.Vec{}
}

func reverseVecs(in []r3.Vec) []r3.Vec {
	out := make([]r3.Vec, len(in))
	for i, p := range in {
		out[len(in)-1-i] = p
	}
	return out
}

// TestAnalyticArcLoopsRunCounterClockwiseAboutTheirFaceNormal checks the
// analytic arm's orientation decisions against vectorArea on every arc face
// of both prisms. A plane loop carrying an arc must run counter-clockwise
// about the placement axis arcLoopPlacement states. A partial cylinder wall's
// loop, once reversed or not as partialWallSense says, must run
// counter-clockwise about the STEP face normal: the radial direction at the
// wall's middle when the face keeps the surface's sense, its negation
// otherwise. The half disc's wall is convex (sense kept) and the notch's
// concave (sense flipped), so both branches are read. Shown to fail with
// loopAreaAbout's arc segment term deleted (the half disc's caps then read a
// zero chord area and refuse) and with partialWallArea's arc sign inverted
// (both walls came out walked clockwise).
func TestAnalyticArcLoopsRunCounterClockwiseAboutTheirFaceNormal(t *testing.T) {
	t.Parallel()
	for name, body := range arcPrisms(t) {
		t.Run(name, func(t *testing.T) {
			ok, err := supportsAnalyticSTEP(t.Context(), body)
			require.NoError(t, err)
			require.True(t, ok, "an arc-bearing prism takes the analytic writer")
			walls, arcPlanes := 0, 0
			for _, face := range body.Faces() {
				loops := face.Loops()
				switch surface := face.Surface().(type) {
				case decad.Plane:
					if !loopHasArc(loops[0]) {
						continue
					}
					arcPlanes++
					_, axis, _, err := arcLoopPlacement(face, loops[0])
					require.NoError(t, err)
					require.Positive(t, vectorArea(t, loops[0]).Dot(axis))
				case decad.Cylinder:
					require.Len(t, loops, 1)
					walls++
					start := loops[0].CoEdges()[0].Start().Position().Value
					radial := start.Sub(surface.Origin)
					reference, ok := radial.Sub(surface.Axis.Scale(radial.Dot(surface.Axis))).Normalize()
					require.True(t, ok)
					sameSense, reverse, err := partialWallSense(face, surface, loops[0], reference)
					require.NoError(t, err)
					// The patch's mean radial direction, read off the middle of
					// one of its arcs: the vector area of a loop running
					// counter-clockwise about the radial points along it.
					mid := patchMidRadial(t, loops[0])
					area := vectorArea(t, loops[0])
					stepNormal := mid
					if !sameSense {
						stepNormal = mid.Scale(-1)
					}
					emitted := area
					if reverse {
						emitted = area.Scale(-1)
					}
					require.Positive(t, emitted.Dot(stepNormal), "the STEP loop runs counter-clockwise about the face normal")
				}
			}
			require.Equal(t, 1, walls)
			require.Equal(t, 2, arcPlanes)
		})
	}
}

// TestAnalyticArcLoopAreas pins the two orientation readings' magnitudes on
// the half disc: a cap's loop encloses 50π mm² (its chord through the centre
// encloses none, so the arc's segment term is the whole of it), and the wall
// spans π radians over 5 mm in the (θ, z) parameter plane. Shown to fail with
// loopAreaAbout's segment term deleted and with partialWallArea's θ advance
// deleted.
func TestAnalyticArcLoopAreas(t *testing.T) {
	t.Parallel()
	body := arcPrisms(t)["half disc"]
	for _, face := range body.Faces() {
		loops := face.Loops()
		switch surface := face.Surface().(type) {
		case decad.Plane:
			if !loopHasArc(loops[0]) {
				continue
			}
			area, ok := loopAreaAbout(loops[0], surface.Frame.U(), surface.Frame.V(), surface.Frame.N())
			require.True(t, ok)
			require.InDelta(t, 50*math.Pi, math.Abs(area), 1e-9)
		case decad.Cylinder:
			area, ok := partialWallArea(loops[0], surface)
			require.True(t, ok)
			require.InDelta(t, 5*math.Pi, math.Abs(area), 1e-9)
		}
	}
}
