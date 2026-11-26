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

// This file covers an open tangent chain that ends on an oblique end face
// (agent-docs/modify-design.md Part M §5.5 step 5, "An oblique end
// face"): a 20 mm wide arm, 17 mm tall with a round end at y = 55, cut down to
// 8 mm beyond a ramp that rises from the outer face at its foot to the top at
// y = 10. The outer face's outline (two sides and the round end) is one convex
// chain whose ends meet the ramp, and the side walls run on up the ramp's sides
// past them.

const (
	rampArmHalf  = 10.0
	rampArmFace  = 8.0
	rampArmTop   = 17.0
	rampArmPin   = 55.0
	rampArmSize  = 0.5
	rampArmAngle = 30 * math.Pi / 180
)

// rampArmFoot is where a ramp of rampArmAngle from the outer face meets it.
var rampArmFoot = rampArmHalf + (rampArmTop-rampArmFace)/math.Tan(rampArmAngle)

// rampArm builds the arm. riser makes the end face upright at the ramp's foot (a
// step) in place of the ramp; capChamfer chamfers both caps' outlines before
// the cut; skew turns the cutting tool by that many radians about z, so the
// ramp no longer meets the side walls square.
func rampArm(t *testing.T, riser, capChamfer bool, skew float64) *decad.Body {
	t.Helper()
	w, doc := sketch.NewWorld(), decad.New()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	corners := [][2]float64{{rampArmHalf, -10}, {rampArmHalf, rampArmPin}, {-rampArmHalf, rampArmPin}, {-rampArmHalf, -10}}
	pts := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		pts[i] = s.CreatePoint(c[0], c[1])
		s.Fix(pts[i])
	}
	center := s.CreatePoint(0, rampArmPin)
	s.Fix(center)
	s.CreateLine(pts[0], pts[1])
	s.CreateArc(center, pts[1], pts[2])
	s.CreateLine(pts[2], pts[3])
	s.CreateLine(pts[3], pts[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(rampArmTop), Dir: decad.Along})
	require.NoError(t, err)
	if capChamfer {
		loops := decad.Edges(decad.OuterLoopOf(decad.CapStart(body))).Or(decad.OuterLoopOf(decad.CapEnd(body)))
		body, err = decad.Chamfer(body, loops, units.Millimeters(rampArmSize)).Do(t.Context())
		require.NoError(t, err)
	}
	return rampCutOf(t, w, doc, body, riser, skew)
}

// rampCut cuts body down to rampArmFace beyond a ramp of rampArmAngle that rises from rampArmFoot to the top at
// y = rampArmHalf, across the whole width of a body that lies within x = ±15.
func rampCut(t *testing.T, w *sketch.World, doc *decad.Document, body *decad.Body) *decad.Body {
	t.Helper()
	return rampCutOf(t, w, doc, body, false, 0)
}

// rampCutOf is rampCut with an upright riser at the foot in place of the ramp (riser), and the tool turned by skew
// radians about z.
func rampCutOf(t *testing.T, w *sketch.World, doc *decad.Document, body *decad.Body, riser bool, skew float64) *decad.Body {
	t.Helper()
	// The tool's section lies in the plane x = −15, with u along +y and v along +z, turned by skew.
	across := r3.Vec{X: math.Cos(skew), Y: math.Sin(skew)}
	along := r3.Vec{X: -math.Sin(skew), Y: math.Cos(skew)}
	f, err := r3.NewFrame(across.Scale(-15), along, r3.Vec{Z: 1})
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	top := rampArmHalf - 1/math.Tan(rampArmAngle)
	if riser {
		top = rampArmFoot
	}
	far := rampArmPin + rampArmHalf + 1
	section := [][2]float64{{rampArmFoot, rampArmFace}, {far, rampArmFace}, {far, rampArmTop + 1}, {top, rampArmTop + 1}}
	ts, err := w.CreateSketch(plane)
	require.NoError(t, err)
	tp := make([]*sketch.Point, len(section))
	for i, c := range section {
		tp[i] = ts.CreatePoint(c[0], c[1])
		ts.Fix(tp[i])
	}
	for i := range tp {
		ts.CreateLine(tp[i], tp[(i+1)%len(tp)])
	}
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, ts, ts.Profiles()[0],
		decad.Distance{D: units.Millimeters(30), Dir: decad.Along})
	require.NoError(t, err)
	out, err := decad.Cut(body, tool).Do(t.Context())
	require.NoError(t, err)
	return out
}

// rampArmOutlineEdges are the outer face's outline edges beyond the ramp: two sides and the round end.
func rampArmOutlineEdges(t *testing.T, b *decad.Body) []*decad.Edge {
	t.Helper()
	var edges []*decad.Edge
	for _, e := range b.Edges() {
		p, q := e.Start().Position().Value, e.End().Position().Value
		if math.Abs(p.Z-rampArmFace) > 1e-9 || math.Abs(q.Z-rampArmFace) > 1e-9 || p.Y < 30 && q.Y < 30 {
			continue
		}
		edges = append(edges, e)
	}
	require.Len(t, edges, 3)
	return edges
}

// rampArmTopEdge is the ramp's edge with the arm's top face.
func rampArmTopEdge(t *testing.T, b *decad.Body) *decad.Edge {
	t.Helper()
	var found *decad.Edge
	for _, e := range b.Edges() {
		p, q := e.Start().Position().Value, e.End().Position().Value
		if math.Abs(p.Z-rampArmTop) < 1e-9 && math.Abs(q.Z-rampArmTop) < 1e-9 && math.Abs(p.Y-rampArmHalf) < 1e-9 &&
			math.Abs(q.Y-rampArmHalf) < 1e-9 {
			require.Nil(t, found, "one ramp top edge")
			found = e
		}
	}
	require.NotNil(t, found)
	return found
}

// rampFace is the arm's one planar face whose normal leans 30 degrees from z.
func rampFace(t *testing.T, b *decad.Body) *decad.Face {
	t.Helper()
	var found *decad.Face
	for _, f := range b.Faces() {
		p, ok := f.Surface().(decad.Plane)
		if !ok {
			continue
		}
		n := p.Frame.N()
		if math.Abs(math.Abs(n.Y)-math.Sin(rampArmAngle)) < 1e-9 && math.Abs(math.Abs(n.Z)-math.Cos(rampArmAngle)) < 1e-9 {
			require.Nil(t, found, "one ramp face")
			found = f
		}
	}
	require.NotNil(t, found)
	return found
}

// removedVolume is the held volume a blend took from before to make after.
func removedVolume(t *testing.T, before, after *decad.Body) float64 {
	t.Helper()
	v0, err := before.Volume()
	require.NoError(t, err)
	v1, err := after.Volume()
	require.NoError(t, err)
	return v0.Value.Mag() - v1.Value.Mag()
}

func TestBlendChainEndsOnRamp(t *testing.T) {
	t.Parallel()
	const size = rampArmSize
	tri := size * size / 2
	// A row at depth h below the outer face meets the ramp h·cot(30°) farther
	// along the side than the riser of a step at the same foot, so each end of
	// the chain removes the section's first moment about the face times
	// cot(30°) less: tri·(size/3)·cot(30°). The ramp takes a cap of tri over
	// |d·n_G| = sin(30°).
	taper := tri * size / 3 / math.Tan(rampArmAngle)
	capArea := tri / math.Sin(rampArmAngle)

	t.Run("the outline chamfers onto the ramp, which takes both caps", func(t *testing.T) {
		t.Parallel()
		ramp := rampArm(t, false, false, 0)
		out, err := decad.Chamfer(ramp, namedEdges(rampArmOutlineEdges(t, ramp)), units.Millimeters(size)).Do(t.Context())
		require.NoError(t, err)
		// Two sides from the foot to the round end, and a half turn of radius 10
		// whose material lies inside its circle (Pappus on the section).
		want := 2*(tri*(rampArmPin-rampArmFoot)-taper) + math.Pi*tri*(rampArmHalf-size/3)
		v0, err := ramp.Volume()
		require.NoError(t, err)
		v1, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, v0, v1, want)
		a0, err := rampFace(t, ramp).Area()
		require.NoError(t, err)
		a1, err := rampFace(t, out).Area()
		require.NoError(t, err)
		got := a1.Value.Mag() - a0.Value.Mag()
		t.Logf("ramp area change %.9f, want %.9f", got, 2*capArea)
		require.InDelta(t, 2*capArea, got, a0.Bound.Mag()+a1.Bound.Mag())
		requireVerifiedMesh(t, out)

		// The same arm with an upright riser at the foot holds the same facets
		// along the chain's round end, so the two removals differ by the ends
		// alone.
		step := rampArm(t, true, false, 0)
		stepOut, err := decad.Chamfer(step, namedEdges(rampArmOutlineEdges(t, step)),
			units.Millimeters(size)).Do(t.Context())
		require.NoError(t, err)
		diff := removedVolume(t, step, stepOut) - removedVolume(t, ramp, out)
		t.Logf("step less ramp %.12f, want %.12f", diff, 2*taper)
		require.InDelta(t, 2*taper, diff, 1e-9)
	})

	t.Run("the outline fillets onto the ramp", func(t *testing.T) {
		t.Parallel()
		// The fillet's corner region has area r²(1 − π/4), and its centroid lies
		// r(10 − 3π)/(3(4 − π)) below the face.
		const r = rampArmSize
		corner := r * r * (1 - math.Pi/4)
		depth := r * (10 - 3*math.Pi) / (3 * (4 - math.Pi))
		ramp := rampArm(t, false, false, 0)
		out, err := decad.Fillet(ramp, namedEdges(rampArmOutlineEdges(t, ramp)), units.Millimeters(r)).Do(t.Context())
		require.NoError(t, err)
		// The rows on an oblique end step finer than a step's (rowStep), so the
		// held strips differ along the whole chain: the removal is read against
		// its closed form within the bound, not against the step's.
		want := 2*corner*(rampArmPin-rampArmFoot-depth/math.Tan(rampArmAngle)) + math.Pi*corner*(rampArmHalf-depth)
		v0, err := ramp.Volume()
		require.NoError(t, err)
		v1, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, v0, v1, want)
		a0, err := rampFace(t, ramp).Area()
		require.NoError(t, err)
		a1, err := rampFace(t, out).Area()
		require.NoError(t, err)
		got := a1.Value.Mag() - a0.Value.Mag()
		t.Logf("ramp area change %.9f, want %.9f", got, 2*corner/math.Sin(rampArmAngle))
		require.InDelta(t, 2*corner/math.Sin(rampArmAngle), got, a0.Bound.Mag()+a1.Bound.Mag())
		requireVerifiedMesh(t, out)
	})

	t.Run("one selection holds the chain and the ramp's top edge", func(t *testing.T) {
		t.Parallel()
		// The top edge ends on the cap chamfer's planes and shares no vertex with
		// the chain: it takes its own route first, and the chain is then found
		// again and built, as two calls in that order build it.
		arm := rampArm(t, false, true, 0)
		both, err := decad.Chamfer(arm,
			namedEdges(append(rampArmOutlineEdges(t, arm), rampArmTopEdge(t, arm))), units.Millimeters(size)).Do(t.Context())
		require.NoError(t, err)
		requireVerifiedMesh(t, both)

		apart := rampArm(t, false, true, 0)
		first, err := decad.Chamfer(apart, namedEdges([]*decad.Edge{rampArmTopEdge(t, apart)}),
			units.Millimeters(size)).Do(t.Context())
		require.NoError(t, err)
		second, err := decad.Chamfer(first, namedEdges(rampArmOutlineEdges(t, first)),
			units.Millimeters(size)).Do(t.Context())
		require.NoError(t, err)
		require.InDelta(t, removedVolume(t, apart, second), removedVolume(t, arm, both), 1e-12)
	})

	t.Run("a ramp that does not meet the side walls square is refused", func(t *testing.T) {
		t.Parallel()
		ramp := rampArm(t, false, false, 0.05)
		_, err := decad.Chamfer(ramp, namedEdges(rampArmOutlineEdges(t, ramp)), units.Millimeters(size)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "an open tangent chain's end face does not meet its wall square")
	})
}
