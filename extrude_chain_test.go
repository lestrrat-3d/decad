package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md §13's public-surface tests for the
// first chain-extrude increment: ExtrudeChain over a chain of exactly one
// straight segment, and the SweepChain/LoftChain staged refusals (§13.5,
// §14). Multi-segment and curved-wall chains, and RevolveChain, land in a
// later increment (§15's T131-T135, T140-T141).

// lineChainSketch builds a sketch holding one 40 mm horizontal line on the XY
// plane and nothing else — docs/surface-design.md §15's T130 fixture:
// s.Profiles() is empty and s.Chains() has length 1.
func lineChainSketch(tb testing.TB) (*sketch.Sketch, *sketch.Chain) {
	tb.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(tb, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(40, 0)
	s.Fix(a)
	s.Fix(b)
	s.CreateLine(a, b)
	_, err = s.Solve(tb.Context())
	require.NoError(tb, err)
	require.Empty(tb, s.Profiles(), "an open line closes no region")
	chains := s.Chains()
	require.Len(tb, chains, 1, "one connected open run")
	return s, chains[0]
}

// TestExtrudeChainOneLineRibbon is docs/surface-design.md's T130 in full: a
// single 40 mm line, the only non-construction entity in its sketch,
// extruded 10 mm Along into a single-face ribbon sheet.
func TestExtrudeChainOneLineRibbon(t *testing.T) {
	t.Parallel()
	s, ch := lineChainSketch(t)
	doc := decad.New()
	body, err := doc.ExtrudeChain(s, ch, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, body.Kind())
	require.False(t, body.IsSolid())
	_, err = body.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	_, err = body.Centroid()
	require.ErrorIs(t, err, decad.ErrNotSolid)

	require.Len(t, body.Faces(), 1)
	decadtest.MeasuresArea(t, body, units.SquareMillimeters(400), decadtest.Exactly())

	free, err := decad.Edges(decad.Free()).Exactly(4).SelectEdges(body)
	require.NoError(t, err)
	for _, e := range free {
		require.True(t, e.IsFree())
	}

	decadtest.MeasuresBounds(t, body, r3.NewVec(0, 0, 0), r3.NewVec(40, 0, 10), decadtest.Exactly())

	require.Equal(t, []*decad.Body{body}, doc.Bodies())
}

// TestExtrudeChainRibbonReadsValid is docs/surface-design.md §9.1's fourth
// validity leg, restated for a chain-fed prism: `sketch` already proved
// T130's single line simple, ExtrudeChain refuses a non-positive height, and
// a simple planar curve crossed with a positive interval cannot
// self-intersect (§13.4). The ribbon's three structural legs hold by
// construction, so the whole audit admits ValidityValid with no diagnostics
// — never ValidityUndecided, which is what a payload without this leg's own
// arm would read instead.
func TestExtrudeChainRibbonReadsValid(t *testing.T) {
	t.Parallel()
	s, ch := lineChainSketch(t)
	doc := decad.New()
	body, err := doc.ExtrudeChain(s, ch, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br, err := rep.ForBody(body)
	require.NoError(t, err)
	require.Equal(t, decad.ValidityValid, br.Validity.Outcome,
		"a chain-fed prism ribbon earns the fourth leg by construction, exactly as its profile-fed sibling does")
	require.Empty(t, br.Validity.Diagnostics)
}

// TestExtrudeChainDiagonalLineAreaBoundEnclosesAnalyticValue builds a
// diagonal (irrational-length) one-line chain, so the wall area's bound is
// genuinely nonzero (docs/surface-design.md §13.4's boundedAdd/boundedMul
// composition, never a raw float product). Shown-to-fail: replacing
// evalChainExtrudeContext's boundedMul(measuredScalar(w.length,
// w.lengthBound), height) with an uncharged exactScalar(w.length*height.value)
// turns the enclosure assertion below red — see this PR's report for the
// exact failure text.
func TestExtrudeChainDiagonalLineAreaBoundEnclosesAnalyticValue(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(10, 10)
	s.Fix(a)
	s.Fix(b)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Empty(t, s.Profiles())
	chains := s.Chains()
	require.Len(t, chains, 1)

	doc := decad.New()
	body, err := doc.ExtrudeChain(s, chains[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness, "a diagonal line's length is irrational")
	require.Greater(t, area.Bound.Base(), 0.0)
	analytic := 10 * math.Sqrt(2) * 10
	require.LessOrEqual(t, math.Abs(area.Value.Base()-analytic), area.Bound.Base(),
		"the published area must enclose the analytic value within its own proven bound")
}

// TestExtrudeChainRejectsUncertifiedFragment is docs/surface-design.md's
// T136: two lines crossing at their midpoints split into four one-edge
// chains — sketch's own C3 fixture — each Partial fragment TExact == true
// under a pure line/line kernel. Adding an untouched spline elsewhere
// withholds certification sketch-wide (docs/sketch-seam-design.md §1), so
// every fragment — the chain's included — reads TExact == false, and a whole
// edge could never demonstrate this refusal: it never consults the flag at
// all.
func TestExtrudeChainRejectsUncertifiedFragment(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateLine(s.CreatePoint(-5, 0), s.CreatePoint(5, 0))
	s.CreateLine(s.CreatePoint(0, -5), s.CreatePoint(0, 5))
	_, err = s.CreateSpline(s.CreatePoint(100, 100), s.CreatePoint(110, 100), s.CreatePoint(110, 110), s.CreatePoint(100, 110))
	require.NoError(t, err)
	require.Empty(t, s.Profiles(), "a bare crossing encloses nothing")

	chains := s.Chains()
	require.Len(t, chains, 5, "four half-lines meet at the crossing, plus the untouched spline's own chain")
	var ch *sketch.Chain
	for _, candidate := range chains {
		if _, ok := candidate.Entities[0].(*sketch.Line); ok {
			ch = candidate
			break
		}
	}
	require.NotNil(t, ch, "one of the crossing's own four half-line fragments")
	require.Len(t, ch.Edges, 1, "each walk stops at the degree-4 vertex")
	require.True(t, ch.Edges[0].Partial, "each edge is half of its line")
	require.False(t, ch.Edges[0].TExact, "the untouched spline withholds certification sketch-wide")

	doc := decad.New()
	_, err = doc.ExtrudeChain(s, ch, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
	require.Empty(t, doc.Bodies())
}

// TestExtrudeChainRejectsStaleChain is docs/surface-design.md's T137's first
// half: a chain held across a later Solve reads stale.
func TestExtrudeChainRejectsStaleChain(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(40, 0)
	s.Fix(a)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	ch := s.Chains()[0]

	s.AddConstraint(sketch.NewDistance(a, b, 55))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.True(t, ch.IsStale(), "the solve should have moved the sketch under the chain")

	doc := decad.New()
	_, err = doc.ExtrudeChain(s, ch, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrStaleProfile)
	require.Empty(t, doc.Bodies())

	fresh := s.Chains()[0]
	_, err = doc.ExtrudeChain(s, fresh, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
}

// TestExtrudeChainRejectsInvalidChain is docs/surface-design.md's T137's
// second half: a chain that doubles back over itself — sketch's own C6
// fixture — reads Chain.Valid == false, and ExtrudeChain refuses it rather
// than silently sweeping it.
func TestExtrudeChainRejectsInvalidChain(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(10, 0)
	s.CreateLine(a, b)
	s.CreateLine(b, s.CreatePoint(2, 0)) // back along the first line

	chains := s.Chains()
	require.Len(t, chains, 1, "the shared point joins them into one walk")
	require.True(t, chains[0].SelfIntersecting, "the walk retraces its own path")
	require.False(t, chains[0].Valid)

	doc := decad.New()
	_, err = doc.ExtrudeChain(s, chains[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrInvalidProfile)
	require.Empty(t, doc.Bodies())
}

// TestExtrudeChainMultiSegmentWallSet is docs/surface-design.md's T131: an
// open three-segment walk — line, arc, line — each meeting the next at a
// shared point, built into a three-face ribbon whose per-wall area bounds
// compose through boundedAdd rather than as raw floats.
func TestExtrudeChainMultiSegmentWallSet(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(10, 0)
	c := s.CreatePoint(15, 5)
	d := s.CreatePoint(25, 5)
	s.Fix(a)
	s.CreateLine(a, b)
	s.CreateArc(s.CreatePoint(10, 5), b, c)
	s.CreateLine(c, d)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Empty(t, s.Profiles())
	chains := s.Chains()
	require.Len(t, chains, 1, "one connected open run")
	require.Len(t, chains[0].Edges, 3)

	doc := decad.New()
	body, err := doc.ExtrudeChain(s, chains[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	require.Len(t, body.Faces(), 3)

	free, err := decad.Edges(decad.Free()).Exactly(8).SelectEdges(body)
	require.NoError(t, err)
	for _, e := range free {
		require.True(t, e.IsFree())
	}

	var lineWalls, arcWalls int
	sumValue, sumBound := 0.0, 0.0
	for _, f := range body.Faces() {
		area, err := f.Area()
		require.NoError(t, err)
		switch f.Surface().Kind() {
		case decad.KindPlane:
			lineWalls++
			require.Equal(t, decad.Exact, area.Exactness, "a line wall's area is exact")
			require.Zero(t, area.Bound.Base())
		case decad.KindCylinder:
			arcWalls++
			require.Equal(t, decad.Approximate, area.Exactness, "the arc wall's area carries rθ's own evaluation bound")
			require.Greater(t, area.Bound.Base(), 0.0)
		default:
			t.Fatalf("unexpected chain wall surface kind %v", f.Surface().Kind())
		}
		sumValue += area.Value.Base()
		sumBound += area.Bound.Base()
	}
	require.Equal(t, 2, lineWalls)
	require.Equal(t, 1, arcWalls)

	bodyArea, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, sumValue, bodyArea.Value.Base(), "the body's own area is the three walls' own values summed")
	require.Equal(t, decad.Approximate, bodyArea.Exactness)
	require.InDelta(t, sumBound, bodyArea.Bound.Base(), 1e-9,
		"the body's own area bound is the three walls' own bounds folded through boundedAdd")
}

// TestExtrudeChainRectangleMinusOneSideBuildsThreeWalls is docs/surface-design.md's
// T132: a rectangle with one side erased mints one wall per surviving side —
// three, not four — the ribbon this section opens on.
func TestExtrudeChainRectangleMinusOneSideBuildsThreeWalls(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(10, 0)
	c := s.CreatePoint(10, 6)
	d := s.CreatePoint(0, 6)
	s.Fix(a)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, d)
	// The fourth side, d-a, is never drawn.
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Empty(t, s.Profiles())
	chains := s.Chains()
	require.Len(t, chains, 1)
	require.Len(t, chains[0].Edges, 3)

	doc := decad.New()
	body, err := doc.ExtrudeChain(s, chains[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	require.Len(t, body.Faces(), 3, "the erased side mints no wall")
	_, err = body.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)

	_, err = decad.Edges(decad.Free()).Exactly(8).SelectEdges(body)
	require.NoError(t, err)
}

// TestExtrudeChainOverEachChainOfACutVertex is docs/surface-design.md's T133:
// three lines meeting at one point publish three separate one-edge chains
// (§13.1's cut-vertex rule), and ExtrudeChain over each in turn builds its
// own one-face ribbon — three bodies, never one.
func TestExtrudeChainOverEachChainOfACutVertex(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateLine(center, s.CreatePoint(10, 0))
	s.CreateLine(center, s.CreatePoint(0, 10))
	s.CreateLine(center, s.CreatePoint(-10, 0))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Empty(t, s.Profiles())
	chains := s.Chains()
	require.Len(t, chains, 3, "three lines meeting at one point publish three chains")

	doc := decad.New()
	for _, ch := range chains {
		body, err := doc.ExtrudeChain(s, ch, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		require.NoError(t, err)
		require.Len(t, body.Faces(), 1)
	}
	require.Len(t, doc.Bodies(), 3, "each chain builds its own body, never one shared body")
}

// TestExtrudeChainFreeformWallAreaNeverPublishesTheLengthUnderestimate is
// docs/surface-design.md's T141: a chain holding a Tier A free-form fragment
// builds a NURBSSurface wall whose area is Approximate over
// spline_length.go's proven bracket, and Chain.Length·h — the sampling-
// convergent underestimate §13.3 forbids ExtrudeChain from ever publishing —
// sits at or below that interval's own lower end, never inside it.
func TestExtrudeChainFreeformWallAreaNeverPublishesTheLengthUnderestimate(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	p0 := s.CreatePoint(0, 0)
	p1 := s.CreatePoint(10, 5)
	p2 := s.CreatePoint(20, 8)
	p3 := s.CreatePoint(30, 9)
	s.Fix(p0)
	_, err = s.CreateSpline(p0, p1, p2, p3)
	require.NoError(t, err)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	chains := s.Chains()
	require.Len(t, chains, 1)
	chainLength := chains[0].Length

	const h = 10.0
	doc := decad.New()
	body, err := doc.ExtrudeChain(s, chains[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)

	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Greater(t, area.Bound.Base(), 0.0)

	lo := area.Value.Base() - area.Bound.Base()
	underestimate := chainLength * h
	require.LessOrEqual(t, underestimate, lo,
		"Chain.Length's sampling-convergent underestimate must sit at or below the enclosure's own lower end, never inside it")
}

// TestChainFedRefusals is docs/surface-design.md's T139: a SweepChain over a
// COMPOSITE path is ErrUnsupported (R34) ahead of the increment that builds
// docs/sweep-design.md §15.1's join, and RevolveChain handed a chain with both
// free ends on the resolved axis is ErrUnsupported (R22), pending its
// closed-sheet pole topology. A one-span straight path builds instead, which
// sweep_chain_test.go owns; LoftChain's own staged refusal is the curved pair
// (R36), which loft_chain_test.go owns beside the pairing it does build.
func TestChainFedRefusals(t *testing.T) {
	t.Parallel()
	s, ch := lineChainSketch(t)
	doc := decad.New()

	_, err := doc.SweepChain(t.Context(), s, ch, nil)
	require.ErrorIs(t, err, decad.ErrDegenerate, "a nil path is refused before the staged refusal")

	path, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 20)},
		decad.ArcThrough{Through: r3.NewVec(5, 0, 25), End: r3.NewVec(10, 0, 20)},
	)
	require.NoError(t, err)
	_, err = doc.SweepChain(t.Context(), s, ch, path)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Empty(t, doc.Bodies())

	// R22: a half-circle chain with both free ends on the resolved axis.
	axisWorld := sketch.NewWorld()
	axisSketch, err := axisWorld.CreateSketch(axisWorld.XY())
	require.NoError(t, err)
	start := axisSketch.CreatePoint(0, 0)
	axisSketch.Fix(start)
	end := axisSketch.CreatePoint(10, 0)
	center := axisSketch.CreatePoint(5, 0)
	axisSketch.CreateArc(center, end, start)
	_, err = axisSketch.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, axisSketch.Chains(), 1)
	axisChain := axisSketch.Chains()[0]

	_, err = doc.RevolveChain(axisSketch, axisChain,
		decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}},
		decad.FullRevolution{})
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "both free ends")
	require.Empty(t, doc.Bodies())
}

// TestExtrudeChainRibbonsWeldAtAProvenCoincidentPair is docs/surface-design.md's
// T140: a ribbon is an ordinary Stitch operand, admitted by the existing
// gates and not by a new one. Two independently built ribbons whose free-end
// edges are a proven-coincident, zero-bound pair weld under Table J's Line3
// row, and the result is a BodySheet whose Edges(Free()) resolves to the
// remaining 6 edges, not 8.
func TestExtrudeChainRibbonsWeldAtAProvenCoincidentPair(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s1, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s1.CreatePoint(0, 0)
	b := s1.CreatePoint(10, 0)
	s1.Fix(a)
	s1.CreateLine(a, b)
	_, err = s1.Solve(t.Context())
	require.NoError(t, err)
	ch1 := s1.Chains()[0]

	// A separate sketch, so the two lines never arrange into one connected
	// chain even though their free ends are bit-identical: (10, 0) is where
	// the first ribbon's wall ends and the second's begins.
	s2, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s2.CreatePoint(10, 0)
	d := s2.CreatePoint(10, 10)
	s2.Fix(c)
	s2.CreateLine(c, d)
	_, err = s2.Solve(t.Context())
	require.NoError(t, err)
	ch2 := s2.Chains()[0]

	doc := decad.New()
	body1, err := doc.ExtrudeChain(s1, ch1, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	body2, err := doc.ExtrudeChain(s2, ch2, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)

	stitched, err := decad.Stitch(t.Context(), body1, body2)
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, stitched.Kind())

	_, err = decad.Edges(decad.Free()).Exactly(6).SelectEdges(stitched)
	require.NoError(t, err)
}

// TestChainFedRefusalsStillRunTheSeamGates confirms SweepChain and LoftChain
// run RecordChain's own gates before the staged refusal (docs/api-design.md
// §8: "All four run §7's four gates over RecordChain"), so a stale chain is
// reported as stale rather than masked by ErrUnsupported.
func TestChainFedRefusalsStillRunTheSeamGates(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(40, 0)
	s.Fix(a)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	ch := s.Chains()[0]

	s.AddConstraint(sketch.NewDistance(a, b, 55))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.True(t, ch.IsStale())

	doc := decad.New()
	path, err := decad.NewPath(r3.NewVec(0, 0, 0), decad.LineTo{End: r3.NewVec(0, 0, 10)})
	require.NoError(t, err)
	_, err = doc.SweepChain(t.Context(), s, ch, path)
	require.ErrorIs(t, err, decad.ErrStaleProfile)

	_, err = doc.LoftChain(t.Context(), s, ch, s, ch)
	require.ErrorIs(t, err, decad.ErrStaleProfile)
}
