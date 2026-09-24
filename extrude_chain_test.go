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

// TestExtrudeChainRefusesMultiSegmentChain confirms this increment's own
// staged boundary: ExtrudeChain builds a chain of exactly one straight
// segment (docs/surface-design.md §14's own two-PR split for this
// capability); a genuine multi-segment chain is ErrUnsupported, staged to a
// later increment rather than built incorrectly.
func TestExtrudeChainRefusesMultiSegmentChain(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(10, 0)
	c := s.CreatePoint(10, 6)
	s.Fix(a)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Empty(t, s.Profiles())
	chains := s.Chains()
	require.Len(t, chains, 1)
	require.Len(t, chains[0].Edges, 2)

	doc := decad.New()
	_, err = doc.ExtrudeChain(s, chains[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Empty(t, doc.Bodies())
}

// TestChainFedRefusals is docs/surface-design.md's T139 for SweepChain and
// LoftChain: each is ErrUnsupported (Table R row R23) even for a valid
// chain, ahead of the increment that states its own pairing rule. The
// RevolveChain-on-axis third of T139 is deferred with RevolveChain itself.
func TestChainFedRefusals(t *testing.T) {
	t.Parallel()
	s, ch := lineChainSketch(t)
	doc := decad.New()

	_, err := doc.SweepChain(t.Context(), s, ch, nil)
	require.ErrorIs(t, err, decad.ErrDegenerate, "a nil path is refused before the staged refusal")

	path, err := decad.NewPath(r3.NewVec(0, 0, 0), decad.LineTo{End: r3.NewVec(0, 0, 10)})
	require.NoError(t, err)
	_, err = doc.SweepChain(t.Context(), s, ch, path)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Empty(t, doc.Bodies())

	_, err = doc.LoftChain(t.Context(), s, ch, s, ch)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Empty(t, doc.Bodies())
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
