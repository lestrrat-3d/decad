package apitest_test

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

// This file tests SweepChain's one-span straight path and the composite paths
// outside §15.8's cardinal class. Standalone arc tests live in sweep_chain_arc_test.go.

// straightSweepPath is the one-span path every axis-aligned fixture below
// sweeps along: from the XY plane's origin, h mm along that plane's own
// positive normal.
func straightSweepPath(tb testing.TB, h float64) *decad.Path {
	tb.Helper()
	path, err := decad.NewPath(r3.NewVec(0, 0, 0), decad.LineTo{End: r3.NewVec(0, 0, h)})
	require.NoError(tb, err)
	return path
}

// TestSweepChainOneLineRibbonMatchesExtrudeChain is docs/surface-design.md's
// T190: the same 40 mm line chain T130 extrudes, swept instead along a
// one-span LineTo path 10 mm up the sketch plane's positive normal. Every
// structural reading is T130's, and the Area comparison against the
// chain Extrude reading is the row's own assertion: the two calls denote one
// ribbon and reach it through two different height derivations — a resolved
// Extent on one side and the path's two recorded points on the other — so
// their agreement pins the new derivation to the landed one, which neither
// reading alone would do.
func TestSweepChainOneLineRibbonMatchesExtrudeChain(t *testing.T) {
	t.Parallel()
	s, ch := lineChainSketch(t)

	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, ch, straightSweepPath(t, 10))
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, body.Kind())
	require.False(t, body.IsSolid())
	_, err = body.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	_, err = body.Centroid()
	require.ErrorIs(t, err, decad.ErrNotSolid)

	require.Len(t, body.Faces(), 1)
	decadtest.MeasuresArea(t, body, units.SquareMillimeters(400), decadtest.Exactly())
	decadtest.MeasuresBounds(t, body, r3.NewVec(0, 0, 0), r3.NewVec(40, 0, 10), decadtest.Exactly())

	free, err := decad.Edges(decad.Free()).Exactly(4).SelectEdges(body)
	require.NoError(t, err)
	for _, e := range free {
		require.True(t, e.IsFree())
	}

	// The same walk through chain Extrude, in its own document so neither build
	// can observe the other.
	sibling := decad.New()
	ribbon, err := decad.Extrude(t.Context(), sibling, s, ch, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	sweptArea, err := body.Area()
	require.NoError(t, err)
	ribbonArea, err := ribbon.Area()
	require.NoError(t, err)
	require.Equal(t, ribbonArea, sweptArea,
		"a chain sweep and the same walk's chain Extrude denote one ribbon, so their areas agree in value, exactness and bound")

	// D7: a placement re-evaluates the payload's own record under the composed
	// motion, so the ribbon's area is unmoved and its box translates with it.
	motion, err := r3.Translation(r3.NewVec(100, 0, 0))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, motion)
	require.NoError(t, err)
	placedArea, err := placed.Area()
	require.NoError(t, err)
	require.Equal(t, sweptArea, placedArea, "a rigid motion moves no area")
	decadtest.MeasuresBounds(t, placed, r3.NewVec(100, 0, 0), r3.NewVec(140, 0, 10), decadtest.Exactly())
}

// TestSweepChainRibbonReadsValid is docs/surface-design.md §9.1's fourth
// validity leg, restated for a chain-fed sweep: the one-span straight
// reduction builds through the identical chainrecord.Build a plain
// chain Extrude does, so it earns the same construction proof
// payloadProvesSimple's chainPayload arm does, re-read off the
// chainSweepPayload it wraps that build in — never ValidityUndecided, which
// is what a payload without this leg's own arm would read instead.
func TestSweepChainRibbonReadsValid(t *testing.T) {
	t.Parallel()
	s, ch := lineChainSketch(t)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, ch, straightSweepPath(t, 10))
	require.NoError(t, err)

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br, err := rep.ForBody(body)
	require.NoError(t, err)
	require.Equal(t, decad.ValidityValid, br.Validity.Outcome,
		"a chain-fed sweep's one-span straight reduction earns the fourth leg exactly as chain Extrude's own does")
	require.Empty(t, br.Validity.Diagnostics)
}

// TestSweepChainMultiSegmentWallSet is docs/surface-design.md's T191: T131's
// open line/arc/line walk swept along the same one-span path mints one wall
// per recorded segment, the two junction sweep edges are shared rather than
// free, and the body's own area is the three walls' own areas folded through
// boundedAdd.
func TestSweepChainMultiSegmentWallSet(t *testing.T) {
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
	chains := s.Chains()
	require.Len(t, chains, 1, "one connected open run")
	require.Len(t, chains[0].Edges, 3)

	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, chains[0], straightSweepPath(t, 10))
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
			require.Equal(t, decad.Exact, area.Exactness, "a line wall swept an exact height has an exact area")
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

// tiltedLineChainSketch builds the 40 mm line chain of T130 on a plane whose
// positive normal is (0, -1, 1)/sqrt(2) rather than a signed coordinate
// vector. The plane's own V is orthonormalized to two BIT-IDENTICAL
// components, so its U x V normal is exactly of the form (0, -c, c) and the
// path direction (0, -7, 7) is exactly codirectional with it over rationals —
// which is what lets the fixture clear the initial-tangent gate while still
// giving the path a length no float can state exactly.
func tiltedLineChainSketch(tb testing.TB) (*sketch.Sketch, *sketch.Chain) {
	tb.Helper()
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 1))
	require.NoError(tb, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(tb, err)
	s, err := w.CreateSketch(plane)
	require.NoError(tb, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(40, 0)
	s.Fix(a)
	s.Fix(b)
	s.CreateLine(a, b)
	_, err = s.Solve(tb.Context())
	require.NoError(tb, err)
	chains := s.Chains()
	require.Len(tb, chains, 1)
	return s, chains[0]
}

// TestSweepChainTiltedPathAreaBoundEnclosesAnalyticValue is
// docs/surface-design.md's T192: the one term a chain sweep carries that the
// same walk's chain Extrude reading does not is the path's own composed height
// bound (docs/sweep-design.md §15.2). On an axis-aligned frame every term of
// that bound is exactly zero, which is why T190 asserts an Exact reading
// instead; here the path runs 7*sqrt(2) mm up a tilted normal, so the bound is
// genuinely positive and the published area must enclose the analytic value
// within it.
//
// The sweep edge's own length is what isolates that bound. A wall's area is a
// boundedMul of two scalars and so carries the product's own rounding whatever
// the height bound says, while a sweep edge's length IS the height and carries
// nothing else, so it reads Exact at a zero bound the moment the leg is gone.
//
// Shown-to-fail: dropping sweepinput.ValidateStraightSweepPath's composed height bound —
// capcontour.StraightEdgeBound's square-root error against the exact rational squared
// length, plus each dyadicFloatError between the recorded tangent and the held
// sweep vector — turns the two sweep-edge assertions below red. See this PR's
// report for the exact failure text.
func TestSweepChainTiltedPathAreaBoundEnclosesAnalyticValue(t *testing.T) {
	t.Parallel()
	s, ch := tiltedLineChainSketch(t)

	path, err := decad.NewPath(r3.NewVec(0, 0, 0), decad.LineTo{End: r3.NewVec(0, -7, 7)})
	require.NoError(t, err)

	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, ch, path)
	require.NoError(t, err)
	require.Len(t, body.Faces(), 1)

	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness, "the path's own length is irrational")
	require.Greater(t, area.Bound.Base(), 0.0)

	analyticHeight := 7 * math.Sqrt2
	require.LessOrEqual(t, math.Abs(area.Value.Base()-40*analyticHeight), area.Bound.Base(),
		"the published area must enclose the analytic value within its own proven bound")

	edges, err := decad.Edges(decad.Free()).Exactly(4).SelectEdges(body)
	require.NoError(t, err)
	sweepEdges := 0
	for _, e := range edges {
		length, err := e.Length()
		require.NoError(t, err)
		if math.Abs(length.Value.Base()-40) < 1 {
			continue // a rim edge: the walk's own 40 mm line at one sweep level
		}
		sweepEdges++
		require.Equal(t, decad.Approximate, length.Exactness,
			"a sweep edge's length is the path's own irrational length")
		require.Greater(t, length.Bound.Base(), 0.0)
		require.LessOrEqual(t, math.Abs(length.Value.Base()-analyticHeight), length.Bound.Base(),
			"a sweep edge's length must enclose the path's analytic length within its own proven bound")
	}
	require.Equal(t, 2, sweepEdges, "the walk's two free ends each carry one sweep edge")
}

// TestSweepChainRefusesAnOffNormalPath is docs/surface-design.md's T193: a
// path that does not start in the sketch plane, and one whose initial tangent
// does not follow that plane's positive normal, are each ErrDegenerate
// (docs/sweep-design.md Table SC row SC3). Each message is read, not only its
// sentinel, so the two refusals are known to fire for their own reasons.
func TestSweepChainRefusesAnOffNormalPath(t *testing.T) {
	t.Parallel()
	s, ch := lineChainSketch(t)
	doc := decad.New()

	offPlane, err := decad.NewPath(r3.NewVec(0, 0, 3), decad.LineTo{End: r3.NewVec(0, 0, 13)})
	require.NoError(t, err)
	_, err = decad.Sweep(t.Context(), doc, s, ch, offPlane)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "must start in the profile plane")

	reversed, err := decad.NewPath(r3.NewVec(0, 0, 0), decad.LineTo{End: r3.NewVec(0, 0, -10)})
	require.NoError(t, err)
	_, err = decad.Sweep(t.Context(), doc, s, ch, reversed)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "positive normal")

	require.Empty(t, doc.Bodies())
}

// TestSweepChainRefusesCompositeOutsideCardinalClass is docs/surface-design.md's
// T194: a source extending beyond the half-circle radius refuses at SC7.
func TestSweepChainRefusesCompositeOutsideCardinalClass(t *testing.T) {
	t.Parallel()
	s, ch := lineChainSketch(t)
	doc := decad.New()

	composite, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 20)},
		decad.ArcThrough{Through: r3.NewVec(5, 0, 25), End: r3.NewVec(10, 0, 20)},
	)
	require.NoError(t, err)
	_, err = decad.Sweep(t.Context(), doc, s, ch, composite)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "radial source inside its radius")

	require.Empty(t, doc.Bodies())
}

// TestSweepChainSeamGatesPrecedeThePathGates is docs/surface-design.md's T195:
// a stale chain and an invalid one are reported as such even when the path
// handed beside them would refuse at SC3, because the seam gate runs first
// (docs/sweep-design.md Table SC). A seam refusal names a repair the caller
// makes in the sketch, and a path refusal reported first would hide it.
func TestSweepChainSeamGatesPrecedeThePathGates(t *testing.T) {
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

	// A path that would itself refuse at SC3, so the assertion below can only
	// pass if the seam gate ran first.
	offPlane, err := decad.NewPath(r3.NewVec(0, 0, 3), decad.LineTo{End: r3.NewVec(0, 0, 13)})
	require.NoError(t, err)

	doc := decad.New()
	_, err = decad.Sweep(t.Context(), doc, s, ch, offPlane)
	require.ErrorIs(t, err, decad.ErrStaleProfile)
	require.Empty(t, doc.Bodies())
}

// TestSweepChainRefusesProfileOnlyOptions covers the options a chain section
// cannot honour: a chain sweep always returns a sheet, so WithSurfaceResult has
// no solid to toggle, and twist, mitred joins and section scale build solids
// from a profile only. Each refuses with ErrUnsupported and leaves the
// document unchanged.
func TestSweepChainRefusesProfileOnlyOptions(t *testing.T) {
	t.Parallel()
	s, ch := lineChainSketch(t)

	tests := []struct {
		name string
		opt  decad.SweepOption
	}{
		{"surface result", decad.WithSurfaceResult()},
		{"twist", decad.WithSweepTwist(units.Degrees(30))},
		{"zero twist", decad.WithSweepTwist(units.Degrees(0))},
		{"mitred joins", decad.WithMitredJoins()},
		{"section scale", decad.WithSectionScale(scalars(1)...)},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			_, err := decad.Sweep(t.Context(), doc, s, ch, straightSweepPath(t, 7), tc.opt)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Empty(t, doc.Bodies())
		})
	}
}

// TestSweepChainOptionRefusalFollowsTheSeam pins the refusal point: a chain
// that is not one of the sketch's own reports the seam's sentinel before the
// option refusal, and the option refusal precedes the path-geometry gates.
func TestSweepChainOptionRefusalFollowsTheSeam(t *testing.T) {
	t.Parallel()
	s, ch := lineChainSketch(t)
	otherSketch, _ := lineChainSketch(t)
	offPlane, err := decad.NewPath(r3.NewVec(0, 0, 3), decad.LineTo{End: r3.NewVec(0, 0, 13)})
	require.NoError(t, err)

	doc := decad.New()
	_, err = decad.Sweep(t.Context(), doc, otherSketch, ch, straightSweepPath(t, 10), decad.WithSurfaceResult())
	require.Error(t, err)
	require.NotErrorIs(t, err, decad.ErrUnsupported, "the seam error wins over the option refusal")

	_, err = decad.Sweep(t.Context(), doc, s, ch, offPlane, decad.WithSurfaceResult())
	require.ErrorIs(t, err, decad.ErrUnsupported, "the option refusal wins over the path-geometry gate")
	require.Empty(t, doc.Bodies())
}

// TestSweepTypedNilSectionIsDegenerate pins that a typed-nil section of either
// kind returns ErrDegenerate and leaves the document unchanged.
func TestSweepTypedNilSectionIsDegenerate(t *testing.T) {
	t.Parallel()
	s, _ := lineChainSketch(t)
	doc := decad.New()

	_, err := decad.Sweep(t.Context(), doc, s, (*sketch.Profile)(nil), straightSweepPath(t, 10))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Sweep(t.Context(), doc, s, (*sketch.Chain)(nil), straightSweepPath(t, 10))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Empty(t, doc.Bodies())
}

// TestSweepChainStagedSurveyNamesPayloadClass pins the class name a chain
// ribbon's staged pull survey prints: the pull survey has no chain sweep arm,
// so the message names the payload class, not the Go type that holds it.
//
// Shown to fail first: with SurveyClass deleted from internal/sweeprecord's
// ChainPayload, the message named ChainPayload.
func TestSweepChainStagedSurveyNamesPayloadClass(t *testing.T) {
	t.Parallel()
	s, ch := lineChainSketch(t)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, ch, straightSweepPath(t, 10))
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	reading, err := report.ForBody(body)
	require.NoError(t, err)
	diag, found := findDiagnostic(reading.Diagnostics, decad.DiagUnsupportedSurveyPayload)
	require.True(t, found)
	require.Equal(t, "chainSweepPayload pull survey support is not implemented; use an analytic body or wait for wider "+
		"chainSweepPayload survey support", diag.Message)
}
