package decad

import (
	"context"
	"math"
	"testing"
	"time"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/prism-boolean-design.md's per-gate (G1-G6, §3.1)
// white-box test suite: each gate is isolated directly against
// admitPrismPair/tryPrismBoolean so a miss is confirmed precisely, without
// depending on whichever refusal the mesh path's own fallback happens to
// produce for a given geometry (prism_boolean_test.go covers that richer,
// public-API shape separately). A synthetic prismPayload is used where no
// live body can carry the shape under test (G1, G4): no evaluator path
// today builds a live prismPayload holding a free-form segment or lets an
// operand answer with a non-prismPayload payload while still resembling
// one, so the gate is exercised against a value built directly.

// canonicalPrismFrame is the plane-local frame every synthetic payload below
// starts from: literal-zero U/V/origin, so its own N() is exactly (0,0,1)
// with no float rounding of its own to confuse a gate result with.
func canonicalPrismFrame(t *testing.T) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return f
}

// synthLineLoop is placeholder single-segment geometry for a gate test that
// never reaches resolution (admitPrismPair reads only segment KIND, never
// shape) — every caller uses the same coordinates.
func synthLineLoop() LoopRecord {
	return LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 10, V: 0}, TStart: 0, TEnd: 1},
	}}
}

// synthRectLoop is a proper closed, CCW rectangular loop — the shape G5/G6's
// full tryPrismBoolean tests need, since (unlike admitPrismPair's own gates)
// resolution actually arranges the operands' geometry through sketch.
func synthRectLoop(u0, v0, u1, v1 float64) LoopRecord {
	return LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: u0, V: v0}, End: Point2{U: u1, V: v0}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: u1, V: v0}, End: Point2{U: u1, V: v1}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: u1, V: v1}, End: Point2{U: u0, V: v1}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: u0, V: v1}, End: Point2{U: u0, V: v0}, TStart: 0, TEnd: 1},
	}}
}

func synthDenseRectLoop(segmentsPerSide int) LoopRecord {
	point := func(i int) Point2 {
		switch {
		case i <= segmentsPerSide:
			return Point2{U: float64(i)}
		case i <= 2*segmentsPerSide:
			return Point2{U: float64(segmentsPerSide), V: float64(i - segmentsPerSide)}
		case i <= 3*segmentsPerSide:
			return Point2{U: float64(3*segmentsPerSide - i), V: float64(segmentsPerSide)}
		default:
			return Point2{V: float64(4*segmentsPerSide - i)}
		}
	}
	count := 4 * segmentsPerSide
	segs := make([]CurveSegment, count)
	for i := range count {
		segs[i] = LineSeg{Start: point(i), End: point(i + 1), TStart: 0, TEnd: 1}
	}
	return LoopRecord{Segments: segs}
}

func TestPrismBooleanGateG1RequiresBothOperandsPrismPayload(t *testing.T) {
	frame := canonicalPrismFrame(t)
	pp := prismPayload{
		profile: ProfileRecord{Outer: synthLineLoop()},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	a := &Body{payload: pp}
	b := &Body{payload: facetedPayload{}} // any non-prismPayload featurePayload

	_, _, ok := admitPrismPair(a, b)
	require.False(t, ok, "G1: a non-prismPayload operand must never admit")
}

func TestPrismBooleanGateG2RejectsAReflectedOperand(t *testing.T) {
	frame := canonicalPrismFrame(t)
	pp := prismPayload{
		profile: ProfileRecord{Outer: synthLineLoop()},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	a := &Body{payload: pp}

	mirror, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	refl, err := r3.Reflection(mirror)
	require.NoError(t, err)
	require.True(t, refl.IsReflection())
	reflectedPP := pp
	reflectedPP.xform = refl
	b := &Body{payload: reflectedPP}

	_, _, ok := admitPrismPair(a, b)
	require.False(t, ok, "G2: a reflected operand must never admit")

	// The identical pair without the reflection clears G1-G4 (isolates G2).
	c := &Body{payload: pp}
	_, _, ok = admitPrismPair(a, c)
	require.True(t, ok)
}

func TestPrismBooleanGateG3RequiresCoDirectionalCoplanarPlanes(t *testing.T) {
	frame := canonicalPrismFrame(t)
	pp := prismPayload{
		profile: ProfileRecord{Outer: synthLineLoop()},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	a := &Body{payload: pp}

	t.Run("antiparallel normal (co-planar but not co-directional)", func(t *testing.T) {
		antiFrame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, -1, 0))
		require.NoError(t, err)
		require.Equal(t, r3.NewVec(0, 0, -1), antiFrame.N())
		antiPP := pp
		antiPP.frame = antiFrame
		b := &Body{payload: antiPP}
		_, _, ok := admitPrismPair(a, b)
		require.False(t, ok)
	})

	t.Run("co-directional but not coplanar", func(t *testing.T) {
		shifted, err := r3.Translation(r3.Vec{Z: 3})
		require.NoError(t, err)
		shiftedPP := pp
		shiftedPP.xform = shifted
		b := &Body{payload: shiftedPP}
		_, _, ok := admitPrismPair(a, b)
		require.False(t, ok)
	})

	t.Run("exactly co-directional and coplanar clears G3", func(t *testing.T) {
		b := &Body{payload: pp}
		_, _, ok := admitPrismPair(a, b)
		require.True(t, ok)
	})
}

func TestPrismBooleanGateG4RefusesNonAnalyticSegment(t *testing.T) {
	frame := canonicalPrismFrame(t)
	linePP := prismPayload{
		profile: ProfileRecord{Outer: synthLineLoop()},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	ellipsePP := linePP
	ellipsePP.profile = ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		EllipseSeg{Center: Point2{U: 20, V: 0}, Rx: units.Millimeters(4), Ry: units.Millimeters(3), CCW: true, TStart: 0, TEnd: 1},
	}}}

	a := &Body{payload: linePP}
	b := &Body{payload: ellipsePP}
	_, _, ok := admitPrismPair(a, b)
	require.False(t, ok, "G4: a non-analytic (ellipse) segment must never admit")

	// A line/circle/arc-only pair still clears G1-G4 (isolates G4).
	otherLinePP := linePP
	otherLinePP.profile = ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		CircleSeg{Center: Point2{U: 5, V: 5}, Radius: units.Millimeters(3), CCW: true, TStart: 0, TEnd: 1},
	}}}
	c := &Body{payload: otherLinePP}
	_, _, ok = admitPrismPair(a, c)
	require.True(t, ok)
}

func TestPrismBooleanGateG5RequiresMatchingZInterval(t *testing.T) {
	frame := canonicalPrismFrame(t)
	pa := prismPayload{
		profile: ProfileRecord{Outer: synthLineLoop()},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}

	t.Run("unequal heights from the same plane", func(t *testing.T) {
		pb := pa
		pb.z1 = 15
		require.False(t, prismUnionZIntervalMatches(pa, pb))
	})

	t.Run("matching interval, re-expressed through a placed operand", func(t *testing.T) {
		// B starts its own z0/z1 at [0, 10] in its own frame, then is placed
		// 3mm along the shared normal: G5 must read the SHIFTED interval
		// [3, 13], not B's own unshifted one.
		shifted, err := r3.Translation(r3.Vec{Z: 3})
		require.NoError(t, err)
		pb := pa
		pb.xform = shifted
		require.False(t, prismUnionZIntervalMatches(pa, pb), "A's [0,10] must not match B's shifted [3,13]")

		paShiftedToMatch := pa
		paShiftedToMatch.z0, paShiftedToMatch.z1 = 3, 13
		require.True(t, prismUnionZIntervalMatches(paShiftedToMatch, pb))
	})
}

func TestPrismBooleanGateG6RestrictsUnionToHoleFreeOperands(t *testing.T) {
	frame := canonicalPrismFrame(t)
	holeFree := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 10, 10)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	overlapping := holeFree
	overlapping.profile = ProfileRecord{Outer: synthRectLoop(5, 5, 15, 15)}
	holed := holeFree
	holed.profile = ProfileRecord{
		Outer: synthRectLoop(5, 5, 15, 15),
		Holes: []LoopRecord{synthRectLoop(8, 8, 9, 9)},
	}

	a := &Body{payload: holeFree}
	b := &Body{payload: holed}
	_, ok, err := tryPrismBoolean(t.Context(), OpUnion, a, b)
	require.NoError(t, err)
	require.False(t, ok, "G6: a holed operand must never admit a Union")

	c := &Body{payload: overlapping}
	_, ok, err = tryPrismBoolean(t.Context(), OpUnion, a, c)
	require.NoError(t, err)
	require.True(t, ok, "a hole-free pair otherwise identical clears G6")
}

// TestPrismUnionReexpressedSplitFallsBack keeps a nonidentity coordinate
// re-expression from publishing a section bound for a shallow sketch-cut edge.
// A transverse cut can magnify the coordinate error by the crossing angle, so
// the current analytic path must route this pair through the mesh evaluator.
func TestPrismUnionReexpressedSplitFallsBack(t *testing.T) {
	frame := canonicalPrismFrame(t)
	pa := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 10, 10)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	const theta = 0.01
	basis := r3.Basis{
		EX: r3.Vec{X: math.Cos(theta), Y: math.Sin(theta), Z: 0},
		EY: r3.Vec{X: -math.Sin(theta), Y: math.Cos(theta), Z: 0},
		EZ: r3.Vec{X: 0, Y: 0, Z: 1},
	}
	rotation, err := r3.FromBasis(basis, r3.Vec{})
	require.NoError(t, err)
	pb := pa
	// The lower long edge crosses A's upper edge at theta, so this fixture
	// reaches the trim-amplification path without relying on a degenerate
	// contact classification.
	pb.profile = ProfileRecord{Outer: synthRectLoop(-5, 9.9, 15, 11.9)}
	pb.xform = rotation
	_, _, admitted := admitPrismPair(&Body{payload: pa}, &Body{payload: pb})
	require.True(t, admitted, "the fixture must clear G1-G4 before the split guard runs")
	require.True(t, prismUnionZIntervalMatches(pa, pb), "the fixture must clear G5")

	reexpression, err := newPrismReexpression(pa, pb)
	require.NoError(t, err)
	require.False(t, reexpression.identity)
	scene, _, err := buildPrismScene(newWorkBudget(t.Context()), pa, pb, reexpression)
	require.NoError(t, err)
	profiles, err := prismProfilesContext(t.Context(), scene.Profiles)
	require.NoError(t, err)

	split := false
	for _, profile := range profiles {
		require.True(t, profile.Valid, "the shallow crossing must not depend on an invalid arrangement")
		for _, loop := range append([][]sketch.BoundaryEdge{profile.Outer}, profile.Holes...) {
			for _, edge := range loop {
				split = split || edge.Partial
			}
		}
	}
	require.True(t, split, "the overlapping rectangles must produce a split boundary")

	_, ok, err := tryPrismBoolean(t.Context(), OpUnion, &Body{payload: pa}, &Body{payload: pb})
	require.NoError(t, err)
	require.False(t, ok, "a re-expressed arrangement with a split boundary must fall back")
}

// TestPrismUnionDisplacedSourceSplitFallsBack covers a chained union whose
// second re-expression is identity. The first union carries its own section
// displacement from re-expressing a containing operand. A shallow crossing in
// the second union must still fall back: moving the prior section can move the
// new trim by that displacement divided by the crossing sine.
func TestPrismUnionDisplacedSourceSplitFallsBack(t *testing.T) {
	frame := canonicalPrismFrame(t)
	inner := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(2, 2, 8, 8)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	const shift = 1e8
	translation, err := r3.Translation(r3.NewVec(shift, 0, 0))
	require.NoError(t, err)
	containing := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(-shift, 0, 10-shift, 10)},
		frame:   frame, z0: 0, z1: 10, xform: translation,
	}
	first, ok, err := tryPrismBoolean(t.Context(), OpUnion, &Body{payload: inner}, &Body{payload: containing})
	require.NoError(t, err)
	require.True(t, ok, "the containing first union must resolve analytically")
	require.Positive(t, first.sectionDelta, "the nonidentity first union must carry its re-expression displacement")

	shallow := prismPayload{
		profile: ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
			LineSeg{Start: Point2{U: -5, V: 9.9}, End: Point2{U: 15, V: 10.1}, TStart: 0, TEnd: 1},
			LineSeg{Start: Point2{U: 15, V: 10.1}, End: Point2{U: 15, V: 12.1}, TStart: 0, TEnd: 1},
			LineSeg{Start: Point2{U: 15, V: 12.1}, End: Point2{U: -5, V: 11.9}, TStart: 0, TEnd: 1},
			LineSeg{Start: Point2{U: -5, V: 11.9}, End: Point2{U: -5, V: 9.9}, TStart: 0, TEnd: 1},
		}}},
		frame: first.frame, z0: first.z0, z1: first.z1, xform: first.xform,
	}
	reexpression, err := newPrismReexpression(first, shallow)
	require.NoError(t, err)
	require.True(t, reexpression.identity, "the second union must take the identity re-expression path")

	scene, _, err := buildPrismScene(newWorkBudget(t.Context()), first, shallow, reexpression)
	require.NoError(t, err)
	profiles, err := prismProfilesContext(t.Context(), scene.Profiles)
	require.NoError(t, err)
	split, err := prismProfilesHaveSplitBoundary(newWorkBudget(t.Context()), profiles)
	require.NoError(t, err)
	require.True(t, split, "the shallow crossing must create a trimmed edge")

	_, ok, err = tryPrismBoolean(t.Context(), OpUnion, &Body{payload: first}, &Body{payload: shallow})
	require.NoError(t, err)
	require.False(t, ok, "a split boundary with an uncertain source must fall back before recordEdge")
}

// TestTryPrismBooleanSingleOpenSegmentIsUnresolvedForCutAndIntersect covers
// Cut and Intersect against a pair whose G1-G5 all pass but whose "loop" is a
// single open LineSeg (synthLineLoop's own shape — a placeholder G1-G4 never
// looks past the segment kind for): the private scene bounds no closed region
// at all, so §4.2's clean-nesting search finds no candidate profile and both
// ops fall through unresolved (§4.4), not admitted, exactly like any other
// topology this increment's resolution does not cover.
func TestTryPrismBooleanSingleOpenSegmentIsUnresolvedForCutAndIntersect(t *testing.T) {
	frame := canonicalPrismFrame(t)
	pp := prismPayload{
		profile: ProfileRecord{Outer: synthLineLoop()},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	a := &Body{payload: pp}
	b := &Body{payload: pp}

	for _, op := range []OpKind{OpCut, OpIntersect} {
		_, ok, err := tryPrismBoolean(t.Context(), op, a, b)
		require.NoError(t, err)
		require.False(t, ok)
	}
}

func TestPrismUnionArrangementCapRejectsLargeLineOnlyScene(t *testing.T) {
	frame := canonicalPrismFrame(t)
	pp := prismPayload{
		profile: ProfileRecord{Outer: synthDenseRectLoop(prismMaxArrangementSegments/8 + 1)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	a := &Body{payload: pp}
	b := &Body{payload: pp}

	_, ok, err := tryPrismBoolean(t.Context(), OpUnion, a, b)
	require.ErrorIs(t, err, ErrUnsupported)
	require.False(t, ok)
}

func TestPrismUnionPreservesEndDisplacements(t *testing.T) {
	frame := canonicalPrismFrame(t)
	pa := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 10, 10)},
		frame:   frame,
		z0:      0,
		z1:      10,
		z0Delta: 0.125,
		z1Delta: 0.75,
		xform:   r3.Identity(),
	}
	pb := pa
	pb.profile = ProfileRecord{Outer: synthRectLoop(5, 5, 15, 15)}
	pb.z0Delta = 0.5
	pb.z1Delta = 0.25

	result, ok, err := tryPrismBoolean(t.Context(), OpUnion, &Body{payload: pa}, &Body{payload: pb})
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, 0.5, result.z0Delta)
	require.Equal(t, 0.75, result.z1Delta)
}

func TestPrismProfilesContextWaitsForArrangementAfterCancellation(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	started := make(chan struct{})
	release := make(chan struct{})
	finished := make(chan struct{})
	profiles := func() []*sketch.Profile {
		close(started)
		<-release
		close(finished)
		return []*sketch.Profile{}
	}
	result := make(chan error, 1)
	go func() {
		_, err := prismProfilesContext(ctx, profiles)
		result <- err
	}()

	<-started
	cancel()
	select {
	case err := <-result:
		t.Fatalf("prismProfilesContext returned before arrangement finished: %v", err)
	case <-time.After(100 * time.Millisecond):
	}

	close(release)
	<-finished
	require.ErrorIs(t, <-result, context.Canceled)
}
