package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/prism-boolean-design.md PR1's per-gate (G1-G6, §3.1)
// white-box test suite: each gate is isolated directly against
// admitPrismPair/tryPrismUnion so a miss is confirmed precisely, without
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
// full tryPrismUnion tests need, since (unlike admitPrismPair's own gates)
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
	_, ok, err := tryPrismUnion(t.Context(), OpUnion, a, b)
	require.NoError(t, err)
	require.False(t, ok, "G6: a holed operand must never admit a Union")

	c := &Body{payload: overlapping}
	_, ok, err = tryPrismUnion(t.Context(), OpUnion, a, c)
	require.NoError(t, err)
	require.True(t, ok, "a hole-free pair otherwise identical clears G6")
}

// TestTryPrismUnionOnlyImplementsUnion confirms §4.2's stated PR1 scope: Cut
// and Intersect fall through unconditionally even where G1-G5 would pass,
// since PR1 implements region resolution for Union's select-all path only
// (Cut/Intersect's clean-nesting match is PR2).
func TestTryPrismUnionOnlyImplementsUnion(t *testing.T) {
	frame := canonicalPrismFrame(t)
	pp := prismPayload{
		profile: ProfileRecord{Outer: synthLineLoop()},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	a := &Body{payload: pp}
	b := &Body{payload: pp}

	for _, op := range []OpKind{OpCut, OpIntersect} {
		_, ok, err := tryPrismUnion(t.Context(), op, a, b)
		require.NoError(t, err)
		require.False(t, ok)
	}
}

func TestPrismUnionArrangementCapRejectsLargeLineOnlyScene(t *testing.T) {
	frame := canonicalPrismFrame(t)
	pp := prismPayload{
		profile: ProfileRecord{Outer: synthDenseRectLoop(prismUnionMaxArrangementSegments/8 + 1)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	a := &Body{payload: pp}
	b := &Body{payload: pp}

	_, ok, err := tryPrismUnion(t.Context(), OpUnion, a, b)
	require.ErrorIs(t, err, ErrUnsupported)
	require.False(t, ok)
}

func TestPrismProfilesContextReturnsCancellationDuringArrangement(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	started := make(chan struct{})
	release := make(chan struct{})
	finished := make(chan struct{})
	profiles := func() []*sketch.Profile {
		close(started)
		<-release
		close(finished)
		return nil
	}
	go func() {
		<-started
		cancel()
	}()

	_, err := prismProfilesContext(ctx, profiles)
	require.ErrorIs(t, err, context.Canceled)

	close(release)
	<-finished
}
