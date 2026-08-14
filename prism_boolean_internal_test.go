package decad

import (
	"context"
	"math"
	"math/big"
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
// public-API shape separately). A synthetic prismPayload is used where a gate
// is easiest isolated from one built directly (G1, G4): a live prismPayload
// can hold a free-form segment via Extrude since §10 P4b, but G4's own
// analytic-profile gate still refuses it (this file's own G4 tests cover
// that), and no evaluator path lets an operand answer with a
// non-prismPayload payload while still resembling one, so both gates are
// exercised against a value built directly.

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
	scene, _, _, err := buildPrismScene(newWorkBudget(t.Context()), pa, pb, reexpression)
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

	scene, _, _, err := buildPrismScene(newWorkBudget(t.Context()), first, shallow, reexpression)
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

// --- fu141 (docs/prism-boolean-design.md §9's RB9): the merged union loop's
// own recorded coordinates must join at every junction.

// requireMergedLoopSegmentsJoin is T1's own invariant, reused by T3: whenever
// a merge resolves (tryPrismBoolean's ok == true), every consecutive pair of
// WHOLE LineSeg segments in the merged Outer loop joins bit-exactly — the
// same property falsifyLoopJoins now proves before resolvePrismUnion returns.
// It reads the recorded coordinates directly (segs[i].End == segs[i+1].Start,
// wrap included), the same comparison loopJoinPointsAgree makes for a
// same-source pair.
func requireMergedLoopSegmentsJoin(t *testing.T, segs []CurveSegment) {
	t.Helper()
	n := len(segs)
	for i := range n {
		j := (i + 1) % n
		li, oki := segs[i].(LineSeg)
		lj, okj := segs[j].(LineSeg)
		if !oki || !okj || li.TStart != 0 || li.TEnd != 1 || lj.TStart != 0 || lj.TEnd != 1 {
			continue
		}
		require.Equal(t, li.End, lj.Start, "merged segment %d end must bit-exactly equal segment %d start", i, j)
	}
}

// synthGapRectLoop is synthRectLoop with one deliberate defect: seg 0's End
// and seg 1's Start name the SAME corner but differ by gap — a mismatch
// authored directly on the record, never computed by any merge. sketch's own
// arrangement still accepts the shape as one region on its proximity
// threshold (docs/sketch-seam-design.md), so the defect survives all the way
// to resolvePrismUnion's own recorded chain.
func synthGapRectLoop(u0, v0, u1, v1, gap float64) LoopRecord {
	return LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: u0, V: v0}, End: Point2{U: u1, V: v0}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: u1 + gap, V: v0}, End: Point2{U: u1, V: v1}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: u1, V: v1}, End: Point2{U: u0, V: v1}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: u0, V: v1}, End: Point2{U: u0, V: v0}, TStart: 0, TEnd: 1},
	}}
}

// TestPrismUnionMergedLoopJunctionsClose is RB9's own guard, pinned directly
// against resolvePrismUnion's wiring rather than against a live cut-fragment
// fixture.
//
// The one LIVE-reachable way to produce this defect is an operand recorded as
// Partial cut fragments (drawn as overshooting lines sketch trims at both
// ends): buildPrismScene's walkOf interpolates each surviving fragment's
// walked endpoint independently, from its own entity's Start/End and T, so
// two fragments naming the same corner can round to bit-different floats
// before the merge ever sees them. That is fu157's class (#158): welding the
// private scene's own recorded junctions closes exactly that fixture's loop,
// so a test keyed on it would silently start asserting the wrong thing once
// that PR lands (prism_boolean_test.go's
// TestPrismUnionCutFragmentOperandRefusesNonClosingMerge is that fixture, and
// carries the same risk explicitly).
//
// This test instead authors the defect directly on a synthetic operand
// (bypassing RecordProfile's own seam checks, the same way the G1-G6 gate
// tests above do) — a corner sketch's own proximity threshold still accepts
// as one region, but whose recorded coordinates no merge computed and so
// never rounds into agreement. Nothing here depends on how any operand's
// Partial fragments get built, so the refusal half stays correct whichever
// PR lands first.
func TestPrismUnionMergedLoopJunctionsClose(t *testing.T) {
	frame := canonicalPrismFrame(t)
	const gap = 1e-9
	pa := prismPayload{
		profile: ProfileRecord{Outer: synthGapRectLoop(0, 0, 10, 10, gap)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	pb := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(2, 2, 4, 4)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	a := &Body{payload: pa}
	b := &Body{payload: pb}

	_, ok, err := tryPrismBoolean(t.Context(), OpUnion, a, b)
	require.False(t, ok)
	require.ErrorIs(t, err, ErrUnrecordableProfile)
	require.Contains(t, err.Error(), "does not close")
	require.Contains(t, err.Error(), "10.000000001")

	// The invariant half, which must survive fu157's weld regardless of
	// landing order: a pair with no authored defect resolves, and its merged
	// loop DOES join bit-exactly at every whole junction.
	clean := pa
	clean.profile = ProfileRecord{Outer: synthRectLoop(0, 0, 10, 10)}
	res, ok, err := tryPrismBoolean(t.Context(), OpUnion, &Body{payload: clean}, b)
	require.NoError(t, err)
	require.True(t, ok)
	requireMergedLoopSegmentsJoin(t, res.profile.Outer.Segments)
}

// TestPrismUnionCleanOperandsMergedLoopClosesExactly is the control: the
// ordinary class this guard must never fire on. Two operands whose own
// records carry no Partial fragment (drawn as four shared-point lines each,
// the live-reachable whole-edge shape) merge into a loop that closes
// bit-exactly, with no error and no displacement.
func TestPrismUnionCleanOperandsMergedLoopClosesExactly(t *testing.T) {
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 5)
	b := internalBoxBody(t, doc, 5, 5, 15, 15, 5)
	res, ok, err := tryPrismBoolean(t.Context(), OpUnion, a, b)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, res.profile.Outer.Segments, 8)
	requireMergedLoopSegmentsJoin(t, res.profile.Outer.Segments)

	doc2 := New()
	c := internalBoxBody(t, doc2, 0, 0, 10, 10, 5)
	d := internalBoxBody(t, doc2, 2, 2, 4, 4, 5)
	res2, ok, err := tryPrismBoolean(t.Context(), OpUnion, c, d)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, res2.profile.Outer.Segments, 4)
	requireMergedLoopSegmentsJoin(t, res2.profile.Outer.Segments)
	require.Zero(t, res2.sectionDelta)
}

// This is task fu143's own test suite: §7's fourth displacement source,
// δ_walk — a consumed source segment's own walked endpoint, computed rather
// than read off the record whenever that segment's recorded range narrows
// its entity's own natural domain.

// prismSplitLeftCellBody builds the rectangle [1,11]×[0,10] split by a fixed
// line through (5,-2)-(5,14) and extrudes the LEFT cell h mm — task fu143's
// own fixture (its investigation section 1). The left cell's bottom and top
// walls are Partial fragments of the rectangle's own bottom/top lines,
// recorded with the entity's full [1,11] Start/End and a narrowed
// TStart/TEnd denoting the split at u≈5, so the corner where they meet the
// right wall is a coordinate this evaluator's own scene construction
// computes, not one either wall's record states outright.
//
// Every one of those corners must come out the SAME coordinate on every host,
// or the loop buildPrismScene hands back no longer closes and RecordProfile
// refuses it before the charge under test is ever reached. Each corner is
// reached twice by separate arithmetic — once along each of the two walls that
// meet there, through lerp2's start + t·(end − start) — and a host that
// contracts that expression into a fused multiply-add rounds it once where a
// host without the fusion rounds it twice. So this fixture states the split
// line's own endpoints as (5,-2)-(5,14): its span is 16 and the rectangle's
// walls cut it at v = 0 and v = 10, making the recorded parameters 1/8 and
// 3/4 exactly. Every product and sum lerp2 then forms is representable, so
// both spellings return the identical corner and neither rounds at all. The
// bottom and top walls' own parameters, 0.4 and 0.6, are not exact binary
// fractions, but the value 1 + 0.4·10 sits a quarter of an ulp above 5 under
// either spelling, well inside the half ulp that rounds it back to 5.
//
// prismFixtureHeight is every fixture below's own sweep height: every test in
// this suite compares two operands and needs no other value.
const prismFixtureHeight = 10.0

func prismSplitLeftCellBody(t *testing.T, doc *Document) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(1, 0, 11, 10)
	s.Fix(r.A)
	lo := s.CreatePoint(5, -2)
	hi := s.CreatePoint(5, 14)
	s.Fix(lo)
	s.Fix(hi)
	s.CreateLine(lo, hi)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var left *sketch.Profile
	for _, p := range s.Profiles() {
		minU, maxU := math.Inf(1), math.Inf(-1)
		for _, e := range p.Outer {
			for _, pt := range e.Polyline {
				minU = math.Min(minU, pt[0])
				maxU = math.Max(maxU, pt[0])
			}
		}
		if minU == 1 && maxU <= 5.0000001 {
			left = p
		}
	}
	require.NotNil(t, left, "the split rectangle's left cell must exist")

	body, err := doc.Extrude(s, left, Distance{D: units.Millimeters(prismFixtureHeight), Dir: Along})
	require.NoError(t, err)
	prismRequireSplitWallRange(t, body.payload.(prismPayload).profile)
	return body
}

// prismRequireSplitWallRange pins the recorded range of the split-left-cell
// fixture's own right wall to the two exact binary fractions its doc comment
// derives every corner's host independence from. A host whose sketch reports
// any other parameter must fail here, naming the fixture, rather than at
// whichever consumer first walks that parameter to a corner its neighbour
// does not share.
func prismRequireSplitWallRange(t *testing.T, p ProfileRecord) {
	t.Helper()
	for _, seg := range p.Outer.Segments {
		ls, ok := seg.(LineSeg)
		if !ok || ls.Start != (Point2{U: 5, V: -2}) || ls.End != (Point2{U: 5, V: 14}) {
			continue
		}
		require.Equal(t, 0.125, ls.TStart)
		require.Equal(t, 0.75, ls.TEnd)
		return
	}
	t.Fatal("the split-left-cell fixture's own right wall was not found")
}

// prismRectBody extrudes the axis-aligned rectangle (x0, y0)-(x1, y1)
// prismFixtureHeight mm — every segment WHOLE, TStart=0, TEnd=1, the
// every-caller-drawn-body shape.
func prismRectBody(t *testing.T, doc *Document, x0, y0, x1, y1 float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(prismFixtureHeight), Dir: Along})
	require.NoError(t, err)
	return body
}

// prismRatOf lifts a float64 to the exact rational it is; a test fixture
// coordinate is always finite, so a non-finite input is a fixture bug.
func prismRatOf(t *testing.T, f float64) *big.Rat {
	t.Helper()
	r := new(big.Rat)
	require.NotNil(t, r.SetFloat64(f), "fixture coordinate must be finite")
	return r
}

// prismExactLineOnlyArea is the exact signed area of a line-only recorded
// outer loop (no holes), integrated over each segment's own RECORDED range
// via ratLerp — the same Green's-theorem boundary term moments.go
// accumulates, with no rounding at all. Every fixture below is a plain
// rectangle or a footprint difference of rectangles, so the outer loop alone
// is always line-only.
func prismExactLineOnlyArea(t *testing.T, p ProfileRecord) *big.Rat {
	t.Helper()
	total := new(big.Rat)
	for _, seg := range p.Outer.Segments {
		ls, ok := seg.(LineSeg)
		require.True(t, ok, "fixture must be line-only: %T", seg)
		u0 := ratLerp(ls.Start.U, ls.End.U, ls.TStart)
		v0 := ratLerp(ls.Start.V, ls.End.V, ls.TStart)
		u1 := ratLerp(ls.Start.U, ls.End.U, ls.TEnd)
		v1 := ratLerp(ls.Start.V, ls.End.V, ls.TEnd)
		term := new(big.Rat).Sub(new(big.Rat).Mul(u0, v1), new(big.Rat).Mul(u1, v0))
		total.Add(total, new(big.Rat).Mul(term, big.NewRat(1, 2)))
	}
	return total
}

// prismExactResidual is |reported − truth| taken entirely over rationals and
// rounded UP into a float64 — differencing against a float64 conversion of
// truth would fold half an ulp of the reported magnitude into the answer,
// which could flatter a bound that failed to contain the true error
// (prism_boolean_displacement_test.go's exactResidual documents the same
// point for the external test suite; this is its internal-package twin).
func prismExactResidual(t *testing.T, reported float64, truth *big.Rat) float64 {
	t.Helper()
	d := new(big.Rat).Sub(prismRatOf(t, reported), truth)
	d.Abs(d)
	f, exact := d.Float64()
	if !exact {
		f = math.Nextafter(f, math.Inf(1))
	}
	return f
}

// prismBottomWallTEnd locates the split-left-cell fixture's own bottom wall
// (the full rectangle's [1,11] bottom line, Start=(1,0), End=(11,0)) and
// returns its recorded TEnd — the fraction §7's δ_walk charges a walked
// endpoint against.
func prismBottomWallTEnd(t *testing.T, p ProfileRecord) float64 {
	t.Helper()
	for _, seg := range p.Outer.Segments {
		ls, ok := seg.(LineSeg)
		if !ok {
			continue
		}
		if ls.Start == (Point2{U: 1, V: 0}) && ls.End == (Point2{U: 11, V: 0}) {
			return ls.TEnd
		}
	}
	t.Fatal("the split-left-cell fixture's own bottom wall was not found")
	return 0
}

// TestPrismSplitLeftCellFixtureWalksHostIndependently proves on THIS host the
// property the fixture's own doc comment derives, and which only another host
// could otherwise disprove: every corner the fixture's walls walk to is the
// same coordinate whether or not the host fuses lerp2's multiply and add.
// Each segment's endpoints are computed both ways — prismLerpSplit and
// prismLerpFused, this file's owner of lerp2's two readings — and compared
// exactly, so a future edit that reintroduces a parameter needing a rounding
// decision fails here rather than on whichever host makes that decision
// differently.
func TestPrismSplitLeftCellFixtureWalksHostIndependently(t *testing.T) {
	doc := New()
	p := prismSplitLeftCellBody(t, doc).payload.(prismPayload).profile
	for i, seg := range p.Outer.Segments {
		ls, ok := seg.(LineSeg)
		require.Truef(t, ok, "the fixture is line-only: segment %d is a %T", i, seg)
		for _, at := range []float64{ls.TStart, ls.TEnd} {
			require.Equalf(t, prismLerpSplit(ls.Start, ls.End, at), prismLerpFused(ls.Start, ls.End, at),
				"segment %d walks to a different endpoint at t=%v when the host fuses the multiply-add", i, at)
		}
	}
}

// TestPrismUnionTrimmedSourceSegmentChargesItsWalkedEndpoint is fu143's own
// Union reproduction: operand A carries a Partial bottom-wall fragment of a
// wider rectangle line, operand B sits strictly inside A's own footprint, and
// the union must charge A's own walk displacement into its published
// sectionDelta and volume bound rather than publish Exact/zero over a section
// this union's own scene construction moved.
func TestPrismUnionTrimmedSourceSegmentChargesItsWalkedEndpoint(t *testing.T) {
	const h = 10.0
	doc := New()
	a := prismSplitLeftCellBody(t, doc)
	pa := a.payload.(prismPayload)

	// Pin the fixture: if sketch's own cut parameter for this split ever
	// changes, this fixture must fail loudly rather than silently stop
	// testing anything.
	require.Equal(t, 0.4000000000000000222, prismBottomWallTEnd(t, pa.profile))

	b := prismRectBody(t, doc, 2, 2, 4, 8)

	u, err := Union(a, b)
	require.NoError(t, err)
	pu, ok := u.payload.(prismPayload)
	require.True(t, ok, "the analytic reduction must own this pair")

	// The exact distance between the recorded corner (the right wall's own
	// u = 5, exactly) and the denoted corner the bottom wall's own TEnd
	// names (1 + TEnd·10), computed over math/big.Rat rather than typed as a
	// literal.
	tEnd := prismBottomWallTEnd(t, pa.profile)
	denotedCorner := new(big.Rat).Add(big.NewRat(1, 1), new(big.Rat).Mul(prismRatOf(t, tEnd), big.NewRat(10, 1)))
	minDelta := prismExactResidual(t, 5, denotedCorner)
	require.Positive(t, minDelta, "the fixture must actually disagree with itself, or it proves nothing")
	require.GreaterOrEqual(t, pu.sectionDelta, minDelta,
		"the union's own sectionDelta must charge at least the corner disagreement its own operand carries")

	// B sits strictly inside A's own footprint, so the union the two records
	// DENOTE is A's own recorded section swept h mm.
	truth := new(big.Rat).Mul(prismExactLineOnlyArea(t, pa.profile), prismRatOf(t, h))
	uv, err := u.Volume()
	require.NoError(t, err)
	residual := prismExactResidual(t, uv.Value.Base(), truth)
	require.Equal(t, Approximate, uv.Exactness)
	require.LessOrEqualf(t, residual, uv.Bound.Base(),
		"the published volume bound %g must contain the true error %g", uv.Bound.Base(), residual)
}

// TestPrismUnionChargesEachWalkExactlyOnce pins §7's composition
// δ = up(max(up(δ_A + δ_walkA), up(δ_B + δ_walkB + δ_reexpress)) + δ_cut) on
// the fixture where every term but one is zero: operand A owes a walk charge,
// operand B is drawn whole, the re-expression is the identity, and B sits
// strictly inside A so the merge cuts nothing. The published displacement must
// therefore be A's own walk charge and nothing more — charged ONCE, on A's own
// side of the max. A composition that also added a separate walk term outside
// the max would publish about twice this value and fail here.
func TestPrismUnionChargesEachWalkExactlyOnce(t *testing.T) {
	doc := New()
	a := prismSplitLeftCellBody(t, doc)
	pa := a.payload.(prismPayload)
	require.Zero(t, pa.sectionDelta, "δ_A must be zero, or the fixture cannot isolate the walk charge")

	b := prismRectBody(t, doc, 2, 2, 4, 8) // strictly inside A's own footprint
	pb := b.payload.(prismPayload)
	require.Zero(t, pb.sectionDelta, "δ_B must be zero")

	reexpression, err := newPrismReexpression(pa, pb)
	require.NoError(t, err)
	require.True(t, reexpression.identity, "both operands share one frame with no placement between them")
	require.Zero(t, reexpression.delta, "δ_reexpress must be zero")

	_, _, sceneDelta, err := buildPrismScene(newWorkBudget(t.Context()), pa, pb, reexpression)
	require.NoError(t, err)
	require.Positive(t, sceneDelta.a, "operand A's own trimmed walls must carry a walk charge")
	require.Zero(t, sceneDelta.b, "operand B is drawn whole, so δ_walkB is zero")

	u, err := Union(a, b)
	require.NoError(t, err)
	pu, ok := u.payload.(prismPayload)
	require.True(t, ok, "the analytic reduction must own this pair")
	// §7's formula, term by term, with δ_cut = 0 because the merge cuts
	// nothing: every walk charge sits inside its own operand's fold.
	const cutDelta = 0.0
	want := absSumUpper(
		max(
			absSumUpper(pa.sectionDelta, sceneDelta.a),
			absSumUpper(pb.sectionDelta, sceneDelta.b, reexpression.delta),
		),
		cutDelta,
	)
	require.Equal(t, want, pu.sectionDelta,
		"with every other term zero the published displacement is A's own walk charge, folded in once")
	require.Less(t, pu.sectionDelta, absSumUpper(want, sceneDelta.a),
		"a second, separate walk charge outside the max would roughly double the published displacement")
}

// TestPrismCutTrimmedTargetChargesItsWalkedEndpoint is fu143's Cut
// reproduction: the clean-nesting path's own target carries the same Partial
// bottom-wall fragment, and the tool is fully nested inside it.
func TestPrismCutTrimmedTargetChargesItsWalkedEndpoint(t *testing.T) {
	const h = 10.0
	doc := New()
	target := prismSplitLeftCellBody(t, doc)
	ptarget := target.payload.(prismPayload)
	tool := prismRectBody(t, doc, 2, 2, 4, 8)

	got, err := Cut(target, tool)
	require.NoError(t, err)
	pg, ok := got.payload.(prismPayload)
	require.True(t, ok, "the clean-nesting cut must build analytically")
	require.Positive(t, pg.sectionDelta, "the target's own walk charge must reach the cut's result")

	// Truth: the target's own denoted section, minus the tool's own 2×6
	// footprint (fully inside it), swept h mm.
	truthArea := new(big.Rat).Sub(prismExactLineOnlyArea(t, ptarget.profile), big.NewRat(12, 1))
	truth := new(big.Rat).Mul(truthArea, prismRatOf(t, h))
	gv, err := got.Volume()
	require.NoError(t, err)
	residual := prismExactResidual(t, gv.Value.Base(), truth)
	require.Equal(t, Approximate, gv.Exactness)
	require.LessOrEqualf(t, residual, gv.Bound.Base(),
		"the published volume bound %g must contain the true error %g", gv.Bound.Base(), residual)
}

// TestPrismIntersectTrimmedOperandChargesItsWalkedEndpoint is fu143's
// Intersect reproduction: a big outer box fully contains the split-left-cell
// fixture, so the nested operand's own walk charge must reach the result.
func TestPrismIntersectTrimmedOperandChargesItsWalkedEndpoint(t *testing.T) {
	const h = 10.0
	doc := New()
	outer := prismRectBody(t, doc, 0, -1, 12, 11)
	a := prismSplitLeftCellBody(t, doc)
	pa := a.payload.(prismPayload)

	got, err := Intersect(outer, a)
	require.NoError(t, err)
	pg, ok := got.payload.(prismPayload)
	require.True(t, ok, "the clean-nesting intersect must build analytically")
	require.Positive(t, pg.sectionDelta, "the nested operand's own walk charge must reach the result")

	truth := new(big.Rat).Mul(prismExactLineOnlyArea(t, pa.profile), prismRatOf(t, h))
	gv, err := got.Volume()
	require.NoError(t, err)
	residual := prismExactResidual(t, gv.Value.Base(), truth)
	require.Equal(t, Approximate, gv.Exactness)
	require.LessOrEqualf(t, residual, gv.Bound.Base(),
		"the published volume bound %g must contain the true error %g", gv.Bound.Base(), residual)
}

// TestPrismBooleanWholeSourceSegmentsChargeNothing is the guard that §7's new
// term does not quietly retire the decidable zero case: two whole-segment
// boxes, the same pair TestPrismUnionWholeEdgeMergeStaysExact already covers
// from the outside, must publish a sectionDelta of exactly 0.0 on all three
// ops.
func TestPrismBooleanWholeSourceSegmentsChargeNothing(t *testing.T) {
	t.Run("Union", func(t *testing.T) {
		doc := New()
		a := prismRectBody(t, doc, 0, 0, 10, 10)
		b := prismRectBody(t, doc, 2, 2, 8, 8)
		u, err := Union(a, b)
		require.NoError(t, err)
		pu, ok := u.payload.(prismPayload)
		require.True(t, ok)
		require.Equal(t, 0.0, pu.sectionDelta)
	})

	t.Run("Cut", func(t *testing.T) {
		doc := New()
		target := prismRectBody(t, doc, 0, 0, 10, 10)
		tool := prismRectBody(t, doc, 2, 2, 8, 8)
		got, err := Cut(target, tool)
		require.NoError(t, err)
		pg, ok := got.payload.(prismPayload)
		require.True(t, ok)
		require.Equal(t, 0.0, pg.sectionDelta)
	})

	t.Run("Intersect", func(t *testing.T) {
		doc := New()
		outer := prismRectBody(t, doc, 0, 0, 10, 10)
		inner := prismRectBody(t, doc, 2, 2, 8, 8)
		got, err := Intersect(outer, inner)
		require.NoError(t, err)
		pg, ok := got.payload.(prismPayload)
		require.True(t, ok)
		require.Equal(t, 0.0, pg.sectionDelta)
	})
}

// TestWalkChargeOf is a table test over walkChargeOf itself: 0 for a whole
// segment of every admitted kind, a positive finite value for a trimmed one,
// and +Inf for a non-finite coordinate — never 0 for an unknown or uncertain
// case (this task's own risk: an absent bound must never read as a small
// one). The trimmed circular rows exercise the function directly; through the
// boolean they are unreachable, which
// TestPrismCircularWalkChargeImpliesRefusal pins.
func TestWalkChargeOf(t *testing.T) {
	line := LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 10, V: 0}}
	arc := ArcSeg{
		Center: Point2{U: 0, V: 0},
		Start:  Point2{U: 5, V: 0},
		End:    Point2{U: 0, V: 5},
	}
	circle := CircleSeg{Center: Point2{U: 0, V: 0}, Radius: units.Millimeters(5), CCW: true}

	for _, tc := range []struct {
		name string
		seg  CurveSegment
	}{
		{"whole LineSeg", func() CurveSegment { s := line; s.TStart, s.TEnd = 0, 1; return s }()},
		{"whole reversed LineSeg", func() CurveSegment { s := line; s.TStart, s.TEnd = 1, 0; return s }()},
		{"whole ArcSeg", func() CurveSegment { s := arc; s.TStart, s.TEnd = 0, 1; return s }()},
		{"whole reversed ArcSeg", func() CurveSegment { s := arc; s.TStart, s.TEnd = 1, 0; return s }()},
		{"whole CircleSeg", func() CurveSegment { s := circle; s.TStart, s.TEnd = 0, 1; return s }()},
	} {
		t.Run(tc.name, func(t *testing.T) {
			w, err := walkOf(tc.seg, nil)
			require.NoError(t, err)
			got, err := walkChargeOf(tc.seg, w)
			require.NoError(t, err)
			require.Equal(t, 0.0, got)
		})
	}

	for _, tc := range []struct {
		name string
		seg  CurveSegment
	}{
		{"trimmed LineSeg", func() CurveSegment { s := line; s.TStart, s.TEnd = 0, 0.4; return s }()},
		{"trimmed ArcSeg", func() CurveSegment { s := arc; s.TStart, s.TEnd = 0, 0.4; return s }()},
		{"trimmed CircleSeg", func() CurveSegment { s := circle; s.TStart, s.TEnd = 0, 0.4; return s }()},
		// One ulp short of the natural bound: the walk's own closed-ness
		// tolerance calls this circle closed, and the charge must still be
		// positive — wholeness is the recorded range, never that tolerance.
		{"CircleSeg one ulp short of whole", func() CurveSegment {
			s := circle
			s.TStart, s.TEnd = 0, math.Nextafter(1, 0)
			return s
		}()},
	} {
		t.Run(tc.name, func(t *testing.T) {
			w, err := walkOf(tc.seg, nil)
			require.NoError(t, err)
			got, err := walkChargeOf(tc.seg, w)
			require.NoError(t, err)
			require.Positive(t, got)
			require.False(t, math.IsInf(got, 0))
		})
	}

	t.Run("non-finite coordinate answers +Inf", func(t *testing.T) {
		bad := LineSeg{Start: Point2{U: math.NaN(), V: 0}, End: Point2{U: 10, V: 0}, TStart: 0, TEnd: 0.4}
		w, err := walkOf(bad, nil)
		require.NoError(t, err)
		got, err := walkChargeOf(bad, w)
		require.NoError(t, err)
		require.True(t, math.IsInf(got, 1), "an absent bound must never read as a small one")
	})
}

// TestPrismCircularWalkChargeImpliesRefusal mechanises the reach §7 states for
// δ_walk: a positive charge is only ever computed over a trimmed LineSeg,
// because every circular carrier walkChargeOf could charge is one
// prismProfileHasTrimmedCircularSource refuses before buildPrismScene runs
// (§4.1). Over the circular ranges either side of that boundary, the charge
// being positive and the refusal firing must be the SAME condition — so a
// circular carrier admitted into a scene always charges zero, and the two
// answers cannot drift apart into a silently under-charged bound.
func TestPrismCircularWalkChargeImpliesRefusal(t *testing.T) {
	arc := ArcSeg{
		Center: Point2{U: 0, V: 0},
		Start:  Point2{U: 5, V: 0},
		End:    Point2{U: 0, V: 5},
	}
	circle := CircleSeg{Center: Point2{U: 0, V: 0}, Radius: units.Millimeters(5), CCW: true}
	withRange := func(seg CurveSegment, tStart, tEnd float64) CurveSegment {
		switch s := seg.(type) {
		case ArcSeg:
			s.TStart, s.TEnd = tStart, tEnd
			return s
		case CircleSeg:
			// A CircleSeg's CCW flag must agree with its range order
			// (validateSegmentRange), so a reversed range is a CW circle.
			s.TStart, s.TEnd, s.CCW = tStart, tEnd, tStart < tEnd
			return s
		}
		t.Fatalf("unexpected segment kind %T", seg)
		return nil
	}

	for _, base := range []struct {
		kind string
		seg  CurveSegment
	}{{"ArcSeg", arc}, {"CircleSeg", circle}} {
		for _, rng := range []struct {
			name         string
			tStart, tEnd float64
			wantRefusal  bool
		}{
			{name: "whole", tStart: 0, tEnd: 1, wantRefusal: false},
			{name: "whole reversed", tStart: 1, tEnd: 0, wantRefusal: false},
			{name: "trimmed", tStart: 0, tEnd: 0.4, wantRefusal: true},
			{name: "one ulp short of whole", tStart: 0, tEnd: math.Nextafter(1, 0), wantRefusal: true},
		} {
			t.Run(base.kind+"/"+rng.name, func(t *testing.T) {
				seg := withRange(base.seg, rng.tStart, rng.tEnd)
				w, err := walkOf(seg, nil)
				require.NoError(t, err)
				charge, err := walkChargeOf(seg, w)
				require.NoError(t, err)

				profile := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{seg}}}
				refused, err := prismProfileHasTrimmedCircularSource(newWorkBudget(t.Context()), profile)
				require.NoError(t, err)

				require.Equal(t, rng.wantRefusal, refused,
					"the fixture's own premise: this range must be the refusal case it names")
				require.Equal(t, refused, charge > 0,
					"a circular carrier charges exactly when it is refused, so an admitted one charges zero")
				if !refused {
					require.Equal(t, 0.0, charge,
						"a circular carrier that reaches a scene contributes nothing to δ_walk")
				}
			})
		}
	}
}

// prismWalkEndpointResidualSq is the EXACT squared distance between one walked
// endpoint of a LineSeg and the endpoint its record denotes, taken entirely
// over math/big.Rat: lerp2's own float answer against ratLerp's exact one, per
// coordinate, squared and summed. Nothing here rounds, so a charge compared
// against it is compared against the true error and not against a second float
// estimate of it.
func prismWalkEndpointResidualSq(t *testing.T, s LineSeg, walkedU, walkedV, at float64) *big.Rat {
	t.Helper()
	exactU := ratLerp(s.Start.U, s.End.U, at)
	exactV := ratLerp(s.Start.V, s.End.V, at)
	require.NotNil(t, exactU)
	require.NotNil(t, exactV)
	du := new(big.Rat).Sub(prismRatOf(t, walkedU), exactU)
	dv := new(big.Rat).Sub(prismRatOf(t, walkedV), exactV)
	return new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
}

// prismLerpFused and prismLerpSplit are this file's single owner of the two
// answers a conforming Go implementation may produce for lerp2's general arm
// start + t·(end − start), and every test here that needs either reading calls
// them rather than spelling the expression again.
//
// The spec (Floating-point operators) lets a target fuse the multiply and the
// add into one rounding, and the gc arm64 backend does exactly that: lerp2's
// general arm compiles to FSUBD followed by FMADDD there, while amd64 rounds
// the product and then the sum. Both helpers reproduce lerp2's natural-bound
// arms verbatim, so they differ from it only where it computes.
//
// prismLerpSplit's explicit float64 conversion is the barrier the spec names —
// it rounds the product to float64 precision and so forbids the fusion that
// would discard that rounding. Writing the sum without it does not state the
// unfused reading at all: the compiler is free to contract that spelling too,
// and on arm64 it does, which would leave a test comparing the fused answer
// against itself.
func prismLerpFused(start, end Point2, at float64) Point2 {
	if p, ok := prismLerpNaturalBound(start, end, at); ok {
		return p
	}
	return Point2{U: math.FMA(at, end.U-start.U, start.U), V: math.FMA(at, end.V-start.V, start.V)}
}

func prismLerpSplit(start, end Point2, at float64) Point2 {
	if p, ok := prismLerpNaturalBound(start, end, at); ok {
		return p
	}
	return Point2{U: start.U + float64(at*(end.U-start.U)), V: start.V + float64(at*(end.V-start.V))}
}

// prismLerpNaturalBound is lerp2's own t=0/t=1 arms, which return the record's
// coordinate verbatim and so round the same way on every target.
func prismLerpNaturalBound(start, end Point2, at float64) (Point2, bool) {
	switch at {
	case 0:
		return start, true
	case 1:
		return end, true
	}
	return Point2{}, false
}

// TestWalkChargeOfCoversLerpCancellation is the cancellation regression for
// §7's δ_walk: the charge a trimmed LineSeg owes must contain the EXACT
// rational residual of the endpoint lerp2 actually walked to, including when
// the carrier's own End − Start cancels and leaves the walked endpoint far
// smaller than the coordinates the rounding happened at.
//
// A Partial line fragment records its source line's full Start/End with a
// narrowed range (recordEdge, seam.go), so this shape is what an extruded
// profile carries whenever a sketch entity is cut near the sketch origin: the
// walked endpoint sits at ~0 while lerp2 rounds at the carrier's magnitude.
// Charging the walked endpoint's own envelope therefore under-charges without
// limit, which the premise assertion below pins per row — a row whose premise
// stops holding is a row that has stopped testing this defect, and it fails
// here rather than passing quietly.
//
// Every row's carrier is deliberately built so its own End − Start is NOT
// exactly representable (oneUlpDown below), because that subtraction is the
// only part of lerp2 whose rounding no target can fuse away. A symmetric
// carrier such as ±1e12 subtracts to an exactly representable difference, and
// a target that fuses the remaining multiply-add — the gc arm64 backend does;
// see prismLerpFused — then walks such a row to its exact endpoint, which
// leaves the row with nothing to discriminate. Every row is therefore also checked at both evaluations
// lerp2 is allowed to produce, not only at the one this target chose, so a
// row's discriminating power is proven here rather than assumed from the
// machine running the test.
//
// Every comparison is exact and squared, so no square root of the residual is
// ever taken and no float rounding can flatter a bound that failed to contain
// the true error.
func TestWalkChargeOfCoversLerpCancellation(t *testing.T) {
	// A parameter one ulp wide about the carrier's midpoint: the walked
	// endpoints all but coincide with the plane origin while the carrier
	// reaches ±1e12.
	const nearHalf = 0.4999999999999999

	// oneUlpDown moves a carrier's End one ulp down, which is what stops the
	// carrier's own End − Start landing on a representable float.
	oneUlpDown := func(x float64) float64 { return math.Nextafter(x, math.Inf(-1)) }

	for _, tc := range []struct {
		name string
		seg  LineSeg
		// endpointOnlyUnderCharges says the walked-endpoint envelope alone
		// (segmentWalk.coordUpper, which is what the answer must NOT be
		// charged at) fails to contain this row's own residual.
		endpointOnlyUnderCharges bool
	}{
		{
			name: "cancelling carrier reaching a million kilometres",
			seg: LineSeg{
				Start:  Point2{U: 1e12, V: 0},
				End:    Point2{U: oneUlpDown(-1e12), V: 0},
				TStart: nearHalf,
				TEnd:   math.Nextafter(nearHalf, 1),
			},
			endpointOnlyUnderCharges: true,
		},
		{
			// No extreme coordinate is needed: a plain 200 mm carrier with a
			// 0.4 mm fragment centred on the sketch origin already escapes a
			// charge read off the fragment's own magnitude.
			name: "200 mm carrier, 0.4 mm fragment on the origin",
			seg: LineSeg{
				Start:  Point2{U: -100, V: 0},
				End:    Point2{U: oneUlpDown(100), V: 0},
				TStart: 0.499,
				TEnd:   0.501,
			},
			endpointOnlyUnderCharges: true,
		},
		{
			name: "200 mm carrier, 0.02 mm fragment on the origin",
			seg: LineSeg{
				Start:  Point2{U: -100, V: 0},
				End:    Point2{U: oneUlpDown(100), V: 0},
				TStart: 0.49995,
				TEnd:   0.50005,
			},
			endpointOnlyUnderCharges: true,
		},
		{
			name: "diagonal cancelling carrier moves both coordinates",
			seg: LineSeg{
				Start:  Point2{U: -1e6, V: -1e6},
				End:    Point2{U: oneUlpDown(1e6), V: oneUlpDown(1e6)},
				TStart: 0.4999999999,
				TEnd:   0.5000000001,
			},
			endpointOnlyUnderCharges: true,
		},
		{
			// The ordinary shape the split-left-cell fixture carries: no
			// cancellation, the fragment sits on its own carrier's scale.
			// It must still be covered, and its premise must NOT hold — the
			// answer may not have become a blanket inflation of every row.
			name: "ordinary fragment of a 1..11 carrier",
			seg: LineSeg{
				Start:  Point2{U: 1, V: 0},
				End:    Point2{U: 11, V: 0},
				TStart: 0,
				TEnd:   0.4000000000000000222,
			},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			w, err := walkOf(tc.seg, nil)
			require.NoError(t, err)
			charge, err := walkChargeOf(tc.seg, w)
			require.NoError(t, err)
			require.Positive(t, charge)
			require.False(t, math.IsInf(charge, 0))

			chargeSq := new(big.Rat).Mul(prismRatOf(t, charge), prismRatOf(t, charge))
			endpointOnly := walkEndpointAllow(w.coordUpper)
			endpointOnlySq := new(big.Rat).Mul(prismRatOf(t, endpointOnly), prismRatOf(t, endpointOnly))

			// The three evaluations every row is judged at: the walk this
			// target actually performed, plus both answers lerp2's general arm
			// is allowed to produce. Judging all three is what keeps the row's
			// verdict — coverage AND premise — the same on a fused target as
			// on an unfused one.
			evaluations := [...]string{
				"as this target's own lerp2 evaluated it",
				"fused into one rounding, as the gc arm64 backend compiles it",
				"with the product rounded before the sum, as amd64 compiles it",
			}
			var premiseSeen [len(evaluations)]bool
			for _, end := range []struct {
				what string
				u, v float64
				at   float64
			}{
				{"start", w.startU, w.startV, tc.seg.TStart},
				{"end", w.endU, w.endV, tc.seg.TEnd},
			} {
				fused := prismLerpFused(tc.seg.Start, tc.seg.End, end.at)
				split := prismLerpSplit(tc.seg.Start, tc.seg.End, end.at)
				require.Truef(t, end.u == fused.U || end.u == split.U,
					"the %s endpoint's u (%g) must be one of the two answers lerp2 may give (fused %g, split %g) — an unmodelled evaluation is a row this table no longer judges",
					end.what, end.u, fused.U, split.U)
				require.Truef(t, end.v == fused.V || end.v == split.V,
					"the %s endpoint's v (%g) must be one of the two answers lerp2 may give (fused %g, split %g) — an unmodelled evaluation is a row this table no longer judges",
					end.what, end.v, fused.V, split.V)

				for i, walked := range [len(evaluations)]Point2{
					{U: end.u, V: end.v},
					fused,
					split,
				} {
					residualSq := prismWalkEndpointResidualSq(t, tc.seg, walked.U, walked.V, end.at)
					require.GreaterOrEqualf(t, chargeSq.Cmp(residualSq), 0,
						"the %s endpoint %s: the charge %g must contain its exact residual (squared residual %s)",
						end.what, evaluations[i], charge, residualSq.FloatString(40))
					if endpointOnlySq.Cmp(residualSq) < 0 {
						premiseSeen[i] = true
					}
				}
			}
			for i, seen := range premiseSeen {
				require.Equalf(t, tc.endpointOnlyUnderCharges, seen,
					"the premise, with the endpoint taken %s: charging the walked endpoint's own envelope (%g) instead of the carrier's must under-charge exactly on the rows this table says it does",
					evaluations[i], endpointOnly)
			}
		})
	}
}

// TestPrismBooleanTrimmedCircularSourceFallsBack is task fu143's own circular
// carrier row: a trimmed ArcSeg source segment (the arrangement's own arc
// through two cos/sin-computed points would move by more than a coordinate
// displacement can state, §4.1) refuses the analytic reduction before the
// scene is even built — ok=false, err=nil, the same silent §3.4 fallback
// every other entry-gate miss uses — rather than publish an under-charged
// bound for it.
func TestPrismBooleanTrimmedCircularSourceFallsBack(t *testing.T) {
	frame := canonicalPrismFrame(t)
	trimmedArc := prismPayload{
		profile: ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
			ArcSeg{
				Center: Point2{U: 5, V: 5},
				Start:  Point2{U: 8, V: 5},
				End:    Point2{U: 5, V: 8},
				TStart: 0, TEnd: 0.4,
			},
			LineSeg{Start: Point2{U: 5, V: 8}, End: Point2{U: 2, V: 8}, TStart: 0, TEnd: 1},
			LineSeg{Start: Point2{U: 2, V: 8}, End: Point2{U: 2, V: 2}, TStart: 0, TEnd: 1},
			LineSeg{Start: Point2{U: 2, V: 2}, End: Point2{U: 8, V: 5}, TStart: 0, TEnd: 1},
		}}},
		frame: frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	whole := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 20, 20)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}

	for _, op := range []OpKind{OpUnion, OpCut, OpIntersect} {
		_, ok, err := tryPrismBoolean(t.Context(), op, &Body{payload: trimmedArc}, &Body{payload: whole})
		require.NoError(t, err)
		require.False(t, ok, "%s over a trimmed circular source segment must fall back, never publish zero", op)
	}
}

// TestPrismProfileHasTrimmedCircularSourceReadsTheRecordedRange pins the
// refusal's own criterion on both circular kinds: wholeness is the recorded
// range compared exactly against 0 and 1, never the walk's closed-ness, which
// circularWalk decides within a tolerance of a full turn. A CircleSeg one ulp
// short of its natural bound walks as closed and is a trimmed carrier all the
// same, so the refusal must fire for it.
func TestPrismProfileHasTrimmedCircularSourceReadsTheRecordedRange(t *testing.T) {
	circle := func(tStart, tEnd float64) CircleSeg {
		return CircleSeg{
			Center: Point2{U: 5, V: 5},
			Radius: units.Millimeters(10),
			CCW:    tStart < tEnd,
			TStart: tStart, TEnd: tEnd,
		}
	}
	arc := func(tStart, tEnd float64) ArcSeg {
		return ArcSeg{
			Center: Point2{U: 5, V: 5},
			Start:  Point2{U: 8, V: 5},
			End:    Point2{U: 5, V: 8},
			TStart: tStart, TEnd: tEnd,
		}
	}

	// The two near-whole circles below are the whole point of this test: both
	// are ranges the walk's own tolerance reads as a closed turn, so a refusal
	// that consulted the walk would let them through.
	for _, seg := range []CircleSeg{circle(0, math.Nextafter(1, 0)), circle(math.Nextafter(0, 1), 1)} {
		w, err := walkOf(seg, nil)
		require.NoError(t, err)
		require.True(t, w.closed,
			"fixture [%v, %v] must be one circularWalk's own tolerance calls closed", seg.TStart, seg.TEnd)
	}

	for _, tc := range []struct {
		name    string
		seg     CurveSegment
		trimmed bool
	}{
		{"whole CircleSeg", circle(0, 1), false},
		{"whole reversed CircleSeg", circle(1, 0), false},
		{"CircleSeg one ulp short of 1", circle(0, math.Nextafter(1, 0)), true},
		{"CircleSeg one ulp past 0", circle(math.Nextafter(0, 1), 1), true},
		{"plainly trimmed CircleSeg", circle(0, 0.4), true},
		{"whole ArcSeg", arc(0, 1), false},
		{"whole reversed ArcSeg", arc(1, 0), false},
		{"trimmed ArcSeg", arc(0, 0.4), true},
		{"trimmed LineSeg carries no circular carrier", LineSeg{
			Start: Point2{U: 0, V: 0}, End: Point2{U: 10, V: 0}, TStart: 0, TEnd: 0.4,
		}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			profile := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{tc.seg}}}
			got, err := prismProfileHasTrimmedCircularSource(newWorkBudget(t.Context()), profile)
			require.NoError(t, err)
			require.Equal(t, tc.trimmed, got)
		})
	}
}

// TestPrismBooleanNearWholeCircleSourceFallsBack is the end-to-end half of the
// same row: a CircleSeg one ulp short of its natural bound must reroute every
// op to the mesh path, while the SAME pair with the bound recorded exactly is
// admitted analytically — so the fallback is caused by the trim itself and not
// by some unrelated miss elsewhere in the gate chain.
func TestPrismBooleanNearWholeCircleSourceFallsBack(t *testing.T) {
	frame := canonicalPrismFrame(t)
	circleOperand := func(tEnd float64) prismPayload {
		return prismPayload{
			profile: ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{CircleSeg{
				Center: Point2{U: 10, V: 10},
				Radius: units.Millimeters(4),
				CCW:    true,
				TStart: 0, TEnd: tEnd,
			}}}},
			frame: frame, z0: 0, z1: 10, xform: r3.Identity(),
		}
	}
	box := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 20, 20)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}

	for _, op := range []OpKind{OpUnion, OpCut, OpIntersect} {
		t.Run(op.String(), func(t *testing.T) {
			nearWhole := circleOperand(math.Nextafter(1, 0))
			_, ok, err := tryPrismBoolean(t.Context(), op, &Body{payload: box}, &Body{payload: nearWhole})
			require.NoError(t, err)
			require.False(t, ok, "a circle recorded one ulp short of whole must fall back, never publish zero")
		})
	}

	t.Run("the whole-circle control is admitted", func(t *testing.T) {
		_, ok, err := tryPrismBoolean(t.Context(), OpCut, &Body{payload: box}, &Body{payload: circleOperand(1)})
		require.NoError(t, err)
		require.True(t, ok, "the same pair with an exactly whole circle must reach the analytic result")
	})
}

// TestPrismUnionTrimmedSourceSplitBoundaryFallsBack is task fu143's own
// split-boundary reroute test (task 5's condition change), mirroring the
// existing TestPrismUnionDisplacedSourceSplitFallsBack: the trimmed-source
// operand A from prismSplitLeftCellBody, unioned with a box that genuinely
// overlaps its right wall (the split line itself), must fall back to the
// mesh path with no error rather than record a fragment whose crossing
// A's own walk charge could amplify.
func TestPrismUnionTrimmedSourceSplitBoundaryFallsBack(t *testing.T) {
	doc := New()
	a := prismSplitLeftCellBody(t, doc)
	pa := a.payload.(prismPayload)
	require.Zero(t, pa.sectionDelta, "the fixture's own displacement must come from the walk charge alone")

	b := prismRectBody(t, doc, 4, 3, 6, 7) // straddles A's right wall at u=5
	pb := b.payload.(prismPayload)

	reexpression, err := newPrismReexpression(pa, pb)
	require.NoError(t, err)
	require.True(t, reexpression.identity, "both operands share one frame with no placement between them")

	scene, _, sceneDelta, err := buildPrismScene(newWorkBudget(t.Context()), pa, pb, reexpression)
	require.NoError(t, err)
	require.Positive(t, sceneDelta.a, "operand A's own trimmed bottom/top walls must carry a walk charge")
	profiles, err := prismProfilesContext(t.Context(), scene.Profiles)
	require.NoError(t, err)
	split, err := prismProfilesHaveSplitBoundary(newWorkBudget(t.Context()), profiles)
	require.NoError(t, err)
	require.True(t, split, "the overlapping box must genuinely split A's own right wall")

	_, ok, err := tryPrismBoolean(t.Context(), OpUnion, &Body{payload: pa}, &Body{payload: pb})
	require.NoError(t, err)
	require.False(t, ok, "a split boundary with a nonzero walk charge must fall back")
}
