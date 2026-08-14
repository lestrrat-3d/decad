package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is prism_overlap.go's own white-box test suite: the admission and
// decline mechanics prism_overlap_test.go's black-box Verify-level suite does
// not isolate on its own — the exactly-tangent decline, the arrangement cap,
// mid-reading cancellation, and the G1-G4-style regression fallbacks. It
// reuses prism_boolean_internal_test.go's canonicalPrismFrame/synthRectLoop
// synthetic-payload machinery.

// internalPolyPrismBody extrudes a closed polygon (one outer loop, no holes)
// from z=0 to z=h into a real Document, every vertex pinned at its authored
// coordinate — the internal-package twin of prism_overlap_test.go's
// polyPrismBody, needed here because prismOverlapVolume's per-cell
// measurement calls d.nextStepRef() and requires a live *Document.
func internalPolyPrismBody(t *testing.T, doc *Document, pts [][2]float64, h float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
		s.Fix(sp[i])
	}
	for i := range sp {
		s.CreateLine(sp[i], sp[(i+1)%len(sp)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profs := s.Profiles()
	require.Len(t, profs, 1)
	body, err := doc.Extrude(s, profs[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	return body
}

// TestPrismOverlapVolumeDeclinesExactlyTangentPair is §4.5's own "what the
// reading declines" paragraph: two coplanar prisms whose sections meet along
// a shared wall but enclose no common area must never publish a zero-volume
// overlap. Whichever internal reason declines it — resolvePrismCrossingCells
// reporting unresolved (the coincident-carrier case its own header names) or
// resolved with an empty selected set — prismOverlapVolume's own contract is
// the same either way: ok=false, err=nil.
func TestPrismOverlapVolumeDeclinesExactlyTangentPair(t *testing.T) {
	doc := New()
	frame := canonicalPrismFrame(t)
	pa := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 10, 10)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	pb := pa
	pb.profile = ProfileRecord{Outer: synthRectLoop(10, 0, 20, 10)}
	a := &Body{doc: doc, payload: pa}
	b := &Body{doc: doc, payload: pb}

	volume, ok, err := prismOverlapVolume(t.Context(), a, b)
	require.NoError(t, err)
	require.False(t, ok, "a shared-wall contact must decline, never publish a zero-volume overlap")
	require.Equal(t, Measurement{}, volume)
}

// TestPrismOverlapVolumeArrangementCapRefuses is §9's RB7: a combined scene
// exceeding prismMaxArrangementSegments refuses with ErrUnsupported before
// s.Profiles runs, wrapped as booleanExpectedUnsupported exactly as
// evaluateAnalyticIntersect's own RB7 is, so measuredInterference reads it as
// interferenceUnsupportedPipeline and Verify reports Suspect rather than
// erroring.
func TestPrismOverlapVolumeArrangementCapRefuses(t *testing.T) {
	doc := New()
	frame := canonicalPrismFrame(t)
	pp := prismPayload{
		profile: ProfileRecord{Outer: synthDenseRectLoop(prismMaxArrangementSegments/8 + 1)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	a := &Body{doc: doc, payload: pp}
	b := &Body{doc: doc, payload: pp}

	_, ok, err := prismOverlapVolume(t.Context(), a, b)
	require.False(t, ok)
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
	expected, isExpected := asExpectedBoolean(err)
	require.True(t, isExpected, "RB7 must wrap as a booleanExpectedError, matching evaluateAnalyticIntersect")
	require.Equal(t, booleanExpectedUnsupported, expected.kind)
}

// crossTeethPts and crossBarPts build the same two-disjoint-region U-and-bar
// shape as prism_overlap_test.go's own fixture, restated here so this
// internal-package file needs no cross-package helper.
var (
	crossUPts = [][2]float64{
		{0, 0}, {4, 0}, {4, 8}, {6, 8}, {6, 0}, {10, 0}, {10, 12}, {0, 12},
	}
	crossBarPts = [][2]float64{{-2, 2}, {12, 2}, {12, 5}, {-2, 5}}
)

// TestPrismOverlapVolumeCancellationLeavesDocumentUntouched is §15's
// cancellation row: a context canceled during the per-cell measurement
// returns ctx.Err() unchanged and leaves the document and both operands
// untouched — the reading never calls nextStepRef for a real step, appends a
// Step, retires an operand, or registers a body (§12's Interference row).
func TestPrismOverlapVolumeCancellationLeavesDocumentUntouched(t *testing.T) {
	doc := New()
	u := internalPolyPrismBody(t, doc, crossUPts, 5)
	bar := internalPolyPrismBody(t, doc, crossBarPts, 5)
	beforeBodies := append([]*Body{}, doc.Bodies()...)
	beforeSteps := len(doc.steps)

	ctx := &internalCancelContext{Context: t.Context(), limit: 40}
	_, ok, err := prismOverlapVolume(ctx, u, bar)
	require.False(t, ok)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, beforeBodies, doc.Bodies())
	require.Equal(t, beforeSteps, len(doc.steps))
	require.NoError(t, doc.requireLive(u), "the operand must not be retired by a canceled reading")
	require.NoError(t, doc.requireLive(bar))
}

// TestPrismOverlapVolumeRegressionFallbacks is §15's regression row over
// prismOverlapVolume specifically: a non-coplanar pair, a reflected
// placement, and a pair carrying a free-form segment each take the exact
// silent-fallback path they take through tryPrismBoolean/
// admitPrismIntersectPair today — this reading shares that gate unchanged
// (Task 1) rather than restating it.
func TestPrismOverlapVolumeRegressionFallbacks(t *testing.T) {
	doc := New()
	frame := canonicalPrismFrame(t)
	pa := prismPayload{
		profile: ProfileRecord{Outer: synthRectLoop(0, 0, 10, 10)},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}

	t.Run("non-coplanar pair", func(t *testing.T) {
		shifted, err := r3.Translation(r3.Vec{Z: 3})
		require.NoError(t, err)
		pb := pa
		pb.profile = ProfileRecord{Outer: synthRectLoop(5, 5, 15, 15)}
		pb.xform = shifted
		a := &Body{doc: doc, payload: pa}
		b := &Body{doc: doc, payload: pb}
		_, ok, err := prismOverlapVolume(t.Context(), a, b)
		require.NoError(t, err)
		require.False(t, ok, "G3: a non-coplanar pair must never admit")
	})

	t.Run("reflected operand", func(t *testing.T) {
		mirror, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
		require.NoError(t, err)
		refl, err := r3.Reflection(mirror)
		require.NoError(t, err)
		require.True(t, refl.IsReflection())
		pb := pa
		pb.profile = ProfileRecord{Outer: synthRectLoop(5, 5, 15, 15)}
		pb.xform = refl
		a := &Body{doc: doc, payload: pa}
		b := &Body{doc: doc, payload: pb}
		_, ok, err := prismOverlapVolume(t.Context(), a, b)
		require.NoError(t, err)
		require.False(t, ok, "G2: a reflected operand must never admit")
	})

	t.Run("free-form segment", func(t *testing.T) {
		pb := pa
		pb.profile = ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
			EllipseSeg{Center: Point2{U: 20, V: 0}, Rx: units.Millimeters(4), Ry: units.Millimeters(3), CCW: true, TStart: 0, TEnd: 1},
		}}}
		a := &Body{doc: doc, payload: pa}
		b := &Body{doc: doc, payload: pb}
		_, ok, err := prismOverlapVolume(t.Context(), a, b)
		require.NoError(t, err)
		require.False(t, ok, "G4: a non-analytic (ellipse) segment must never admit")
	})
}

// TestPrismOverlapVolumeMatchesEvalPrismOnASingleCell is a targeted sanity
// check that prismOverlapVolume's per-cell measurement agrees with
// resolveAndBuildPrismIntersectCrossing's own crossing-sub-case answer for a
// pair whose overlap happens to be ONE region: both routes record the same
// single arrangement cell through the same recordEdge/edgeJoin/
// prismUnionCutDelta sequence and the same evalPrism math, so their published
// Value must agree exactly. Bound is not required to match bit for bit — the
// one-cell sum still charges exactSumRound's own accumulated-rounding term
// (§4.5's "The sum" paragraph), which is a legitimate, slightly more
// conservative outward rounding a single-term sum still commits, never a
// tighter one.
func TestPrismOverlapVolumeMatchesEvalPrismOnASingleCell(t *testing.T) {
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 5)
	b := internalBoxBody(t, doc, 5, 5, 15, 15, 5)

	want, ok, err := evaluateAnalyticIntersect(t.Context(), a, b)
	require.NoError(t, err)
	require.True(t, ok, "the single-region crossing sub-case must resolve")

	got, ok, err := prismOverlapVolume(t.Context(), a, b)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, want.volume.Value, got.Value)
	require.Equal(t, want.volume.Exactness, got.Exactness)
	require.GreaterOrEqual(t, got.Bound.Base(), want.volume.Bound.Base(),
		"the sum's own accumulated rounding can only widen the bound, never narrow it")
	require.InDelta(t, 125.0, got.Value.Base(), 1e-9)
	require.Less(t, got.Bound.Base(), 1e-9)
}

// TestPrismOverlapVolumeSoundAgainstIndependentTriangleSum is §15's cell-sum
// soundness row, at the internal-package level: for a three-region comb
// fixture, the published interval must contain the exact area computed
// independently over each tooth as a plain axis-aligned rectangle, never a
// second float computation.
func TestPrismOverlapVolumeSoundAgainstIndependentTriangleSum(t *testing.T) {
	doc := New()
	comb := [][2]float64{
		{0, 0}, {16, 0}, {16, 2}, {15, 2}, {15, 10}, {13, 10}, {13, 2},
		{9, 2}, {9, 10}, {7, 10}, {7, 2}, {3, 2}, {3, 10}, {1, 10}, {1, 2}, {0, 2},
	}
	bar := [][2]float64{{-2, 4}, {18, 4}, {18, 6}, {-2, 6}}
	const h = 3.0
	a := internalPolyPrismBody(t, doc, comb, h)
	b := internalPolyPrismBody(t, doc, bar, h)

	volume, ok, err := prismOverlapVolume(t.Context(), a, b)
	require.NoError(t, err)
	require.True(t, ok)
	const want = 3 * 2 * 2 * h // three teeth, each 2x2 mm^2, times h
	require.InDelta(t, want, volume.Value.Base(), volume.Bound.Base())
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(), 0.0)
	require.Less(t, volume.Bound.Base(), 1e-9)
}
