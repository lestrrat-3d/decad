package decad

import (
	"context"
	"errors"
	"math"
	"runtime"
	"strings"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is a deliberate internal-test exception (like
// selector_internal_test.go): the kernel's sub-resolution web semantic is
// unreachable through the public API — sketch's arrangement refuses the
// near-identical concentric circles that would carry a web under the
// kernel's candidate floor — yet the kernel must still never let such a
// candidate underwrite a pass if a future geometry source produces one.

func TestWallKernelFlagsOffJunctionSubTolerance(t *testing.T) {
	// Two concentric full circles 2e-8 apart at scale 10: the web disk is
	// under the candidate floor (4·1e-9·10) and no junction vertex is
	// supplied, so the kernel must flag it rather than silently treat the
	// boundary as web-free.
	outer, ok := arcElem(0, 0, 10, 0, 2*math.Pi, true)
	require.True(t, ok)
	inner, ok := arcElem(0, 0, 10-2e-8, 2*math.Pi, 0, true)
	require.True(t, ok)
	k := newWallKernel([]surveyElem{outer, inner}, nil, nil, 15*math.Pi/180, 0, false, math.Inf(1))
	out := k.run()
	require.True(t, out.subTolFar, `an off-junction sub-tolerance candidate must be flagged`)
}

func TestWallKernelCleanProfileDoesNotFlag(t *testing.T) {
	// A plain 100×60 rectangle produces no off-junction sub-tolerance
	// candidates: the flag stays clear and the reading is decided.
	pts := [][2]float64{{0, 0}, {100, 0}, {100, 60}, {0, 60}}
	elems := make([]surveyElem, 0, 4)
	for i := range pts {
		e, ok := lineElem(pts[i][0], pts[i][1], pts[(i+1)%4][0], pts[(i+1)%4][1])
		require.True(t, ok)
		elems = append(elems, e)
	}
	k := newWallKernel(elems, nil, pts, 15*math.Pi/180, 0, false, math.Inf(1))
	out := k.run()
	require.True(t, out.ok)
	require.False(t, out.subTolFar)
	require.True(t, out.hasSpan)
	require.InDelta(t, 60.0, out.span, 1e-9)
}

func TestWallKernelGenerateStreamsCandidates(t *testing.T) {
	arc, ok := arcElem(0, 0, 10, 0, 2*math.Pi, true)
	require.True(t, ok)
	k := newWallKernel([]surveyElem{arc}, nil, nil, 15*math.Pi/180, 0, false, math.Inf(1))
	stop := errors.New("stop after the first candidate")
	seen := 0

	err := k.generate(nil, func(diskCand) error {
		seen++
		return stop
	})

	require.ErrorIs(t, err, stop)
	require.Equal(t, 1, seen)
}

func TestWallKernelGenerateCancellationIsBounded(t *testing.T) {
	arc, ok := arcElem(0, 0, 10, 0, 2*math.Pi, true)
	require.True(t, ok)
	elems := make([]surveyElem, workPollInterval+64)
	for i := range elems {
		elems[i] = arc
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	seen := 0

	err := newWallKernel(elems, nil, nil, 15*math.Pi/180, 0, false, math.Inf(1)).
		generate(newWorkBudget(ctx), func(diskCand) error {
			seen++
			return nil
		})

	require.ErrorIs(t, err, context.Canceled)
	require.LessOrEqual(t, seen, workPollInterval-1)
}

func TestWallKernelValidateCancellationIsBounded(t *testing.T) {
	elems := make([]surveyElem, workPollInterval+64)
	for i := range elems {
		e, ok := lineElem(100, float64(i+1), 101, float64(i+1))
		require.True(t, ok)
		elems[i] = e
	}
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "validate"}
	k := newWallKernel(elems, nil, nil, 15*math.Pi/180, 0, false, math.Inf(1))

	spanning, empty, valid, err := k.validate(diskCand{x: 0, y: 0, r: 1}, newWorkBudget(ctx))
	_ = spanning
	_ = empty
	_ = valid

	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
}

func TestWallKernelContainsCancellationIsBounded(t *testing.T) {
	elems := make([]surveyElem, workPollInterval+64)
	for i := range elems {
		e, ok := arcElem(1000+float64(i), 1000, 1, 0, 2*math.Pi, true)
		require.True(t, ok)
		elems[i] = e
	}
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "contains"}
	k := newWallKernel(elems, nil, nil, 15*math.Pi/180, 0, false, math.Inf(1))

	_, _, err := k.contains(0, 0, newWorkBudget(ctx))

	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
}

func TestWallKernelBudgetedRunKeepsNormalResult(t *testing.T) {
	pts := [][2]float64{{0, 0}, {100, 0}, {100, 60}, {0, 60}}
	elems := make([]surveyElem, 0, len(pts))
	for i := range pts {
		e, ok := lineElem(pts[i][0], pts[i][1], pts[(i+1)%len(pts)][0], pts[(i+1)%len(pts)][1])
		require.True(t, ok)
		elems = append(elems, e)
	}
	k := newWallKernel(elems, nil, pts, 15*math.Pi/180, 0, false, math.Inf(1))

	out, err := k.runBudget(newWorkBudget(t.Context()))

	require.NoError(t, err)
	require.True(t, out.ok)
	require.True(t, out.hasSpan)
	require.InDelta(t, 60.0, out.span, 1e-9)
}

func TestPrismWallSubToleranceWebIsUndecided(t *testing.T) {
	// The prism path must apply the same rule as the revolve path: a
	// near-concentric annular profile whose 2e-8 web sits under the kernel
	// floor reads undecided — never a proven absence or a positive wall.
	pp := prismPayload{
		profile: ProfileRecord{
			Outer: LoopRecord{Segments: []CurveSegment{
				CircleSeg{Center: Point2{U: 0, V: 0}, Radius: units.Millimeters(10), CCW: true, TStart: 0, TEnd: 1},
			}},
			Holes: []LoopRecord{{Segments: []CurveSegment{
				CircleSeg{Center: Point2{U: 0, V: 0}, Radius: units.Millimeters(10 - 2e-8), CCW: false, TStart: 1, TEnd: 0},
			}}},
		},
		z0: 0, z1: 10,
	}
	out, err := prismWall(newWorkBudget(t.Context()), pp, 15*math.Pi/180)
	require.NoError(t, err)
	require.False(t, out.ok, `undecided, never a silent pass`)
}

func TestCupWallRequiresExactMorphology(t *testing.T) {
	line := func(u0, v0, u1, v1 float64) CurveSegment {
		return LineSeg{
			Start:  Point2{U: u0, V: v0},
			End:    Point2{U: u1, V: v1},
			TStart: 0,
			TEnd:   1,
		}
	}
	outer := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		line(0, 0, 100, 0),
		line(100, 0, 100, 60),
		line(100, 60, 0, 60),
		line(0, 60, 0, 0),
	}}}
	cavity, err := offsetProfile(outer, 1, 5)
	require.NoError(t, err)
	cp := cupPayload{
		outer:     outer,
		cavity:    cavity,
		zOuter:    0,
		zCav:      5,
		zOpen:     20,
		thickness: 5,
		sense:     Inward,
	}

	out, err := cupWall(newWorkBudget(t.Context()), cp, 15*math.Pi/180)
	require.NoError(t, err)
	require.True(t, out.ok)
	require.NotNil(t, out.reading)
	require.Equal(t, 5.0, *out.reading)

	// Move the stored cavity sideways from the exact five-millimetre offset.
	// The loop stays closed with the same positive area and loop count, but the
	// morphology certificate no longer holds, so the survey stays undecided.
	bad := cp
	bad.cavity.Outer.Segments = append([]CurveSegment(nil), cp.cavity.Outer.Segments...)
	for i, seg := range bad.cavity.Outer.Segments {
		moved := seg.(LineSeg)
		moved.Start.U += 0.25
		moved.End.U += 0.25
		bad.cavity.Outer.Segments[i] = moved
	}

	out, err = cupWall(newWorkBudget(t.Context()), bad, 15*math.Pi/180)
	require.NoError(t, err)
	require.False(t, out.ok, `a malformed offset relation must not return the recipe thickness`)

	body := &Body{payload: bad}
	br := BodyReport{Body: body, Solid: true}
	diags, err := runSurveys(newWorkBudget(t.Context()), &br, verifyConfig{
		wall:     &wallSpec{tool: units.Millimeters(1)},
		toolMM:   1,
		allowRad: 15 * math.Pi / 180,
	})
	require.NoError(t, err)
	require.Nil(t, br.MinWallThickness)
	require.Len(t, diags, 1)
	require.Equal(t, DiagUndecidedWall, diags[0].Code)
	require.NotEqual(t, DiagUnsupportedSurveyPayload, diags[0].Code)
	require.Equal(t, Suspect, diags[0].Status)
	require.NotContains(t, diags[0].Message, "facetedPayload",
		`an undecided analytic survey must not be reported as an unsupported payload`)
}

func manySegmentProfile(segmentCount int) ProfileRecord {
	segs := make([]CurveSegment, segmentCount)
	for i := range segmentCount {
		th0 := 2 * math.Pi * float64(i) / float64(segmentCount)
		th1 := 2 * math.Pi * float64(i+1) / float64(segmentCount)
		segs[i] = LineSeg{
			Start:  Point2{U: 100 * math.Cos(th0), V: 100 * math.Sin(th0)},
			End:    Point2{U: 100 * math.Cos(th1), V: 100 * math.Sin(th1)},
			TStart: 0,
			TEnd:   1,
		}
	}
	return ProfileRecord{Outer: LoopRecord{Segments: segs}}
}

func newFrameWorkBudget(target string) (*workBudget, *bool) {
	entered := false
	cancelled := false
	inTarget := func() bool {
		pcs := make([]uintptr, 32)
		frames := runtime.CallersFrames(pcs[:runtime.Callers(2, pcs)])
		for {
			frame, more := frames.Next()
			if strings.HasSuffix(frame.Function, "."+target) {
				return true
			}
			if !more {
				return false
			}
		}
	}
	cancelInTarget := func() error {
		if cancelled {
			return context.Canceled
		}
		if !inTarget() {
			return nil
		}
		entered = true
		cancelled = true
		return context.Canceled
	}
	return &workBudget{stepFn: cancelInTarget, errFn: cancelInTarget}, &entered
}

func TestRecordLoopsCancellationIsBounded(t *testing.T) {
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "recordLoops"}
	_, err := recordLoops(newWorkBudget(ctx), manySegmentProfile(workPollInterval+64))
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered, `profile segment resolution must poll inside recordLoops`)
}

func TestRevolveLoopsCancellationIsBounded(t *testing.T) {
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "revolveLoops"}
	_, err := revolveLoops(newWorkBudget(ctx), revolvePayload{
		profile: manySegmentProfile(workPollInterval + 64),
		ax:      axisFrame{dU: 1},
	})
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered, `profile segment resolution must poll inside revolveLoops`)
}

func TestCupWallCancellationCoversOffsetAuditAndReverse(t *testing.T) {
	outer := manySegmentProfile(workPollInterval + 64)
	cavity, err := offsetProfile(outer, 1, 5)
	require.NoError(t, err)
	cp := cupPayload{
		outer:     outer,
		cavity:    cavity,
		zOuter:    0,
		zCav:      5,
		zOpen:     20,
		thickness: 5,
		sense:     Inward,
	}
	out, err := cupWall(newWorkBudget(t.Context()), cp, 15*math.Pi/180)
	require.NoError(t, err)
	require.True(t, out.ok)

	for _, target := range []string{"coalesceWalksBudget", "crossingAuditBudget", "reverseLoopRecordBudget"} {
		t.Run(target, func(t *testing.T) {
			budget, entered := newFrameWorkBudget(target)
			_, err := cupWall(budget, cp, 15*math.Pi/180)
			require.ErrorIs(t, err, context.Canceled)
			require.True(t, *entered, `cup wall work must poll inside the named phase`)
		})
	}
}
