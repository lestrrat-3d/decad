package decad

import (
	"errors"
	"math"
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

func TestWallCandidateWorkChecked(t *testing.T) {
	work, ok := wallCandidateWork(4, 4, false)
	require.True(t, ok)
	require.Equal(t, uint64(88), work)

	work, ok = wallCandidateWork(4, 4, true)
	require.True(t, ok)
	require.Equal(t, uint64(124), work)

	maxInt := int(^uint(0) >> 1)
	_, ok = wallCandidateWork(maxInt, maxInt, true)
	require.False(t, ok)
}

func TestWallKernelStreamsCandidates(t *testing.T) {
	circle, ok := arcElem(0, 0, 10, 0, 2*math.Pi, true)
	require.True(t, ok)
	k := newWallKernel([]surveyElem{circle}, nil, nil, 0, 0, false, math.Inf(1))
	stop := errors.New("stop after first candidate")
	seen := 0
	err := k.generate(nil, func(diskCand) error {
		seen++
		return stop
	})
	require.ErrorIs(t, err, stop)
	require.Equal(t, 1, seen)
}

func TestWallKernelSharesWorkBudgetAcrossGenerationAndValidation(t *testing.T) {
	pts := [][2]float64{{0, 0}, {100, 0}, {100, 60}, {0, 60}}
	elems := make([]surveyElem, 0, len(pts))
	for i := range pts {
		e, ok := lineElem(pts[i][0], pts[i][1], pts[(i+1)%len(pts)][0], pts[(i+1)%len(pts)][1])
		require.True(t, ok)
		elems = append(elems, e)
	}
	k := newWallKernel(elems, nil, pts, 0, 0, false, math.Inf(1))

	_, err := k.runBudget(newWallWorkBudget(1))
	require.ErrorIs(t, err, errWallWorkBudget)

	_, _, ok, err := k.validate(diskCand{x: 50, y: 30, r: 30}, newWallWorkBudget(1))
	require.False(t, ok)
	require.ErrorIs(t, err, errWallWorkBudget)
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
				CircleSeg{Center: Point2{U: 0, V: 0}, Radius: units.Millimeters(10 - 2e-8), CCW: false, TStart: 0, TEnd: 1},
			}}},
		},
		z0: 0, z1: 10,
	}
	out := prismWall(pp, 15*math.Pi/180)
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

	out := cupWall(cp, 15*math.Pi/180)
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

	out = cupWall(bad, 15*math.Pi/180)
	require.False(t, out.ok, `a malformed offset relation must not return the recipe thickness`)

	body := &Body{payload: bad}
	br := BodyReport{Body: body, Status: Sound, Solid: true}
	diags := runSurveys(&br, verifyConfig{
		wall:     &wallSpec{tool: units.Millimeters(1)},
		toolMM:   1,
		allowRad: 15 * math.Pi / 180,
	})
	require.Nil(t, br.MinWallThickness)
	require.Len(t, diags, 1)
	require.Equal(t, DiagUndecidedWall, diags[0].Code)
	require.NotEqual(t, DiagUnsupportedSurveyPayload, diags[0].Code)
	require.Equal(t, Suspect, diags[0].Status)
	require.NotContains(t, diags[0].Message, "facetedPayload",
		`an undecided analytic survey must not be reported as an unsupported payload`)
}
