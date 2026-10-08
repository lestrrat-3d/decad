package survey2d_test

import (
	"math"

	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// identityFrameMap maps plane-local u, v and n onto world x, y and z.
func identityFrameMap(t *testing.T) survey2d.PlacedFrameMap {
	t.Helper()
	iv := func(v r3.Vec) proofbound.IvVec3 {
		out, ok := proofbound.IvVec3Of(v)
		require.True(t, ok)
		return out
	}
	return survey2d.PlacedFrameMap{
		Origin: iv(r3.NewVec(0, 0, 0)),
		Du:     iv(r3.NewVec(1, 0, 0)),
		Dv:     iv(r3.NewVec(0, 1, 0)),
		Dn:     iv(r3.NewVec(0, 0, 1)),
	}
}

func lineWalk(t *testing.T, seg sectionrecord.LineSeg) survey2d.SideWalk {
	t.Helper()
	w, err := boundarywalk.WalkOf(seg, freeform.NewFreeformWork())
	require.NoError(t, err)
	return survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}}
}

// TestWallNormalDecisionReadsRecordedDirection pins that a straight wall's
// verdict follows the direction its recorded endpoints denote. The wall runs
// (24, 32) → (−1e−15, 0), whose endpoint difference −24 − 1e−15 rounds to −24,
// so the walk holds the tangent (−24, −32). Against the in-plane pull
// (−3, −4) the held tangent gives the component exactly zero, a clear wall,
// while the recorded direction gives −4e−15 / (40·5) and the wall opposes. The
// pull (3, 4) is clear either way.
//
// Shown to fail: with WallNormalDecision reading FloatRat(TanInU, TanInV)
// again, the pull (−3, −4) answers PullClear.
func TestWallNormalDecisionReadsRecordedDirection(t *testing.T) {
	t.Parallel()
	w := lineWalk(t, sectionrecord.LineSeg{
		Start: sectionrecord.Point2{U: 24, V: 32},
		End:   sectionrecord.Point2{U: -1e-15, V: 0},
		TEnd:  1,
	})
	require.Equal(t, [2]float64{-24, -32}, [2]float64{w.TanInU, w.TanInV}, `the fixture needs the rounded tangent`)
	m := identityFrameMap(t)

	verdict, ok := survey2d.WallNormalDecision(w, m, r3.NewVec(-3, -4, 0))
	require.True(t, ok)
	require.Equal(t, survey2d.PullOpposes, verdict)

	verdict, ok = survey2d.WallNormalDecision(w, m, r3.NewVec(3, 4, 0))
	require.True(t, ok)
	require.Equal(t, survey2d.PullClear, verdict)
}

// TestWallNormalDecisionUndecidedOnComputedEndpoint pins the refusal when a
// straight wall's endpoint is computed rather than recorded. The wall is
// (0, 0) → (3, 4) trimmed to t ∈ [1/3, 1], so its start is a float lerp under
// a nonzero end bound. The pull (3, 4) runs along the wall, an exact
// component of zero, and the box the start's bound allows holds directions
// on both sides of it, so no verdict is proven.
//
// Shown to fail: with WallNormalDecision reading FloatRat(TanInU, TanInV)
// again, the held tangent decides the sign and the reader answers
// PullClear, which the computed start does not prove.
func TestWallNormalDecisionUndecidedOnComputedEndpoint(t *testing.T) {
	t.Parallel()
	w := lineWalk(t, sectionrecord.LineSeg{
		Start:  sectionrecord.Point2{U: 0, V: 0},
		End:    sectionrecord.Point2{U: 3, V: 4},
		TStart: 1.0 / 3, TEnd: 1,
	})
	require.Positive(t, proofbound.WalkEndBoundAllow(w.StartBound), `the fixture needs a computed start`)

	verdict, ok := survey2d.WallNormalDecision(w, identityFrameMap(t), r3.NewVec(3, 4, 0))
	require.True(t, ok)
	require.Equal(t, survey2d.PullUndecided, verdict)
}

func circularWalk(t *testing.T, seg sectionrecord.CurveSegment) survey2d.SideWalk {
	t.Helper()
	w, err := boundarywalk.WalkOf(seg, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.True(t, w.IsCircular())
	require.False(t, w.Closed)
	return survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}}
}

// TestWallNormalDecisionReadsDenotedArcWindow pins that a circular wall's
// verdict follows the window its record denotes, not the held Th0 and Th1.
// Every arc runs counterclockwise about the origin, so its outward normal at
// angle θ is (cos θ, sin θ).
//
// The quarter arc (1, 0) → (0, 1) denotes the window [0, π/2], and the walk
// holds math.Atan2(1, 0) = π/2 rounded down by about 6.1e−17. Against the pull
// (1, −3e−17) the component cos θ − 3e−17·sin θ is −3e−17 at θ = π/2, so the
// wall opposes, while every angle the held window reaches gives a positive
// component.
//
// The quarter arc (0, 1) → (−1, 0) denotes [π/2, π]. Against the pull
// (−1, 0) the component −cos θ is at least zero there, so the wall is clear,
// while the held window starts below π/2, where −cos θ is negative.
//
// The CircleSeg quarter turn denotes [0, π/2] too, but the walk computes its
// end point through math.Sincos and states a nonzero end bound. The box that
// bound allows straddles the direction where the first case's component
// changes sign, so no verdict is proven.
//
// Shown to fail: with WallNormalDecision reading [Th0, Th1] as the exact
// window again, the three cases answer PullClear, PullOpposes and PullClear.
func TestWallNormalDecisionReadsDenotedArcWindow(t *testing.T) {
	t.Parallel()
	m := identityFrameMap(t)
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }

	first := circularWalk(t, sectionrecord.ArcSeg{Start: p(1, 0), End: p(0, 1), TEnd: 1})
	require.Equal(t, math.Pi/2, first.Th1, `the fixture needs the held end π/2 rounded down`)
	verdict, ok := survey2d.WallNormalDecision(first, m, r3.NewVec(1, -3e-17, 0))
	require.True(t, ok)
	require.Equal(t, survey2d.PullOpposes, verdict)

	second := circularWalk(t, sectionrecord.ArcSeg{Start: p(0, 1), End: p(-1, 0), TEnd: 1})
	require.Equal(t, math.Pi/2, second.Th0, `the fixture needs the held start π/2 rounded down`)
	verdict, ok = survey2d.WallNormalDecision(second, m, r3.NewVec(-1, 0, 0))
	require.True(t, ok)
	require.Equal(t, survey2d.PullClear, verdict)

	quarter := circularWalk(t, sectionrecord.CircleSeg{Radius: units.Millimeters(1), CCW: true, TEnd: 0.25})
	require.Equal(t, math.Pi/2, quarter.Th1, `the fixture needs the held end π/2 rounded down`)
	require.Positive(t, proofbound.WalkEndBoundAllow(quarter.EndBound), `the fixture needs a computed end`)
	verdict, ok = survey2d.WallNormalDecision(quarter, m, r3.NewVec(1, -3e-17, 0))
	require.True(t, ok)
	require.Equal(t, survey2d.PullUndecided, verdict)
}
