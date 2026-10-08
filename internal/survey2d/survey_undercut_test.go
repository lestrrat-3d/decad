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

// TestWallNormalDecisionUnbracketedWindowReadsItsEnds pins that a circular
// wall whose window cannot be cut into arcs shorter than a half turn is
// decided from its two ends alone. The walk is about the origin with radius 1
// and holds the window [0, π]. Its start (1, 0) and its end
// (cos 2.94, sin 2.94) each carry an end bound that reaches 0.95 from the
// point, so each end's angle enclosure spreads about ±1.49 and the two
// enclosures overlap across a window whose held sweep is a half turn.
//
// Against the pull (1, 0) the start's box reads a component above zero and
// the end's box one below it, so a point strictly between −1 and 0 lies
// between them and the wall opposes. The pull (0, 0, 1) runs along the sweep,
// a component of zero everywhere, so the wall is clear. Against (0, −1) both
// boxes straddle zero and nothing is proven.
//
// Shown to fail: with WallNormalDecision answering PullUndecided for every
// unbracketed window again, the pulls (1, 0) and (0, 0, 1) answer
// PullUndecided.
func TestWallNormalDecisionUnbracketedWindowReadsItsEnds(t *testing.T) {
	t.Parallel()
	const reach, endAngle = 0.95, 2.94
	perAxis := reach / math.Sqrt(3)
	bound := proofbound.WalkEndBound{U: perAxis, V: perAxis}
	allow := proofbound.WalkEndBoundAllow(bound)
	require.Less(t, allow, 1.0, `each end's box must stay off the centre`)
	require.Greater(t, allow, 0.9, `each end's angle enclosure must spread past a quarter turn`)
	w := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		Kind:       survey2d.WalkCircular,
		Radius:     1,
		Th0:        0,
		Th1:        math.Pi,
		StartU:     1,
		EndU:       math.Cos(endAngle),
		EndV:       math.Sin(endAngle),
		StartBound: bound,
		EndBound:   bound,
	}, Segs: []int{0}}

	low, ok := survey2d.EndAngleEnclosure(0, 0, w.StartU, w.StartV, allow, w.Th0)
	require.True(t, ok)
	high, ok := survey2d.EndAngleEnclosure(0, 0, w.EndU, w.EndV, allow, w.Th1)
	require.True(t, ok)
	require.LessOrEqual(t, high.Lo.Cmp(low.Hi), 0, `the fixture needs the two ends' angle enclosures to overlap`)

	requireWallVerdicts(t, w, map[r3.Vec]survey2d.PullVerdict{
		r3.NewVec(1, 0, 0):  survey2d.PullOpposes,
		r3.NewVec(0, 0, 1):  survey2d.PullClear,
		r3.NewVec(0, -1, 0): survey2d.PullUndecided,
	})
}

// TestWallNormalDecisionUnenclosedEndStaysWithItsWall pins that a circular
// wall with an end whose direction cannot be enclosed answers for itself
// instead of refusing, so the body's survey keeps every other wall's verdict.
// Each walk is a counterclockwise quarter about the origin with radius 1,
// holding [0, π/2]. Its start (1, 0) carries an end bound that reaches 1.5,
// so the start's box holds the centre and no direction is read from it.
//
// With the end (0, 1) recorded exactly, the pull (1, −1) gives the component
// −1/√2 there, strictly between −1 and 0, so the wall opposes. The pull (1, 0) reads exactly zero at that end and nothing proves
// a point below it, so nothing is decided. The pull (0, 0, 1) is clear.
//
// With the end's bound underivable as well, no end is read, and only the
// pull (0, 0, 1), whose component is zero everywhere, is decided.
//
// Shown to fail: with circularWindowOf refusing an end whose box reaches the
// centre again, every pull answers ok false.
func TestWallNormalDecisionUnenclosedEndStaysWithItsWall(t *testing.T) {
	t.Parallel()
	perAxis := 1.5 / math.Sqrt(3)
	wide := proofbound.WalkEndBound{U: perAxis, V: perAxis}
	require.GreaterOrEqual(t, proofbound.WalkEndBoundAllow(wide), 1.0, `the start's box must reach the centre`)
	quarter := func(endBound proofbound.WalkEndBound) survey2d.SideWalk {
		return survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
			Kind:       survey2d.WalkCircular,
			Radius:     1,
			Th0:        0,
			Th1:        math.Pi / 2,
			StartU:     1,
			EndV:       1,
			StartBound: wide,
			EndBound:   endBound,
		}, Segs: []int{0}}
	}

	requireWallVerdicts(t, quarter(proofbound.WalkEndBound{}), map[r3.Vec]survey2d.PullVerdict{
		r3.NewVec(1, -1, 0): survey2d.PullOpposes,
		r3.NewVec(1, 0, 0):  survey2d.PullUndecided,
		r3.NewVec(0, 0, 1):  survey2d.PullClear,
	})
	requireWallVerdicts(t, quarter(proofbound.WalkEndBound{U: math.Inf(1)}), map[r3.Vec]survey2d.PullVerdict{
		r3.NewVec(1, -1, 0): survey2d.PullUndecided,
		r3.NewVec(0, 0, 1):  survey2d.PullClear,
	})
}

func requireWallVerdicts(t *testing.T, w survey2d.SideWalk, want map[r3.Vec]survey2d.PullVerdict) {
	t.Helper()
	for pull, verdict := range want {
		got, ok := survey2d.WallNormalDecision(w, identityFrameMap(t), pull)
		require.True(t, ok, `pull %v: the wall answers for itself`, pull)
		require.Equal(t, verdict, got, `pull %v`, pull)
	}
}
