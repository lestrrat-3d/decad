package survey2d_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
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
