package boundarywalk_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

func TestJunctionVertexAtReplayEqualRadiusArc(t *testing.T) {
	right, left := sectionrecord.Point2{U: 1}, sectionrecord.Point2{U: -1}
	arc := sectionrecord.ArcSeg{Start: right, End: left, TEnd: 1}
	line := sectionrecord.LineSeg{Start: left, End: right, TEnd: 1}
	aw, err := boundarywalk.WalkOf(arc, nil)
	require.NoError(t, err)
	lw, err := boundarywalk.WalkOf(line, nil)
	require.NoError(t, err)
	segs := []sectionrecord.CurveSegment{arc, line}
	prev := survey2d.SideWalk{SegmentWalk: aw, Segs: []int{0}}
	next := survey2d.SideWalk{SegmentWalk: lw, Segs: []int{1}}
	wantU, wantV, wantBound := boundarywalk.JunctionVertexAt(segs, prev, next)
	meter := proofbound.NewReplayTrialBudget()
	u, v, bound, err := boundarywalk.JunctionVertexAtReplay(t.Context(), segs, prev, next, meter)
	require.NoError(t, err)
	require.Equal(t, [3]float64{wantU, wantV, wantBound}, [3]float64{u, v, bound})
	w, r := meter.Usage()
	bytes := meter.Bytes()
	for _, limits := range [...][3]uint64{{w - 1, r, bytes}, {w, r - 1, bytes}, {w, r, bytes - 1}} {
		short, e := proofbound.NewReplayBudgetLimits(limits[0], limits[1], limits[2])
		require.NoError(t, e)
		_, _, _, e = boundarywalk.JunctionVertexAtReplay(t.Context(), segs, prev, next, short)
		require.Error(t, e)
	}
	drifted := arc
	drifted.End.V = 0.01
	segs[0] = drifted
	_, _, _, err = boundarywalk.JunctionVertexAtReplay(t.Context(), segs, prev, next, //nolint:dogsled // This test checks only the refusal and its error.
		proofbound.NewReplayTrialBudget())
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	cancelled := proofbound.NewReplayTrialBudget()
	_, _, _, err = boundarywalk.JunctionVertexAtReplay(ctx, segs, prev, next, cancelled) //nolint:dogsled // This test checks only the refusal and its error.
	require.ErrorIs(t, err, context.Canceled)
	w, r = cancelled.Usage()
	require.Zero(t, w)
	require.Zero(t, r)
	require.Zero(t, cancelled.Bytes())
}
