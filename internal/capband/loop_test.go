package capband_test

import (
	"context"
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// lineLoop is a closed line loop through pts, one recorded segment per side.
func lineLoop(pts ...[2]float64) sectionrecord.LoopRecord {
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		segs[i] = sectionrecord.LineSeg{
			Start:  sectionrecord.Point2{U: p[0], V: p[1]},
			End:    sectionrecord.Point2{U: q[0], V: q[1]},
			TStart: 0, TEnd: 1,
		}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

func squareLoop() sectionrecord.LoopRecord {
	return lineLoop([2]float64{0, 0}, [2]float64{4, 0}, [2]float64{4, 4}, [2]float64{0, 4})
}

func splitSquareLoop() sectionrecord.LoopRecord {
	return lineLoop([2]float64{0, 0}, [2]float64{2, 0}, [2]float64{4, 0}, [2]float64{4, 4}, [2]float64{0, 4})
}

func circleLoop() sectionrecord.LoopRecord {
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{
		Center: sectionrecord.Point2{}, Radius: units.Millimeters(2), CCW: true, TStart: 0, TEnd: 1,
	}}}
}

func loopWalks(t *testing.T, loop sectionrecord.LoopRecord) (*proofbound.WorkBudget, []survey2d.SideWalk) {
	t.Helper()
	budget := proofbound.NewWorkBudget(t.Context())
	walks, err := capband.LoopWalks(budget, loop, freeform.NewFreeformWork())
	require.NoError(t, err)
	return budget, walks
}

func TestLoopWalksCoalesces(t *testing.T) {
	_, walks := loopWalks(t, squareLoop())
	require.Len(t, walks, 4)
	_, walks = loopWalks(t, splitSquareLoop())
	require.Len(t, walks, 4)
	require.Equal(t, []int{0, 1}, walks[0].Segs)
}

func TestOffsetJoinsAndBoundary(t *testing.T) {
	budget, walks := loopWalks(t, squareLoop())
	_, err := capband.OffsetJoins(budget, nil, 0.5, 1e-9)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.Contains(t, err.Error(), "a cap loop holds no walks")

	joins, err := capband.OffsetJoins(budget, walks, 0.5, 1e-9)
	require.NoError(t, err)
	require.Len(t, joins, 4)
	require.Equal(t, sectionrecord.Point2{U: 0.5, V: 0.5}, joins[0].M)

	_, circle := loopWalks(t, circleLoop())
	_, err = capband.OffsetJoins(budget, circle, 3, 1e-9)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Contains(t, err.Error(), "drops a section feature")

	boundary, err := capband.LoopBoundary(t.Context(), squareLoop(), 0.5, 1e-9)
	require.NoError(t, err)
	want := [][2]float64{{0.5, 0.5}, {3.5, 0.5}, {3.5, 3.5}, {0.5, 3.5}}
	require.Len(t, boundary.Segments, len(want))
	for i, s := range boundary.Segments {
		ls, ok := s.(sectionrecord.LineSeg)
		require.True(t, ok)
		next := want[(i+1)%len(want)]
		require.Equal(t, sectionrecord.Point2{U: want[i][0], V: want[i][1]}, ls.Start)
		require.Equal(t, sectionrecord.Point2{U: next[0], V: next[1]}, ls.End)
	}
}

func TestContourJoinsCopiesEveryField(t *testing.T) {
	in := []offset2d.Join{{
		Arc: true, G1: true, VertU: 1, VertV: 2,
		M:  sectionrecord.Point2{U: 3, V: 4},
		PA: sectionrecord.Point2{U: 5, V: 6},
		PB: sectionrecord.Point2{U: 7, V: 8},
	}}
	out := capband.ContourJoins(in)
	require.Len(t, out, 1)
	require.True(t, out[0].Arc)
	require.True(t, out[0].G1)
	require.Equal(t, 1.0, out[0].VU)
	require.Equal(t, 2.0, out[0].VV)
	require.Equal(t, in[0].M, out[0].M)
	require.Equal(t, in[0].PA, out[0].PA)
	require.Equal(t, in[0].PB, out[0].PB)
}

func TestLoopContourDeltaRoutes(t *testing.T) {
	budget, walks := loopWalks(t, squareLoop())
	joins, err := capband.OffsetJoins(budget, walks, 0.5, 1e-9)
	require.NoError(t, err)
	wantSquare, err := capband.ContourDisplacement(walks, capband.ContourJoins(joins), 0.5, 0.001, 1e-9)
	require.NoError(t, err)
	gotSquare, err := capband.LoopContourDelta(t.Context(), squareLoop(), 0.5, 0.001, 1e-9)
	require.NoError(t, err)
	require.Equal(t, wantSquare, gotSquare)

	_, circle := loopWalks(t, circleLoop())
	wantCircle, err := capband.WholeCircleDisplacement(circle[0], 0.5, 0.001, 1e-9)
	require.NoError(t, err)
	gotCircle, err := capband.LoopContourDelta(t.Context(), circleLoop(), 0.5, 0.001, 1e-9)
	require.NoError(t, err)
	require.Equal(t, wantCircle, gotCircle)
}

func TestChamferErrors(t *testing.T) {
	drop := capband.ChamferDropError(fmt.Errorf("%w", offset2d.ErrDrop))
	require.ErrorIs(t, drop, decaderr.ErrDegenerate)
	require.NotErrorIs(t, drop, offset2d.ErrDrop)
	require.NotErrorIs(t, drop, decaderr.ErrUnsupported)
	other := fmt.Errorf("other")
	require.True(t, capband.ChamferDropError(other) == other)

	require.True(t, capband.ChamferAuditError(fmt.Errorf("w: %w", context.Canceled)) == context.Canceled)
	require.True(t, capband.ChamferAuditError(fmt.Errorf("w: %w", context.DeadlineExceeded)) == context.DeadlineExceeded)
	degenerate := capband.ChamferAuditError(fmt.Errorf("w: %w", decaderr.ErrDegenerate))
	require.ErrorIs(t, degenerate, decaderr.ErrDegenerate)
	require.Contains(t, degenerate.Error(), "the cap-loop chamfer's own offset")
	generic := capband.ChamferAuditError(other)
	require.ErrorIs(t, generic, decaderr.ErrUnsupported)
	require.Contains(t, generic.Error(), "a trimming kernel is not available")
}
