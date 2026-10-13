package offset2d_test

import (
	"context"
	"errors"
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// draftLoop is a closed line loop through pts, one recorded segment per side.
func draftLoop(pts ...[2]float64) sectionrecord.LoopRecord {
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

func draftSquare() momentinput.Profile {
	return momentinput.Profile{Outer: draftLoop([2]float64{0, 0}, [2]float64{4, 0}, [2]float64{4, 4}, [2]float64{0, 4})}
}

func draftSplitSquare() momentinput.Profile {
	return momentinput.Profile{Outer: draftLoop(
		[2]float64{0, 0}, [2]float64{2, 0}, [2]float64{4, 0}, [2]float64{4, 4}, [2]float64{0, 4})}
}

func segStarts(t *testing.T, loop sectionrecord.LoopRecord) [][2]float64 {
	t.Helper()
	out := make([][2]float64, len(loop.Segments))
	for i, s := range loop.Segments {
		ls, ok := s.(sectionrecord.LineSeg)
		require.True(t, ok)
		out[i] = [2]float64{ls.Start.U, ls.Start.V}
	}
	return out
}

func TestDraftWalkAmountsKeepsFirstSegment(t *testing.T) {
	walks := []survey2d.SideWalk{{Segs: []int{0}}, {Segs: []int{2, 3}}, {Segs: []int{1}}}
	kept := map[offset2d.DraftWall]struct{}{{Loop: 0, Seg: 2}: {}}
	require.Equal(t, []float64{0.5, 0, 0.5}, offset2d.DraftWalkAmounts(kept, 0, walks, 0.5))
	require.Equal(t, []float64{0.5, 0.5, 0.5}, offset2d.DraftWalkAmounts(kept, 1, walks, 0.5))
	require.Equal(t, []float64{0.5, 0.5, 0.5}, offset2d.DraftWalkAmounts(nil, 0, walks, 0.5))
}

func TestDraftSectionStages(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	work := freeform.NewFreeformWork()
	walks, err := offset2d.DraftSectionWalks(budget, draftSquare(), nil, work, 1e-9)
	require.NoError(t, err)
	require.Len(t, walks, 1)
	require.Len(t, walks[0], 4)

	far, err := offset2d.DraftFarSection(budget, walks, nil, 0.5, 1e-9)
	require.NoError(t, err)
	require.Equal(t, [][2]float64{{0.5, 0.5}, {3.5, 0.5}, {3.5, 3.5}, {0.5, 3.5}}, segStarts(t, far.Outer))

	kept := map[offset2d.DraftWall]struct{}{{Loop: 0, Seg: 0}: {}}
	keptWalks, err := offset2d.DraftSectionWalks(budget, draftSquare(), kept, work, 1e-9)
	require.NoError(t, err)
	far, err = offset2d.DraftFarSection(budget, keptWalks, kept, 0.5, 1e-9)
	require.NoError(t, err)
	require.Equal(t, [][2]float64{{0.5, 0}, {3.5, 0}, {3.5, 3.5}, {0.5, 3.5}}, segStarts(t, far.Outer))

	_, err = offset2d.DraftFarSection(budget, walks, nil, 0, 1e-9)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Contains(t, err.Error(), "(draft SD13)")

	_, err = offset2d.DraftFarSection(budget, walks, nil, 3, 1e-9)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.Contains(t, err.Error(), "(draft SD6)")

	_, err = offset2d.DraftSectionWalks(budget, draftSplitSquare(), kept, work, 1e-9)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Contains(t, err.Error(), "recorded segments [0 1] form one face")
}

func TestDraftSharpJoins(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	walks, err := offset2d.DraftSectionWalks(budget, draftSquare(), nil, freeform.NewFreeformWork(), 1e-9)
	require.NoError(t, err)
	joins, err := offset2d.DraftSharpJoins(budget, walks[0], offset2d.UniformAmounts(4, 0.5), 1e-9)
	require.NoError(t, err)
	require.Len(t, joins, 4)
	require.Equal(t, 0.5, joins[0].M.U)
	require.Equal(t, 0.5, joins[0].M.V)
}

func TestDraftOffsetErrorRows(t *testing.T) {
	for _, tc := range []struct {
		in   error
		text string
	}{
		{offset2d.ErrMixedCircularCorner, "select both walls or neither"},
		{offset2d.ErrCircularMiter, "join the wall tangentially"},
		{offset2d.ErrLoopConsumed, "(draft SD7)"},
	} {
		err := offset2d.DraftOffsetError(tc.in)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, tc.text)
		require.Contains(t, err.Error(), tc.text)
	}
	other := errors.New("x")
	require.True(t, offset2d.DraftOffsetError(other) == other)
}

func TestDraftAuditError(t *testing.T) {
	other := errors.New("x")
	wrapped := offset2d.DraftAuditError(other)
	require.ErrorIs(t, wrapped, other)
	require.Contains(t, wrapped.Error(), "tapered extrude's far section")
	cancelled := fmt.Errorf("w: %w", context.Canceled)
	require.True(t, offset2d.DraftAuditError(cancelled) == cancelled)
	late := fmt.Errorf("w: %w", context.DeadlineExceeded)
	require.True(t, offset2d.DraftAuditError(late) == late)
}
