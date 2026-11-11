package motionengine

import (
	"errors"
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	vocab "github.com/lestrrat-3d/decad/report"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestNextRefinementMatchesFullScan(t *testing.T) {
	t.Parallel()
	tests := []struct {
		name   string
		cursor int
		setup  func(*Run, []*Pose) []Span
	}{
		{
			name:   "undecided span",
			cursor: 1,
			setup: func(_ *Run, _ []*Pose) []Span {
				return []Span{{outcome: vocab.IntervalClear}, {outcome: vocab.IntervalUndecided}, {outcome: vocab.IntervalUndecided}}
			},
		},
		{
			name:   "collision onset after collision at both ends",
			cursor: 0,
			setup: func(_ *Run, poses []*Pose) []Span {
				poses[0].Collisions = []Collision{{}}
				poses[1].Collisions = []Collision{{}}
				poses[3].Collisions = []Collision{{}}
				return []Span{{outcome: vocab.IntervalColliding}, {outcome: vocab.IntervalClear}, {outcome: vocab.IntervalColliding}}
			},
		},
		{
			name:   "collision at both ends is not split",
			cursor: 0,
			setup: func(_ *Run, poses []*Pose) []Span {
				poses[0].Collisions = []Collision{{}}
				poses[1].Collisions = []Collision{{}}
				return []Span{{outcome: vocab.IntervalColliding}, {outcome: vocab.IntervalClear}, {outcome: vocab.IntervalClear}}
			},
		},
		{
			name:   "two unbuildable endpoints are not split",
			cursor: 0,
			setup: func(_ *Run, poses []*Pose) []Span {
				poses[0].Unbuildable = errors.New("unbuildable")
				poses[1].Unbuildable = errors.New("unbuildable")
				return []Span{{outcome: vocab.IntervalUndecided}, {outcome: vocab.IntervalClear}, {outcome: vocab.IntervalClear}}
			},
		},
		{
			name:   "clearance ties keep traversal order",
			cursor: 2,
			setup: func(run *Run, _ []*Pose) []Span {
				run.Cfg.MinimumMM = big.NewRat(2, 1)
				return []Span{
					{outcome: vocab.IntervalClear, clearance: testClearance(1)},
					{outcome: vocab.IntervalClear, clearance: testClearance(1)},
					{outcome: vocab.IntervalClear, clearance: testClearance(1)},
				}
			},
		},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()
			run, poses := refinementFixture()
			spans := test.setup(run, poses)
			want, wantStep5 := nextRefinementFullScan(run, poses, spans)
			got, gotStep5 := run.nextRefinement(poses, spans, test.cursor)
			require.Equal(t, want, got)
			require.Equal(t, wantStep5, gotStep5)
		})
	}

	t.Run("step 6 split rewinds to an earlier step 5 candidate", func(t *testing.T) {
		t.Parallel()
		run, poses := refinementFixture()
		run.Cfg.ResolutionP.Base = big.NewRat(1, 16)
		run.Cfg.MinimumMM = big.NewRat(2, 1)
		spans := []Span{
			{outcome: vocab.IntervalClear, clearance: testClearance(1)},
			{outcome: vocab.IntervalClear, clearance: testClearance(1)},
			{outcome: vocab.IntervalClear, clearance: testClearance(1)},
			{outcome: vocab.IntervalClear, clearance: testClearance(1)},
		}
		cursor := 3
		want, wantStep5 := nextRefinementFullScan(run, poses, spans)
		got, gotStep5 := run.nextRefinement(poses, spans, cursor)
		require.Equal(t, want, got)
		require.False(t, wantStep5)
		require.Equal(t, wantStep5, gotStep5)

		cursor = got
		poses = slices.Insert(poses, got+1, refinementMidpoint(poses[got], poses[got+1]))
		spans[got] = Span{outcome: vocab.IntervalUndecided}
		spans = slices.Insert(spans, got+1, Span{outcome: vocab.IntervalClear})

		want, wantStep5 = nextRefinementFullScan(run, poses, spans)
		got, gotStep5 = run.nextRefinement(poses, spans, cursor)
		require.Equal(t, want, got)
		require.True(t, wantStep5)
		require.Equal(t, wantStep5, gotStep5)
	})

	t.Run("step 5 resumes at the split interval's left child", func(t *testing.T) {
		t.Parallel()
		run, poses := refinementFixture()
		run.Cfg.ResolutionP.Base = big.NewRat(1, 16)
		spans := []Span{{outcome: vocab.IntervalClear}, {outcome: vocab.IntervalUndecided}, {outcome: vocab.IntervalUndecided}, {outcome: vocab.IntervalClear}}
		want, wantStep5 := nextRefinementFullScan(run, poses, spans)
		got, gotStep5 := run.nextRefinement(poses, spans, 1)
		require.Equal(t, want, got)
		require.True(t, wantStep5)
		require.Equal(t, wantStep5, gotStep5)

		poses = slices.Insert(poses, got+1, refinementMidpoint(poses[got], poses[got+1]))
		spans[got] = Span{outcome: vocab.IntervalUndecided}
		spans = slices.Insert(spans, got+1, Span{outcome: vocab.IntervalUndecided})
		want, wantStep5 = nextRefinementFullScan(run, poses, spans)
		got, gotStep5 = run.nextRefinement(poses, spans, got)
		require.Equal(t, want, got)
		require.True(t, wantStep5)
		require.Equal(t, wantStep5, gotStep5)
	})

	t.Run("step 6 split after the cursor searches from the split", func(t *testing.T) {
		t.Parallel()
		run, poses := refinementFixture()
		run.Cfg.ResolutionP.Base = big.NewRat(1, 16)
		run.Cfg.MinimumMM = big.NewRat(2, 1)
		spans := []Span{
			{outcome: vocab.IntervalClear, clearance: testClearance(2)},
			{outcome: vocab.IntervalClear, clearance: testClearance(2)},
			{outcome: vocab.IntervalClear, clearance: testClearance(1)},
			{outcome: vocab.IntervalClear, clearance: testClearance(2)},
		}
		cursor := 1
		want, wantStep5 := nextRefinementFullScan(run, poses, spans)
		got, gotStep5 := run.nextRefinement(poses, spans, cursor)
		require.Equal(t, want, got)
		require.Equal(t, 2, got)
		require.False(t, wantStep5)
		require.Equal(t, wantStep5, gotStep5)

		cursor = got
		poses = slices.Insert(poses, got+1, refinementMidpoint(poses[got], poses[got+1]))
		spans[got] = Span{outcome: vocab.IntervalUndecided}
		spans = slices.Insert(spans, got+1, Span{outcome: vocab.IntervalClear})

		want, wantStep5 = nextRefinementFullScan(run, poses, spans)
		got, gotStep5 = run.nextRefinement(poses, spans, cursor)
		require.Equal(t, want, got)
		require.Equal(t, 2, got)
		require.True(t, wantStep5)
		require.Equal(t, wantStep5, gotStep5)
	})
}

func refinementFixture() (*Run, []*Pose) {
	const intervals = 4
	run := &Run{Cfg: Config{ResolutionP: motionbound.MotionParam{Turn: new(big.Rat), Base: big.NewRat(1, 8)}}}
	poses := make([]*Pose, intervals+1)
	for i := range poses {
		poses[i] = &Pose{Param: motionbound.MotionParam{Turn: new(big.Rat), Base: big.NewRat(int64(i), int64(intervals))}, F: new(big.Rat).SetFrac64(int64(i), int64(intervals))}
	}
	return run, poses
}

func refinementMidpoint(a, b *Pose) *Pose {
	f := new(big.Rat).Add(a.F, b.F)
	f.Quo(f, big.NewRat(2, 1))
	param := motionbound.MotionParam{
		Turn: new(big.Rat).Add(a.Param.Turn, b.Param.Turn),
		Base: new(big.Rat).Add(a.Param.Base, b.Param.Base),
	}
	param.Turn.Quo(param.Turn, big.NewRat(2, 1))
	param.Base.Quo(param.Base, big.NewRat(2, 1))
	return &Pose{F: f, Param: param}
}

func testClearance(mm float64) *vocab.Measurement {
	return &vocab.Measurement{Value: units.Millimeters(mm)}
}

func nextRefinementFullScan(r *Run, poses []*Pose, spans []Span) (int, bool) {
	allClear := true
	smallest := -1
	for k, span := range spans {
		if poses[k].Unbuildable != nil && poses[k+1].Unbuildable != nil {
			allClear = false
			continue
		}
		startHits, endHits := len(poses[k].Collisions) > 0, len(poses[k+1].Collisions) > 0
		onset := span.outcome == vocab.IntervalColliding && startHits != endHits
		if (span.outcome == vocab.IntervalUndecided || onset) && r.wide(poses[k], poses[k+1]) {
			return k, true
		}
		if span.outcome != vocab.IntervalClear {
			allClear = false
		}
		if span.clearance != nil && (smallest < 0 || span.clearance.Value.Base() < spans[smallest].clearance.Value.Base()) {
			smallest = k
		}
	}
	if smallest < 0 {
		return -1, false
	}
	a, b := poses[smallest], poses[smallest+1]
	if allClear && r.wideForReading(a, b) {
		if reading, _ := r.PathClearance(poses, spans[smallest].clearance, "whole-path"); reading != nil && reading.Tolerance.State != vocab.ToleranceSatisfied {
			return smallest, false
		}
	}
	if r.Cfg.MinimumMM != nil && r.wide(a, b) && !anyViolated(poses) && !r.MeetsMinimum(spans[smallest].clearance) {
		return smallest, false
	}
	return -1, false
}
