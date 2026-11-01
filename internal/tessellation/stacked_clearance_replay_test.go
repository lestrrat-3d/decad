package tessellation

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

func TestReplayStackedSlabClearance(t *testing.T) {
	square := func(x float64) StackedRing[int] {
		return StackedRing[int]{Samples: []sectionrecord.Point2{
			{U: x, V: 0}, {U: x + 1, V: 0},
			{U: x + 1, V: 1}, {U: x, V: 1},
		}, Sag: .01}
	}
	rings := []StackedRing[int]{square(0), square(3)}
	ctx := t.Context()
	if err := ReplayStackedSlabClearance(ctx, rings, []int{0, 1}, proofbound.NewReplayTrialBudget()); err != nil {
		t.Fatal(err)
	}
	rings[1] = square(0)
	err := ReplayStackedSlabClearance(ctx, rings, []int{0, 1}, proofbound.NewReplayTrialBudget())
	if !errors.Is(err, decaderr.ErrDegenerate) {
		t.Fatalf("overlap error=%v", err)
	}
}

func TestReplayStackedSlabClearanceScanBudget(t *testing.T) {
	rings := make([]StackedRing[int], 7)
	columns := make([]int, len(rings))
	for i := range rings {
		x := float64(3 * i)
		rings[i].Samples = []sectionrecord.Point2{{U: x}, {U: x + 1}, {U: x, V: 1}}
		columns[i] = i
	}
	baseline := proofbound.NewReplayTrialBudget()
	if err := ReplayStackedSlabClearance(t.Context(), rings, columns, baseline); err != nil {
		t.Fatal(err)
	}
	work, records := baseline.Usage()
	bytes := baseline.Bytes()
	const points, pairs, scans = 21, 21 * 9, 21
	wantWork := uint64(len(columns) + scans + 64*points + 256*pairs + 64*len(columns))
	if work != wantWork {
		t.Fatalf("work=%d want=%d", work, wantWork)
	}
	for _, tc := range []struct {
		name string
		work uint64
		pass bool
	}{{replayTestCaseExact, work, true}, {"one short", work - 1, false}} {
		t.Run(tc.name, func(t *testing.T) {
			budget, err := proofbound.NewReplayBudgetLimits(tc.work, records, bytes)
			if err != nil {
				t.Fatal(err)
			}
			err = ReplayStackedSlabClearance(t.Context(), rings, columns, budget)
			if (err == nil) != tc.pass || (!tc.pass && !errors.Is(err, decaderr.ErrUnsupported)) {
				t.Fatalf("error=%v", err)
			}
		})
	}
	budget, err := proofbound.NewReplayBudgetLimits(3, records, bytes)
	if err != nil {
		t.Fatal(err)
	}
	if err := ReplayStackedSlabClearance(t.Context(), rings, columns, budget); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("scan budget error=%v", err)
	}
	overflow := proofbound.NewReplayTrialBudget()
	overflow.MultiwayBudget().Work = math.MaxUint64
	if err := ReplayStackedSlabClearance(t.Context(), rings, columns, overflow); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("overflow error=%v", err)
	}
	base, cancel := context.WithCancel(t.Context())
	defer cancel()
	late := &replayCloseCancel{Context: base, cancel: cancel}
	if err := ReplayStackedSlabClearance(late, rings, columns, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("scan cancellation error=%v", err)
	}
}
