package facetproof

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplayExactMeterAtomicWork(t *testing.T) {
	ctx := t.Context()
	for _, short := range []bool{false, true} {
		b := proofbound.NewReplayTrialBudget()
		remaining := uint64(19)
		if short {
			remaining--
		}
		if err := b.SpendWork(ctx, proofbound.ReplayTrialWork-remaining); err != nil {
			t.Fatal(err)
		}
		workBudget := b.WorkBudget(ctx)
		workBudget.SpendWorkFn = func(n uint64) error { return b.SpendWork(ctx, n) }
		err := (replayExactMeter{workBudget}).SpendWork(ctx, 19)
		w, r := b.Usage()
		if short {
			var limit *proofbound.WorkLimitError
			matched := errors.As(err, &limit)
			if !matched || limit.Resource != "multiway_work" || w != proofbound.ReplayTrialWork-remaining {
				t.Fatalf("atomic refusal work=%d matched=%v resource=%q observed=%d maximum=%d err=%v", w, matched, limit.Resource, limit.Observed, limit.Maximum, err)
			}
		} else if err != nil || w != proofbound.ReplayTrialWork {
			t.Fatalf("exact work=%d err=%v", w, err)
		}
		if r != 0 || b.Bytes() != 0 {
			t.Fatal("logical work changed allocation counters")
		}
	}
	ctx, cancel := context.WithCancel(ctx)
	cancel()
	b := proofbound.NewReplayTrialBudget()
	workBudget := b.WorkBudget(ctx)
	workBudget.SpendWorkFn = func(n uint64) error { return b.SpendWork(ctx, n) }
	err := (replayExactMeter{workBudget}).SpendWork(ctx, 19)
	w, r := b.Usage()
	if !errors.Is(err, context.Canceled) || w != 0 || r != 0 || b.Bytes() != 0 {
		t.Fatalf("cancellation work=%d err=%v", w, err)
	}
}
