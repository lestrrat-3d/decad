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
		b := proofbound.NewBrakeReplayBudget()
		remaining := uint64(19)
		if short {
			remaining--
		}
		if err := b.SpendWork(ctx, proofbound.BrakeReplayWork-remaining); err != nil {
			t.Fatal(err)
		}
		err := (replayExactMeter{b.WorkBudget(ctx)}).SpendWork(ctx, 19)
		w, r := b.Usage()
		if short {
			var limit *proofbound.WorkLimitError
			if !errors.As(err, &limit) || limit.Resource != "multiway_work" || w != proofbound.BrakeReplayWork-remaining {
				t.Fatalf("atomic refusal work=%d err=%v", w, err)
			}
		} else if err != nil || w != proofbound.BrakeReplayWork {
			t.Fatalf("exact work=%d err=%v", w, err)
		}
		if r != 0 || b.Bytes() != 0 {
			t.Fatal("logical work changed allocation counters")
		}
	}
	ctx, cancel := context.WithCancel(ctx)
	cancel()
	b := proofbound.NewBrakeReplayBudget()
	err := (replayExactMeter{b.WorkBudget(ctx)}).SpendWork(ctx, 19)
	w, r := b.Usage()
	if !errors.Is(err, context.Canceled) || w != 0 || r != 0 || b.Bytes() != 0 {
		t.Fatalf("cancellation work=%d err=%v", w, err)
	}
}
