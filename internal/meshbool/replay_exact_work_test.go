package meshbool

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplayExactWorkAtomicPairCounters(t *testing.T) {
	ctx := t.Context()
	for _, short := range []bool{false, true} {
		meter := proofbound.NewBrakeReplayBudget()
		remaining := uint64(23)
		if short {
			remaining--
		}
		if err := meter.SpendWork(ctx, proofbound.BrakeReplayWork-remaining); err != nil {
			t.Fatal(err)
		}
		pair := &PairCutsBudget{replay: meter}
		work, err := newBrakeParityWork(ctx, pair)
		if err != nil {
			t.Fatal(err)
		}
		_, records := meter.Usage()
		bytes := meter.Bytes()
		err = (replayExactWork{work}).SpendWork(ctx, 23)
		w, gotRecords := meter.Usage()
		if short {
			var limit *proofbound.WorkLimitError
			if !errors.As(err, &limit) || limit.Resource != "multiway_work" || pair.Work != 0 || w != proofbound.BrakeReplayWork-remaining {
				t.Fatalf("one-short global=%d local=%d err=%v", w, pair.Work, err)
			}
		} else if err != nil || w != proofbound.BrakeReplayWork || pair.Work != 23 {
			t.Fatalf("exact global=%d local=%d err=%v", w, pair.Work, err)
		}
		if gotRecords != records || meter.Bytes() != bytes {
			t.Fatal("logical work changed allocation counters")
		}
	}
	ctx, cancel := context.WithCancel(ctx)
	meter := proofbound.NewBrakeReplayBudget()
	pair := &PairCutsBudget{replay: meter}
	work, err := newBrakeParityWork(ctx, pair)
	if err != nil {
		t.Fatal(err)
	}
	cancel()
	err = (replayExactWork{work}).SpendWork(ctx, 23)
	w, _ := meter.Usage()
	if !errors.Is(err, context.Canceled) || w != 0 || pair.Work != 0 {
		t.Fatalf("cancellation global=%d local=%d err=%v", w, pair.Work, err)
	}
}
