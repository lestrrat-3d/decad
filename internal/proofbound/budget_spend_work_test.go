package proofbound

import (
	"context"
	"errors"
	"testing"
)

func TestWorkBudgetLogicalChargeKeepsStepNBehavior(t *testing.T) {
	stop := errors.New("step stopped")
	for _, atomic := range []bool{false, true} {
		calls, bulkCalls := 0, 0
		b := &WorkBudget{StepFn: func() error {
			calls++
			if calls == 3 {
				return stop
			}
			return nil
		}}
		if atomic {
			b.SpendWorkFn = func(uint64) error { bulkCalls++; return nil }
		}
		done, err := b.StepN(5)
		if done != 2 || !errors.Is(err, stop) || calls != 3 || bulkCalls != 0 {
			t.Fatalf("atomic=%t done=%d calls=%d bulk=%d err=%v", atomic, done, calls, bulkCalls, err)
		}
		calls = 0
		err = b.SpendWork(5)
		if atomic {
			if err != nil || calls != 0 || bulkCalls != 1 {
				t.Fatalf("atomic callback: calls=%d bulk=%d err=%v", calls, bulkCalls, err)
			}
		} else if !errors.Is(err, stop) || calls != 3 {
			t.Fatalf("fallback prefix: calls=%d err=%v", calls, err)
		}
	}
	if NewReplayTrialBudget().WorkBudget(t.Context()).SpendWorkFn != nil {
		t.Fatal("ordinary trial changed its callback behavior")
	}
}

func TestReplayAreaMeterAtomicLimitsAndCancellation(t *testing.T) {
	ctx := t.Context()
	for _, short := range []bool{false, true} {
		b := NewBrakeReplayBudget()
		remaining := uint64(7)
		if short {
			remaining--
		}
		if err := b.SpendWork(ctx, BrakeReplayWork-remaining); err != nil {
			t.Fatal(err)
		}
		m := replayAreaMeter{b.WorkBudget(ctx)}
		err := m.SpendWork(ctx, 7)
		work, records := b.Usage()
		if short {
			var limit *WorkLimitError
			if !errors.As(err, &limit) || limit.Resource != "multiway_work" || limit.Observed != BrakeReplayWork+1 || work != BrakeReplayWork-remaining {
				t.Fatalf("one-short work=%d err=%v", work, err)
			}
		} else if err != nil || work != BrakeReplayWork {
			t.Fatalf("exact work=%d err=%v", work, err)
		}
		if records != 0 || b.Bytes() != 0 {
			t.Fatal("work charge changed allocation counters")
		}
	}
	ctx, cancel := context.WithCancel(ctx)
	cancel()
	b := NewBrakeReplayBudget()
	err := (replayAreaMeter{b.WorkBudget(ctx)}).SpendWork(ctx, 7)
	w, r := b.Usage()
	if !errors.Is(err, context.Canceled) || w != 0 || r != 0 || b.Bytes() != 0 {
		t.Fatalf("cancellation work=%d records=%d bytes=%d err=%v", w, r, b.Bytes(), err)
	}
}
