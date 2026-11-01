package proofbound

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

func TestCircleReplayBudgetRefusals(t *testing.T) {
	ctx := t.Context()
	for _, kind := range []string{replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		t.Run(kind, func(t *testing.T) {
			b := NewCircleReplayBudget()
			work := b.WorkBudget(ctx)
			_, initialRecords := b.Usage()
			initialBytes := b.Bytes()
			var charge func() error
			switch kind {
			case replayBudgetTestWork:
				if err := b.SpendWork(ctx, ReplayTrialWork); err != nil {
					t.Fatal(err)
				}
				charge = work.Step
			case replayBudgetTestRecords:
				if err := b.ReserveRecords(ctx, ReplayTrialRecords-initialRecords); err != nil {
					t.Fatal(err)
				}
				charge = func() error { return work.Reserve(1) }
			case replayBudgetTestBytes:
				if err := b.ReserveBytes(ctx, ReplayTrialBytes-initialBytes); err != nil {
					t.Fatal(err)
				}
				charge = func() error { return work.ReserveBytes(1) }
			}
			var got error
			allocs := testing.AllocsPerRun(10, func() { got = charge() })
			var detail *WorkLimitError
			want := map[string]string{replayBudgetTestWork: "multiway_work", replayBudgetTestRecords: "multiway_records", replayBudgetTestBytes: "replay_bytes"}[kind]
			if allocs != 0 || !errors.Is(got, decaderr.ErrUnsupported) || !errors.As(got, &detail) || detail.Resource != want {
				t.Fatalf("refusal allocs=%g resource=%s error=%v", allocs, want, got)
			}
			if err := b.ReserveRecords(ctx, 0); err != got {
				t.Fatal("failed meter resumed")
			}
			cancelled, cancel := context.WithCancel(ctx)
			cancel()
			if err := b.SpendWork(cancelled, 0); !errors.Is(err, context.Canceled) {
				t.Fatalf("cancellation: %v", err)
			}
		})
	}
	b := NewCircleReplayBudget()
	allocs := testing.AllocsPerRun(10, func() { _ = b.ReserveArray(ctx, ^uint64(0), 2) })
	if allocs != 0 {
		t.Fatalf("overflow allocated: %g", allocs)
	}
	if err := b.SpendWork(ctx, 0); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatal("overflow did not close meter")
	}
}
