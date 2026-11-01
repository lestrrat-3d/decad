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
	base := NewCircleReplayBudget()
	_, initialRecords := base.Usage()
	initialBytes := base.Bytes()
	diagnostic := func(b *ReplayBudget) error {
		return ReplayIndexedRefusal(ctx, b, "facets ", " and ", " overlap", -1, 23, true)
	}
	want := decaderr.ErrUnsupported.Error() + ": facets -1 and 23 overlap"
	if err := diagnostic(base); err.Error() != want {
		t.Fatalf("diagnostic: %v", err)
	}
	work, records := base.Usage()
	records -= initialRecords
	usedBytes := base.Bytes() - initialBytes
	for _, mode := range []string{replayBudgetTestExact, replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		t.Run("diagnostic_"+mode, func(t *testing.T) {
			b := NewCircleReplayBudget()
			_, initialRecords := b.Usage()
			initialBytes := b.Bytes()
			w, r, z := ReplayTrialWork-work, ReplayTrialRecords-initialRecords-records, ReplayTrialBytes-initialBytes-usedBytes
			switch mode {
			case replayBudgetTestWork:
				w++
			case replayBudgetTestRecords:
				r++
			case replayBudgetTestBytes:
				z++
			}
			if err := b.SpendWork(ctx, w); err != nil {
				t.Fatal(err)
			}
			if err := b.ReserveRecords(ctx, r); err != nil {
				t.Fatal(err)
			}
			if err := b.ReserveBytes(ctx, z); err != nil {
				t.Fatal(err)
			}
			err := diagnostic(b)
			if mode == replayBudgetTestExact && err.Error() != want {
				t.Fatalf("exact diagnostic: %v", err)
			}
			if mode != replayBudgetTestExact {
				if _, ok := errors.AsType[*WorkLimitError](err); !ok {
					t.Fatalf("one-short diagnostic %s: %v", mode, err)
				}
			}
		})
	}
	cancelled, cancel := context.WithCancel(ctx)
	cancel()
	if err := ReplayIndexedRefusal(cancelled, NewCircleReplayBudget(), "", "", "", 0, 0, false); !errors.Is(err, context.Canceled) {
		t.Fatalf("diagnostic cancellation: %v", err)
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
