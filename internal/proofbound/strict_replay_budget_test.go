package proofbound

import (
	"context"
	"errors"
	"testing"
)

func TestStrictReplayBudgetBundle(t *testing.T) {
	ctx := t.Context()
	for _, resource := range []string{replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		t.Run(resource, func(t *testing.T) {
			b := NewStrictReplayBudget()
			if !b.IsStrictReplay() || b.IsCircleReplay() {
				t.Fatal("generic meter has wrong scope")
			}
			_, initialRecords := b.Usage()
			initialBytes := b.Bytes()
			work, records, bytes := ReplayTrialWork-2, ReplayTrialRecords-initialRecords-2, ReplayTrialBytes-initialBytes-2
			switch resource {
			case replayBudgetTestWork:
				work++
			case replayBudgetTestRecords:
				records++
			case replayBudgetTestBytes:
				bytes++
			}
			if err := b.ReserveBundle(ctx, work, records, bytes); err != nil {
				t.Fatal(err)
			}
			w0, r0 := b.Usage()
			z0 := b.Bytes()
			err := b.ReserveBundle(ctx, 2, 2, 2)
			if _, ok := errors.AsType[*WorkLimitError](err); !ok {
				t.Fatalf("missing limit: %v", err)
			}
			w1, r1 := b.Usage()
			if w1 != w0 || r1 != r0 || b.Bytes() != z0 {
				t.Fatal("failed bundle charged counters")
			}
			if got := b.SpendWork(ctx, 0); got != err {
				t.Fatal("failure was not sticky")
			}
		})
	}
	b := NewStrictReplayBudget()
	_, r := b.Usage()
	if err := b.ReserveBundle(ctx, ReplayTrialWork, ReplayTrialRecords-r, ReplayTrialBytes-b.Bytes()); err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(ctx)
	cancel()
	if err := b.ReserveBundle(ctx, 0, 0, 0); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel: %v", err)
	}
	circle := NewCircleReplayBudget()
	if !circle.IsCircleReplay() || !circle.IsStrictReplay() {
		t.Fatal("circle meter lost scope")
	}
	if err := circle.ReserveBundle(t.Context(), 0, 0, 0); err == nil {
		t.Fatal("circle bundle guard changed")
	}
}

//nolint:containedctx // The context is confined to one synchronous reservation.
type bundleReentryContext struct {
	context.Context
	call   func()
	called bool
}

func (c *bundleReentryContext) Err() error {
	if !c.called {
		c.called = true
		c.call()
	}
	return nil
}

func TestReplayBundlePreservesContextReentry(t *testing.T) {
	for _, strict := range []bool{false, true} {
		for _, resource := range []string{replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
			t.Run(resource+map[bool]string{false: "_ordinary", true: "_strict"}[strict], func(t *testing.T) {
				b := NewReplayTrialBudget()
				if strict {
					b = NewStrictReplayBudget()
				}
				_, initialRecords := b.Usage()
				initialBytes := b.Bytes()
				var nested func() error
				var addWork, addRecords, addBytes uint64
				switch resource {
				case replayBudgetTestWork:
					if err := b.SpendWork(t.Context(), ReplayTrialWork-10); err != nil {
						t.Fatal(err)
					}
					nested = func() error { return b.SpendWork(context.Background(), 8) }
					addWork = 5
				case replayBudgetTestRecords:
					if err := b.ReserveRecords(t.Context(), ReplayTrialRecords-initialRecords-10); err != nil {
						t.Fatal(err)
					}
					nested = func() error { return b.ReserveRecords(context.Background(), 8) }
					addRecords = 5
				case replayBudgetTestBytes:
					if err := b.ReserveBytes(t.Context(), ReplayTrialBytes-initialBytes-10); err != nil {
						t.Fatal(err)
					}
					nested = func() error { return b.ReserveBytes(context.Background(), 8) }
					addBytes = 5
				}
				var inner error
				ctx := &bundleReentryContext{Context: t.Context(), call: func() { inner = nested() }}
				err := b.ReserveBundle(ctx, addWork, addRecords, addBytes)
				if inner != nil || err == nil {
					t.Fatalf("inner=%v outer=%v", inner, err)
				}
				w, r := b.Usage()
				z := b.Bytes()
				switch resource {
				case replayBudgetTestWork:
					if w != ReplayTrialWork-2 {
						t.Fatalf("work=%d", w)
					}
				case replayBudgetTestRecords:
					if r != ReplayTrialRecords-2 {
						t.Fatalf("records=%d", r)
					}
				case replayBudgetTestBytes:
					if z != ReplayTrialBytes-2 {
						t.Fatalf("bytes=%d", z)
					}
				}
				if strict && b.SpendWork(context.Background(), 0) != err {
					t.Fatal("failure was not sticky")
				}
			})
		}
	}
}

//nolint:containedctx // The context is confined to one synchronous reservation.
type bundleLateCancelContext struct {
	context.Context
	polls int
}

func (c *bundleLateCancelContext) Err() error {
	c.polls++
	if c.polls == 3 {
		return context.Canceled
	}
	return nil
}

func TestReplayBundleObservesLaterCancellation(t *testing.T) {
	for _, strict := range []bool{false, true} {
		b := NewReplayTrialBudget()
		if strict {
			b = NewStrictReplayBudget()
		}
		w0, r0 := b.Usage()
		z0 := b.Bytes()
		ctx := &bundleLateCancelContext{Context: t.Context()}
		if err := b.ReserveBundle(ctx, 1, 2, 3); !errors.Is(err, context.Canceled) {
			t.Fatalf("strict=%t error=%v", strict, err)
		}
		w, r := b.Usage()
		if w != w0 || r != r0 || b.Bytes() != z0 || ctx.polls != 3 {
			t.Fatalf("strict=%t work=%d records=%d bytes=%d polls=%d", strict, w, r, b.Bytes(), ctx.polls)
		}
	}
}

//nolint:containedctx // The context is confined to one synchronous reservation.
type bundleDoneContext struct {
	context.Context
	done  chan struct{}
	polls int
	meter *ReplayBudget
}

func (c *bundleDoneContext) Done() <-chan struct{} { return c.done }
func (c *bundleDoneContext) Err() error {
	c.polls++
	if c.polls > 4 {
		_ = c.meter.SpendWork(context.Background(), 1)
		return context.Canceled
	}
	return nil
}

func TestReplayBundleObservesDoneBeforeCommit(t *testing.T) {
	for _, strict := range []bool{false, true} {
		b := NewReplayTrialBudget()
		if strict {
			b = NewStrictReplayBudget()
		}
		w0, r0 := b.Usage()
		z0 := b.Bytes()
		ctx := &bundleDoneContext{Context: t.Context(), done: make(chan struct{}), meter: b}
		close(ctx.done)
		if err := b.ReserveBundle(ctx, 2, 3, 4); !errors.Is(err, context.Canceled) {
			t.Fatalf("strict=%t err=%v", strict, err)
		}
		w, r := b.Usage()
		if w != w0+1 || r != r0 || b.Bytes() != z0 || ctx.polls != 5 {
			t.Fatalf("strict=%t work=%d records=%d bytes=%d polls=%d", strict, w, r, b.Bytes(), ctx.polls)
		}
	}
}
