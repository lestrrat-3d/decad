package proofbound

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

func TestReplayBudgetSpansSteps(t *testing.T) {
	ctx := t.Context()
	var meter ReplayBudget
	if err := meter.SpendWork(ctx, MaxMultiwayWorkPerCall-2); err != nil {
		t.Fatal(err)
	}
	if err := meter.ReserveRecords(ctx, MaxMultiwayRecordsPerCall-1); err != nil {
		t.Fatal(err)
	}
	// A later operation receives an adapter to the same counters.
	later := meter.WorkBudget(ctx)
	if _, err := later.StepN(2); err != nil {
		t.Fatal(err)
	}
	if err := later.Reserve(1); err != nil {
		t.Fatal(err)
	}
	for _, run := range []func() error{later.Step, func() error { return later.Reserve(1) }} {
		if err := run(); !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("expected budget refusal, got %v", err)
		}
	}
	work, records := meter.Usage()
	if work != MaxMultiwayWorkPerCall || records != MaxMultiwayRecordsPerCall {
		t.Fatalf("usage after refused charges: work=%d records=%d", work, records)
	}
}

func TestReplayBudgetOverflowAndCancellation(t *testing.T) {
	ctx := t.Context()
	meter := ReplayBudget{shared: MultiwayBudget{Work: ^uint64(0), Records: ^uint64(0)}}
	if err := meter.SpendWork(ctx, 1); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("overflowing work: %v", err)
	}
	if err := meter.ReserveRecords(ctx, 1); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("overflowing records: %v", err)
	}
	work, records := meter.Usage()
	if work != ^uint64(0) || records != ^uint64(0) {
		t.Fatalf("overflow changed usage: work=%d records=%d", work, records)
	}
	cancelled, cancel := context.WithCancel(ctx)
	cancel()
	if err := meter.SpendWork(cancelled, 0); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancelled work: %v", err)
	}
	if err := meter.ReserveRecords(cancelled, 0); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancelled reservation: %v", err)
	}
}

func TestReplayTrialLimitsAndBytes(t *testing.T) {
	ctx := t.Context()
	meter := NewReplayTrialBudget()
	if err := meter.WorkBudget(ctx).Reserve(ReplayTrialRecords); err != nil {
		t.Fatal(err)
	}
	if err := meter.MultiwayBudget().SpendWork(ctx, ReplayTrialWork); err != nil {
		t.Fatal(err)
	}
	if err := meter.WorkBudget(ctx).ReserveBytes(ReplayTrialBytes); err != nil {
		t.Fatal(err)
	}
	for _, charge := range []func() error{
		func() error { return meter.ReserveRecords(ctx, 1) },
		func() error { return meter.SpendWork(ctx, 1) },
		func() error { return meter.ReserveBytes(ctx, 1) },
	} {
		err := charge()
		var limit *WorkLimitError
		if !errors.Is(err, decaderr.ErrUnsupported) || !errors.As(err, &limit) {
			t.Fatalf("expected typed limit refusal: %v", err)
		}
	}
	work, records := meter.Usage()
	if work != ReplayTrialWork || records != ReplayTrialRecords {
		t.Fatalf("usage: work=%d records=%d", work, records)
	}
	if meter.Bytes() != ReplayTrialBytes {
		t.Fatalf("byte count=%d", meter.Bytes())
	}
	cancelled, cancel := context.WithCancel(ctx)
	cancel()
	if err := meter.ReserveBytes(cancelled, 0); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancellation: %v", err)
	}
	if err := meter.ReserveArray(ctx, ^uint64(0), 2); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("array overflow: %v", err)
	}
	meter.bytes = ^uint64(0)
	if err := meter.ReserveBytes(ctx, 1); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("overflow: %v", err)
	}
}

func TestReplayBudgetAdaptersAndOverflow(t *testing.T) {
	ctx := t.Context()
	b := NewReplayTrialBudget()
	w := b.WorkBudget(ctx)
	m := b.MultiwayBudget()
	for range 2 {
		if err := w.Step(); err != nil {
			t.Fatal(err)
		}
		if err := m.SpendWork(ctx, 2); err != nil {
			t.Fatal(err)
		}
		if err := w.Reserve(1); err != nil {
			t.Fatal(err)
		}
		if err := m.ReserveRecords(ctx, 2); err != nil {
			t.Fatal(err)
		}
		if err := w.ReserveBytes(1); err != nil {
			t.Fatal(err)
		}
		if err := m.ReserveBytes(ctx, 2); err != nil {
			t.Fatal(err)
		}
	}
	work, records := b.Usage()
	if work != 6 || records != 6 || b.Bytes() != 6 {
		t.Fatalf("usage: %d %d %d", work, records, b.Bytes())
	}
	for _, call := range []func() error{
		func() error { return b.ReserveArray(ctx, ^uint64(0), 2) },
		func() error { return b.ReserveArray(ctx, ^uint64(0)/2, 1) },
		func() error { return w.ReserveMul(^uint64(0), 2) },
		func() error { return w.ReserveBytesMul(^uint64(0), 2) },
		func() error { return w.ReserveBytesMul(^uint64(0)/2, 1) },
	} {
		if err := call(); !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("overflow: %v", err)
		}
	}
	if b.Bytes() != 6 {
		t.Fatalf("bytes after overflow: %d", b.Bytes())
	}
	cancelled, cancel := context.WithCancel(ctx)
	cancel()
	cw := b.WorkBudget(cancelled)
	cm := b.MultiwayBudget()
	for _, call := range []func() error{cw.Step, func() error { return cw.Reserve(0) }, func() error { return cw.ReserveBytes(0) }, func() error { return cm.SpendWork(cancelled, 0) }, func() error { return cm.ReserveRecords(cancelled, 0) }, func() error { return cm.ReserveBytes(cancelled, 0) }} {
		if err := call(); !errors.Is(err, context.Canceled) {
			t.Fatalf("cancellation: %v", err)
		}
	}
	work, records = b.Usage()
	if work != 6 || records != 6 || b.Bytes() != 6 {
		t.Fatalf("usage after cancellation: %d %d %d", work, records, b.Bytes())
	}
}
