package decad

import (
	"context"
	"errors"
	"slices"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type brakeCancelAfterAdmission struct {
	context.Context //nolint:containedctx // The value lives only for one bounded replay call.
	cancel          context.CancelFunc
	admitted        atomic.Bool
	polls           atomic.Int64
}

const (
	brakeTestExact   = "exact"
	brakeTestRecords = "records"
	brakeTestWork    = "work"
	brakeTestBytes   = "bytes"
)

func (c *brakeCancelAfterAdmission) Value(key any) any {
	if _, ok := key.(brakeReplayMeterKey); ok {
		c.admitted.Store(true)
	}
	return c.Context.Value(key)
}

func (c *brakeCancelAfterAdmission) Err() error {
	if c.admitted.Load() {
		if c.polls.Add(1) == 100 {
			c.cancel()
		}
	}
	return c.Context.Err()
}

func TestBrakeRetryCancellationRollback(t *testing.T) {
	doc, target, clevis := brakeGalleryFixture(t, t.Context())
	before := doc.OperationHistory().Operations()
	live := doc.Bodies()
	inputs := make([][]OperationInput, len(before))
	outputs := make([][]*Body, len(before))
	var sourceBodies []*Body
	seen := make(map[*Body]bool)
	certificates := brakeRollbackCertificates(t, before)
	for i, op := range before {
		inputs[i], outputs[i] = op.Inputs(), op.Outputs()
		for _, b := range outputs[i] {
			if b != nil && !seen[b] {
				seen[b] = true
				sourceBodies = append(sourceBodies, b)
			}
		}
	}
	sourceSnapshot := brakeRollbackSnapshot(t, sourceBodies)
	targetOrigin, clevisOrigin := target.n.Origin, clevis.n.Origin
	targetShift, clevisShift := target.n.PrivateShift, clevis.n.PrivateShift
	base, cancel := context.WithCancel(t.Context())
	defer cancel()
	var meter *proofbound.ReplayBudget
	base = context.WithValue(base, brakeReplayMeterKey{}, &meter)
	ctx := &brakeCancelAfterAdmission{Context: base, cancel: cancel}
	result, err := Union(ctx, target, clevis)
	if !errors.Is(err, context.Canceled) || result != nil || meter == nil || ctx.polls.Load() < 100 {
		t.Fatalf("cancellation: error=%v result=%t replay=%t polls=%d", err, result != nil, meter != nil, ctx.polls.Load())
	}
	after := doc.OperationHistory().Operations()
	if !slices.Equal(before, after) || !slices.Equal(live, doc.Bodies()) {
		t.Fatal("cancelled replay changed history")
	}
	for i, op := range after {
		if !slices.Equal(inputs[i], op.Inputs()) || !slices.Equal(outputs[i], op.Outputs()) {
			t.Fatalf("cancelled replay changed operation %d inputs or outputs", i)
		}
	}
	if target.n.Origin != targetOrigin || clevis.n.Origin != clevisOrigin ||
		target.n.PrivateShift != targetShift || clevis.n.PrivateShift != clevisShift {
		t.Fatal("cancelled replay changed source provenance or certificates")
	}
	brakeRollbackCheck(t, sourceSnapshot)
	brakeRollbackCertificatesCheck(t, certificates)
}

func TestBrakeReplayInputsReservation(t *testing.T) {
	ctx := t.Context()
	measured := proofbound.NewBrakeReplayBudget()
	if err := reserveBrakeReplayInputs(ctx, measured); err != nil {
		t.Fatal(err)
	}
	_, records := measured.Usage()
	bytes := measured.Bytes()
	for _, kind := range []string{brakeTestExact, brakeTestRecords, brakeTestBytes} {
		t.Run(kind, func(t *testing.T) {
			budget := proofbound.NewBrakeReplayBudget()
			r, b := proofbound.BrakeReplayRecords-records, proofbound.BrakeReplayBytes-bytes
			if kind == brakeTestRecords {
				r++
			}
			if kind == brakeTestBytes {
				b++
			}
			if err := budget.ReserveRecords(ctx, r); err != nil {
				t.Fatal(err)
			}
			if err := budget.ReserveBytes(ctx, b); err != nil {
				t.Fatal(err)
			}
			err := reserveBrakeReplayInputs(ctx, budget)
			if kind == brakeTestExact && err != nil {
				t.Fatalf("exact reservation: %v", err)
			}
			if kind != brakeTestExact && !errors.Is(err, ErrUnsupported) {
				t.Fatalf("%s one-short: %v", kind, err)
			}
		})
	}
}

func TestBrakeBudgetFailureDoesNotAllocate(t *testing.T) {
	ctx := t.Context()
	for _, kind := range []string{brakeTestWork, brakeTestRecords, brakeTestBytes} {
		t.Run(kind, func(t *testing.T) {
			meter := proofbound.NewBrakeReplayBudget()
			var charge func() error
			switch kind {
			case brakeTestWork:
				if err := meter.SpendWork(ctx, proofbound.BrakeReplayWork); err != nil {
					t.Fatal(err)
				}
				charge = func() error { return meter.SpendWork(ctx, 1) }
			case brakeTestRecords:
				if err := meter.ReserveRecords(ctx, proofbound.BrakeReplayRecords); err != nil {
					t.Fatal(err)
				}
				charge = func() error { return meter.ReserveRecords(ctx, 1) }
			case brakeTestBytes:
				if err := meter.ReserveBytes(ctx, proofbound.BrakeReplayBytes); err != nil {
					t.Fatal(err)
				}
				charge = func() error { return meter.ReserveBytes(ctx, 1) }
			}
			allocs := testing.AllocsPerRun(100, func() {
				limitErr := charge()
				traceBrakeReplay(ctx, "refined output ancestry", limitErr)
				_, _, attempted, accepted, err := brakeRetryFailure(ctx, limitErr)
				if !attempted || accepted || err != nil {
					panic("unexpected budget failure result")
				}
			})
			if allocs != 0 {
				t.Fatalf("budget failure allocated %.0f objects", allocs)
			}
		})
	}
}
