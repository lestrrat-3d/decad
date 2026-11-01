package booleaneval

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestBrakeCandidateRefusalAccounting(t *testing.T) {
	ctx := t.Context()
	measured := proofbound.NewBrakeReplayBudget()
	err := newBrakeCandidateGroupError(ctx, measured, 7)
	var detail *brakeCandidateGroupError
	if !errors.As(err, &detail) || !errors.Is(err, decaderr.ErrUnsupported) || err.Error() != decaderr.ErrUnsupported.Error()+": candidate source group 7 changed" {
		t.Fatalf("typed refusal: %v", err)
	}
	work, records := measured.Usage()
	bytes := measured.Bytes()
	for _, name := range []string{shiftBudgetExactCase, shiftBudgetWorkCase, shiftBudgetRecordsCase, shiftBudgetBytesCase} {
		t.Run(name, func(t *testing.T) {
			budget := proofbound.NewBrakeReplayBudget()
			w, r, b := proofbound.BrakeReplayWork-work, proofbound.BrakeReplayRecords-records, proofbound.BrakeReplayBytes-bytes
			switch name {
			case shiftBudgetWorkCase:
				w++
			case shiftBudgetRecordsCase:
				r++
			case shiftBudgetBytesCase:
				b++
			}
			if err := budget.SpendWork(ctx, w); err != nil {
				t.Fatal(err)
			}
			if err := budget.ReserveRecords(ctx, r); err != nil {
				t.Fatal(err)
			}
			if err := budget.ReserveBytes(ctx, b); err != nil {
				t.Fatal(err)
			}
			err := newBrakeCandidateGroupError(ctx, budget, 7)
			if name == shiftBudgetExactCase {
				if !errors.As(err, &detail) {
					t.Fatalf("exact: %v", err)
				}
				return
			}
			if _, ok := errors.AsType[*proofbound.WorkLimitError](err); !ok {
				t.Fatalf("one-short: %v", err)
			}
			if n := testing.AllocsPerRun(100, func() { _ = newBrakeCandidateGroupError(ctx, budget, 7) }); n != 0 {
				t.Fatalf("one-short refusal allocated %.0f objects", n)
			}
		})
	}
}
