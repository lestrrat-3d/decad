package decad

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestBrakeRefusalAccounting(t *testing.T) {
	ctx := t.Context()
	static := errBrakeRefinedReplayProducerOverflow
	if !errors.Is(static, ErrUnsupported) {
		t.Fatal("static refusal lost sentinel")
	}
	if n := testing.AllocsPerRun(100, func() {
		if !errors.Is(static, ErrUnsupported) {
			panic("sentinel")
		}
	}); n != 0 {
		t.Fatalf("static refusal allocated %.0f objects", n)
	}
	measured := proofbound.NewBrakeReplayBudget()
	err := newBrakeAncestryError(ctx, measured, brakeSourceGroupProducer, 7, 9, 1)
	var detail *brakeAncestryError
	if !errors.As(err, &detail) || !errors.Is(err, ErrUnsupported) || err.Error() != ErrUnsupported.Error()+": refined clevis source group 7 producer=9 operand=1" {
		t.Fatalf("typed refusal: %v", err)
	}
	work, records := measured.Usage()
	bytes := measured.Bytes()
	for _, kind := range []string{brakeTestExact, brakeTestWork, brakeTestRecords, brakeTestBytes} {
		t.Run(kind, func(t *testing.T) {
			budget := proofbound.NewBrakeReplayBudget()
			w, r, b := proofbound.BrakeReplayWork-work, proofbound.BrakeReplayRecords-records, proofbound.BrakeReplayBytes-bytes
			switch kind {
			case brakeTestWork:
				w++
			case brakeTestRecords:
				r++
			case brakeTestBytes:
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
			err := newBrakeAncestryError(ctx, budget, brakeSourceGroupProducer, 7, 9, 1)
			if kind == brakeTestExact {
				if !errors.As(err, &detail) {
					t.Fatalf("exact: %v", err)
				}
			} else {
				if _, ok := errors.AsType[*proofbound.WorkLimitError](err); !ok {
					t.Fatalf("one-short %s: %v", kind, err)
				}
				if n := testing.AllocsPerRun(100, func() { _ = newBrakeAncestryError(ctx, budget, brakeSourceGroupProducer, 7, 9, 1) }); n != 0 {
					t.Fatalf("one-short refusal allocated %.0f objects", n)
				}
			}
		})
	}
}
