package tessdispatch

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestBrakeReplayRefusalDoesNotAllocate(t *testing.T) {
	ctx := t.Context()
	budget := proofbound.NewBrakeReplayBudget()
	if err := budget.SpendWork(ctx, proofbound.BrakeReplayWork); err != nil {
		t.Fatal(err)
	}
	if err := budget.ReserveRecords(ctx, proofbound.BrakeReplayRecords); err != nil {
		t.Fatal(err)
	}
	if err := budget.ReserveBytes(ctx, proofbound.BrakeReplayBytes); err != nil {
		t.Fatal(err)
	}
	run := func() error { _, err := FacetedReplay(ctx, nil, 1, budget); return err }
	if err := run(); !errors.Is(err, decaderr.ErrBooleanFailed) {
		t.Fatalf("refusal: %v", err)
	}
	if n := testing.AllocsPerRun(100, func() {
		if !errors.Is(run(), decaderr.ErrBooleanFailed) {
			panic("refusal")
		}
	}); n != 0 {
		t.Fatalf("refusal allocated %.0f objects", n)
	}
}
