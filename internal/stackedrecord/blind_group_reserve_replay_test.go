package stackedrecord

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReserveBlindPayloadArrays(t *testing.T) {
	const floors = 4
	baseline := proofbound.NewReplayTrialBudget()
	if err := reserveBlindPayloadArrays(context.Background(), baseline, floors); err != nil {
		t.Fatal(err)
	}
	w, r := baseline.Usage()
	b := baseline.Bytes()
	if w != 0 || r != floors+5 {
		t.Fatalf("work=%d records=%d", w, r)
	}
	exact, err := proofbound.NewReplayBudgetLimits(1, r, b)
	if err != nil {
		t.Fatal(err)
	}
	if err := reserveBlindPayloadArrays(context.Background(), exact, floors); err != nil {
		t.Fatal(err)
	}
	short, err := proofbound.NewReplayBudgetLimits(1, r-1, b)
	if err != nil {
		t.Fatal(err)
	}
	if err := reserveBlindPayloadArrays(context.Background(), short, floors); err == nil {
		t.Fatal("one-short record budget succeeded")
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if err := reserveBlindPayloadArrays(ctx, proofbound.NewReplayTrialBudget(), floors); err == nil {
		t.Fatal("cancelled reservation succeeded")
	}
}
