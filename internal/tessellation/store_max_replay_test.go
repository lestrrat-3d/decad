package tessellation

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplayStoreMax(t *testing.T) {
	input := []float64{0, 1e-12, .002, 2e-5}
	want, err := StoreMax(input)
	if err != nil {
		t.Fatal(err)
	}
	baseline := proofbound.NewReplayTrialBudget()
	got, err := ReplayStoreMax(t.Context(), input, baseline)
	if err != nil || got != want {
		t.Fatalf("got=%g error=%v", got, err)
	}
	w, r := baseline.Usage()
	if r != 0 || baseline.Bytes() != 0 {
		t.Fatalf("records=%d bytes=%d", r, baseline.Bytes())
	}
	exact, err := proofbound.NewReplayBudgetLimits(w, 1, 1)
	if err != nil {
		t.Fatal(err)
	}
	if got, err := ReplayStoreMax(t.Context(), input, exact); err != nil || got != want {
		t.Fatalf("exact=%g error=%v", got, err)
	}
	short, err := proofbound.NewReplayBudgetLimits(w-1, 1, 1)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := ReplayStoreMax(t.Context(), input, short); err == nil {
		t.Fatal("one-short work succeeded")
	}
	for _, v := range []float64{-1, math.NaN(), math.Inf(1)} {
		if _, err := ReplayStoreMax(t.Context(), []float64{v}, proofbound.NewReplayTrialBudget()); err == nil {
			t.Fatal("invalid displacement accepted")
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, err := ReplayStoreMax(ctx, input, proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("cancellation ignored")
	}
}
