package tessellation

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplaySymDiff(t *testing.T) {
	terms := []float64{1e-12, -.1, 0, .03}
	want, err := SymDiff(terms)
	if err != nil {
		t.Fatal(err)
	}
	m := proofbound.NewReplayTrialBudget()
	got, err := ReplaySymDiff(t.Context(), terms, m)
	if err != nil || got != want {
		t.Fatalf("got=%g error=%v", got, err)
	}
	w, r := m.Usage()
	if r != 0 || m.Bytes() != 0 {
		t.Fatalf("records=%d bytes=%d", r, m.Bytes())
	}
	exact, err := proofbound.NewReplayBudgetLimits(w, 1, 1)
	if err != nil {
		t.Fatal(err)
	}
	if got, err := ReplaySymDiff(t.Context(), terms, exact); err != nil || got != want {
		t.Fatalf("exact=%g error=%v", got, err)
	}
	short, err := proofbound.NewReplayBudgetLimits(w-1, 1, 1)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := ReplaySymDiff(t.Context(), terms, short); err == nil {
		t.Fatal("one-short work succeeded")
	}
	for _, v := range []float64{math.NaN(), math.Inf(1)} {
		if _, err := ReplaySymDiff(t.Context(), []float64{v}, proofbound.NewReplayTrialBudget()); err == nil {
			t.Fatal("nonfinite term accepted")
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, err := ReplaySymDiff(ctx, terms, proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("cancellation ignored")
	}
}
