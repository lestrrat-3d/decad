package proof_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplaySqrtDyadic(t *testing.T) {
	for _, x := range []float64{math.SmallestNonzeroFloat64, math.Ldexp(1, -900), .1, 1, 2, math.Ldexp(1, 900)} {
		d := proof.DyOfFinite(x)
		for _, down := range []bool{false, true} {
			b := proofbound.NewReplayTrialBudget()
			got, e := proof.ReplaySqrtDyadic(t.Context(), d, down, b)
			if e != nil {
				t.Fatal(e)
			}
			want := proof.DySqrtUp(d)
			if down {
				want = proof.DySqrtDown(d)
			}
			if got != want {
				t.Fatalf("x=%g down=%t got=%g want=%g", x, down, got, want)
			}
			w, r := b.Usage()
			z := b.Bytes()
			for _, mode := range []string{replayBudgetTestExact, replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
				if mode == replayBudgetTestRecords && r == 0 || mode == replayBudgetTestBytes && z == 0 {
					continue
				}
				m := proofbound.NewReplayTrialBudget()
				ww, rr, zz := proofbound.ReplayTrialWork-w, proofbound.ReplayTrialRecords-r, proofbound.ReplayTrialBytes-z
				switch mode {
				case replayBudgetTestWork:
					ww++
				case replayBudgetTestRecords:
					rr++
				case replayBudgetTestBytes:
					zz++
				}
				if e = m.SpendWork(t.Context(), ww); e != nil {
					t.Fatal(e)
				}
				if e = m.ReserveRecords(t.Context(), rr); e != nil {
					t.Fatal(e)
				}
				if e = m.ReserveBytes(t.Context(), zz); e != nil {
					t.Fatal(e)
				}
				_, e = proof.ReplaySqrtDyadic(t.Context(), d, down, m)
				if mode == replayBudgetTestExact && e != nil || mode != replayBudgetTestExact && !errors.Is(e, decaderr.ErrUnsupported) {
					t.Fatalf("%s: %v", mode, e)
				}
			}
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, e := proof.ReplaySqrtDyadic(ctx, proof.DyOfFinite(2), false, proofbound.NewReplayTrialBudget()); !errors.Is(e, context.Canceled) {
		t.Fatalf("cancel: %v", e)
	}
}
