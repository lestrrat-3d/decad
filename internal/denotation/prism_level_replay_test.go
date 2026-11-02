package denotation

import (
	"context"
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestValidateZeroLevelReplay(t *testing.T) {
	d := DenoteLevel(big.NewRat(3, 2), 1.5, 0)
	if d.exact == nil {
		t.Fatal("denotation missing")
	}
	base := proofbound.NewCircleReplayBudget()
	if err := ValidateZeroLevelReplay(t.Context(), 1.5, d, base); err != nil {
		t.Fatal(err)
	}
	w, r := base.Usage()
	z := base.Bytes()
	for _, kind := range []string{replayTestCaseExact, "work", "records", "bytes"} {
		t.Run(kind, func(t *testing.T) {
			b := proofbound.NewCircleReplayBudget()
			ww, rr, zz := proofbound.ReplayTrialWork-w, proofbound.ReplayTrialRecords-r, proofbound.ReplayTrialBytes-z
			switch kind {
			case "work":
				ww++
			case "records":
				rr++
			case "bytes":
				zz++
			}
			if err := b.SpendWork(t.Context(), ww); err != nil {
				t.Fatal(err)
			}
			if err := b.ReserveRecords(t.Context(), rr); err != nil {
				t.Fatal(err)
			}
			if err := b.ReserveBytes(t.Context(), zz); err != nil {
				t.Fatal(err)
			}
			err := ValidateZeroLevelReplay(t.Context(), 1.5, d, b)
			if kind == replayTestCaseExact && err != nil || kind != replayTestCaseExact && !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("budget: %v", err)
			}
		})
	}
	d.exact.SetFrac64(7, 4)
	if err := ValidateZeroLevelReplay(t.Context(), 1.5, d, proofbound.NewCircleReplayBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatal("accepted changed denotation")
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if err := ValidateZeroLevelReplay(ctx, 1.5, LevelDenotation{}, proofbound.NewCircleReplayBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancellation: %v", err)
	}
}
