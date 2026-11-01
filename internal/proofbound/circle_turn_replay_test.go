package proofbound

import (
	"context"
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

func TestCircleTurnFractionReplayParity(t *testing.T) {
	for _, n := range []int{3, 4, 7, 8, 31, 221} {
		for k := 1; k < n; k++ {
			s, c, err := CircleTurnFractionReplay(t.Context(), k, n, NewReplayTrialBudget())
			if err != nil {
				t.Fatalf("%d/%d: %v", k, n, err)
			}
			var wantS, wantC RatInterval
			if 4*k%n == 0 {
				v := [4][2]int64{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}[4*k/n]
				wantS, wantC = PointInterval(big.NewRat(v[0], 1)), PointInterval(big.NewRat(v[1], 1))
			} else {
				wantS, wantC = TurnSinCosInterval(big.NewRat(int64(k), int64(n)))
			}
			if s.Lo.Cmp(wantS.Lo) != 0 || s.Hi.Cmp(wantS.Hi) != 0 || c.Lo.Cmp(wantC.Lo) != 0 || c.Hi.Cmp(wantC.Hi) != 0 {
				t.Fatalf("%d/%d intervals differ", k, n)
			}
		}
	}
}

func TestCircleTurnFractionReplayLimits(t *testing.T) {
	meter := NewReplayTrialBudget()
	if _, _, err := CircleTurnFractionReplay(t.Context(), 1, 221, meter); err != nil {
		t.Fatal(err)
	}
	work, records := meter.Usage()
	bytes := meter.Bytes()
	for _, mode := range []string{"exact", replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		t.Run(mode, func(t *testing.T) {
			b := NewReplayTrialBudget()
			w, r, z := ReplayTrialWork-work, ReplayTrialRecords-records, ReplayTrialBytes-bytes
			switch mode {
			case replayBudgetTestWork:
				w++
			case replayBudgetTestRecords:
				r++
			case replayBudgetTestBytes:
				z++
			}
			if err := b.SpendWork(t.Context(), w); err != nil {
				t.Fatal(err)
			}
			if err := b.ReserveRecords(t.Context(), r); err != nil {
				t.Fatal(err)
			}
			if err := b.ReserveBytes(t.Context(), z); err != nil {
				t.Fatal(err)
			}
			_, _, err := CircleTurnFractionReplay(t.Context(), 1, 221, b)
			if mode == "exact" && err != nil || mode != "exact" && !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("error=%v", err)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, _, err := CircleTurnFractionReplay(ctx, 1, 221, NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel=%v", err)
	}
}
