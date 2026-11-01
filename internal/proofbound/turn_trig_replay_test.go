package proofbound_test

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplayTurnSinCosIntervalParityAndLimits(t *testing.T) {
	for k := -8; k <= 16; k++ {
		u := big.NewRat(int64(2*k+1), 16)
		wantSin, wantCos := proofbound.TurnSinCosInterval(u)
		budget := proofbound.NewReplayTrialBudget()
		sin, cos, err := proofbound.ReplayTurnSinCosInterval(t.Context(), u, budget)
		if err != nil || sin.Lo.Cmp(wantSin.Lo) != 0 || sin.Hi.Cmp(wantSin.Hi) != 0 ||
			cos.Lo.Cmp(wantCos.Lo) != 0 || cos.Hi.Cmp(wantCos.Hi) != 0 {
			t.Fatalf("turn numerator %d: interval differs: %v", 2*k+1, err)
		}
	}
	u := big.NewRat(-7, 16)
	trial := proofbound.NewReplayTrialBudget()
	if _, _, err := proofbound.ReplayTurnSinCosInterval(t.Context(), u, trial); err != nil {
		t.Fatal(err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	t.Logf("turn interval: work=%d records=%d bytes=%d", work, records, bytes)
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		want                 bool
	}{
		{replayTestCaseExact, work, records, bytes, true},
		{replayTestCaseWorkShort, work - 1, records, bytes, false},
		{replayTestCaseRecordsShort, work, records - 1, bytes, false},
		{replayTestCaseBytesShort, work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
			if err != nil {
				t.Fatal(err)
			}
			_, _, err = proofbound.ReplayTurnSinCosInterval(t.Context(), u, budget)
			if (err == nil) != tc.want {
				t.Fatalf("admission=%t want %t", err == nil, tc.want)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, _, err := proofbound.ReplayTurnSinCosInterval(ctx, u,
		proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("cancelled turn was admitted")
	}
}
