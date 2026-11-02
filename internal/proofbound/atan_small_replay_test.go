package proofbound_test

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplayAtanSmallIntervalParityAndLimits(t *testing.T) {
	for i, x := range []*big.Rat{big.NewRat(-1, 2), big.NewRat(-1, 3),
		big.NewRat(0, 1), big.NewRat(1, 3), big.NewRat(1, 2)} {
		want := proofbound.AtanSmallInterval(x)
		budget := proofbound.NewReplayTrialBudget()
		got, err := proofbound.ReplayAtanSmallInterval(t.Context(), x, budget)
		if err != nil || got.Lo.Cmp(want.Lo) != 0 || got.Hi.Cmp(want.Hi) != 0 {
			t.Fatalf("input %d: arctangent interval differs: %v", i, err)
		}
	}
	x := big.NewRat(-1, 3)
	trial := proofbound.NewReplayTrialBudget()
	if _, err := proofbound.ReplayAtanSmallInterval(t.Context(), x, trial); err != nil {
		t.Fatal(err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	t.Logf("small arctangent: work=%d records=%d bytes=%d", work, records, bytes)
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
			_, err = proofbound.ReplayAtanSmallInterval(t.Context(), x, budget)
			if (err == nil) != tc.want {
				t.Fatalf("admission=%t want %t", err == nil, tc.want)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, err := proofbound.ReplayAtanSmallInterval(ctx, x,
		proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("cancelled arctangent was admitted")
	}
	if _, err := proofbound.ReplayAtanSmallInterval(t.Context(), big.NewRat(3, 4),
		proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("out-of-range arctangent was admitted")
	}
}
