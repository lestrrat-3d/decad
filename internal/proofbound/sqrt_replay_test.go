package proofbound_test

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplayRatSqrtBoundsParityAndLimits(t *testing.T) {
	for i, q := range []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 1),
		big.NewRat(2, 1), big.NewRat(9, 4), big.NewRat(225, 4)} {
		budget := proofbound.NewReplayTrialBudget()
		lo, err := proofbound.ReplayRatSqrtDown(t.Context(), q, budget)
		if err != nil || math.Float64bits(lo) != math.Float64bits(proofbound.RatSqrtDown(q)) {
			t.Fatalf("input %d: lower sqrt differs: %v", i, err)
		}
		hi, err := proofbound.ReplayRatSqrtUp(t.Context(), q, budget)
		if err != nil || math.Float64bits(hi) != math.Float64bits(proofbound.RatSqrtUp(q)) {
			t.Fatalf("input %d: upper sqrt differs: %v", i, err)
		}
	}
	q := big.NewRat(2, 1)
	trial := proofbound.NewReplayTrialBudget()
	if _, err := proofbound.ReplayRatSqrtDown(t.Context(), q, trial); err != nil {
		t.Fatal(err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	t.Logf("sqrt down: work=%d records=%d bytes=%d", work, records, bytes)
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
			_, err = proofbound.ReplayRatSqrtDown(t.Context(), q, budget)
			if (err == nil) != tc.want {
				t.Fatalf("admission=%t want %t", err == nil, tc.want)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, err := proofbound.ReplayRatSqrtDown(ctx, q,
		proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("cancelled sqrt was admitted")
	}
}
