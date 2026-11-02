package proofbound_test

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

const replayTestCaseBytesShort = "bytes short"
const replayTestCaseRecordsShort = "records short"
const replayTestCaseWorkShort = "work short"
const replayTestCaseExact = "exact"

func TestReplayAtan2IntervalParityAndLimits(t *testing.T) {
	cases := []struct {
		y, x          *big.Rat
		negativeZeroY bool
	}{
		{big.NewRat(0, 1), big.NewRat(1, 1), false},
		{big.NewRat(0, 1), big.NewRat(-1, 1), false},
		{big.NewRat(0, 1), big.NewRat(-1, 1), true},
		{big.NewRat(1, 1), big.NewRat(0, 1), false},
		{big.NewRat(-1, 1), big.NewRat(0, 1), false},
		{big.NewRat(1, 1), big.NewRat(1, 1), false},
		{big.NewRat(3, 1), big.NewRat(1, 1), false},
		{big.NewRat(1, 1), big.NewRat(3, 1), false},
		{big.NewRat(-3, 1), big.NewRat(1, 1), false},
		{big.NewRat(3, 1), big.NewRat(-1, 1), false},
		{big.NewRat(-3, 1), big.NewRat(-1, 1), false},
	}
	for i, tc := range cases {
		want := proofbound.Atan2Interval(tc.y, tc.x, tc.negativeZeroY)
		budget := proofbound.NewReplayTrialBudget()
		got, err := proofbound.ReplayAtan2Interval(t.Context(), tc.y, tc.x,
			tc.negativeZeroY, budget)
		if err != nil || got.Lo.Cmp(want.Lo) != 0 || got.Hi.Cmp(want.Hi) != 0 {
			t.Fatalf("input %d: atan2 interval differs: %v", i, err)
		}
	}
	y, x := big.NewRat(-3, 1), big.NewRat(-1, 1)
	trial := proofbound.NewReplayTrialBudget()
	if _, err := proofbound.ReplayAtan2Interval(t.Context(), y, x, false, trial); err != nil {
		t.Fatal(err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	t.Logf("atan2: work=%d records=%d bytes=%d", work, records, bytes)
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
			_, err = proofbound.ReplayAtan2Interval(t.Context(), y, x, false, budget)
			if (err == nil) != tc.want {
				t.Fatalf("admission=%t want %t", err == nil, tc.want)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, err := proofbound.ReplayAtan2Interval(ctx, y, x, false,
		proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("cancelled atan2 was admitted")
	}
}
