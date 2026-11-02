package proofbound_test

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplayTrigFixedSeriesParityAndLimits(t *testing.T) {
	inputs := []*big.Int{
		new(big.Int),
		new(big.Int).Rsh(new(big.Int).Set(proofbound.TrigFixedOne), 3),
		new(big.Int).Rsh(new(big.Int).Set(proofbound.TrigFixedOne), 1),
		new(big.Int).Set(proofbound.TrigFixedOne),
	}
	for i, x := range inputs {
		wantSin, wantCos, wantError := proofbound.TrigFixedSeries(x)
		budget := proofbound.NewReplayTrialBudget()
		sin, cos, errUnits, err := proofbound.ReplayTrigFixedSeries(t.Context(), x, budget)
		if err != nil || sin.Cmp(wantSin) != 0 || cos.Cmp(wantCos) != 0 ||
			errUnits.Cmp(wantError) != 0 {
			t.Fatalf("input %d: replay differs from ordinary series: %v", i, err)
		}
	}
	x := new(big.Int).Rsh(new(big.Int).Set(proofbound.TrigFixedOne), 3)
	trial := proofbound.NewReplayTrialBudget()
	if _, _, _, err := proofbound.ReplayTrigFixedSeries(t.Context(), x, trial); err != nil {
		t.Fatal(err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	t.Logf("fixed series: work=%d records=%d bytes=%d", work, records, bytes)
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
			_, _, _, err = proofbound.ReplayTrigFixedSeries(t.Context(), x, budget)
			if (err == nil) != tc.want {
				t.Fatalf("admission=%t want %t", err == nil, tc.want)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, _, _, err := proofbound.ReplayTrigFixedSeries(ctx, x,
		proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("cancelled series was admitted")
	}
}

func TestReplayFixedRoundParityAndLimits(t *testing.T) {
	values := []*big.Rat{
		big.NewRat(0, 1), big.NewRat(1, 7), big.NewRat(-1, 7),
		big.NewRat(1, 8), big.NewRat(-1, 8),
	}
	for i, q := range values {
		budget := proofbound.NewReplayTrialBudget()
		floor, err := proofbound.ReplayFixedFloor(t.Context(), q, budget)
		if err != nil || floor.Cmp(proofbound.FixedFloor(q)) != 0 {
			t.Fatalf("input %d: floor parity failed: %v", i, err)
		}
		ceil, err := proofbound.ReplayFixedCeil(t.Context(), q, budget)
		if err != nil || ceil.Cmp(proofbound.FixedCeil(q)) != 0 {
			t.Fatalf("input %d: ceil parity failed: %v", i, err)
		}
	}
	q := big.NewRat(-1, 7)
	trial := proofbound.NewReplayTrialBudget()
	if _, err := proofbound.ReplayFixedFloor(t.Context(), q, trial); err != nil {
		t.Fatal(err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	t.Logf("fixed floor: work=%d records=%d bytes=%d", work, records, bytes)
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
			_, err = proofbound.ReplayFixedFloor(t.Context(), q, budget)
			if (err == nil) != tc.want {
				t.Fatalf("admission=%t want %t", err == nil, tc.want)
			}
		})
	}
	tooWide := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 2049))
	if got, err := proofbound.ReplayFixedFloor(t.Context(), tooWide,
		proofbound.NewReplayTrialBudget()); err == nil || got != nil {
		t.Fatal("integer wider than the audited basic arithmetic range was admitted")
	}
}

func TestReplayTurnFirstOctantSinCosParityAndLimits(t *testing.T) {
	for i, u := range []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 16), big.NewRat(1, 8)} {
		wantSin, wantCos := proofbound.TurnFirstOctantSinCos(u)
		budget := proofbound.NewReplayTrialBudget()
		sin, cos, err := proofbound.ReplayTurnFirstOctantSinCos(t.Context(), u, budget)
		if err != nil || sin.Lo.Cmp(wantSin.Lo) != 0 || sin.Hi.Cmp(wantSin.Hi) != 0 ||
			cos.Lo.Cmp(wantCos.Lo) != 0 || cos.Hi.Cmp(wantCos.Hi) != 0 {
			t.Fatalf("input %d: first-octant interval differs: %v", i, err)
		}
	}
	u := big.NewRat(1, 16)
	trial := proofbound.NewReplayTrialBudget()
	if _, _, err := proofbound.ReplayTurnFirstOctantSinCos(t.Context(), u, trial); err != nil {
		t.Fatal(err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	t.Logf("first octant: work=%d records=%d bytes=%d", work, records, bytes)
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
			_, _, err = proofbound.ReplayTurnFirstOctantSinCos(t.Context(), u, budget)
			if (err == nil) != tc.want {
				t.Fatalf("admission=%t want %t", err == nil, tc.want)
			}
		})
	}
}
