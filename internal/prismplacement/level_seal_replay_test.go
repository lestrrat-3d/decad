package prismplacement

import (
	"context"
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestLevelDenotationSnapshotReplay(t *testing.T) {
	d := DenoteLevel(big.NewRat(5, 3), 1, 1)
	s := SnapshotLevelDenotation(d)
	baseline := proofbound.NewReplayTrialBudget()
	ok, err := s.ReplayMatches(t.Context(), d, baseline)
	if err != nil || !ok {
		t.Fatalf("original admitted=%t error=%v", ok, err)
	}
	w, r := baseline.Usage()
	b := baseline.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		want    bool
	}{
		{replayTestCaseExact, w, r, b, true},
		{"work short", w - 1, r, b, false},
		{"records short", w, r - 1, b, false},
		{"bytes short", w, r, b - 1, false},
	} {
		if tc.r == 0 {
			continue
		}
		t.Run(tc.name, func(t *testing.T) {
			budget, err := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			if err != nil {
				t.Fatal(err)
			}
			ok, err := s.ReplayMatches(t.Context(), d, budget)
			if tc.want && (err != nil || !ok) || !tc.want && (err == nil || ok) {
				t.Fatalf("admitted=%t error=%v", ok, err)
			}
		})
	}
	replaced := DenoteLevel(big.NewRat(5, 3), 1, 1)
	if ok, err := s.ReplayMatches(t.Context(), replaced, proofbound.NewReplayTrialBudget()); err != nil || ok {
		t.Fatalf("equal-valued replacement admitted=%t error=%v", ok, err)
	}
	d.exact.SetInt64(2)
	if ok, err := s.ReplayMatches(t.Context(), d, proofbound.NewReplayTrialBudget()); err != nil || ok {
		t.Fatalf("mutated original admitted=%t error=%v", ok, err)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, err := s.ReplayMatches(ctx, d, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancelled error=%v", err)
	}
}
