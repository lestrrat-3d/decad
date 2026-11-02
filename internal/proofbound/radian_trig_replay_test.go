package proofbound_test

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplayRadSinCosSpanParityAndLimits(t *testing.T) {
	spans := []proofbound.RatInterval{
		proofbound.Interval(big.NewRat(0, 1), big.NewRat(0, 1)),
		proofbound.Interval(big.NewRat(1, 3), big.NewRat(17, 50)),
		proofbound.Interval(big.NewRat(-1, 3), big.NewRat(-3, 10)),
		proofbound.Atan2Interval(big.NewRat(3, 1), big.NewRat(-1, 1), false),
	}
	for i, span := range spans {
		wantSin, wantCos, wantOK := proofbound.RadSinCosSpan(span)
		budget := proofbound.NewReplayTrialBudget()
		sin, cos, ok, err := proofbound.ReplayRadSinCosSpan(t.Context(), span, budget)
		if err != nil || ok != wantOK || !ok || sin.Lo.Cmp(wantSin.Lo) != 0 ||
			sin.Hi.Cmp(wantSin.Hi) != 0 || cos.Lo.Cmp(wantCos.Lo) != 0 ||
			cos.Hi.Cmp(wantCos.Hi) != 0 {
			t.Fatalf("span %d: radian interval differs: ok=%t err=%v", i, ok, err)
		}
	}
	span := spans[3]
	trial := proofbound.NewReplayTrialBudget()
	if _, _, ok, err := proofbound.ReplayRadSinCosSpan(t.Context(), span, trial); err != nil || !ok {
		t.Fatalf("trial span: ok=%t err=%v", ok, err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	t.Logf("radian span: work=%d records=%d bytes=%d", work, records, bytes)
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
			_, _, ok, err := proofbound.ReplayRadSinCosSpan(t.Context(), span, budget)
			if (err == nil && ok) != tc.want {
				t.Fatalf("admission=%t want %t", err == nil && ok, tc.want)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, _, ok, err := proofbound.ReplayRadSinCosSpan(ctx, span,
		proofbound.NewReplayTrialBudget()); err == nil || ok {
		t.Fatal("cancelled radian span was admitted")
	}
}
