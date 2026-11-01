package circularbounds_test

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplayArcEndpointPlanParityAndLimits(t *testing.T) {
	segments := []circularbounds.ArcSeg{
		{Center: circularbounds.Point2{}, Start: circularbounds.Point2{U: 2},
			End: circularbounds.Point2{V: 2}},
		{Center: circularbounds.Point2{U: 1, V: -1},
			Start: circularbounds.Point2{U: 3, V: 0},
			End:   circularbounds.Point2{U: 0, V: 1}},
	}
	for i, seg := range segments {
		budget := proofbound.NewReplayTrialBudget()
		plan, err := circularbounds.NewReplayArcEndpointPlan(t.Context(), seg, budget)
		if err != nil {
			t.Fatalf("source %d: %v", i, err)
		}
		for _, rt := range []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 3), big.NewRat(1, 1)} {
			wantU, wantV, wantOK := circularbounds.EndpointInterval(seg, rt)
			u, v, err := plan.Endpoint(t.Context(), rt, budget)
			if err != nil || !wantOK || u.Lo.Cmp(wantU.Lo) != 0 || u.Hi.Cmp(wantU.Hi) != 0 ||
				v.Lo.Cmp(wantV.Lo) != 0 || v.Hi.Cmp(wantV.Hi) != 0 {
				t.Fatalf("source %d parameter %s: endpoint differs: %v", i, rt, err)
			}
		}
	}
	seg := segments[1]
	rt := big.NewRat(1, 3)
	trial := proofbound.NewReplayTrialBudget()
	plan, err := circularbounds.NewReplayArcEndpointPlan(t.Context(), seg, trial)
	if err != nil {
		t.Fatal(err)
	}
	planWork, planRecords := trial.Usage()
	planBytes := trial.Bytes()
	if _, _, err := plan.Endpoint(t.Context(), rt, trial); err != nil {
		t.Fatal(err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	t.Logf("arc plan: work=%d records=%d bytes=%d", planWork, planRecords, planBytes)
	t.Logf("arc station: work=%d records=%d bytes=%d",
		work-planWork, records-planRecords, bytes-planBytes)
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		want                 bool
	}{
		{"exact", work, records, bytes, true},
		{"work short", work - 1, records, bytes, false},
		{"records short", work, records - 1, bytes, false},
		{"bytes short", work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
			if err != nil {
				t.Fatal(err)
			}
			plan, err := circularbounds.NewReplayArcEndpointPlan(t.Context(), seg, budget)
			if err == nil {
				_, _, err = plan.Endpoint(t.Context(), rt, budget)
			}
			if (err == nil) != tc.want {
				t.Fatalf("admission=%t want %t", err == nil, tc.want)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, err := circularbounds.NewReplayArcEndpointPlan(ctx, seg,
		proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("cancelled arc plan was admitted")
	}
}
