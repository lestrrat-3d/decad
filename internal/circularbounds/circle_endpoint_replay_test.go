package circularbounds_test

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

func TestReplayCircleEndpointPlanParityAndLimits(t *testing.T) {
	seg := circularbounds.CircleSeg{Center: circularbounds.Point2{U: 3, V: -2},
		Radius: units.Millimeters(1.5), CCW: true}
	trial := proofbound.NewReplayTrialBudget()
	plan, err := circularbounds.NewReplayCircleEndpointPlan(t.Context(), seg, trial)
	if err != nil {
		t.Fatal(err)
	}
	for _, rt := range []*big.Rat{big.NewRat(-1, 4), big.NewRat(0, 1),
		big.NewRat(1, 7), big.NewRat(1, 4), big.NewRat(1, 3)} {
		wantU, wantV, ok := circularbounds.EndpointInterval(seg, rt)
		u, v, err := plan.Endpoint(t.Context(), rt, trial)
		if err != nil || !ok || u.Lo.Cmp(wantU.Lo) != 0 || u.Hi.Cmp(wantU.Hi) != 0 ||
			v.Lo.Cmp(wantV.Lo) != 0 || v.Hi.Cmp(wantV.Hi) != 0 {
			t.Fatalf("parameter %s: endpoint differs: %v", rt, err)
		}
	}
	rt := big.NewRat(1, 7)
	trial = proofbound.NewReplayTrialBudget()
	plan, err = circularbounds.NewReplayCircleEndpointPlan(t.Context(), seg, trial)
	if err != nil {
		t.Fatal(err)
	}
	if _, _, err := plan.Endpoint(t.Context(), rt, trial); err != nil {
		t.Fatal(err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	t.Logf("circle plan and station: work=%d records=%d bytes=%d", work, records, bytes)
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
			plan, err := circularbounds.NewReplayCircleEndpointPlan(t.Context(), seg, budget)
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
	if _, err := circularbounds.NewReplayCircleEndpointPlan(ctx, seg,
		proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("cancelled circle plan was admitted")
	}
}
