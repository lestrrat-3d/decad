package topo

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

func TestPrismWalkSealReplayBudgetAndMutation(t *testing.T) {
	run := func(ctx context.Context, budget *proofbound.ReplayBudget) (*Body, *momentinput.ProfileWalks, bool, error) {
		body, _, profile, _ := prismSealFixture(t) //nolint:contextcheck // The test fixture uses t.Context().
		walks := &momentinput.ProfileWalks{Profile: profile, Metered: true,
			Outer: []survey2d.SegmentWalk{{Kind: survey2d.WalkLine, StartU: 1}}}
		ok, err := body.SealPrismBuildWalksReplay(ctx, walks, budget)
		return body, walks, ok, err
	}
	trial := proofbound.NewReplayTrialBudget()
	body, walks, ok, err := run(t.Context(), trial)
	if err != nil || !ok || !body.HasStrictPrismWalkSeal() {
		t.Fatalf("walk snapshot admitted=%t strict=%t err=%v", ok, body.HasStrictPrismWalkSeal(), err)
	}
	sourceBody := body
	work, records := trial.Usage()
	bytes := trial.Bytes()
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		want                 bool
	}{
		{replayTestCaseExact, work, records, bytes, true},
		{"work short", work - 1, records, bytes, false},
		{"records short", work, records - 1, bytes, false},
		{"bytes short", work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
			if err != nil {
				t.Fatal(err)
			}
			body, _, ok, err := run(t.Context(), budget)
			if (err == nil && ok && body.HasStrictPrismWalkSeal()) != tc.want {
				t.Fatalf("snapshot admitted=%t strict=%t want=%t err=%v",
					ok, body.HasStrictPrismWalkSeal(), tc.want, err)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	body, _, ok, err = run(ctx, proofbound.NewReplayTrialBudget())
	if err == nil || ok || body.HasStrictPrismWalkSeal() {
		t.Fatal("cancelled walk snapshot was published")
	}
	original := walks.Outer[0].StartV
	walks.Outer[0].StartV = math.Copysign(0, -1)
	if _, ok, err := sourceBody.ReplayPrismBuiltWalk(t.Context(), walks, 0, 0,
		proofbound.NewReplayTrialBudget()); err != nil || ok {
		t.Fatalf("changed signed zero admitted=%t err=%v", ok, err)
	}
	walks.Outer[0].StartV = original
}
