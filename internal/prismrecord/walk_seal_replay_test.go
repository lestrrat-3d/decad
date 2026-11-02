package prismrecord_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

func TestPrismWalkSealRealProducer(t *testing.T) {
	body, err := prismrecord.Build(t.Context(), captureBuildEnv(), boxPayload(t), freeform.NewFreeformWork())
	if err != nil {
		t.Fatal(err)
	}
	pw := body.Payload.(prismrecord.Payload).Walks
	if pw == nil || !body.HasStrictPrismWalkSeal() {
		t.Fatal("producer did not seal its walks")
	}
	check := func(ctx context.Context, budget *proofbound.ReplayBudget) (bool, error) {
		_, ok, err := prismrecord.ReplayPrismBuiltWalk(ctx, body, pw, 0, 0, budget)
		return ok, err
	}
	meter := proofbound.NewReplayTrialBudget()
	if ok, err := check(t.Context(), meter); err != nil || !ok {
		t.Fatalf("original admitted=%t err=%v", ok, err)
	}
	work, records := meter.Usage()
	bytes := meter.Bytes()
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		want                 bool
	}{
		{"exact", work, records, bytes, true}, {"work short", work - 1, records, bytes, false},
		{"records short", work, records - 1, bytes, false}, {"bytes short", work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			b, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
			if err != nil {
				t.Fatal(err)
			}
			ok, err := check(t.Context(), b)
			if (ok && err == nil) != tc.want {
				t.Fatalf("admitted=%t err=%v", ok, err)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if ok, err := check(ctx, proofbound.NewReplayTrialBudget()); ok || err == nil {
		t.Fatalf("canceled admitted=%t err=%v", ok, err)
	}
	clone := *pw
	if _, ok, err := prismrecord.ReplayPrismBuiltWalk(t.Context(), body, &clone, 0, 0, proofbound.NewReplayTrialBudget()); ok || err != nil {
		t.Fatalf("copied walks admitted=%t err=%v", ok, err)
	}
	old := pw.Outer[0].StartU
	pw.Outer[0].StartU = math.Nextafter(old, math.Inf(1))
	if ok, err := check(t.Context(), proofbound.NewReplayTrialBudget()); ok || err != nil {
		t.Fatalf("changed walk admitted=%t err=%v", ok, err)
	}
	pw.Outer[0].StartU = old
	original := pw.Profile.Outer.Segments[0]
	line := original.(sectionrecord.LineSeg)
	line.Start.U = math.Nextafter(line.Start.U, math.Inf(1))
	pw.Profile.Outer.Segments[0] = line
	if ok, err := check(t.Context(), proofbound.NewReplayTrialBudget()); ok || err != nil {
		t.Fatalf("changed profile admitted=%t err=%v", ok, err)
	}
	pw.Profile.Outer.Segments[0] = original
	if ok, err := check(t.Context(), proofbound.NewReplayTrialBudget()); !ok || err != nil {
		t.Fatalf("restored source admitted=%t err=%v", ok, err)
	}
}
