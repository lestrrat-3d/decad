package booleaneval

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

func TestDiskRingDelta(t *testing.T) {
	type point = sectionrecord.Point2
	ring := []point{{U: 1}, {V: 1}, {U: -1}, {V: -1}}
	delta, ok, err := diskRingDelta(t.Context(), ring, 1, proofbound.NewStrictReplayBudget())
	if err != nil || !ok || delta < 1-math.Sqrt(.5) || delta > .293 {
		t.Fatalf("diamond delta %g, valid=%v, err=%v", delta, ok, err)
	}
	for _, bad := range [][]point{append(append([]point{}, ring...), ring...), {ring[0], ring[2], ring[1], ring[3]}, {{U: math.NaN()}, ring[1], ring[2]}} {
		_, ok, err = diskRingDelta(t.Context(), bad, 1, proofbound.NewStrictReplayBudget())
		if ok || err != nil {
			t.Fatalf("accepted invalid ring: valid=%v, err=%v", ok, err)
		}
	}
	for _, r := range []float64{0, -1, math.Inf(1), math.NaN()} {
		_, ok, err = diskRingDelta(t.Context(), ring, r, proofbound.NewStrictReplayBudget())
		if ok || err != nil {
			t.Fatalf("accepted invalid radius %g", r)
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, _, err = diskRingDelta(ctx, ring, 1, proofbound.NewStrictReplayBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancellation returned %v", err)
	}
	meter, err := proofbound.NewReplayBudgetLimits(1, 1, 1)
	if err != nil {
		t.Fatal(err)
	}
	if _, _, err = diskRingDelta(t.Context(), ring, 1, meter); err == nil {
		t.Fatal("exhausted meter accepted polygon")
	}
}
