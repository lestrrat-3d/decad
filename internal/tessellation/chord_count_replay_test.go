package tessellation_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

func TestReplayChordCountParityAndLimits(t *testing.T) {
	for _, tc := range []struct {
		name      string
		radius    float64
		sweep     float64
		tolerance float64
	}{
		{"whole circle", 1.5, 2 * math.Pi, .12},
		{"quarter arc", 5, math.Pi / 2, .05},
		{"coarse minimum", 2, math.Pi / 2, 5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			walk := survey2d.SegmentWalk{Radius: tc.radius, Th1: tc.sweep}
			wantN, wantSag, err := tessellation.ChordCount(walk, tc.tolerance, 3)
			if err != nil {
				t.Fatal(err)
			}
			meter := proofbound.NewReplayTrialBudget()
			n, sag, err := tessellation.ReplayChordCount(t.Context(), walk, tc.tolerance, 3, meter)
			if err != nil || n != wantN || math.Float64bits(sag) != math.Float64bits(wantSag) {
				t.Fatalf("chord count=%d sag=%g err=%v; want %d, %g", n, sag, err, wantN, wantSag)
			}
		})
	}
	walk := survey2d.SegmentWalk{Radius: 1.5, Th1: 2 * math.Pi}
	trial := proofbound.NewReplayTrialBudget()
	if _, _, err := tessellation.ReplayChordCount(t.Context(), walk, .12, 3, trial); err != nil {
		t.Fatal(err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
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
			meter, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
			if err != nil {
				t.Fatal(err)
			}
			_, _, err = tessellation.ReplayChordCount(t.Context(), walk, .12, 3, meter)
			if (err == nil) != tc.want {
				t.Fatalf("chord count refusal=%v want admission=%t", err, tc.want)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, _, err := tessellation.ReplayChordCount(ctx, walk, .12, 3,
		proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("cancelled chord count was admitted")
	}
	underflow := survey2d.SegmentWalk{Radius: math.SmallestNonzeroFloat64,
		Th1: math.SmallestNonzeroFloat64}
	if _, _, err := tessellation.ReplayChordCount(t.Context(), underflow, .12, 3,
		proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("unmetered exact sagitta fallback was admitted")
	}
}
