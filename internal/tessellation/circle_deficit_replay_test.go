package tessellation_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/units"
)

func TestCircleScheduleReplayRecordedWalk(t *testing.T) {
	seg := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 4, V: -2}, Radius: units.Millimeters(3), CCW: true, TEnd: 1}
	walk, err := boundarywalk.WalkOf(seg, nil)
	if err != nil {
		t.Fatal(err)
	}
	budget := proofbound.NewReplayTrialBudget()
	n, sag, capArea, wallArea, err := tessellation.CircleScheduleReplay(t.Context(), seg, walk, .05, 7, budget)
	if err != nil || n < 3 || sag > .05 || capArea <= 0 || wallArea <= 0 {
		t.Fatalf("count=%d sag=%g cap=%g wall=%g error=%v", n, sag, capArea, wallArea, err)
	}
	if want := tessellation.ChordSagitta(3, proofbound.TwoPiUpper(), n); math.Float64bits(sag) != math.Float64bits(want) {
		t.Fatalf("sag=%g want=%g", sag, want)
	}
	work, records := budget.Usage()
	if records != 2 || budget.Bytes() != 512 {
		t.Fatalf("schedule records=%d bytes=%d", records, budget.Bytes())
	}
	for _, mode := range []string{replayBudgetTestExact, replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		meter := proofbound.NewReplayTrialBudget()
		used, reserved, allocated := proofbound.ReplayTrialWork-work, proofbound.ReplayTrialRecords-records, proofbound.ReplayTrialBytes-budget.Bytes()
		switch mode {
		case replayBudgetTestWork:
			used++
		case replayBudgetTestRecords:
			reserved++
		case replayBudgetTestBytes:
			allocated++
		}
		if err := meter.SpendWork(t.Context(), used); err != nil {
			t.Fatal(err)
		}
		if err := meter.ReserveRecords(t.Context(), reserved); err != nil {
			t.Fatal(err)
		}
		if err := meter.ReserveBytes(t.Context(), allocated); err != nil {
			t.Fatal(err)
		}
		_, _, _, _, err := tessellation.CircleScheduleReplay(t.Context(), seg, walk, .05, 7, meter)
		if mode != replayBudgetTestExact && !errors.Is(err, decaderr.ErrUnsupported) || mode == replayBudgetTestExact && err != nil {
			t.Fatalf("mode=%s error=%v", mode, err)
		}
	}
	walk.CU++
	if _, _, _, _, err := tessellation.CircleScheduleReplay(t.Context(), seg, walk, .05, 7, proofbound.NewReplayTrialBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("changed source walk error=%v", err)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, _, _, _, err := tessellation.CircleScheduleReplay(ctx, seg, walk, .05, 7, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancellation error=%v", err)
	}
}

func TestCircleDeficitReplayBounds(t *testing.T) {
	for _, n := range []int{3, 4, 16, 64} {
		const radius, height = 3.0, 7.0
		capArea, wallArea, err := tessellation.CircleDeficitReplay(t.Context(), radius, n, height, proofbound.NewReplayTrialBudget())
		if err != nil {
			t.Fatal(err)
		}
		angle := 2 * math.Pi / float64(n)
		capDeficit := math.Pi*radius*radius - float64(n)*radius*radius*math.Sin(angle)/2
		wallDeficit := height * (2*math.Pi*radius - 2*float64(n)*radius*math.Sin(angle/2))
		if capArea < capDeficit || wallArea < wallDeficit {
			t.Fatalf("chords=%d cap=%g want>=%g wall=%g want>=%g", n, capArea, capDeficit, wallArea, wallDeficit)
		}
		_, reversed, err := tessellation.CircleDeficitReplay(t.Context(), radius, n, -height, proofbound.NewReplayTrialBudget())
		if err != nil || math.Float64bits(reversed) != math.Float64bits(wallArea) {
			t.Fatalf("negative height changed wall bound: %v", err)
		}
	}
}

func TestCircleDeficitReplayLimits(t *testing.T) {
	for _, short := range []bool{false, true} {
		budget := proofbound.NewReplayTrialBudget()
		used := proofbound.ReplayTrialWork - 64
		if short {
			used++
		}
		if err := budget.SpendWork(t.Context(), used); err != nil {
			t.Fatal(err)
		}
		_, _, err := tessellation.CircleDeficitReplay(t.Context(), 3, 64, 7, budget)
		if short && !errors.Is(err, decaderr.ErrUnsupported) || !short && err != nil {
			t.Fatalf("short=%t error=%v", short, err)
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, _, err := tessellation.CircleDeficitReplay(ctx, 3, 64, 7, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancellation error=%v", err)
	}
	for _, radius := range []float64{0, math.NaN(), math.Inf(1), math.SmallestNonzeroFloat64} {
		if _, _, err := tessellation.CircleDeficitReplay(t.Context(), radius, 64, 7, proofbound.NewReplayTrialBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("radius=%g error=%v", radius, err)
		}
	}
}
