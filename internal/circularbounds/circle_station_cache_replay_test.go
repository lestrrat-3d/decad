package circularbounds

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

func TestCircleStationReplayCache(t *testing.T) {
	seg := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 1.2, V: -3.4}, Radius: units.Millimeters(3.3), CCW: true, TEnd: 1}
	meter := proofbound.NewReplayTrialBudget()
	cache, err := NewCircleStationReplayCache(t.Context(), seg, meter)
	if err != nil {
		t.Fatal(err)
	}
	for _, k := range []int{1, 13, 267, 534, 801, 1067} {
		s, c, err := proofbound.CircleTurnFractionReplay(t.Context(), k, 1068, meter)
		if err != nil {
			t.Fatal(err)
		}
		u, v := 1.2+3.3*math.Cos(2*math.Pi*float64(k)/1068), -3.4+3.3*math.Sin(2*math.Pi*float64(k)/1068)
		got, err := cache.Bound(t.Context(), u, v, s, c, meter)
		if err != nil {
			t.Fatal(err)
		}
		want, err := circleStationFromIntervals(t.Context(), seg, u, v, s, c, proofbound.NewReplayTrialBudget())
		if err != nil {
			t.Fatal(err)
		}
		if got != want {
			t.Fatalf("station %d got=%+v want=%+v", k, got, want)
		}
		if _, err := cache.Bound(t.Context(), u, v, s, c, proofbound.NewReplayTrialBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("wrong meter: %v", err)
		}
	}
}

func TestCircleStationReplayCacheLimits(t *testing.T) {
	seg := sectionrecord.CircleSeg{Radius: units.Millimeters(3.3), CCW: true, TEnd: 1}
	sm := proofbound.NewReplayTrialBudget()
	s, c, err := proofbound.CircleTurnFractionReplay(t.Context(), 1, 13, sm)
	if err != nil {
		t.Fatal(err)
	}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) error {
		cache, err := NewCircleStationReplayCache(ctx, seg, b)
		if err != nil {
			return err
		}
		_, err = cache.Bound(ctx, 1, 2, s, c, b)
		return err
	}
	base := proofbound.NewReplayTrialBudget()
	if err := run(t.Context(), base); err != nil {
		t.Fatal(err)
	}
	work, records := base.Usage()
	bytes := base.Bytes()
	for _, mode := range []string{"exact", "work", "records", "bytes"} {
		b := proofbound.NewReplayTrialBudget()
		w, r, z := proofbound.ReplayTrialWork-work, proofbound.ReplayTrialRecords-records, proofbound.ReplayTrialBytes-bytes
		switch mode {
		case "work":
			w++
		case "records":
			r++
		case "bytes":
			z++
		}
		if err := b.SpendWork(t.Context(), w); err != nil {
			t.Fatal(err)
		}
		if err := b.ReserveRecords(t.Context(), r); err != nil {
			t.Fatal(err)
		}
		if err := b.ReserveBytes(t.Context(), z); err != nil {
			t.Fatal(err)
		}
		err := run(t.Context(), b)
		if mode == "exact" && err != nil || mode != "exact" && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("%s: %v", mode, err)
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if err := run(ctx, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel: %v", err)
	}
}
