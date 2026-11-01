package tessellation_test

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/units"
)

func TestCircleWallsReplay(t *testing.T) {
	seg := sectionrecord.CircleSeg{Radius: units.Millimeters(3), CCW: true, TEnd: 1}
	sample, err := tessellation.CircleSamplesReplay(t.Context(), seg, 7, .2, 4, proofbound.NewReplayTrialBudget())
	if err != nil {
		t.Fatal(err)
	}
	loop := tessellation.PrismLoop[int]{Samples: sample.Samples, FaceOf: sample.FaceOf, SagOf: sample.SagOf, BoundOf: sample.BoundOf, MaxSag: sample.MaxSag, WallSlack: sample.WallSlack, CapSlack: sample.CapSlack, SegmentArea: sample.SegmentArea, PerimeterUpper: sample.PerimeterUpper, Walks: sample.Walks}
	b := proofbound.NewReplayTrialBudget()
	got, err := tessellation.CircleWallsReplay(t.Context(), loop, 0, b)
	if err != nil {
		t.Fatal(err)
	}
	want := tessellation.PrismWalls([]tessellation.PrismLoop[int]{loop}, false, 0)
	if len(got.Triangles) != len(want.Triangles) || len(got.ReplayFaceBounds) != 1 || got.AreaSlack != want.AreaSlack {
		t.Fatal("wall summary mismatch")
	}
	for i := range want.Triangles {
		if got.Triangles[i] != want.Triangles[i] || got.Sources[i] != want.Sources[i] {
			t.Fatalf("wall facet %d differs", i)
		}
	}
	w, r := b.Usage()
	z := b.Bytes()
	for _, mode := range []string{replayBudgetTestExact, replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		m := proofbound.NewReplayTrialBudget()
		ww, rr, zz := proofbound.ReplayTrialWork-w, proofbound.ReplayTrialRecords-r, proofbound.ReplayTrialBytes-z
		switch mode {
		case replayBudgetTestWork:
			ww++
		case replayBudgetTestRecords:
			rr++
		case replayBudgetTestBytes:
			zz++
		}
		if e := m.SpendWork(t.Context(), ww); e != nil {
			t.Fatal(e)
		}
		if e := m.ReserveRecords(t.Context(), rr); e != nil {
			t.Fatal(e)
		}
		if e := m.ReserveBytes(t.Context(), zz); e != nil {
			t.Fatal(e)
		}
		_, e := tessellation.CircleWallsReplay(t.Context(), loop, 0, m)
		if mode == replayBudgetTestExact && e != nil || mode != replayBudgetTestExact && !errors.Is(e, decaderr.ErrUnsupported) {
			t.Fatalf("%s: %v", mode, e)
		}
	}
	loop.FaceOf[1]++
	if _, err = tessellation.CircleWallsReplay(t.Context(), loop, 0, proofbound.NewReplayTrialBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("mixed face: %v", err)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, err = tessellation.CircleWallsReplay(ctx, loop, 0, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel: %v", err)
	}
}
