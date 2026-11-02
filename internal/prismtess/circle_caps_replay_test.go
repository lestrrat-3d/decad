package prismtess

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/units"
)

func TestCircleCapsReplay(t *testing.T) {
	seg := sectionrecord.CircleSeg{Radius: units.Millimeters(3), CCW: true, TEnd: 1}
	samples, e := tessellation.CircleSamplesReplay(t.Context(), seg, 1, .2, 4, proofbound.NewReplayTrialBudget())
	if e != nil {
		t.Fatal(e)
	}
	loop := tessellation.PrismLoop[int]{Samples: samples.Samples, FaceOf: samples.FaceOf, SagOf: samples.SagOf, BoundOf: samples.BoundOf, MaxSag: samples.MaxSag, WallSlack: samples.WallSlack, CapSlack: samples.CapSlack, SegmentArea: samples.SegmentArea, PerimeterUpper: samples.PerimeterUpper, Walks: samples.Walks}
	mk := func() tessellation.PrismTopology[int] {
		x, e := tessellation.CircleWallsReplay(t.Context(), loop, 0, proofbound.NewReplayTrialBudget())
		if e != nil {
			t.Fatal(e)
		}
		return x
	}
	top := mk()
	meter := proofbound.NewReplayTrialBudget()
	if e = circleCapsReplay(t.Context(), &top, 2, 3, meter); e != nil {
		t.Fatal(e)
	}
	n := len(samples.Samples)
	if len(top.Triangles) != 4*n-4 || len(top.ReplayFaceBounds) != 3 {
		t.Fatalf("triangles=%d faces=%d", len(top.Triangles), len(top.ReplayFaceBounds))
	}
	expected, err := triangulation.Triangulate(t.Context(), top.Points, top.LoopIdx)
	if err != nil {
		t.Fatal(err)
	}
	for i, tri := range expected {
		up := [3]int{2*tri[0] + 1, 2*tri[1] + 1, 2*tri[2] + 1}
		down := [3]int{2 * tri[0], 2 * tri[2], 2 * tri[1]}
		if top.Triangles[2*n+2*i] != up || top.Triangles[2*n+2*i+1] != down || top.Sources[2*n+2*i] != 3 || top.Sources[2*n+2*i+1] != 2 {
			t.Fatalf("cap facet %d differs", i)
		}
	}
	w, r := meter.Usage()
	z := meter.Bytes()
	for _, mode := range []string{replayBudgetTestExact, replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		top = mk()
		b := proofbound.NewReplayTrialBudget()
		ww, rr, zz := proofbound.ReplayTrialWork-w, proofbound.ReplayTrialRecords-r, proofbound.ReplayTrialBytes-z
		switch mode {
		case replayBudgetTestWork:
			ww++
		case replayBudgetTestRecords:
			rr++
		case replayBudgetTestBytes:
			zz++
		}
		if e = b.SpendWork(t.Context(), ww); e != nil {
			t.Fatal(e)
		}
		if e = b.ReserveRecords(t.Context(), rr); e != nil {
			t.Fatal(e)
		}
		if e = b.ReserveBytes(t.Context(), zz); e != nil {
			t.Fatal(e)
		}
		e = circleCapsReplay(t.Context(), &top, 2, 3, b)
		if mode == replayBudgetTestExact && e != nil || mode != replayBudgetTestExact && !errors.Is(e, decaderr.ErrUnsupported) {
			t.Fatalf("%s: %v", mode, e)
		}
		if mode != replayBudgetTestExact && (len(top.Triangles) != 2*n || len(top.ReplayFaceBounds) != 1) {
			t.Fatalf("%s partially mutated topology", mode)
		}
	}
	top = mk()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if e = circleCapsReplay(ctx, &top, 2, 3, proofbound.NewReplayTrialBudget()); !errors.Is(e, context.Canceled) {
		t.Fatalf("cancel: %v", e)
	}
	if len(top.Triangles) != 2*n {
		t.Fatal("canceled caps mutated topology")
	}
	// The brake route can carry distinct axial displacement on its two caps.
	top = mk()
	if e = circleCapsAxialReplay(t.Context(), &top, 2, 3, .01, .02, proofbound.NewBrakeReplayBudget()); e != nil {
		t.Fatal(e)
	}
	if len(top.Triangles) != 4*n-4 || len(top.ReplayFaceBounds) != 3 {
		t.Fatal("brake cap topology differs")
	}
	for _, bound := range top.ReplayFaceBounds {
		if bound.Face == 2 && bound.Axial != .01 || bound.Face == 3 && bound.Axial != .02 {
			t.Fatalf("cap %d lost its axial bound: %g", bound.Face, bound.Axial)
		}
	}
}
