package prismtess

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func TestVerifiedFaceBoundsReplay(t *testing.T) {
	f, g := new(topo.Face), new(topo.Face)
	inputs := []tessellation.PrismReplayFaceBound[*topo.Face]{{Face: f, Trim: .1}, {Face: g, Trim: .2}}
	makeMesh := func() *topo.Mesh {
		return &topo.Mesh{Vertices: make([]r3.Vec, 4), Triangles: [][3]int{{0, 1, 2}, {0, 2, 3}}, Source: []*topo.Face{f, g}}
	}
	store := []float64{.01, .02, .03, .04}
	m := makeMesh()
	b := proofbound.NewReplayTrialBudget()
	if e := verifiedFaceBoundsReplay(t.Context(), m, inputs, store, 0, b); e != nil {
		t.Fatal(e)
	}
	for _, face := range []*topo.Face{f, g} {
		value, ok, e := m.ReplayFaceBound(t.Context(), face, proofbound.NewReplayTrialBudget())
		if e != nil || !ok || value <= 0 {
			t.Fatalf("face bound=%g found=%t error=%v", value, ok, e)
		}
	}
	withSection := makeMesh()
	if err := verifiedFaceBoundsReplay(t.Context(), withSection, inputs, store, .03, proofbound.NewReplayTrialBudget()); err != nil {
		t.Fatal(err)
	}
	baseline, _, err := m.ReplayFaceBound(t.Context(), f, proofbound.NewReplayTrialBudget())
	if err != nil {
		t.Fatal(err)
	}
	withSectionBound, _, err := withSection.ReplayFaceBound(t.Context(), f, proofbound.NewReplayTrialBudget())
	if err != nil || withSectionBound < baseline+.03 {
		t.Fatalf("section bound=%g baseline=%g error=%v", withSectionBound, baseline, err)
	}
	w, r := b.Usage()
	z := b.Bytes()
	for _, mode := range []string{replayBudgetTestExact, replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		mesh := makeMesh()
		meter := proofbound.NewReplayTrialBudget()
		ww, rr, zz := proofbound.ReplayTrialWork-w, proofbound.ReplayTrialRecords-r, proofbound.ReplayTrialBytes-z
		switch mode {
		case replayBudgetTestWork:
			ww++
		case replayBudgetTestRecords:
			rr++
		case replayBudgetTestBytes:
			zz++
		}
		if e := meter.SpendWork(t.Context(), ww); e != nil {
			t.Fatal(e)
		}
		if e := meter.ReserveRecords(t.Context(), rr); e != nil {
			t.Fatal(e)
		}
		if e := meter.ReserveBytes(t.Context(), zz); e != nil {
			t.Fatal(e)
		}
		e := verifiedFaceBoundsReplay(t.Context(), mesh, inputs, store, 0, meter)
		if mode == replayBudgetTestExact && e != nil || mode != replayBudgetTestExact && (!errors.Is(e, decaderr.ErrUnsupported) || mesh.ReplayFaceBounds != nil) {
			t.Fatalf("%s: %v", mode, e)
		}
	}
	for _, bad := range [][]tessellation.PrismReplayFaceBound[*topo.Face]{{inputs[0]}, {inputs[0], inputs[0]}, {inputs[0], inputs[1], {Face: new(topo.Face)}}} {
		if e := verifiedFaceBoundsReplay(t.Context(), makeMesh(), bad, store, 0, proofbound.NewReplayTrialBudget()); !errors.Is(e, decaderr.ErrUnsupported) {
			t.Fatalf("bad inputs: %v", e)
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if e := verifiedFaceBoundsReplay(ctx, makeMesh(), inputs, store, 0, proofbound.NewReplayTrialBudget()); !errors.Is(e, context.Canceled) {
		t.Fatalf("cancel: %v", e)
	}
}
