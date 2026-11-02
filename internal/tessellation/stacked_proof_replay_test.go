package tessellation

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestReplayProveStackedConservativeSliver(t *testing.T) {
	in := StackedProofInput[int]{
		Rings:         []StackedRing[int]{{Walks: 3, PerimeterUpper: 12, SegmentArea: .2}},
		ColumnHeights: []float64{4}, SlabHeights: []float64{4},
		SlabColumns: [][]int{{0}}, SectionDelta: 1e-12,
		Faces: []int{7}, Source: []int{7}, FaceAxial: map[int]float64{7: 1e-12},
		IsPlanar: func(int) bool { return true },
		Vertices: []r3.Vec{{X: 1e12, Y: 1e12},
			{X: 1e12 + .125, Y: 1e12},
			{X: 1e12 + .125, Y: 1e12 + .0001220703125}},
		Triangles: [][3]int{{0, 1, 2}}, Store: []float64{1e-12, 2e-12, 1e-12},
		StoreMax: 2e-12, AreaSlack: .1,
	}
	want := ProveStacked(in)
	budget := proofbound.NewReplayTrialBudget()
	got, err := ReplayProveStackedConservative(t.Context(), in, budget)
	if err != nil {
		t.Fatal(err)
	}
	if got.AreaSlack < want.AreaSlack || len(got.Terms) != len(want.Terms) {
		t.Fatalf("area=%g ordinary=%g terms=%d ordinary=%d",
			got.AreaSlack, want.AreaSlack, len(got.Terms), len(want.Terms))
	}
	for i := range got.Terms {
		if got.Terms[i] < want.Terms[i] {
			t.Fatalf("term %d=%g ordinary=%g", i, got.Terms[i], want.Terms[i])
		}
	}
	work, records := budget.Usage()
	if work == 0 || budget.Bytes() == 0 {
		t.Fatal("missing charges")
	}
	for _, limits := range [...][3]uint64{{work - 1, records, budget.Bytes()},
		{work, records, budget.Bytes() - 1}} {
		short, err := proofbound.NewReplayBudgetLimits(limits[0], max(limits[1], 1), limits[2])
		if err != nil {
			t.Fatal(err)
		}
		if _, err := ReplayProveStackedConservative(t.Context(), in, short); err == nil {
			t.Fatal("one-short proof succeeded")
		}
	}
	in.StoreMax = math.SmallestNonzeroFloat64
	if _, err := ReplayProveStackedConservative(t.Context(), in, proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("understated store maximum accepted")
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, err := ReplayProveStackedConservative(ctx, in, proofbound.NewReplayTrialBudget()); err != context.Canceled {
		t.Fatalf("cancellation error=%v", err)
	}
}
