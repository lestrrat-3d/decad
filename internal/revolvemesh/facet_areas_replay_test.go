package revolvemesh_test

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/r3"
)

func TestFacetAreasReplay(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
	tris := [][3]int{{0, 1, 2}, {0, 3, 1}}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) error {
		_, err := revolvemesh.FacetAreasReplay(ctx, verts, tris, 0, b)
		return err
	}
	assertReplayBudgetLimits(t, run)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if e := run(ctx, proofbound.NewReplayTrialBudget()); !errors.Is(e, context.Canceled) {
		t.Fatalf("cancel: %v", e)
	}
	tris[1] = [3]int{0, 0, 1}
	if e := run(t.Context(), proofbound.NewReplayTrialBudget()); e == nil {
		t.Fatal("zero area accepted")
	}
}

func assertReplayBudgetLimits(t *testing.T, run func(context.Context, *proofbound.ReplayBudget) error) {
	t.Helper()
	trial := proofbound.NewReplayTrialBudget()
	if err := run(t.Context(), trial); err != nil {
		t.Fatal(err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	for _, mode := range []string{replayBudgetTestExact, replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		budget := proofbound.NewReplayTrialBudget()
		w := proofbound.ReplayTrialWork - work
		r := proofbound.ReplayTrialRecords - records
		b := proofbound.ReplayTrialBytes - bytes
		switch mode {
		case replayBudgetTestWork:
			w++
		case replayBudgetTestRecords:
			r++
		case replayBudgetTestBytes:
			b++
		}
		if err := budget.SpendWork(t.Context(), w); err != nil {
			t.Fatal(err)
		}
		if err := budget.ReserveRecords(t.Context(), r); err != nil {
			t.Fatal(err)
		}
		if err := budget.ReserveBytes(t.Context(), b); err != nil {
			t.Fatal(err)
		}
		err := run(t.Context(), budget)
		if mode == replayBudgetTestExact && err != nil || mode != replayBudgetTestExact && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("%s: %v", mode, err)
		}
	}
}
