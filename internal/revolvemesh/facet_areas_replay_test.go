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
	b := proofbound.NewReplayTrialBudget()
	if e := run(t.Context(), b); e != nil {
		t.Fatal(e)
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
		e := run(t.Context(), m)
		if mode == replayBudgetTestExact && e != nil || mode != replayBudgetTestExact && !errors.Is(e, decaderr.ErrUnsupported) {
			t.Fatalf("%s: %v", mode, e)
		}
	}
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
