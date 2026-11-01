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

func TestAuditMeshReplay(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {X: 10}, {X: 11}, {X: 10, Y: 1}}
	tris := [][3]int{{0, 1, 2}, {3, 4, 5}}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) error {
		_, err := revolvemesh.AuditMeshReplay(ctx, verts, tris, b)
		return err
	}
	b := proofbound.NewReplayTrialBudget()
	if err := run(t.Context(), b); err != nil {
		t.Fatal(err)
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
		if err := m.SpendWork(t.Context(), ww); err != nil {
			t.Fatal(err)
		}
		if err := m.ReserveRecords(t.Context(), rr); err != nil {
			t.Fatal(err)
		}
		if err := m.ReserveBytes(t.Context(), zz); err != nil {
			t.Fatal(err)
		}
		err := run(t.Context(), m)
		if mode == replayBudgetTestExact && err != nil || mode != replayBudgetTestExact && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("%s: %v", mode, err)
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if err := run(ctx, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel: %v", err)
	}
	copy(verts[3:], verts[:3])
	if err := run(t.Context(), proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("coincident triangles accepted")
	}
}
