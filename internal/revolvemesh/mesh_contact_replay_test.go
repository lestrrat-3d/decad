package revolvemesh_test

import (
	"context"
	"errors"
	"testing"

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
	assertReplayBudgetLimits(t, run)
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
