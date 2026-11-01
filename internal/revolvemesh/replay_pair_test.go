package revolvemesh

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestReplayUncertainContactRefuses(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {}, {X: 1}, {Y: 1}}
	tris := [][3]int{{0, 1, 2}, {3, 4, 5}}
	var data []RevolveAuditTri
	for _, tri := range tris {
		x, ok := NewRevolveAuditTri(verts, tri)
		if !ok {
			t.Fatal("invalid fixture")
		}
		data = append(data, x)
	}
	err := auditRevolvePairReplay(context.Background(), data, tris, [3]int{}, 0, 0, 1, 0, proofbound.NewReplayTrialBudget())
	if !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("uncertain pair: %v", err)
	}
	ordinary := AuditRevolvePair(data, tris, [3]int{}, 0, 0, 1, 0)
	if ordinary == nil || ordinary.Error() != err.Error() {
		t.Fatalf("uncertain error differs: %v / %v", err, ordinary)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if err = auditRevolvePairReplay(ctx, data, tris, [3]int{}, 0, 0, 1, 0, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("canceled pair: %v", err)
	}
	for _, kind := range []string{replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		b := proofbound.NewReplayTrialBudget()
		var e error
		switch kind {
		case replayBudgetTestWork:
			e = b.SpendWork(context.Background(), proofbound.ReplayTrialWork)
		case replayBudgetTestRecords:
			e = b.ReserveRecords(context.Background(), proofbound.ReplayTrialRecords)
		case replayBudgetTestBytes:
			e = b.ReserveBytes(context.Background(), proofbound.ReplayTrialBytes)
		}
		if e != nil {
			t.Fatal(e)
		}
		if e = auditRevolvePairReplay(context.Background(), data, tris, [3]int{}, 0, 0, 1, 0, b); !errors.Is(e, decaderr.ErrUnsupported) {
			t.Fatalf("%s limit: %v", kind, e)
		}
	}
}
