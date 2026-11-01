package booloperand

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func TestPairChordReplayBudgetAndRefusals(t *testing.T) {
	ctx := t.Context()
	a := &topo.Body{Payload: prismrecord.Payload{}}
	a.Bounds.Max = r3.Vec{X: 1, Y: 2, Z: 3}
	b := &topo.Body{Payload: prismrecord.Payload{}}
	b.Bounds.Max = r3.Vec{X: 2, Y: 3, Z: 4}
	meter := proofbound.NewReplayTrialBudget()
	for range 2 {
		if _, _, err := PairChordReplay(ctx, a, b, meter); err != nil {
			t.Fatal(err)
		}
	}
	work, _ := meter.Usage()
	if work != 4 {
		t.Fatalf("work=%d", work)
	}
	for _, remaining := range []uint64{2, 1} {
		meter := proofbound.NewReplayTrialBudget()
		if err := meter.SpendWork(ctx, proofbound.ReplayTrialWork-remaining); err != nil {
			t.Fatal(err)
		}
		_, _, err := PairChordReplay(ctx, a, b, meter)
		if remaining == 2 && err != nil {
			t.Fatal(err)
		}
		if remaining == 1 && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("limit: %v", err)
		}
		work, _ := meter.Usage()
		if work != proofbound.ReplayTrialWork {
			t.Fatalf("work=%d", work)
		}
	}
	meter = proofbound.NewReplayTrialBudget()
	if _, _, err := PairChordReplay(ctx, nil, b, meter); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("nil body: %v", err)
	}
	if _, _, err := PairChordReplay(ctx, a, b, nil); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("nil meter: %v", err)
	}
	revolve := &topo.Body{Payload: revolveplan.Payload{}}
	if _, _, err := PairChordReplay(ctx, revolve, b, meter); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("revolve: %v", err)
	}
	work, _ = meter.Usage()
	if work != 1 {
		t.Fatalf("work=%d", work)
	}
	zero := &topo.Body{Payload: prismrecord.Payload{}}
	if _, _, err := PairChordReplay(ctx, zero, zero, meter); !errors.Is(err, decaderr.ErrDegenerate) {
		t.Fatalf("extent: %v", err)
	}
	cancelled, cancel := context.WithCancel(ctx)
	cancel()
	if _, _, err := PairChordReplay(cancelled, a, b, meter); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancelled: %v", err)
	}
	work, _ = meter.Usage()
	if work != 3 {
		t.Fatalf("work=%d", work)
	}
}
