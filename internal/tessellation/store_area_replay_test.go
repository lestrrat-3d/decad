package tessellation

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestReplayStoreAreaAllow(t *testing.T) {
	vertices := []r3.Vec{{X: 100, Y: -20, Z: 7}, {X: 101, Y: -18, Z: 7}, {X: 98, Y: -19, Z: 9}, {X: 100.6, Y: -20.8, Z: 8}}
	triangles := [][3]int{{0, 1, 2}, {0, 2, 3}}
	store := []float64{.001, .002, .003, .004}
	want := StoreAreaAllow(vertices, triangles, store)
	run := func(ctx context.Context, m *proofbound.ReplayBudget) (float64, error) {
		return ReplayStoreAreaAllow(ctx, vertices, triangles, store, m)
	}
	baseline := proofbound.NewReplayTrialBudget()
	got, err := run(context.Background(), baseline)
	if err != nil || got < want {
		t.Fatalf("got=%g want at least %g error=%v", got, want, err)
	}
	w, r := baseline.Usage()
	if r != 0 || baseline.Bytes() != 0 {
		t.Fatalf("records=%d bytes=%d", r, baseline.Bytes())
	}
	exact, err := proofbound.NewReplayBudgetLimits(w, 1, 1)
	if err != nil {
		t.Fatal(err)
	}
	if got, err := run(context.Background(), exact); err != nil || got < want {
		t.Fatalf("exact=%g error=%v", got, err)
	}
	short, err := proofbound.NewReplayBudgetLimits(w-1, 1, 1)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := run(context.Background(), short); err == nil {
		t.Fatal("one-short work succeeded")
	}
	for _, bad := range []float64{-1, math.NaN(), math.Inf(1)} {
		store[0] = bad
		if _, err := run(context.Background(), proofbound.NewReplayTrialBudget()); err == nil {
			t.Fatal("invalid displacement accepted")
		}
	}
	store[0] = .001
	triangles[0][0] = 99
	if _, err := run(context.Background(), proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("invalid vertex accepted")
	}
	triangles[0][0] = 0
	vertices[0].X = math.MaxFloat64
	if _, err := run(context.Background(), proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("overflow accepted")
	}
	vertices[0].X = 100
	for len(triangles) < 32 {
		triangles = append(triangles, triangles[0])
	}
	lateBase, lateCancel := context.WithCancel(context.Background())
	defer lateCancel()
	late := &replayCloseCancel{Context: lateBase, cancel: lateCancel}
	if _, err := run(late, proofbound.NewReplayTrialBudget()); err == nil || late.polls.Load() < 20 {
		t.Fatalf("late cancellation polls=%d error=%v", late.polls.Load(), err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if _, err := run(ctx, proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("cancellation ignored")
	}
}
