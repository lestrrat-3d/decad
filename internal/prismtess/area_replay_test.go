package prismtess

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

//nolint:containedctx // The context is confined to this synchronous replay.
type replayCancelAfter struct {
	context.Context
	calls int
}

func (c *replayCancelAfter) Err() error {
	c.calls++
	if c.calls >= 2 {
		return context.Canceled
	}
	return nil
}

func TestPerturbedAreaReplay(t *testing.T) {
	vertices := []r3.Vec{{X: 100, Y: -20, Z: 7}, {X: 101, Y: -18, Z: 7}, {X: 98, Y: -19, Z: 9}, {X: 100.6, Y: -20.8, Z: 8}}
	m := &topo.Mesh{Vertices: vertices, Triangles: [][3]int{{0, 1, 2}, {0, 2, 3}}}
	b := proofbound.NewReplayTrialBudget()
	got, err := perturbedAreaReplay(t.Context(), m, .002, b)
	if err != nil {
		t.Fatal(err)
	}
	want := proofbound.AbsSumUpper(proofbound.PerturbedTriangleAreaUpper(vertices[0], vertices[1], vertices[2], .002), proofbound.PerturbedTriangleAreaUpper(vertices[0], vertices[2], vertices[3], .002))
	if got < want {
		t.Fatalf("area bound %g below %g", got, want)
	}
	sliver := []r3.Vec{{X: 4, Y: 9}, {X: 5, Y: 9}, {X: 5, Y: 9 + 1e-12}}
	got, err = perturbedAreaReplay(t.Context(), &topo.Mesh{Vertices: sliver, Triangles: [][3]int{{0, 1, 2}}}, .002, proofbound.NewReplayTrialBudget())
	if err != nil || got < proofbound.PerturbedTriangleAreaUpper(sliver[0], sliver[1], sliver[2], .002) {
		t.Fatalf("sliver bound %g: %v", got, err)
	}
	if _, err := perturbedAreaReplay(t.Context(), m, -1, proofbound.NewReplayTrialBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("negative displacement: %v", err)
	}
	for _, d := range []float64{math.NaN(), math.Inf(1)} {
		if _, err := perturbedAreaReplay(t.Context(), m, d, proofbound.NewReplayTrialBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("nonfinite displacement: %v", err)
		}
	}
	overflow := &topo.Mesh{Vertices: []r3.Vec{{X: math.MaxFloat64}, {Y: math.MaxFloat64}, {Z: math.MaxFloat64}}, Triangles: [][3]int{{0, 1, 2}}}
	if _, err := perturbedAreaReplay(t.Context(), overflow, 1, proofbound.NewReplayTrialBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("overflow: %v", err)
	}
	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	if _, err := perturbedAreaReplay(cancelled, m, .002, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel before: %v", err)
	}
	during := &replayCancelAfter{Context: t.Context()}
	if _, err := perturbedAreaReplay(during, m, .002, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel during: %v", err)
	}
	w, r := b.Usage()
	z := b.Bytes()
	for _, short := range []string{"", replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		if short == replayBudgetTestRecords && r == 0 || short == replayBudgetTestBytes && z == 0 {
			continue
		}
		m := proofbound.NewReplayTrialBudget()
		a, c, d := proofbound.ReplayTrialWork-w, proofbound.ReplayTrialRecords-r, proofbound.ReplayTrialBytes-z
		switch short {
		case replayBudgetTestWork:
			a++
		case replayBudgetTestRecords:
			c++
		case replayBudgetTestBytes:
			d++
		}
		if err := m.SpendWork(t.Context(), a); err != nil {
			t.Fatal(err)
		}
		if err := m.ReserveRecords(t.Context(), c); err != nil {
			t.Fatal(err)
		}
		if err := m.ReserveBytes(t.Context(), d); err != nil {
			t.Fatal(err)
		}
		_, err := perturbedAreaReplay(t.Context(), &topo.Mesh{Vertices: vertices, Triangles: [][3]int{{0, 1, 2}, {0, 2, 3}}}, .002, m)
		if short == "" && err != nil || short != "" && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("%s: %v", short, err)
		}
	}
}

func TestStoreAreaReplay(t *testing.T) {
	vertices := []r3.Vec{{X: 100, Y: -20, Z: 7}, {X: 101, Y: -18, Z: 7}, {X: 98, Y: -19, Z: 9}, {X: 100.6, Y: -20.8, Z: 8}}
	mesh := &topo.Mesh{Vertices: vertices, Triangles: [][3]int{{0, 1, 2}, {0, 2, 3}}}
	store := []float64{.001, .002, .003, .004}
	b := proofbound.NewReplayTrialBudget()
	got, err := storeAreaReplay(t.Context(), mesh, store, b)
	if err != nil {
		t.Fatal(err)
	}
	want := tessellation.StoreAreaAllow(vertices, mesh.Triangles, store)
	if got < want {
		t.Fatalf("bound=%g want at least %g", got, want)
	}
	w, r := b.Usage()
	z := b.Bytes()
	for _, mode := range []string{replayBudgetTestExact, replayBudgetTestWork} {
		m := proofbound.NewReplayTrialBudget()
		work := proofbound.ReplayTrialWork - w
		if mode == replayBudgetTestWork {
			work++
		}
		if e := m.SpendWork(t.Context(), work); e != nil {
			t.Fatal(e)
		}
		if e := m.ReserveRecords(t.Context(), proofbound.ReplayTrialRecords-r); e != nil {
			t.Fatal(e)
		}
		if e := m.ReserveBytes(t.Context(), proofbound.ReplayTrialBytes-z); e != nil {
			t.Fatal(e)
		}
		_, e := storeAreaReplay(t.Context(), mesh, store, m)
		if mode == replayBudgetTestExact && e != nil || mode == replayBudgetTestWork && !errors.Is(e, decaderr.ErrUnsupported) {
			t.Fatalf("%s: %v", mode, e)
		}
	}
	for _, bad := range []float64{-1, math.NaN(), math.Inf(1)} {
		store[0] = bad
		if _, e := storeAreaReplay(t.Context(), mesh, store, proofbound.NewReplayTrialBudget()); !errors.Is(e, decaderr.ErrUnsupported) {
			t.Fatalf("bad displacement %g: %v", bad, e)
		}
	}
}
