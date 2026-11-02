package prismcells

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

type scanCancelContext struct {
	context.Context //nolint:containedctx // The context is confined to one synchronous replay or test.
	polls           int
}

func (c *scanCancelContext) Err() error {
	c.polls++
	if c.polls >= 4 {
		return context.Canceled
	}
	return nil
}

func TestReplayGroupCutMatchLoops(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		t.Fatal(err)
	}
	entity := func(x float64) sketch.Entity { return s.CreateCircle(s.CreatePoint(x, 0), 1) }
	outer, hole, a, b, c, d := entity(0), entity(3), entity(6), entity(9), entity(12), entity(15)
	edge := func(e sketch.Entity) []sketch.BoundaryEdge { return []sketch.BoundaryEdge{{Entity: e}} }
	tags := map[sketch.Entity]Origin{
		outer: {Hole: -1}, hole: {Hole: 0}, a: {IsB: true, Hole: -1}, b: {IsB: true, Region: 1, Hole: -1}, c: {IsB: true, Region: 2, Hole: -1}, d: {IsB: true, Region: 3, Hole: -1},
	}
	p := &sketch.Profile{Outer: edge(outer), Holes: [][]sketch.BoundaryEdge{edge(b), edge(d), edge(hole), edge(a), edge(c)}}
	run := func(ctx context.Context, budget *proofbound.ReplayBudget) (*sketch.Profile, bool, error) {
		return replayGroupCutMatchLoops(ctx, budget, []*sketch.Profile{p}, tags, 1, 4)
	}
	meter := proofbound.NewReplayTrialBudget()
	got, ok, err := run(context.Background(), meter)
	if err != nil || !ok || got != p {
		t.Fatalf("match ok=%t error=%v", ok, err)
	}
	work, records := meter.Usage()
	allocationMeter := proofbound.NewReplayTrialBudget()
	allocations := testing.AllocsPerRun(10, func() {
		_, _, _ = run(context.Background(), allocationMeter)
	})
	if allocations != 0 {
		t.Fatalf("matching allocated %.0f times", allocations)
	}
	exact, err := proofbound.NewReplayBudgetLimits(work, 1, 1)
	if err != nil {
		t.Fatal(err)
	}
	if _, ok, err = run(context.Background(), exact); err != nil || !ok {
		t.Fatalf("exact ok=%t error=%v", ok, err)
	}
	short, err := proofbound.NewReplayBudgetLimits(work-1, 1, 1)
	if err != nil {
		t.Fatal(err)
	}
	if _, ok, err = run(context.Background(), short); err == nil || ok {
		t.Fatalf("short ok=%t error=%v", ok, err)
	}
	if records != 0 || meter.Bytes() != 0 {
		t.Fatalf("unexpected allocation charge %d %d", records, meter.Bytes())
	}
	saved := p.Holes[0]
	p.Holes[0] = p.Holes[3]
	if _, ok, err = run(context.Background(), proofbound.NewReplayTrialBudget()); err != nil || ok {
		t.Fatalf("duplicate ok=%t error=%v", ok, err)
	}
	p.Holes[0] = saved
	if _, ok, err = replayGroupCutMatchLoops(context.Background(), proofbound.NewReplayTrialBudget(), []*sketch.Profile{p, p}, tags, 1, 4); err != nil || ok {
		t.Fatalf("ambiguous ok=%t error=%v", ok, err)
	}
	oldOrigin := tags[b]
	tags[b] = Origin{IsB: true, Region: 7, Hole: -1}
	if _, ok, err = run(context.Background(), proofbound.NewReplayTrialBudget()); err != nil || ok {
		t.Fatalf("changed origin ok=%t error=%v", ok, err)
	}
	tags[b] = oldOrigin
	p.Holes[0][0].Partial = true
	if _, ok, err = run(context.Background(), proofbound.NewReplayTrialBudget()); err != nil || ok {
		t.Fatalf("partial ok=%t error=%v", ok, err)
	}
	scanCtx := &scanCancelContext{Context: context.Background()}
	if _, ok, err = run(scanCtx, proofbound.NewReplayTrialBudget()); err == nil || ok || scanCtx.polls < 4 {
		t.Fatalf("scan cancellation ok=%t polls=%d error=%v", ok, scanCtx.polls, err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if _, ok, err = run(ctx, proofbound.NewReplayTrialBudget()); err == nil || ok {
		t.Fatalf("cancel ok=%t error=%v", ok, err)
	}
}
