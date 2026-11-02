package topobuild

import (
	"context"
	"math"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

type vertexReplayCancel struct {
	context.Context //nolint:containedctx // The context is confined to one synchronous replay or test.
	polls           atomic.Int64
	cancel          context.CancelFunc
}

func (c *vertexReplayCancel) Err() error {
	if c.polls.Add(1) == 10 {
		c.cancel()
	}
	return c.Context.Err()
}

func TestReplayVertexBounds(t *testing.T) {
	a, b := &topo.Face{}, &topo.Face{}
	m := &topo.Mesh{Vertices: make([]r3.Vec, 4), Triangles: [][3]int{{0, 1, 2}, {1, 2, 3}}, Source: []*topo.Face{a, b}, FaceBound: map[*topo.Face]float64{a: .1, b: .2}}
	want, err := VertexBounds(m)
	if err != nil {
		t.Fatal(err)
	}
	run := func(ctx context.Context, budget *proofbound.ReplayBudget) ([]float64, error) {
		return ReplayVertexBounds(ctx, m, budget)
	}
	base := proofbound.NewReplayTrialBudget()
	got, err := run(context.Background(), base)
	if err != nil || len(got) != len(want) {
		t.Fatalf("count=%d error=%v", len(got), err)
	}
	for i := range got {
		if got[i] != want[i] {
			t.Fatalf("vertex %d differs", i)
		}
	}
	w, r := base.Usage()
	bytes := base.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		pass    bool
	}{{"exact", w, r, bytes, true}, {"work short", w - 1, r, bytes, false}, {"records short", w, r - 1, bytes, false}, {"bytes short", w, r, bytes - 1, false}} {
		t.Run(tc.name, func(t *testing.T) {
			m, e := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			if e != nil {
				t.Fatal(e)
			}
			out, e := run(context.Background(), m)
			if tc.pass && (e != nil || len(out) != 4) {
				t.Fatalf("exact error=%v", e)
			}
			if !tc.pass && (e == nil || out != nil) {
				t.Fatalf("rollback error=%v", e)
			}
		})
	}
	m.VertexBound = []float64{.2, .3, .4, .5}
	cloned, err := run(context.Background(), proofbound.NewReplayTrialBudget())
	if err != nil {
		t.Fatal(err)
	}
	m.VertexBound[0] = 9
	if cloned[0] != .2 {
		t.Fatal("copy aliases source")
	}
	m.VertexBound = []float64{math.NaN(), 0, 0, 0}
	if out, err := run(context.Background(), proofbound.NewReplayTrialBudget()); err == nil || out != nil {
		t.Fatal("nonfinite vertex bound accepted")
	}
	m.VertexBound = nil
	delete(m.FaceBound, b)
	if out, err := run(context.Background(), proofbound.NewReplayTrialBudget()); err == nil || out != nil {
		t.Fatal("missing face bound accepted")
	}
	m.FaceBound[b] = .2
	for len(m.Triangles) < 32 {
		m.Triangles = append(m.Triangles, m.Triangles[0])
		m.Source = append(m.Source, a)
	}
	lateBase, lateCancel := context.WithCancel(context.Background())
	defer lateCancel()
	late := &vertexReplayCancel{Context: lateBase, cancel: lateCancel}
	if out, err := run(late, proofbound.NewReplayTrialBudget()); err == nil || out != nil || late.polls.Load() < 10 {
		t.Fatalf("late cancellation polls=%d error=%v", late.polls.Load(), err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if out, err := run(ctx, proofbound.NewReplayTrialBudget()); err == nil || out != nil {
		t.Fatal("cancelled audit succeeded")
	}
}
