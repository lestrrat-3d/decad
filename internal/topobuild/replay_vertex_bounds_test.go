package topobuild_test

import (
	"context"
	"errors"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

func TestVertexBoundsReplay(t *testing.T) {
	face := new(topo.Face)
	m := &topo.Mesh{Vertices: []r3.Vec{{}, {X: 1}, {Y: 1}}, Triangles: [][3]int{{0, 1, 2}}, Source: []*topo.Face{face}, FaceBound: map[*topo.Face]float64{face: .2}}
	for _, explicit := range []bool{false, true} {
		if explicit {
			m.VertexBound = []float64{.1, .2, .3}
		}
		want, e := topobuild.VertexBounds(m)
		if e != nil {
			t.Fatal(e)
		}
		meter := proofbound.NewReplayTrialBudget()
		got, e := topobuild.VertexBoundsReplay(t.Context(), m, meter)
		if e != nil || !slices.Equal(got, want) {
			t.Fatalf("explicit=%t bounds differ: %v", explicit, e)
		}
		w, r := meter.Usage()
		z := meter.Bytes()
		for _, short := range []string{replayTestCaseExact, "work", "records", "bytes"} {
			b := proofbound.NewReplayTrialBudget()
			ww, rr, zz := proofbound.ReplayTrialWork-w, proofbound.ReplayTrialRecords-r, proofbound.ReplayTrialBytes-z
			switch short {
			case "work":
				ww++
			case "records":
				rr++
			case "bytes":
				zz++
			}
			if e = b.SpendWork(t.Context(), ww); e != nil {
				t.Fatal(e)
			}
			if e = b.ReserveRecords(t.Context(), rr); e != nil {
				t.Fatal(e)
			}
			if e = b.ReserveBytes(t.Context(), zz); e != nil {
				t.Fatal(e)
			}
			_, e = topobuild.VertexBoundsReplay(t.Context(), m, b)
			if short == replayTestCaseExact && e != nil || short != replayTestCaseExact && !errors.Is(e, decaderr.ErrUnsupported) {
				t.Fatalf("explicit=%t %s: %v", explicit, short, e)
			}
		}
	}
	m.VertexBound[0] = math.NaN()
	if _, e := topobuild.VertexBoundsReplay(t.Context(), m, proofbound.NewReplayTrialBudget()); e == nil {
		t.Fatal("nonfinite bound accepted")
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, e := topobuild.VertexBoundsReplay(ctx, m, proofbound.NewReplayTrialBudget()); !errors.Is(e, context.Canceled) {
		t.Fatalf("cancel=%v", e)
	}
}
