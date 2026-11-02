package topobuild

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func TestReplayMeshBoundChecks(t *testing.T) {
	f := &topo.Face{}
	mesh := &topo.Mesh{Vertices: []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}, Triangles: [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}}, Source: []*topo.Face{f, f, f, f}}
	faces := []*topo.Face{f}
	lumps := []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: faces}}}}
	trim := []float64{.01}
	axial := []float64{.001}
	store := []float64{.0001, .0002, .0003, .0004}
	ordinary, err := tessellation.FaceBounds(mesh.Triangles, mesh.Source, map[*topo.Face]float64{f: trim[0]}, map[*topo.Face]float64{f: axial[0]}, store, .002)
	if err != nil {
		t.Fatal(err)
	}
	mesh.FaceBound = ordinary
	mesh.Bound = ordinary[f]
	terms := []float64{.1, .2}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) (*ReplayMeshBoundChecksResult, error) {
		return ReplayMeshBoundChecks(ctx, mesh, lumps, faces, trim, axial, store, .002, terms, b)
	}
	baseline := proofbound.NewReplayTrialBudget()
	result, err := run(context.Background(), baseline)
	if err != nil || result == nil || len(result.FaceBounds) != 1 || len(result.VertexBounds) != 4 || result.FaceBounds[0] != ordinary[f] {
		t.Fatalf("result error=%v", err)
	}
	for i, v := range result.VertexBounds {
		if v != ordinary[f] {
			t.Fatalf("vertex %d bound=%g", i, v)
		}
	}
	w, r := baseline.Usage()
	bytes := baseline.Bytes()
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
			if tc.pass && (e != nil || out == nil) {
				t.Fatalf("exact error=%v", e)
			}
			if !tc.pass && (e == nil || out != nil) {
				t.Fatalf("rollback error=%v", e)
			}
		})
	}
	terms[1] = math.NaN()
	if out, e := run(context.Background(), proofbound.NewReplayTrialBudget()); e == nil || out != nil {
		t.Fatal("nonfinite volume term accepted")
	}
	terms[1] = .2
	mesh.FaceBound[f]++
	if out, e := run(context.Background(), proofbound.NewReplayTrialBudget()); e == nil || out != nil {
		t.Fatal("changed face bound accepted")
	}
	mesh.FaceBound = ordinary
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if out, e := run(ctx, proofbound.NewReplayTrialBudget()); e == nil || out != nil {
		t.Fatal("cancelled checks succeeded")
	}
}
