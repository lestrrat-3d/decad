package topobuild

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func TestReplayRequireSingleClosedShell(t *testing.T) {
	a, b := &topo.Face{}, &topo.Face{}
	m := &topo.Mesh{Triangles: [][3]int{{0, 1, 2}, {0, 2, 3}}, Source: []*topo.Face{a, b}}
	shell := &topo.FaceShell{Faces: []*topo.Face{a, b}}
	lumps := []*topo.Lump{{Shells: []*topo.FaceShell{shell}}}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) error {
		return ReplayRequireSingleClosedShell(ctx, m, lumps, b)
	}
	baseline := proofbound.NewReplayTrialBudget()
	if err := run(context.Background(), baseline); err != nil {
		t.Fatal(err)
	}
	w, r := baseline.Usage()
	if r != 0 || baseline.Bytes() != 0 {
		t.Fatalf("records=%d bytes=%d", r, baseline.Bytes())
	}
	exact, err := proofbound.NewReplayBudgetLimits(w, 1, 1)
	if err != nil {
		t.Fatal(err)
	}
	if err := run(context.Background(), exact); err != nil {
		t.Fatal(err)
	}
	short, err := proofbound.NewReplayBudgetLimits(w-1, 1, 1)
	if err != nil {
		t.Fatal(err)
	}
	if err := run(context.Background(), short); err == nil {
		t.Fatal("one-short work succeeded")
	}
	m.Source[1] = &topo.Face{}
	if err := run(context.Background(), proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("unowned face accepted")
	}
	m.Source[1] = b
	shell.Faces[1] = a
	if err := run(context.Background(), proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("duplicate face accepted")
	}
	shell.Faces[1] = b
	shell.Faces = append(shell.Faces, &topo.Face{})
	if err := run(context.Background(), proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("unmeshed shell face accepted")
	}
	shell.Faces = shell.Faces[:2]
	shell.Open = true
	if err := run(context.Background(), proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("open shell accepted")
	}
	shell.Open = false
	shell.Void = true
	if err := run(context.Background(), proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("void shell accepted")
	}
	shell.Void = false
	lumps[0].Shells = append(lumps[0].Shells, &topo.FaceShell{})
	if err := run(context.Background(), proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("multiple shells accepted")
	}
	lumps[0].Shells = lumps[0].Shells[:1]
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if err := run(ctx, proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("cancellation ignored")
	}
}

func TestReplayRequirePositiveSingleShell(t *testing.T) {
	f := &topo.Face{}
	m := &topo.Mesh{Vertices: []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}, Triangles: [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}}, Source: []*topo.Face{f, f, f, f}}
	lumps := []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: []*topo.Face{f}}}}}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) error {
		return ReplayRequirePositiveSingleShell(ctx, m, lumps, b)
	}
	base := proofbound.NewReplayTrialBudget()
	if err := run(context.Background(), base); err != nil {
		t.Fatal(err)
	}
	w, r := base.Usage()
	n := base.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, n uint64
		pass    bool
	}{{"exact", w, r, n, true}, {"work short", w - 1, r, n, false}, {"records short", w, r - 1, n, false}, {"bytes short", w, r, n - 1, false}} {
		t.Run(tc.name, func(t *testing.T) {
			b, err := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.n)
			if err != nil {
				t.Fatal(err)
			}
			if err := run(context.Background(), b); (err == nil) != tc.pass {
				t.Fatalf("error=%v", err)
			}
		})
	}
	for i := range m.Triangles {
		m.Triangles[i][1], m.Triangles[i][2] = m.Triangles[i][2], m.Triangles[i][1]
	}
	if err := run(context.Background(), proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("inward winding accepted")
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if err := run(ctx, proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("cancellation ignored")
	}
}
