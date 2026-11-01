package topobuild_test

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

func TestSingleShellWindingReplay(t *testing.T) {
	mesh := &topo.Mesh{Triangles: [][3]int{{0, 1, 2}}, Source: []*topo.Face{{}}}
	lumps := []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: mesh.Source}}}}
	var budget proofbound.ReplayBudget
	if err := topobuild.RequireSingleShellWindingReplay(t.Context(), mesh, lumps, &budget); err != nil {
		t.Fatal(err)
	}
	if err := topobuild.RequireShellWinding(t.Context(), mesh, lumps); err != nil {
		t.Fatal(err)
	}
	work, _ := budget.Usage()
	if work == 0 {
		t.Fatal("shell traversal was not charged")
	}
	lumps[0].Shells = append(lumps[0].Shells, &topo.FaceShell{})
	if err := topobuild.RequireSingleShellWindingReplay(t.Context(), mesh, lumps, &budget); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("multiple shells: %v", err)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if err := topobuild.RequireSingleShellWindingReplay(ctx, mesh, lumps, &budget); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancelled: %v", err)
	}
	var exhausted proofbound.ReplayBudget
	if err := exhausted.SpendWork(t.Context(), proofbound.MaxMultiwayWorkPerCall); err != nil {
		t.Fatal(err)
	}
	if err := topobuild.RequireSingleShellWindingReplay(t.Context(), mesh, lumps, &exhausted); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("work cap: %v", err)
	}
}
