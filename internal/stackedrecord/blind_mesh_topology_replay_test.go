package stackedrecord

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func TestReplayBlindMeshTopologyRejectsPinchedVertex(t *testing.T) {
	triangles := [][3]int{
		{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3},
		{0, 4, 5}, {0, 5, 6}, {0, 6, 4}, {4, 6, 5},
	}
	if err := tessellation.ReplayRequireClosedMesh(t.Context(), triangles, proofbound.NewReplayTrialBudget()); err != nil {
		t.Fatalf("pinched mesh must be closed: %v", err)
	}
	mesh := &topo.Mesh{Vertices: make([]r3.Vec, 7), Triangles: triangles}
	out, ok, err := replayBlindMeshTopology(t.Context(), &topo.Body{}, ReplayBlindMeshAssembly{Mesh: mesh},
		proofbound.NewReplayTrialBudget())
	if ok || !errors.Is(err, decaderr.ErrUnsupported) || out.Mesh != nil || mesh.BoundaryOK {
		t.Fatalf("pinched mesh admitted=%t output mesh=%t boundary=%t error=%v", ok, out.Mesh != nil, mesh.BoundaryOK, err)
	}
}
