package booleaneval

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// PrepBoolMeshReplay performs the binary operand preparation on the shared counter.
func PrepBoolMeshReplay(ctx context.Context, m *topo.Mesh, src []int, retained bool,
	budget *proofbound.ReplayBudget) (*meshbool.BoolMesh, error) {
	if m == nil || budget == nil {
		return nil, errBrakeEvalBoundedMeshPreparationNeedsAMeshAndBudget
	}
	bm, err := meshbool.PrepContextReplay(ctx, m.Vertices, m.Triangles, src, budget)
	if err != nil {
		return nil, err
	}
	return finishBoolMeshReplay(ctx, m, bm, retained, budget)
}

func finishBoolMeshReplay(ctx context.Context, m *topo.Mesh, bm *meshbool.BoolMesh, retained bool,
	budget *proofbound.ReplayBudget) (*meshbool.BoolMesh, error) {
	vertices, facets := uint64(len(m.Vertices)), uint64(len(m.Triangles))
	// Vertex-bound construction visits three corners per facet; attaching
	// bounds and retained slabs each visit each facet once.
	if err := budget.SpendWork(ctx, vertices+5*facets); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, 1+vertices+2*facets); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 2*facets, uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(meshbool.ParityMesh{}))); err != nil {
		return nil, err
	}
	beta, err := topobuild.VertexBoundsReplay(ctx, m, budget)
	if err != nil {
		return nil, err
	}
	bm.SetVertexBounds(beta)
	if retained {
		if err := bm.SetChargedSlabBoundsReplay(ctx, retainedPlaneOf(m), budget); err != nil {
			return nil, err
		}
	}

	return bm, nil
}
