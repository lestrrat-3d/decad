package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// blindTessBinding ties a cached entry to the body and committed operation.
type blindTessBinding struct {
	body      *Body
	operation *Operation
	entry     *topo.TessEntry
	mesh      *topo.Mesh
}

// replayBlindCachedMesh adapts Body.blindProof, the operation handle and
// Mesh.Owner to the internal replay and mesh snapshot comparison.
func replayBlindCachedMesh(ctx context.Context, body *Body, chord float64, entry *topo.TessEntry,
	budget *proofbound.ReplayBudget) (*Mesh, bool, error) {
	if body == nil || body.blindProof == nil || entry == nil || budget == nil {
		return nil, false, nil
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return nil, false, err
	}
	binding := body.blindProof.cache.Load()
	if binding == nil || binding.entry != entry {
		return nil, false, nil
	}
	if binding.body != body || binding.operation != body.blindProof.cut ||
		binding.mesh != entry.Mesh {
		return nil, false, decaderr.ErrUnsupported
	}
	fresh, bounded, err := groupBlindVerifyAllWithBudget(ctx, body, chord, budget)
	if err != nil {
		return nil, false, err
	}
	if !bounded || fresh == nil {
		return nil, false, decaderr.ErrUnsupported
	}
	equal, err := meshbool.EqualMeshSnapshots(ctx, fresh, entry.Mesh, budget)
	if err != nil {
		return nil, false, err
	}
	if !equal {
		return nil, false, decaderr.ErrUnsupported
	}
	mesh, ok := entry.Mesh.Owner.(*Mesh)
	if !ok || mesh == nil || &mesh.n != entry.Mesh {
		return nil, false, decaderr.ErrUnsupported
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	return mesh, true, nil
}
