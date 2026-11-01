package revolvemesh

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// AuditMeshReplay checks held facet area and contacts from the supplied mesh.
// It does not assume a particular source face or skip same-face pairs.
func AuditMeshReplay(ctx context.Context, verts []r3.Vec, tris [][3]int, budget *proofbound.ReplayBudget) (uint64, error) {
	if budget == nil {
		return 0, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 32); err != nil {
		return 0, err
	}
	data, err := FacetAreasReplay(ctx, verts, tris, 0, budget)
	if err != nil {
		return 0, err
	}
	return revolveContactAuditReplay(ctx, data, tris, 0, 0, budget)
}
