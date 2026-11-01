package revolvemesh

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// FacetAreasReplay validates held facets with one replay budget.
func FacetAreasReplay(ctx context.Context, verts []r3.Vec, tris [][3]int, delta float64, budget *proofbound.ReplayBudget) ([]RevolveAuditTri, error) {
	if budget == nil {
		return nil, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 32); err != nil {
		return nil, err
	}
	if delta < 0 || proofbound.IsNonFinite(delta) || len(tris) > 65536 {
		return nil, decaderr.ErrUnsupported
	}
	if err := budget.ReserveRecords(ctx, uint64(len(tris))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(tris)), uint64(unsafe.Sizeof(RevolveAuditTri{}))); err != nil {
		return nil, err
	}
	out := make([]RevolveAuditTri, len(tris))
	for i := range tris {
		if err := budget.SpendWork(ctx, 32); err != nil {
			return nil, err
		}
		t, err := newRevolveAuditTriReplay(ctx, verts, tris[i], budget)
		if err != nil {
			return nil, err
		}
		if err = requireRevolveFacetAreaReplay(ctx, t, i, delta, budget); err != nil {
			return nil, err
		}
		out[i] = t
	}
	return out, nil
}
