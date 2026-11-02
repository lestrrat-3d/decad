package topobuild

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ReplayMeshBoundChecksResult contains only the checks named below. It is not
// a complete boundary, winding, facet-contact, or occupied-volume certificate.
type ReplayMeshBoundChecksResult struct {
	FaceBounds, VertexBounds          []float64
	StoreMax, StoreAreaAllow, SymDiff float64
}

// ReplayMeshBoundChecks runs the available isolated output checks on one meter.
// Callers must authenticate the mesh, every supplied source bound and every
// volume term, and separately complete all remaining geometric verification.
func ReplayMeshBoundChecks(ctx context.Context, m *topo.Mesh, lumps []*topo.Lump, faces []*topo.Face, trim, axial, store []float64,
	section float64, volumeTerms []float64, budget *proofbound.ReplayBudget) (*ReplayMeshBoundChecksResult, error) {
	if m == nil || budget == nil {
		return nil, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, err
	}
	// Force a derivation from the face table until an independent vertex record
	// can be bound to the same authenticated sources.
	if m.VertexBound != nil || len(store) != len(m.Vertices) || m.Bound < 0 || proofbound.IsNonFinite(m.Bound) {
		return nil, decaderr.ErrUnsupported
	}
	if err := tessellation.ReplayRequireClosedMesh(ctx, m.Triangles, budget); err != nil {
		return nil, err
	}
	if err := tessellation.ReplayRequireVertexLinks(ctx, len(m.Vertices), m.Triangles, budget); err != nil {
		return nil, err
	}
	if err := ReplayRequirePositiveSingleShell(ctx, m, lumps, budget); err != nil {
		return nil, err
	}
	maximum, err := tessellation.ReplayStoreMax(ctx, store, budget)
	if err != nil {
		return nil, err
	}
	bounds, err := tessellation.ReplayFaceBounds(ctx, m.Triangles, m.Source, faces, trim, axial, store, section, budget)
	if err != nil {
		return nil, err
	}
	for i := range faces {
		if err := budget.SpendWork(ctx, 16); err != nil {
			return nil, err
		}
		if faces[i] == nil {
			return nil, decaderr.ErrUnsupported
		}
		stored, ok := m.FaceBound[faces[i]]
		if !ok || stored != bounds[i] || stored > m.Bound {
			return nil, decaderr.ErrUnsupported
		}
	}
	vertices, err := ReplayVertexBounds(ctx, m, budget)
	if err != nil {
		return nil, err
	}
	area, err := tessellation.ReplayStoreAreaAllow(ctx, m.Vertices, m.Triangles, store, budget)
	if err != nil {
		return nil, err
	}
	volume, err := tessellation.ReplaySymDiff(ctx, volumeTerms, budget)
	if err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(ReplayMeshBoundChecksResult{}))); err != nil {
		return nil, err
	}
	return &ReplayMeshBoundChecksResult{FaceBounds: bounds, VertexBounds: vertices, StoreMax: maximum, StoreAreaAllow: area, SymDiff: volume}, nil
}
