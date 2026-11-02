package topobuild

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ReplayVertexBounds copies an existing per-vertex proof or derives one from
// all incident source-face bounds. The caller authenticates the mesh and face
// bounds before using this reading in a replay.
func ReplayVertexBounds(ctx context.Context, m *topo.Mesh, budget *proofbound.ReplayBudget) ([]float64, error) {
	if m == nil || budget == nil {
		return nil, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, err
	}
	n := len(m.Vertices)
	if m.VertexBound != nil && len(m.VertexBound) != n {
		return nil, decaderr.ErrUnsupported
	}
	if len(m.Triangles) != len(m.Source) {
		return nil, decaderr.ErrUnsupported
	}
	if err := budget.ReserveRecords(ctx, uint64(n)); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return nil, err
	}
	out := make([]float64, n)
	if m.VertexBound != nil {
		for i := range m.VertexBound {
			if err := budget.SpendWork(ctx, 8); err != nil {
				return nil, err
			}
			d := m.VertexBound[i]
			if d < 0 || proofbound.IsNonFinite(d) {
				return nil, decaderr.ErrUnsupported
			}
			out[i] = d
		}
	} else {
		for i := range m.Triangles {
			if err := budget.SpendWork(ctx, 32); err != nil {
				return nil, err
			}
			face := m.Source[i]
			d, ok := m.FaceBound[face]
			if !ok || face == nil || d < 0 || proofbound.IsNonFinite(d) {
				return nil, decaderr.ErrUnsupported
			}
			for _, v := range m.Triangles[i] {
				if v < 0 || v >= n {
					return nil, decaderr.ErrUnsupported
				}
				out[v] = math.Max(out[v], d)
			}
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return out, nil
}
