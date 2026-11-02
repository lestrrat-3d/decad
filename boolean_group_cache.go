package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
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

// sameBlindMesh compares every exported and proof field after a fresh replay.
// The owned handle is checked separately by the root cache adapter.
func sameBlindMesh(ctx context.Context, a, b *topo.Mesh, budget *proofbound.ReplayBudget) (bool, error) {
	if err := budget.SpendWork(ctx, 64); err != nil {
		return false, err
	}
	if a == nil || b == nil || len(a.Vertices) != len(b.Vertices) || len(a.Triangles) != len(b.Triangles) ||
		len(a.Source) != len(b.Source) || len(a.VertexBound) != len(b.VertexBound) || len(a.FaceBound) != len(b.FaceBound) ||
		len(a.Vertices) > 65536 || len(a.Triangles) > 4096 || len(a.Source) > 4096 ||
		len(a.VertexBound) > 65536 || len(a.FaceBound) > 4096 {
		return false, nil
	}
	work := 16*uint64(len(a.Vertices)) + 4*uint64(len(a.Triangles)+len(a.Source)+len(a.VertexBound)+len(a.FaceBound))
	if err := budget.SpendWork(ctx, work); err != nil {
		return false, err
	}
	if math.Float64bits(a.Bound) != math.Float64bits(b.Bound) ||
		math.Float64bits(a.CoordBound) != math.Float64bits(b.CoordBound) ||
		math.Float64bits(a.AreaSlack) != math.Float64bits(b.AreaSlack) ||
		math.Float64bits(a.VolSymDiff) != math.Float64bits(b.VolSymDiff) ||
		a.SymDiffOK != b.SymDiffOK || a.BoundaryOK != b.BoundaryOK {
		return false, nil
	}
	for i, v := range a.Vertices {
		w := b.Vertices[i]
		if math.Float64bits(v.X) != math.Float64bits(w.X) || math.Float64bits(v.Y) != math.Float64bits(w.Y) ||
			math.Float64bits(v.Z) != math.Float64bits(w.Z) {
			return false, nil
		}
	}
	for i := range a.Triangles {
		if a.Triangles[i] != b.Triangles[i] {
			return false, nil
		}
	}
	for i := range a.Source {
		if a.Source[i] != b.Source[i] {
			return false, nil
		}
	}
	for i := range a.VertexBound {
		if math.Float64bits(a.VertexBound[i]) != math.Float64bits(b.VertexBound[i]) {
			return false, nil
		}
	}
	for face, bound := range a.FaceBound {
		other, ok := b.FaceBound[face]
		if !ok || math.Float64bits(bound) != math.Float64bits(other) {
			return false, nil
		}
	}
	return true, ctx.Err()
}

// replayBlindCachedMesh returns a hit only after replaying the bound source and
// comparing the complete cached mesh with the freshly verified result.
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
	equal, err := sameBlindMesh(ctx, fresh, entry.Mesh, budget)
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
