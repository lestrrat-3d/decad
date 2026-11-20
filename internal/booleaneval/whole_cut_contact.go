package booleaneval

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// WholeCutBounds returns finite-patch bounds valid wherever any part of
// the prepared tool can contact the specified target facets. The native
// source checks include each tool facet's original displacement.
type WholeCutBounds func(context.Context, *topo.Mesh, *topo.Mesh, *proofbound.ReplayBudget) (map[int]meshbool.CutFacetBound, bool, error)
type wholeCutKey struct{}
type wholeCutTrialKey struct{}
type wholeCutSource struct {
	a, b   *topo.Body
	meter  *proofbound.ReplayBudget
	bounds WholeCutBounds
}

// WithWholeCutBounds scopes a surface proof to one ordered ordinary Cut.
func WithWholeCutBounds(ctx context.Context, a, b *topo.Body, m *proofbound.ReplayBudget, f WholeCutBounds) context.Context {
	return context.WithValue(ctx, wholeCutKey{}, wholeCutSource{a, b, m, f})
}

func bindWholeCut(ctx context.Context, a, b *topo.Body, ma, mb *topo.Mesh, bm *meshbool.BoolMesh) (bool, error) {
	s, ok := ctx.Value(wholeCutKey{}).(wholeCutSource)
	if !ok || a == nil || b == nil || a == b || a != s.a || b != s.b || s.meter == nil || s.bounds == nil {
		return false, nil
	}
	for _, part := range []struct {
		body *topo.Body
		mesh *topo.Mesh
	}{{a, ma}, {b, mb}} {
		if part.mesh == nil || len(part.mesh.Triangles) == 0 || len(part.mesh.Triangles) != len(part.mesh.Source) {
			return false, nil
		}
		if err := s.meter.SpendWork(ctx, uint64(len(part.mesh.Source))); err != nil {
			return false, err
		}
		for _, f := range part.mesh.Source {
			if f == nil || f.Body != part.body {
				return false, nil
			}
		}
	}
	m, ok, err := s.bounds(ctx, ma, mb, s.meter)
	if err != nil || !ok || len(m) == 0 {
		return false, err
	}
	if err := s.meter.SpendWork(ctx, uint64(len(m))); err != nil {
		return false, err
	}
	if len(bm.FacetBound) != len(ma.Triangles) || len(bm.SlabBound) != len(ma.Triangles) {
		return false, nil
	}
	if err := s.meter.ReserveBundle(ctx, uint64(len(m)), uint64(len(m)), uint64(len(m))*64); err != nil {
		return false, err
	}
	levels := make(map[int]float64, len(m))
	for i, v := range m {
		if i < 0 || i >= len(ma.Triangles) || proofbound.IsNonFinite(v.Distance) || v.Distance < 0 || v.Distance > bm.FacetBound[i] {
			return false, nil
		}
		level := v.Distance
		if v.Preserve {
			level = max(level, bm.SlabBound[i])
		}
		levels[i] = level
	}
	bm.CutGateBounds = levels
	return true, nil
}
