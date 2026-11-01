package prismtess

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// These refusals are constructed once so rejected replay does not allocate.
var (
	errAreaReplay1 = fmt.Errorf("%w: invalid facet vertex", decaderr.ErrUnsupported)
	errAreaReplay2 = fmt.Errorf("%w: nonfinite store area allowance", decaderr.ErrUnsupported)
	errAreaReplay3 = fmt.Errorf("%w: invalid perturbed area input", decaderr.ErrUnsupported)
	errAreaReplay4 = fmt.Errorf("%w: nonfinite perturbed area", decaderr.ErrUnsupported)
)

func storeAreaReplay(ctx context.Context, mesh *topo.Mesh, store []float64, b *proofbound.ReplayBudget) (float64, error) {
	if mesh == nil || b == nil {
		return 0, decaderr.ErrUnsupported
	}
	if err := b.SpendWork(ctx, 16); err != nil {
		return 0, err
	}
	total := 0.0
	for _, tri := range mesh.Triangles {
		if err := b.SpendWork(ctx, 128); err != nil {
			return 0, err
		}
		for _, i := range tri {
			if i < 0 || i >= len(store) || i >= len(mesh.Vertices) {
				return 0, errAreaReplay1
			}
		}
		for _, i := range tri {
			if store[i] < 0 || proofbound.IsNonFinite(store[i]) || !proofbound.FiniteVec(mesh.Vertices[i]) {
				return 0, decaderr.ErrUnsupported
			}
		}
		d := max(store[tri[0]], store[tri[1]], store[tri[2]])
		a, p, q := mesh.Vertices[tri[0]], mesh.Vertices[tri[1]], mesh.Vertices[tri[2]]
		// Coordinate L1 sums bound both exact edge lengths without cancellation.
		length := proofbound.AbsSumUpper(a.X, a.Y, a.Z, a.X, a.Y, a.Z, p.X, p.Y, p.Z, q.X, q.Y, q.Z)
		allow := proofbound.AbsSumUpper(proofbound.ProductUpper(d, length), proofbound.ProductUpper(2, proofbound.ProductUpper(d, d)))
		total = proofbound.AbsSumUpper(total, allow)
	}
	if proofbound.IsNonFinite(total) {
		return 0, errAreaReplay2
	}
	return total, nil
}

func perturbedAreaReplay(ctx context.Context, mesh *topo.Mesh, d float64, b *proofbound.ReplayBudget) (float64, error) {
	if mesh == nil || b == nil || d < 0 || proofbound.IsNonFinite(d) {
		return 0, errAreaReplay3
	}
	total := 0.0
	for _, tri := range mesh.Triangles {
		if err := b.SpendWork(ctx, 128); err != nil {
			return 0, err
		}
		for _, i := range tri {
			if i < 0 || i >= len(mesh.Vertices) {
				return 0, errAreaReplay1
			}
		}
		a, p, q := mesh.Vertices[tri[0]], mesh.Vertices[tri[1]], mesh.Vertices[tri[2]]
		if !proofbound.FiniteVec(a) || !proofbound.FiniteVec(p) || !proofbound.FiniteVec(q) {
			return 0, decaderr.ErrUnsupported
		}
		u := proofbound.AbsSumUpper(a.X, a.Y, a.Z, p.X, p.Y, p.Z)
		v := proofbound.AbsSumUpper(a.X, a.Y, a.Z, q.X, q.Y, q.Z)
		area := proofbound.ProductUpper(.5, proofbound.ProductUpper(u, v))
		allow := proofbound.AbsSumUpper(proofbound.ProductUpper(d, proofbound.AbsSumUpper(u, v)), proofbound.ProductUpper(2, proofbound.ProductUpper(d, d)))
		total = proofbound.AbsSumUpper(total, area, allow)
	}
	if proofbound.IsNonFinite(total) {
		return 0, errAreaReplay4
	}
	return total, nil
}
