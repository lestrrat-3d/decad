package tessellation

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ReplayStoreAreaAllow sums a conservative L1 bound on area movement from
// authenticated per-vertex displacements. It does not establish those bounds.
func ReplayStoreAreaAllow(ctx context.Context, vertices []r3.Vec, triangles [][3]int, store []float64, budget *proofbound.ReplayBudget) (float64, error) {
	if budget == nil {
		return 0, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return 0, err
	}
	total := 0.0
	for i := range triangles {
		if err := budget.SpendWork(ctx, 128); err != nil {
			return 0, err
		}
		tri := triangles[i]
		for _, j := range tri {
			if j < 0 || j >= len(vertices) || j >= len(store) || proofbound.IsNonFinite(store[j]) || store[j] < 0 || !proofbound.FiniteVec(vertices[j]) {
				return 0, decaderr.ErrUnsupported
			}
		}
		d := math.Max(store[tri[0]], math.Max(store[tri[1]], store[tri[2]]))
		a, b, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
		length := proofbound.AbsSumUpper(a.X, a.Y, a.Z, a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z)
		allow := proofbound.AbsSumUpper(proofbound.ProductUpper(d, length), proofbound.ProductUpper(2, proofbound.ProductUpper(d, d)))
		total = proofbound.AbsSumUpper(total, allow)
	}
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	if proofbound.IsNonFinite(total) {
		return 0, decaderr.ErrUnsupported
	}
	return total, nil
}
