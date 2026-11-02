package tessellation

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ReplayFaceBounds computes bounds in certifiedFaces order. Every source must
// identify one distinct certified face and every certified face must occur.
// The caller must authenticate that table and each supplied displacement.
func ReplayFaceBounds[F comparable](ctx context.Context, triangles [][3]int, source, certifiedFaces []F,
	trim, axial, store []float64, section float64, budget *proofbound.ReplayBudget) ([]float64, error) {
	if budget == nil {
		return nil, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, err
	}
	if len(triangles) != len(source) || len(certifiedFaces) != len(trim) || len(trim) != len(axial) || proofbound.IsNonFinite(section) || section < 0 {
		return nil, decaderr.ErrUnsupported
	}
	n := len(certifiedFaces)
	if uint64(n) > ^uint64(0)/2 {
		return nil, decaderr.ErrUnsupported
	}
	if err := budget.ReserveRecords(ctx, uint64(n)*2); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(false))); err != nil {
		return nil, err
	}
	bounds := make([]float64, n)
	used := make([]bool, n)
	for i := range certifiedFaces {
		if err := budget.SpendWork(ctx, 16); err != nil {
			return nil, err
		}
		if proofbound.IsNonFinite(trim[i]) || trim[i] < 0 || proofbound.IsNonFinite(axial[i]) || axial[i] < 0 {
			return nil, decaderr.ErrUnsupported
		}
		for j := range i {
			if err := budget.SpendWork(ctx, 8); err != nil {
				return nil, err
			}
			if certifiedFaces[i] == certifiedFaces[j] {
				return nil, decaderr.ErrUnsupported
			}
		}
	}
	for i := range source {
		if err := budget.SpendWork(ctx, 8); err != nil {
			return nil, err
		}
		f := source[i]
		found := -1
		for j := range certifiedFaces {
			if err := budget.SpendWork(ctx, 8); err != nil {
				return nil, err
			}
			if f == certifiedFaces[j] {
				found = j
				break
			}
		}
		if found < 0 {
			return nil, decaderr.ErrUnsupported
		}
		for _, v := range triangles[i] {
			if err := budget.SpendWork(ctx, 8); err != nil {
				return nil, err
			}
			if v < 0 || v >= len(store) || proofbound.IsNonFinite(store[v]) || store[v] < 0 {
				return nil, decaderr.ErrUnsupported
			}
			bounds[found] = math.Max(bounds[found], store[v])
		}
		used[found] = true
	}
	for i := range bounds {
		if err := budget.SpendWork(ctx, 16); err != nil {
			return nil, err
		}
		if !used[i] {
			return nil, decaderr.ErrUnsupported
		}
		bounds[i] = proofbound.UpRound(trim[i] + bounds[i] + section + axial[i])
		if proofbound.IsNonFinite(bounds[i]) {
			return nil, decaderr.ErrUnsupported
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return bounds, nil
}
