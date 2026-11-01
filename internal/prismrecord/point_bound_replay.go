package prismrecord

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PointBoundSignedReplay evaluates a bounded point lift after proving that
// both recorded bases are signed coordinate permutations with unit stretch.
func (pp Payload) PointBoundSignedReplay(ctx context.Context, u, v, z proofbound.BoundedScalar,
	budget *proofbound.ReplayBudget) (r3.Vec, float64, error) {
	if budget == nil {
		return r3.Vec{}, 0, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 128); err != nil {
		return r3.Vec{}, 0, err
	}
	basis := pp.Xform.Basis()
	if !replaySignedBasis([3]r3.Vec{pp.Frame.U(), pp.Frame.V(), pp.Frame.N()}) ||
		!replaySignedBasis([3]r3.Vec{basis.EX, basis.EY, basis.EZ}) {
		return r3.Vec{}, 0, decaderr.ErrUnsupported
	}
	held, round, err := pp.LiftedVertexReplay(ctx, u.Value, v.Value, z.Value, budget)
	if err != nil {
		return r3.Vec{}, 0, err
	}
	if err := budget.SpendWork(ctx, 4); err != nil {
		return r3.Vec{}, 0, err
	}
	source := proofbound.Radius3D(max(u.Bound, v.Bound, z.Bound))
	return held, proofbound.AbsSumUpper(proofbound.ProductUpper(1, source), round), nil
}
