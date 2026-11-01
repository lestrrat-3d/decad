package prismrecord

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// CentroidEnvelopeSignedReplay bounds every point in this prism from a
// producer-bound plane-local L1 envelope, without resolving profile walks.
func (pp Payload) CentroidEnvelopeSignedReplay(ctx context.Context, coordUpper float64,
	held r3.Vec, budget *proofbound.ReplayBudget) (float64, error) {
	if budget == nil {
		return 0, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 160); err != nil {
		return 0, err
	}
	basis := pp.Xform.Basis()
	if !replaySignedBasis([3]r3.Vec{pp.Frame.U(), pp.Frame.V(), pp.Frame.N()}) ||
		!replaySignedBasis([3]r3.Vec{basis.EX, basis.EY, basis.EZ}) ||
		proofbound.IsNonFinite(coordUpper) || coordUpper < 0 {
		return 0, decaderr.ErrUnsupported
	}
	zUpper := math.Max(math.Abs(pp.Z0), math.Abs(pp.Z1))
	frameUpper := proofbound.AbsSumUpper(vecL1(pp.Frame.Origin()),
		proofbound.ProductUpper(vecL1(pp.Frame.U()), coordUpper),
		proofbound.ProductUpper(vecL1(pp.Frame.V()), coordUpper),
		proofbound.ProductUpper(vecL1(pp.Frame.N()), zUpper))
	placedUpper := proofbound.AbsSumUpper(proofbound.ProductUpper(3, frameUpper),
		vecL1(pp.Xform.Translation()))
	bound := proofbound.AbsSumUpper(vecL1(held), placedUpper)
	if proofbound.IsNonFinite(bound) {
		return 0, decaderr.ErrUnsupported
	}
	return bound, nil
}
