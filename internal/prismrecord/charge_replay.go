package prismrecord

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// ChargeMapSignedReplay proves the placed section map is an exact signed
// permutation. Its determinant magnitude and stretch are exactly one and zero,
// so ordinary ChargeMap would leave every reading unchanged.
func ChargeMapSignedReplay(ctx context.Context, body *topo.Body, frame r3.Frame,
	xform r3.Transform, budget *proofbound.ReplayBudget) error {
	if budget == nil || body == nil {
		return decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 128); err != nil {
		return err
	}
	basis := xform.Basis()
	if !replaySignedBasis([3]r3.Vec{frame.U(), frame.V(), frame.N()}) ||
		!replaySignedBasis([3]r3.Vec{basis.EX, basis.EY, basis.EZ}) {
		return decaderr.ErrUnsupported
	}
	return nil
}
