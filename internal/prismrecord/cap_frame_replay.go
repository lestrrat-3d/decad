package prismrecord

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// CapFrameSignedReplay charges a cap's placed origin and checks both bases
// before evaluating the ordinary frame on an exact signed coordinate map.
func (pp Payload) CapFrameSignedReplay(ctx context.Context, z float64, flip bool,
	budget *proofbound.ReplayBudget) (r3.Frame, error) {
	if budget == nil {
		return r3.Frame{}, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 128); err != nil {
		return r3.Frame{}, err
	}
	basis := pp.Xform.Basis()
	if !replaySignedBasis([3]r3.Vec{pp.Frame.U(), pp.Frame.V(), pp.Frame.N()}) ||
		!replaySignedBasis([3]r3.Vec{basis.EX, basis.EY, basis.EZ}) {
		return r3.Frame{}, decaderr.ErrUnsupported
	}
	held, _, err := pp.LiftedVertexReplay(ctx, 0, 0, z, budget)
	if err != nil {
		return r3.Frame{}, err
	}
	frame, err := pp.CapFrame(z, flip)
	if err != nil {
		return r3.Frame{}, err
	}
	if !replaySameVec(frame.Origin(), held) {
		return r3.Frame{}, decaderr.ErrUnsupported
	}
	return frame, nil
}
