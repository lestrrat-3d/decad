package revolveplan

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
)

// BlendAxis reruns the revolve axis gates (agent-docs/evaluator-design.md §6)
// on a rewritten meridian about the receiver's own oriented axis: the side
// gate, the axis-contact audit and the snap allowances, each with the sentinel
// the base Revolve call gives it — a new interior axis contact is
// decaderr.ErrDegenerate, a blend arc centred across the axis (a spindle torus) is
// ErrUnsupported. The receiver's axis already has the region on its
// non-negative side, so a rewrite whose region the gate puts on the far side
// has crossed the axis and is decaderr.ErrDegenerate.
func (rp Payload) BlendAxis(ctx context.Context, profile momentinput.Profile, work *freeform.FreeformWork) (revolveaxis.Frame, error) {
	ax, side, err := ResolveAxisSide(ctx, profile, revolveaxis.Line2{
		AU: rp.Ax.AU, AV: rp.Ax.AV, AUBound: rp.Ax.AUBound, AVBound: rp.Ax.AVBound,
		DU: rp.Ax.DU, DV: rp.Ax.DV, DUBound: rp.Ax.DUBound, DVBound: rp.Ax.DVBound,
	}, work)
	if err != nil {
		return revolveaxis.Frame{}, err
	}
	if side < 0 {
		return revolveaxis.Frame{}, fmt.Errorf(`%w: the rewritten meridian lies across the revolve axis from the receiver's`, decaderr.ErrDegenerate)
	}
	return ax, nil
}
