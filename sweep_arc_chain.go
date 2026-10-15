package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/r3"
)

// sweepArcChain reduces one spatial arc and one open Sketch walk to the same
// analytic chain Revolve wall used by a standalone chain Revolve. The path
// owns the axis and angle; the open source walk still determines sheet kind
// and free rims.
func (d *Document) sweepArcChain(ctx context.Context, chain chainRecord,
	plane planeRecord, frame r3.Frame, path *sweepinput.Path) (*Body, error) {
	work := freeform.NewFreeformWork()
	view := profileRecord{Outer: loopRecord(chain)}
	span, err := compositeArcSweepSpan(ctx, view, plane, frame, path.Records[0], work)
	if err != nil {
		return nil, err
	}
	arc := span.Revolve
	reduction := chainRevolvePayload{
		Chains: []chainRecord{chain}, Frame: arc.Frame, Ax: arc.Ax,
		Phi0: arc.Phi0, Phi1: arc.Phi1, Den: arc.Den, Xform: r3.Identity(),
	}
	ref := d.nextProducerID()
	body, err := bodyResult(revolveplan.BuildChain(ctx, envOf(documentNode(d), ref), reduction, work))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	finishChainSweepBody(body, chainSweepPayload{Revolve: reduction, Arc: true, Path: path})
	d.commit(body)
	return body, nil
}
