package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/sweeptransport"
	"github.com/lestrrat-3d/r3"
)

func (d *Document) sweepCompositeChain(ctx context.Context, chain chainRecord, plane planeRecord,
	path *sweepinput.Path) (*Body, error) {
	if err := sweepinput.CardinalChainSweepClass(chain, plane, path); err != nil {
		return nil, err
	}
	frames, err := sweeptransport.TransportSweepFramesContext(ctx, path, plane)
	if err != nil {
		return nil, err
	}
	if len(frames) != 3 {
		return nil, fmt.Errorf(`%w: composite chain Sweep has no complete transported frame chain`, ErrUnsupported)
	}
	for i, frame := range frames {
		if frame.OriginBound != 0 || frame.UBound != 0 || frame.VBound != 0 || frame.NBound != 0 {
			return nil, fmt.Errorf(`%w: composite chain Sweep frame %d has a nonzero displacement`, ErrUnsupported, i)
		}
	}
	work := freeform.NewFreeformWork()
	view := profileRecord{Outer: loopRecord(chain)}
	straight, err := sweeprecord.LineSpan(view, frames[0].Frame, path.Records[0])
	if err != nil {
		return nil, err
	}
	arcPlane := planeRecord{Origin: frames[1].Frame.Origin(), U: frames[1].Frame.U(), V: frames[1].Frame.V()}
	arc, err := sweeprecord.ArcSpan(ctx, view, arcPlane, frames[1].Frame, path.Records[1], work)
	if err != nil {
		return nil, err
	}
	payload := chainSweepPayload{Path: path, Spans: []chainSweepSpanPayload{
		{Chain: chainPayload{
			Chains: []chainRecord{chain}, Frame: straight.Prism.Frame,
			Z0: straight.Prism.Z0, Z1: straight.Prism.Z1,
			Z0Delta: straight.Prism.Z0Delta, Z1Delta: straight.Prism.Z1Delta,
			Xform: r3.Identity(),
		}},
		{Revolve: chainRevolvePayload{
			Chains: []chainRecord{chain}, Frame: arc.Revolve.Frame, Ax: arc.Revolve.Ax,
			Phi0: arc.Revolve.Phi0, Phi1: arc.Revolve.Phi1,
			Den: arc.Revolve.Den, Xform: r3.Identity(),
		}, Arc: true, ReverseArcRims: arc.ReverseArcCaps},
	}}
	ref := d.nextProducerID()
	body, err := bodyResult(sweeprecord.ReplayChainWork(ctx, envOf(documentNode(d), ref), payload, work))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

// sweepArcChain reduces one spatial arc and one open Sketch walk to the same
// analytic chain Revolve wall used by a standalone chain Revolve. The path
// owns the axis and angle; the open source walk still determines sheet kind
// and free rims.
func (d *Document) sweepArcChain(ctx context.Context, chain chainRecord,
	plane planeRecord, frame r3.Frame, path *sweepinput.Path) (*Body, error) {
	work := freeform.NewFreeformWork()
	view := profileRecord{Outer: loopRecord(chain)}
	span, err := sweeprecord.ArcSpan(ctx, view, plane, frame, path.Records[0], work)
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
	sweeprecord.FinishChain(bodyNode(body), chainSweepPayload{Revolve: reduction, Arc: true, Path: path})
	d.commit(body)
	return body, nil
}
