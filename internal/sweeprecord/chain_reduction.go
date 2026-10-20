package sweeprecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweeptransport"
	"github.com/lestrrat-3d/r3"
)

// CompositeChainPayload reduces one open Sketch walk swept along the admitted
// straight-then-arc path to its two-span record: the cardinal class gate, the
// transported frame chain (exactly three frames, each exact), the straight
// span's chain prism and the arc span's chain revolve. The caller replays the
// returned payload with ReplayChainWork and the returned work.
func CompositeChainPayload(ctx context.Context, chain sectionrecord.ChainRecord, plane sectionrecord.PlaneRecord,
	path *sweepinput.Path) (ChainPayload, *freeform.FreeformWork, error) {
	if err := sweepinput.CardinalChainSweepClass(chain, plane, path); err != nil {
		return ChainPayload{}, nil, err
	}
	frames, err := sweeptransport.TransportSweepFramesContext(ctx, path, plane)
	if err != nil {
		return ChainPayload{}, nil, err
	}
	if len(frames) != 3 {
		return ChainPayload{}, nil, fmt.Errorf(`%w: composite chain Sweep has no complete transported frame chain`, decaderr.ErrUnsupported)
	}
	for i, frame := range frames {
		if frame.OriginBound != 0 || frame.UBound != 0 || frame.VBound != 0 || frame.NBound != 0 {
			return ChainPayload{}, nil, fmt.Errorf(`%w: composite chain Sweep frame %d has a nonzero displacement`, decaderr.ErrUnsupported, i)
		}
	}
	work := freeform.NewFreeformWork()
	view := momentinput.Profile{Outer: sectionrecord.LoopRecord(chain)}
	straight, err := LineSpan(view, frames[0].Frame, path.Records[0])
	if err != nil {
		return ChainPayload{}, nil, err
	}
	arcPlane := sectionrecord.PlaneRecord{Origin: frames[1].Frame.Origin(), U: frames[1].Frame.U(), V: frames[1].Frame.V()}
	arc, err := ArcSpan(ctx, view, arcPlane, frames[1].Frame, path.Records[1], work)
	if err != nil {
		return ChainPayload{}, nil, err
	}
	return ChainPayload{Path: path, Spans: []ChainSpanPayload{
		{Chain: chainrecord.Payload{
			Chains: []sectionrecord.ChainRecord{chain}, Frame: straight.Prism.Frame,
			Z0: straight.Prism.Z0, Z1: straight.Prism.Z1,
			Z0Delta: straight.Prism.Z0Delta, Z1Delta: straight.Prism.Z1Delta,
			Xform: r3.Identity(),
		}},
		{Revolve: revolveplan.ChainPayload{
			Chains: []sectionrecord.ChainRecord{chain}, Frame: arc.Revolve.Frame, Ax: arc.Revolve.Ax,
			Phi0: arc.Revolve.Phi0, Phi1: arc.Revolve.Phi1,
			Den: arc.Revolve.Den, Xform: r3.Identity(),
		}, Arc: true, ReverseArcRims: arc.ReverseArcCaps},
	}}, work, nil
}

// ArcChainReduction reduces one spatial arc and one open Sketch walk to the
// same analytic chain Revolve wall a standalone chain Revolve uses. The path
// owns the axis and angle; the open source walk still determines sheet kind
// and free rims.
func ArcChainReduction(ctx context.Context, chain sectionrecord.ChainRecord, plane sectionrecord.PlaneRecord,
	frame r3.Frame, path *sweepinput.Path) (revolveplan.ChainPayload, *freeform.FreeformWork, error) {
	work := freeform.NewFreeformWork()
	view := momentinput.Profile{Outer: sectionrecord.LoopRecord(chain)}
	span, err := ArcSpan(ctx, view, plane, frame, path.Records[0], work)
	if err != nil {
		return revolveplan.ChainPayload{}, nil, err
	}
	arc := span.Revolve
	return revolveplan.ChainPayload{
		Chains: []sectionrecord.ChainRecord{chain}, Frame: arc.Frame, Ax: arc.Ax,
		Phi0: arc.Phi0, Phi1: arc.Phi1, Den: arc.Den, Xform: r3.Identity(),
	}, work, nil
}
