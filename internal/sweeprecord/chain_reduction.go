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

// CompositeChainPayload reduces one open Sketch walk swept along a composite
// path to one chain span per path record: the path gate, the transported
// frame chain (one frame per join, each exact), a chain prism for every line
// and a chain revolve for every arc. Every span carries the same recorded
// walk, so the sew in ReplayChainWork can pair rims by recorded segment
// index. The caller replays the returned payload with ReplayChainWork and the
// returned work.
func CompositeChainPayload(ctx context.Context, chain sectionrecord.ChainRecord, plane sectionrecord.PlaneRecord,
	path *sweepinput.Path) (ChainPayload, *freeform.FreeformWork, error) {
	if err := sweepinput.CompositeChainSweepPath(path); err != nil {
		return ChainPayload{}, nil, err
	}
	frames, err := sweeptransport.TransportSweepFramesContext(ctx, path, plane)
	if err != nil {
		return ChainPayload{}, nil, err
	}
	if len(frames) != len(path.Records)+1 {
		return ChainPayload{}, nil, fmt.Errorf(`%w: composite chain Sweep has no complete transported frame chain`, decaderr.ErrUnsupported)
	}
	for i, frame := range frames {
		if frame.OriginBound != 0 || frame.UBound != 0 || frame.VBound != 0 || frame.NBound != 0 {
			return ChainPayload{}, nil, fmt.Errorf(`%w: composite chain Sweep frame %d has a nonzero displacement`, decaderr.ErrUnsupported, i)
		}
	}
	work := freeform.NewFreeformWork()
	view := momentinput.Profile{Outer: sectionrecord.LoopRecord(chain)}
	spans := make([]ChainSpanPayload, len(path.Records))
	for i, record := range path.Records {
		if err := ctx.Err(); err != nil {
			return ChainPayload{}, nil, err
		}
		current := frames[i].Frame
		if record.Arc == nil {
			straight, err := LineSpan(view, current, record)
			if err != nil {
				return ChainPayload{}, nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
			}
			spans[i] = ChainSpanPayload{Chain: chainrecord.Payload{
				Chains: []sectionrecord.ChainRecord{chain}, Frame: straight.Prism.Frame,
				Z0: straight.Prism.Z0, Z1: straight.Prism.Z1,
				Z0Delta: straight.Prism.Z0Delta, Z1Delta: straight.Prism.Z1Delta,
				Xform: r3.Identity(),
			}}
			continue
		}
		spanPlane := sectionrecord.PlaneRecord{Origin: current.Origin(), U: current.U(), V: current.V()}
		arc, err := ArcSpan(ctx, view, spanPlane, current, record, work)
		if err != nil {
			return ChainPayload{}, nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		spans[i] = ChainSpanPayload{Revolve: revolveplan.ChainPayload{
			Chains: []sectionrecord.ChainRecord{chain}, Frame: arc.Revolve.Frame, Ax: arc.Revolve.Ax,
			Phi0: arc.Revolve.Phi0, Phi1: arc.Revolve.Phi1,
			Den: arc.Revolve.Den, Xform: r3.Identity(),
		}, Arc: true, ReverseArcRims: arc.ReverseArcCaps}
	}
	return ChainPayload{Path: path, Spans: spans}, work, nil
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
