package sweeprecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Payload deliberately remains distinct from prismPayload and
// revolvePayload. Downstream consumers whose Sweep proof has not landed must
// dispatch on this payload and refuse instead of treating the analytic
// reduction as proof for the original operation.
//
// SurfaceResult is WithSurfaceResult's own flag, held here too rather than
// read only off prism/revolve: every finishing step in finish.go overwrites the
// built body's payload with this struct, so a flag that rode only the reduced
// prism/revolve payload would still reach Kind()/solid correctly on first
// build (prismrecord.Build/revolveplan.Build read it there) but would give
// a later consumer no uniform field to read across all three build paths. For
// the composite path it is the ONLY place the flag lives at all: every span
// stays solid (SpanPayload carries no such field), and
// compositesweep.Assemble reads this field alone to decide which two caps
// to omit from the published face set.
type Payload struct {
	Prism           prismrecord.Payload
	Revolve         revolveplan.Payload
	Arc             bool
	ClosedCircle    bool
	ClosedComposite bool
	ReverseArcCaps  bool
	Spans           []SpanPayload
	Path            *sweepinput.Path
	SurfaceResult   bool
}

// StraightPrismReduction returns the exact prism built for one solid,
// zero-twist, one-line profile Sweep. Modifiers still apply their own edge or
// face selection gates before using the reduction.
func StraightPrismReduction(payload topo.Payload) (prismrecord.Payload, bool) {
	sp, ok := payload.(Payload)
	if !ok || sp.SurfaceResult || sp.Arc || sp.ClosedCircle || sp.ClosedComposite ||
		len(sp.Spans) != 0 || sp.Path == nil || len(sp.Path.Records) != 1 || len(sp.Path.Segments) != 1 {
		return prismrecord.Payload{}, false
	}
	if sp.Path.Segments[0].Arc {
		return prismrecord.Payload{}, false
	}
	return sp.Prism, true
}

func (sp Payload) Transform() r3.Transform {
	if len(sp.Spans) != 0 {
		return sp.Spans[0].transform()
	}
	if sp.Arc {
		return sp.Revolve.Xform
	}
	return sp.Prism.Xform
}

func (sp Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	if len(sp.Spans) != 0 {
		sp.Spans = append([]SpanPayload(nil), sp.Spans...)
		sp.Prism.Xform = composed
		for i := range sp.Spans {
			sp.Spans[i].Prism.Xform = composed
			sp.Spans[i].Revolve.Xform = composed
		}
		return replay(ctx, env, sp)
	}
	if sp.Arc {
		sp.Revolve.Xform = composed
		sp.Prism.Xform = composed
		body, err := revolveplan.Build(ctx, env, sp.Revolve, freeform.NewFreeformWork())
		if err != nil {
			return nil, err
		}
		FinishArc(body, sp)
		return body, nil
	}
	sp.Prism.Xform = composed
	body, err := prismrecord.Build(ctx, env, sp.Prism, freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	FinishStraight(body, sp)
	return body, nil
}

// ProvesSimple implements topo.SimpleProver. A sweep admits leg 4 for the
// ONE-SPAN STRAIGHT reduction alone (len(Spans) == 0, Arc false), on exactly
// the prism argument, which transfers verbatim: the line reduction IS a prism
// build. The arc-reduced and composite sheets read undecided instead, and
// deliberately — this is not an oversight to lift later without a new proof:
//
//   - the arc reduction is a revolve build whose own sweep is a PARTIAL turn
//     (full == false) — the revolve admission needs a full turn, so an
//     arc-reduced sweep never qualifies;
//   - the composite build's own compositesweep.AuditBoundary/AuditVertexLinks
//     (internal/compositesweep/boundary.go) prove a closed
//     two-manifold-with-boundary TOPOLOGY — every edge's face count matches
//     its use count, every vertex link is one cycle or path — which is a
//     combinatorial fact about how the spans sew together, never a geometric
//     claim that the swept walls do not fold back and cross themselves in
//     space. That geometric claim is precisely what this leg
//     (non-self-intersection) asks, and it is exactly the audit this increment
//     relaxes to admit a sheet's free rims in the first place, so it cannot
//     also be read as proving the thing it was relaxed away from.
func (sp Payload) ProvesSimple(context.Context) bool {
	return len(sp.Spans) == 0 && !sp.Arc && sp.Prism.SurfaceResult && sp.Prism.SectionDelta == 0
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (docs/evaluator-design.md §8): the root type this payload replaced.
func (Payload) SurveyClass() string { return "sweepPayload" }

// ChainPayload is the chain Sweep's own record of a ribbon body: one chain
// prism or Revolve reduction, or two built chain spans, beside its source Path.
// It stays DISTINCT from chainPayload for the reason sweepPayload stays
// distinct from prismPayload — a downstream consumer whose Sweep proof has not
// landed dispatches on this type and refuses, rather than reading the analytic
// reduction as proof for the original operation (docs/sweep-design.md §10).
type ChainPayload struct {
	Chain   chainrecord.Payload
	Revolve revolveplan.ChainPayload
	Arc     bool
	Path    *sweepinput.Path
	Spans   []ChainSpanPayload
}

// Transform is the accumulated rigid placement.
func (sp ChainPayload) Transform() r3.Transform {
	if len(sp.Spans) != 0 {
		return sp.Spans[0].transform()
	}
	if sp.Arc {
		return sp.Revolve.Transform()
	}
	return sp.Chain.Transform()
}

// Placed re-evaluates either one-span reduction under the composed motion
// (core §8). The cardinal composite refuses until its transformed join plane
// and mesh stations have their own bound.
func (sp ChainPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	if len(sp.Spans) != 0 {
		return nil, fmt.Errorf(`%w: composite chain Sweep placement has no transformed separation proof`, decaderr.ErrUnsupported)
	}
	if sp.Arc {
		sp.Revolve.Xform = composed
		body, err := revolveplan.BuildChain(ctx, env, sp.Revolve, freeform.NewFreeformWork())
		if err != nil {
			return nil, err
		}
		FinishChain(body, sp)
		return body, nil
	}
	sp.Chain.Xform = composed
	body, err := chainrecord.Build(ctx, env, sp.Chain, freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	FinishChain(body, sp)
	return body, nil
}

// ProvesSimple implements topo.SimpleProver. The one-span straight reduction
// is the only admitted case (docs/sweep-design.md §15): chain Sweep builds the
// body through the identical chainrecord.Build a plain chain Extrude does,
// over a chainrecord.Payload it then wraps unchanged (FinishChain). So this is
// the SAME construction argument chainrecord.Payload.ProvesSimple makes,
// re-read off the wrapped payload — never a weaker version of it. The arc and
// composite cases are undecided because their partial turns have no simplicity
// proof.
func (sp ChainPayload) ProvesSimple(context.Context) bool {
	if len(sp.Spans) != 0 || sp.Arc {
		return false
	}
	return sp.Chain.SectionDelta == 0
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (docs/evaluator-design.md §8): the root type this payload replaced.
func (ChainPayload) SurveyClass() string { return "chainSweepPayload" }
