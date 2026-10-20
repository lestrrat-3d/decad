package brepmodify

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// draftCircleRim reads the one-sided, inward-tapered circular extrude whose
// far rim a Fillet or Chamfer names: the draft record authenticates its near
// circle, axial interval and taper span. ok is false for any other receiver
// or selection.
func draftCircleRim(b *topo.Body, edges []*topo.Edge) (draftrecord.Payload, sectionrecord.CircleSeg, bool) {
	dp, ok := b.Payload.(draftrecord.Payload)
	if !ok || !dp.NearStart || dp.Taper <= 0 || len(dp.Kept) != 0 ||
		len(dp.Profile.Holes) != 0 || len(dp.Profile.Outer.Segments) != 1 || len(edges) != 1 {
		return draftrecord.Payload{}, sectionrecord.CircleSeg{}, false
	}
	circle, ok := dp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !circle.CCW || len(dp.Far.Outer.Segments) != 1 || !draftrecord.ExactSignedAxisFrame(dp.Frame) {
		return draftrecord.Payload{}, sectionrecord.CircleSeg{}, false
	}
	if _, ok := dp.Far.Outer.Segments[0].(sectionrecord.CircleSeg); !ok {
		return draftrecord.Payload{}, sectionrecord.CircleSeg{}, false
	}
	caps := CapsOf(b)
	if caps.End == nil || len(caps.End.Loops) != 1 || len(caps.End.Loops[0].CoEdges) != 1 ||
		caps.End.Loops[0].CoEdges[0].Edge != edges[0] {
		return draftrecord.Payload{}, sectionrecord.CircleSeg{}, false
	}
	return dp, circle, true
}

// draftCircleRevolve builds the full revolve of a tapered circular extrude's
// rewritten meridian: the full-revolve evaluator gives its cones, torus, mass
// readings and occupied-volume proof. crosses is the refusal for a meridian
// that crosses its axis; edit sets the payload fields that name the blend.
func draftCircleRevolve(ctx context.Context, env topo.Env, dp draftrecord.Payload, meridian draftrecord.CircleMeridian,
	crosses string, edit func(*revolveplan.Payload)) (*topo.Body, error) {
	work := freeform.NewFreeformWork()
	ax, side, err := revolveplan.ResolveAxisSide(ctx, meridian.Profile, revolveaxis.Line2{DU: 1}, work)
	if err != nil {
		return nil, err
	}
	if side < 0 {
		return nil, fmt.Errorf(`%w: %s`, decaderr.ErrDegenerate, crosses)
	}
	p := revolveplan.Payload{
		Profile: meridian.Profile, Frame: meridian.Frame, Ax: ax, Phi0: 0, Phi1: fullTurnRadians, Full: true,
		Den: revolveangle.FullTurn(), Xform: dp.Xform, RadialProof: ax.RadialProof,
		SectionDelta: meridian.SectionDelta, SectionWhole: true,
	}
	edit(&p)
	return revolveplan.Build(ctx, env, p, work)
}

// DraftCircleRimChamfer bevels the far rim of one inward-tapered circular
// extrude. The source draft proves the circle and sweep; a five-line meridian
// gives the full-revolve evaluator two cones and its mass and mesh proofs.
func DraftCircleRimChamfer(ctx context.Context, env topo.Env, b *topo.Body, edges []*topo.Edge,
	setback, setbackDelta float64) (*topo.Body, bool, error) {
	dp, circle, ok := draftCircleRim(b, edges)
	if !ok {
		return nil, false, nil
	}
	meridian, err := draftrecord.CircleChamferMeridian(dp, circle, setback, setbackDelta)
	if err != nil {
		return nil, true, err
	}
	out, err := draftCircleRevolve(ctx, env, dp, meridian, `the tapered rim's bevel meridian crosses its axis`,
		func(p *revolveplan.Payload) {
			p.BlendSegs, p.BlendKind, p.DraftCircleCaps = []map[int]struct{}{{2: {}}}, "chamfer", true
		})
	return out, true, err
}

// DraftCircleRimFillet rounds one circular end rim of a one-sided, inward
// tapered circular extrude. The draft record authenticates its near circle,
// axial interval and taper span. The result's meridian has one circular arc
// between its top plane and cone, so the full-revolve evaluator builds the
// torus, mass readings and occupied-volume proof from that meridian.
func DraftCircleRimFillet(ctx context.Context, env topo.Env, b *topo.Body, edges []*topo.Edge,
	radius, radiusDelta float64) (*topo.Body, bool, error) {
	dp, circle, ok := draftCircleRim(b, edges)
	if !ok {
		return nil, false, nil
	}
	meridian, err := draftrecord.CircleFilletMeridian(dp, circle, radius, radiusDelta)
	if err != nil {
		return nil, true, err
	}
	out, err := draftCircleRevolve(ctx, env, dp, meridian, `the tapered rim's rounded meridian crosses its axis`,
		func(p *revolveplan.Payload) {
			p.BlendSegs, p.BlendKind, p.DraftCircleCaps = []map[int]struct{}{{2: {}}}, "fillet", true
		})
	return out, true, err
}

// draftCircleShell shells the far cap of one positive-taper, full-circle
// draft. Its authenticated radius is affine in axial distance. A normal
// inward offset moves its conical wall radially by t/cos(alpha), while the
// kept near cap puts the cavity floor at axial distance t. The six straight
// meridian segments built by draftrecord bound the denoted cup. Their source
// and construction displacement is charged as one whole-section revolve bound.
func draftCircleShell(ctx context.Context, env topo.Env, b *topo.Body, removed []*topo.Face, inward bool,
	tmm, tDelta float64) (*topo.Body, bool, error) {
	dp, ok := b.Payload.(draftrecord.Payload)
	if !ok || !inward || !dp.NearStart || dp.Taper <= 0 || len(dp.Kept) != 0 ||
		len(dp.Profile.Holes) != 0 || len(dp.Profile.Outer.Segments) != 1 ||
		len(dp.Far.Outer.Segments) != 1 || !draftrecord.ExactSignedAxisFrame(dp.Frame) {
		return nil, false, nil
	}
	nearCircle, ok := dp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !nearCircle.CCW {
		return nil, false, nil
	}
	farCircle, ok := dp.Far.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok {
		return nil, false, nil
	}
	caps := CapsOf(b)
	if len(removed) != 1 || caps.End == nil || removed[0] != caps.End {
		return nil, false, nil
	}
	meridian, err := draftrecord.CircleShellMeridian(dp, nearCircle, farCircle, tmm, tDelta)
	if err != nil {
		return nil, true, err
	}
	out, err := draftCircleRevolve(ctx, env, dp, meridian, `the tapered circular shell crosses its axis`,
		func(p *revolveplan.Payload) { p.DraftCircleShellRoles = true })
	return out, true, err
}
