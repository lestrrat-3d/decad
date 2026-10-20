package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
)

// tryDraftCircleRimChamfer bevels the far rim of one inward-tapered circular
// extrude. The source draft proves the circle and sweep; a five-line meridian
// gives the full-revolve evaluator two cones and its mass and mesh proofs.
func tryDraftCircleRimChamfer(ctx context.Context, b *Body, edges []*Edge, setback, setbackDelta float64) (*Body, bool, error) {
	dp, ok := b.n.Payload.(draftPayload)
	if !ok || !dp.NearStart || dp.Taper <= 0 || len(dp.Kept) != 0 ||
		len(dp.Profile.Holes) != 0 || len(dp.Profile.Outer.Segments) != 1 || len(edges) != 1 {
		return nil, false, nil
	}
	circle, ok := dp.Profile.Outer.Segments[0].(circleSeg)
	if !ok || !circle.CCW || len(dp.Far.Outer.Segments) != 1 || !draftrecord.ExactSignedAxisFrame(dp.Frame) {
		return nil, false, nil
	}
	if _, ok := dp.Far.Outer.Segments[0].(circleSeg); !ok {
		return nil, false, nil
	}
	caps := prismCapsOf(b)
	if caps.end == nil || len(caps.end.Loops()) != 1 || len(caps.end.Loops()[0].Edges()) != 1 ||
		caps.end.Loops()[0].Edges()[0] != edges[0] {
		return nil, false, nil
	}

	meridian, err := draftrecord.CircleChamferMeridian(dp, circle, setback, setbackDelta)
	if err != nil {
		return nil, true, err
	}
	work := freeform.NewFreeformWork()
	ax, side, err := revolveplan.ResolveAxisSide(ctx, meridian.Profile, revolveaxis.Line2{DU: 1}, work)
	if err != nil {
		return nil, true, err
	}
	if side < 0 {
		return nil, true, fmt.Errorf(`%w: the tapered rim's bevel meridian crosses its axis`, ErrDegenerate)
	}
	phi0, phi1, full, den, _, err := documentHandle(b.n.Doc).resolveAngularExtent(FullRevolution{}, angularStops{})
	if err != nil {
		return nil, true, err
	}
	result, err := bodyResult(revolveplan.Build(ctx, envOf(b.n.Doc, b.n.Doc.NextProducer), revolvePayload{
		Profile: meridian.Profile, Frame: meridian.Frame, Ax: ax, Phi0: phi0, Phi1: phi1, Full: full,
		Den: den, Xform: dp.Xform, RadialProof: ax.RadialProof,
		SectionDelta: meridian.SectionDelta, SectionWhole: true,
		BlendSegs: []map[int]struct{}{{2: {}}}, BlendKind: "chamfer",
		DraftCircleCaps: true,
	}, work))
	if err != nil {
		return nil, true, err
	}
	out, err := commitModifyResult(ctx, b, result)
	return out, true, err
}

// tryDraftCircleRimFillet rounds one circular end rim of a one-sided, inward
// tapered circular extrude. The draft record authenticates its near circle,
// axial interval and taper span. The result's meridian has one circular arc
// between its top plane and cone, so the full-revolve evaluator builds the
// torus, mass readings and occupied-volume proof from that meridian.
func tryDraftCircleRimFillet(ctx context.Context, b *Body, edges []*Edge, radius, radiusDelta float64) (*Body, bool, error) {
	dp, ok := b.n.Payload.(draftPayload)
	if !ok || !dp.NearStart || dp.Taper <= 0 || len(dp.Kept) != 0 ||
		len(dp.Profile.Holes) != 0 || len(dp.Profile.Outer.Segments) != 1 || len(edges) != 1 {
		return nil, false, nil
	}
	circle, ok := dp.Profile.Outer.Segments[0].(circleSeg)
	if !ok || !circle.CCW || len(dp.Far.Outer.Segments) != 1 || !draftrecord.ExactSignedAxisFrame(dp.Frame) {
		return nil, false, nil
	}
	if _, ok := dp.Far.Outer.Segments[0].(circleSeg); !ok {
		return nil, false, nil
	}
	caps := prismCapsOf(b)
	if caps.end == nil || len(caps.end.Loops()) != 1 || len(caps.end.Loops()[0].Edges()) != 1 ||
		caps.end.Loops()[0].Edges()[0] != edges[0] {
		return nil, false, nil
	}

	meridian, err := draftrecord.CircleFilletMeridian(dp, circle, radius, radiusDelta)
	if err != nil {
		return nil, true, err
	}
	work := freeform.NewFreeformWork()
	ax, side, err := revolveplan.ResolveAxisSide(ctx, meridian.Profile, revolveaxis.Line2{DU: 1}, work)
	if err != nil {
		return nil, true, err
	}
	if side < 0 {
		return nil, true, fmt.Errorf(`%w: the tapered rim's rounded meridian crosses its axis`, ErrDegenerate)
	}
	phi0, phi1, full, den, _, err := documentHandle(b.n.Doc).resolveAngularExtent(FullRevolution{}, angularStops{})
	if err != nil {
		return nil, true, err
	}
	result, err := bodyResult(revolveplan.Build(ctx, envOf(b.n.Doc, b.n.Doc.NextProducer), revolvePayload{
		Profile: meridian.Profile, Frame: meridian.Frame, Ax: ax, Phi0: phi0, Phi1: phi1, Full: full,
		Den: den, Xform: dp.Xform, RadialProof: ax.RadialProof,
		SectionDelta: meridian.SectionDelta, SectionWhole: true,
		BlendSegs: []map[int]struct{}{{2: {}}}, BlendKind: "fillet",
		DraftCircleCaps: true,
	}, work))
	if err != nil {
		return nil, true, err
	}
	out, err := commitModifyResult(ctx, b, result)
	return out, true, err
}

// tryDraftCircleShell shells the far cap of one positive-taper, full-circle
// draft. Its authenticated radius is affine in axial distance. A normal
// inward offset moves its conical wall radially by t/cos(alpha), while the
// kept near cap puts the cavity floor at axial distance t. The six straight
// meridian segments built by draftrecord bound the denoted cup. Their source
// and construction displacement is charged as one whole-section revolve bound.
func tryDraftCircleShell(ctx context.Context, b *Body, removed []*Face, sense ShellSense,
	tmm, tDelta float64) (*Body, bool, error) {
	dp, ok := b.n.Payload.(draftPayload)
	if !ok || sense != Inward || !dp.NearStart || dp.Taper <= 0 || len(dp.Kept) != 0 ||
		len(dp.Profile.Holes) != 0 || len(dp.Profile.Outer.Segments) != 1 ||
		len(dp.Far.Outer.Segments) != 1 || !draftrecord.ExactSignedAxisFrame(dp.Frame) {
		return nil, false, nil
	}
	nearCircle, ok := dp.Profile.Outer.Segments[0].(circleSeg)
	if !ok || !nearCircle.CCW {
		return nil, false, nil
	}
	farCircle, ok := dp.Far.Outer.Segments[0].(circleSeg)
	if !ok {
		return nil, false, nil
	}
	caps := prismCapsOf(b)
	if len(removed) != 1 || caps.end == nil || removed[0] != caps.end {
		return nil, false, nil
	}

	meridian, err := draftrecord.CircleShellMeridian(dp, nearCircle, farCircle, tmm, tDelta)
	if err != nil {
		return nil, true, err
	}
	work := freeform.NewFreeformWork()
	ax, side, err := revolveplan.ResolveAxisSide(ctx, meridian.Profile, revolveaxis.Line2{DU: 1}, work)
	if err != nil {
		return nil, true, err
	}
	if side < 0 {
		return nil, true, fmt.Errorf(`%w: the tapered circular shell crosses its axis`, ErrDegenerate)
	}
	phi0, phi1, full, den, _, err := documentHandle(b.n.Doc).resolveAngularExtent(FullRevolution{}, angularStops{})
	if err != nil {
		return nil, true, err
	}
	result, err := bodyResult(revolveplan.Build(ctx, envOf(b.n.Doc, b.n.Doc.NextProducer), revolvePayload{
		Profile: meridian.Profile, Frame: meridian.Frame, Ax: ax, Phi0: phi0, Phi1: phi1, Full: full,
		Den: den, Xform: dp.Xform, RadialProof: ax.RadialProof,
		SectionDelta: meridian.SectionDelta, SectionWhole: true, DraftCircleShellRoles: true,
	}, work))
	if err != nil {
		return nil, true, err
	}
	out, err := commitModifyResult(ctx, b, result)
	return out, true, err
}
