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
	if !ok || !circle.CCW || len(dp.Far.Outer.Segments) != 1 || !exactSignedAxisFrame(dp.Frame) {
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
