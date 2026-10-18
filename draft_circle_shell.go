package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
)

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
		len(dp.Far.Outer.Segments) != 1 || !exactSignedAxisFrame(dp.Frame) {
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
