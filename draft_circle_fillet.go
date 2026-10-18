package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/r3"
)

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

func exactSignedAxisFrame(frame r3.Frame) bool {
	u, v := frame.U(), frame.V()
	axis := func(a r3.Vec) bool {
		return (math.Abs(a.X) == 1 && a.Y == 0 && a.Z == 0) ||
			(math.Abs(a.Y) == 1 && a.X == 0 && a.Z == 0) ||
			(math.Abs(a.Z) == 1 && a.X == 0 && a.Y == 0)
	}
	return axis(u) && axis(v) && u.Dot(v) == 0
}
