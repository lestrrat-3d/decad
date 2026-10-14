package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// tryDraftCircleRimChamfer bevels the far rim of one inward-tapered circular
// extrude. The source draft proves the circle and sweep; a five-line meridian
// gives the full-revolve evaluator two cones and its mass and mesh proofs.
func tryDraftCircleRimChamfer(ctx context.Context, b *Body, edges []*Edge, setback, setbackDelta float64) (*Body, bool, error) {
	dp, ok := b.n.Payload.(draftPayload)
	if !ok || !dp.nearStart || dp.taper <= 0 || len(dp.kept) != 0 ||
		len(dp.profile.Holes) != 0 || len(dp.profile.Outer.Segments) != 1 || len(edges) != 1 {
		return nil, false, nil
	}
	circle, ok := dp.profile.Outer.Segments[0].(circleSeg)
	if !ok || !circle.CCW || len(dp.far.Outer.Segments) != 1 || !exactSignedAxisFrame(dp.frame) {
		return nil, false, nil
	}
	if _, ok := dp.far.Outer.Segments[0].(circleSeg); !ok {
		return nil, false, nil
	}
	caps := prismCapsOf(b)
	if caps.end == nil || len(caps.end.Loops()) != 1 || len(caps.end.Loops()[0].Edges()) != 1 ||
		caps.end.Loops()[0].Edges()[0] != edges[0] {
		return nil, false, nil
	}

	nearRadius, nearDelta, err := extent.MagnitudeInBounded(circle.Radius, units.Length, units.Millimeter,
		"the source circle radius")
	if err != nil {
		return nil, true, err
	}
	near := proofbound.MeasuredScalar(nearRadius, nearDelta)
	height := proofbound.BoundedSub(proofbound.MeasuredScalar(dp.z1, dp.z1Delta),
		proofbound.MeasuredScalar(dp.z0, dp.z0Delta))
	amount := proofbound.MeasuredScalar(dp.d, dp.dDelta)
	cut := proofbound.MeasuredScalar(setback, setbackDelta)
	slope := proofbound.BoundedDiv(amount, height)
	norm := proofbound.BoundedSqrt(proofbound.BoundedAdd(proofbound.ExactScalar(1),
		proofbound.BoundedMul(slope, slope)))
	far := proofbound.BoundedSub(near, amount)
	sideStep := proofbound.BoundedDiv(cut, norm)
	capR := proofbound.BoundedSub(far, cut)
	sideZ := proofbound.BoundedSub(height, sideStep)
	sideR := proofbound.BoundedAdd(far, proofbound.BoundedMul(slope, sideStep))
	positive := func(q proofbound.BoundedScalar) bool {
		lo, _ := proofbound.BoundedEnds(q)
		return lo > 0
	}
	if !positive(near) || !positive(far) || !positive(height) || !positive(slope) ||
		!positive(cut) || !positive(capR) || !positive(sideZ) || !positive(sideR) ||
		!positive(proofbound.BoundedSub(height, sideZ)) ||
		!positive(proofbound.BoundedSub(near, sideR)) {
		return nil, true, fmt.Errorf(`%w: the tapered circular rim chamfer reaches the axis or the opposite cap`, ErrDegenerate)
	}

	// A line's true-to-held distance is at most its endpoints' maximum
	// distance when both are read at the same parameter. The source circle,
	// axial levels, taper and setback supply all five meridian endpoints.
	point := prismrecord.Payload{Profile: dp.profile, Frame: dp.frame, Z0: dp.z0,
		Z0Delta: dp.z0Delta, Xform: r3.Identity()}
	originDelta := point.PointBound(proofbound.ExactScalar(circle.Center.U),
		proofbound.ExactScalar(circle.Center.V), proofbound.MeasuredScalar(dp.z0, dp.z0Delta))

	epsilon := math.Max(originDelta, dp.farDelta)
	for _, q := range []proofbound.BoundedScalar{near, far, height, cut, slope, norm,
		sideStep, capR, sideZ, sideR} {
		epsilon = math.Max(epsilon, q.Bound)
	}
	sectionDelta := proofbound.ProductUpper(64, epsilon)
	if proofbound.IsNonFinite(sectionDelta) ||
		sectionDelta >= math.Min(setback, math.Min(capR.Value, sideZ.Value))/16 {
		return nil, true, fmt.Errorf(`%w: the tapered circular rim chamfer's source displacement cannot certify its chord`,
			ErrUnsupported)
	}

	pt := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	zero := pt(0, 0)
	axisEnd := pt(height.Value, 0)
	capEnd := pt(height.Value, capR.Value)
	sideEnd := pt(sideZ.Value, sideR.Value)
	baseEnd := pt(0, near.Value)
	line := func(a, z sectionrecord.Point2) lineSeg { return lineSeg{Start: a, End: z, TEnd: 1} }
	profile := profileRecord{Outer: loopRecord{Segments: []curveSegment{
		line(zero, axisEnd),
		line(axisEnd, capEnd),
		line(capEnd, sideEnd),
		line(sideEnd, baseEnd),
		line(baseEnd, zero),
	}}}
	origin := dp.frame.ToWorldUV(circle.Center.U, circle.Center.V).Add(dp.frame.N().Scale(dp.z0))
	frame, err := r3.NewFrame(origin, dp.frame.N(), dp.frame.U())
	if err != nil {
		return nil, true, fmt.Errorf(`%w: the tapered circle's meridian frame: %v`, ErrUnsupported, err)
	}
	work := freeform.NewFreeformWork()
	ax, side, err := resolveAxisSide(ctx, profile, revolveaxis.Line2{DU: 1}, work)
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
	result, err := evalRevolveContextWork(ctx, documentHandle(b.n.Doc), b.n.Doc.NextProducer, revolvePayload{
		profile: profile, frame: frame, ax: ax, phi0: phi0, phi1: phi1, full: full,
		den: den, xform: dp.xform, radialProof: ax.RadialProof,
		sectionDelta: sectionDelta, sectionWhole: true,
		blendSegs: []map[int]struct{}{{2: {}}}, blendKind: "chamfer",
		draftCircleCaps: true,
	}, work)
	if err != nil {
		return nil, true, err
	}
	out, err := commitModifyResult(ctx, b, result)
	return out, true, err
}
