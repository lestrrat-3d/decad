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
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// tryDraftCircleShell shells the far cap of one positive-taper, full-circle
// draft. Its authenticated radius is affine in axial distance. A normal
// inward offset moves its conical wall radially by t/cos(alpha), while the
// kept near cap puts the cavity floor at axial distance t. The six straight
// meridian segments below therefore bound the denoted cup. Their source and
// construction displacement is charged as one whole-section revolve bound.
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

	radius, radiusDelta, err := extent.MagnitudeInBounded(nearCircle.Radius,
		units.Length, units.Millimeter, "the source circle radius")
	if err != nil {
		return nil, true, err
	}
	farHeld, farHeldDelta, err := extent.MagnitudeInBounded(farCircle.Radius,
		units.Length, units.Millimeter, "the drafted far circle radius")
	if err != nil {
		return nil, true, err
	}
	near := proofbound.MeasuredScalar(radius, radiusDelta)
	height := proofbound.BoundedSub(proofbound.MeasuredScalar(dp.Z1, dp.Z1Delta),
		proofbound.MeasuredScalar(dp.Z0, dp.Z0Delta))
	amount := proofbound.MeasuredScalar(dp.D, dp.DDelta)
	thickness := proofbound.MeasuredScalar(tmm, tDelta)
	positive := func(q proofbound.BoundedScalar) bool {
		lo, _ := proofbound.BoundedEnds(q)
		return lo > 0
	}
	if !positive(height) {
		return nil, true, fmt.Errorf(`%w: the tapered circular shell has no proven sweep height`, ErrUnsupported)
	}
	slope := proofbound.BoundedDiv(amount, height)
	norm := proofbound.BoundedSqrt(proofbound.BoundedAdd(proofbound.ExactScalar(1),
		proofbound.BoundedMul(slope, slope)))
	shift := proofbound.BoundedMul(thickness, norm)
	far := proofbound.BoundedSub(near, amount)
	innerTop := proofbound.BoundedSub(far, shift)
	innerFloor := proofbound.BoundedSub(proofbound.BoundedSub(near,
		proofbound.BoundedMul(slope, thickness)), shift)
	wallHeight := proofbound.BoundedSub(height, thickness)
	if !positive(near) || !positive(amount) || !positive(thickness) || !positive(wallHeight) ||
		!positive(innerTop) || !positive(innerFloor) || !positive(shift) {
		return nil, true, fmt.Errorf(`%w: the tapered circular shell leaves no positive cavity, floor or top rim`,
			ErrDegenerate)
	}

	// Every denoted point of a straight meridian edge pairs with its held
	// point by the same affine parameter. Each coordinate's interval reaches
	// the true endpoint, so the endpoint maximum reaches its entire edge.
	// Include the existing draft's far-contour displacement and frame lift.
	point := prismrecord.Payload{Profile: dp.Profile, Frame: dp.Frame, Z0: dp.Z0,
		Z0Delta: dp.Z0Delta, Xform: r3.Identity()}
	originDelta := point.PointBound(proofbound.ExactScalar(nearCircle.Center.U),
		proofbound.ExactScalar(nearCircle.Center.V), proofbound.MeasuredScalar(dp.Z0, dp.Z0Delta))

	epsilon := math.Max(originDelta, dp.FarDelta)
	epsilon = math.Max(epsilon, math.Abs(farHeld-far.Value)+farHeldDelta)
	epsilon = math.Max(epsilon, math.Hypot(farCircle.Center.U-nearCircle.Center.U,
		farCircle.Center.V-nearCircle.Center.V))
	for _, q := range []proofbound.BoundedScalar{near, height, amount, thickness,
		slope, norm, shift, far, innerTop, innerFloor, wallHeight} {
		epsilon = math.Max(epsilon, q.Bound)
	}
	sectionDelta := proofbound.ProductUpper(64, epsilon)
	gap := math.Min(tmm, math.Min(wallHeight.Value, math.Min(innerTop.Value, shift.Value)))
	if proofbound.IsNonFinite(sectionDelta) || sectionDelta >= gap/16 {
		return nil, true, fmt.Errorf(`%w: the tapered circular shell's source displacement cannot certify its cavity`,
			ErrUnsupported)
	}

	pt := func(z, r float64) sectionrecord.Point2 { return sectionrecord.Point2{U: z, V: r} }
	line := func(a, c sectionrecord.Point2) lineSeg { return lineSeg{Start: a, End: c, TEnd: 1} }
	axisNear := pt(0, 0)
	axisFloor := pt(thickness.Value, 0)
	floorRim := pt(thickness.Value, innerFloor.Value)
	innerRim := pt(height.Value, innerTop.Value)
	outerRim := pt(height.Value, far.Value)
	baseRim := pt(0, near.Value)
	profile := profileRecord{Outer: loopRecord{Segments: []curveSegment{
		line(axisNear, axisFloor),
		line(axisFloor, floorRim),
		line(floorRim, innerRim),
		line(innerRim, outerRim),
		line(outerRim, baseRim),
		line(baseRim, axisNear),
	}}}
	origin := dp.Frame.ToWorldUV(nearCircle.Center.U, nearCircle.Center.V).Add(dp.Frame.N().Scale(dp.Z0))
	frame, err := r3.NewFrame(origin, dp.Frame.N(), dp.Frame.U())
	if err != nil {
		return nil, true, fmt.Errorf(`%w: the tapered circle's meridian frame: %v`, ErrUnsupported, err)
	}
	work := freeform.NewFreeformWork()
	ax, side, err := revolveplan.ResolveAxisSide(ctx, profile, revolveaxis.Line2{DU: 1}, work)
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
		Profile: profile, Frame: frame, Ax: ax, Phi0: phi0, Phi1: phi1, Full: full,
		Den: den, Xform: dp.Xform, RadialProof: ax.RadialProof,
		SectionDelta: sectionDelta, SectionWhole: true, DraftCircleShellRoles: true,
	}, work))
	if err != nil {
		return nil, true, err
	}
	out, err := commitModifyResult(ctx, b, result)
	return out, true, err
}
