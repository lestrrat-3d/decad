package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// tryDraftCircleRimFillet rounds one circular end rim of a one-sided, inward
// tapered circular extrude. The draft record authenticates its near circle,
// axial interval and taper span. The result's meridian has one circular arc
// between its top plane and cone, so the full-revolve evaluator builds the
// torus, mass readings and occupied-volume proof from that meridian.
func tryDraftCircleRimFillet(ctx context.Context, b *Body, edges []*Edge, radius, radiusDelta float64) (*Body, bool, error) {
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

	nearRadius, nearDelta, err := extent.MagnitudeInBounded(circle.Radius, units.Length, units.Millimeter, "the source circle radius")
	if err != nil {
		return nil, true, err
	}
	near := proofbound.MeasuredScalar(nearRadius, nearDelta)
	height := proofbound.BoundedSub(
		proofbound.MeasuredScalar(dp.z1, dp.z1Delta),
		proofbound.MeasuredScalar(dp.z0, dp.z0Delta))
	amount := proofbound.MeasuredScalar(dp.d, dp.dDelta)
	fillet := proofbound.MeasuredScalar(radius, radiusDelta)
	slope := proofbound.BoundedDiv(amount, height)
	unit := proofbound.ExactScalar(1)
	norm := proofbound.BoundedSqrt(proofbound.BoundedAdd(unit, proofbound.BoundedMul(slope, slope)))
	far := proofbound.BoundedSub(near, amount)
	centerR := proofbound.BoundedSub(
		proofbound.BoundedAdd(far, proofbound.BoundedMul(slope, fillet)),
		proofbound.BoundedMul(fillet, norm))
	centerZ := proofbound.BoundedSub(height, fillet)
	sideZ := proofbound.BoundedAdd(centerZ,
		proofbound.BoundedDiv(proofbound.BoundedMul(fillet, slope), norm))
	sideR := proofbound.BoundedAdd(centerR, proofbound.BoundedDiv(fillet, norm))
	if !draftCircleFilletFits(near, far, height, slope, fillet, centerR, centerZ, sideR, sideZ) {
		return nil, true, fmt.Errorf(`%w: the tapered circular rim fillet reaches the axis or the opposite cap`, ErrDegenerate)
	}

	// Source and held meridian pair line for line and arc for arc. Each
	// centre/end coordinate differs by at most epsilon, so its plane vector
	// differs by at most sqrt(2)*epsilon and each endpoint ray changes by at
	// most 2*sqrt(2)*epsilon. The gate below makes epsilon < radius/1024:
	// true/held radii stay above radius-2*sqrt(2)*epsilon, so each endpoint
	// angle moves by less than 4*epsilon/radius. Interpolating the unwrapped
	// sweep of at most a quarter turn, and adding centre and radius motion,
	// moves every arc point by less than 16*epsilon. The 64-fold charge also
	// covers every constructed line and the frame-origin lift.
	point := prismrecord.Payload{Profile: dp.profile, Frame: dp.frame, Z0: dp.z0,
		Z0Delta: dp.z0Delta, Xform: r3.Identity()}
	originDelta := point.PointBound(proofbound.ExactScalar(circle.Center.U),
		proofbound.ExactScalar(circle.Center.V), proofbound.MeasuredScalar(dp.z0, dp.z0Delta))

	epsilon := math.Max(originDelta, dp.farDelta)
	for _, q := range []proofbound.BoundedScalar{near, far, height, fillet,
		centerR, centerZ, sideR, sideZ} {
		epsilon = math.Max(epsilon, q.Bound)
	}
	sectionDelta := proofbound.ProductUpper(64, epsilon)
	if proofbound.IsNonFinite(sectionDelta) || sectionDelta >= radius/16 {
		return nil, true, fmt.Errorf(`%w: the tapered circular rim fillet's source displacement cannot certify its arc`, ErrUnsupported)
	}

	pt := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	zero := pt(0, 0)
	axisEnd := pt(height.Value, 0)
	capEnd := pt(height.Value, centerR.Value)
	sideEnd := pt(sideZ.Value, sideR.Value)
	baseEnd := pt(0, near.Value)
	line := func(a, z sectionrecord.Point2) lineSeg { return lineSeg{Start: a, End: z, TEnd: 1} }
	profile := profileRecord{Outer: loopRecord{Segments: []curveSegment{
		line(zero, axisEnd),
		line(axisEnd, capEnd),
		arcSeg{Center: pt(centerZ.Value, centerR.Value), Start: capEnd, End: sideEnd, TEnd: 1},
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
		return nil, true, fmt.Errorf(`%w: the tapered rim's rounded meridian crosses its axis`, ErrDegenerate)
	}
	phi0, phi1, full, den, _, err := documentHandle(b.n.Doc).resolveAngularExtent(FullRevolution{}, angularStops{})
	if err != nil {
		return nil, true, err
	}
	result, err := evalRevolveContextWork(ctx, documentHandle(b.n.Doc), b.n.Doc.NextProducer, revolvePayload{
		Profile: profile, Frame: frame, Ax: ax, Phi0: phi0, Phi1: phi1, Full: full,
		Den: den, Xform: dp.xform, RadialProof: ax.RadialProof,
		SectionDelta: sectionDelta, SectionWhole: true,
		BlendSegs: []map[int]struct{}{{2: {}}}, BlendKind: "fillet",
		DraftCircleCaps: true,
	}, work)
	if err != nil {
		return nil, true, err
	}
	out, err := commitModifyResult(ctx, b, result)
	return out, true, err
}

// A full revolution normally has no cap role: its meridian walls are all
// side faces. Both tapered-circle rim rewrites have two planar disks.
// Preserve their axial cap names so face and rim selectors still work after
// the draft's rebuild and after placement.
func addDraftCircleCapRoles(ctx context.Context, body *Body, ref producerID) error {
	roles := map[string]string{"side(0,1)": provenance.RoleCapEnd, "side(0,4)": provenance.RoleCapStart}
	seen := map[string]bool{}
	for _, face := range body.Faces() {
		if err := ctx.Err(); err != nil {
			return err
		}
		for _, origin := range face.n.Origins {
			capRole, ok := roles[origin.Role]
			if !ok || origin.Producer != ref {
				continue
			}
			if seen[capRole] || face.n.Surface.Kind() != surfacegeom.KindPlane {
				return fmt.Errorf(`%w: the tapered circular rim has no unique planar %s`, ErrUnsupported, capRole)
			}
			face.n.Origins = append(face.n.Origins, provenance.Ref{Producer: ref, Role: capRole})
			seen[capRole] = true
		}
	}
	if !seen[provenance.RoleCapStart] || !seen[provenance.RoleCapEnd] {
		return fmt.Errorf(`%w: the tapered circular rim lost a planar cap`, ErrUnsupported)
	}
	return nil
}

func draftCircleFilletFits(near, far, height, slope, fillet, centerR, centerZ, sideR, sideZ proofbound.BoundedScalar) bool {
	positive := func(q proofbound.BoundedScalar) bool { lo, _ := proofbound.BoundedEnds(q); return lo > 0 }
	if !positive(near) || !positive(far) || !positive(height) || !positive(slope) ||
		!positive(fillet) || !positive(centerZ) || !positive(sideR) || !positive(sideZ) {
		return false
	}
	if !positive(proofbound.BoundedSub(centerR, fillet)) ||
		!positive(proofbound.BoundedSub(height, sideZ)) ||
		!positive(proofbound.BoundedSub(near, sideR)) {
		return false
	}
	return true
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
