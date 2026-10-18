package draftrecord

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// CircleMeridian carries a checked draft-circle meridian into the root
// revolve operation, which owns live topology and result publication.
type CircleMeridian struct {
	Profile      momentinput.Profile
	Frame        r3.Frame
	SectionDelta float64
}

// CircleFilletMeridian constructs the bounded meridian for an admitted
// far-rim fillet. The caller authenticates the draft and selected rim.
func CircleFilletMeridian(dp Payload, circle sectionrecord.CircleSeg, radius, radiusDelta float64) (CircleMeridian, error) {
	nearRadius, nearDelta, err := extent.MagnitudeInBounded(circle.Radius, units.Length, units.Millimeter, "the source circle radius")
	if err != nil {
		return CircleMeridian{}, err
	}
	near := proofbound.MeasuredScalar(nearRadius, nearDelta)
	height := proofbound.BoundedSub(
		proofbound.MeasuredScalar(dp.Z1, dp.Z1Delta),
		proofbound.MeasuredScalar(dp.Z0, dp.Z0Delta))
	amount := proofbound.MeasuredScalar(dp.D, dp.DDelta)
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
		return CircleMeridian{}, fmt.Errorf(`%w: the tapered circular rim fillet reaches the axis or the opposite cap`, decaderr.ErrDegenerate)
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
	point := prismrecord.Payload{Profile: dp.Profile, Frame: dp.Frame, Z0: dp.Z0,
		Z0Delta: dp.Z0Delta, Xform: r3.Identity()}
	originDelta := point.PointBound(proofbound.ExactScalar(circle.Center.U),
		proofbound.ExactScalar(circle.Center.V), proofbound.MeasuredScalar(dp.Z0, dp.Z0Delta))

	epsilon := math.Max(originDelta, dp.FarDelta)
	for _, q := range []proofbound.BoundedScalar{near, far, height, fillet,
		centerR, centerZ, sideR, sideZ} {
		epsilon = math.Max(epsilon, q.Bound)
	}
	sectionDelta := proofbound.ProductUpper(64, epsilon)
	if proofbound.IsNonFinite(sectionDelta) || sectionDelta >= radius/16 {
		return CircleMeridian{}, fmt.Errorf(`%w: the tapered circular rim fillet's source displacement cannot certify its arc`, decaderr.ErrUnsupported)
	}

	pt := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	zero := pt(0, 0)
	axisEnd := pt(height.Value, 0)
	capEnd := pt(height.Value, centerR.Value)
	sideEnd := pt(sideZ.Value, sideR.Value)
	baseEnd := pt(0, near.Value)
	line := func(a, z sectionrecord.Point2) sectionrecord.LineSeg {
		return sectionrecord.LineSeg{Start: a, End: z, TEnd: 1}
	}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(zero, axisEnd),
		line(axisEnd, capEnd),
		sectionrecord.ArcSeg{Center: pt(centerZ.Value, centerR.Value), Start: capEnd, End: sideEnd, TEnd: 1},
		line(sideEnd, baseEnd),
		line(baseEnd, zero),
	}}}
	origin := dp.Frame.ToWorldUV(circle.Center.U, circle.Center.V).Add(dp.Frame.N().Scale(dp.Z0))
	frame, err := r3.NewFrame(origin, dp.Frame.N(), dp.Frame.U())
	if err != nil {
		return CircleMeridian{}, fmt.Errorf(`%w: the tapered circle's meridian frame: %v`, decaderr.ErrUnsupported, err)
	}
	return CircleMeridian{Profile: profile, Frame: frame, SectionDelta: sectionDelta}, nil
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

// CircleChamferMeridian constructs the bounded meridian for an admitted
// far-rim chamfer. The caller authenticates the draft and selected rim.
func CircleChamferMeridian(dp Payload, circle sectionrecord.CircleSeg, setback, setbackDelta float64) (CircleMeridian, error) {
	nearRadius, nearDelta, err := extent.MagnitudeInBounded(circle.Radius, units.Length, units.Millimeter,
		"the source circle radius")
	if err != nil {
		return CircleMeridian{}, err
	}
	near := proofbound.MeasuredScalar(nearRadius, nearDelta)
	height := proofbound.BoundedSub(proofbound.MeasuredScalar(dp.Z1, dp.Z1Delta),
		proofbound.MeasuredScalar(dp.Z0, dp.Z0Delta))
	amount := proofbound.MeasuredScalar(dp.D, dp.DDelta)
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
		return CircleMeridian{}, fmt.Errorf(`%w: the tapered circular rim chamfer reaches the axis or the opposite cap`, decaderr.ErrDegenerate)
	}

	// A line's true-to-held distance is at most its endpoints' maximum
	// distance when both are read at the same parameter. The source circle,
	// axial levels, taper and setback supply all five meridian endpoints.
	point := prismrecord.Payload{Profile: dp.Profile, Frame: dp.Frame, Z0: dp.Z0,
		Z0Delta: dp.Z0Delta, Xform: r3.Identity()}
	originDelta := point.PointBound(proofbound.ExactScalar(circle.Center.U),
		proofbound.ExactScalar(circle.Center.V), proofbound.MeasuredScalar(dp.Z0, dp.Z0Delta))

	epsilon := math.Max(originDelta, dp.FarDelta)
	for _, q := range []proofbound.BoundedScalar{near, far, height, cut, slope, norm,
		sideStep, capR, sideZ, sideR} {
		epsilon = math.Max(epsilon, q.Bound)
	}
	sectionDelta := proofbound.ProductUpper(64, epsilon)
	if proofbound.IsNonFinite(sectionDelta) ||
		sectionDelta >= math.Min(setback, math.Min(capR.Value, sideZ.Value))/16 {
		return CircleMeridian{}, fmt.Errorf(`%w: the tapered circular rim chamfer's source displacement cannot certify its chord`,
			decaderr.ErrUnsupported)
	}

	pt := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	zero := pt(0, 0)
	axisEnd := pt(height.Value, 0)
	capEnd := pt(height.Value, capR.Value)
	sideEnd := pt(sideZ.Value, sideR.Value)
	baseEnd := pt(0, near.Value)
	line := func(a, z sectionrecord.Point2) sectionrecord.LineSeg {
		return sectionrecord.LineSeg{Start: a, End: z, TEnd: 1}
	}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(zero, axisEnd),
		line(axisEnd, capEnd),
		line(capEnd, sideEnd),
		line(sideEnd, baseEnd),
		line(baseEnd, zero),
	}}}
	origin := dp.Frame.ToWorldUV(circle.Center.U, circle.Center.V).Add(dp.Frame.N().Scale(dp.Z0))
	frame, err := r3.NewFrame(origin, dp.Frame.N(), dp.Frame.U())
	if err != nil {
		return CircleMeridian{}, fmt.Errorf(`%w: the tapered circle's meridian frame: %v`, decaderr.ErrUnsupported, err)
	}
	return CircleMeridian{Profile: profile, Frame: frame, SectionDelta: sectionDelta}, nil
}

// CircleShellMeridian constructs the bounded meridian for an admitted
// far-cap inward shell. The caller authenticates the draft and selected cap.
func CircleShellMeridian(dp Payload, nearCircle, farCircle sectionrecord.CircleSeg, tmm, tDelta float64) (CircleMeridian, error) {
	radius, radiusDelta, err := extent.MagnitudeInBounded(nearCircle.Radius,
		units.Length, units.Millimeter, "the source circle radius")
	if err != nil {
		return CircleMeridian{}, err
	}
	farHeld, farHeldDelta, err := extent.MagnitudeInBounded(farCircle.Radius,
		units.Length, units.Millimeter, "the drafted far circle radius")
	if err != nil {
		return CircleMeridian{}, err
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
		return CircleMeridian{}, fmt.Errorf(`%w: the tapered circular shell has no proven sweep height`, decaderr.ErrUnsupported)
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
		return CircleMeridian{}, fmt.Errorf(`%w: the tapered circular shell leaves no positive cavity, floor or top rim`,
			decaderr.ErrDegenerate)
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
		return CircleMeridian{}, fmt.Errorf(`%w: the tapered circular shell's source displacement cannot certify its cavity`,
			decaderr.ErrUnsupported)
	}

	pt := func(z, r float64) sectionrecord.Point2 { return sectionrecord.Point2{U: z, V: r} }
	line := func(a, c sectionrecord.Point2) sectionrecord.LineSeg {
		return sectionrecord.LineSeg{Start: a, End: c, TEnd: 1}
	}
	axisNear := pt(0, 0)
	axisFloor := pt(thickness.Value, 0)
	floorRim := pt(thickness.Value, innerFloor.Value)
	innerRim := pt(height.Value, innerTop.Value)
	outerRim := pt(height.Value, far.Value)
	baseRim := pt(0, near.Value)
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
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
		return CircleMeridian{}, fmt.Errorf(`%w: the tapered circle's meridian frame: %v`, decaderr.ErrUnsupported, err)
	}
	return CircleMeridian{Profile: profile, Frame: frame, SectionDelta: sectionDelta}, nil
}
