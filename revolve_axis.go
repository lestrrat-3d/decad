package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// This file resolves a revolve's axis into the sketch plane and decides what
// the profile may do around it: axisLine2, the 2D line the axis projects to;
// axisFrame, the axis-local frame every later reading is taken in; and the
// two gates that refuse a profile crossing or touching the axis where the
// sweep would be degenerate.
//
// wallKind is decided here because it follows from the axis alone: a segment
// parallel to the axis sweeps a cylinder, one meeting it at an angle a cone,
// one perpendicular an annulus. A segment whose kind the axis cannot decide
// exactly refuses rather than being assigned the nearest one. See
// docs/evaluator-design.md §6.

// axisLine2 is a revolve axis resolved into the profile plane: a point on
// the axis and its unit direction, plane-local (u, v).
type axisLine2 struct {
	aU, aV           float64
	aUBound, aVBound float64
	dU, dV           float64
	dUBound, dVBound float64
}

// finiteAxisValues reports whether every derived axis value is representable.
// Axis inputs are checked before arithmetic, but subtracting finite endpoints
// or transforming a finite world point can still overflow.
func finiteAxisValues(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

// axisInPlane resolves an axis variant into plane-local coordinates,
// validating it non-degenerate and coplanar with the profile plane
// (docs/evaluator-design.md §6).
func axisInPlane(a Axis, frame r3.Frame) (axisLine2, error) {
	switch a := a.(type) {
	case SketchLine:
		for _, c := range []float64{a.Start.U, a.Start.V, a.End.U, a.End.V} {
			if math.IsNaN(c) || math.IsInf(c, 0) {
				return axisLine2{}, fmt.Errorf(`%w: a sketch-line axis endpoint is not finite`, ErrNotFinite)
			}
		}
		du, dv := a.End.U-a.Start.U, a.End.V-a.Start.V
		if !finiteAxisValues(du, dv) {
			return axisLine2{}, fmt.Errorf(`%w: a sketch-line axis delta is not finite`, ErrNotFinite)
		}
		scale := math.Max(math.Abs(du), math.Abs(dv))
		if scale == 0 {
			return axisLine2{}, fmt.Errorf(`%w: a zero-length sketch line names no axis`, ErrDegenerate)
		}
		scaledU, scaledV := du/scale, dv/scale
		l := math.Hypot(scaledU, scaledV)
		if !finiteAxisValues(l) {
			return axisLine2{}, fmt.Errorf(`%w: a sketch-line axis length is not finite`, ErrNotFinite)
		}
		dU, dV := scaledU/l, scaledV/l
		if !finiteAxisValues(dU, dV) {
			return axisLine2{}, fmt.Errorf(`%w: a sketch-line axis direction is not finite`, ErrNotFinite)
		}
		// Recover the held length for exact direction-bound checks. An overflowing
		// magnitude falls back to the conservative direction bound.
		l = scale * l
		dUBound, dVBound := sketchAxisDirectionBounds(a, l, dU, dV)
		return axisLine2{
			aU: a.Start.U, aV: a.Start.V,
			dU: dU, dV: dV,
			dUBound: dUBound,
			dVBound: dVBound,
		}, nil
	case ConstructionAxis:
		for _, c := range []float64{a.Origin.X, a.Origin.Y, a.Origin.Z, a.Dir.X, a.Dir.Y, a.Dir.Z} {
			if math.IsNaN(c) || math.IsInf(c, 0) {
				return axisLine2{}, fmt.Errorf(`%w: a construction axis component is not finite`, ErrNotFinite)
			}
		}
		dir, ok := a.Dir.Normalize()
		if !ok {
			return axisLine2{}, fmt.Errorf(`%w: a zero-direction construction axis names no axis`, ErrDegenerate)
		}
		if !finiteAxisValues(dir.X, dir.Y, dir.Z) {
			return axisLine2{}, fmt.Errorf(`%w: a normalized construction axis direction is not finite`, ErrNotFinite)
		}
		local := frame.ToLocal(a.Origin)
		if !finiteAxisValues(local.X, local.Y, local.Z) {
			return axisLine2{}, fmt.Errorf(`%w: a construction axis has non-finite plane-local coordinates`, ErrNotFinite)
		}
		localLen := local.Len()
		if !finiteAxisValues(localLen) {
			return axisLine2{}, fmt.Errorf(`%w: a construction axis plane-local length is not finite`, ErrNotFinite)
		}
		scale := math.Max(1, localLen)
		if math.Abs(local.Z) > 1e-9*scale {
			return axisLine2{}, fmt.Errorf(`%w: the revolve axis does not lie in the profile plane`, ErrDegenerate)
		}
		du, dv, dn := dir.Dot(frame.U()), dir.Dot(frame.V()), dir.Dot(frame.N())
		if !finiteAxisValues(du, dv, dn) {
			return axisLine2{}, fmt.Errorf(`%w: a construction axis has a non-finite plane-local direction`, ErrNotFinite)
		}
		if math.Abs(dn) > 1e-9 {
			return axisLine2{}, fmt.Errorf(`%w: the revolve axis does not lie in the profile plane`, ErrDegenerate)
		}
		l := math.Hypot(du, dv)
		if !finiteAxisValues(l) {
			return axisLine2{}, fmt.Errorf(`%w: a construction axis plane-local direction length is not finite`, ErrNotFinite)
		}
		if l == 0 {
			return axisLine2{}, fmt.Errorf(`%w: a construction axis has no direction in the profile plane`, ErrDegenerate)
		}
		dU, dV := du/l, dv/l
		if !finiteAxisValues(dU, dV) {
			return axisLine2{}, fmt.Errorf(`%w: a normalized construction axis plane-local direction is not finite`, ErrNotFinite)
		}
		// The anchor's plane-local coordinates take the ROUNDING their own
		// projection committed (bounds.go's exactFrameLocalRound), measured
		// exactly against the frame and the world origin as the exact leaves
		// they are — zero for an exactly representable projection, and never
		// the anchor's own distance from the frame origin, which bounds the
		// coordinate's magnitude and not its error. The magnitude envelope
		// survives only as the fallback for a component no rational holds.
		anchorUpper := absSumUpper(
			a.Origin.X, a.Origin.Y, a.Origin.Z,
			frame.Origin().X, frame.Origin().Y, frame.Origin().Z,
		)
		aUBound := math.Min(
			exactFrameLocalRound(frame, a.Origin, frame.U(), local.X),
			conservativeValueError(local.X, anchorUpper),
		)
		aVBound := math.Min(
			exactFrameLocalRound(frame, a.Origin, frame.V(), local.Y),
			conservativeValueError(local.Y, anchorUpper),
		)
		dUBound, dVBound := conservativeValueError(dU, 1), conservativeValueError(dV, 1)
		if (dU == 0 || math.Abs(dU) == 1) &&
			(dV == 0 || math.Abs(dV) == 1) &&
			dU*dU+dV*dV == 1 {
			dUBound, dVBound = 0, 0
		}
		return axisLine2{
			aU: local.X, aV: local.Y,
			aUBound: aUBound,
			aVBound: aVBound,
			dU:      dU, dV: dV,
			dUBound: dUBound,
			dVBound: dVBound,
		}, nil
	default:
		// EdgeAxis is gated before extent resolution; any other variant is
		// staged, never guessed.
		return axisLine2{}, fmt.Errorf(`%w: axis %T is not supported by this evaluator`, ErrUnsupported, a)
	}
}

func sketchAxisDirectionBounds(a SketchLine, heldLength, heldU, heldV float64) (float64, float64) {
	u0, v0 := floatRat(a.Start.U), floatRat(a.Start.V)
	u1, v1 := floatRat(a.End.U), floatRat(a.End.V)
	length := floatRat(heldLength)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil || length == nil || length.Sign() == 0 {
		return conservativeValueError(heldU, 1), conservativeValueError(heldV, 1)
	}
	du := new(big.Rat).Sub(u1, u0)
	dv := new(big.Rat).Sub(v1, v0)
	lengthSquared := new(big.Rat).Add(
		new(big.Rat).Mul(du, du),
		new(big.Rat).Mul(dv, dv),
	)
	if new(big.Rat).Mul(length, length).Cmp(lengthSquared) != 0 {
		return conservativeValueError(heldU, 1), conservativeValueError(heldV, 1)
	}
	componentBound := func(delta *big.Rat, held float64) float64 {
		exact := new(big.Rat).Quo(delta, length)
		heldRat := floatRat(held)
		if heldRat != nil && exact.Cmp(heldRat) == 0 {
			return 0
		}
		return conservativeValueError(held, 1)
	}
	return componentBound(du, heldU), componentBound(dv, heldV)
}

// axisFrame is the revolve axis as a proper plane-local frame with the
// region on its non-negative side: z = (p−a)·d runs along the axis and
// ρ = cross(d, p−a) ≥ 0 is the radial coordinate. snapTol is the
// scale-relative tolerance that classified axis contact; a coordinate
// within it of the axis IS on the axis.
type axisFrame struct {
	aU, aV           float64
	aUBound, aVBound float64
	dU, dV           float64
	dUBound, dVBound float64
	snapTol          float64
}

// toAxis maps a plane-local point into (z, ρ) axis coordinates. It reads
// aU/aV/dU/dV as exact leaves and states no bound of its own: axisMoments
// (below) folds their proven dUBound/dVBound/aUBound/aVBound into the
// region's moments through bounded arithmetic instead, and a reading built
// from ONE point's re-expressed ρ takes toAxisRhoBound beside it.
func (ax axisFrame) toAxis(u, v float64) (float64, float64) {
	du, dv := u-ax.aU, v-ax.aV
	return du*ax.dU + dv*ax.dV, dv*ax.dU - du*ax.dV
}

// toAxisRhoBound bounds how far the ρ toAxis computes for plane-local point
// (u, v) can sit from the ρ the axis's own TRUE (unrounded) direction and
// anchor would give, folding in dUBound/dVBound/aUBound/aVBound
// (axisInPlane) the same way axisMoments already folds them into the
// region's moments — read here for one point through the same bounded
// arithmetic (moments.go) rather than a whole integral. u and v are exact
// recorded coordinates (a walk's own startU/startV/cU/cV before axisFrame.walk
// re-expresses them); a caller whose (u, v) is itself only bounded folds that
// in separately.
func (ax axisFrame) toAxisRhoBound(u, v float64) float64 {
	du := boundedSub(exactScalar(u), measuredScalar(ax.aU, ax.aUBound))
	dv := boundedSub(exactScalar(v), measuredScalar(ax.aV, ax.aVBound))
	dU := measuredScalar(ax.dU, ax.dUBound)
	dV := measuredScalar(ax.dV, ax.dVBound)
	rho := boundedSub(boundedMul(dv, dU), boundedMul(du, dV))
	return rho.bound
}

// radialUpper is the single owner of the ρ envelope: a proven upper bound on
// the radial distance ρ = |cross(d, p−a)| from THIS resolved axis to any
// boundary point p the caller's own coordUpper covers. coordUpper is a proven
// upper bound on p's plane-local coordinates about the FRAME origin
// (profileCoordinateUpper for a whole profile, segmentWalk.coordUpper for one
// walk), and ρ is measured from the AXIS, so the anchor a's own offset is the
// whole difference between the two: |p−a| ≤ |p| + |a|, with the anchor read
// through its own recorded bounds. Every reading whose error scales with ρ —
// a walk's swept radius and moment envelopes, and the sweep-extreme
// perturbation sweepBoundAlong charges — takes its envelope from here, because a
// caller that charges the frame-origin envelope instead understates the
// reading without limit as the axis moves away from the frame origin.
func (ax axisFrame) radialUpper(coordUpper float64) float64 {
	return absSumUpper(
		coordUpper,
		ax.aU, ax.aUBound,
		ax.aV, ax.aVBound,
	)
}

// planeDirection is the PLANE-LOCAL direction whose extreme over the recorded
// boundary is the extreme of the axis-coordinate functional wg·z + k·ρ. It is
// the rotation that carries (z, ρ) back to (u, v), spelled once here so the
// reading that evaluates the extreme (axisExtremeContext) and any later reading
// over the same functional cannot drift apart by spelling it twice.
func (ax axisFrame) planeDirection(wg, k float64) (float64, float64) {
	return wg*ax.dU - k*ax.dV, wg*ax.dV + k*ax.dU
}

// walk re-expresses one boundary walk in axis coordinates (the U fields
// carry z, the V fields ρ), snapping an endpoint within snapTol onto the
// axis so contact classification and vertex placement agree exactly.
//
// The re-expressed tangent keeps the bound the plane-local tangent proved
// (tanInBound/tanOutBound): the rotation itself contributes error of its own,
// through the frame's direction and its two rounded products, and that error is
// the frame's rather than the walk's. It is charged nowhere here for the
// TANGENT — the same place the re-expressed endpoints leave it for
// CONTACT CLASSIFICATION and vertex placement, both decided against snapTol's
// own generous margin — so those uses speak for the walk's own arithmetic
// under an exactly-stated frame. An axis along a coordinate direction through
// the origin is that frame exactly: every product is by 1 or 0 and nothing
// rounds. The ρ (V) component's OWN proven bound is stated separately, in
// startVBound/endVBound/cVBound, through toAxisRhoBound: a reading that folds
// the re-expressed ρ into a published measurement (survey.go's
// revolveMinRadius) takes it rather than treating startV/endV/cV as an exact
// leaf the way contact classification does.
//
// The SNAP is charged into that same bound, through bounds.go's
// snapToZeroAllow: assigning exactly 0 to an endpoint the arithmetic put a
// positive distance from the axis displaces it by that whole discarded
// magnitude, which is error the walk commits here and nowhere else. So a
// snapped endpoint's startVBound/endVBound covers the assigned zero rather than
// the coordinate it replaced, and only an endpoint the arithmetic already put
// exactly ON the axis keeps a zero bound. Charging it leaves the snap itself
// untouched: the assigned value, and with it every classification and vertex
// placement decided on snapTol's margin, is exactly what it was.
func (ax axisFrame) walk(w segmentWalk) segmentWalk {
	out := w
	out.startU, out.startV = ax.toAxis(w.startU, w.startV)
	out.endU, out.endV = ax.toAxis(w.endU, w.endV)
	out.startVBound = ax.toAxisRhoBound(w.startU, w.startV)
	out.endVBound = ax.toAxisRhoBound(w.endU, w.endV)
	out.tanInU = w.tanInU*ax.dU + w.tanInV*ax.dV
	out.tanInV = w.tanInV*ax.dU - w.tanInU*ax.dV
	out.tanOutU = w.tanOutU*ax.dU + w.tanOutV*ax.dV
	out.tanOutV = w.tanOutV*ax.dU - w.tanOutU*ax.dV
	if m := math.Abs(out.startV); m <= ax.snapTol {
		out.startVBound = snapToZeroAllow(out.startVBound, m)
		out.startV = 0
	}
	if m := math.Abs(out.endV); m <= ax.snapTol {
		out.endVBound = snapToZeroAllow(out.endVBound, m)
		out.endV = 0
	}
	if w.isCircular() {
		beta := math.Atan2(ax.dV, ax.dU)
		out.cU, out.cV = ax.toAxis(w.cU, w.cV)
		out.cVBound = ax.toAxisRhoBound(w.cU, w.cV)
		out.th0 = w.th0 - beta
		out.th1 = w.th1 - beta
	}
	rhoUpper := ax.radialUpper(w.coordUpper)
	out.axisRadiusUpper = rhoUpper
	out.axisMomentUpper = productUpper(w.lengthUpper, rhoUpper)
	return out
}

// wallKind classifies what one boundary walk sweeps.
type wallKind int

const (
	// wallAxis is a line lying along the axis: it sweeps a zero-area set
	// and emits no face — the neighboring segments' faces close the solid.
	wallAxis wallKind = iota
	// wallCylinder is a line parallel to the axis.
	wallCylinder
	// wallPlane is a line perpendicular to the axis: a planar annulus, or a
	// disk when it reaches the axis.
	wallPlane
	// wallCone is an inclined line; an endpoint on the axis is its apex.
	wallCone
	// wallSphere is a circular walk whose center lies on the axis.
	wallSphere
	// wallTorus is a circular walk whose center lies off the axis.
	wallTorus
)

// classify names the surface of revolution one axis-coordinate walk sweeps.
func (ax axisFrame) classify(w segmentWalk) wallKind {
	if w.isCircular() {
		if math.Abs(w.cV) <= ax.snapTol {
			return wallSphere
		}
		return wallTorus
	}
	if w.startV == 0 && w.endV == 0 {
		return wallAxis
	}
	dz, dr := w.endU-w.startU, w.endV-w.startV
	l := math.Hypot(dz, dr)
	if math.Abs(dr) <= 1e-9*l {
		return wallCylinder
	}
	if math.Abs(dz) <= 1e-9*l {
		return wallPlane
	}
	return wallCone
}

// resolveAxisSide orients the axis so the recorded region lies on its
// non-negative-ρ side, enforcing the §6 half-plane and contact rules: a
// region with boundary on both sides of the axis is rejected, as is a curve
// tangent to the axis at an interior point (a circle kissing the axis would
// sweep a self-touching horn torus). It returns the oriented frame and the
// side sign the sweep interval must be remapped by.
//
// The two radial extremes are read through the boundary scan's bounded form,
// so a section whose extreme rides a computed arc radius arrives with a proven
// interval rather than a held float claiming to be exact. Each of the three
// decisions below is taken on that interval and must be DECIDED by it: the
// side sign is a structural fact about the region, not a measurement, so a
// radial extreme whose own interval straddles the ±tol band names no side and
// is refused rather than guessed. The band is 1e-9 of the section's own scale
// and the bound is the scan's own last-ulp figure, so an ordinary section is
// decided with eight orders of magnitude to spare; reaching the refusal means
// the region genuinely sits on the axis to within its own arithmetic.
func resolveAxisSide(profile ProfileRecord, line axisLine2, work *freeformWork) (axisFrame, float64, error) {
	ctx := context.Background()
	nU, nV := -line.dV, line.dU
	rlo, rhi, rBound, err := boundaryExtremesBoundedContext(ctx, profile, nU, nV, work, nil)
	if err != nil {
		return axisFrame{}, 0, err
	}
	roff := nU*line.aU + nV*line.aV
	rlo, rhi = rlo-roff, rhi-roff
	zlo, zhi, _, err := boundaryExtremesBoundedContext(ctx, profile, line.dU, line.dV, work, nil)
	if err != nil {
		return axisFrame{}, 0, err
	}
	zoff := line.dU*line.aU + line.dV*line.aV
	zlo, zhi = zlo-zoff, zhi-zoff

	scale := math.Max(math.Max(math.Abs(rlo), math.Abs(rhi)), math.Max(math.Abs(zlo), math.Abs(zhi)))
	tol := 1e-9 * math.Max(1, scale)
	// Each side test is decided on the extreme's own proven interval: above
	// names "clear of the axis on the + side", below "clear on the − side",
	// and an interval spanning the threshold decides neither.
	hiAbove := admitAbove(measuredScalar(rhi, rBound), tol)
	loBelow := admitBelow(measuredScalar(rlo, rBound), -tol)
	if hiAbove == survStraddle || loBelow == survStraddle {
		return axisFrame{}, 0, fmt.Errorf(`%w: the recorded region's radial extreme about this axis is known only to ±%v mm, which does not decide which side of the axis the region lies on`, ErrDegenerate, rBound)
	}
	switch {
	case hiAbove == survAdmit && loBelow == survAdmit:
		return axisFrame{}, 0, fmt.Errorf(`%w: the revolve axis passes through the region`, ErrDegenerate)
	case hiAbove == survReject && loBelow == survReject:
		return axisFrame{}, 0, fmt.Errorf(`%w: the region collapses onto the revolve axis`, ErrDegenerate)
	}
	side := 1.0
	if hiAbove == survReject {
		side = -1
	}
	ax := axisFrame{
		aU: line.aU, aV: line.aV,
		aUBound: line.aUBound, aVBound: line.aVBound,
		dU: side * line.dU, dV: side * line.dV,
		dUBound: line.dUBound, dVBound: line.dVBound,
		snapTol: tol,
	}
	if err := ax.rejectInteriorContact(profile, work); err != nil {
		return axisFrame{}, 0, err
	}
	return ax, side, nil
}

// rejectInteriorContact rejects the circular boundary walks a revolve
// cannot sweep soundly: a walk tangent to the axis at a point interior to
// the walk — the horn-torus contact §6 forbids — and a walk whose circle
// center lies across the axis, whose swept surface is a spindle-branch
// torus the shipped Torus (non-negative Major) cannot represent; the solid
// is valid, so that one is the staged ErrUnsupported, never a wrong face.
// For the tangency, the circle's radial minimum sits at its lowest angle;
// when that angle is strictly inside the walked range and the minimum
// reaches the axis, the contact is neither of the two allowed forms.
func (ax axisFrame) rejectInteriorContact(profile ProfileRecord, work *freeformWork) error {
	const angEps = 1e-9
	loops := append([]LoopRecord{profile.Outer}, profile.Holes...)
	for _, loop := range loops {
		for _, seg := range loop.Segments {
			w, err := walkOf(seg, work)
			if err != nil {
				return err
			}
			if err := requireAnalyticWalk(w, "the revolve axis-contact audit"); err != nil {
				return err
			}
			w = ax.walk(w)
			if !w.isCircular() {
				continue
			}
			if w.cV < -ax.snapTol {
				return fmt.Errorf(`%w: a boundary arc centered across the revolve axis sweeps a spindle torus this evaluator cannot represent`, ErrUnsupported)
			}
			if w.cV-w.radius > ax.snapTol {
				continue
			}
			lo, hi := math.Min(w.th0, w.th1), math.Max(w.th0, w.th1)
			if w.closed {
				return fmt.Errorf(`%w: a closed curve touching the revolve axis sweeps a self-touching solid`, ErrDegenerate)
			}
			// The minimum-ρ angle is −π/2 modulo a full turn.
			for th := -math.Pi/2 + 2*math.Pi*math.Floor((lo+math.Pi/2)/(2*math.Pi)); th <= hi+angEps; th += 2 * math.Pi {
				if th > lo+angEps && th < hi-angEps {
					return fmt.Errorf(`%w: the boundary touches the revolve axis at an interior point`, ErrDegenerate)
				}
			}
		}
	}
	return nil
}
