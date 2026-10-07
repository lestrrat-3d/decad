package revolveaxis

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// Frame holds the resolved axis coordinates and their proven rounding bounds.
type Frame struct {
	AU, AV, AUBound, AVBound float64
	DU, DV, DUBound, DVBound float64
	SnapTol                  float64
}

// ToAxis maps a plane-local point into (z, ρ) axis coordinates. It reads
// aU/aV/dU/dV as exact leaves and states no bound of its own: decad's
// axisMoments folds their proven dUBound/dVBound/aUBound/aVBound into the
// region's moments through bounded arithmetic instead, and a reading built
// from ONE point's re-expressed ρ takes ToAxisRhoBound beside it.
func (ax Frame) ToAxis(u, v float64) (float64, float64) {
	du, dv := u-ax.AU, v-ax.AV
	return du*ax.DU + dv*ax.DV, dv*ax.DU - du*ax.DV
}

// ToAxisRhoBound bounds how far the ρ ToAxis computes for plane-local point
// (u, v) can sit from the ρ the axis's own TRUE (unrounded) direction and
// anchor would give, folding in dUBound/dVBound/aUBound/aVBound
// (axisInPlane) the same way axisMoments already folds them into the
// region's moments — read here for one point through the same bounded
// arithmetic rather than a whole integral. u and v are exact
// recorded coordinates (a walk's own startU/startV/cU/cV before Frame.Walk
// re-expresses them); a caller whose (u, v) is itself only bounded folds that
// in separately.
func (ax Frame) ToAxisRhoBound(u, v float64) float64 {
	du := proofbound.BoundedSub(proofbound.ExactScalar(u), proofbound.MeasuredScalar(ax.AU, ax.AUBound))
	dv := proofbound.BoundedSub(proofbound.ExactScalar(v), proofbound.MeasuredScalar(ax.AV, ax.AVBound))
	dU := proofbound.MeasuredScalar(ax.DU, ax.DUBound)
	dV := proofbound.MeasuredScalar(ax.DV, ax.DVBound)
	rho := proofbound.BoundedSub(proofbound.BoundedMul(dv, dU), proofbound.BoundedMul(du, dV))
	return rho.Bound
}

// RadialUpper is the single owner of the ρ envelope: a proven upper bound on
// the radial distance ρ = |cross(d, p−a)| from THIS resolved axis to any
// boundary point p the caller's own coordUpper covers. coordUpper is a proven
// upper bound on p's plane-local coordinates about the FRAME origin
// (profileCoordinateUpper for a whole profile, survey2d.SegmentWalk.coordUpper for one
// walk), and ρ is measured from the AXIS, so the anchor a's own offset is the
// whole difference between the two: |p−a| ≤ |p| + |a|, with the anchor read
// through its own recorded bounds. Every reading whose error scales with ρ —
// a walk's swept radius and moment envelopes, and the sweep-extreme
// perturbation sweepBoundAlong charges — takes its envelope from here, because a
// caller that charges the frame-origin envelope instead understates the
// reading without limit as the axis moves away from the frame origin.
func (ax Frame) RadialUpper(coordUpper float64) float64 {
	return proofbound.AbsSumUpper(
		coordUpper,
		ax.AU, ax.AUBound,
		ax.AV, ax.AVBound,
	)
}

// PlaneDirection is the PLANE-LOCAL direction whose extreme over the recorded
// boundary is the extreme of the axis-coordinate functional wg·z + k·ρ. It is
// the rotation that carries (z, ρ) back to (u, v), spelled once here so the
// reading that evaluates the extreme (axisExtremeContext) and any later reading
// over the same functional cannot drift apart by spelling it twice.
func (ax Frame) PlaneDirection(wg, k float64) (float64, float64) {
	return wg*ax.DU - k*ax.DV, wg*ax.DV + k*ax.DU
}

// Walk re-expresses one boundary walk in axis coordinates (the U fields
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
// startVBound/endVBound/cVBound, through ToAxisRhoBound: a reading that folds
// the re-expressed ρ into a published measurement (survey.go's
// revolveMinRadius) takes it rather than treating startV/endV/cV as an exact
// leaf the way contact classification does.
//
// The SNAP is charged into that same bound, through internal/proofbound/bounds.go's
// proofbound.SnapToZeroAllow: assigning exactly 0 to an endpoint the arithmetic put a
// positive distance from the axis displaces it by that whole discarded
// magnitude, which is error the walk commits here and nowhere else. So a
// snapped endpoint's startVBound/endVBound covers the assigned zero rather than
// the coordinate it replaced, and only an endpoint the arithmetic already put
// exactly ON the axis keeps a zero bound. Charging it leaves the snap itself
// untouched: the assigned value, and with it every classification and vertex
// placement decided on snapTol's margin, is exactly what it was.
//
// The snap is charged a SECOND time, into w.lengthBound, because moving an
// endpoint radially moves the wall's two ends apart as well as inward, and
// w.length is the RECORDED, unsnapped segment's length while every wall this
// walk goes on to build runs between the SNAPPED endpoints. Writing the walk in
// axis coordinates as (z0, r0)-(z1, r1) with a = |z1−z0| and the snapped radii
// r0', r1', the recorded length is L = hypot(a, r1−r0) and the built wall's is
// L' = hypot(a, r1'−r0'). Euclidean norm is 1-Lipschitz in each argument, so
//
//	|L' − L| = |hypot(a, r1'−r0') − hypot(a, r1−r0)|
//	         ≤ |(r1'−r0') − (r1−r0)|
//	         ≤ |r1'−r1| + |r0'−r0|
//
// — the sum of the two discarded magnitudes, and no approximation anywhere:
// each step is an inequality in the widening direction, so the charge is an
// upper bound rather than a first-order estimate of one. Both terms go in
// through proofbound.SnapToZeroAllow, which adds under proofbound.UpRound, so the composed float is
// at or above the exact sum; an endpoint the arithmetic already put exactly on
// the axis discards nothing and leaves the length bound untouched, which is
// what keeps every on-axis fixture's wall area exactly as proven.
//
// Charging it is what makes walkAxisMoment's straight arm (revolve_build.go)
// enclose the wall it actually built: that arm reads w.length against a mean
// radius whose own bound already carries the snap, so without this term the
// product covers L·(r0'+r1')/2 while the truth is L'·(r0'+r1')/2, and the
// shortfall |L'−L|·(r0'+r1')/2 grows without limit as the wall turns from steep
// (a cone, where |L'−L| is a fraction of the discarded radius) toward radial (a
// disk, where it is the whole of it).
func (ax Frame) Walk(w survey2d.SegmentWalk) survey2d.SegmentWalk {
	return ax.WalkCharged(w, proofbound.WalkEndBound{}, proofbound.WalkEndBound{})
}

// axisCharge is docs/surface-intersection-design.md §7.1's fold: how far a
// plane-local endpoint displacement (δu, δv) can move that endpoint's two AXIS
// coordinates. ToAxis is the stored-float rotation z = Δu·dU + Δv·dV,
// ρ = Δv·dU − Δu·dV about the resolved anchor, and it is linear, so each
// coordinate moves by at most the two products' sum:
//
//	δz ≤ |δu·dU| + |δv·dV|      δρ ≤ |δv·dU| + |δu·dV|
//
// Every step is a product and a sum of magnitudes through proofbound.ProductUpper and
// proofbound.AbsSumUpper, so no square root and no transcendental appears and no step
// needs an accuracy contract math does not give.
//
// An ABSENT charge answers two zeros and the caller folds nothing. That is not
// a convenience: proofbound.AbsSumUpper up-rounds every term it folds, so composing a
// literal zero would still nudge a published bound by an ulp per term, and an
// untrimmed revolve must read exactly as it reads today.
func (ax Frame) axisCharge(c proofbound.WalkEndBound) (float64, float64) {
	if c.U == 0 && c.V == 0 {
		return 0, 0
	}
	dz := proofbound.AbsSumUpper(proofbound.ProductUpper(math.Abs(c.U), math.Abs(ax.DU)), proofbound.ProductUpper(math.Abs(c.V), math.Abs(ax.DV)))
	drho := proofbound.AbsSumUpper(proofbound.ProductUpper(math.Abs(c.V), math.Abs(ax.DU)), proofbound.ProductUpper(math.Abs(c.U), math.Abs(ax.DV)))
	return dz, drho
}

// WalkCharged is walk with a section displacement folded in, per endpoint.
// startCharge and endCharge are the plane-local per-component displacements
// docs/surface-intersection-design.md §7.1 charges — trimCutChargeUV's own
// figures, taken at exactly the endpoint whose recorded parameter is not a
// natural bound, and at neither endpoint of a segment the arrangement did not
// cut. Both zero is the ordinary revolve, and every fold below is skipped.
//
// Three fields carry the charge and every reading downstream is already
// composed from them, so no reading gains arithmetic of its own:
//
//   - startVBound/endVBound gain δρ, folded BEFORE the axis snap so the snap's
//     own discarded magnitude composes on top of the widened figure.
//   - lengthBound gains both endpoints' total displacement, the chord's own
//     triangle inequality: a segment whose two ends each move by at most that
//     much changes length by at most their sum.
//   - coordUpper and lengthUpper — the ENVELOPES — gain the same figures, so
//     RadialUpper and axisMomentUpper enclose the TRUE meridian rather than
//     only the recorded one. This is the step that makes the charge survive:
//     walkAxisMoment clamps its composed bound with math.Min against
//     proofbound.ConservativeValueError(value, axisMomentUpper), and an envelope covering
//     only the recorded meridian would clamp the charge straight back off.
func (ax Frame) WalkCharged(w survey2d.SegmentWalk, startCharge, endCharge proofbound.WalkEndBound) survey2d.SegmentWalk {
	out := w
	out.StartU, out.StartV = ax.ToAxis(w.StartU, w.StartV)
	out.EndU, out.EndV = ax.ToAxis(w.EndU, w.EndV)
	out.StartVBound = ax.ToAxisRhoBound(w.StartU, w.StartV)
	out.EndVBound = ax.ToAxisRhoBound(w.EndU, w.EndV)
	startZ, startRho := ax.axisCharge(startCharge)
	endZ, endRho := ax.axisCharge(endCharge)
	if startRho > 0 {
		out.StartVBound = proofbound.AbsSumUpper(out.StartVBound, startRho)
	}
	if endRho > 0 {
		out.EndVBound = proofbound.AbsSumUpper(out.EndVBound, endRho)
	}
	out.TanInU = w.TanInU*ax.DU + w.TanInV*ax.DV
	out.TanInV = w.TanInV*ax.DU - w.TanInU*ax.DV
	out.TanOutU = w.TanOutU*ax.DU + w.TanOutV*ax.DV
	out.TanOutV = w.TanOutV*ax.DU - w.TanOutU*ax.DV
	if m := math.Abs(out.StartV); m <= ax.SnapTol {
		out.StartVBound = proofbound.SnapToZeroAllow(out.StartVBound, m)
		out.LengthBound = proofbound.SnapToZeroAllow(out.LengthBound, m)
		out.StartV = 0
	}
	if m := math.Abs(out.EndV); m <= ax.SnapTol {
		out.EndVBound = proofbound.SnapToZeroAllow(out.EndVBound, m)
		out.LengthBound = proofbound.SnapToZeroAllow(out.LengthBound, m)
		out.EndV = 0
	}
	if w.IsCircular() {
		beta := math.Atan2(ax.DV, ax.DU)
		out.CU, out.CV = ax.ToAxis(w.CU, w.CV)
		out.CVBound = ax.ToAxisRhoBound(w.CU, w.CV)
		out.Th0 = w.Th0 - beta
		out.Th1 = w.Th1 - beta
	}
	// §7.1's remaining two folds. chord is the displacement the wall's own two
	// ends can put between them; coord is the L1 magnitude the same two
	// displacements can add to the envelope, which coordUpper is measured in.
	if chord := proofbound.AbsSumUpper(startZ, startRho, endZ, endRho); chord > 0 {
		out.LengthBound = proofbound.AbsSumUpper(out.LengthBound, chord)
		out.LengthUpper = proofbound.AbsSumUpper(out.LengthUpper, chord)
	}
	if coord := math.Max(proofbound.AbsSumUpper(startCharge.U, startCharge.V), proofbound.AbsSumUpper(endCharge.U, endCharge.V)); coord > 0 {
		out.CoordUpper = proofbound.AbsSumUpper(out.CoordUpper, coord)
	}
	rhoUpper := ax.RadialUpper(out.CoordUpper)
	out.AxisRadiusUpper = rhoUpper
	out.AxisMomentUpper = proofbound.ProductUpper(out.LengthUpper, rhoUpper)
	return out
}
