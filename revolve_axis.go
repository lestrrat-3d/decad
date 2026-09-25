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
		// The bracket needs the axis direction's raw, PRE-normalize exact
		// rational dot products against the frame's in-plane axes: rawDU,
		// rawDV = a.Dir·frame.U(), a.Dir·frame.V(). dU/dV above take TWO
		// normalize steps — a.Dir.Normalize() in 3D, then Hypot(du,dv)
		// re-normalizes the projected pair to unit length within the plane
		// — and algebraically the two steps' magnitudes cancel:
		// du = rawDU/|a.Dir|, dv = rawDV/|a.Dir|, so
		// l = Hypot(du,dv) = sqrt(rawDU²+rawDV²)/|a.Dir|, and dU = du/l =
		// rawDU/sqrt(rawDU²+rawDV²) with |a.Dir| gone. The exact closed
		// form these two steps compute is exactly the du/dv shape the
		// SketchLine arm bounds, whatever frame.N() component a.Dir
		// carries — the coplanarity gate above rejects a direction the
		// N component makes materially non-planar, but the bracket below
		// needs no such assumption to be sound.
		rawDU, rawDV := ratVecDot(a.Dir, frame.U()), ratVecDot(a.Dir, frame.V())
		var dUBound, dVBound float64
		if rawDU == nil || rawDV == nil {
			dUBound, dVBound = conservativeValueError(dU, 1), conservativeValueError(dV, 1)
		} else {
			dUBound, dVBound = axisDirectionSqrtBracket(rawDU, rawDV, dU, dV)
		}
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

// ratVecDot is the exact rational dot product of two r3.Vec, each component
// read as the exact rational value its own float64 bit pattern denotes
// (floatRat). It returns nil only for a non-finite component; every caller
// here has already validated its vectors finite.
func ratVecDot(a, b r3.Vec) *big.Rat {
	ax, ay, az := floatRat(a.X), floatRat(a.Y), floatRat(a.Z)
	bx, by, bz := floatRat(b.X), floatRat(b.Y), floatRat(b.Z)
	if ax == nil || ay == nil || az == nil || bx == nil || by == nil || bz == nil {
		return nil
	}
	sum := new(big.Rat).Mul(ax, bx)
	sum.Add(sum, new(big.Rat).Mul(ay, by))
	sum.Add(sum, new(big.Rat).Mul(az, bz))
	return sum
}

func sketchAxisDirectionBounds(a SketchLine, heldLength, heldU, heldV float64) (float64, float64) {
	u0, v0 := floatRat(a.Start.U), floatRat(a.Start.V)
	u1, v1 := floatRat(a.End.U), floatRat(a.End.V)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil {
		return conservativeValueError(heldU, 1), conservativeValueError(heldV, 1)
	}
	du := new(big.Rat).Sub(u1, u0)
	dv := new(big.Rat).Sub(v1, v0)
	fallbackU, fallbackV := axisDirectionSqrtBracket(du, dv, heldU, heldV)
	length := floatRat(heldLength)
	if length == nil || length.Sign() == 0 {
		return fallbackU, fallbackV
	}
	lengthSquared := new(big.Rat).Add(
		new(big.Rat).Mul(du, du),
		new(big.Rat).Mul(dv, dv),
	)
	if new(big.Rat).Mul(length, length).Cmp(lengthSquared) != 0 {
		return fallbackU, fallbackV
	}
	// The held length already proves an exact rational quotient for each
	// component: a Pythagorean or axis-aligned length that lands exactly
	// keeps a zero bound even where the sqrt bracket above cannot collapse
	// to a point (its own float division still rounds).
	exactComponent := func(delta *big.Rat, held, fallback float64) float64 {
		exact := new(big.Rat).Quo(delta, length)
		heldRat := floatRat(held)
		if heldRat != nil && exact.Cmp(heldRat) == 0 {
			return 0
		}
		return fallback
	}
	return exactComponent(du, heldU, fallbackU), exactComponent(dv, heldV, fallbackV)
}

// axisDirectionSqrtBracket proves how far the held unit-direction components
// heldU, heldV — each dU = du/L, dV = dv/L with L = sqrt(du²+dv²) — can sit
// from the axis's own exact direction, through the same sqrt bracket the
// straight-prism campaign proved (segment_walk.go's lineWalkBounds /
// sqrtIntervalError): L² = du²+dv² is exact rational arithmetic, and
// ratSqrtDown/ratSqrtUp (spline_length.go) bracket its root by exact
// comparison, without assuming any libm accuracy from the division that
// produced the held float. du and dv are the axis's own exact-rational
// leaves — a SketchLine's endpoint coordinate differences, or a
// ConstructionAxis's exact rational dot products of its held direction
// against the frame's in-plane axes. A degenerate direction, or a component
// the bracket cannot confirm sits as tightly as this proof can show, keeps
// conservativeValueError's magnitude envelope: math.Min only ever shrinks
// it, never replaces it with a wider answer.
func axisDirectionSqrtBracket(du, dv *big.Rat, heldU, heldV float64) (float64, float64) {
	fallbackU, fallbackV := conservativeValueError(heldU, 1), conservativeValueError(heldV, 1)
	lengthSquared := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
	if lengthSquared.Sign() == 0 {
		return fallbackU, fallbackV
	}
	sqrtIv, ok := intervalSqrt(pointInterval(lengthSquared))
	if !ok {
		return fallbackU, fallbackV
	}
	uBound := fallbackU
	if enc, ok := intervalQuo(pointInterval(du), sqrtIv); ok {
		uBound = math.Min(fallbackU, intervalFloatError(enc, heldU))
	}
	vBound := fallbackV
	if enc, ok := intervalQuo(pointInterval(dv), sqrtIv); ok {
		vBound = math.Min(fallbackV, intervalFloatError(enc, heldV))
	}
	return uBound, vBound
}

// axisFrame is the revolve axis as a proper plane-local frame with the
// region on its non-negative side: z = (p−a)·d runs along the axis and
// ρ = cross(d, p−a) ≥ 0 is the radial coordinate. snapTol is the
// scale-relative tolerance that classified axis contact; a coordinate
// within it of the axis IS on the axis.
//
// radialAdmitAllow and axialExtentUpper are resolveAxisSide's own charge for
// admitting a region under UNCERTAINTY rather than proof: they are zero
// whenever resolveAxisSide proved the region's radial minimum non-negative —
// which is every axis-aligned fixture in the tree, since the arithmetic is
// then exact and admits nothing it has not proven — and otherwise they carry
// the worst case a genuine straddle leaves open. radialAdmitAllow bounds how
// far below zero the TRUE radial minimum can sit despite being admitted, and
// axialExtentUpper bounds the recorded region's own axial reach; together they
// are what revolve_build.go charges into the published volume, cap area and
// centroid bounds (revolvePayload.ax's own doc comment there), since the
// admitted region's faces are built from the SNAPPED profile while the
// integrals behind those measurements read the UNSNAPPED one. Both are the
// zero value for any axisFrame not built by resolveAxisSide (a full-sweep
// composite payload's own literal), which is the safe default: no admitted
// uncertainty, no charge.
//
// snap is the SNAP's own share of that same mismatch, and it answers a
// displacement decad COMMITS rather than one it admits without proof: wherever
// axisFrame.walk assigns an endpoint exactly 0 that the arithmetic put a
// positive distance out, the region whose boundary the built faces follow is no
// longer the recorded one, and every measurement integrated over the recorded
// one owes the difference. Its four fields bound how far the region's own area
// and its three axis-frame moments can move, and revolve_build.go adds each to
// the reading it belongs to. Every one of them is exactly zero for a profile
// whose on-axis endpoints already sit on the axis, which is every axis-incident
// fixture in the tree.
type axisFrame struct {
	aU, aV           float64
	aUBound, aVBound float64
	dU, dV           float64
	dUBound, dVBound float64
	snapTol          float64
	radialAdmitAllow float64
	// radialProof is a strict zero-threshold proof from this profile's own
	// build scan. Only a payload retaining that profile and axis may reuse it.
	radialProof      bool
	axialExtentUpper float64
	snap             regionSnapAllow
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
// through snapToZeroAllow, which adds under upRound, so the composed float is
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
func (ax axisFrame) walk(w segmentWalk) segmentWalk {
	return ax.walkCharged(w, walkEndBound{}, walkEndBound{})
}

// axisCharge is docs/surface-intersection-design.md §7.1's fold: how far a
// plane-local endpoint displacement (δu, δv) can move that endpoint's two AXIS
// coordinates. toAxis is the stored-float rotation z = Δu·dU + Δv·dV,
// ρ = Δv·dU − Δu·dV about the resolved anchor, and it is linear, so each
// coordinate moves by at most the two products' sum:
//
//	δz ≤ |δu·dU| + |δv·dV|      δρ ≤ |δv·dU| + |δu·dV|
//
// Every step is a product and a sum of magnitudes through productUpper and
// absSumUpper, so no square root and no transcendental appears and no step
// needs an accuracy contract math does not give.
//
// An ABSENT charge answers two zeros and the caller folds nothing. That is not
// a convenience: absSumUpper up-rounds every term it folds, so composing a
// literal zero would still nudge a published bound by an ulp per term, and an
// untrimmed revolve must read exactly as it reads today.
func (ax axisFrame) axisCharge(c walkEndBound) (float64, float64) {
	if c.u == 0 && c.v == 0 {
		return 0, 0
	}
	dz := absSumUpper(productUpper(math.Abs(c.u), math.Abs(ax.dU)), productUpper(math.Abs(c.v), math.Abs(ax.dV)))
	drho := absSumUpper(productUpper(math.Abs(c.v), math.Abs(ax.dU)), productUpper(math.Abs(c.u), math.Abs(ax.dV)))
	return dz, drho
}

// walkCharged is walk with a section displacement folded in, per endpoint.
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
//     radialUpper and axisMomentUpper enclose the TRUE meridian rather than
//     only the recorded one. This is the step that makes the charge survive:
//     walkAxisMoment clamps its composed bound with math.Min against
//     conservativeValueError(value, axisMomentUpper), and an envelope covering
//     only the recorded meridian would clamp the charge straight back off.
func (ax axisFrame) walkCharged(w segmentWalk, startCharge, endCharge walkEndBound) segmentWalk {
	out := w
	out.startU, out.startV = ax.toAxis(w.startU, w.startV)
	out.endU, out.endV = ax.toAxis(w.endU, w.endV)
	out.startVBound = ax.toAxisRhoBound(w.startU, w.startV)
	out.endVBound = ax.toAxisRhoBound(w.endU, w.endV)
	startZ, startRho := ax.axisCharge(startCharge)
	endZ, endRho := ax.axisCharge(endCharge)
	if startRho > 0 {
		out.startVBound = absSumUpper(out.startVBound, startRho)
	}
	if endRho > 0 {
		out.endVBound = absSumUpper(out.endVBound, endRho)
	}
	out.tanInU = w.tanInU*ax.dU + w.tanInV*ax.dV
	out.tanInV = w.tanInV*ax.dU - w.tanInU*ax.dV
	out.tanOutU = w.tanOutU*ax.dU + w.tanOutV*ax.dV
	out.tanOutV = w.tanOutV*ax.dU - w.tanOutU*ax.dV
	if m := math.Abs(out.startV); m <= ax.snapTol {
		out.startVBound = snapToZeroAllow(out.startVBound, m)
		out.lengthBound = snapToZeroAllow(out.lengthBound, m)
		out.startV = 0
	}
	if m := math.Abs(out.endV); m <= ax.snapTol {
		out.endVBound = snapToZeroAllow(out.endVBound, m)
		out.lengthBound = snapToZeroAllow(out.lengthBound, m)
		out.endV = 0
	}
	if w.isCircular() {
		beta := math.Atan2(ax.dV, ax.dU)
		out.cU, out.cV = ax.toAxis(w.cU, w.cV)
		out.cVBound = ax.toAxisRhoBound(w.cU, w.cV)
		out.th0 = w.th0 - beta
		out.th1 = w.th1 - beta
	}
	// §7.1's remaining two folds. chord is the displacement the wall's own two
	// ends can put between them; coord is the L1 magnitude the same two
	// displacements can add to the envelope, which coordUpper is measured in.
	if chord := absSumUpper(startZ, startRho, endZ, endRho); chord > 0 {
		out.lengthBound = absSumUpper(out.lengthBound, chord)
		out.lengthUpper = absSumUpper(out.lengthUpper, chord)
	}
	if coord := math.Max(absSumUpper(startCharge.u, startCharge.v), absSumUpper(endCharge.u, endCharge.v)); coord > 0 {
		out.coordUpper = absSumUpper(out.coordUpper, coord)
	}
	rhoUpper := ax.radialUpper(out.coordUpper)
	out.axisRadiusUpper = rhoUpper
	out.axisMomentUpper = productUpper(out.lengthUpper, rhoUpper)
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
//
// roff/zoff are the axis anchor's own offset along each functional: the scan
// above reads the profile about the FRAME origin, and the axis's anchor is
// what shifts that reading onto the axis. A bare `rlo -= roff` would leave
// the two products, their sum, and the subtraction's OWN round-to-nearest
// error unaccounted for, on top of whatever aU/aV/dU/dV's own proven bounds
// already are — the identical gap verify.go's revolvePayloadProvesSimple
// exists to close one level up, only here it feeds the gate that decides
// which side the region is admitted on, not a review AFTER the fact. Reading
// the offset through boundedMul/boundedAdd/boundedSub (the same vocabulary
// axisFrame.toAxisRhoBound already uses for the identical ρ formula at one
// point) charges every one of those roundings into rloB/rhiB, so the strict
// admission gate below reads a number the accompanying bound provably covers.
//
// THE ADMISSION GATE ITSELF is the two-part repair CLAUDE.md's reject-only
// rule requires: admitBelow(near, 0) reads whether the CHOSEN side's
// near-axis extreme is proven negative, proven non-negative, or neither, off
// the extreme's own proven interval — never off a tolerance.
//
//   - Proven negative (survAdmit) refuses outright, however the interval got
//     that wide: a region that truly dips across the axis is a real defect,
//     not an artifact of the bound charging it wide.
//   - Proven non-negative (survReject) needs no allowance, and that holds
//     even where the bound is nonzero — an interval whose own lower end
//     already clears zero commits nothing further. This is what keeps
//     radialAdmitAllow at exactly zero for every axis-aligned fixture: their
//     arithmetic is exact (a zero bound), so admitBelow can only answer
//     survReject or survAdmit, never straddle, and a zero-bound interval that
//     is not proven negative IS proven non-negative.
//   - Neither (survStraddle) is the genuine case a nonzero bound creates — a
//     tilted or offset axis whose own direction/anchor rounding leaves the
//     true radial minimum undecided. Admitting it is sound only because the
//     interval's own worst case is bounded (by the coarse ±tol classification
//     above, which already proved the chosen side's near extreme sits no
//     lower than −tol), and radialAdmitAllow carries exactly that worst case
//     forward to revolve_build.go, which charges it into every published
//     measurement the snap/unsnap mismatch can touch.
func resolveAxisSide(ctx context.Context, profile ProfileRecord, line axisLine2, work *freeformWork) (axisFrame, float64, error) {
	nU, nV := -line.dV, line.dU
	rlo, rhi, rBound, err := boundaryExtremesBoundedContext(ctx, profile, nU, nV, work, nil)
	if err != nil {
		return axisFrame{}, 0, err
	}
	zlo, zhi, zBound, err := boundaryExtremesBoundedContext(ctx, profile, line.dU, line.dV, work, nil)
	if err != nil {
		return axisFrame{}, 0, err
	}

	// boundaryExtremesBoundedContext's own returned bound charges each
	// candidate's POSITIONAL uncertainty (a walked endpoint's own proven
	// displacement, a circular candidate's own enclosure) but NOT the
	// multiply-and-sum arithmetic of evaluating gu·u + gv·v itself for a
	// direction that is not exactly 0, 1 or −1 — the identical gap
	// axisExtremeContext closes for its own, structurally identical scan
	// through planeDotDecompositionRoundAllow (bounds.go). A profile vertex
	// the record states verbatim therefore still commits real rounding
	// forming its dot with a tilted axis's own (nU, nV)/(dU, dV), and that
	// rounding is architecture-sensitive (FMA differs amd64/arm64):
	// left uncharged, the SAME recorded profile can compute as provably
	// negative on one platform and merely uncertain on another, for a region
	// whose true radial minimum is exactly zero. Folding it in here — once,
	// for THIS scan, never composed with axisExtremeContext's own copy of the
	// identical charge on a different reading — is what makes the admission
	// decision agree across platforms: the charge is zero for an axis-aligned
	// direction (planeDotDecompositionRoundAllow's own trivial-coefficient
	// case), so it costs nothing for the tree's axis-aligned fixtures, and it
	// dominates a tilted axis's few-ulp discrepancy by orders of magnitude,
	// which turns a coin-flip sign into a proven straddle everywhere.
	coordUpper, err := profileCoordinateEnvelope(profile, work, nil)
	if err != nil {
		return axisFrame{}, 0, err
	}
	rBound = absSumUpper(rBound, planeDotDecompositionRoundAllow(nU, nV, coordUpper))
	zBound = absSumUpper(zBound, planeDotDecompositionRoundAllow(line.dU, line.dV, coordUpper))

	roffB := boundedAdd(
		boundedMul(measuredScalar(nU, line.dVBound), measuredScalar(line.aU, line.aUBound)),
		boundedMul(measuredScalar(nV, line.dUBound), measuredScalar(line.aV, line.aVBound)),
	)
	zoffB := boundedAdd(
		boundedMul(measuredScalar(line.dU, line.dUBound), measuredScalar(line.aU, line.aUBound)),
		boundedMul(measuredScalar(line.dV, line.dVBound), measuredScalar(line.aV, line.aVBound)),
	)
	rloB := boundedSub(measuredScalar(rlo, rBound), roffB)
	rhiB := boundedSub(measuredScalar(rhi, rBound), roffB)
	zloB := boundedSub(measuredScalar(zlo, zBound), zoffB)
	zhiB := boundedSub(measuredScalar(zhi, zBound), zoffB)

	scale := math.Max(math.Max(math.Abs(rloB.value), math.Abs(rhiB.value)), math.Max(math.Abs(zloB.value), math.Abs(zhiB.value)))
	tol := 1e-9 * math.Max(1, scale)
	// Each side test is decided on the extreme's own proven interval: above
	// names "clear of the axis on the + side", below "clear on the − side",
	// and an interval spanning the threshold decides neither.
	hiAbove := admitAbove(rhiB, tol)
	loBelow := admitBelow(rloB, -tol)
	if hiAbove == survStraddle || loBelow == survStraddle {
		return axisFrame{}, 0, fmt.Errorf(`%w: the recorded region's radial extreme about this axis is known only to ±%v mm, which does not decide which side of the axis the region lies on`, ErrDegenerate, math.Max(rloB.bound, rhiB.bound))
	}
	switch {
	case hiAbove == survAdmit && loBelow == survAdmit:
		return axisFrame{}, 0, fmt.Errorf(`%w: the revolve axis passes through the region`, ErrDegenerate)
	case hiAbove == survReject && loBelow == survReject:
		return axisFrame{}, 0, fmt.Errorf(`%w: the region collapses onto the revolve axis`, ErrDegenerate)
	}
	side := 1.0
	near := rloB
	if hiAbove == survReject {
		side = -1
		near = boundedNeg(rhiB)
	}

	var radialAdmitAllow float64
	var radialProof bool
	switch admitBelow(near, 0) {
	case survAdmit:
		return axisFrame{}, 0, fmt.Errorf(`%w: the recorded region's radial minimum about this axis is proven negative, so the axis cuts through material`, ErrDegenerate)
	case survStraddle:
		radialAdmitAllow = math.Max(0, near.bound-near.value)
	case survReject:
		// The positive-side reading is the same extreme and offset that
		// revolvePayloadProvesSimple would scan again. The build adds an
		// extra dot-product charge, so this proof is at least as strict.
		// A flipped axis keeps the old scan because its coefficient signs
		// and rounding path differ.
		radialProof = side > 0
	}

	axialExtent := boundedSub(zhiB, zloB)
	axialExtentUpper := absSumUpper(axialExtent.value, axialExtent.bound)

	ax := axisFrame{
		aU: line.aU, aV: line.aV,
		aUBound: line.aUBound, aVBound: line.aVBound,
		dU: side * line.dU, dV: side * line.dV,
		dUBound:          line.dUBound,
		dVBound:          line.dVBound,
		snapTol:          tol,
		radialAdmitAllow: radialAdmitAllow,
		radialProof:      radialProof,
		axialExtentUpper: axialExtentUpper,
	}
	snap, err := ax.auditAxisContact(profile, work)
	if err != nil {
		return axisFrame{}, 0, err
	}
	ax.snap = snap
	return ax, side, nil
}

// regionSnapAllow is one profile's accumulated snap allowance, one field per
// integral the revolve publishes a measurement from: area bounds Σ|Δ∫dA| (the
// cap's own reading), first bounds Σ|Δ∫ρ dA| (Pappus's second theorem, so the
// volume), mixed bounds Σ|Δ∫zρ dA| (the solid centroid's axial term) and second
// bounds Σ|Δ∫ρ² dA| (a partial sweep's in-plane centroid term). ρ and z are the
// AXIS-frame coordinates, which is what keeps the last three tight: a profile
// far down the axis has a large |z| and a small ρ, and charging both at one
// frame-origin envelope would inflate the volume by the axial offset.
type regionSnapAllow struct {
	area   float64
	first  float64
	mixed  float64
	second float64
}

// add composes another walk's allowance into this one, each order summed
// outward. The integrals are additive over the boundary, so the region's total
// displacement is at most its walks' displacements summed.
func (a regionSnapAllow) add(b regionSnapAllow) regionSnapAllow {
	return regionSnapAllow{
		area:   absSumUpper(a.area, b.area),
		first:  absSumUpper(a.first, b.first),
		mixed:  absSumUpper(a.mixed, b.mixed),
		second: absSumUpper(a.second, b.second),
	}
}

// snapAllowOf is ONE walk's contribution to the region snap allowance, and the
// place the charge is derived.
//
// axisFrame.walk assigns an endpoint exactly 0 when the arithmetic put it
// within snapTol of the axis, discarding a magnitude δ ≤ snapTol. The built
// wall follows the snapped walk, the region integrals follow the recorded one,
// and the two curves bound a ribbon between them. Every point of that ribbon
// sits within δ of the recorded walk measured radially, so the ribbon lies in a
// band of width δ along a curve no longer than the longer of the two walks:
//
//	area(ribbon) ≤ δ · max(L, L') ≤ δ · (w.length + w.lengthBound)
//
// — w.lengthBound already carries both discarded magnitudes by the time this
// reads it (axisFrame.walk's own doc comment), so absSumUpper over the pair
// covers whichever of the two is longer.
//
// The symmetric difference between the recorded region and the snapped one is
// contained in the union of those ribbons, so for any integrand f,
// |Δ∫f dA| ≤ area(ribbon) · sup|f| over the ribbon, and each order's charge is
// that product against the matching envelope: nothing for ∫dA, one radial
// envelope for ∫ρ dA, a radial and an axial one for ∫zρ dA, and two radial ones
// for ∫ρ² dA.
//
// Every step widens. productUpper and absSumUpper each round outward, δ is the
// discarded magnitude itself rather than an estimate of what it costs, and
// every sup|f| is replaced by an envelope that dominates it. A walk with
// nothing discarded contributes exactly zero, which is what leaves an
// axis-incident profile's published volume, cap area and centroid as proven as
// they were.
//
// A CIRCULAR walk with a snapped endpoint takes the same charge although its
// built surface keeps the recorded circle's own center, radius and angles: the
// snap still displaces the endpoint its neighbouring walls meet it at, by the
// same δ over the same walk, so the same ribbon dominates the difference.
func snapAllowOf(walked segmentWalk, discarded float64) regionSnapAllow {
	if !(discarded > 0) {
		return regionSnapAllow{}
	}
	ribbon := productUpper(absSumUpper(walked.length, walked.lengthBound), discarded)
	rhoUp := walkRadialUpper(walked, discarded)
	// |z| = |(p−a)·d| ≤ |p−a| ≤ |p| + |a|, which is exactly what
	// axisFrame.radialUpper composes — it is the envelope of the whole axis-
	// local position, so it bounds the axial coordinate as well as the radial
	// one, and for a profile far down the axis it is the axial one that is
	// large.
	zUp := absSumUpper(walked.axisRadiusUpper, discarded)
	first := productUpper(ribbon, rhoUp)
	return regionSnapAllow{
		area:   ribbon,
		first:  first,
		mixed:  productUpper(first, zUp),
		second: productUpper(first, rhoUp),
	}
}

// walkRadialUpper is a proven upper bound on |ρ| over one walk already
// re-expressed in axis coordinates, widened by the snap magnitude discarded
// along it. A straight walk's ρ runs linearly between its two endpoints, so its
// extremes ARE those endpoints, each read through the radial bound
// axisFrame.walk proved for it; a circular walk reaches at most its center's
// radial coordinate plus its radius, each read through its own bound. The
// answer is capped by the walk's own axis-radius envelope, which is proven
// independently, so this can only ever tighten and never widen it.
func walkRadialUpper(w segmentWalk, discarded float64) float64 {
	held := absSumUpper(
		math.Max(math.Abs(w.startV), math.Abs(w.endV)),
		math.Max(w.startVBound, w.endVBound),
		discarded,
	)
	if w.isCircular() {
		held = absSumUpper(math.Abs(w.cV), w.cVBound, w.radius, w.radiusBound, discarded)
	}
	return math.Min(held, absSumUpper(w.axisRadiusUpper, discarded))
}

// snapDiscarded is the largest radial magnitude axisFrame.walk's snap discards
// over one walk's two endpoints, read from the walk BEFORE it was re-expressed
// — the same toAxis reading and the same snapTol comparison walk itself makes,
// so the two can never disagree about whether an endpoint snapped.
func (ax axisFrame) snapDiscarded(w segmentWalk) float64 {
	var discarded float64
	for _, end := range [][2]float64{{w.startU, w.startV}, {w.endU, w.endV}} {
		_, rho := ax.toAxis(end[0], end[1])
		if m := math.Abs(rho); m <= ax.snapTol && m > discarded {
			discarded = m
		}
	}
	return discarded
}

// auditAxisContact makes ONE pass over the profile's recorded walks for two
// jobs that both need every walk re-expressed in axis coordinates.
//
// It rejects the circular boundary walks a revolve cannot sweep soundly: a
// walk tangent to the axis at a point interior to
// the walk — the horn-torus contact §6 forbids — and a walk whose circle
// center lies across the axis, whose swept surface is a spindle-branch
// torus the shipped Torus (non-negative Major) cannot represent; the solid
// is valid, so that one is the staged ErrUnsupported, never a wrong face.
// For the tangency, the circle's radial minimum sits at its lowest angle;
// when that angle is strictly inside the walked range and the minimum
// reaches the axis, the contact is neither of the two allowed forms.
//
// It also sums the region snap allowance (snapAllowOf) the same walks earn,
// here rather than in a second pass of its own: re-walking a profile costs a
// second free-form conversion and a second rational bracket per segment for an
// answer this loop already holds.
func (ax axisFrame) auditAxisContact(profile ProfileRecord, work *freeformWork) (regionSnapAllow, error) {
	const angEps = 1e-9
	var snap regionSnapAllow
	loops := append([]LoopRecord{profile.Outer}, profile.Holes...)
	for _, loop := range loops {
		for _, seg := range loop.Segments {
			w, err := walkOf(seg, work)
			if err != nil {
				return regionSnapAllow{}, err
			}
			if err := requireAnalyticWalk(w, "the revolve axis-contact audit"); err != nil {
				return regionSnapAllow{}, err
			}
			discarded := ax.snapDiscarded(w)
			w = ax.walk(w)
			snap = snap.add(snapAllowOf(w, discarded))
			if !w.isCircular() {
				continue
			}
			if w.cV < -ax.snapTol {
				return regionSnapAllow{}, fmt.Errorf(`%w: a boundary arc centered across the revolve axis sweeps a spindle torus this evaluator cannot represent`, ErrUnsupported)
			}
			if w.cV-w.radius > ax.snapTol {
				continue
			}
			lo, hi := math.Min(w.th0, w.th1), math.Max(w.th0, w.th1)
			if w.closed {
				return regionSnapAllow{}, fmt.Errorf(`%w: a closed curve touching the revolve axis sweeps a self-touching solid`, ErrDegenerate)
			}
			// The minimum-ρ angle is −π/2 modulo a full turn.
			for th := -math.Pi/2 + 2*math.Pi*math.Floor((lo+math.Pi/2)/(2*math.Pi)); th <= hi+angEps; th += 2 * math.Pi {
				if th > lo+angEps && th < hi-angEps {
					return regionSnapAllow{}, fmt.Errorf(`%w: the boundary touches the revolve axis at an interior point`, ErrDegenerate)
				}
			}
		}
	}
	return snap, nil
}
