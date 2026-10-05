package decad

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/units"
)

// This file integrates a recorded circular segment — an arc or a whole
// circle — in exact rational interval arithmetic: for the Green's-theorem
// boundary sums moments.go accumulates, and for the per-wall axis moment
// revolve_build.go's walkAxisMoment reads Pappus's first theorem's area from
// (circularAxisMomentInterval).
//
// Each reader returns an enclosure and a flag, and the flag is false whenever
// the segment's own record does not determine the answer exactly. A false
// flag withholds the exact term and leaves the caller to fall back on its
// float accumulation with that fallback's own bound, never a rational term
// standing in for one the record could not state.

func exactCoordinateDelta(a, b float64) *big.Rat {
	return new(big.Rat).Sub(proofarith.FloatRat(a), proofarith.FloatRat(b))
}

// arcEndRadialRatio brackets ρ = |Start − Center| / |End − Center| for a recorded
// ArcSeg, given the two exact squared distances. The denoted arc runs on Start's
// radius and ends at End's ANGLE, so the point the arc actually ends at is
// Center + ρ·(End − Center); every circular bracket substitutes that for the
// recorded End. ρ is the ratSqrtDown/ratSqrtUp bracket of the exact rational
// r²/endR², rounded outward once at each end and never a float sqrt of a float.
// Equal squared radii answer the exact point 1 — the record states an exact
// circle and the substitution is the identity. A zero endR² (End == Center)
// answers false; the preflight refuses that record before any bracket runs.
func arcEndRadialRatio(r2, endR2 *big.Rat) (ratInterval, bool) {
	if endR2.Sign() == 0 {
		return ratInterval{}, false
	}
	if endR2.Cmp(r2) == 0 {
		return pointInterval(big.NewRat(1, 1)), true
	}
	q := new(big.Rat).Quo(r2, endR2)
	lo, hi := proofarith.FloatRat(ratSqrtDown(q)), proofarith.FloatRat(ratSqrtUp(q))
	if lo == nil || hi == nil {
		return ratInterval{}, false
	}
	return interval(lo, hi), true
}

// circularAreaInterval brackets one circular walk's exact area contribution
// about the walk anchor. The segment holds the RECORDED coordinates and the
// anchor is subtracted here over rationals: every radial term is a difference
// the shift cancels out of exactly, and only the centre term carries it, so
// this bracket stays a proof about the recorded arc rather than about a
// float-shifted copy of it.
//
// A CircleSeg's fractional-turn arm (moments_trig.go's turnSinCosInterval)
// covers a trimmed fragment the same way the whole-turn fast path covers a
// full sweep: every non-trig factor (the radius, the recentred centre
// coordinates, the swept angle) is an exact rational, and only the endpoint
// sine/cosine terms are enclosed, exactly the substitution the ArcSeg arm
// below makes for its own endpoints — differing only in where those
// sine/cosine values come from (an exact ratio there, a certified bracket
// here, because a CircleSeg's endpoints are not recorded coordinates).
func circularAreaInterval(seg CurveSegment, anchor Point2) (ratInterval, bool) {
	anchorU, anchorV := proofarith.FloatRat(anchor.U), proofarith.FloatRat(anchor.V)
	if anchorU == nil || anchorV == nil {
		return ratInterval{}, false
	}
	switch seg := seg.(type) {
	case CircleSeg:
		dt := exactCoordinateDelta(seg.TEnd, seg.TStart)
		radius, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return ratInterval{}, false
		}
		r := proofarith.FloatRat(radius)
		if r == nil {
			return ratInterval{}, false
		}
		if dt.IsInt() {
			// An integer number of turns has equal endpoint sine/cosine terms,
			// leaving exactly dt·π·r².
			scale := new(big.Rat).Mul(dt, new(big.Rat).Mul(r, r))
			return intervalScale(interval(piLower, piUpper), scale), true
		}
		t0, t1 := proofarith.FloatRat(seg.TStart), proofarith.FloatRat(seg.TEnd)
		if t0 == nil || t1 == nil {
			return ratInterval{}, false
		}
		s0, c0 := turnSinCosInterval(t0)
		s1, c1 := turnSinCosInterval(t1)
		centerU := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.U), anchorU)
		centerV := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.V), anchorV)
		piIv := interval(piLower, piUpper)
		dtheta := intervalScale(piIv, new(big.Rat).Mul(big.NewRat(2, 1), dt))
		sector := intervalScale(dtheta, new(big.Rat).Mul(r, r))
		uTerm := intervalScale(intervalSub(s1, s0), new(big.Rat).Mul(centerU, r))
		vTerm := intervalScale(intervalSub(c1, c0), new(big.Rat).Mul(centerV, r))
		// A = ½ ( r²·dθ + c_u·r·(sin θ1 − sin θ0) − c_v·r·(cos θ1 − cos θ0) ) —
		// addCircular's own closed form (moments.go:1505), every non-trig
		// factor exact and every trig factor enclosed.
		return intervalScale(intervalSub(intervalAdd(sector, uTerm), vTerm), big.NewRat(1, 2)), true
	case ArcSeg:
		forward := seg.TStart == 0 && seg.TEnd == 1
		reverse := seg.TStart == 1 && seg.TEnd == 0
		if !forward && !reverse {
			return ratInterval{}, false
		}
		dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
		dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
		dx1 := exactCoordinateDelta(seg.End.U, seg.Center.U)
		dy1 := exactCoordinateDelta(seg.End.V, seg.Center.V)
		// Arc endpoints may retain solver drift within the accepted radius join
		// tolerance. The integrated path uses the start radius and endpoint angles,
		// so the area interval charges the radial mismatch before it is published.
		r2 := new(big.Rat).Add(
			new(big.Rat).Mul(dx0, dx0),
			new(big.Rat).Mul(dy0, dy0),
		)
		endR2 := new(big.Rat).Add(
			new(big.Rat).Mul(dx1, dx1),
			new(big.Rat).Mul(dy1, dy1),
		)
		// The held angles read the same float-shifted coordinates the float
		// evaluation does, so this bracket brackets THAT walk's sweep branch.
		heldCenter := shiftPoint(seg.Center, anchor)
		heldStart := shiftPoint(seg.Start, anchor)
		heldEnd := shiftPoint(seg.End, anchor)
		heldDY0 := heldStart.V - heldCenter.V
		heldDY1 := heldEnd.V - heldCenter.V
		a0 := atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
		a1 := atan2Interval(dy1, dx1, heldDY1 == 0 && math.Signbit(heldDY1))
		sweep := intervalSub(a1, a0)
		heldA0 := math.Atan2(heldStart.V-heldCenter.V, heldStart.U-heldCenter.U)
		heldA1 := math.Atan2(heldEnd.V-heldCenter.V, heldEnd.U-heldCenter.U)
		if heldA1-heldA0 <= 0 {
			sweep = intervalAdd(sweep, twoPiInterval())
		}
		sign := big.NewRat(1, 1)
		dx, dy := new(big.Rat).Sub(dx1, dx0), new(big.Rat).Sub(dy1, dy0)
		if reverse {
			sign.Neg(sign)
			dx.Neg(dx)
			dy.Neg(dy)
		}
		sector := intervalScale(sweep, new(big.Rat).Mul(sign, r2))
		// Only the centre carries the anchor: every radial term above is a
		// difference the shift cancels out of.
		centerU := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.U), anchorU)
		centerV := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.V), anchorV)
		centerTerm := new(big.Rat).Sub(
			new(big.Rat).Mul(centerU, dy),
			new(big.Rat).Mul(centerV, dx),
		)
		areaProof := intervalAdd(sector, pointInterval(centerTerm))
		if endR2.Cmp(r2) != 0 {
			if endR2.Sign() == 0 {
				return ratInterval{}, false
			}
			radialGap := new(big.Rat).Sub(endR2, r2)
			radialGap.Abs(radialGap)
			radialRatioUpper := new(big.Rat).Quo(radialGap, endR2)
			endpointScale := new(big.Rat).Add(
				new(big.Rat).Mul(
					new(big.Rat).Abs(centerU),
					new(big.Rat).Abs(dy1),
				),
				new(big.Rat).Mul(
					new(big.Rat).Abs(centerV),
					new(big.Rat).Abs(dx1),
				),
			)
			correction := new(big.Rat).Mul(radialRatioUpper, endpointScale)
			correctionFloat, exact := correction.Float64()
			if !exact {
				correctionFloat = math.Nextafter(correctionFloat, math.Inf(1))
			}
			correctionFloat = absSumUpper(
				correctionFloat,
				analyticRoundBound(absSumUpper(
					heldCenter.U,
					heldCenter.V,
					heldEnd.U-heldCenter.U,
					heldEnd.V-heldCenter.V,
				)),
			)
			correction = proofarith.FloatRat(correctionFloat)
			areaProof = intervalAdd(
				areaProof,
				intervalScale(interval(big.NewRat(-1, 1), big.NewRat(1, 1)), correction),
			)
		}
		return intervalScale(
			areaProof,
			big.NewRat(1, 2),
		), true
	default:
		return ratInterval{}, false
	}
}

// circularWalkEnclosures brackets the two quantities a recorded circular
// segment's own walk (extrude.go's circularWalk) is BUILT from and the record
// itself states — its RADIUS, and the ABSOLUTE ANGLE that walk sweeps over the
// segment's own recorded parameter range — as exact rational intervals. It is
// the single owner of both brackets, with no dependence on Sin/Cos/Atan2/Hypot's
// undocumented accuracy:
//
//   - a CircleSeg states its radius outright, so the radius is a point interval
//     of the recorded value converted to millimetres, and the walk's sweep is
//     the exact rational turn 2π·|TEnd − TStart|;
//   - an ArcSeg states Start, End and Center only, so the radius is the
//     ratSqrtDown/ratSqrtUp bracket of the exact squared Start-to-Center
//     distance — the same radius circularWalk holds as a math.Hypot float — and
//     the walk's sweep is the atan2Interval difference of the two recorded
//     endpoint angles under the +2π branch correction circularAreaInterval
//     applies, scaled by the recorded |TEnd − TStart|, the same trimming
//     circularWalk applies to its own held a0 + t·sweep angles.
//
// Both are what a consumer needs that would otherwise read the walk's own held
// w.radius and |w.th1 − w.th0|: neither of those floats is a quantity the walk
// can enclose (circularWalk's own doc comment), so a published bound composed
// from them would be a held value wearing a proof's clothes. A record this
// bracket cannot state answers false, and the consumer refuses.
func circularWalkEnclosures(seg CurveSegment) (ratInterval, ratInterval, bool) {
	switch seg := seg.(type) {
	case CircleSeg:
		radius, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return ratInterval{}, ratInterval{}, false
		}
		r := proofarith.FloatRat(radius)
		if r == nil {
			return ratInterval{}, ratInterval{}, false
		}
		r.Abs(r)
		dt := exactCoordinateDelta(seg.TEnd, seg.TStart)
		dt.Abs(dt)
		return pointInterval(r), intervalScale(twoPiInterval(), dt), true
	case ArcSeg:
		dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
		dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
		dx1 := exactCoordinateDelta(seg.End.U, seg.Center.U)
		dy1 := exactCoordinateDelta(seg.End.V, seg.Center.V)
		r2 := new(big.Rat).Add(new(big.Rat).Mul(dx0, dx0), new(big.Rat).Mul(dy0, dy0))
		rLo, rHi := proofarith.FloatRat(ratSqrtDown(r2)), proofarith.FloatRat(ratSqrtUp(r2))
		if rLo == nil || rHi == nil {
			return ratInterval{}, ratInterval{}, false
		}
		heldDY0 := seg.Start.V - seg.Center.V
		heldDY1 := seg.End.V - seg.Center.V
		a0 := atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
		a1 := atan2Interval(dy1, dx1, heldDY1 == 0 && math.Signbit(heldDY1))
		sweep := intervalSub(a1, a0)
		heldA0 := math.Atan2(heldDY0, seg.Start.U-seg.Center.U)
		heldA1 := math.Atan2(heldDY1, seg.End.U-seg.Center.U)
		if heldA1-heldA0 <= 0 {
			sweep = intervalAdd(sweep, twoPiInterval())
		}
		dt := exactCoordinateDelta(seg.TEnd, seg.TStart)
		dt.Abs(dt)
		return interval(rLo, rHi), intervalScale(sweep, dt), true
	default:
		return ratInterval{}, ratInterval{}, false
	}
}

// circularLengthInterval brackets one circular walk's exact arc length as the
// product of circularWalkEnclosures' two brackets: an arc's length IS its radius
// times its swept angle, and a length has no cross-term to bracket, so unlike
// circularAreaInterval and circularFirstMomentInterval it never needed
// moments_trig.go's endpoint sine/cosine enclosure to admit a trimmed fragment.
func circularLengthInterval(seg CurveSegment) (ratInterval, bool) {
	r, sweep, ok := circularWalkEnclosures(seg)
	if !ok {
		return ratInterval{}, false
	}
	return intervalMul(r, sweep), true
}

// circularEndpointInterval encloses the (u, v) position a recorded circular
// segment DENOTES at parameter t, as a pair of exact rational intervals. It is
// circularLengthInterval's endpoint twin — the same recorded data and, for an
// arc, the same swept-angle branch — read AT one parameter instead of over the
// whole range, and it exists because a walk's endpoint at a trimmed parameter
// is a math.Cos/math.Sin evaluation at an angle this package computed, never a
// coordinate the record states.
//
// A CircleSeg's point at t is Center + r·(cos 2πt, sin 2πt) for the recorded
// centre and radius, so the turn is exactly rational and moments_trig.go's
// turnSinCosInterval encloses the pair with no π-comparison anywhere. A whole
// multiple of a quarter turn does not even need the series: its sine and cosine
// are 0 or ±1 exactly (quarterTurnSinCos), which is what keeps a whole circle's
// own endpoint a zero-width reading.
//
// An ArcSeg states no angle at all — three pinned points, swept
// counter-clockwise from Start to End about Center (record.go) — so its point
// at t is Center + r·(cos θ, sin θ) with r the exact Start-to-Center distance
// (ratSqrtDown/ratSqrtUp) and θ = a0 + t·sweep, both angles enclosed by
// atan2Interval under the same +2π branch correction circularLengthInterval
// applies, and the sine and cosine of that enclosed angle taken by
// radSinCosSpan.
//
// The parameter is taken as an EXACT RATIONAL, never a float. A caller reading
// a walk's own endpoint converts its held float parameter (floatRat) at the
// call; one generating a point at a parameter the record's own arithmetic
// states — a uniform station division t_k = TStart + (k/m)·(TEnd − TStart)
// (loft_build.go's circularStationChain) — hands that value in unrounded,
// because rounding it to a float first would enclose the curve at a
// NEIGHBOURING parameter and prove a bound about a point no construction
// named.
func circularEndpointInterval(seg CurveSegment, rt *big.Rat) (ratInterval, ratInterval, bool) {
	return circularOffsetEndpointInterval(seg, rt, new(big.Rat))
}

// circularOffsetEndpointInterval is circularEndpointInterval read on the
// CONCENTRIC circle whose radius is the segment's own plus radiusOffset (an
// exact rational; zero gives the segment itself). The offset joins the radius
// before any product, so for an ArcSeg it shifts BOTH ends of the
// ratSqrtDown/ratSqrtUp bracket and the held radius's own rounding stays
// enclosed rather than assumed. A nonzero offset whose resulting radius is
// not positive denotes no offset circle and answers ok == false. A zero offset
// keeps circularEndpointInterval's collapsed-radius reading unchanged.
func circularOffsetEndpointInterval(seg CurveSegment, rt, radiusOffset *big.Rat) (ratInterval, ratInterval, bool) {
	switch seg := seg.(type) {
	case CircleSeg:
		radius, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return ratInterval{}, ratInterval{}, false
		}
		r := proofarith.FloatRat(radius)
		cu, cv := proofarith.FloatRat(seg.Center.U), proofarith.FloatRat(seg.Center.V)
		if r == nil || cu == nil || cv == nil {
			return ratInterval{}, ratInterval{}, false
		}
		r.Add(r, radiusOffset)
		if radiusOffset.Sign() != 0 && r.Sign() <= 0 {
			return ratInterval{}, ratInterval{}, false
		}
		sin, cos := quarterTurnSinCos(rt)
		return intervalAdd(pointInterval(cu), intervalScale(cos, r)),
			intervalAdd(pointInterval(cv), intervalScale(sin, r)), true
	case ArcSeg:
		cu, cv := proofarith.FloatRat(seg.Center.U), proofarith.FloatRat(seg.Center.V)
		if cu == nil || cv == nil {
			return ratInterval{}, ratInterval{}, false
		}
		dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
		dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
		dx1 := exactCoordinateDelta(seg.End.U, seg.Center.U)
		dy1 := exactCoordinateDelta(seg.End.V, seg.Center.V)
		r2 := new(big.Rat).Add(new(big.Rat).Mul(dx0, dx0), new(big.Rat).Mul(dy0, dy0))
		rLo, rHi := proofarith.FloatRat(ratSqrtDown(r2)), proofarith.FloatRat(ratSqrtUp(r2))
		if rLo == nil || rHi == nil {
			return ratInterval{}, ratInterval{}, false
		}
		rLo.Add(rLo, radiusOffset)
		rHi.Add(rHi, radiusOffset)
		if radiusOffset.Sign() != 0 && rLo.Sign() <= 0 {
			return ratInterval{}, ratInterval{}, false
		}
		heldDY0 := seg.Start.V - seg.Center.V
		heldDY1 := seg.End.V - seg.Center.V
		a0 := atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
		a1 := atan2Interval(dy1, dx1, heldDY1 == 0 && math.Signbit(heldDY1))
		sweep := intervalSub(a1, a0)
		heldA0 := math.Atan2(heldDY0, seg.Start.U-seg.Center.U)
		heldA1 := math.Atan2(heldDY1, seg.End.U-seg.Center.U)
		if heldA1-heldA0 <= 0 {
			sweep = intervalAdd(sweep, twoPiInterval())
		}
		sin, cos, ok := radSinCosSpan(intervalAdd(a0, intervalScale(sweep, rt)))
		if !ok {
			return ratInterval{}, ratInterval{}, false
		}
		r := interval(rLo, rHi)
		return intervalAdd(pointInterval(cu), intervalMul(r, cos)),
			intervalAdd(pointInterval(cv), intervalMul(r, sin)), true
	default:
		return ratInterval{}, ratInterval{}, false
	}
}

// quarterTurnSinCos encloses sin(2πt) and cos(2πt) for an exact rational turn,
// answering with a POINT interval whenever 4t is an integer: the quadrant turns
// have sine and cosine 0 or ±1 exactly, which no series can improve on and a
// series would only widen. Every other turn goes to turnSinCosInterval.
func quarterTurnSinCos(t *big.Rat) (ratInterval, ratInterval) {
	quadrants := new(big.Rat).Mul(t, big.NewRat(4, 1))
	if !quadrants.IsInt() {
		return turnSinCosInterval(t)
	}
	zero, one := new(big.Rat), big.NewRat(1, 1)
	minusOne := big.NewRat(-1, 1)
	switch new(big.Int).Mod(quadrants.Num(), big.NewInt(4)).Int64() {
	case 0:
		return pointInterval(zero), pointInterval(one)
	case 1:
		return pointInterval(one), pointInterval(zero)
	case 2:
		return pointInterval(zero), pointInterval(minusOne)
	default: // 3
		return pointInterval(minusOne), pointInterval(zero)
	}
}

// axisComponentInterval encloses one axis-frame scalar — the anchor's aU/aV
// or the outward normal's nU/nV (nU = −dV, nV = dU, axisMoments' own pairing,
// revolve_build.go) — as an interval centred on its held float and widened by
// its own proven bound: junctionRadiusInterval's construction
// (revolve_build.go), generalized from a junction's radial coordinate to the
// axis's own anchor and direction. A component this evaluator cannot state as
// a finite rational — a +Inf bound, the sqrt bracket a tilted axis's
// direction does not yet carry — answers ok == false, and
// circularAxisMomentInterval refuses with it.
func axisComponentInterval(value, bound float64) (ratInterval, bool) {
	v, b := proofarith.FloatRat(value), proofarith.FloatRat(math.Abs(bound))
	if v == nil || b == nil {
		return ratInterval{}, false
	}
	return intervalWiden(pointInterval(v), b), true
}

// circularAxisMomentInterval brackets one recorded circular segment's exact
// first moment about the revolve axis, M = ∫ρ ds, where ρ = nU·(u−aU) +
// nV·(v−aV) is the axis's own radial coordinate (axisFrame.toAxis) expressed
// in the RECORD's plane-local frame rather than the axis-re-expressed one:
// nU = −dV, nV = dU is the axis's outward normal and aU, aV its anchor
// (axisFrame's own dU/dV/aU/aV, each widened by its proven bound through
// axisComponentInterval — sound for a tilted axis too, narrower only once a
// later sqrt bracket tightens dUBound/dVBound toward zero).
//
// Writing the segment's own arc as (cU + r·cosθ, cV + r·sinθ), ds = r·dθ:
//
//	∫(u−aU) ds = r·[(cU−aU)·Δθ + r·(sin(hi) − sin(lo))]
//	∫(v−aV) ds = r·[(cV−aV)·Δθ + r·(cos(lo) − cos(hi))]
//	M = nU·∫(u−aU) ds + nV·∫(v−aV) ds
//
// lo, hi are the segment's own two angles in ascending order and Δθ = hi−lo
// is UNSIGNED, mirroring walkAxisMoment's pre-existing axis-frame arm
// (lo, hi := min(w.th0, w.th1), max(...)): M sums an unsigned arc-length
// element, never a signed area, so it takes no direction sign the way
// circularAreaInterval's shoelace form does. Ascending T order decides lo/hi
// exactly as it decides w.th0 < w.th1 after axisFrame.walk's constant angular
// shift by β = atan2(dV, dU) (a shift preserves order), so this needs neither
// β nor the walk's own re-expressed th0/th1 — both math.Atan2 results with no
// enclosure — to agree with them.
//
// r and Δθ are circularWalkEnclosures' own brackets; a CircleSeg's endpoint
// sin/cos come from turnSinCosInterval, with an exact zero-width fast path
// when the recorded range spans a whole number of turns (the sine/cosine
// difference terms above vanish exactly, leaving Pappus's own r·Δθ·ρ_centre
// form — the torus/whole-circle case); an ArcSeg's come from radSinCosSpan of
// the atan2Interval endpoint enclosure, exactly as circularEndpointInterval
// evaluates them, and — like circularAreaInterval and
// circularFirstMomentInterval — only over its own full recorded range
// (forward or reverse), never a trimmed fragment, whose actual endpoint the
// record alone does not state.
func circularAxisMomentInterval(seg CurveSegment, ax axisFrame) (ratInterval, bool) {
	r, dtheta, ok := circularWalkEnclosures(seg)
	if !ok {
		return ratInterval{}, false
	}
	var cU, cV *big.Rat
	var sinDiff, cosDiff ratInterval // sin(hi)-sin(lo), cos(lo)-cos(hi)
	switch seg := seg.(type) {
	case CircleSeg:
		cU, cV = proofarith.FloatRat(seg.Center.U), proofarith.FloatRat(seg.Center.V)
		t0, t1 := proofarith.FloatRat(seg.TStart), proofarith.FloatRat(seg.TEnd)
		if cU == nil || cV == nil || t0 == nil || t1 == nil {
			return ratInterval{}, false
		}
		dt := new(big.Rat).Sub(t1, t0)
		switch {
		case dt.IsInt():
			zero := new(big.Rat)
			sinDiff, cosDiff = pointInterval(zero), pointInterval(zero)
		default:
			loT, hiT := t0, t1
			if loT.Cmp(hiT) > 0 {
				loT, hiT = hiT, loT
			}
			sinLo, cosLo := turnSinCosInterval(loT)
			sinHi, cosHi := turnSinCosInterval(hiT)
			sinDiff = intervalSub(sinHi, sinLo)
			cosDiff = intervalSub(cosLo, cosHi)
		}
	case ArcSeg:
		forward := seg.TStart == 0 && seg.TEnd == 1
		reverse := seg.TStart == 1 && seg.TEnd == 0
		if !forward && !reverse {
			return ratInterval{}, false
		}
		cU, cV = proofarith.FloatRat(seg.Center.U), proofarith.FloatRat(seg.Center.V)
		if cU == nil || cV == nil {
			return ratInterval{}, false
		}
		dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
		dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
		heldDY0 := seg.Start.V - seg.Center.V
		a0 := atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
		a1 := intervalAdd(a0, dtheta)
		sinLo, cosLo, ok0 := radSinCosSpan(a0)
		sinHi, cosHi, ok1 := radSinCosSpan(a1)
		if !ok0 || !ok1 {
			return ratInterval{}, false
		}
		sinDiff = intervalSub(sinHi, sinLo)
		cosDiff = intervalSub(cosLo, cosHi)
	default:
		return ratInterval{}, false
	}

	anchorU, ok := axisComponentInterval(ax.aU, ax.aUBound)
	if !ok {
		return ratInterval{}, false
	}
	anchorV, ok := axisComponentInterval(ax.aV, ax.aVBound)
	if !ok {
		return ratInterval{}, false
	}
	nU, ok := axisComponentInterval(-ax.dV, ax.dVBound)
	if !ok {
		return ratInterval{}, false
	}
	nV, ok := axisComponentInterval(ax.dU, ax.dUBound)
	if !ok {
		return ratInterval{}, false
	}

	duU := intervalSub(pointInterval(cU), anchorU)
	duV := intervalSub(pointInterval(cV), anchorV)
	intU := intervalMul(r, intervalAdd(intervalMul(duU, dtheta), intervalMul(r, sinDiff)))
	intV := intervalMul(r, intervalAdd(intervalMul(duV, dtheta), intervalMul(r, cosDiff)))
	return intervalAdd(intervalMul(nU, intU), intervalMul(nV, intV)), true
}

// circularFirstMomentInterval brackets one circular walk's exact first-moment
// contributions (∫u dA, ∫v dA) about the walk anchor: a CircleSeg over any
// recorded range, whole or fractional, and — under a narrower admission — an
// ArcSeg only over its own full recorded range (forward or reverse), never a
// trimmed fragment.
//
// A CircleSeg's whole turns are the enclosed disk's own boundary, whose first
// moment about each axis is its centroid times its area: every odd trig
// moment over a whole period cancels exactly, leaving mu = c.U·r²·π·dt and
// mv = c.V·r²·π·dt (dt the signed turn count). A fractional turn instead
// restates addCircular's own mu/mv closed forms (moments.go:1511/1516) with
// every sine/cosine factor enclosed by moments_trig.go's turnSinCosInterval
// and every other factor — the radius, the recentred centre coordinates, the
// swept angle — taken as an exact rational: the same substitution
// circularAreaInterval's fractional arm makes, one order higher.
//
// An ArcSeg's fragment restates addCircular's own mu/mv closed forms
// (0.5·r·(c.U²·intCos + 2·c.U·r·intCos2 + r²·intCos3), and the mv analogue)
// with sin/cos of the endpoints read as the exact ratios dy/r, dx/r rather
// than evaluated: every r that multiplies one of those ratios cancels it
// back to a rational coordinate difference, and the one term that does not
// cancel — the θ term inside intCos2/intSin2 — is exactly the swept angle
// atan2Interval already brackets. What is left after multiplying through is
// rational except for that single c.U·r²·dth (respectively c.V·r²·dth) term.
// Forward walks th0→th1 through (Start, End) in the sweep direction; reverse
// walks the same arc the other way, so the two endpoints swap which
// "th0"/"th1" role they play and the signed sweep negates.
//
// The denoted arc runs on Start's radius r and ends at End's ANGLE, so End's
// r·sinθ1, r·cosθ1 are ρ·(End − Center) with ρ = r/|End − Center|, not the
// recorded difference itself. The arm reads ρ through arcEndRadialRatio — a
// proven bracket of the exact ratio — and substitutes ρ·(End − Center) for
// End's coordinate differences before the reverse swap, evaluating the same
// polynomial in interval arithmetic. Equal pinned radii give the point ρ = 1,
// so every term is a rational point and the expression is one rational point
// plus one scaled interval; unequal radii carry ρ's width into the enclosure.
func circularFirstMomentInterval(seg CurveSegment, anchor Point2) (ratInterval, ratInterval, bool) {
	anchorU, anchorV := proofarith.FloatRat(anchor.U), proofarith.FloatRat(anchor.V)
	if anchorU == nil || anchorV == nil {
		return ratInterval{}, ratInterval{}, false
	}
	switch seg := seg.(type) {
	case CircleSeg:
		dt := exactCoordinateDelta(seg.TEnd, seg.TStart)
		radius, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return ratInterval{}, ratInterval{}, false
		}
		r := proofarith.FloatRat(radius)
		if r == nil {
			return ratInterval{}, ratInterval{}, false
		}
		centerU := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.U), anchorU)
		centerV := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.V), anchorV)
		piIv := interval(piLower, piUpper)
		if dt.IsInt() {
			r2dt := ratMul(r, r, dt)
			return intervalScale(piIv, ratMul(centerU, r2dt)), intervalScale(piIv, ratMul(centerV, r2dt)), true
		}
		t0, t1 := proofarith.FloatRat(seg.TStart), proofarith.FloatRat(seg.TEnd)
		if t0 == nil || t1 == nil {
			return ratInterval{}, ratInterval{}, false
		}
		s0, c0 := turnSinCosInterval(t0)
		s1, c1 := turnSinCosInterval(t1)
		dtheta := intervalScale(piIv, new(big.Rat).Mul(big.NewRat(2, 1), dt))
		sin2_0 := intervalScale(intervalMul(s0, c0), big.NewRat(2, 1))
		sin2_1 := intervalScale(intervalMul(s1, c1), big.NewRat(2, 1))
		cube := func(x ratInterval) ratInterval { return intervalMul(intervalMul(x, x), x) }

		intCos := intervalSub(s1, s0)
		intCos2 := intervalAdd(
			intervalScale(dtheta, big.NewRat(1, 2)),
			intervalScale(intervalSub(sin2_1, sin2_0), big.NewRat(1, 4)),
		)
		intCos3 := intervalSub(
			intervalSub(s1, intervalScale(cube(s1), big.NewRat(1, 3))),
			intervalSub(s0, intervalScale(cube(s0), big.NewRat(1, 3))),
		)
		cu2 := new(big.Rat).Mul(centerU, centerU)
		cur2 := ratScale(new(big.Rat).Mul(centerU, r), 2, 1)
		r2 := new(big.Rat).Mul(r, r)
		muInner := intervalAdd(
			intervalAdd(intervalScale(intCos, cu2), intervalScale(intCos2, cur2)),
			intervalScale(intCos3, r2),
		)
		mu := intervalScale(muInner, ratScale(r, 1, 2))

		intSin := intervalSub(c0, c1)
		intSin2 := intervalSub(
			intervalScale(dtheta, big.NewRat(1, 2)),
			intervalScale(intervalSub(sin2_1, sin2_0), big.NewRat(1, 4)),
		)
		intSin3 := intervalSub(
			intervalSub(c0, intervalScale(cube(c0), big.NewRat(1, 3))),
			intervalSub(c1, intervalScale(cube(c1), big.NewRat(1, 3))),
		)
		cv2 := new(big.Rat).Mul(centerV, centerV)
		cvr2 := ratScale(new(big.Rat).Mul(centerV, r), 2, 1)
		mvInner := intervalAdd(
			intervalAdd(intervalScale(intSin, cv2), intervalScale(intSin2, cvr2)),
			intervalScale(intSin3, r2),
		)
		mv := intervalScale(mvInner, ratScale(r, 1, 2))
		return mu, mv, true
	case ArcSeg:
		forward := seg.TStart == 0 && seg.TEnd == 1
		reverse := seg.TStart == 1 && seg.TEnd == 0
		if !forward && !reverse {
			return ratInterval{}, ratInterval{}, false
		}
		dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
		dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
		dx1 := exactCoordinateDelta(seg.End.U, seg.Center.U)
		dy1 := exactCoordinateDelta(seg.End.V, seg.Center.V)
		r2 := ratAdd(ratMul(dx0, dx0), ratMul(dy0, dy0))
		endR2 := ratAdd(ratMul(dx1, dx1), ratMul(dy1, dy1))
		rho, ok := arcEndRadialRatio(r2, endR2)
		if !ok {
			return ratInterval{}, ratInterval{}, false
		}
		heldCenter := shiftPoint(seg.Center, anchor)
		heldStart := shiftPoint(seg.Start, anchor)
		heldEnd := shiftPoint(seg.End, anchor)
		heldDY0 := heldStart.V - heldCenter.V
		heldDY1 := heldEnd.V - heldCenter.V
		a0 := atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
		a1 := atan2Interval(dy1, dx1, heldDY1 == 0 && math.Signbit(heldDY1))
		sweep := intervalSub(a1, a0)
		heldA0 := math.Atan2(heldStart.V-heldCenter.V, heldStart.U-heldCenter.U)
		heldA1 := math.Atan2(heldEnd.V-heldCenter.V, heldEnd.U-heldCenter.U)
		if heldA1-heldA0 <= 0 {
			sweep = intervalAdd(sweep, twoPiInterval())
		}
		// End contributes its ANGLE (the sweep above reads the recorded
		// deltas); its point on the denoted circle is ρ·(End − Center).
		s0x, s0y := pointInterval(dx0), pointInterval(dy0)
		e1x, e1y := intervalScale(rho, dx1), intervalScale(rho, dy1)
		p0x, p0y, p1x, p1y, dth := s0x, s0y, e1x, e1y, sweep
		if reverse {
			p0x, p0y, p1x, p1y = e1x, e1y, s0x, s0y
			dth = intervalNeg(sweep)
		}
		centerU := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.U), anchorU)
		centerV := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.V), anchorV)
		dy := intervalSub(p1y, p0y)
		dx := intervalSub(p1x, p0x)
		cross := intervalSub(intervalMul(p1y, p1x), intervalMul(p0y, p0x))
		cube := func(x ratInterval) ratInterval { return intervalMul(intervalMul(x, x), x) }
		third := big.NewRat(1, 3)

		// muConst = c.U²·dy + c.U·cross + r²·dy − (p1y³ − p0y³)/3
		muConst := intervalSub(
			intervalAdd(
				intervalAdd(intervalScale(dy, ratMul(centerU, centerU)), intervalScale(cross, centerU)),
				intervalScale(dy, r2),
			),
			intervalScale(intervalSub(cube(p1y), cube(p0y)), third),
		)
		muDthCoeff := ratMul(centerU, r2)
		mu := intervalScale(intervalAdd(muConst, intervalScale(dth, muDthCoeff)), big.NewRat(1, 2))

		// mvConst = −c.V²·dx − c.V·cross − r²·dx + (p1x³ − p0x³)/3
		mvConst := intervalAdd(
			intervalNeg(intervalAdd(
				intervalAdd(intervalScale(dx, ratMul(centerV, centerV)), intervalScale(cross, centerV)),
				intervalScale(dx, r2),
			)),
			intervalScale(intervalSub(cube(p1x), cube(p0x)), third),
		)
		mvDthCoeff := ratMul(centerV, r2)
		mv := intervalScale(intervalAdd(mvConst, intervalScale(dth, mvDthCoeff)), big.NewRat(1, 2))

		return mu, mv, true
	default:
		return ratInterval{}, ratInterval{}, false
	}
}

// circularSecondMomentInterval brackets one circular walk's exact
// second-moment contributions (∫u² dA, ∫u·v dA, ∫v² dA) about the walk
// anchor, admitted under exactly the same conditions as its first-moment
// sibling circularFirstMomentInterval and restating addCircular's own
// muu/muv/mvv closed forms one order higher still: a CircleSeg over any
// recorded range, whole or fractional, and an ArcSeg only over its own full
// recorded range (forward or reverse), never a trimmed fragment.
//
// A CircleSeg's whole turns restate Pappus's own parallel-axis form: every
// odd trig moment over a whole period cancels exactly, leaving
// muu = π·dt·(c.U²·r² + r⁴/4), mvv = π·dt·(c.V²·r² + r⁴/4), muv = π·dt·c.U·c.V·r²
// (dt the signed turn count) — the disc's own second moments about its
// centre, shifted by the parallel-axis theorem. A fractional turn instead
// restates addCircular's own muu/muv/mvv formulas with every sine/cosine
// factor enclosed by moments_trig.go's turnSinCosInterval, and every
// higher trig multiple — sin(2θ), cos(2θ), sin(4θ) — taken as an exact
// DOUBLE-ANGLE algebraic combination of that same enclosure (sin2θ = 2·sinθ·cosθ,
// cos2θ = cos²θ−sin²θ, sin4θ = 2·sin2θ·cos2θ): no new transcendental is ever
// evaluated, only interval arithmetic over the one enclosure
// circularFirstMomentInterval already trusts.
//
// An ArcSeg's fragment goes one step further: because its two endpoints are
// RECORDED coordinates (not evaluated trig), every r·sinθ / r·cosθ that
// appears — at any power up to four — cancels back to an exact rational
// coordinate difference, the same substitution circularFirstMomentInterval's
// own ArcSeg arm makes for mu/mv. Expanding addCircular's muu/muv/mvv this
// way leaves exactly one term that does not collapse to a rational: the
// piece proportional to the swept angle dθ itself (a rational COEFFICIENT
// times the atan2Interval-bracketed sweep), mirroring mu/mv's own
// muDthCoeff/mvDthCoeff term one order higher. Every other term is built from
// the endpoints' own dx/dy differences and their squares/cubes/quads, with
// End's pair read as ρ·(End − Center) through arcEndRadialRatio exactly as
// the first-moment arm reads it: a single rational point interval when the
// pinned radii are equal (ρ = 1), and an enclosure carrying ρ's width when
// they differ.
func circularSecondMomentInterval(seg CurveSegment, anchor Point2) (ratInterval, ratInterval, ratInterval, bool) {
	anchorU, anchorV := proofarith.FloatRat(anchor.U), proofarith.FloatRat(anchor.V)
	if anchorU == nil || anchorV == nil {
		return ratInterval{}, ratInterval{}, ratInterval{}, false
	}
	switch seg := seg.(type) {
	case CircleSeg:
		dt := exactCoordinateDelta(seg.TEnd, seg.TStart)
		radius, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return ratInterval{}, ratInterval{}, ratInterval{}, false
		}
		r := proofarith.FloatRat(radius)
		if r == nil {
			return ratInterval{}, ratInterval{}, ratInterval{}, false
		}
		centerU := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.U), anchorU)
		centerV := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.V), anchorV)
		piIv := interval(piLower, piUpper)
		r2 := ratMul(r, r)
		r3 := ratMul(r2, r)
		r4 := ratMul(r2, r2)
		if dt.IsInt() {
			// A whole number of turns is the enclosed disc's own second
			// moments about its centre, shifted by the parallel-axis theorem;
			// every odd trig moment cancels exactly over a full period.
			muuVal := ratAdd(ratMul(centerU, centerU, r2, dt), ratScale(ratMul(r4, dt), 1, 4))
			mvvVal := ratAdd(ratMul(centerV, centerV, r2, dt), ratScale(ratMul(r4, dt), 1, 4))
			muvVal := ratMul(centerU, centerV, r2, dt)
			return intervalScale(piIv, muuVal), intervalScale(piIv, muvVal), intervalScale(piIv, mvvVal), true
		}
		t0, t1 := proofarith.FloatRat(seg.TStart), proofarith.FloatRat(seg.TEnd)
		if t0 == nil || t1 == nil {
			return ratInterval{}, ratInterval{}, ratInterval{}, false
		}
		s0, c0 := turnSinCosInterval(t0)
		s1, c1 := turnSinCosInterval(t1)
		dtheta := intervalScale(piIv, new(big.Rat).Mul(big.NewRat(2, 1), dt))
		two := big.NewRat(2, 1)
		sin2_0 := intervalScale(intervalMul(s0, c0), two)
		sin2_1 := intervalScale(intervalMul(s1, c1), two)
		cos2_0 := intervalSub(intervalMul(c0, c0), intervalMul(s0, s0))
		cos2_1 := intervalSub(intervalMul(c1, c1), intervalMul(s1, s1))
		sin4_0 := intervalScale(intervalMul(sin2_0, cos2_0), two)
		sin4_1 := intervalScale(intervalMul(sin2_1, cos2_1), two)
		cube := func(x ratInterval) ratInterval { return intervalMul(intervalMul(x, x), x) }
		sq := func(x ratInterval) ratInterval { return intervalMul(x, x) }

		intCos := intervalSub(s1, s0)
		intCos2 := intervalAdd(
			intervalScale(dtheta, big.NewRat(1, 2)),
			intervalScale(intervalSub(sin2_1, sin2_0), big.NewRat(1, 4)),
		)
		intCos3 := intervalSub(
			intervalSub(s1, intervalScale(cube(s1), big.NewRat(1, 3))),
			intervalSub(s0, intervalScale(cube(s0), big.NewRat(1, 3))),
		)
		intCos4 := intervalAdd(
			intervalAdd(
				intervalScale(dtheta, big.NewRat(3, 8)),
				intervalScale(intervalSub(sin2_1, sin2_0), big.NewRat(1, 4)),
			),
			intervalScale(intervalSub(sin4_1, sin4_0), big.NewRat(1, 32)),
		)

		intSin := intervalSub(c0, c1)
		intSin2 := intervalSub(
			intervalScale(dtheta, big.NewRat(1, 2)),
			intervalScale(intervalSub(sin2_1, sin2_0), big.NewRat(1, 4)),
		)
		intSin3 := intervalSub(
			intervalSub(c0, intervalScale(cube(c0), big.NewRat(1, 3))),
			intervalSub(c1, intervalScale(cube(c1), big.NewRat(1, 3))),
		)
		intSin4 := intervalAdd(
			intervalSub(
				intervalScale(dtheta, big.NewRat(3, 8)),
				intervalScale(intervalSub(sin2_1, sin2_0), big.NewRat(1, 4)),
			),
			intervalScale(intervalSub(sin4_1, sin4_0), big.NewRat(1, 32)),
		)

		intSC := intervalScale(intervalSub(intervalMul(s1, s1), intervalMul(s0, s0)), big.NewRat(1, 2))
		intSC2 := intervalScale(intervalSub(cube(c0), cube(c1)), big.NewRat(1, 3))
		intSC3 := intervalScale(intervalSub(sq(sq(c0)), sq(sq(c1))), big.NewRat(1, 4))

		cu2 := ratMul(centerU, centerU)
		cv2 := ratMul(centerV, centerV)
		cu3 := ratMul(cu2, centerU)
		cv3 := ratMul(cv2, centerV)

		muuInner := intervalAdd(
			intervalAdd(
				intervalAdd(intervalScale(intCos, cu3), intervalScale(intCos2, ratMul(cu2, r, big.NewRat(3, 1)))),
				intervalScale(intCos3, ratMul(centerU, r2, big.NewRat(3, 1))),
			),
			intervalScale(intCos4, r3),
		)
		muuVal := intervalScale(muuInner, ratScale(r, 1, 3))

		mvvInner := intervalAdd(
			intervalAdd(
				intervalAdd(intervalScale(intSin, cv3), intervalScale(intSin2, ratMul(cv2, r, big.NewRat(3, 1)))),
				intervalScale(intSin3, ratMul(centerV, r2, big.NewRat(3, 1))),
			),
			intervalScale(intSin4, r3),
		)
		mvvVal := intervalScale(mvvInner, ratScale(r, 1, 3))

		part1Inner := intervalAdd(
			intervalAdd(intervalScale(intCos, cu2), intervalScale(intCos2, ratMul(centerU, r, big.NewRat(2, 1)))),
			intervalScale(intCos3, r2),
		)
		part1 := intervalScale(part1Inner, centerV)
		part2Inner := intervalAdd(
			intervalAdd(intervalScale(intSC, cu2), intervalScale(intSC2, ratMul(centerU, r, big.NewRat(2, 1)))),
			intervalScale(intSC3, r2),
		)
		part2 := intervalScale(part2Inner, r)
		muvVal := intervalScale(intervalAdd(part1, part2), ratScale(r, 1, 2))

		return muuVal, muvVal, mvvVal, true
	case ArcSeg:
		forward := seg.TStart == 0 && seg.TEnd == 1
		reverse := seg.TStart == 1 && seg.TEnd == 0
		if !forward && !reverse {
			return ratInterval{}, ratInterval{}, ratInterval{}, false
		}
		dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
		dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
		dx1 := exactCoordinateDelta(seg.End.U, seg.Center.U)
		dy1 := exactCoordinateDelta(seg.End.V, seg.Center.V)
		r2 := ratAdd(ratMul(dx0, dx0), ratMul(dy0, dy0))
		endR2 := ratAdd(ratMul(dx1, dx1), ratMul(dy1, dy1))
		rho, ok := arcEndRadialRatio(r2, endR2)
		if !ok {
			return ratInterval{}, ratInterval{}, ratInterval{}, false
		}
		heldCenter := shiftPoint(seg.Center, anchor)
		heldStart := shiftPoint(seg.Start, anchor)
		heldEnd := shiftPoint(seg.End, anchor)
		heldDY0 := heldStart.V - heldCenter.V
		heldDY1 := heldEnd.V - heldCenter.V
		a0 := atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
		a1 := atan2Interval(dy1, dx1, heldDY1 == 0 && math.Signbit(heldDY1))
		sweep := intervalSub(a1, a0)
		heldA0 := math.Atan2(heldStart.V-heldCenter.V, heldStart.U-heldCenter.U)
		heldA1 := math.Atan2(heldEnd.V-heldCenter.V, heldEnd.U-heldCenter.U)
		if heldA1-heldA0 <= 0 {
			sweep = intervalAdd(sweep, twoPiInterval())
		}
		// End contributes its ANGLE (the sweep above reads the recorded
		// deltas); its point on the denoted circle is ρ·(End − Center).
		s0x, s0y := pointInterval(dx0), pointInterval(dy0)
		e1x, e1y := intervalScale(rho, dx1), intervalScale(rho, dy1)
		p0x, p0y, p1x, p1y, dth := s0x, s0y, e1x, e1y, sweep
		if reverse {
			p0x, p0y, p1x, p1y = e1x, e1y, s0x, s0y
			dth = intervalNeg(sweep)
		}
		centerU := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.U), anchorU)
		centerV := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.V), anchorV)

		dx := intervalSub(p1x, p0x)
		dy := intervalSub(p1y, p0y)
		cross := intervalSub(intervalMul(p1y, p1x), intervalMul(p0y, p0x))
		p0sq := intervalMul(p0x, p0x)
		p0ysq := intervalMul(p0y, p0y)
		p1sq := intervalMul(p1x, p1x)
		p1ysq := intervalMul(p1y, p1y)
		quad := intervalSub(
			intervalMul(intervalMul(p1x, p1y), intervalSub(p1sq, p1ysq)),
			intervalMul(intervalMul(p0x, p0y), intervalSub(p0sq, p0ysq)),
		)
		cube := func(x ratInterval) ratInterval { return intervalMul(intervalMul(x, x), x) }

		cu2 := ratMul(centerU, centerU)
		cv2 := ratMul(centerV, centerV)

		// muu = r/3·(c.U³·intCos + 3c.U²·r·intCos2 + 3c.U·r²·intCos3 + r³·intCos4),
		// every r·sinθ/r·cosθ power substituted by the matching endpoint
		// coordinate difference (dy, cross, quad — this file's own
		// circularFirstMomentInterval doc comment names the same collapse one
		// order down), leaving one constant enclosure plus one term scaled by
		// the enclosed sweep dth.
		muuConst := intervalAdd(
			intervalAdd(
				intervalAdd(
					intervalScale(dy, ratMul(centerU, cu2)),
					intervalScale(cross, ratScale(ratAdd(ratScale(cu2, 3, 1), r2), 1, 2)),
				),
				intervalScale(dy, ratMul(centerU, r2, big.NewRat(3, 1))),
			),
			intervalAdd(
				intervalScale(intervalSub(cube(p1y), cube(p0y)), new(big.Rat).Neg(centerU)),
				intervalScale(quad, big.NewRat(1, 8)),
			),
		)
		muuDthCoeff := ratAdd(ratScale(ratMul(cu2, r2), 1, 2), ratScale(ratMul(r2, r2), 1, 8))
		muuVal := intervalAdd(intervalScale(muuConst, big.NewRat(1, 3)), intervalScale(dth, muuDthCoeff))

		mvvConst := intervalAdd(
			intervalNeg(intervalAdd(
				intervalAdd(
					intervalScale(dx, ratMul(centerV, cv2)),
					intervalScale(cross, ratScale(ratAdd(ratScale(cv2, 3, 1), r2), 1, 2)),
				),
				intervalScale(dx, ratMul(centerV, r2, big.NewRat(3, 1))),
			)),
			intervalAdd(
				intervalScale(intervalSub(cube(p1x), cube(p0x)), centerV),
				intervalScale(quad, big.NewRat(1, 8)),
			),
		)
		mvvDthCoeff := ratAdd(ratScale(ratMul(cv2, r2), 1, 2), ratScale(ratMul(r2, r2), 1, 8))
		mvvVal := intervalAdd(intervalScale(mvvConst, big.NewRat(1, 3)), intervalScale(dth, mvvDthCoeff))

		// muv = ½r·(c.V·(c.U²intCos+2c.U·r·intCos2+r²intCos3) +
		// r·(c.U²intSC+2c.U·r·intSC2+r²intSC3)) — the same substitution,
		// collapsing to one constant enclosure except the c.U·c.V·r²·dth piece.
		muvConst := intervalAdd(
			intervalAdd(
				intervalAdd(
					intervalScale(
						intervalAdd(intervalScale(dy, cu2), intervalScale(cross, centerU)),
						ratScale(centerV, 1, 2),
					),
					intervalScale(dy, ratScale(ratMul(centerV, r2), 1, 2)),
				),
				intervalAdd(
					intervalScale(intervalSub(cube(p1y), cube(p0y)), ratScale(new(big.Rat).Neg(centerV), 1, 6)),
					intervalScale(intervalSub(p1ysq, p0ysq), ratScale(cu2, 1, 4)),
				),
			),
			intervalAdd(
				intervalScale(intervalSub(cube(p0x), cube(p1x)), ratScale(centerU, 1, 3)),
				intervalScale(intervalSub(intervalMul(p0sq, p0sq), intervalMul(p1sq, p1sq)), big.NewRat(1, 8)),
			),
		)
		muvDthCoeff := ratScale(ratMul(centerU, centerV, r2), 1, 2)
		muvVal := intervalAdd(muvConst, intervalScale(dth, muvDthCoeff))

		return muuVal, muvVal, mvvVal, true
	default:
		return ratInterval{}, ratInterval{}, ratInterval{}, false
	}
}

// circularMomentWalk is one recorded circular walk stated for circularMonomials: the
// centre (cU, cV) and r² as exact rationals, the signed swept angle θ1 − θ0 as
// an enclosure, and the RADIUS-SCALED endpoint offsets (X, Y) = r·(cos θ, sin θ)
// at the walk's two ends. Scaling by r is what keeps an ArcSeg's endpoint
// terms rational: its Start offset is a recorded coordinate difference, so no
// sine, cosine or square-root radius ever enters them. closed reports that the
// two ends are the same point exactly — a whole number of CircleSeg turns —
// so every endpoint difference vanishes exactly rather than as the width of
// two equal enclosures subtracted.
type circularMomentWalk struct {
	cU, cV         *big.Rat
	r2             *big.Rat
	dtheta         ratInterval
	x0, y0, x1, y1 ratInterval
	closed         bool
}

// circularMomentWalkOf states seg for circularMonomials under the admission every
// circular moment enclosure in this file shares: a CircleSeg over any recorded
// range, whole or fractional, and an ArcSeg only over its own full recorded
// range (forward or reverse), never a trimmed fragment, whose actual end the
// record alone does not state. A CircleSeg's endpoints come from
// quarterTurnSinCos (exact at every quarter turn); an ArcSeg's Start is its
// recorded offset and its End is ρ·(End − Center) through arcEndRadialRatio,
// with the swept angle bracketed by atan2Interval under the same +2π branch
// correction circularWalkEnclosures applies.
func circularMomentWalkOf(seg CurveSegment) (circularMomentWalk, bool) {
	switch seg := seg.(type) {
	case CircleSeg:
		radius, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return circularMomentWalk{}, false
		}
		r := proofarith.FloatRat(radius)
		cU, cV := proofarith.FloatRat(seg.Center.U), proofarith.FloatRat(seg.Center.V)
		t0, t1 := proofarith.FloatRat(seg.TStart), proofarith.FloatRat(seg.TEnd)
		if r == nil || cU == nil || cV == nil || t0 == nil || t1 == nil {
			return circularMomentWalk{}, false
		}
		dt := new(big.Rat).Sub(t1, t0)
		s0, c0 := quarterTurnSinCos(t0)
		s1, c1 := quarterTurnSinCos(t1)
		return circularMomentWalk{
			cU: cU, cV: cV,
			r2:     ratMul(r, r),
			dtheta: intervalScale(twoPiInterval(), dt),
			x0:     intervalScale(c0, r), y0: intervalScale(s0, r),
			x1: intervalScale(c1, r), y1: intervalScale(s1, r),
			closed: dt.IsInt(),
		}, true
	case ArcSeg:
		forward := seg.TStart == 0 && seg.TEnd == 1
		reverse := seg.TStart == 1 && seg.TEnd == 0
		if !forward && !reverse {
			return circularMomentWalk{}, false
		}
		cU, cV := proofarith.FloatRat(seg.Center.U), proofarith.FloatRat(seg.Center.V)
		if cU == nil || cV == nil {
			return circularMomentWalk{}, false
		}
		dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
		dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
		dx1 := exactCoordinateDelta(seg.End.U, seg.Center.U)
		dy1 := exactCoordinateDelta(seg.End.V, seg.Center.V)
		r2 := ratAdd(ratMul(dx0, dx0), ratMul(dy0, dy0))
		rho, ok := arcEndRadialRatio(r2, ratAdd(ratMul(dx1, dx1), ratMul(dy1, dy1)))
		if !ok {
			return circularMomentWalk{}, false
		}
		heldDY0 := seg.Start.V - seg.Center.V
		heldDY1 := seg.End.V - seg.Center.V
		a0 := atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
		a1 := atan2Interval(dy1, dx1, heldDY1 == 0 && math.Signbit(heldDY1))
		sweep := intervalSub(a1, a0)
		if math.Atan2(heldDY1, seg.End.U-seg.Center.U)-math.Atan2(heldDY0, seg.Start.U-seg.Center.U) <= 0 {
			sweep = intervalAdd(sweep, twoPiInterval())
		}
		walk := circularMomentWalk{
			cU: cU, cV: cV, r2: r2, dtheta: sweep,
			x0: pointInterval(dx0), y0: pointInterval(dy0),
			x1: intervalScale(rho, dx1), y1: intervalScale(rho, dy1),
		}
		if reverse {
			walk.x0, walk.y0, walk.x1, walk.y1 = walk.x1, walk.y1, walk.x0, walk.y0
			walk.dtheta = intervalNeg(sweep)
		}
		return walk, true
	default:
		return circularMomentWalk{}, false
	}
}

// intervalPow is x^n by repeated outward multiplication; x^0 is the exact 1.
func intervalPow(x ratInterval, n int) ratInterval {
	out := pointInterval(big.NewRat(1, 1))
	for range n {
		out = intervalMul(out, x)
	}
	return out
}

// circularMonomials returns J[a][b] = ∫ X^a·Y^b dθ over the walk for every
// a + b ≤ degree, where X = r·cos θ and Y = r·sin θ. It is the trig-power
// reduction ∫cos^a·sin^b dθ scaled by r^(a+b), which is what lets every term
// stay an endpoint product of X and Y or a power of the exact r²:
//
//	J(0,0) = θ1 − θ0          J(1,0) = [Y]    J(0,1) = −[X]    J(1,1) = [Y²]/2
//	J(a,b) =  [X^(a−1)·Y^(b+1)]/(a+b) + (a−1)·r²·J(a−2, b)/(a+b)   a ≥ 2
//	J(a,b) = −[X^(a+1)·Y^(b−1)]/(a+b) + (b−1)·r²·J(a, b−2)/(a+b)   b ≥ 2
//
// with [g] = g(θ1) − g(θ0). Both reductions are the product rule on
// cos^(a∓1)·sin^(b±1) with sin² + cos² = 1, and hold for a signed sweep of
// any length. A closed walk's [g] is the exact zero.
func circularMonomials(walk circularMomentWalk, degree int) [][]ratInterval {
	endpoint := func(m, n int) ratInterval {
		if walk.closed {
			return pointInterval(new(big.Rat))
		}
		return intervalSub(
			intervalMul(intervalPow(walk.x1, m), intervalPow(walk.y1, n)),
			intervalMul(intervalPow(walk.x0, m), intervalPow(walk.y0, n)),
		)
	}
	j := make([][]ratInterval, degree+1)
	for a := range j {
		j[a] = make([]ratInterval, degree+1-a)
	}
	for total := 0; total <= degree; total++ {
		for a := 0; a <= total; a++ {
			b := total - a
			switch {
			case a == 0 && b == 0:
				j[0][0] = walk.dtheta
			case a == 1 && b == 0:
				j[1][0] = endpoint(0, 1)
			case a == 0 && b == 1:
				j[0][1] = intervalNeg(endpoint(1, 0))
			case a == 1 && b == 1:
				j[1][1] = intervalScale(endpoint(0, 2), big.NewRat(1, 2))
			case a >= 2:
				boundary := intervalScale(endpoint(a-1, b+1), big.NewRat(1, int64(total)))
				lower := intervalScale(j[a-2][b], ratScale(walk.r2, int64(a-1), int64(total)))
				j[a][b] = intervalAdd(boundary, lower)
			default:
				boundary := intervalScale(endpoint(a+1, b-1), big.NewRat(-1, int64(total)))
				lower := intervalScale(j[a][b-2], ratScale(walk.r2, int64(b-1), int64(total)))
				j[a][b] = intervalAdd(boundary, lower)
			}
		}
	}
	return j
}

// circularGreenMoment encloses one circular walk's contribution to
// ∫u^p·v^q dA through the boundary form (1/(p+1))·∮u^(p+1)·v^q dv, about the
// plane origin. With u = cU + X, v = cV + Y and dv = X dθ, the binomial
// expansion leaves only the J monomials circularMonomials enclosed:
//
//	Σ C(p+1,i)·cU^(p+1−i)·C(q,k)·cV^(q−k)·J(i+1, k) / (p+1)
func circularGreenMoment(walk circularMomentWalk, j [][]ratInterval, p, q int) ratInterval {
	sum := pointInterval(new(big.Rat))
	for i := 0; i <= p+1; i++ {
		cuPow := new(big.Rat).SetInt64(1)
		for range p + 1 - i {
			cuPow.Mul(cuPow, walk.cU)
		}
		for k := 0; k <= q; k++ {
			cvPow := new(big.Rat).SetInt64(1)
			for range q - k {
				cvPow.Mul(cvPow, walk.cV)
			}
			coefficient := ratMul(binomialRat(p+1, i), binomialRat(q, k), cuPow, cvPow)
			sum = intervalAdd(sum, intervalScale(j[i+1][k], coefficient))
		}
	}
	return intervalScale(sum, big.NewRat(1, int64(p+1)))
}

// circularThirdMomentInterval encloses one circular walk's third-order
// contributions (∫u³ dA, ∫u²v dA, ∫uv² dA, ∫v³ dA) about the plane origin,
// under circularMomentWalkOf's admission. The boundary form is the dv one for all
// four, the same form moments.go's line and spline_moments.go's span
// contributions take, so a loop mixing the three kinds sums one consistent
// Green's-theorem integral.
func circularThirdMomentInterval(seg CurveSegment) ([4]ratInterval, bool) {
	walk, ok := circularMomentWalkOf(seg)
	if !ok {
		return [4]ratInterval{}, false
	}
	j := circularMonomials(walk, 5)
	return [4]ratInterval{
		circularGreenMoment(walk, j, 3, 0),
		circularGreenMoment(walk, j, 2, 1),
		circularGreenMoment(walk, j, 1, 2),
		circularGreenMoment(walk, j, 0, 3),
	}, true
}
