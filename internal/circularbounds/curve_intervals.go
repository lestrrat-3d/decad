package circularbounds

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// Point2 is a plane-local coordinate in millimetres.
type Point2 struct{ U, V float64 }

// CurveSegment is the circular subset of a recorded section's segments.
type CurveSegment interface{ circularSegment() }

// CircleSeg records a circular walk without deriving its endpoints.
type CircleSeg struct {
	Center       Point2
	Radius       units.Value
	CCW          bool
	TStart, TEnd float64
}

func (CircleSeg) circularSegment() {}

// ArcSeg records an arc's pinned points and its walked range.
type ArcSeg struct {
	Center, Start, End Point2
	TStart, TEnd       float64
}

func (ArcSeg) circularSegment() {}

func shiftPoint(point, anchor Point2) Point2 {
	return Point2{U: point.U - anchor.U, V: point.V - anchor.V}
}

func exactCoordinateDelta(a, b float64) *big.Rat {
	return new(big.Rat).Sub(proofarith.FloatRat(a), proofarith.FloatRat(b))
}

// arcEndRadialRatio brackets ρ = |Start − Center| / |End − Center| for a recorded
// ArcSeg, given the two exact squared distances. The denoted arc runs on Start's
// radius and ends at End's ANGLE, so the point the arc actually ends at is
// Center + ρ·(End − Center); every circular bracket substitutes that for the
// recorded End. ρ is the proofbound.RatSqrtDown/proofbound.RatSqrtUp bracket of the exact rational
// r²/endR², rounded outward once at each end and never a float sqrt of a float.
// Equal squared radii answer the exact point 1 — the record states an exact
// circle and the substitution is the identity. A zero endR² (End == Center)
// answers false; the preflight refuses that record before any bracket runs.
func arcEndRadialRatio(r2, endR2 *big.Rat) (proofbound.RatInterval, bool) {
	if endR2.Sign() == 0 {
		return proofbound.RatInterval{}, false
	}
	if endR2.Cmp(r2) == 0 {
		return proofbound.PointInterval(big.NewRat(1, 1)), true
	}
	q := new(big.Rat).Quo(r2, endR2)
	lo, hi := proofarith.FloatRat(proofbound.RatSqrtDown(q)), proofarith.FloatRat(proofbound.RatSqrtUp(q))
	if lo == nil || hi == nil {
		return proofbound.RatInterval{}, false
	}
	return proofbound.Interval(lo, hi), true
}

// arcAngleSweep encloses a recorded ArcSeg's Start angle a0 and its natural
// sweep, the counter-clockwise angle from Start to End, from the exact
// coordinate differences (dx0, dy0) = Start − Center and (dx1, dy1) =
// End − Center. Each angle is proofbound.Atan2Interval's enclosure, and the
// sweep takes +2π where the caller's own float angles put End at or before
// Start (heldDX0..heldDY1 are those float differences), so the bracket
// encloses the same branch the float evaluation walks.
func arcAngleSweep(dx0, dy0, dx1, dy1 *big.Rat, heldDX0, heldDY0, heldDX1, heldDY1 float64) (proofbound.RatInterval, proofbound.RatInterval) {
	a0 := proofbound.Atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
	a1 := proofbound.Atan2Interval(dy1, dx1, heldDY1 == 0 && math.Signbit(heldDY1))
	sweep := proofbound.IntervalSub(a1, a0)
	if math.Atan2(heldDY1, heldDX1)-math.Atan2(heldDY0, heldDX0) <= 0 {
		sweep = proofbound.IntervalAdd(sweep, proofbound.TwoPiInterval())
	}
	return a0, sweep
}

// arcDeltas holds a recorded ArcSeg's exact offsets from its Center, Start's
// exact squared radius r2, and rho, arcEndRadialRatio's bracket of
// |Start − Center| / |End − Center|.
type arcDeltas struct {
	dx0, dy0, dx1, dy1, r2 *big.Rat
	rho                    proofbound.RatInterval
}

// arcDeltasOf states seg's arcDeltas. It answers false where End sits on
// Center, which has no radial ratio.
func arcDeltasOf(seg ArcSeg) (arcDeltas, bool) {
	d := arcDeltas{
		dx0: exactCoordinateDelta(seg.Start.U, seg.Center.U),
		dy0: exactCoordinateDelta(seg.Start.V, seg.Center.V),
		dx1: exactCoordinateDelta(seg.End.U, seg.Center.U),
		dy1: exactCoordinateDelta(seg.End.V, seg.Center.V),
	}
	d.r2 = proofbound.RatAdd(proofbound.RatMul(d.dx0, d.dx0), proofbound.RatMul(d.dy0, d.dy0))
	rho, ok := arcEndRadialRatio(d.r2, proofbound.RatAdd(proofbound.RatMul(d.dx1, d.dx1), proofbound.RatMul(d.dy1, d.dy1)))
	if !ok {
		return arcDeltas{}, false
	}
	d.rho = rho
	return d, true
}

// arcOffsets is a recorded ArcSeg's walk over its own recorded range, stated
// for the moment brackets: the radius-scaled offsets (x, y) = r·(cos θ, sin θ)
// from Center at the walk's start θ(TStart) and end θ(TEnd), and the signed
// swept angle θ(TEnd) − θ(TStart).
type arcOffsets struct {
	x0, y0, x1, y1, dtheta proofbound.RatInterval
}

// arcWalkOffsets encloses seg's walk over [TStart, TEnd], in the record's
// order, so a reversed record walks backwards. The arc denotes
// θ(t) = a0 + t·sweep on Start's radius r (docs/evaluator-design.md §4), so
// the signed swept angle is the exact rational (TEnd − TStart) times the
// sweep enclosure. Each end's offset comes from the record wherever the
// record states it: Start's own difference at t == 0, and ρ·(End − Center)
// at t == 1, the point on Start's circle at End's angle. A cut end at any
// other t states no point, so its offset is r·(cos θ(t), sin θ(t)), with r
// the proofbound.RatSqrtDown/proofbound.RatSqrtUp bracket of the exact r² and
// the sine and cosine proofbound.RadSinCosSpan's enclosure over the enclosed
// angle. A whole arc therefore reads exactly the intervals its forward
// (0 → 1) or reverse (1 → 0) walk always has.
func arcWalkOffsets(seg ArcSeg, d arcDeltas, a0, sweep proofbound.RatInterval) (arcOffsets, bool) {
	t0, t1 := proofarith.FloatRat(seg.TStart), proofarith.FloatRat(seg.TEnd)
	if t0 == nil || t1 == nil {
		return arcOffsets{}, false
	}
	x0, y0, ok := arcOffsetAt(seg.TStart, t0, d, a0, sweep)
	if !ok {
		return arcOffsets{}, false
	}
	x1, y1, ok := arcOffsetAt(seg.TEnd, t1, d, a0, sweep)
	if !ok {
		return arcOffsets{}, false
	}
	return arcOffsets{
		x0: x0, y0: y0, x1: x1, y1: y1,
		dtheta: proofbound.IntervalScale(sweep, new(big.Rat).Sub(t1, t0)),
	}, true
}

// arcOffsetAt encloses one arc walk end's radius-scaled offset from Center
// at parameter t (rt is t as an exact rational); see arcWalkOffsets.
func arcOffsetAt(t float64, rt *big.Rat, d arcDeltas, a0, sweep proofbound.RatInterval) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	switch t {
	case 0:
		return proofbound.PointInterval(d.dx0), proofbound.PointInterval(d.dy0), true
	case 1:
		return proofbound.IntervalScale(d.rho, d.dx1), proofbound.IntervalScale(d.rho, d.dy1), true
	}
	sin, cos, ok := proofbound.RadSinCosSpan(proofbound.IntervalAdd(a0, proofbound.IntervalScale(sweep, rt)))
	if !ok {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	rLo, rHi := proofarith.FloatRat(proofbound.RatSqrtDown(d.r2)), proofarith.FloatRat(proofbound.RatSqrtUp(d.r2))
	if rLo == nil || rHi == nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	r := proofbound.Interval(rLo, rHi)
	return proofbound.IntervalMul(r, cos), proofbound.IntervalMul(r, sin), true
}

// isWholeArc reports whether seg walks its full recorded range, forward or
// reverse.
func isWholeArc(seg ArcSeg) bool {
	return (seg.TStart == 0 && seg.TEnd == 1) || (seg.TStart == 1 && seg.TEnd == 0)
}

// circularAreaInterval brackets one circular walk's exact area contribution
// about the walk anchor. The segment holds the RECORDED coordinates and the
// anchor is subtracted here over rationals: every radial term is a difference
// the shift cancels out of exactly, and only the centre term carries it, so
// this bracket stays a proof about the recorded arc rather than about a
// float-shifted copy of it.
//
// A CircleSeg's fractional-turn arm (internal/proofbound/turn_trig.go's proofbound.TurnSinCosInterval)
// covers a trimmed fragment the same way the whole-turn fast path covers a
// full sweep: every non-trig factor (the radius, the recentred centre
// coordinates, the swept angle) is an exact rational, and only the endpoint
// sine/cosine terms are enclosed, exactly the substitution the ArcSeg arms
// below make for their own endpoints — differing only in where those
// sine/cosine values come from (an exact ratio at a whole arc's ends, a
// certified bracket here, because a CircleSeg's endpoints are not recorded
// coordinates). An ArcSeg over a narrowed range takes
// narrowedArcAreaInterval, whose cut ends are certified brackets too.
func circularAreaInterval(seg CurveSegment, anchor Point2) (proofbound.RatInterval, bool) {
	anchorU, anchorV := proofarith.FloatRat(anchor.U), proofarith.FloatRat(anchor.V)
	if anchorU == nil || anchorV == nil {
		return proofbound.RatInterval{}, false
	}
	switch seg := seg.(type) {
	case CircleSeg:
		dt := exactCoordinateDelta(seg.TEnd, seg.TStart)
		radius, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return proofbound.RatInterval{}, false
		}
		r := proofarith.FloatRat(radius)
		if r == nil {
			return proofbound.RatInterval{}, false
		}
		if dt.IsInt() {
			// An integer number of turns has equal endpoint sine/cosine terms,
			// leaving exactly dt·π·r².
			scale := new(big.Rat).Mul(dt, new(big.Rat).Mul(r, r))
			return proofbound.IntervalScale(proofbound.Interval(proofbound.PiLower, proofbound.PiUpper), scale), true
		}
		t0, t1 := proofarith.FloatRat(seg.TStart), proofarith.FloatRat(seg.TEnd)
		if t0 == nil || t1 == nil {
			return proofbound.RatInterval{}, false
		}
		s0, c0 := proofbound.TurnSinCosInterval(t0)
		s1, c1 := proofbound.TurnSinCosInterval(t1)
		centerU := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.U), anchorU)
		centerV := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.V), anchorV)
		piIv := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
		dtheta := proofbound.IntervalScale(piIv, new(big.Rat).Mul(big.NewRat(2, 1), dt))
		sector := proofbound.IntervalScale(dtheta, new(big.Rat).Mul(r, r))
		uTerm := proofbound.IntervalScale(proofbound.IntervalSub(s1, s0), new(big.Rat).Mul(centerU, r))
		vTerm := proofbound.IntervalScale(proofbound.IntervalSub(c1, c0), new(big.Rat).Mul(centerV, r))
		// A = ½ ( r²·dθ + c_u·r·(sin θ1 − sin θ0) − c_v·r·(cos θ1 − cos θ0) ) —
		// addCircular's own closed form (moments.go:1505), every non-trig
		// factor exact and every trig factor enclosed.
		return proofbound.IntervalScale(proofbound.IntervalSub(proofbound.IntervalAdd(sector, uTerm), vTerm), big.NewRat(1, 2)), true
	case ArcSeg:
		if !isWholeArc(seg) {
			return narrowedArcAreaInterval(seg, anchor)
		}
		reverse := seg.TStart == 1
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
		a0 := proofbound.Atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
		a1 := proofbound.Atan2Interval(dy1, dx1, heldDY1 == 0 && math.Signbit(heldDY1))
		sweep := proofbound.IntervalSub(a1, a0)
		heldA0 := math.Atan2(heldStart.V-heldCenter.V, heldStart.U-heldCenter.U)
		heldA1 := math.Atan2(heldEnd.V-heldCenter.V, heldEnd.U-heldCenter.U)
		if heldA1-heldA0 <= 0 {
			sweep = proofbound.IntervalAdd(sweep, proofbound.TwoPiInterval())
		}
		sign := big.NewRat(1, 1)
		dx, dy := new(big.Rat).Sub(dx1, dx0), new(big.Rat).Sub(dy1, dy0)
		if reverse {
			sign.Neg(sign)
			dx.Neg(dx)
			dy.Neg(dy)
		}
		sector := proofbound.IntervalScale(sweep, new(big.Rat).Mul(sign, r2))
		// Only the centre carries the anchor: every radial term above is a
		// difference the shift cancels out of.
		centerU := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.U), anchorU)
		centerV := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.V), anchorV)
		centerTerm := new(big.Rat).Sub(
			new(big.Rat).Mul(centerU, dy),
			new(big.Rat).Mul(centerV, dx),
		)
		areaProof := proofbound.IntervalAdd(sector, proofbound.PointInterval(centerTerm))
		if endR2.Cmp(r2) != 0 {
			if endR2.Sign() == 0 {
				return proofbound.RatInterval{}, false
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
			correctionFloat = proofbound.AbsSumUpper(
				correctionFloat,
				proofbound.AnalyticRoundBound(proofbound.AbsSumUpper(
					heldCenter.U,
					heldCenter.V,
					heldEnd.U-heldCenter.U,
					heldEnd.V-heldCenter.V,
				)),
			)
			correction = proofarith.FloatRat(correctionFloat)
			areaProof = proofbound.IntervalAdd(
				areaProof,
				proofbound.IntervalScale(proofbound.Interval(big.NewRat(-1, 1), big.NewRat(1, 1)), correction),
			)
		}
		return proofbound.IntervalScale(
			areaProof,
			big.NewRat(1, 2),
		), true
	default:
		return proofbound.RatInterval{}, false
	}
}

// narrowedArcAreaInterval is circularAreaInterval's arm for an ArcSeg
// recorded over a narrowed range. With u = cU + X, v = cV + Y about the
// recentred centre (cU, cV), dv = X dθ and du = −Y dθ, so the Green's-theorem
// area ½∮(u dv − v du) of the walk is
//
//	A = ½·(r²·Δθ + cU·(Y1 − Y0) − cV·(X1 − X0))
//
// with r² exact, Δθ the signed swept angle, and (X, Y) each end's
// radius-scaled offset, all from arcWalkOffsets. The held branch test reads
// the anchor-shifted coordinates, as the whole-arc arm does.
func narrowedArcAreaInterval(seg ArcSeg, anchor Point2) (proofbound.RatInterval, bool) {
	walk, r2, ok := anchoredArcWalk(seg, anchor)
	if !ok {
		return proofbound.RatInterval{}, false
	}
	centerU := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.U), proofarith.FloatRat(anchor.U))
	centerV := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.V), proofarith.FloatRat(anchor.V))
	sector := proofbound.IntervalScale(walk.dtheta, r2)
	uTerm := proofbound.IntervalScale(proofbound.IntervalSub(walk.y1, walk.y0), centerU)
	vTerm := proofbound.IntervalScale(proofbound.IntervalSub(walk.x1, walk.x0), centerV)
	return proofbound.IntervalScale(proofbound.IntervalSub(proofbound.IntervalAdd(sector, uTerm), vTerm), big.NewRat(1, 2)), true
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
//     proofbound.RatSqrtDown/proofbound.RatSqrtUp bracket of the exact squared Start-to-Center
//     distance — the same radius circularWalk holds as a math.Hypot float — and
//     the walk's sweep is the proofbound.Atan2Interval difference of the two recorded
//     endpoint angles under the +2π branch correction circularAreaInterval
//     applies, scaled by the recorded |TEnd − TStart|, the same trimming
//     circularWalk applies to its own held a0 + t·sweep angles.
//
// Both are what a consumer needs that would otherwise read the walk's own held
// w.radius and |w.th1 − w.th0|: neither of those floats is a quantity the walk
// can enclose (circularWalk's own doc comment), so a published bound composed
// from them would be a held value wearing a proof's clothes. A record this
// bracket cannot state answers false, and the consumer refuses.
func circularWalkEnclosures(seg CurveSegment) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	switch seg := seg.(type) {
	case CircleSeg:
		radius, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		r := proofarith.FloatRat(radius)
		if r == nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		r.Abs(r)
		dt := exactCoordinateDelta(seg.TEnd, seg.TStart)
		dt.Abs(dt)
		return proofbound.PointInterval(r), proofbound.IntervalScale(proofbound.TwoPiInterval(), dt), true
	case ArcSeg:
		dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
		dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
		dx1 := exactCoordinateDelta(seg.End.U, seg.Center.U)
		dy1 := exactCoordinateDelta(seg.End.V, seg.Center.V)
		r2 := new(big.Rat).Add(new(big.Rat).Mul(dx0, dx0), new(big.Rat).Mul(dy0, dy0))
		rLo, rHi := proofarith.FloatRat(proofbound.RatSqrtDown(r2)), proofarith.FloatRat(proofbound.RatSqrtUp(r2))
		if rLo == nil || rHi == nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		heldDY0 := seg.Start.V - seg.Center.V
		heldDY1 := seg.End.V - seg.Center.V
		a0 := proofbound.Atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
		a1 := proofbound.Atan2Interval(dy1, dx1, heldDY1 == 0 && math.Signbit(heldDY1))
		sweep := proofbound.IntervalSub(a1, a0)
		heldA0 := math.Atan2(heldDY0, seg.Start.U-seg.Center.U)
		heldA1 := math.Atan2(heldDY1, seg.End.U-seg.Center.U)
		if heldA1-heldA0 <= 0 {
			sweep = proofbound.IntervalAdd(sweep, proofbound.TwoPiInterval())
		}
		dt := exactCoordinateDelta(seg.TEnd, seg.TStart)
		dt.Abs(dt)
		return proofbound.Interval(rLo, rHi), proofbound.IntervalScale(sweep, dt), true
	default:
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
}

// circularLengthInterval brackets one circular walk's exact arc length as the
// product of circularWalkEnclosures' two brackets: an arc's length IS its radius
// times its swept angle, and a length has no cross-term to bracket, so unlike
// circularAreaInterval and circularFirstMomentInterval it never needed
// internal/proofbound/turn_trig.go's endpoint sine/cosine enclosure to admit a trimmed fragment.
func circularLengthInterval(seg CurveSegment) (proofbound.RatInterval, bool) {
	r, sweep, ok := circularWalkEnclosures(seg)
	if !ok {
		return proofbound.RatInterval{}, false
	}
	return proofbound.IntervalMul(r, sweep), true
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
// centre and radius, so the turn is exactly rational and internal/proofbound/turn_trig.go's
// proofbound.TurnSinCosInterval encloses the pair with no π-comparison anywhere. A whole
// multiple of a quarter turn does not even need the series: its sine and cosine
// are 0 or ±1 exactly (quarterTurnSinCos), which is what keeps a whole circle's
// own endpoint a zero-width reading.
//
// An ArcSeg states no angle at all — three pinned points, swept
// counter-clockwise from Start to End about Center (record.go) — so its point
// at t is Center + r·(cos θ, sin θ) with r the exact Start-to-Center distance
// (proofbound.RatSqrtDown/proofbound.RatSqrtUp) and θ = a0 + t·sweep, both angles enclosed by
// proofbound.Atan2Interval under the same +2π branch correction circularLengthInterval
// applies, and the sine and cosine of that enclosed angle taken by
// proofbound.RadSinCosSpan.
//
// The parameter is taken as an EXACT RATIONAL, never a float. A caller reading
// a walk's own endpoint converts its held float parameter (floatRat) at the
// call; one generating a point at a parameter the record's own arithmetic
// states — a uniform station division t_k = TStart + (k/m)·(TEnd − TStart)
// (loft_build.go's circularStationChain) — hands that value in unrounded,
// because rounding it to a float first would enclose the curve at a
// NEIGHBOURING parameter and prove a bound about a point no construction
// named.
func circularEndpointInterval(seg CurveSegment, rt *big.Rat) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularOffsetEndpointInterval(seg, rt, new(big.Rat))
}

// circularOffsetEndpointInterval is circularEndpointInterval read on the
// CONCENTRIC circle whose radius is the segment's own plus radiusOffset (an
// exact rational; zero gives the segment itself). The offset joins the radius
// before any product, so for an ArcSeg it shifts BOTH ends of the
// proofbound.RatSqrtDown/proofbound.RatSqrtUp bracket and the held radius's own rounding stays
// enclosed rather than assumed. A nonzero offset whose resulting radius is
// not positive denotes no offset circle and answers ok == false. A zero offset
// keeps circularEndpointInterval's collapsed-radius reading unchanged.
func circularOffsetEndpointInterval(seg CurveSegment, rt, radiusOffset *big.Rat) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	switch seg := seg.(type) {
	case CircleSeg:
		radius, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		r := proofarith.FloatRat(radius)
		cu, cv := proofarith.FloatRat(seg.Center.U), proofarith.FloatRat(seg.Center.V)
		if r == nil || cu == nil || cv == nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		r.Add(r, radiusOffset)
		if r.Sign() == 0 {
			return proofbound.PointInterval(cu), proofbound.PointInterval(cv), true
		}
		if radiusOffset.Sign() != 0 && r.Sign() <= 0 {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		sin, cos := quarterTurnSinCos(rt)
		return proofbound.IntervalAdd(proofbound.PointInterval(cu), proofbound.IntervalScale(cos, r)),
			proofbound.IntervalAdd(proofbound.PointInterval(cv), proofbound.IntervalScale(sin, r)), true
	case ArcSeg:
		cu, cv := proofarith.FloatRat(seg.Center.U), proofarith.FloatRat(seg.Center.V)
		if cu == nil || cv == nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
		dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
		dx1 := exactCoordinateDelta(seg.End.U, seg.Center.U)
		dy1 := exactCoordinateDelta(seg.End.V, seg.Center.V)
		r2 := new(big.Rat).Add(new(big.Rat).Mul(dx0, dx0), new(big.Rat).Mul(dy0, dy0))
		if radiusOffset.Sign() < 0 && r2.Cmp(new(big.Rat).Mul(radiusOffset, radiusOffset)) == 0 {
			return proofbound.PointInterval(cu), proofbound.PointInterval(cv), true
		}
		rLo, rHi := proofarith.FloatRat(proofbound.RatSqrtDown(r2)), proofarith.FloatRat(proofbound.RatSqrtUp(r2))
		if rLo == nil || rHi == nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		rLo.Add(rLo, radiusOffset)
		rHi.Add(rHi, radiusOffset)
		if radiusOffset.Sign() != 0 && rLo.Sign() <= 0 {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		heldDY0 := seg.Start.V - seg.Center.V
		heldDY1 := seg.End.V - seg.Center.V
		a0 := proofbound.Atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
		a1 := proofbound.Atan2Interval(dy1, dx1, heldDY1 == 0 && math.Signbit(heldDY1))
		sweep := proofbound.IntervalSub(a1, a0)
		heldA0 := math.Atan2(heldDY0, seg.Start.U-seg.Center.U)
		heldA1 := math.Atan2(heldDY1, seg.End.U-seg.Center.U)
		if heldA1-heldA0 <= 0 {
			sweep = proofbound.IntervalAdd(sweep, proofbound.TwoPiInterval())
		}
		sin, cos, ok := proofbound.RadSinCosSpan(proofbound.IntervalAdd(a0, proofbound.IntervalScale(sweep, rt)))
		if !ok {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		r := proofbound.Interval(rLo, rHi)
		return proofbound.IntervalAdd(proofbound.PointInterval(cu), proofbound.IntervalMul(r, cos)),
			proofbound.IntervalAdd(proofbound.PointInterval(cv), proofbound.IntervalMul(r, sin)), true
	default:
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
}

// quarterTurnSinCos encloses sin(2πt) and cos(2πt) for an exact rational turn,
// answering with a POINT interval whenever 4t is an integer: the quadrant turns
// have sine and cosine 0 or ±1 exactly, which no series can improve on and a
// series would only widen. Every other turn goes to proofbound.TurnSinCosInterval.
func quarterTurnSinCos(t *big.Rat) (proofbound.RatInterval, proofbound.RatInterval) {
	quadrants := new(big.Rat).Mul(t, big.NewRat(4, 1))
	if !quadrants.IsInt() {
		return proofbound.TurnSinCosInterval(t)
	}
	zero, one := new(big.Rat), big.NewRat(1, 1)
	minusOne := big.NewRat(-1, 1)
	switch new(big.Int).Mod(quadrants.Num(), big.NewInt(4)).Int64() {
	case 0:
		return proofbound.PointInterval(zero), proofbound.PointInterval(one)
	case 1:
		return proofbound.PointInterval(one), proofbound.PointInterval(zero)
	case 2:
		return proofbound.PointInterval(zero), proofbound.PointInterval(minusOne)
	default: // 3
		return proofbound.PointInterval(minusOne), proofbound.PointInterval(zero)
	}
}

// ExactCoordinateDelta preserves the rational difference of two held coordinates.
func ExactCoordinateDelta(a, b float64) *big.Rat { return exactCoordinateDelta(a, b) }

// AreaInterval encloses a circular walk's area contribution.
func AreaInterval(seg CurveSegment, anchor Point2) (proofbound.RatInterval, bool) {
	return circularAreaInterval(seg, anchor)
}

// WalkEnclosures brackets a circular walk's radius and swept angle.
func WalkEnclosures(seg CurveSegment) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularWalkEnclosures(seg)
}

// LengthInterval encloses a circular walk's arc length.
func LengthInterval(seg CurveSegment) (proofbound.RatInterval, bool) {
	return circularLengthInterval(seg)
}

// EndpointInterval encloses a circular walk's point at the given parameter.
func EndpointInterval(seg CurveSegment, rt *big.Rat) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularEndpointInterval(seg, rt)
}

// OffsetEndpointInterval encloses a radially offset circular point.
func OffsetEndpointInterval(seg CurveSegment, rt, radiusOffset *big.Rat) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularOffsetEndpointInterval(seg, rt, radiusOffset)
}

// QuarterTurnSinCos encloses sine and cosine of a quarter-turn parameter.
func QuarterTurnSinCos(t *big.Rat) (proofbound.RatInterval, proofbound.RatInterval) {
	return quarterTurnSinCos(t)
}
