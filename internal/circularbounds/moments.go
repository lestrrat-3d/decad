package circularbounds

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

type axisFrame struct {
	aU, aV, aUBound, aVBound float64
	dU, dV, dUBound, dVBound float64
}

// AxisFrame carries only the axis readings the circular moment proof uses.
type AxisFrame = axisFrame

// NewAxisFrame records the axis and each coordinate's proven error bound.
func NewAxisFrame(aU, aV, aUBound, aVBound, dU, dV, dUBound, dVBound float64) AxisFrame {
	return axisFrame{aU, aV, aUBound, aVBound, dU, dV, dUBound, dVBound}
}

func ratScale(value *big.Rat, num, den int64) *big.Rat {
	return new(big.Rat).Mul(value, big.NewRat(num, den))
}

// The interval readers integrate recorded circular segments for root region
// moments and the per-wall axial moment in revolve_build.go.
//
// Each reader returns an enclosure and a flag, and the flag is false whenever
// the segment's own record does not determine the answer exactly. A false
// flag withholds the exact term and leaves the caller to fall back on its
// float accumulation with that fallback's own bound, never a rational term
// standing in for one the record could not state.

// axisComponentInterval encloses one axis-frame scalar — the anchor's aU/aV
// or the outward normal's nU/nV (nU = −dV, nV = dU, axisMoments' own pairing,
// revolve_build.go) — as an interval centred on its held float and widened by
// its own proven bound: junctionRadiusInterval's construction
// (revolve_build.go), generalized from a junction's radial coordinate to the
// axis's own anchor and direction. A component this evaluator cannot state as
// a finite rational — a +Inf bound, the sqrt bracket a tilted axis's
// direction does not yet carry — answers ok == false, and
// circularAxisMomentInterval refuses with it.
func axisComponentInterval(value, bound float64) (proofbound.RatInterval, bool) {
	v, b := proofarith.FloatRat(value), proofarith.FloatRat(math.Abs(bound))
	if v == nil || b == nil {
		return proofbound.RatInterval{}, false
	}
	return proofbound.IntervalWiden(proofbound.PointInterval(v), b), true
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
// sin/cos come from proofbound.TurnSinCosInterval, with an exact zero-width fast path
// when the recorded range spans a whole number of turns (the sine/cosine
// difference terms above vanish exactly, leaving Pappus's own r·Δθ·ρ_centre
// form — the torus/whole-circle case); an ArcSeg's come from proofbound.RadSinCosSpan of
// the proofbound.Atan2Interval endpoint enclosure, exactly as circularEndpointInterval
// evaluates them, and — like circularAreaInterval and
// circularFirstMomentInterval — only over its own full recorded range
// (forward or reverse), never a trimmed fragment, whose actual endpoint the
// record alone does not state.
func circularAxisMomentInterval(seg CurveSegment, ax axisFrame) (proofbound.RatInterval, bool) {
	r, dtheta, ok := circularWalkEnclosures(seg)
	if !ok {
		return proofbound.RatInterval{}, false
	}
	var cU, cV *big.Rat
	var sinDiff, cosDiff proofbound.RatInterval // sin(hi)-sin(lo), cos(lo)-cos(hi)
	switch seg := seg.(type) {
	case CircleSeg:
		cU, cV = proofarith.FloatRat(seg.Center.U), proofarith.FloatRat(seg.Center.V)
		t0, t1 := proofarith.FloatRat(seg.TStart), proofarith.FloatRat(seg.TEnd)
		if cU == nil || cV == nil || t0 == nil || t1 == nil {
			return proofbound.RatInterval{}, false
		}
		dt := new(big.Rat).Sub(t1, t0)
		switch {
		case dt.IsInt():
			zero := new(big.Rat)
			sinDiff, cosDiff = proofbound.PointInterval(zero), proofbound.PointInterval(zero)
		default:
			loT, hiT := t0, t1
			if loT.Cmp(hiT) > 0 {
				loT, hiT = hiT, loT
			}
			sinLo, cosLo := proofbound.TurnSinCosInterval(loT)
			sinHi, cosHi := proofbound.TurnSinCosInterval(hiT)
			sinDiff = proofbound.IntervalSub(sinHi, sinLo)
			cosDiff = proofbound.IntervalSub(cosLo, cosHi)
		}
	case ArcSeg:
		forward := seg.TStart == 0 && seg.TEnd == 1
		reverse := seg.TStart == 1 && seg.TEnd == 0
		if !forward && !reverse {
			return proofbound.RatInterval{}, false
		}
		cU, cV = proofarith.FloatRat(seg.Center.U), proofarith.FloatRat(seg.Center.V)
		if cU == nil || cV == nil {
			return proofbound.RatInterval{}, false
		}
		dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
		dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
		heldDY0 := seg.Start.V - seg.Center.V
		a0 := proofbound.Atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
		a1 := proofbound.IntervalAdd(a0, dtheta)
		sinLo, cosLo, ok0 := proofbound.RadSinCosSpan(a0)
		sinHi, cosHi, ok1 := proofbound.RadSinCosSpan(a1)
		if !ok0 || !ok1 {
			return proofbound.RatInterval{}, false
		}
		sinDiff = proofbound.IntervalSub(sinHi, sinLo)
		cosDiff = proofbound.IntervalSub(cosLo, cosHi)
	default:
		return proofbound.RatInterval{}, false
	}

	anchorU, ok := axisComponentInterval(ax.aU, ax.aUBound)
	if !ok {
		return proofbound.RatInterval{}, false
	}
	anchorV, ok := axisComponentInterval(ax.aV, ax.aVBound)
	if !ok {
		return proofbound.RatInterval{}, false
	}
	nU, ok := axisComponentInterval(-ax.dV, ax.dVBound)
	if !ok {
		return proofbound.RatInterval{}, false
	}
	nV, ok := axisComponentInterval(ax.dU, ax.dUBound)
	if !ok {
		return proofbound.RatInterval{}, false
	}

	duU := proofbound.IntervalSub(proofbound.PointInterval(cU), anchorU)
	duV := proofbound.IntervalSub(proofbound.PointInterval(cV), anchorV)
	intU := proofbound.IntervalMul(r, proofbound.IntervalAdd(proofbound.IntervalMul(duU, dtheta), proofbound.IntervalMul(r, sinDiff)))
	intV := proofbound.IntervalMul(r, proofbound.IntervalAdd(proofbound.IntervalMul(duV, dtheta), proofbound.IntervalMul(r, cosDiff)))
	return proofbound.IntervalAdd(proofbound.IntervalMul(nU, intU), proofbound.IntervalMul(nV, intV)), true
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
// every sine/cosine factor enclosed by internal/proofbound/turn_trig.go's proofbound.TurnSinCosInterval
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
// proofbound.Atan2Interval already brackets. What is left after multiplying through is
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
func circularFirstMomentInterval(seg CurveSegment, anchor Point2) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	anchorU, anchorV := proofarith.FloatRat(anchor.U), proofarith.FloatRat(anchor.V)
	if anchorU == nil || anchorV == nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	switch seg := seg.(type) {
	case CircleSeg:
		dt := exactCoordinateDelta(seg.TEnd, seg.TStart)
		radius, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		r := proofarith.FloatRat(radius)
		if r == nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		centerU := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.U), anchorU)
		centerV := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.V), anchorV)
		piIv := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
		if dt.IsInt() {
			r2dt := proofbound.RatMul(r, r, dt)
			return proofbound.IntervalScale(piIv, proofbound.RatMul(centerU, r2dt)), proofbound.IntervalScale(piIv, proofbound.RatMul(centerV, r2dt)), true
		}
		t0, t1 := proofarith.FloatRat(seg.TStart), proofarith.FloatRat(seg.TEnd)
		if t0 == nil || t1 == nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		s0, c0 := proofbound.TurnSinCosInterval(t0)
		s1, c1 := proofbound.TurnSinCosInterval(t1)
		dtheta := proofbound.IntervalScale(piIv, new(big.Rat).Mul(big.NewRat(2, 1), dt))
		sin2_0 := proofbound.IntervalScale(proofbound.IntervalMul(s0, c0), big.NewRat(2, 1))
		sin2_1 := proofbound.IntervalScale(proofbound.IntervalMul(s1, c1), big.NewRat(2, 1))
		cube := func(x proofbound.RatInterval) proofbound.RatInterval {
			return proofbound.IntervalMul(proofbound.IntervalMul(x, x), x)
		}

		intCos := proofbound.IntervalSub(s1, s0)
		intCos2 := proofbound.IntervalAdd(
			proofbound.IntervalScale(dtheta, big.NewRat(1, 2)),
			proofbound.IntervalScale(proofbound.IntervalSub(sin2_1, sin2_0), big.NewRat(1, 4)),
		)
		intCos3 := proofbound.IntervalSub(
			proofbound.IntervalSub(s1, proofbound.IntervalScale(cube(s1), big.NewRat(1, 3))),
			proofbound.IntervalSub(s0, proofbound.IntervalScale(cube(s0), big.NewRat(1, 3))),
		)
		cu2 := new(big.Rat).Mul(centerU, centerU)
		cur2 := ratScale(new(big.Rat).Mul(centerU, r), 2, 1)
		r2 := new(big.Rat).Mul(r, r)
		muInner := proofbound.IntervalAdd(
			proofbound.IntervalAdd(proofbound.IntervalScale(intCos, cu2), proofbound.IntervalScale(intCos2, cur2)),
			proofbound.IntervalScale(intCos3, r2),
		)
		mu := proofbound.IntervalScale(muInner, ratScale(r, 1, 2))

		intSin := proofbound.IntervalSub(c0, c1)
		intSin2 := proofbound.IntervalSub(
			proofbound.IntervalScale(dtheta, big.NewRat(1, 2)),
			proofbound.IntervalScale(proofbound.IntervalSub(sin2_1, sin2_0), big.NewRat(1, 4)),
		)
		intSin3 := proofbound.IntervalSub(
			proofbound.IntervalSub(c0, proofbound.IntervalScale(cube(c0), big.NewRat(1, 3))),
			proofbound.IntervalSub(c1, proofbound.IntervalScale(cube(c1), big.NewRat(1, 3))),
		)
		cv2 := new(big.Rat).Mul(centerV, centerV)
		cvr2 := ratScale(new(big.Rat).Mul(centerV, r), 2, 1)
		mvInner := proofbound.IntervalAdd(
			proofbound.IntervalAdd(proofbound.IntervalScale(intSin, cv2), proofbound.IntervalScale(intSin2, cvr2)),
			proofbound.IntervalScale(intSin3, r2),
		)
		mv := proofbound.IntervalScale(mvInner, ratScale(r, 1, 2))
		return mu, mv, true
	case ArcSeg:
		forward := seg.TStart == 0 && seg.TEnd == 1
		reverse := seg.TStart == 1 && seg.TEnd == 0
		if !forward && !reverse {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
		dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
		dx1 := exactCoordinateDelta(seg.End.U, seg.Center.U)
		dy1 := exactCoordinateDelta(seg.End.V, seg.Center.V)
		r2 := proofbound.RatAdd(proofbound.RatMul(dx0, dx0), proofbound.RatMul(dy0, dy0))
		endR2 := proofbound.RatAdd(proofbound.RatMul(dx1, dx1), proofbound.RatMul(dy1, dy1))
		rho, ok := arcEndRadialRatio(r2, endR2)
		if !ok {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
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
		// End contributes its ANGLE (the sweep above reads the recorded
		// deltas); its point on the denoted circle is ρ·(End − Center).
		s0x, s0y := proofbound.PointInterval(dx0), proofbound.PointInterval(dy0)
		e1x, e1y := proofbound.IntervalScale(rho, dx1), proofbound.IntervalScale(rho, dy1)
		p0x, p0y, p1x, p1y, dth := s0x, s0y, e1x, e1y, sweep
		if reverse {
			p0x, p0y, p1x, p1y = e1x, e1y, s0x, s0y
			dth = proofbound.IntervalNeg(sweep)
		}
		centerU := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.U), anchorU)
		centerV := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.V), anchorV)
		dy := proofbound.IntervalSub(p1y, p0y)
		dx := proofbound.IntervalSub(p1x, p0x)
		cross := proofbound.IntervalSub(proofbound.IntervalMul(p1y, p1x), proofbound.IntervalMul(p0y, p0x))
		cube := func(x proofbound.RatInterval) proofbound.RatInterval {
			return proofbound.IntervalMul(proofbound.IntervalMul(x, x), x)
		}
		third := big.NewRat(1, 3)

		// muConst = c.U²·dy + c.U·cross + r²·dy − (p1y³ − p0y³)/3
		muConst := proofbound.IntervalSub(
			proofbound.IntervalAdd(
				proofbound.IntervalAdd(proofbound.IntervalScale(dy, proofbound.RatMul(centerU, centerU)), proofbound.IntervalScale(cross, centerU)),
				proofbound.IntervalScale(dy, r2),
			),
			proofbound.IntervalScale(proofbound.IntervalSub(cube(p1y), cube(p0y)), third),
		)
		muDthCoeff := proofbound.RatMul(centerU, r2)
		mu := proofbound.IntervalScale(proofbound.IntervalAdd(muConst, proofbound.IntervalScale(dth, muDthCoeff)), big.NewRat(1, 2))

		// mvConst = −c.V²·dx − c.V·cross − r²·dx + (p1x³ − p0x³)/3
		mvConst := proofbound.IntervalAdd(
			proofbound.IntervalNeg(proofbound.IntervalAdd(
				proofbound.IntervalAdd(proofbound.IntervalScale(dx, proofbound.RatMul(centerV, centerV)), proofbound.IntervalScale(cross, centerV)),
				proofbound.IntervalScale(dx, r2),
			)),
			proofbound.IntervalScale(proofbound.IntervalSub(cube(p1x), cube(p0x)), third),
		)
		mvDthCoeff := proofbound.RatMul(centerV, r2)
		mv := proofbound.IntervalScale(proofbound.IntervalAdd(mvConst, proofbound.IntervalScale(dth, mvDthCoeff)), big.NewRat(1, 2))

		return mu, mv, true
	default:
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
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
// factor enclosed by internal/proofbound/turn_trig.go's proofbound.TurnSinCosInterval, and every
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
// times the proofbound.Atan2Interval-bracketed sweep), mirroring mu/mv's own
// muDthCoeff/mvDthCoeff term one order higher. Every other term is built from
// the endpoints' own dx/dy differences and their squares/cubes/quads, with
// End's pair read as ρ·(End − Center) through arcEndRadialRatio exactly as
// the first-moment arm reads it: a single rational point interval when the
// pinned radii are equal (ρ = 1), and an enclosure carrying ρ's width when
// they differ.
func circularSecondMomentInterval(seg CurveSegment, anchor Point2) (proofbound.RatInterval, proofbound.RatInterval, proofbound.RatInterval, bool) {
	anchorU, anchorV := proofarith.FloatRat(anchor.U), proofarith.FloatRat(anchor.V)
	if anchorU == nil || anchorV == nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	switch seg := seg.(type) {
	case CircleSeg:
		dt := exactCoordinateDelta(seg.TEnd, seg.TStart)
		radius, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		r := proofarith.FloatRat(radius)
		if r == nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		centerU := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.U), anchorU)
		centerV := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.V), anchorV)
		piIv := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
		r2 := proofbound.RatMul(r, r)
		r3 := proofbound.RatMul(r2, r)
		r4 := proofbound.RatMul(r2, r2)
		if dt.IsInt() {
			// A whole number of turns is the enclosed disc's own second
			// moments about its centre, shifted by the parallel-axis theorem;
			// every odd trig moment cancels exactly over a full period.
			muuVal := proofbound.RatAdd(proofbound.RatMul(centerU, centerU, r2, dt), ratScale(proofbound.RatMul(r4, dt), 1, 4))
			mvvVal := proofbound.RatAdd(proofbound.RatMul(centerV, centerV, r2, dt), ratScale(proofbound.RatMul(r4, dt), 1, 4))
			muvVal := proofbound.RatMul(centerU, centerV, r2, dt)
			return proofbound.IntervalScale(piIv, muuVal), proofbound.IntervalScale(piIv, muvVal), proofbound.IntervalScale(piIv, mvvVal), true
		}
		t0, t1 := proofarith.FloatRat(seg.TStart), proofarith.FloatRat(seg.TEnd)
		if t0 == nil || t1 == nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		s0, c0 := proofbound.TurnSinCosInterval(t0)
		s1, c1 := proofbound.TurnSinCosInterval(t1)
		dtheta := proofbound.IntervalScale(piIv, new(big.Rat).Mul(big.NewRat(2, 1), dt))
		two := big.NewRat(2, 1)
		sin2_0 := proofbound.IntervalScale(proofbound.IntervalMul(s0, c0), two)
		sin2_1 := proofbound.IntervalScale(proofbound.IntervalMul(s1, c1), two)
		cos2_0 := proofbound.IntervalSub(proofbound.IntervalMul(c0, c0), proofbound.IntervalMul(s0, s0))
		cos2_1 := proofbound.IntervalSub(proofbound.IntervalMul(c1, c1), proofbound.IntervalMul(s1, s1))
		sin4_0 := proofbound.IntervalScale(proofbound.IntervalMul(sin2_0, cos2_0), two)
		sin4_1 := proofbound.IntervalScale(proofbound.IntervalMul(sin2_1, cos2_1), two)
		cube := func(x proofbound.RatInterval) proofbound.RatInterval {
			return proofbound.IntervalMul(proofbound.IntervalMul(x, x), x)
		}
		sq := func(x proofbound.RatInterval) proofbound.RatInterval { return proofbound.IntervalMul(x, x) }

		intCos := proofbound.IntervalSub(s1, s0)
		intCos2 := proofbound.IntervalAdd(
			proofbound.IntervalScale(dtheta, big.NewRat(1, 2)),
			proofbound.IntervalScale(proofbound.IntervalSub(sin2_1, sin2_0), big.NewRat(1, 4)),
		)
		intCos3 := proofbound.IntervalSub(
			proofbound.IntervalSub(s1, proofbound.IntervalScale(cube(s1), big.NewRat(1, 3))),
			proofbound.IntervalSub(s0, proofbound.IntervalScale(cube(s0), big.NewRat(1, 3))),
		)
		intCos4 := proofbound.IntervalAdd(
			proofbound.IntervalAdd(
				proofbound.IntervalScale(dtheta, big.NewRat(3, 8)),
				proofbound.IntervalScale(proofbound.IntervalSub(sin2_1, sin2_0), big.NewRat(1, 4)),
			),
			proofbound.IntervalScale(proofbound.IntervalSub(sin4_1, sin4_0), big.NewRat(1, 32)),
		)

		intSin := proofbound.IntervalSub(c0, c1)
		intSin2 := proofbound.IntervalSub(
			proofbound.IntervalScale(dtheta, big.NewRat(1, 2)),
			proofbound.IntervalScale(proofbound.IntervalSub(sin2_1, sin2_0), big.NewRat(1, 4)),
		)
		intSin3 := proofbound.IntervalSub(
			proofbound.IntervalSub(c0, proofbound.IntervalScale(cube(c0), big.NewRat(1, 3))),
			proofbound.IntervalSub(c1, proofbound.IntervalScale(cube(c1), big.NewRat(1, 3))),
		)
		intSin4 := proofbound.IntervalAdd(
			proofbound.IntervalSub(
				proofbound.IntervalScale(dtheta, big.NewRat(3, 8)),
				proofbound.IntervalScale(proofbound.IntervalSub(sin2_1, sin2_0), big.NewRat(1, 4)),
			),
			proofbound.IntervalScale(proofbound.IntervalSub(sin4_1, sin4_0), big.NewRat(1, 32)),
		)

		intSC := proofbound.IntervalScale(proofbound.IntervalSub(proofbound.IntervalMul(s1, s1), proofbound.IntervalMul(s0, s0)), big.NewRat(1, 2))
		intSC2 := proofbound.IntervalScale(proofbound.IntervalSub(cube(c0), cube(c1)), big.NewRat(1, 3))
		intSC3 := proofbound.IntervalScale(proofbound.IntervalSub(sq(sq(c0)), sq(sq(c1))), big.NewRat(1, 4))

		cu2 := proofbound.RatMul(centerU, centerU)
		cv2 := proofbound.RatMul(centerV, centerV)
		cu3 := proofbound.RatMul(cu2, centerU)
		cv3 := proofbound.RatMul(cv2, centerV)

		muuInner := proofbound.IntervalAdd(
			proofbound.IntervalAdd(
				proofbound.IntervalAdd(proofbound.IntervalScale(intCos, cu3), proofbound.IntervalScale(intCos2, proofbound.RatMul(cu2, r, big.NewRat(3, 1)))),
				proofbound.IntervalScale(intCos3, proofbound.RatMul(centerU, r2, big.NewRat(3, 1))),
			),
			proofbound.IntervalScale(intCos4, r3),
		)
		muuVal := proofbound.IntervalScale(muuInner, ratScale(r, 1, 3))

		mvvInner := proofbound.IntervalAdd(
			proofbound.IntervalAdd(
				proofbound.IntervalAdd(proofbound.IntervalScale(intSin, cv3), proofbound.IntervalScale(intSin2, proofbound.RatMul(cv2, r, big.NewRat(3, 1)))),
				proofbound.IntervalScale(intSin3, proofbound.RatMul(centerV, r2, big.NewRat(3, 1))),
			),
			proofbound.IntervalScale(intSin4, r3),
		)
		mvvVal := proofbound.IntervalScale(mvvInner, ratScale(r, 1, 3))

		part1Inner := proofbound.IntervalAdd(
			proofbound.IntervalAdd(proofbound.IntervalScale(intCos, cu2), proofbound.IntervalScale(intCos2, proofbound.RatMul(centerU, r, big.NewRat(2, 1)))),
			proofbound.IntervalScale(intCos3, r2),
		)
		part1 := proofbound.IntervalScale(part1Inner, centerV)
		part2Inner := proofbound.IntervalAdd(
			proofbound.IntervalAdd(proofbound.IntervalScale(intSC, cu2), proofbound.IntervalScale(intSC2, proofbound.RatMul(centerU, r, big.NewRat(2, 1)))),
			proofbound.IntervalScale(intSC3, r2),
		)
		part2 := proofbound.IntervalScale(part2Inner, r)
		muvVal := proofbound.IntervalScale(proofbound.IntervalAdd(part1, part2), ratScale(r, 1, 2))

		return muuVal, muvVal, mvvVal, true
	case ArcSeg:
		forward := seg.TStart == 0 && seg.TEnd == 1
		reverse := seg.TStart == 1 && seg.TEnd == 0
		if !forward && !reverse {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
		dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
		dx1 := exactCoordinateDelta(seg.End.U, seg.Center.U)
		dy1 := exactCoordinateDelta(seg.End.V, seg.Center.V)
		r2 := proofbound.RatAdd(proofbound.RatMul(dx0, dx0), proofbound.RatMul(dy0, dy0))
		endR2 := proofbound.RatAdd(proofbound.RatMul(dx1, dx1), proofbound.RatMul(dy1, dy1))
		rho, ok := arcEndRadialRatio(r2, endR2)
		if !ok {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
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
		// End contributes its ANGLE (the sweep above reads the recorded
		// deltas); its point on the denoted circle is ρ·(End − Center).
		s0x, s0y := proofbound.PointInterval(dx0), proofbound.PointInterval(dy0)
		e1x, e1y := proofbound.IntervalScale(rho, dx1), proofbound.IntervalScale(rho, dy1)
		p0x, p0y, p1x, p1y, dth := s0x, s0y, e1x, e1y, sweep
		if reverse {
			p0x, p0y, p1x, p1y = e1x, e1y, s0x, s0y
			dth = proofbound.IntervalNeg(sweep)
		}
		centerU := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.U), anchorU)
		centerV := new(big.Rat).Sub(proofarith.FloatRat(seg.Center.V), anchorV)

		dx := proofbound.IntervalSub(p1x, p0x)
		dy := proofbound.IntervalSub(p1y, p0y)
		cross := proofbound.IntervalSub(proofbound.IntervalMul(p1y, p1x), proofbound.IntervalMul(p0y, p0x))
		p0sq := proofbound.IntervalMul(p0x, p0x)
		p0ysq := proofbound.IntervalMul(p0y, p0y)
		p1sq := proofbound.IntervalMul(p1x, p1x)
		p1ysq := proofbound.IntervalMul(p1y, p1y)
		quad := proofbound.IntervalSub(
			proofbound.IntervalMul(proofbound.IntervalMul(p1x, p1y), proofbound.IntervalSub(p1sq, p1ysq)),
			proofbound.IntervalMul(proofbound.IntervalMul(p0x, p0y), proofbound.IntervalSub(p0sq, p0ysq)),
		)
		cube := func(x proofbound.RatInterval) proofbound.RatInterval {
			return proofbound.IntervalMul(proofbound.IntervalMul(x, x), x)
		}

		cu2 := proofbound.RatMul(centerU, centerU)
		cv2 := proofbound.RatMul(centerV, centerV)

		// muu = r/3·(c.U³·intCos + 3c.U²·r·intCos2 + 3c.U·r²·intCos3 + r³·intCos4),
		// every r·sinθ/r·cosθ power substituted by the matching endpoint
		// coordinate difference (dy, cross, quad — this file's own
		// circularFirstMomentInterval doc comment names the same collapse one
		// order down), leaving one constant enclosure plus one term scaled by
		// the enclosed sweep dth.
		muuConst := proofbound.IntervalAdd(
			proofbound.IntervalAdd(
				proofbound.IntervalAdd(
					proofbound.IntervalScale(dy, proofbound.RatMul(centerU, cu2)),
					proofbound.IntervalScale(cross, ratScale(proofbound.RatAdd(ratScale(cu2, 3, 1), r2), 1, 2)),
				),
				proofbound.IntervalScale(dy, proofbound.RatMul(centerU, r2, big.NewRat(3, 1))),
			),
			proofbound.IntervalAdd(
				proofbound.IntervalScale(proofbound.IntervalSub(cube(p1y), cube(p0y)), new(big.Rat).Neg(centerU)),
				proofbound.IntervalScale(quad, big.NewRat(1, 8)),
			),
		)
		muuDthCoeff := proofbound.RatAdd(ratScale(proofbound.RatMul(cu2, r2), 1, 2), ratScale(proofbound.RatMul(r2, r2), 1, 8))
		muuVal := proofbound.IntervalAdd(proofbound.IntervalScale(muuConst, big.NewRat(1, 3)), proofbound.IntervalScale(dth, muuDthCoeff))

		mvvConst := proofbound.IntervalAdd(
			proofbound.IntervalNeg(proofbound.IntervalAdd(
				proofbound.IntervalAdd(
					proofbound.IntervalScale(dx, proofbound.RatMul(centerV, cv2)),
					proofbound.IntervalScale(cross, ratScale(proofbound.RatAdd(ratScale(cv2, 3, 1), r2), 1, 2)),
				),
				proofbound.IntervalScale(dx, proofbound.RatMul(centerV, r2, big.NewRat(3, 1))),
			)),
			proofbound.IntervalAdd(
				proofbound.IntervalScale(proofbound.IntervalSub(cube(p1x), cube(p0x)), centerV),
				proofbound.IntervalScale(quad, big.NewRat(1, 8)),
			),
		)
		mvvDthCoeff := proofbound.RatAdd(ratScale(proofbound.RatMul(cv2, r2), 1, 2), ratScale(proofbound.RatMul(r2, r2), 1, 8))
		mvvVal := proofbound.IntervalAdd(proofbound.IntervalScale(mvvConst, big.NewRat(1, 3)), proofbound.IntervalScale(dth, mvvDthCoeff))

		// muv = ½r·(c.V·(c.U²intCos+2c.U·r·intCos2+r²intCos3) +
		// r·(c.U²intSC+2c.U·r·intSC2+r²intSC3)) — the same substitution,
		// collapsing to one constant enclosure except the c.U·c.V·r²·dth piece.
		muvConst := proofbound.IntervalAdd(
			proofbound.IntervalAdd(
				proofbound.IntervalAdd(
					proofbound.IntervalScale(
						proofbound.IntervalAdd(proofbound.IntervalScale(dy, cu2), proofbound.IntervalScale(cross, centerU)),
						ratScale(centerV, 1, 2),
					),
					proofbound.IntervalScale(dy, ratScale(proofbound.RatMul(centerV, r2), 1, 2)),
				),
				proofbound.IntervalAdd(
					proofbound.IntervalScale(proofbound.IntervalSub(cube(p1y), cube(p0y)), ratScale(new(big.Rat).Neg(centerV), 1, 6)),
					proofbound.IntervalScale(proofbound.IntervalSub(p1ysq, p0ysq), ratScale(cu2, 1, 4)),
				),
			),
			proofbound.IntervalAdd(
				proofbound.IntervalScale(proofbound.IntervalSub(cube(p0x), cube(p1x)), ratScale(centerU, 1, 3)),
				proofbound.IntervalScale(proofbound.IntervalSub(proofbound.IntervalMul(p0sq, p0sq), proofbound.IntervalMul(p1sq, p1sq)), big.NewRat(1, 8)),
			),
		)
		muvDthCoeff := ratScale(proofbound.RatMul(centerU, centerV, r2), 1, 2)
		muvVal := proofbound.IntervalAdd(muvConst, proofbound.IntervalScale(dth, muvDthCoeff))

		return muuVal, muvVal, mvvVal, true
	default:
		return proofbound.RatInterval{}, proofbound.RatInterval{}, proofbound.RatInterval{}, false
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
	dtheta         proofbound.RatInterval
	x0, y0, x1, y1 proofbound.RatInterval
	closed         bool
}

// circularMomentWalkOf states seg for circularMonomials under the admission every
// circular moment enclosure in this file shares: a CircleSeg over any recorded
// range, whole or fractional, and an ArcSeg only over its own full recorded
// range (forward or reverse), never a trimmed fragment, whose actual end the
// record alone does not state. A CircleSeg's endpoints come from
// quarterTurnSinCos (exact at every quarter turn); an ArcSeg's Start is its
// recorded offset and its End is ρ·(End − Center) through arcEndRadialRatio,
// with the swept angle bracketed by proofbound.Atan2Interval under the same +2π branch
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
			r2:     proofbound.RatMul(r, r),
			dtheta: proofbound.IntervalScale(proofbound.TwoPiInterval(), dt),
			x0:     proofbound.IntervalScale(c0, r), y0: proofbound.IntervalScale(s0, r),
			x1: proofbound.IntervalScale(c1, r), y1: proofbound.IntervalScale(s1, r),
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
		r2 := proofbound.RatAdd(proofbound.RatMul(dx0, dx0), proofbound.RatMul(dy0, dy0))
		rho, ok := arcEndRadialRatio(r2, proofbound.RatAdd(proofbound.RatMul(dx1, dx1), proofbound.RatMul(dy1, dy1)))
		if !ok {
			return circularMomentWalk{}, false
		}
		heldDY0 := seg.Start.V - seg.Center.V
		heldDY1 := seg.End.V - seg.Center.V
		a0 := proofbound.Atan2Interval(dy0, dx0, heldDY0 == 0 && math.Signbit(heldDY0))
		a1 := proofbound.Atan2Interval(dy1, dx1, heldDY1 == 0 && math.Signbit(heldDY1))
		sweep := proofbound.IntervalSub(a1, a0)
		if math.Atan2(heldDY1, seg.End.U-seg.Center.U)-math.Atan2(heldDY0, seg.Start.U-seg.Center.U) <= 0 {
			sweep = proofbound.IntervalAdd(sweep, proofbound.TwoPiInterval())
		}
		walk := circularMomentWalk{
			cU: cU, cV: cV, r2: r2, dtheta: sweep,
			x0: proofbound.PointInterval(dx0), y0: proofbound.PointInterval(dy0),
			x1: proofbound.IntervalScale(rho, dx1), y1: proofbound.IntervalScale(rho, dy1),
		}
		if reverse {
			walk.x0, walk.y0, walk.x1, walk.y1 = walk.x1, walk.y1, walk.x0, walk.y0
			walk.dtheta = proofbound.IntervalNeg(sweep)
		}
		return walk, true
	default:
		return circularMomentWalk{}, false
	}
}

// intervalPow is x^n by repeated outward multiplication; x^0 is the exact 1.
func intervalPow(x proofbound.RatInterval, n int) proofbound.RatInterval {
	out := proofbound.PointInterval(big.NewRat(1, 1))
	for range n {
		out = proofbound.IntervalMul(out, x)
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
func circularMonomials(walk circularMomentWalk, degree int) [][]proofbound.RatInterval {
	endpoint := func(m, n int) proofbound.RatInterval {
		if walk.closed {
			return proofbound.PointInterval(new(big.Rat))
		}
		return proofbound.IntervalSub(
			proofbound.IntervalMul(intervalPow(walk.x1, m), intervalPow(walk.y1, n)),
			proofbound.IntervalMul(intervalPow(walk.x0, m), intervalPow(walk.y0, n)),
		)
	}
	j := make([][]proofbound.RatInterval, degree+1)
	for a := range j {
		j[a] = make([]proofbound.RatInterval, degree+1-a)
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
				j[0][1] = proofbound.IntervalNeg(endpoint(1, 0))
			case a == 1 && b == 1:
				j[1][1] = proofbound.IntervalScale(endpoint(0, 2), big.NewRat(1, 2))
			case a >= 2:
				boundary := proofbound.IntervalScale(endpoint(a-1, b+1), big.NewRat(1, int64(total)))
				lower := proofbound.IntervalScale(j[a-2][b], ratScale(walk.r2, int64(a-1), int64(total)))
				j[a][b] = proofbound.IntervalAdd(boundary, lower)
			default:
				boundary := proofbound.IntervalScale(endpoint(a+1, b-1), big.NewRat(-1, int64(total)))
				lower := proofbound.IntervalScale(j[a][b-2], ratScale(walk.r2, int64(b-1), int64(total)))
				j[a][b] = proofbound.IntervalAdd(boundary, lower)
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
func circularGreenMoment(walk circularMomentWalk, j [][]proofbound.RatInterval, p, q int) proofbound.RatInterval {
	sum := proofbound.PointInterval(new(big.Rat))
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
			coefficient := proofbound.RatMul(freeform.BinomialRat(p+1, i), freeform.BinomialRat(q, k), cuPow, cvPow)
			sum = proofbound.IntervalAdd(sum, proofbound.IntervalScale(j[i+1][k], coefficient))
		}
	}
	return proofbound.IntervalScale(sum, big.NewRat(1, int64(p+1)))
}

// circularThirdMomentInterval encloses one circular walk's third-order
// contributions (∫u³ dA, ∫u²v dA, ∫uv² dA, ∫v³ dA) about the plane origin,
// under circularMomentWalkOf's admission. The boundary form is the dv one for all
// four, the same form moments.go's line and spline_moments.go's span
// contributions take, so a loop mixing the three kinds sums one consistent
// Green's-theorem integral.
func circularThirdMomentInterval(seg CurveSegment) ([4]proofbound.RatInterval, bool) {
	walk, ok := circularMomentWalkOf(seg)
	if !ok {
		return [4]proofbound.RatInterval{}, false
	}
	j := circularMonomials(walk, 5)
	return [4]proofbound.RatInterval{
		circularGreenMoment(walk, j, 3, 0),
		circularGreenMoment(walk, j, 2, 1),
		circularGreenMoment(walk, j, 1, 2),
		circularGreenMoment(walk, j, 0, 3),
	}, true
}

// AxisMomentInterval encloses a circular wall's first axial moment.
func AxisMomentInterval(seg CurveSegment, ax AxisFrame) (proofbound.RatInterval, bool) {
	return circularAxisMomentInterval(seg, ax)
}

// FirstMomentInterval encloses a circular walk's first moments.
func FirstMomentInterval(seg CurveSegment, anchor Point2) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularFirstMomentInterval(seg, anchor)
}

// SecondMomentInterval encloses a circular walk's second moments.
func SecondMomentInterval(seg CurveSegment, anchor Point2) (proofbound.RatInterval, proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularSecondMomentInterval(seg, anchor)
}

// ThirdMomentInterval encloses a circular walk's third moments.
func ThirdMomentInterval(seg CurveSegment) ([4]proofbound.RatInterval, bool) {
	return circularThirdMomentInterval(seg)
}
