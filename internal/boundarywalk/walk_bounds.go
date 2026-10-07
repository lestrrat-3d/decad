package boundarywalk

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// lineWalkTangentBound is the single owner of the proven bound on a line
// walk's tangent, and arcWalkRadiusBound's twin one field over: the record
// states the segment's endpoints and its parameter range, never the tangent,
// so the walk's held tangent is the float difference u1−u0, v1−v0 of two
// endpoints the float lerp already rounded. The tangent the record DENOTES is
// the difference of the exact lerps (dyLerp), which carries no rounding at
// either step, and the bound is the wider of the two components' gaps from it,
// rounded outward. A lerp that is not representable as a rational yields +Inf
// — the underivable bound consumers refuse on.
func lineWalkTangentBound(seg LineSeg, heldU, heldV float64) float64 {
	u0, okU0 := proofarith.DyLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0, okV0 := proofarith.DyLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1, okU1 := proofarith.DyLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1, okV1 := proofarith.DyLerp(seg.Start.V, seg.End.V, seg.TEnd)
	if !okU0 || !okV0 || !okU1 || !okV1 {
		return math.Inf(1)
	}
	return math.Max(
		proofarith.DyRoundedFloatError(proofarith.DySubScalar(u1, u0), heldU),
		proofarith.DyRoundedFloatError(proofarith.DySubScalar(v1, v0), heldV),
	)
}

// lineWalkEndBound is the single owner of the proven bound on a LINE walk's
// endpoint, and lineWalkTangentBound's twin one field over: the record states
// the segment's endpoints and its parameter range, never the point at a trimmed
// parameter, so the walk's held endpoint is lerp2's float evaluation. The point
// the record DENOTES is the exact lerp (dyLerp), which carries no rounding at
// either step, and the bound is the wider of the two components' gaps from it,
// rounded outward. A natural bound needs no argument of its own: lerp2 and
// dyLerp both special-case t = 0 and t = 1 to the recorded Point2 verbatim, so
// the two agree exactly and this answers zero. A lerp that is not
// representable as a rational yields +Inf on its component — the underivable
// bound consumers refuse on.
func lineWalkEndBound(seg LineSeg, t, heldU, heldV float64) proofbound.WalkEndBound {
	out := proofbound.WalkEndBound{U: math.Inf(1), V: math.Inf(1)}
	if u, ok := proofarith.DyLerp(seg.Start.U, seg.End.U, t); ok {
		out.U = proofarith.DyRoundedFloatError(u, heldU)
	}
	if v, ok := proofarith.DyLerp(seg.Start.V, seg.End.V, t); ok {
		out.V = proofarith.DyRoundedFloatError(v, heldV)
	}
	return out
}

// circularWalkEndBound is the single owner of the proven bound on a CIRCULAR
// walk's endpoint: circularWalk reaches every endpoint through math.Sincos at
// an angle this package computed — a CircleSeg's from a float multiply by 2π,
// an ArcSeg's from math.Atan2 of the recorded differences — and neither the
// trig nor its argument is a quantity that walk can enclose from the record
// alone (circularWalk's own comment). circularEndpointInterval encloses the
// point the record DENOTES at that parameter instead, from the recorded data
// and certified trigonometry, and each component's bound is its own gap from
// that enclosure.
//
// An enclosure the recorded data cannot state yields +Inf — an underivable
// bound, which every consumer refuses on rather than publishes.
func circularWalkEndBound(seg CurveSegment, t, heldU, heldV float64) proofbound.WalkEndBound {
	rt := proofarith.FloatRat(t)
	if rt == nil {
		return proofbound.WalkEndBound{U: math.Inf(1), V: math.Inf(1)}
	}
	return circularPointBound(seg, rt, heldU, heldV)
}

// circularPointBound is circularWalkEndBound read at an EXACT RATIONAL
// parameter rather than a held float, and owns the derivation both spellings
// share. It exists for a caller that generates a point at a parameter the
// record's own arithmetic states exactly — a uniform station division
// t_k = TStart + (k/m)·(TEnd − TStart) (loft_build.go's circularStationChain)
// is the one such caller today. Rounding that parameter to a float first would
// enclose the recorded curve at a NEIGHBOURING parameter, and the bound would
// then be a proof about a point the construction never named: the cells either
// side of it would no longer divide the sweep uniformly, the division
// docs/loft-design.md §5.2's per-cell sagitta row derives that term over.
//
// An enclosure the recorded data cannot state yields +Inf on both components,
// the underivable bound every consumer refuses on.
func circularPointBound(seg CurveSegment, t *big.Rat, heldU, heldV float64) proofbound.WalkEndBound {
	uIv, vIv, ok := circularEndpointInterval(seg, t)
	if !ok {
		return proofbound.WalkEndBound{U: math.Inf(1), V: math.Inf(1)}
	}
	return proofbound.WalkEndBound{
		U: proofbound.IntervalFloatError(uIv, heldU),
		V: proofbound.IntervalFloatError(vIv, heldV),
	}
}

// arcWalkRadiusBound is the single owner of the proven bound on an ArcSeg
// walk's radius, and the reason survey2d.SegmentWalk carries radiusBound at all: the
// record states Start and Center, never the radius, so the walk's held radius
// is the float math.Hypot of their difference. The exact radius is
// √((Su−Cu)² + (Sv−Cv)²) over the recorded coordinates, which proofbound.RatSqrtDown and
// proofbound.RatSqrtUp bracket without rounding, and the bound is the wider side of that
// bracket about the held float, rounded outward. A bracket that overflows
// yields +Inf — an underivable bound, which every consumer refuses on rather
// than publishes.
func arcWalkRadiusBound(seg ArcSeg, held float64) float64 {
	dx := exactCoordinateDelta(seg.Start.U, seg.Center.U)
	dy := exactCoordinateDelta(seg.Start.V, seg.Center.V)
	r2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
	rLo, rHi := proofbound.RatSqrtDown(r2), proofbound.RatSqrtUp(r2)
	if proofbound.IsNonFinite(rLo) || proofbound.IsNonFinite(rHi) {
		return math.Inf(1)
	}
	return arcRadiusBoundFromBracket(held, rLo, rHi)
}

// arcRadiusBoundFromBracket is arcWalkRadiusBound's formula over an already
// built radius bracket [rLo, rHi]: the wider side of the bracket about the
// held radius, rounded outward. It exists so walkOf, which reads the same
// bracket out of circularWalkEnclosures, states the formula through its one
// owner instead of copying it.
func arcRadiusBoundFromBracket(held, rLo, rHi float64) float64 {
	return math.Max(proofbound.UpRound(held-rLo), proofbound.UpRound(rHi-held))
}

// lineWalkBounds compares the held square root with the segment's exact
// squared length, a polynomial in the recorded floats and hence a dyadic
// (dyLerp). A Pythagorean or axis-aligned length that lands exactly keeps a
// zero bound; every other square root uses the exact L1 length as a finite
// magnitude envelope, without assuming a Hypot ulp guarantee. It also returns
// an L1 coordinate envelope for later revolution bounds.
func lineWalkBounds(seg LineSeg, held float64) (float64, float64, float64) {
	u0, okU0 := proofarith.DyLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0, okV0 := proofarith.DyLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1, okU1 := proofarith.DyLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1, okV1 := proofarith.DyLerp(seg.Start.V, seg.End.V, seg.TEnd)
	if !okU0 || !okV0 || !okU1 || !okV1 {
		return math.Inf(1), math.Inf(1), math.Inf(1)
	}
	du := proofarith.DySubScalar(u1, u0)
	dv := proofarith.DySubScalar(v1, v0)
	lengthSquared := proofarith.DyAdd(proofarith.DyMul(du, du), proofarith.DyMul(dv, dv))
	coordUpper := math.Max(proofarith.DyL1Upper(u0, v0), proofarith.DyL1Upper(u1, v1))
	if proofarith.DySquareEquals(held, lengthSquared) {
		return 0, held, coordUpper
	}
	upper := proofarith.DyL1Upper(du, dv)
	bound := math.Min(proofbound.ConservativeValueError(held, upper), dySqrtIntervalError(lengthSquared, held))
	return bound, upper, coordUpper
}

// dySqrtIntervalError proves |held − sqrt(lengthSquared)| from the
// directed-rounding square root bracket (dyadic.go's dySqrtDown/dySqrtUp),
// assuming no ulp contract from Hypot or Sqrt. The answer is the farther of the
// held float's two gaps from the bracket's ends, each rounded outward through
// dyRoundedFloatError — proofbound.IntervalFloatError's rule over this arithmetic. It
// returns +Inf when the bracket cannot be built (an end past MaxFloat64), so a
// math.Min against it can only ever keep the caller's own bound.
func dySqrtIntervalError(lengthSquared proofarith.Dyadic, held float64) float64 {
	lo, okLo := proofarith.DyOf(proofarith.DySqrtDown(lengthSquared))
	hi, okHi := proofarith.DyOf(proofarith.DySqrtUp(lengthSquared))
	if !okLo || !okHi {
		return math.Inf(1)
	}
	return math.Max(proofarith.DyRoundedFloatError(lo, held), proofarith.DyRoundedFloatError(hi, held))
}

func ratL1Upper(values ...*big.Rat) float64 {
	total := new(big.Rat)
	for _, value := range values {
		total.Add(total, new(big.Rat).Abs(value))
	}
	upper, exact := total.Float64()
	if !exact {
		upper = math.Nextafter(upper, math.Inf(1))
	}
	return upper
}
