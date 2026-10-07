package lengthbound

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// ApexJoin holds the recorded corner and two offset feet of a connector arc.
type ApexJoin struct {
	VU, VV float64
	PA, PB sectionrecord.Point2
}

// DySquaredDistance3 is the exact squared distance between two points, every
// coordinate a float64 and hence an exact dyadic, so the returned value is the
// true square of the length the float evaluation approximated.
// boundarywalk.DySqrtIntervalError then reports what that evaluation committed. ok is false
// where a coordinate is not finite, which states no distance at all.
func DySquaredDistance3(a0, a1, a2, b0, b1, b2 float64) (proofarith.Dyadic, bool) {
	sum := proofarith.DyZero()
	for _, pair := range [3][2]float64{{a0, b0}, {a1, b1}, {a2, b2}} {
		x, okX := proofarith.DyOf(pair[0])
		y, okY := proofarith.DyOf(pair[1])
		if !okX || !okY {
			return proofarith.Dyadic{}, false
		}
		diff := proofarith.DySubScalar(x, y)
		sum = proofarith.DyAdd(sum, proofarith.DyMul(diff, diff))
	}
	return sum, true
}

// RatSquaredDistance3 is DySquaredDistance3 as a big.Rat, for the callers that
// go on to divide by it or compare it against a general fraction. It answers
// nil where a coordinate is not finite.
func RatSquaredDistance3(a0, a1, a2, b0, b1, b2 float64) *big.Rat {
	d, ok := DySquaredDistance3(a0, a1, a2, b0, b1, b2)
	if !ok {
		return nil
	}
	return d.Rat()
}

// StraightEdgeBound is the proven bound on a straight cap-level edge's held
// length. It has three independent terms and each speaks for a different thing:
// the square root's own committed error, measured against the exact squared
// length (DySquaredDistance3) rather than against a Hypot ulp contract Go does
// not give; and one displacement per endpoint, since moving an endpoint of a
// segment by e moves its length by at most e. ok false — a squared length the
// coordinates could not state — is an underivable bound, +Inf.
//
// A non-negative held length whose exact square IS the squared length
// (dySquareEquals, an exact dyadic comparison) is the true length, so its
// square-root term is zero and the bracket is not built. A negative held
// length never takes that shortcut: its square can match while the length
// itself is off by twice its magnitude, and the bracket measures that gap.
func StraightEdgeBound(held float64, squared proofarith.Dyadic, ok bool, endpointDeltas ...float64) float64 {
	if !ok {
		return math.Inf(1)
	}
	sqrtErr := 0.0
	if held < 0 || !proofarith.DySquareEquals(held, squared) {
		sqrtErr = boundarywalk.DySqrtIntervalError(squared, held)
	}
	return proofbound.AbsSumUpper(append([]float64{sqrtErr}, endpointDeltas...)...)
}

// CapEdgeLengthBound is StraightEdgeBound for a straight cap-level edge between
// two contour points, each displaced by the band's own delta.
func CapEdgeLengthBound(held float64, end, start sectionrecord.Point2, delta float64) float64 {
	squared, ok := DySquaredDistance3(end.U, end.V, 0, start.U, start.V, 0)
	return StraightEdgeBound(held, squared, ok, delta, delta)
}

// arcSweepAllow converts a contour displacement into the arc length it can move.
// A foot displaced by delta on a circle of radius r turns through at most
// arcsin(delta/r) ≤ (π/2)·delta/r, so the arc between two such feet changes
// length by at most r·2·(π/2)·delta/r = π·delta — independent of the radius.
// A displacement at or past the radius says nothing about the turn at all, and
// the caller then owes the whole-circumference envelope instead.
func arcSweepAllow(radius, delta float64) (float64, bool) {
	if radius <= 0 || delta >= radius || proofbound.IsNonFinite(radius) || proofbound.IsNonFinite(delta) {
		return 0, false
	}
	return proofbound.ProductUpper(math.Nextafter(math.Pi, math.Inf(1)), delta), true
}

// CapApexArcBound bounds the reflex connector arc's held length d·(th0 − th1).
// The arc's centre is the ORIGINAL corner and its radius is exactly the
// setback, both recorded, so the only error is in the sweep: the exact turn
// between the two feet the build actually holds (an proofbound.Atan2Interval bracket, so
// no libm accuracy is assumed) plus the turn those feet's own displacement can
// account for.
func CapApexArcBound(j ApexJoin, d, held float64, wraps int, delta float64) float64 {
	fallback := proofbound.ConservativeValueError(held, proofbound.ProductUpper(proofbound.TwoPiUpper(), math.Abs(d)))
	aU, aV := proofarith.FloatRat(j.PA.U-j.VU), proofarith.FloatRat(j.PA.V-j.VV)
	bU, bV := proofarith.FloatRat(j.PB.U-j.VU), proofarith.FloatRat(j.PB.V-j.VV)
	rd := proofarith.FloatRat(d)
	if aU == nil || aV == nil || bU == nil || bV == nil || rd == nil {
		return fallback
	}
	sweep := proofbound.IntervalSub(proofbound.Atan2Interval(aV, aU, false), proofbound.Atan2Interval(bV, bU, false))
	if wraps != 0 {
		sweep = proofbound.IntervalAdd(sweep, proofbound.IntervalScale(
			proofbound.TwoPiInterval(),
			big.NewRat(int64(wraps), 1),
		))
	}
	// The build's own float differences round, and that rounding displaces the
	// direction the angle is read from just as the contour itself does.
	shift := proofbound.AbsSumUpper(
		delta,
		proofarith.AddRoundError(j.PA.U, -j.VU, j.PA.U-j.VU),
		proofarith.AddRoundError(j.PA.V, -j.VV, j.PA.V-j.VV),
		proofarith.AddRoundError(j.PB.U, -j.VU, j.PB.U-j.VU),
		proofarith.AddRoundError(j.PB.V, -j.VV, j.PB.V-j.VV),
	)
	turn, ok := arcSweepAllow(d, shift)
	if !ok {
		return fallback
	}
	bound := proofbound.AbsSumUpper(proofbound.IntervalFloatError(proofbound.IntervalScale(sweep, rd), held), turn)
	return math.Min(bound, fallback)
}

// CapCircleLengthBound bounds a whole cap-level circle's held 2πr against the
// EXACT offset radius: π is bracketed by proofbound's rational constants,
// so the enclosure needs no float value of π and no libm accuracy.
func CapCircleLengthBound(exactRadius *big.Rat, held float64) float64 {
	if exactRadius == nil {
		return math.Inf(1)
	}
	circumference := proofbound.IntervalScale(proofbound.TwoPiInterval(), exactRadius)
	return proofbound.IntervalFloatError(circumference, held)
}

// capSweepBracket is the proofbound.Atan2Interval enclosure of a cap-level directrix's
// swept angle — atan2(end−centre) − atan2(start−centre), unwrapped by
// wraps·2π to the same branch capWallSweep's own float computation picked —
// plus the coordinate shift (the contour's own displacement, folded in by
// the caller, plus each endpoint's own subtraction rounding) that a caller
// turns into an allowance for how far those feet may sit from the point the
// offset denotes. It is the ONE bracket CapWallArcBound (a length bound,
// scaled by the wall's own radius) and CapSweepAllow (an angle bound, read
// directly) both build from, so the two readers of one wall's cap-level
// sweep are never told two different enclosures of it.
func capSweepBracket(cU, cV float64, start, end sectionrecord.Point2, wraps int, delta float64) (proofbound.RatInterval, float64, bool) {
	aU, aV := proofarith.FloatRat(start.U-cU), proofarith.FloatRat(start.V-cV)
	bU, bV := proofarith.FloatRat(end.U-cU), proofarith.FloatRat(end.V-cV)
	if aU == nil || aV == nil || bU == nil || bV == nil {
		return proofbound.RatInterval{}, 0, false
	}
	sweep := proofbound.IntervalSub(proofbound.Atan2Interval(bV, bU, false), proofbound.Atan2Interval(aV, aU, false))
	if wraps != 0 {
		sweep = proofbound.IntervalAdd(sweep, proofbound.IntervalScale(
			proofbound.TwoPiInterval(),
			big.NewRat(int64(wraps), 1),
		))
	}
	// The build's own float differences round, and that rounding displaces the
	// direction the angle is read from just as the contour itself does.
	shift := proofbound.AbsSumUpper(
		delta,
		proofarith.AddRoundError(start.U, -cU, start.U-cU),
		proofarith.AddRoundError(start.V, -cV, start.V-cV),
		proofarith.AddRoundError(end.U, -cU, end.U-cU),
		proofarith.AddRoundError(end.V, -cV, end.V-cV),
	)
	return sweep, shift, true
}

// CapWallArcBound bounds a wall's own cap-level arc's held sweep
// capRadius·(capTh1 − capTh0) (signed, matching held's own sign convention —
// the caller passes capRadius*sweepSigned, never an absolute value, so the
// bracket below and held agree on which branch they are stating). The cap
// arc runs between the offset corner feet (start, end), whose angle about the
// wall's exact centre is generally DIFFERENT from the wall's own recorded
// th0/th1 wherever the corner is a genuine (non-tangent) miter
// (docs/modify-reach-design.md §8.3) — the cap directrix is TRIMMED there —
// so the sweep is bracketed straight from those feet, exactly the way
// CapApexArcBound brackets a reflex corner's own connector: an proofbound.Atan2Interval
// enclosure of the two feet's own turn about the centre, so no libm accuracy
// is assumed of the sweep itself, plus wraps (capWallSweep's own unwrap count)
// to reproduce the same branch, plus the turn the two feet's own contour
// displacement can account for.
func CapWallArcBound(cU, cV float64, start, end sectionrecord.Point2, capRadius, held float64, wraps int, delta float64) float64 {
	fallback := proofbound.ConservativeValueError(held, proofbound.ProductUpper(proofbound.TwoPiUpper(), math.Abs(capRadius)))
	sweep, shift, ok := capSweepBracket(cU, cV, start, end, wraps, delta)
	if !ok {
		return fallback
	}
	rd := proofarith.FloatRat(capRadius)
	if rd == nil {
		return fallback
	}
	turn, ok := arcSweepAllow(freeform.DownRound(math.Abs(capRadius)), shift)
	if !ok {
		return fallback
	}
	bound := proofbound.AbsSumUpper(proofbound.IntervalFloatError(proofbound.IntervalScale(sweep, rd), held), turn)
	return math.Min(bound, fallback)
}

// CapSweepAllow bounds |held − trueSweep| for a cap-level directrix's own RAW
// swept angle in radians — capSweepBracket's same enclosure, reported against
// the raw sweep rather than against radius·sweep, so patchAreaOf's Δθ factor
// (the frustum-sector area formula's own sweep) and CapWallArcBound's length
// bound both read one proven enclosure of the same sweep
// (docs/modify-reach-design.md §8.4). radius is the circle the two feet lie
// on (capRadius for a regular wall's cap contour, d for a reflex corner's
// connector) and is used only to turn the feet's own contour displacement
// into the angular turn a foot at that radius can still account for:
// arcSweepAllow's own arc-LENGTH allowance (independent of which radius it is
// stated against, by its own derivation — see arcSweepAllow's doc comment)
// divided by that same radius restates it in radians, rounded down before
// dividing so the allowance can only widen, never tighten.
func CapSweepAllow(cU, cV, radius float64, start, end sectionrecord.Point2, held float64, wraps int, delta float64) float64 {
	fallback := proofbound.ConservativeValueError(held, proofbound.TwoPiUpper())
	sweep, shift, ok := capSweepBracket(cU, cV, start, end, wraps, delta)
	if !ok {
		return fallback
	}
	r := freeform.DownRound(math.Abs(radius))
	lengthTurn, ok := arcSweepAllow(r, shift)
	if !ok {
		return fallback
	}
	angularTurn := proofbound.UpRound(lengthTurn / r)
	bound := proofbound.AbsSumUpper(proofbound.IntervalFloatError(sweep, held), angularTurn)
	return math.Min(bound, fallback)
}
