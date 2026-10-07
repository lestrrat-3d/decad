package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capcontour"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// This file owns the CAP CONTOUR's displacement — the one term every cap-level
// reading of a cap-loop chamfer is bounded through (docs/modify-reach-design.md
// §8.3/§8.4).
//
// The band has two directrices and they are not the same kind of number. The
// SIDE contour is the receiver's own recorded loop, held at its own (u, v) and
// moved axially, so every coordinate of it is a float the record already
// carried. The CAP contour is not: each of its corner points comes out of
// shell_offset.go's float offset — a line/line, line/circle or circle/circle
// solve over directions normalize2 divided by a hypot — so it is a COMPUTED
// coordinate that sits some distance from the point the construction denotes.
// Nothing about that distance is recorded anywhere in the offset, and without
// it a cap-level vertex has no honest bound to publish and a cap-level edge has
// no honest length: a zero bound asserts an exactness the solve never had, and
// an infinite one bounds nothing at all.
//
// capContourDelta is that distance, proven. It is derived ONCE per band, from
// the band's own walks and joins, and every cap-level vertex, every cap-level
// edge length and the payload's own directional extent read it.
//
// The proof is an ENCLOSURE, not an error model: the same closed forms the
// float build evaluates are re-evaluated over math/big.Rat intervals, with the
// recorded coordinates taken EXACTLY and the only widening at the square
// roots, which round outward through proofbound.RatSqrtDown/proofbound.RatSqrtUp. Interval arithmetic
// is inclusion-monotonic, so the resulting box holds the exact point the same
// closed form denotes, whatever the platform's sqrt and hypot did; the
// displacement is then the box's own greatest reach from the float point the
// build actually holds. Nothing here assumes an ulp contract, and nothing here
// is a residual test that could ADMIT a point — it only ever states how far
// the held point may be from the denoted one.

// errCapContourUnbounded is the refusal for a corner whose denoted contour
// point this evaluator cannot enclose: two offset carriers whose interval
// intersection is unbounded (a near-parallel miter whose determinant straddles
// zero) or whose exact carriers do not meet at all. The requested body exists —
// the corner has a real offset — and only this evaluator cannot state where it
// is, which is docs/modify-reach-design.md §4's ErrUnsupported side of the
// existence test.
//
// The float construction refuses first in every configuration reached so far:
// intersectOffsets rejects a determinant at or below filletTol (1e-9), while
// the interval widths here are the relative rounding of unit directions, so the
// determinant interval cannot straddle zero once the float one cleared that
// floor, and a G1 join (modify §7) never reaches ivIntersect at all. It stands
// as the honest answer for a configuration that does reach it rather than as a
// case the tests can exhibit.
var errCapContourUnbounded = fmt.Errorf(`%w: this evaluator cannot prove a bound on the cap-loop chamfer's own offset contour at a corner, so no cap-level coordinate it emits there can be published with a proven displacement`, ErrUnsupported)

type ivPoint = capcontour.Point
type ivCarrier = capcontour.Carrier

func ivAxisSpread(iv proofbound.RatInterval, c float64) (*big.Rat, bool) {
	return capcontour.AxisSpread(iv, c)
}

func ivUnion(a, b ivPoint) ivPoint { return capcontour.Union(a, b) }
func intervalHull(a, b proofbound.RatInterval) proofbound.RatInterval {
	return capcontour.IntervalHull(a, b)
}
func ivOffsetFoot(vU, vV, tu, tv, d float64) (ivPoint, bool) {
	return capcontour.OffsetFoot(vU, vV, tu, tv, d)
}
func ivCarrierOf(w survey2d.SideWalk, d float64) (ivCarrier, bool) { return capcontour.CarrierOf(w, d) }
func ivExactOffsetRadius(w survey2d.SideWalk, d float64) (*big.Rat, bool) {
	return capcontour.ExactOffsetRadius(w, d)
}
func ivIntersect(a, b ivCarrier) ([]ivPoint, bool)            { return capcontour.Intersect(a, b) }
func ivNearest(cands []ivPoint, u, v float64) (ivPoint, bool) { return capcontour.Nearest(cands, u, v) }
func ivNearestTo(cands []ivPoint, corner ivPoint) (ivPoint, bool) {
	return capcontour.NearestTo(cands, corner)
}
func miterLocusSpeedUpper(prev, cur survey2d.SideWalk, t0, t1, vU, vV float64) (float64, bool) {
	return capcontour.MiterLocusSpeedUpper(prev, cur, t0, t1, vU, vV)
}

// capContourDelta is ONE chamfer band's contour displacement: a proven upper
// bound on how far any cap-level point the band emits sits from the point the
// construction denotes. Every cap-level vertex bound, every cap-level edge
// length bound and the payload's own contour-held directional extent are
// charged this one number, so no reader can be told a different story about the
// same contour.
//
// It is a MAXIMUM over the contour's own pieces rather than a per-point figure
// because the band's readings mix them: a wall's cap edge runs between two
// corner feet, a patch quad holds two, and a survey walking the band reads
// them in one sequence. Charging each of them the band's worst is honest and
// leaves no piece under-bounded.
//
// The length VALUE a cap-level edge carries must stay the actual held float,
// never a stand-in such as +Inf for "unknown": selector.go's LongerThan
// predicate compares the raw e.length field directly, with no reference to
// Exactness or lengthBound, so an edge whose length field were ever widened
// to signal uncertainty would silently match every LongerThan query instead.
// Uncertainty belongs in lengthBound alone.
func capContourDelta(walks []survey2d.SideWalk, joins []cornerJoin, d float64) (float64, error) {
	delta := 0.0
	for _, w := range walks {
		if !w.IsCircular() {
			continue
		}
		held, err := capBandRadius(w, d)
		if err != nil {
			return 0, err
		}
		exact, ok := ivExactOffsetRadius(w, d)
		if !ok {
			return 0, errCapContourUnbounded
		}
		// The emitted arc sits at the float radius about the exact centre while
		// the denoted one sits at the exact radius about it, so the radial gap
		// between them IS the displacement of every point of that arc.
		delta = math.Max(delta, proofarith.RationalFloatError(exact, held))
	}
	n := len(walks)
	for i, j := range joins {
		if n == 0 {
			break
		}
		prev, cur := walks[(i+n-1)%n], walks[i]
		if j.arc {
			a, okA := ivOffsetFoot(j.vU, j.vV, prev.TanOutU, prev.TanOutV, d)
			b, okB := ivOffsetFoot(j.vU, j.vV, cur.TanInU, cur.TanInV, d)
			if !okA || !okB {
				return 0, errCapContourUnbounded
			}
			delta = math.Max(delta, math.Max(a.Reach(j.pA.U, j.pA.V), b.Reach(j.pB.U, j.pB.V)))
			continue
		}
		if j.g1 {
			// A G1 join intersects no carriers (modify §7; modify-reach §8.4): its
			// denoted corner is v + d·n̂ for the leaving wall's exact unit normal, and
			// the enclosure is the HULL of the two shared-normal feet, so a join the
			// dead zone classified G1 with a residual turn is charged the spread
			// between the two normals it could have taken.
			a, okA := ivOffsetFoot(j.vU, j.vV, prev.TanOutU, prev.TanOutV, d)
			b, okB := ivOffsetFoot(j.vU, j.vV, cur.TanInU, cur.TanInV, d)
			if !okA || !okB {
				return 0, errCapContourUnbounded
			}
			delta = math.Max(delta, ivUnion(a, b).Reach(j.m.U, j.m.V))
			continue
		}
		ca, okA := ivCarrierOf(prev, d)
		cb, okB := ivCarrierOf(cur, d)
		if !okA || !okB {
			return 0, errCapContourUnbounded
		}
		cands, okI := ivIntersect(ca, cb)
		if !okI {
			return 0, errCapContourUnbounded
		}
		enc, okN := ivNearest(cands, j.vU, j.vV)
		if !okN {
			return 0, errCapContourUnbounded
		}
		delta = math.Max(delta, enc.Reach(j.m.U, j.m.V))
	}
	if proofbound.IsNonFinite(delta) {
		return 0, errCapContourUnbounded
	}
	return delta, nil
}

// capWholeCircleDelta is the cornerless closed circle's own contour
// displacement — the one shape with no corner join at all, whose whole contour
// is the concentric circle at the offset radius.
func capWholeCircleDelta(w survey2d.SideWalk, d float64) (float64, error) {
	held, err := capBandRadius(w, d)
	if err != nil {
		return 0, err
	}
	exact, ok := ivExactOffsetRadius(w, d)
	if !ok {
		return 0, errCapContourUnbounded
	}
	return proofarith.RationalFloatError(exact, held), nil
}

// loopContourDelta re-derives one loop's contour displacement from the loop
// record alone, for the readings that hold no built band — the payload's own
// extentAlong, which evaluates the same contour through capLoopBoundary. It
// walks and joins the loop exactly as buildCapBand does, so the two can never
// disagree about the same contour.
func loopContourDelta(ctx context.Context, loop LoopRecord, d float64) (float64, error) {
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	cl, err := oneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return 0, err
	}
	if len(cl.walks) == 1 && cl.walks[0].Closed {
		return capWholeCircleDelta(cl.walks[0], d)
	}
	joins, err := capOffsetJoins(budget, cl, d)
	if err != nil {
		return 0, err
	}
	return capContourDelta(cl.walks, joins, d)
}

// dySquaredDistance3 is the exact squared distance between two points, every
// coordinate a float64 and hence an exact dyadic, so the returned value is the
// true square of the length the float evaluation approximated.
// dySqrtIntervalError then reports what that evaluation committed. ok is false
// where a coordinate is not finite, which states no distance at all.
func dySquaredDistance3(a0, a1, a2, b0, b1, b2 float64) (proofarith.Dyadic, bool) {
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

// ratSquaredDistance3 is dySquaredDistance3 as a big.Rat, for the callers that
// go on to divide by it or compare it against a general fraction. It answers
// nil where a coordinate is not finite.
func ratSquaredDistance3(a0, a1, a2, b0, b1, b2 float64) *big.Rat {
	d, ok := dySquaredDistance3(a0, a1, a2, b0, b1, b2)
	if !ok {
		return nil
	}
	return d.Rat()
}

// straightEdgeBound is the proven bound on a straight cap-level edge's held
// length. It has three independent terms and each speaks for a different thing:
// the square root's own committed error, measured against the exact squared
// length (dySquaredDistance3) rather than against a Hypot ulp contract Go does
// not give; and one displacement per endpoint, since moving an endpoint of a
// segment by e moves its length by at most e. ok false — a squared length the
// coordinates could not state — is an underivable bound, +Inf.
//
// A non-negative held length whose exact square IS the squared length
// (dySquareEquals, an exact dyadic comparison) is the true length, so its
// square-root term is zero and the bracket is not built. A negative held
// length never takes that shortcut: its square can match while the length
// itself is off by twice its magnitude, and the bracket measures that gap.
func straightEdgeBound(held float64, squared proofarith.Dyadic, ok bool, endpointDeltas ...float64) float64 {
	if !ok {
		return math.Inf(1)
	}
	sqrtErr := 0.0
	if held < 0 || !proofarith.DySquareEquals(held, squared) {
		sqrtErr = dySqrtIntervalError(squared, held)
	}
	return proofbound.AbsSumUpper(append([]float64{sqrtErr}, endpointDeltas...)...)
}

// capEdgeLengthBound is straightEdgeBound for a straight cap-level edge between
// two contour points, each displaced by the band's own delta.
func capEdgeLengthBound(held float64, end, start Point2, delta float64) float64 {
	squared, ok := dySquaredDistance3(end.U, end.V, 0, start.U, start.V, 0)
	return straightEdgeBound(held, squared, ok, delta, delta)
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

// capApexArcBound bounds the reflex connector arc's held length d·(th0 − th1).
// The arc's centre is the ORIGINAL corner and its radius is exactly the
// setback, both recorded, so the only error is in the sweep: the exact turn
// between the two feet the build actually holds (an proofbound.Atan2Interval bracket, so
// no libm accuracy is assumed) plus the turn those feet's own displacement can
// account for.
func capApexArcBound(j cornerJoin, d, held float64, wraps int, delta float64) float64 {
	fallback := proofbound.ConservativeValueError(held, proofbound.ProductUpper(proofbound.TwoPiUpper(), math.Abs(d)))
	aU, aV := proofarith.FloatRat(j.pA.U-j.vU), proofarith.FloatRat(j.pA.V-j.vV)
	bU, bV := proofarith.FloatRat(j.pB.U-j.vU), proofarith.FloatRat(j.pB.V-j.vV)
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
		proofarith.AddRoundError(j.pA.U, -j.vU, j.pA.U-j.vU),
		proofarith.AddRoundError(j.pA.V, -j.vV, j.pA.V-j.vV),
		proofarith.AddRoundError(j.pB.U, -j.vU, j.pB.U-j.vU),
		proofarith.AddRoundError(j.pB.V, -j.vV, j.pB.V-j.vV),
	)
	turn, ok := arcSweepAllow(d, shift)
	if !ok {
		return fallback
	}
	bound := proofbound.AbsSumUpper(proofbound.IntervalFloatError(proofbound.IntervalScale(sweep, rd), held), turn)
	return math.Min(bound, fallback)
}

// capCircleLengthBound bounds a whole cap-level circle's held 2πr against the
// EXACT offset radius: π is bracketed by moments.go's own rational constants,
// so the enclosure needs no float value of π and no libm accuracy.
func capCircleLengthBound(exactRadius *big.Rat, held float64) float64 {
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
// offset denotes. It is the ONE bracket capWallArcBound (a length bound,
// scaled by the wall's own radius) and capSweepAllow (an angle bound, read
// directly) both build from, so the two readers of one wall's cap-level
// sweep are never told two different enclosures of it.
func capSweepBracket(cU, cV float64, start, end Point2, wraps int, delta float64) (proofbound.RatInterval, float64, bool) {
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

// capWallArcBound bounds a wall's own cap-level arc's held sweep
// capRadius·(capTh1 − capTh0) (signed, matching held's own sign convention —
// the caller passes capRadius*sweepSigned, never an absolute value, so the
// bracket below and held agree on which branch they are stating). The cap
// arc runs between the offset corner feet (start, end), whose angle about the
// wall's exact centre is generally DIFFERENT from the wall's own recorded
// th0/th1 wherever the corner is a genuine (non-tangent) miter
// (docs/modify-reach-design.md §8.3) — the cap directrix is TRIMMED there —
// so the sweep is bracketed straight from those feet, exactly the way
// capApexArcBound brackets a reflex corner's own connector: an proofbound.Atan2Interval
// enclosure of the two feet's own turn about the centre, so no libm accuracy
// is assumed of the sweep itself, plus wraps (capWallSweep's own unwrap count)
// to reproduce the same branch, plus the turn the two feet's own contour
// displacement can account for.
func capWallArcBound(cU, cV float64, start, end Point2, capRadius, held float64, wraps int, delta float64) float64 {
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

// capSweepAllow bounds |held − trueSweep| for a cap-level directrix's own RAW
// swept angle in radians — capSweepBracket's same enclosure, reported against
// the raw sweep rather than against radius·sweep, so patchAreaOf's Δθ factor
// (the frustum-sector area formula's own sweep) and capWallArcBound's length
// bound both read one proven enclosure of the same sweep
// (docs/modify-reach-design.md §8.4). radius is the circle the two feet lie
// on (capRadius for a regular wall's cap contour, d for a reflex corner's
// connector) and is used only to turn the feet's own contour displacement
// into the angular turn a foot at that radius can still account for:
// arcSweepAllow's own arc-LENGTH allowance (independent of which radius it is
// stated against, by its own derivation — see arcSweepAllow's doc comment)
// divided by that same radius restates it in radians, rounded down before
// dividing so the allowance can only widen, never tighten.
func capSweepAllow(cU, cV, radius float64, start, end Point2, held float64, wraps int, delta float64) float64 {
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
