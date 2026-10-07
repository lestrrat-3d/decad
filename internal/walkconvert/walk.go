package walkconvert

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularmoments"
	"github.com/lestrrat-3d/decad/internal/curveconvert"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/record"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

type (
	CurveSegment = record.CurveSegment
	LineSeg      = record.LineSeg
	CircleSeg    = record.CircleSeg
	ArcSeg       = record.ArcSeg
	Point2       = record.Point2
)

var (
	ErrDegenerate  = decaderr.ErrDegenerate
	ErrUnsupported = decaderr.ErrUnsupported
)

func normalizeSegment(segment CurveSegment) (CurveSegment, error) {
	return record.NormalizeSegment(segment)
}
func isFreeformSegment(segment CurveSegment) bool { return curveconvert.IsFreeformSegment(segment) }
func freeformBezierSpans(segment CurveSegment, work *freeform.FreeformWork) ([]survey2d.BezierSpan, bool, error) {
	return curveconvert.FreeformBezierSpans(segment, work)
}
func freeformEndpoints(spans []survey2d.BezierSpan, reversed bool) (Point2, Point2, error) {
	return curveconvert.FreeformEndpoints(spans, reversed)
}
func freeformEndpointBounds(spans []survey2d.BezierSpan, reversed bool, start, end Point2) (proofbound.WalkEndBound, proofbound.WalkEndBound) {
	return curveconvert.FreeformEndpointBounds(spans, reversed, start, end)
}
func isFitSplineSeg(segment CurveSegment) bool { return curveconvert.IsFitSplineSeg(segment) }
func circularEndpointInterval(segment CurveSegment, t *big.Rat) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularmoments.EndpointInterval(circularmoments.RecordSegment(segment), t)
}
func circularWalkEnclosures(segment CurveSegment) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularmoments.WalkEnclosures(circularmoments.RecordSegment(segment))
}
func circularLengthInterval(segment CurveSegment) (proofbound.RatInterval, bool) {
	return circularmoments.LengthInterval(circularmoments.RecordSegment(segment))
}
func exactCoordinateDelta(a, b float64) *big.Rat { return circularmoments.ExactCoordinateDelta(a, b) }
func lerp2(start, end Point2, t float64) (float64, float64) {
	switch t {
	case 0:
		return start.U, start.V
	case 1:
		return end.U, end.V
	}
	return start.U + t*(end.U-start.U), start.V + t*(end.V-start.V)
}
func arcRadiusUpper(segment ArcSeg) float64 {
	return proofbound.AbsSumUpper(segment.Start.U, segment.Center.U, segment.Start.V, segment.Center.V)
}

// WalkOf resolves a recorded segment into a bounded walk.
func WalkOf(segment CurveSegment, work *freeform.FreeformWork) (survey2d.SegmentWalk, error) {
	return walkOf(segment, work)
}
func LineWalkTangentBound(segment LineSeg, u, v float64) float64 {
	return lineWalkTangentBound(segment, u, v)
}
func LineWalkEndBound(segment LineSeg, t, u, v float64) proofbound.WalkEndBound {
	return lineWalkEndBound(segment, t, u, v)
}
func CircularWalkEndBound(segment CurveSegment, t, u, v float64) proofbound.WalkEndBound {
	return circularWalkEndBound(segment, t, u, v)
}
func CircularPointBound(segment CurveSegment, t *big.Rat, u, v float64) proofbound.WalkEndBound {
	return circularPointBound(segment, t, u, v)
}
func ArcWalkRadiusBound(segment ArcSeg, held float64) float64 {
	return arcWalkRadiusBound(segment, held)
}
func PinArcWalkEnds(walk *survey2d.SegmentWalk, segment ArcSeg) { pinArcWalkEnds(walk, segment) }
func CircularWalk(cu, cv, r, th0, th1, radiusUpper, sweepUpper float64) survey2d.SegmentWalk {
	return circularWalk(cu, cv, r, th0, th1, radiusUpper, sweepUpper)
}
func LineWalkBounds(segment LineSeg, held float64) (float64, float64, float64) {
	return lineWalkBounds(segment, held)
}
func DySqrtIntervalError(lengthSquared proofarith.Dyadic, held float64) float64 {
	return dySqrtIntervalError(lengthSquared, held)
}
func RatL1Upper(values ...*big.Rat) float64                       { return ratL1Upper(values...) }
func CoalesceWalks(walks []survey2d.SideWalk) []survey2d.SideWalk { return coalesceWalks(walks) }
func CoalesceWalksBudget(walks []survey2d.SideWalk, budget *proofbound.WorkBudget) ([]survey2d.SideWalk, error) {
	return coalesceWalksBudget(walks, budget)
}
func CoalesceWalksContext(ctx context.Context, walks []survey2d.SideWalk) ([]survey2d.SideWalk, error) {
	return coalesceWalksContext(ctx, walks)
}
func CoalesceChainWalksContext(ctx context.Context, walks []survey2d.SideWalk) ([]survey2d.SideWalk, error) {
	return coalesceChainWalksContext(ctx, walks)
}

// walkOf resolves one recorded segment into its walk geometry.
//
// work is the RECORD's free-form work counter (docs/spline-design.md §5.2), and
// walkOf NEVER mints one: the R7 ceiling bounds one record's total free-form
// work, so a counter minted per call would hand every segment — and every later
// phase of the same operation — a fresh full ceiling. Callers that already hold
// the counter a moments preflight opened for this record pass THAT one, so the
// walk's arc-length bracket spends what the preflight left rather than a second
// ceiling; callers with no preflight in hand mint exactly one for the whole
// record walk. An analytic segment charges nothing, so a nil counter is harmless
// there and refused on the free-form arm rather than quietly replaced.
func walkOf(seg CurveSegment, work *freeform.FreeformWork) (survey2d.SegmentWalk, error) {
	seg, err := normalizeSegment(seg)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	switch seg := seg.(type) {
	case LineSeg:
		u0, v0 := lerp2(seg.Start, seg.End, seg.TStart)
		u1, v1 := lerp2(seg.Start, seg.End, seg.TEnd)
		du, dv := u1-u0, v1-v0
		length := math.Hypot(du, dv)
		lengthBound, lengthUpper, coordUpper := lineWalkBounds(seg, length)
		tangentBound := lineWalkTangentBound(seg, du, dv)
		return survey2d.SegmentWalk{
			StartU: u0, StartV: v0, EndU: u1, EndV: v1,
			StartBound: lineWalkEndBound(seg, seg.TStart, u0, v0),
			EndBound:   lineWalkEndBound(seg, seg.TEnd, u1, v1),
			TanInU:     du, TanInV: dv, TanOutU: du, TanOutV: dv,
			TanInBound:  tangentBound,
			TanOutBound: tangentBound,
			Length:      length,
			LengthBound: lengthBound,
			LengthUpper: lengthUpper,
			CoordUpper:  coordUpper,
		}, nil
	case CircleSeg:
		r, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return survey2d.SegmentWalk{}, fmt.Errorf(`decad: a circle segment's radius is not a length: %w`, err)
		}
		if seg.CCW != (seg.TStart < seg.TEnd) {
			return survey2d.SegmentWalk{}, fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, ErrDegenerate)
		}
		th0, th1 := 2*math.Pi*seg.TStart, 2*math.Pi*seg.TEnd
		w := circularWalk(
			seg.Center.U,
			seg.Center.V,
			r,
			th0,
			th1,
			math.Abs(r),
			proofbound.CircularSweepUpper(seg.TStart, seg.TEnd),
		)
		w.Closed = math.Abs(math.Abs(th1-th0)-2*math.Pi) < 1e-12
		w.StartBound = circularWalkEndBound(seg, seg.TStart, w.StartU, w.StartV)
		w.EndBound = circularWalkEndBound(seg, seg.TEnd, w.EndU, w.EndV)
		if iv, ok := circularLengthInterval(seg); ok {
			w.LengthBound = math.Min(w.LengthBound, proofbound.IntervalFloatError(iv, w.Length))
		}
		return w, nil
	case ArcSeg:
		radius := math.Hypot(seg.Start.U-seg.Center.U, seg.Start.V-seg.Center.V)
		a0 := math.Atan2(seg.Start.V-seg.Center.V, seg.Start.U-seg.Center.U)
		a1 := math.Atan2(seg.End.V-seg.Center.V, seg.End.U-seg.Center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		w := circularWalk(
			seg.Center.U,
			seg.Center.V,
			radius,
			a0+seg.TStart*sweep,
			a0+seg.TEnd*sweep,
			arcRadiusUpper(seg),
			proofbound.CircularSweepUpper(seg.TStart, seg.TEnd),
		)
		pinArcWalkEnds(&w, seg)
		// circularWalkEnclosures brackets the radius from the same exact
		// squared Start-to-Center distance arcWalkRadiusBound does, so its
		// radius interval IS that function's bracket and is read here rather
		// than built twice. Both ends are floatRat of a float, so Float64
		// returns those floats exactly. The enclosures answer false exactly
		// where the bracket overflows, and arcWalkRadiusBound answers +Inf
		// there on its own.
		if rIv, sweepIv, ok := circularWalkEnclosures(seg); ok {
			rLo, _ := rIv.Lo.Float64()
			rHi, _ := rIv.Hi.Float64()
			w.RadiusBound = arcRadiusBoundFromBracket(radius, rLo, rHi)
			w.LengthBound = math.Min(w.LengthBound, proofbound.IntervalFloatError(proofbound.IntervalMul(rIv, sweepIv), w.Length))
		} else {
			w.RadiusBound = arcWalkRadiusBound(seg, radius)
		}
		return w, nil
	default:
		if !isFreeformSegment(seg) {
			return survey2d.SegmentWalk{}, fmt.Errorf(`%w: this evaluator sweeps profiles of line, arc, circle and Tier A free-form segments only; the profile has a %T segment it cannot sweep into a side face yet`, ErrUnsupported, seg)
		}
		return freeformWalk(seg, work)
	}
}

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

// freeformWalk resolves a Tier A free-form segment into its walk geometry
// (docs/spline-design.md Table F). Every field it fills is a proof:
//
//   - the endpoints are the converted chain's own first and last control
//     points, which a Bézier interpolates exactly, each under the bound of the
//     one rounding that conversion committed (freeformEndpointBounds);
//   - the tangents are the hodograph at those ends, exact directions;
//   - the length is §6.1's proven two-sided bracket, so lengthBound is
//     positive and the walk NEVER claims an exact length — a control net
//     collapsed to a single point has no positive bracket and refuses as
//     ErrDegenerate rather than resolve into a walk (Table R row R14), and a
//     curve whose enclosure runs past MaxFloat64 refuses as ErrUnsupported
//     (R15); freeform.FreeformArcLength owns both;
//   - coordUpper and lengthUpper are convex-hull envelopes, so they bound the
//     curve and not merely its control net.
//
// axisRadiusUpper and axisMomentUpper stay zero: they are revolve's readings,
// and revolve refuses a free-form walk before reaching them.
//
// The conversion and the length bracket are charged against the caller's counter
// — the record's, never one minted here. A caller that reaches this arm with no
// counter has no ceiling at all, which is the one thing §5.2 forbids, so the
// resolution refuses rather than run unbounded work.
func freeformWalk(seg CurveSegment, work *freeform.FreeformWork) (survey2d.SegmentWalk, error) {
	if work == nil {
		return survey2d.SegmentWalk{}, errFreeformWalkUncounted
	}
	spans, reversed, err := freeformBezierSpans(seg, work)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	start, end, err := freeformEndpoints(spans, reversed)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	length, bound, err := freeform.FreeformArcLength(spans, work)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	tangents, err := freeform.FreeformEndTangents(spans, reversed)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	startBound, endBound := freeformEndpointBounds(spans, reversed, start, end)
	return survey2d.SegmentWalk{
		StartU: start.U, StartV: start.V,
		EndU: end.U, EndV: end.V,
		StartBound: startBound,
		EndBound:   endBound,
		// A closed free-form curve returns to its start, so it carries no
		// junction vertex — the same fact CircleSeg's closed walk states.
		Closed:          start == end,
		TanInU:          tangents.InU,
		TanInV:          tangents.InV,
		TanInBound:      tangents.InBound,
		TanOutU:         tangents.OutU,
		TanOutV:         tangents.OutV,
		TanOutBound:     tangents.OutBound,
		Length:          length,
		LengthBound:     bound,
		LengthUpper:     proofbound.UpRound(length + bound),
		CoordUpper:      freeform.FreeformControlExtent(spans),
		Kind:            survey2d.WalkFreeform,
		Spans:           spans,
		Reversed:        reversed,
		FitInterpolated: isFitSplineSeg(seg),
	}, nil
}

// errFreeformWalkUncounted is the refusal of a free-form resolution handed no
// record counter. It is ErrUnsupported because the curve exists and this
// evaluator declines to resolve it without the ceiling §5.2 requires — never a
// silently minted counter, which is the second full ceiling the rule forbids.
var errFreeformWalkUncounted = fmt.Errorf(
	`%w: a free-form segment's walk needs its record's free-form work counter`, ErrUnsupported,
)

// pinArcWalkEnds states an arc walk's natural bounds as the record's own
// endpoints. A recorded arc runs Start → End over [0, 1] about Center
// (record.go), so its value at t = 0 is Start and at t = 1 is End, exactly,
// while circularWalk reaches those same two points through atan2 and cos/sin —
// a route that need not land back on them, because the angle it evaluates at
// the far bound is itself the rounded a0 + sweep. Only the two endpoints are
// restated; the walk's centre, radius, angles and tangents keep circularWalk's
// own values, and every reading derived from them keeps its own bound.
//
// This is the rule lerp2 (moments.go) applies at a line's own bounds, and the
// rule seam.go's edgeJoin applies when it reads an uncut bound off the record
// rather than off sketch's node. It matters for the same reason: buildPrismScene
// (prism_boolean.go) creates one sketch point per walked endpoint, so a walk
// that missed the vertex two segments share would offer sketch two points where
// the record states one, and RecordProfile would then refuse the region the
// arrangement admits on its own proximity threshold.
//
// A trimmed bound's POSITION is left alone: it has no recorded coordinate of
// its own, and inventing one is what this seam never does. What it does get is
// the bound circularWalk's route actually owes — see arcWalkEnd, which owns the
// natural-bound test for both readings so the pinned position and the zero
// bound can never drift apart.
func pinArcWalkEnds(w *survey2d.SegmentWalk, seg ArcSeg) {
	w.StartU, w.StartV, w.StartBound = arcWalkEnd(seg, seg.TStart, w.StartU, w.StartV)
	w.EndU, w.EndV, w.EndBound = arcWalkEnd(seg, seg.TEnd, w.EndU, w.EndV)
}

// arcWalkEnd states one arc walk end: its position and the proven bound on each
// of its components. At a natural bound the record states the point verbatim,
// so the walk reads Start or End and the bound is zero — the pin and the zero
// are one decision, taken here once. At any other parameter the walk keeps
// circularWalk's own held pair under the bound circularWalkEndBound proves for
// it.
//
// What the natural-bound zero states is that the held pair IS the recorded
// coordinate, with no rounding of this walk's own. It does NOT state that the
// recorded coordinate is the point the DENOTED curve passes through there. For
// an arc the two coincide at t == 0 and need not at t == 1: the denoted curve
// takes its radius from Start alone (circularEndpointInterval, moments.go), so
// its t == 1 point sits at Start's radius and End's angle, which is the
// recorded End only where the two recorded radii are equal — an equality
// nothing in this package certifies. A consumer that publishes a station's
// displacement from the DENOTED point owes that radial residual on top of this
// zero; docs/loft-design.md §5.2 names the term and loft_build.go's
// arcNaturalEndRadialUpper charges it for the loft.
func arcWalkEnd(seg ArcSeg, t, heldU, heldV float64) (float64, float64, proofbound.WalkEndBound) {
	switch t {
	case 0:
		return seg.Start.U, seg.Start.V, proofbound.WalkEndBound{}
	case 1:
		return seg.End.U, seg.End.V, proofbound.WalkEndBound{}
	}
	return heldU, heldV, circularWalkEndBound(seg, t, heldU, heldV)
}

// circularWalk builds the walk geometry of a circular path about (cu, cv).
//
// Its tangents REFUSE a bound (+Inf): the held components are math.Sincos
// evaluations at th0/th1, and those angles are themselves computed — a
// CircleSeg's from a float multiply by 2π, an ArcSeg's from math.Atan2 of the
// recorded differences — so neither the trig nor its argument is a quantity
// THIS function can enclose, holding floats alone. Stating zero there would
// hand a consumer an exactness the evaluator never proved; +Inf makes the
// absence visible, which is what every consumer refuses on.
//
// The endpoints are the same floats and carry the same absence, but they are
// not left at it: each caller holds the recorded segment those floats came
// from, and stamps the enclosure that record proves for its own endpoints
// (circularWalkEndBound) over the zero this function leaves behind. What has no
// enclosure is the tangent, not the point.
func circularWalk(cu, cv, r, th0, th1, radiusUpper, sweepUpper float64) survey2d.SegmentWalk {
	sin0, cos0 := math.Sincos(th0)
	sin1, cos1 := math.Sincos(th1)
	sign := 1.0
	if th1 < th0 {
		sign = -1
	}
	length := r * math.Abs(th1-th0)
	lengthUpper := proofbound.ProductUpper(radiusUpper, sweepUpper)
	coordUpper := proofbound.AbsSumUpper(cu, cv, radiusUpper, radiusUpper)
	return survey2d.SegmentWalk{
		StartU: cu + r*cos0, StartV: cv + r*sin0,
		EndU: cu + r*cos1, EndV: cv + r*sin1,
		TanInU: -sign * sin0, TanInV: sign * cos0,
		TanOutU: -sign * sin1, TanOutV: sign * cos1,
		TanInBound:  math.Inf(1),
		TanOutBound: math.Inf(1),
		Length:      length,
		LengthBound: proofbound.ConservativeValueError(length, lengthUpper),
		LengthUpper: lengthUpper,
		CoordUpper:  coordUpper,
		Kind:        survey2d.WalkCircular,
		CU:          cu, CV: cv, Radius: r, Th0: th0, Th1: th1,
	}
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

// coalesceWalks merges consecutive collinear line walks, wrap-around
// included. Circular walks never merge; a loop that is entirely one straight
// line is degenerate and left to the area gate.
func coalesceWalks(walks []survey2d.SideWalk) []survey2d.SideWalk {
	out, _ := coalesceWalksBudget(walks, nil)
	return out
}

func coalesceWalksBudget(walks []survey2d.SideWalk, budget *proofbound.WorkBudget) ([]survey2d.SideWalk, error) {
	return coalesceWalksWithPoll(func() error { return survey2d.WallBudgetStep(budget) }, walks, true)
}

func coalesceWalksContext(ctx context.Context, walks []survey2d.SideWalk) ([]survey2d.SideWalk, error) {
	return coalesceWalksWithPoll(ctx.Err, walks, true)
}

// coalesceChainWalksContext is coalesceWalksContext's OPEN-walk counterpart:
// it merges adjacent collinear segments exactly as a loop's coalescing does,
// but never wraps the last walk into the first. An open chain's two ends are
// free — they meet no neighbour to merge into
// (docs/surface-design.md §13.4).
func coalesceChainWalksContext(ctx context.Context, walks []survey2d.SideWalk) ([]survey2d.SideWalk, error) {
	return coalesceWalksWithPoll(ctx.Err, walks, false)
}

func coalesceWalksWithPoll(poll func() error, walks []survey2d.SideWalk, wrap bool) ([]survey2d.SideWalk, error) {
	collinear := func(a, b survey2d.SideWalk) bool {
		if !a.IsLine() || !b.IsLine() {
			return false
		}
		cross := a.TanOutU*b.TanInV - a.TanOutV*b.TanInU
		dot := a.TanOutU*b.TanInU + a.TanOutV*b.TanInV
		scale := math.Hypot(a.TanOutU, a.TanOutV) * math.Hypot(b.TanInU, b.TanInV)
		return dot > 0 && math.Abs(cross) <= 1e-12*scale
	}
	merge := func(a, b survey2d.SideWalk) survey2d.SideWalk {
		a.EndU, a.EndV = b.EndU, b.EndV
		// The merged walk leaves where b leaves, so it inherits b's leaving
		// tangent AND the bound b proved on it — never a's, and never zero.
		a.TanOutU, a.TanOutV = b.TanOutU, b.TanOutV
		a.TanOutBound = b.TanOutBound
		length := proofbound.BoundedAdd(proofbound.MeasuredScalar(a.Length, a.LengthBound), proofbound.MeasuredScalar(b.Length, b.LengthBound))
		a.Length, a.LengthBound = length.Value, length.Bound
		a.LengthUpper = proofbound.AbsSumUpper(a.LengthUpper, b.LengthUpper)
		a.CoordUpper = math.Max(a.CoordUpper, b.CoordUpper)
		a.AxisRadiusUpper = math.Max(a.AxisRadiusUpper, b.AxisRadiusUpper)
		a.AxisMomentUpper = proofbound.AbsSumUpper(a.AxisMomentUpper, b.AxisMomentUpper)
		a.Segs = append(a.Segs, b.Segs...)
		return a
	}
	out := make([]survey2d.SideWalk, 0, len(walks))
	for _, w := range walks {
		if poll != nil {
			if err := poll(); err != nil {
				return nil, err
			}
		}
		if len(out) > 0 && collinear(out[len(out)-1], w) {
			out[len(out)-1] = merge(out[len(out)-1], w)
			continue
		}
		out = append(out, w)
	}
	// Wrap-around: a closed loop's last walk may continue into its first. An
	// open chain's never does (wrap is false), since its last segment meets
	// no neighbour at all.
	for wrap {
		if len(out) <= 1 || !collinear(out[len(out)-1], out[0]) {
			break
		}
		if poll != nil {
			if err := poll(); err != nil {
				return nil, err
			}
		}
		out[0] = merge(out[len(out)-1], out[0])
		out = out[:len(out)-1]
	}
	return out, nil
}
