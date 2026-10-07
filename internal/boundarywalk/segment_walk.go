package boundarywalk

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

type (
	CurveSegment = sectionrecord.CurveSegment
	LineSeg      = sectionrecord.LineSeg
	CircleSeg    = sectionrecord.CircleSeg
	ArcSeg       = sectionrecord.ArcSeg
	Point2       = sectionrecord.Point2
)

var (
	ErrDegenerate  = decaderr.ErrDegenerate
	ErrUnsupported = decaderr.ErrUnsupported
)

func normalizeSegment(segment CurveSegment) (CurveSegment, error) {
	return sectionrecord.NormalizeSegment(segment)
}
func isFreeformSegment(segment CurveSegment) bool { return splinebezier.IsFreeformSegment(segment) }
func freeformBezierSpans(segment CurveSegment, work *freeform.FreeformWork) ([]freeform.BezierSpan, bool, error) {
	return splinebezier.FreeformBezierSpans(segment, work)
}
func freeformEndpoints(spans []freeform.BezierSpan, reversed bool) (Point2, Point2, error) {
	return splinebezier.FreeformEndpoints(spans, reversed)
}
func freeformEndpointBounds(spans []freeform.BezierSpan, reversed bool, start, end Point2) (proofbound.WalkEndBound, proofbound.WalkEndBound) {
	return splinebezier.FreeformEndpointBounds(spans, reversed, start, end)
}
func isFitSplineSeg(segment CurveSegment) bool { return splinebezier.IsFitSplineSeg(segment) }
func circularEndpointInterval(segment CurveSegment, t *big.Rat) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularbounds.EndpointInterval(circularbounds.RecordSegment(segment), t)
}
func circularWalkEnclosures(segment CurveSegment) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularbounds.WalkEnclosures(circularbounds.RecordSegment(segment))
}
func circularLengthInterval(segment CurveSegment) (proofbound.RatInterval, bool) {
	return circularbounds.LengthInterval(circularbounds.RecordSegment(segment))
}
func exactCoordinateDelta(a, b float64) *big.Rat { return circularbounds.ExactCoordinateDelta(a, b) }
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
func RatL1Upper(values ...*big.Rat) float64 { return ratL1Upper(values...) }

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
