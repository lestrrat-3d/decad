package curveconvert

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/record"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

type (
	Point2           = record.Point2
	CurveSegment     = record.CurveSegment
	SplineSeg        = record.SplineSeg
	NURBSSeg         = record.NURBSSeg
	ClosedSplineSeg  = record.ClosedSplineSeg
	FitSplineSeg     = record.FitSplineSeg
	ConicSeg         = record.ConicSeg
	EllipseSeg       = record.EllipseSeg
	EllipticalArcSeg = record.EllipticalArcSeg
)

var (
	ErrDegenerate  = decaderr.ErrDegenerate
	ErrNotFinite   = decaderr.ErrNotFinite
	ErrUnsupported = decaderr.ErrUnsupported
)

func normalizeSegment(segment CurveSegment) (CurveSegment, error) {
	return record.NormalizeSegment(segment)
}
func validateNURBSSegmentSizes(segment NURBSSeg) error {
	return record.ValidateNURBSSegmentSizes(segment)
}
func validateNURBSSegmentContent(segment NURBSSeg) error {
	return record.ValidateNURBSSegmentContent(segment)
}
func finiteSegmentValue(value float64) bool { return record.FiniteSegmentValue(value) }

func IsFreeformSegment(segment CurveSegment) bool { return isFreeformSegment(segment) }
func FreeformBezierSpans(segment CurveSegment, work *freeform.FreeformWork) ([]survey2d.BezierSpan, bool, error) {
	return freeformBezierSpans(segment, work)
}
func RatPointsOf(points []Point2) ([]survey2d.RatPoint, error) { return ratPointsOf(points) }
func SplineBezierSpans(segment SplineSeg, work *freeform.FreeformWork) ([]survey2d.BezierSpan, error) {
	return splineBezierSpans(segment, work)
}
func NURBSBezierSpans(segment NURBSSeg, work *freeform.FreeformWork) ([]survey2d.BezierSpan, error) {
	return nurbsBezierSpans(segment, work)
}
func ClosedSplineBezierSpans(segment ClosedSplineSeg, work *freeform.FreeformWork) ([]survey2d.BezierSpan, error) {
	return closedSplineBezierSpans(segment, work)
}
func ShiftFreeformSpans(spans []survey2d.BezierSpan, anchor Point2) error {
	return shiftFreeformSpans(spans, anchor)
}
func FreeformEndpoints(spans []survey2d.BezierSpan, reversed bool) (Point2, Point2, error) {
	return freeformEndpoints(spans, reversed)
}
func FreeformEndpointBounds(spans []survey2d.BezierSpan, reversed bool, start, end Point2) (proofbound.WalkEndBound, proofbound.WalkEndBound) {
	return freeformEndpointBounds(spans, reversed, start, end)
}
func Point2Of(point survey2d.RatPoint) (Point2, bool) { return point2Of(point) }

// This file is docs/spline-design.md §5.1's exact reduction: a recorded
// free-form curve becomes piecewise polynomial Bézier control points over
// math/big.Rat, with no rounding anywhere along the way. Control coordinates
// and knots are floats, so they are exact rationals, and knot insertion is a
// rational convex combination — so the converted spans ARE the recorded curve,
// not an approximation of it, and every integral taken over them
// (spline_moments.go) inherits that exactness.
//
// Every number the conversion reads is UPSTREAM's, taken exactly and never
// re-derived. A SplineSeg's knot vector is geom.ClampedKnots's own floats lifted
// into rationals, not the exact rationals j/(n−3) those floats are the roundings
// of: sketch defines the curve over the floats it stores, so integrating the
// exact rationals would integrate a DIFFERENT curve — and publish its
// representable area as an Exact claim about the recorded one.
//
// This file's own Boehm-insertion machinery converts SplineSeg,
// ClosedSplineSeg, and a NURBSSeg whose weights are all equal — three of
// Table F's four Tier A kinds; the fourth, FitSplineSeg, is Tier A too, but
// its own reduction lives in spline_fit.go, a natural cubic having no knot
// vector to insert into (freeformBezierSpans's own doc comment below
// dispatches to it). A rational NURBS is Tier C and is refused here — a
// rational span is not a polynomial Bézier, and pretending otherwise would
// silently integrate a different curve.
//
// Only a FULL recorded domain is converted, because §2 proves no other
// free-form range is recordable. The walk direction is not baked in: the
// caller reads the recorded range order and negates the signed result.

// isFreeformSegment reports whether the kind is one of the free-form five. It
// answers the KIND question only; whether the evaluator can integrate it is
// freeformBezierSpans's answer.
func isFreeformSegment(seg CurveSegment) bool {
	switch seg.(type) {
	case SplineSeg, ClosedSplineSeg, NURBSSeg, FitSplineSeg, ConicSeg, EllipseSeg, EllipticalArcSeg:
		return true
	default:
		return false
	}
}

// freeformBezierSpans converts one recorded free-form segment into exact
// polynomial Bézier spans, and reports whether the recorded walk runs against
// the curve's natural sense.
//
// FitSplineSeg is Tier A too, converted by spline_fit.go's own reduction over
// sketch's EXPORTED interpolant (geom.FitInterpolant) rather than by this
// file's Boehm-insertion machinery — a natural cubic has no knot vector to
// insert into, so it owns a closed form of its own.
//
// Every other kind outside Tier A refuses with its own reason, so a caller
// never has to infer which table row it hit:
//
//   - EllipticalArcSeg is row R2 — its pinned ends and its parametric ellipse
//     disagree, and no exact reconciliation exists (spline design §2.2).
//   - ConicSeg and EllipseSeg are Tier B: exact closed forms carrying
//     transcendental terms, so they are integrated by their own bracketed
//     formulas, not through a polynomial Bézier.
//   - A rational NURBSSeg is Tier C, refused above.
func freeformBezierSpans(seg CurveSegment, work *freeform.FreeformWork) ([]survey2d.BezierSpan, bool, error) {
	seg, err := normalizeSegment(seg)
	if err != nil {
		return nil, false, err
	}
	// The finiteness of the recorded range is decided for EVERY free-form kind
	// before the kind itself is, because it is a core §12 refusal about an input
	// field and not a statement about which kinds this evaluator reaches. Reading
	// it inside the Tier A arms alone would report a NaN range as ErrNotFinite on
	// a spline and as the kind's own ErrUnsupported reason on the other four.
	if tStart, tEnd, what, ok := freeformSegmentRange(seg); ok {
		if err := freeform.RequireFiniteFreeformRange(tStart, tEnd, what); err != nil {
			return nil, false, err
		}
	}
	switch seg := seg.(type) {
	case SplineSeg:
		spans, err := splineBezierSpans(seg, work)
		return spans, seg.TStart > seg.TEnd, err
	case ClosedSplineSeg:
		spans, err := closedSplineBezierSpans(seg, work)
		return spans, seg.TStart > seg.TEnd, err
	case NURBSSeg:
		spans, err := nurbsBezierSpans(seg, work)
		return spans, seg.TStart > seg.TEnd, err
	case FitSplineSeg:
		spans, err := fitSplineBezierSpans(seg, work)
		return spans, seg.TStart > seg.TEnd, err
	case EllipticalArcSeg:
		return nil, false, fmt.Errorf(
			`%w: an elliptical arc's pinned endpoints and its parametric ellipse disagree, so the record states no single exact curve`,
			ErrUnsupported,
		)
	case ConicSeg, EllipseSeg:
		return nil, false, fmt.Errorf(
			`%w: this evaluator has no closed form for a %T yet; its moments carry transcendental terms and need their own bracket`,
			ErrUnsupported, seg,
		)
	default:
		return nil, false, fmt.Errorf(`%w: %T is not a free-form segment`, ErrDegenerate, seg)
	}
}

// freeformSegmentRange returns a recognized free-form segment's recorded
// parameter range beside the name its refusals call it by. It is the one place
// the free-form kinds' range fields are read generically, so the finiteness
// refusal above reaches every one of them.
func freeformSegmentRange(seg CurveSegment) (float64, float64, string, bool) {
	switch seg := seg.(type) {
	case SplineSeg:
		return seg.TStart, seg.TEnd, "spline segment", true
	case ClosedSplineSeg:
		return seg.TStart, seg.TEnd, "closed spline segment", true
	case NURBSSeg:
		return seg.TStart, seg.TEnd, "NURBS segment", true
	case FitSplineSeg:
		return seg.TStart, seg.TEnd, "fit spline segment", true
	case ConicSeg:
		return seg.TStart, seg.TEnd, "conic segment", true
	case EllipseSeg:
		return seg.TStart, seg.TEnd, "ellipse segment", true
	case EllipticalArcSeg:
		return seg.TStart, seg.TEnd, "elliptical arc segment", true
	default:
		return 0, 0, "", false
	}
}

// ratPointsOf lifts recorded control points into exact rationals. A
// non-finite coordinate has no rational form and is rejected.
func ratPointsOf(points []Point2) ([]survey2d.RatPoint, error) {
	out := make([]survey2d.RatPoint, len(points))
	for i, point := range points {
		u, okU := proofbound.RatOf(point.U)
		v, okV := proofbound.RatOf(point.V)
		if !okU || !okV {
			return nil, fmt.Errorf(`%w: control point %d is not finite`, ErrNotFinite, i)
		}
		out[i] = survey2d.RatPoint{U: u, V: v}
	}
	return out, nil
}

// splineBezierSpans converts a SplineSeg — geom.Spline's clamped uniform cubic
// B-spline. Degree 3 and the knot vector geom.ClampedKnots(n) builds are the
// entity's DEFINITION rather than recorded data (seam §2), so the degree is
// restated here and the knot vector is READ FROM geom: its interior knots are
// float64(j)/float64(n−3), and those floats — not the exact rationals they round
// — are the curve sketch defines.
func splineBezierSpans(seg SplineSeg, work *freeform.FreeformWork) ([]survey2d.BezierSpan, error) {
	const degree = 3
	if err := freeform.RequireFullFreeformRange(seg.TStart, seg.TEnd, "spline segment"); err != nil {
		return nil, err
	}
	// The control count is a SIZE, so it refuses before the scan below rather
	// than after it.
	if len(seg.Control) < degree+1 {
		return nil, fmt.Errorf(
			`%w: a cubic B-spline needs at least %d control points, got %d`,
			ErrDegenerate, degree+1, len(seg.Control),
		)
	}
	// The whole conversion is charged BEFORE the first rational exists, and before
	// geom.ClampedKnots is even asked for its vector. degree is fixed and that
	// vector's shape is derived from the control count, so this record's insertion
	// demand is a pure function of that count — no knot has to be built or lifted to
	// learn it. Charging after the lift would let a record whose conversion is
	// hopelessly over budget allocate two rationals per control point first: the
	// open-spline charge is quadratic, so a refused record allocated three orders of
	// magnitude more than any accepted one.
	knots := len(seg.Control) + 4
	if err := work.Step(freeform.CostAdd(
		freeform.RationalLiftCost(len(seg.Control), knots, 0),
		freeform.ClampedConversionCost(len(seg.Control), knots, freeform.UniformKnotDemand(len(seg.Control), degree)),
	)); err != nil {
		return nil, err
	}
	ctrl, err := ratPointsOf(seg.Control)
	if err != nil {
		return nil, err
	}
	return freeform.ClampedBezierSpans(degree, ctrl, freeform.ClampedUniformKnots(len(ctrl)))
}

// nurbsBezierSpans converts a NURBSSeg. Only a NON-RATIONAL one — every weight
// equal — is Tier A: its spans are polynomial Béziers and its moments integrate
// exactly. A genuinely rational NURBS is Tier C and refuses here, because
// converting it to a polynomial Bézier would integrate a DIFFERENT curve and
// report the result as exact.
func nurbsBezierSpans(seg NURBSSeg, work *freeform.FreeformWork) ([]survey2d.BezierSpan, error) {
	// Order is the preflight's own: the O(1) refusals — the recorded range, then
	// every slice size — decide first, because they read no element, so a record
	// whose knot count cannot match its control count is refused in constant time
	// however many control points it holds. The SIZE-DERIVED lift charge comes
	// next, still O(1), and only then is the record's own CONTENT read and its TIER
	// decided.
	//
	// The BOUND wins over the tier here, and the trade is exact. Deciding the tier
	// means reading all n weights, so it is inherently linear and no O(1) charge can
	// follow it: a scan placed ahead of every charge is unbounded and uncancellable,
	// which is precisely what freeform.FreeformWorkLimit exists to stop (the public
	// ProfileRecord methods take no context). Levying the lift charge first bounds
	// that scan under the same ceiling, under freeform.ChargeRationalLift's own invariant:
	// every pass between that charge and the conversion charge below — the content
	// checks, the tier test and freeform.FloatKnotDemand — is a single walk over one array
	// whose length the charge counts.
	//
	// What it costs is small and worth stating. A record whose SIZE alone fits the
	// ceiling — every record that could ever yield a measurement — still reads its
	// own Tier C reason below. Only a record too large for the lift charge loses it,
	// and such a record is refused either way, so R7 is equally true of it.
	if err := freeform.RequireFullFreeformRange(seg.TStart, seg.TEnd, "NURBS segment"); err != nil {
		return nil, err
	}
	if err := validateNURBSSegmentSizes(seg); err != nil {
		return nil, err
	}
	// The rational lift is the linear floor under everything below: the content
	// scan, the tier test and the conversion all walk the same arrays it counts.
	if err := freeform.ChargeRationalLift(work, len(seg.Control), len(seg.Knots), len(seg.Weights)); err != nil {
		return nil, err
	}
	if err := validateNURBSSegmentContent(seg); err != nil {
		return nil, err
	}
	for i, weight := range seg.Weights {
		if weight != seg.Weights[0] {
			return nil, fmt.Errorf(
				`%w: a rational NURBS segment (weight %d differs from weight 0) needs certified quadrature this evaluator does not have`,
				ErrUnsupported, i,
			)
		}
	}
	// The conversion is charged before the rational lift RUNS, not after it. The
	// insertion demand reads the RECORDED float knots, over a vector the content
	// check above just proved finite and non-decreasing, so its runs are the
	// multiplicities the rational vector will hold. Charging after the lift would
	// let a record whose conversion is quadratically over budget allocate every one
	// of its rationals first and refuse afterwards.
	if err := work.Step(freeform.ClampedConversionCost(
		len(seg.Control),
		len(seg.Knots),
		freeform.FloatKnotDemand(seg.Degree, len(seg.Control), seg.Knots),
	)); err != nil {
		return nil, err
	}
	ctrl, err := ratPointsOf(seg.Control)
	if err != nil {
		return nil, err
	}
	knots := make([]*big.Rat, len(seg.Knots))
	for i, knot := range seg.Knots {
		rat, ok := proofbound.RatOf(knot)
		if !ok {
			return nil, fmt.Errorf(`%w: NURBS knot %d is not finite`, ErrNotFinite, i)
		}
		knots[i] = rat
	}
	return freeform.ClampedBezierSpans(seg.Degree, ctrl, knots)
}

// closedSplineBezierSpans converts a ClosedSplineSeg — geom.ClosedSpline's
// periodic uniform cubic B-spline. Its definition is per-span rather than
// through a knot vector: span i blends the four cyclic controls P[i..i+3] with
// the standard uniform cubic basis, so it converts by the closed-form uniform
// B-spline to Bézier identity and needs no knot insertion. n control points
// give n spans, which is what closes the loop.
func closedSplineBezierSpans(seg ClosedSplineSeg, work *freeform.FreeformWork) ([]survey2d.BezierSpan, error) {
	if err := freeform.RequireFullFreeformRange(seg.TStart, seg.TEnd, "closed spline segment"); err != nil {
		return nil, err
	}
	// The control count is a SIZE, so it refuses before the scan below rather
	// than after it.
	if len(seg.Control) < 3 {
		return nil, fmt.Errorf(
			`%w: a closed cubic B-spline needs at least 3 control points, got %d`,
			ErrDegenerate, len(seg.Control),
		)
	}
	// The lift and the whole conversion are charged together, before either
	// allocates: this conversion needs no knot insertion, so its cost is the 4n
	// control points of the n spans below and is known from the control count
	// alone.
	if err := work.Step(freeform.CostAdd(
		freeform.RationalLiftCost(len(seg.Control), 0, 0),
		freeform.CostMul(4, uint64(len(seg.Control))),
	)); err != nil {
		return nil, err
	}
	ctrl, err := ratPointsOf(seg.Control)
	if err != nil {
		return nil, err
	}
	n := len(ctrl)
	spans := make([]survey2d.BezierSpan, n)
	for i := range n {
		// The four cyclic controls of span i, matching geom's own indexing.
		q0, q1 := ctrl[i], ctrl[(i+1)%n]
		q2, q3 := ctrl[(i+2)%n], ctrl[(i+3)%n]
		// The uniform cubic B-spline to Bézier identity, per coordinate:
		// B₀ = (Q₀+4Q₁+Q₂)/6, B₁ = (2Q₁+Q₂)/3, B₂ = (Q₁+2Q₂)/3,
		// B₃ = (Q₁+4Q₂+Q₃)/6.
		spans[i] = survey2d.BezierSpan{
			freeform.RatWeighted([]survey2d.RatPoint{q0, q1, q2}, []int64{1, 4, 1}, 6),
			freeform.RatWeighted([]survey2d.RatPoint{q1, q2}, []int64{2, 1}, 3),
			freeform.RatWeighted([]survey2d.RatPoint{q1, q2}, []int64{1, 2}, 3),
			freeform.RatWeighted([]survey2d.RatPoint{q1, q2, q3}, []int64{1, 4, 1}, 6),
		}
	}
	return spans, nil
}

// shiftFreeformSpans re-references a converted chain to the walk anchor over
// EXACT rationals, in place. Its cost is charged by freeform.ChargeFreeformShift at the
// preflight, so it takes no counter of its own.
//
// The anchor must never be subtracted from the recorded floats first. A Bézier
// span IS the recorded curve (§5.1) only while its control coordinates are the
// recorded ones taken exactly; fl(p−anchor) rounds, so a chain built from
// pre-shifted floats is the exact form of a DIFFERENT curve, and every
// exactness claim downstream — including the zero bound that publishes as
// Exact — would then speak for that curve instead of the recorded one.
// Subtracting here is exact, because a float and the anchor are both exact
// rationals.
//
// It writes through the caller's own slice, so the chain it is handed MUST be
// one the caller owns. validateFreeformMomentSegment converts its own through
// freeformBezierSpans and passes that, which is what makes this safe today. A
// survey2d.SegmentWalk's spans are NOT such a chain: one profileWalks set is read by the
// build, the tessellation, the extent readings and every rigid re-evaluation of
// the record, and this write would reach all of them at once, past a cache
// guard that only ever compares the record (internal/walkconvert/walk.go's spans field).
// Hand it a copy, or a fresh conversion.
func shiftFreeformSpans(spans []survey2d.BezierSpan, anchor Point2) error {
	u, okU := proofbound.RatOf(anchor.U)
	v, okV := proofbound.RatOf(anchor.V)
	if !okU || !okV {
		return fmt.Errorf(`%w: a free-form walk's anchor is not finite`, ErrNotFinite)
	}
	for _, span := range spans {
		for i, point := range span {
			span[i] = survey2d.RatPoint{
				U: new(big.Rat).Sub(point.U, u),
				V: new(big.Rat).Sub(point.V, v),
			}
		}
	}
	return nil
}

// freeformEndpoints returns the converted chain's own endpoints in the recorded
// walk order — the first and last Bézier control point, which a Bézier
// interpolates exactly, so these are the curve's endpoints and not samples.
func freeformEndpoints(spans []survey2d.BezierSpan, reversed bool) (Point2, Point2, error) {
	first, last, ok := freeform.FreeformEndControls(spans, reversed)
	if !ok {
		return Point2{}, Point2{}, fmt.Errorf(`%w: a converted free-form curve holds no span`, ErrDegenerate)
	}
	start, okStart := point2Of(first)
	end, okEnd := point2Of(last)
	if !okStart || !okEnd {
		return Point2{}, Point2{}, fmt.Errorf(`%w: a converted free-form endpoint is not representable`, ErrNotFinite)
	}
	return start, end, nil
}

// freeformEndpointBounds is the proven per-component bound on each endpoint
// freeformEndpoints published — survey2d.SegmentWalk's startBound/endBound for this
// kind. A Bézier interpolates its end control points exactly, so the only error
// an endpoint carries is the ONE rounding point2Of commits taking the exact
// rational control point into float64, and that is measured here against the
// rational itself. A chain with no span answers +Inf, the underivable bound
// every consumer refuses on.
func freeformEndpointBounds(spans []survey2d.BezierSpan, reversed bool, start, end Point2) (proofbound.WalkEndBound, proofbound.WalkEndBound) {
	first, last, ok := freeform.FreeformEndControls(spans, reversed)
	if !ok {
		unbounded := proofbound.WalkEndBound{U: math.Inf(1), V: math.Inf(1)}
		return unbounded, unbounded
	}
	bound := func(p survey2d.RatPoint, held Point2) proofbound.WalkEndBound {
		return proofbound.WalkEndBound{
			U: proofarith.RationalFloatError(p.U, held.U),
			V: proofarith.RationalFloatError(p.V, held.V),
		}
	}
	return bound(first, start), bound(last, end)
}

func point2Of(p survey2d.RatPoint) (Point2, bool) {
	u, _ := p.U.Float64()
	v, _ := p.V.Float64()
	if proofbound.IsNonFinite(u) || proofbound.IsNonFinite(v) {
		return Point2{}, false
	}
	return Point2{U: u, V: v}, true
}
