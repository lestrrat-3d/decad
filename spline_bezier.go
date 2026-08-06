package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/sketch/geom"
)

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
// The conversion serves the Tier A kinds of Table F: SplineSeg,
// ClosedSplineSeg, and a NURBSSeg whose weights are all equal. A rational
// NURBS is Tier C and is refused here — a rational span is not a polynomial
// Bézier, and pretending otherwise would silently integrate a different curve.
//
// Only a FULL recorded domain is converted, because §2 proves no other
// free-form range is recordable. The walk direction is not baked in: the
// caller reads the recorded range order and negates the signed result.

// ratPoint is a plane-local coordinate over exact rationals — Point2's exact
// counterpart, in millimetres by the same core §5.2 convention.
type ratPoint struct{ u, v *big.Rat }

// bezierSpan is one polynomial Bézier piece of a converted free-form curve:
// its control points in order, so its degree is len(bezierSpan)-1. A Bézier
// interpolates its first and last control point exactly, which is why
// consecutive spans join on a shared coordinate value and the chain's first and
// last control points ARE the recorded curve's own endpoints.
type bezierSpan []ratPoint

// freeformWorkLimit is the fixed ceiling on ONE RECORD's free-form conversion,
// integration and length-bracket work, in charged units (one scanned or copied
// knot-insertion entry, or one integrand coefficient product). It bounds a whole
// ProfileRecord rather than each of its segments, and it bounds the whole
// OPERATION over that record rather than each pass through it: a counter opened
// per segment reads a record of individually cheap curves as cheap however many
// of them it holds, and a counter opened per pass lets a later pass run work an
// earlier one already proved unaffordable. Either way the aggregate — which is
// what actually runs — would be unbounded. Public ProfileRecord methods take no
// context, so the limit is fixed rather than caller-set, exactly as shellInradiusWorkLimit is for the inward shell survey.
// Reaching it is Table R row R7: ErrUnsupported, never a widened float path.
const freeformWorkLimit uint64 = 1 << 20

// freeformCostCeiling is where the conservative cost arithmetic below
// saturates. Any estimate that reaches it is already over budget on its own —
// step refuses at freeformWorkLimit and this sits one unit above it — so the
// helpers never need to represent a larger number and can never wrap.
const freeformCostCeiling = freeformWorkLimit + 1

// costAdd and costMul are the saturating arithmetic every preflight estimate is
// built from. A preflight is charged BEFORE the work it pays for is allocated,
// so it must be an UPPER bound: saturating (never wrapping) is what keeps an
// astronomically large shape refusing instead of charging a small number.
func costAdd(a, b uint64) uint64 {
	if a >= freeformCostCeiling || b >= freeformCostCeiling || a > freeformCostCeiling-b {
		return freeformCostCeiling
	}
	return a + b
}

func costMul(a, b uint64) uint64 {
	if a == 0 || b == 0 {
		return 0
	}
	if a >= freeformCostCeiling || b >= freeformCostCeiling || a > freeformCostCeiling/b {
		return freeformCostCeiling
	}
	return a * b
}

// freeformWork is the charged counter behind freeformWorkLimit.
type freeformWork struct{ spent uint64 }

// newFreeformWork opens ONE record's counter. Minting is deliberately explicit
// and rare: the ceiling bounds a record's total free-form work across the whole
// operation, so every new counter is a new full ceiling. Mint one where a record
// walk begins with no preflight counter in hand; everywhere else, pass the
// counter the record already has.
func newFreeformWork() *freeformWork { return &freeformWork{} }

func (w *freeformWork) step(n uint64) error {
	if w == nil {
		return nil
	}
	if n > freeformWorkLimit-w.spent {
		w.spent = freeformWorkLimit
		return fmt.Errorf(
			`%w: free-form exact integration needs more than the fixed work budget of %d`,
			ErrUnsupported, freeformWorkLimit,
		)
	}
	w.spent += n
	return nil
}

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
func freeformBezierSpans(seg CurveSegment, work *freeformWork) ([]bezierSpan, bool, error) {
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
		if err := requireFiniteFreeformRange(tStart, tEnd, what); err != nil {
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

// requireFiniteFreeformRange rejects a non-finite recorded range on ANY
// free-form kind. Core §12 gives [ErrNotFinite] for a non-finite input, which is
// what every other segment kind reports for exactly this field and what the
// free-form path already reports for a non-finite control coordinate. It is
// therefore decided ahead of the kind dispatch, never inside one kind's arm: a
// NaN fails both full-domain equality tests, so a kind that tested the range
// alone would report it as a trimmed range and a kind that never tested it at
// all would report its own staging reason instead. The test is O(1) on the
// recorded parameters, so it stands with the structural size refusals, ahead of
// any content scan.
func requireFiniteFreeformRange(tStart, tEnd float64, what string) error {
	if finiteMomentValues(tStart, tEnd) {
		return nil
	}
	return fmt.Errorf(
		`%w: a %s's recorded range is not finite (range [%v, %v])`,
		ErrNotFinite, what, tStart, tEnd,
	)
}

// requireFullFreeformRange rejects a recorded free-form range that is not the
// entity's full domain. spline design §2 proves none is recordable, so reaching
// this is a caller-built or decoded record that bypassed the seam — refuse
// rather than integrate a piece the conversion does not cover.
//
// It is the Tier A arms' own gate, and it stays there. Table R states R2 and R6
// unconditionally and carries no row for a trimmed range reaching the evaluator,
// so a kind refused for its own cause reports that cause whatever its range
// says. Finiteness is the separate refusal above, already decided for every kind
// before this runs.
func requireFullFreeformRange(tStart, tEnd float64, what string) error {
	if (tStart == 0 && tEnd == 1) || (tStart == 1 && tEnd == 0) {
		return nil
	}
	return fmt.Errorf(
		`%w: a %s must span its full domain; a trimmed free-form range is never recordable (range [%v, %v])`,
		ErrUnsupported, what, tStart, tEnd,
	)
}

// chargeRationalLift charges the rational lift of a recorded curve's own
// arrays before any of them is allocated: two rationals per control point and
// one per knot. It is the linear floor under the conversion, so a record too
// large to hold rationally refuses at the ceiling rather than allocating first
// and refusing afterwards.
//
// It is computed from SLICE LENGTHS alone — every array a preflight pass walks,
// the weights among them — which is what lets it be levied ahead of every
// element scan, the content checks and the tier test included.
//
// THE INVARIANT IT CARRIES IS NOT "charge equals work". It is that the work is a
// FIXED MULTIPLE of the charge: every element-touching pass in the preflight is
// a SINGLE walk over one array whose own length is a term of this charge, so a
// segment's element visits are at most K times the units it levies here, and a
// whole record's are at most K·freeformWorkLimit however the record is split
// into segments. K is 4 today — the widest kind is a NURBSSeg, whose preflight
// walks its controls once (validateSegmentPoints), its knots three times (the
// finite/monotone scan, validateNURBSInteriorMultiplicity and floatKnotDemand),
// its weights twice (the finite/positive scan and the equal-weight tier test),
// and reads two knots per degree for the clamping check, where the degree is
// below the control count this charge already counts twice. That is under
// 3·controls + 3·knots + 2·weights, hence under 4·(2·controls + knots +
// weights).
//
// Stated that way, ADDING a validator can only raise K and can never unbound the
// work: one more single walk over Control, Knots or Weights adds at most one to
// the multiple. What the invariant does NOT cover is a pass that walks anything
// those three lengths do not measure, or that walks one of them more than a
// constant number of times — a nested or superlinear pass. Such a pass owes its
// own charge, exactly as the conversion's own quadratic does
// (clampedConversionCost).
func chargeRationalLift(work *freeformWork, controls, knots, weights int) error {
	return work.step(rationalLiftCost(controls, knots, weights))
}

func rationalLiftCost(controls, knots, weights int) uint64 {
	return costAdd(costAdd(costMul(2, uint64(controls)), uint64(knots)), uint64(weights))
}

// ratPointsOf lifts recorded control points into exact rationals. A
// non-finite coordinate has no rational form and is rejected.
func ratPointsOf(points []Point2) ([]ratPoint, error) {
	out := make([]ratPoint, len(points))
	for i, point := range points {
		u, okU := ratOf(point.U)
		v, okV := ratOf(point.V)
		if !okU || !okV {
			return nil, fmt.Errorf(`%w: control point %d is not finite`, ErrNotFinite, i)
		}
		out[i] = ratPoint{u: u, v: v}
	}
	return out, nil
}

// splineBezierSpans converts a SplineSeg — geom.Spline's clamped uniform cubic
// B-spline. Degree 3 and the knot vector geom.ClampedKnots(n) builds are the
// entity's DEFINITION rather than recorded data (seam §2), so the degree is
// restated here and the knot vector is READ FROM geom: its interior knots are
// float64(j)/float64(n−3), and those floats — not the exact rationals they round
// — are the curve sketch defines.
func splineBezierSpans(seg SplineSeg, work *freeformWork) ([]bezierSpan, error) {
	const degree = 3
	if err := requireFullFreeformRange(seg.TStart, seg.TEnd, "spline segment"); err != nil {
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
	if err := work.step(costAdd(
		rationalLiftCost(len(seg.Control), knots, 0),
		clampedConversionCost(len(seg.Control), knots, uniformKnotDemand(len(seg.Control), degree)),
	)); err != nil {
		return nil, err
	}
	ctrl, err := ratPointsOf(seg.Control)
	if err != nil {
		return nil, err
	}
	return clampedBezierSpans(degree, ctrl, clampedUniformKnots(len(ctrl)))
}

// clampedUniformKnots is geom.ClampedKnots's OWN float knot vector lifted
// exactly into rationals — sketch's answer taken as it stands, never re-derived.
//
// The distinction is load-bearing rather than cosmetic. geom builds each interior
// knot as float64(j)/float64(n−3), so for six control points it holds the
// roundings of 1/3 and 2/3, not those rationals: the two differ by
// 1/54043195528445952 and 1/27021597764222976. Rebuilding the vector from the
// closed form therefore converts a curve sketch does not define, and the error is
// not confined to the bound — the exact rational area over the closed-form knots
// is often representable while the area over sketch's own knots is not, so such a
// conversion publishes a false zero bound and an Exact claim, and on a
// near-cancelling section it moves the published magnitude too.
//
// Every knot geom returns is 0, 1 or a quotient of finite floats, so each lifts.
func clampedUniformKnots(n int) []*big.Rat {
	floats := geom.ClampedKnots(n)
	knots := make([]*big.Rat, len(floats))
	for i, knot := range floats {
		knots[i] = mustRatOf(knot)
	}
	return knots
}

// nurbsBezierSpans converts a NURBSSeg. Only a NON-RATIONAL one — every weight
// equal — is Tier A: its spans are polynomial Béziers and its moments integrate
// exactly. A genuinely rational NURBS is Tier C and refuses here, because
// converting it to a polynomial Bézier would integrate a DIFFERENT curve and
// report the result as exact.
func nurbsBezierSpans(seg NURBSSeg, work *freeformWork) ([]bezierSpan, error) {
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
	// which is precisely what freeformWorkLimit exists to stop (the public
	// ProfileRecord methods take no context). Levying the lift charge first bounds
	// that scan under the same ceiling, under chargeRationalLift's own invariant:
	// every pass between that charge and the conversion charge below — the content
	// checks, the tier test and floatKnotDemand — is a single walk over one array
	// whose length the charge counts.
	//
	// What it costs is small and worth stating. A record whose SIZE alone fits the
	// ceiling — every record that could ever yield a measurement — still reads its
	// own Tier C reason below. Only a record too large for the lift charge loses it,
	// and such a record is refused either way, so R7 is equally true of it.
	if err := requireFullFreeformRange(seg.TStart, seg.TEnd, "NURBS segment"); err != nil {
		return nil, err
	}
	if err := validateNURBSSegmentSizes(seg); err != nil {
		return nil, err
	}
	// The rational lift is the linear floor under everything below: the content
	// scan, the tier test and the conversion all walk the same arrays it counts.
	if err := chargeRationalLift(work, len(seg.Control), len(seg.Knots), len(seg.Weights)); err != nil {
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
	if err := work.step(clampedConversionCost(
		len(seg.Control),
		len(seg.Knots),
		floatKnotDemand(seg.Degree, len(seg.Control), seg.Knots),
	)); err != nil {
		return nil, err
	}
	ctrl, err := ratPointsOf(seg.Control)
	if err != nil {
		return nil, err
	}
	knots := make([]*big.Rat, len(seg.Knots))
	for i, knot := range seg.Knots {
		rat, ok := ratOf(knot)
		if !ok {
			return nil, fmt.Errorf(`%w: NURBS knot %d is not finite`, ErrNotFinite, i)
		}
		knots[i] = rat
	}
	return clampedBezierSpans(seg.Degree, ctrl, knots)
}

// closedSplineBezierSpans converts a ClosedSplineSeg — geom.ClosedSpline's
// periodic uniform cubic B-spline. Its definition is per-span rather than
// through a knot vector: span i blends the four cyclic controls P[i..i+3] with
// the standard uniform cubic basis, so it converts by the closed-form uniform
// B-spline to Bézier identity and needs no knot insertion. n control points
// give n spans, which is what closes the loop.
func closedSplineBezierSpans(seg ClosedSplineSeg, work *freeformWork) ([]bezierSpan, error) {
	if err := requireFullFreeformRange(seg.TStart, seg.TEnd, "closed spline segment"); err != nil {
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
	if err := work.step(costAdd(
		rationalLiftCost(len(seg.Control), 0, 0),
		costMul(4, uint64(len(seg.Control))),
	)); err != nil {
		return nil, err
	}
	ctrl, err := ratPointsOf(seg.Control)
	if err != nil {
		return nil, err
	}
	n := len(ctrl)
	spans := make([]bezierSpan, n)
	for i := range n {
		// The four cyclic controls of span i, matching geom's own indexing.
		q0, q1 := ctrl[i], ctrl[(i+1)%n]
		q2, q3 := ctrl[(i+2)%n], ctrl[(i+3)%n]
		// The uniform cubic B-spline to Bézier identity, per coordinate:
		// B₀ = (Q₀+4Q₁+Q₂)/6, B₁ = (2Q₁+Q₂)/3, B₂ = (Q₁+2Q₂)/3,
		// B₃ = (Q₁+4Q₂+Q₃)/6.
		spans[i] = bezierSpan{
			ratWeighted([]ratPoint{q0, q1, q2}, []int64{1, 4, 1}, 6),
			ratWeighted([]ratPoint{q1, q2}, []int64{2, 1}, 3),
			ratWeighted([]ratPoint{q1, q2}, []int64{1, 2}, 3),
			ratWeighted([]ratPoint{q1, q2, q3}, []int64{1, 4, 1}, 6),
		}
	}
	return spans, nil
}

// ratWeighted returns (Σ wᵢ·pᵢ)/den exactly. Callers pass equal-length slices.
func ratWeighted(points []ratPoint, weights []int64, den int64) ratPoint {
	axis := func(get func(ratPoint) *big.Rat) *big.Rat {
		out := new(big.Rat)
		for i, point := range points {
			out.Add(out, new(big.Rat).Mul(get(point), big.NewRat(weights[i], 1)))
		}
		return out.Quo(out, big.NewRat(den, 1))
	}
	return ratPoint{
		u: axis(func(p ratPoint) *big.Rat { return p.u }),
		v: axis(func(p ratPoint) *big.Rat { return p.v }),
	}
}

// clampedBezierSpans extracts the Bézier form of a clamped polynomial B-spline
// by Boehm knot insertion (docs/spline-design.md §5.1). Every interior knot is
// raised to multiplicity degree; the control points then split into consecutive
// blocks of degree+1 that share their boundary values, which is exactly the
// per-span Bézier form. bezierSliceCount proves that shape holds before a
// single block is cut, so a knot vector the insertion loop leaves outside it —
// one whose interior multiplicity already exceeded degree, so no insertion was
// owed — refuses instead of being sliced on a stride it does not have.
//
// The whole pass is charged by the CALLER, before it lifts a coordinate into a
// rational: every caller knows its own insertion demand from data it already
// holds as floats, and a charge levied here would land after the lift it is
// there to bound.
//
// Every arithmetic step is rational, so the extracted spans are the curve.
func clampedBezierSpans(degree int, ctrl []ratPoint, knots []*big.Rat) ([]bezierSpan, error) {
	if degree < 1 {
		return nil, fmt.Errorf(`%w: a B-spline degree must be at least 1, got %d`, ErrDegenerate, degree)
	}
	if want := len(ctrl) + degree + 1; len(knots) != want {
		return nil, fmt.Errorf(
			`%w: a degree-%d B-spline over %d control points needs %d knots, got %d`,
			ErrDegenerate, degree, len(ctrl), want, len(knots),
		)
	}
	targets, _, _ := interiorKnotRuns(degree, len(ctrl), knots)
	for _, target := range targets {
		for knotMultiplicity(knots, target) < degree {
			inserted, insertedKnots, err := insertKnot(degree, ctrl, knots, target)
			if err != nil {
				return nil, err
			}
			ctrl, knots = inserted, insertedKnots
		}
	}
	count, err := bezierSliceCount(degree, ctrl, knots)
	if err != nil {
		return nil, err
	}
	spans := make([]bezierSpan, count)
	for j := range count {
		span := make(bezierSpan, degree+1)
		copy(span, ctrl[j*degree:j*degree+degree+1])
		spans[j] = span
	}
	return spans, nil
}

// bezierSliceCount establishes the precondition the stride-degree slicing above
// rests on, and returns the number of spans it may cut.
//
// The slicing reads control points [j*degree, j*degree+degree], so consecutive
// spans SHARE their boundary control point. That holds only while every interior
// knot sits at multiplicity EXACTLY degree and the control count divides into
// whole spans. A record that misses either shape is refused here rather than
// sliced across a stride it does not have.
//
// The SENTINEL is not the same for every miss, and the difference is a fact
// about the recorded curve rather than about this slicer. At an interior
// multiplicity m ≥ degree+1 the curve's two one-sided limits at that knot are
// exactly two recorded control points — for the knot occupying indices j+1..j+m
// they are P_j and P_{j+m−degree} — so continuity there is decided SOLELY by
// whether those two coordinates are identical, and weights never enter. When
// they DIFFER the curve genuinely breaks apart, the record states several
// disjoint pieces rather than one boundary curve, and no such body exists:
// [ErrDegenerate]. When they are IDENTICAL the curve is continuous and the body
// does exist — this evaluator simply cannot slice a stride whose spans share no
// boundary control point, which is a limitation of the evaluator and so
// [ErrUnsupported]. Equality is exact identity on the recorded coordinates,
// never a tolerance: both directions are exactly decidable over rationals, which
// is what the falsify-never-bless rule requires.
//
// The structural misses below carry the same reading. A control count that is
// not a whole number of spans is reachable from a record record.go admits — a
// knot vector over-clamped past degree+1 at an end, whose extra repeat leaves a
// dead control point and no discontinuity anywhere — so it is this evaluator's
// own stride precondition failing on a curve that exists: [ErrUnsupported].
func bezierSliceCount(degree int, ctrl []ratPoint, knots []*big.Rat) (int, error) {
	values, runs, starts := interiorKnotRuns(degree, len(ctrl), knots)
	for i, run := range runs {
		if run == degree {
			continue
		}
		if run > degree && brokenKnot(degree, ctrl, starts[i], run) {
			return 0, fmt.Errorf(
				`%w: interior knot %d repeats %d times at degree %d and its two one-sided limits are different control points, so the recorded curve breaks into disjoint pieces`,
				ErrDegenerate, i, run, degree,
			)
		}
		return 0, fmt.Errorf(
			`%w: interior knot %d sits at multiplicity %d rather than %d, so consecutive Bézier spans share no boundary control point`,
			ErrUnsupported, i, run, degree,
		)
	}
	if (len(ctrl)-1)%degree != 0 {
		return 0, fmt.Errorf(
			`%w: knot insertion left %d control points, which is not a whole number of degree-%d Bézier spans`,
			ErrUnsupported, len(ctrl), degree,
		)
	}
	count := (len(ctrl) - 1) / degree
	if count == 0 {
		return 0, fmt.Errorf(`%w: a B-spline with an empty knot domain bounds no curve`, ErrDegenerate)
	}
	if len(values) != count-1 {
		return 0, fmt.Errorf(
			`%w: a degree-%d B-spline over %d Bézier spans needs %d interior knots, got %d`,
			ErrUnsupported, degree, count, count-1, len(values),
		)
	}
	return count, nil
}

// brokenKnot reports whether the curve is discontinuous at an interior knot of
// multiplicity run beginning at knot index start. The two one-sided limits at a
// knot occupying indices j+1..j+m are the recorded control points P_j and
// P_{j+m−degree}; the curve breaks apart exactly when those two coordinates
// differ, which is an exact comparison over rationals.
func brokenKnot(degree int, ctrl []ratPoint, start, run int) bool {
	left, right := start-1, start-1+run-degree
	if left < 0 || right < 0 || left >= len(ctrl) || right >= len(ctrl) {
		// No pair of recorded limits to compare, so nothing is proven broken.
		return false
	}
	return ctrl[left].u.Cmp(ctrl[right].u) != 0 || ctrl[left].v.Cmp(ctrl[right].v) != 0
}

// interiorKnotRuns returns each distinct knot strictly inside the clamped
// domain — the values insertion must raise — beside the length of its
// contiguous run and the index that run starts at. The run is a SUBSET of the
// value's whole-vector multiplicity, so treating it as that multiplicity can
// only OVERSTATE the insertions still owed, which is what clampedConversionCost
// needs to stay an upper bound. The insertion loop itself reads
// knotMultiplicity, so an unsorted vector costs a wider estimate and never a
// wrong span.
func interiorKnotRuns(degree, n int, knots []*big.Rat) ([]*big.Rat, []int, []int) {
	lo, hi := knots[degree], knots[n]
	var values []*big.Rat
	var runs []int
	var starts []int
	for offset, knot := range knots[degree+1 : n] {
		if knot.Cmp(lo) <= 0 || knot.Cmp(hi) >= 0 {
			continue
		}
		if len(values) > 0 && values[len(values)-1].Cmp(knot) == 0 {
			runs[len(runs)-1]++
			continue
		}
		values = append(values, knot)
		runs = append(runs, 1)
		starts = append(starts, degree+1+offset)
	}
	return values, runs, starts
}

// knotInsertionDemand is what the insertion pass will owe a knot vector: the
// total single-knot insertions and the number of distinct interior targets it
// probes. It is stated separately from the vector itself because every caller
// can derive it WITHOUT lifting a knot into a rational, which is what lets the
// conversion be charged before it allocates.
type knotInsertionDemand struct {
	insertions uint64
	targets    uint64
}

func (d *knotInsertionDemand) add(degree, run int) {
	d.targets++
	if run < degree {
		d.insertions = costAdd(d.insertions, uint64(degree-run))
	}
}

// uniformKnotDemand is the demand of geom.ClampedKnots(n) at the given degree,
// read from the control count WITHOUT asking geom for the vector: it holds
// n−degree−1 interior knots, each at multiplicity one, so each owes degree−1
// insertions. Being a pure function of the control count is exactly what lets a
// SplineSeg charge its whole conversion before it builds or lifts a single knot.
//
// It stays an UPPER bound on the vector geom actually returns. Those interior
// knots are the floats float64(j)/float64(n−degree), and counting each as its own
// multiplicity-one target can only OVERSTATE what the insertion pass owes: were
// two of them to round to the same float, they would form one longer run needing
// fewer insertions and one fewer probe target.
func uniformKnotDemand(n, degree int) knotInsertionDemand {
	if n <= degree+1 || degree < 1 {
		return knotInsertionDemand{}
	}
	targets := uint64(n - degree - 1)
	return knotInsertionDemand{
		insertions: costMul(targets, uint64(degree-1)),
		targets:    targets,
	}
}

// floatKnotDemand reads the demand off the RECORDED float knots, so a NURBS
// record can be charged for its conversion before it allocates one rational. It
// runs the same contiguous-run walk interiorKnotRuns does; its callers have
// already proven the vector finite and non-decreasing, so the runs it counts are
// the multiplicities the lifted rational vector holds.
func floatKnotDemand(degree, n int, knots []float64) knotInsertionDemand {
	lo, hi := knots[degree], knots[n]
	var demand knotInsertionDemand
	run, previous := 0, 0.0
	for _, knot := range knots[degree+1 : n] {
		if knot <= lo || knot >= hi {
			continue
		}
		if run > 0 && knot == previous {
			run++
			continue
		}
		if run > 0 {
			demand.add(degree, run)
		}
		previous, run = knot, 1
	}
	if run > 0 {
		demand.add(degree, run)
	}
	return demand
}

// clampedConversionCost is the conservative preflight of the whole knot
// insertion pass, charged before a single control point is allocated.
//
// The charge is the work the pass actually pays, not the number of insertions:
// one insertKnot scans every control point looking for the span and then COPIES
// both vectors, and the loop's own knotMultiplicity condition rescans the knot
// vector once per attempt. So an insertion costs the length of the vectors it
// touches, and since each insertion lengthens both by one, the FINAL lengths
// bound every insertion in the pass. Quadratic total work therefore charges
// quadratically, which is what keeps a hundred-thousand-control degree-3
// record refusing at the ceiling instead of running for hours inside it.
//
// Every knotMultiplicity PROBE is charged, not only the ones that go on to
// insert. The loop probes each target once more than it inserts at it, and a
// record already at degree multiplicity everywhere owes no insertion at all yet
// still pays one full knot-vector scan per target — quadratic work a charge
// counting insertions alone reads as nothing (a degree-1 record with thousands
// of distinct interior knots is that shape exactly).
func clampedConversionCost(controls, knots int, demand knotInsertionDemand) uint64 {
	finalControls := costAdd(uint64(controls), demand.insertions)
	finalKnots := costAdd(uint64(knots), demand.insertions)
	perInsertion := costAdd(costMul(2, finalControls), costMul(3, finalKnots))
	probes := costMul(demand.targets, finalKnots)
	// The interiorKnotRuns pass itself walks the knot vector once.
	return costAdd(costAdd(costMul(demand.insertions, perInsertion), probes), uint64(knots))
}

func knotMultiplicity(knots []*big.Rat, target *big.Rat) int {
	count := 0
	for _, knot := range knots {
		if knot.Cmp(target) == 0 {
			count++
		}
	}
	return count
}

// insertKnot inserts target once by Boehm's algorithm. The three control-point
// ranges are the standard ones: unchanged below the affected window, a rational
// convex blend inside it, and shifted above it.
func insertKnot(degree int, ctrl []ratPoint, knots []*big.Rat, target *big.Rat) ([]ratPoint, []*big.Rat, error) {
	span := -1
	for i := degree; i < len(ctrl); i++ {
		if knots[i].Cmp(target) <= 0 && target.Cmp(knots[i+1]) < 0 {
			span = i
		}
	}
	if span < 0 {
		return nil, nil, fmt.Errorf(`%w: a knot to insert lies outside the B-spline's own domain`, ErrDegenerate)
	}
	multiplicity := knotMultiplicity(knots, target)

	out := make([]ratPoint, len(ctrl)+1)
	for i := 0; i <= span-degree; i++ {
		out[i] = ctrl[i]
	}
	for i := span - degree + 1; i <= span-multiplicity; i++ {
		denominator := new(big.Rat).Sub(knots[i+degree], knots[i])
		if denominator.Sign() == 0 {
			return nil, nil, fmt.Errorf(`%w: a B-spline knot window has zero width`, ErrDegenerate)
		}
		alpha := new(big.Rat).Quo(new(big.Rat).Sub(target, knots[i]), denominator)
		beta := new(big.Rat).Sub(big.NewRat(1, 1), alpha)
		out[i] = ratPoint{
			u: new(big.Rat).Add(new(big.Rat).Mul(alpha, ctrl[i].u), new(big.Rat).Mul(beta, ctrl[i-1].u)),
			v: new(big.Rat).Add(new(big.Rat).Mul(alpha, ctrl[i].v), new(big.Rat).Mul(beta, ctrl[i-1].v)),
		}
	}
	for i := span - multiplicity + 1; i < len(out); i++ {
		out[i] = ctrl[i-1]
	}

	outKnots := make([]*big.Rat, 0, len(knots)+1)
	outKnots = append(outKnots, knots[:span+1]...)
	outKnots = append(outKnots, target)
	outKnots = append(outKnots, knots[span+1:]...)
	return out, outKnots, nil
}

// The chord counts sketch's own reconstruction sampler produces, restated from
// geom/arrange.go's sampleParams. They are restated rather than read because the
// reconstruction charge below has to be levied before that sampler runs and no
// recorded field reports them: a free-form source is chorded
// freeformChordsPerControl times per control point with freeformChordFloor as
// its floor, and a curved analytic source analyticChordsPerTurn times per full
// turn. They are sketch's numbers, so a change upstream is a change here — and
// they are FLOORED, which is why a record of many tiny curves arranges far more
// chords than its control count suggests.
const (
	freeformChordsPerControl = 16
	freeformChordFloor       = 64
	analyticChordsPerTurn    = 256
)

// freeformReconstruction is the whole-RECORD model of the sketch reconstruction
// momentRecordMatchesSketch runs (moments_validate.go) — the pass that asks
// sketch to decide the recorded region's topology.
//
// That pass is neither cheap nor cancellable, and its cost belongs to the record
// rather than to any one segment. sketch chords every source it is given and
// then ARRANGES THE WHOLE SCENE AT ONCE: geom's arranger tests every PAIR of
// chords in one global i<j loop over every chord of every source. So the charge
// is a quadratic in the record-wide chord TOTAL, and a charge summing per-source
// squares drops every cross-source pair — which is nearly all of them once a
// record holds more than one curve.
//
// Analytic sources are counted too. The arrangement is global, so a chord total
// that skips the lines, arcs and circles beside a spline bounds nothing about
// the pass those sources are arranged in.
//
// The pass also runs the arrangement MORE THAN ONCE. It arranges the scene to
// list the candidate profiles, RecordProfile arranges it again for each
// candidate it authenticates (Sketch.Profiles rebuilds the arrangement on every
// call), and validateMomentRecord repeats the whole pass on a rescaled record.
// The cost is therefore one arrangement's quadratic times the number of
// arrangements, which is what makes an uncharged record CUBIC in its source
// count rather than quadratic.
//
// The charge is split so that every arrangement is paid for before it happens:
// the two whole-scene arrangements the validation always runs are levied once,
// at the record-level preflight, ahead of the first reconstruction, and each
// candidate profile's own re-arrangement is charged on the same record counter
// immediately before it runs.
type freeformReconstruction struct {
	// chords is the record-wide chord total the arrangement will hold.
	chords uint64
	// arrangement is one whole-scene arrangement's charge: the ORDERED pair
	// count, twice the i<j pairs the intersection loop runs, so the doubling
	// stands for the rest of the pass — the vertex table, the chord splitting
	// and the region walk — rather than being modelled separately.
	arrangement uint64
}

// reconstructionOf reads that model off a checked record. Every segment is
// counted, whatever its kind: the chord total is the arrangement's own element
// count, and the arrangement is global.
func reconstructionOf(record ProfileRecord) freeformReconstruction {
	var chords uint64
	for _, loop := range append([]LoopRecord{record.Outer}, record.Holes...) {
		for _, segment := range loop.Segments {
			chords = costAdd(chords, reconstructionChords(segment))
		}
	}
	return freeformReconstruction{chords: chords, arrangement: costMul(chords, chords)}
}

// chargeFreeformReconstruction levies the record-level part of that charge — the
// scene arrangement the validation runs to list its candidate profiles, and the
// one its rescaled retry runs — and returns the per-arrangement charge the
// candidate loop then levies for itself.
func chargeFreeformReconstruction(record ProfileRecord, work *freeformWork) (uint64, error) {
	demand := reconstructionOf(record)
	if err := work.step(costMul(2, demand.arrangement)); err != nil {
		return 0, err
	}
	return demand.arrangement, nil
}

// reconstructionChords is how many polyline chords sketch will create for the
// entity one recorded segment names. It is sampleParams's own table, keyed on
// the kind: a line is its own single chord, a free-form curve is chorded per
// control point over a floor, and everything curved and analytic is chorded per
// turn.
func reconstructionChords(segment CurveSegment) uint64 {
	switch segment := segment.(type) {
	case LineSeg:
		return 1
	case ArcSeg:
		return arcChords(segment)
	case SplineSeg:
		// An open cubic B-spline is sampled per SPAN, so the count comes off the
		// control count less the degree — the one free-form kind whose sample
		// count is not simply its control count.
		return freeformChords(len(segment.Control) - 3)
	case ClosedSplineSeg:
		return freeformChords(len(segment.Control))
	case NURBSSeg:
		return freeformChords(len(segment.Control))
	case FitSplineSeg:
		return freeformChords(len(segment.Fit))
	default:
		// A circle, an ellipse, a conic and an elliptical arc all reach the same
		// per-turn branch. A whole turn is its own upper bound, and a partial
		// sweep is charged for a whole one rather than re-deriving a sweep the
		// record states as no field of its own.
		return analyticChordsPerTurn
	}
}

// freeformChords is the per-control sample count with sketch's own floor. The
// floor is what makes a record of many three-control splines expensive: each one
// arranges freeformChordFloor chords however few controls it holds.
func freeformChords(count int) uint64 {
	if count <= 0 {
		return freeformChordFloor
	}
	chords := costMul(freeformChordsPerControl, uint64(count))
	if chords < freeformChordFloor {
		return freeformChordFloor
	}
	return chords
}

// arcChords is the per-turn count scaled by the arc's own sweep — an ArcSeg is
// swept counter-clockwise from Start to End about Center, which is the sweep
// sketch derives from the same three pinned points. A sweep that is not a
// positive angle inside one turn is charged a whole turn, which the sampler
// itself cannot exceed.
func arcChords(seg ArcSeg) uint64 {
	sweep := math.Atan2(seg.End.V-seg.Center.V, seg.End.U-seg.Center.U) -
		math.Atan2(seg.Start.V-seg.Center.V, seg.Start.U-seg.Center.U)
	if sweep <= 0 {
		sweep += 2 * math.Pi
	}
	if !(sweep > 0) || sweep > 2*math.Pi {
		return analyticChordsPerTurn
	}
	chords := uint64(math.Ceil(analyticChordsPerTurn * sweep / (2 * math.Pi)))
	if chords < 2 {
		return 2
	}
	return chords
}

// chargeFreeformShift preflights the re-anchoring of a whole converted chain:
// two rational subtractions per control point of every span. It is levied at
// the record-level preflight beside the conversion and integration charges
// (moments_validate.go), never where the shift itself runs — a charge levied at
// the point of use lands after the sketch reconstruction the ceiling exists to
// precede, so a chain that fits the budget everywhere except its re-anchoring
// would run minutes of uncancellable work before refusing.
func chargeFreeformShift(spans []bezierSpan, work *freeformWork) error {
	for _, span := range spans {
		if err := work.step(costMul(2, uint64(len(span)))); err != nil {
			return err
		}
	}
	return nil
}

// shiftFreeformSpans re-references a converted chain to the walk anchor over
// EXACT rationals, in place. Its cost is charged by chargeFreeformShift at the
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
func shiftFreeformSpans(spans []bezierSpan, anchor Point2) error {
	u, okU := ratOf(anchor.U)
	v, okV := ratOf(anchor.V)
	if !okU || !okV {
		return fmt.Errorf(`%w: a free-form walk's anchor is not finite`, ErrNotFinite)
	}
	for _, span := range spans {
		for i, point := range span {
			span[i] = ratPoint{
				u: new(big.Rat).Sub(point.u, u),
				v: new(big.Rat).Sub(point.v, v),
			}
		}
	}
	return nil
}

// freeformEndpoints returns the converted chain's own endpoints in the recorded
// walk order — the first and last Bézier control point, which a Bézier
// interpolates exactly, so these are the curve's endpoints and not samples.
func freeformEndpoints(spans []bezierSpan, reversed bool) (Point2, Point2, error) {
	if len(spans) == 0 || len(spans[0]) == 0 || len(spans[len(spans)-1]) == 0 {
		return Point2{}, Point2{}, fmt.Errorf(`%w: a converted free-form curve holds no span`, ErrDegenerate)
	}
	first := spans[0][0]
	last := spans[len(spans)-1][len(spans[len(spans)-1])-1]
	if reversed {
		first, last = last, first
	}
	start, okStart := point2Of(first)
	end, okEnd := point2Of(last)
	if !okStart || !okEnd {
		return Point2{}, Point2{}, fmt.Errorf(`%w: a converted free-form endpoint is not representable`, ErrNotFinite)
	}
	return start, end, nil
}

func point2Of(p ratPoint) (Point2, bool) {
	u, _ := p.u.Float64()
	v, _ := p.v.Float64()
	if isNonFinite(u) || isNonFinite(v) {
		return Point2{}, false
	}
	return Point2{U: u, V: v}, true
}

// freeformEndTangents returns the walk's tangent directions at its start and
// end. A Bézier's derivative at an end is degree·(the adjacent control leg), so
// these are exact directions read off the control net — no sampling, and no
// normalization (a segmentWalk tangent is a direction, not a unit vector).
//
// A reversed walk enters where the curve leaves, so both the order and the sign
// of the two legs flip.
func freeformEndTangents(spans []bezierSpan, reversed bool) (float64, float64, float64, float64, error) {
	if len(spans) == 0 {
		return 0, 0, 0, 0, fmt.Errorf(`%w: a converted free-form curve holds no span`, ErrDegenerate)
	}
	first, last := spans[0], spans[len(spans)-1]
	if len(first) < 2 || len(last) < 2 {
		return 0, 0, 0, 0, fmt.Errorf(`%w: a converted free-form span holds fewer than two control points`, ErrDegenerate)
	}
	leg := func(from, to ratPoint, degree int) (float64, float64, bool) {
		scale := big.NewRat(int64(degree), 1)
		du := new(big.Rat).Mul(scale, new(big.Rat).Sub(to.u, from.u))
		dv := new(big.Rat).Mul(scale, new(big.Rat).Sub(to.v, from.v))
		u, _ := du.Float64()
		v, _ := dv.Float64()
		if isNonFinite(u) || isNonFinite(v) {
			return 0, 0, false
		}
		return u, v, true
	}
	inU, inV, okIn := leg(first[0], first[1], len(first)-1)
	outU, outV, okOut := leg(last[len(last)-2], last[len(last)-1], len(last)-1)
	if !okIn || !okOut {
		return 0, 0, 0, 0, fmt.Errorf(`%w: a free-form end tangent is not representable`, ErrNotFinite)
	}
	if reversed {
		return -outU, -outV, -inU, -inV, nil
	}
	return inU, inV, outU, outV, nil
}

// freeformControlExtent is an upper envelope on |u|+|v| over the curve, read
// off the control points. The convex hull property makes it a PROVEN envelope
// for the curve itself, not just for its control net.
func freeformControlExtent(spans []bezierSpan) float64 {
	extent := 0.0
	for _, span := range spans {
		for _, point := range span {
			u, _ := new(big.Rat).Abs(point.u).Float64()
			v, _ := new(big.Rat).Abs(point.v).Float64()
			if sum := absSumUpper(u, v); sum > extent {
				extent = sum
			}
		}
	}
	return extent
}
