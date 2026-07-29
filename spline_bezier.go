package decad

import (
	"fmt"
	"math/big"
)

// This file is docs/spline-design.md §5.1's exact reduction: a recorded
// free-form curve becomes piecewise polynomial Bézier control points over
// math/big.Rat, with no rounding anywhere along the way. Control coordinates
// are floats, so they are exact rationals, and knot insertion is a rational
// convex combination — so the converted spans ARE the recorded curve, not an
// approximation of it, and every integral taken over them (spline_moments.go)
// inherits that exactness.
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

// freeformWorkLimit is the fixed ceiling on one free-form curve's conversion
// and integration work, in charged units (one knot insertion's control-point
// update, or one integrand coefficient product). Public ProfileRecord methods
// take no context, so the limit is fixed rather than caller-set, exactly as
// shellInradiusWorkLimit is for the inward shell survey. Reaching it is Table R
// row R7: ErrUnsupported, never a widened float path.
const freeformWorkLimit uint64 = 1 << 20

// freeformWork is the charged counter behind freeformWorkLimit.
type freeformWork struct{ spent uint64 }

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
// Every kind outside Tier A refuses with its own reason, so a caller never has
// to infer which table row it hit:
//
//   - FitSplineSeg is Table R row R6 — sketch owns the interpolation solve and
//     decad NEVER re-runs it (seam §2), so the curve's coefficients are not
//     available to convert.
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
		return nil, false, fmt.Errorf(
			`%w: a fit spline's curve is sketch's own interpolation solve, which decad never re-runs; its B-spline form is not available to integrate`,
			ErrUnsupported,
		)
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

// requireFullFreeformRange rejects a recorded free-form range that is not the
// entity's full domain. spline design §2 proves none is recordable, so reaching
// this is a caller-built or decoded record that bypassed the seam — refuse
// rather than integrate a piece the conversion does not cover.
func requireFullFreeformRange(tStart, tEnd float64, what string) error {
	if (tStart == 0 && tEnd == 1) || (tStart == 1 && tEnd == 0) {
		return nil
	}
	return fmt.Errorf(
		`%w: a %s must span its full domain; a trimmed free-form range is never recordable (range [%v, %v])`,
		ErrUnsupported, what, tStart, tEnd,
	)
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
// B-spline. Degree 3 and geom.ClampedKnots(n) are the entity's DEFINITION, not
// recorded data (seam §2), so they are restated here rather than read: four
// zeros, n−4 evenly spaced interior knots, four ones. Those interior knots are
// j/(n−3), exact rationals.
func splineBezierSpans(seg SplineSeg, work *freeformWork) ([]bezierSpan, error) {
	if err := requireFullFreeformRange(seg.TStart, seg.TEnd, "spline segment"); err != nil {
		return nil, err
	}
	ctrl, err := ratPointsOf(seg.Control)
	if err != nil {
		return nil, err
	}
	const degree = 3
	if len(ctrl) < degree+1 {
		return nil, fmt.Errorf(
			`%w: a cubic B-spline needs at least %d control points, got %d`,
			ErrDegenerate, degree+1, len(ctrl),
		)
	}
	return clampedBezierSpans(degree, ctrl, clampedUniformKnots(len(ctrl), degree), work)
}

// clampedUniformKnots is geom.ClampedKnots in exact rationals: degree+1
// repeats of 0, the interior knots j/spans, degree+1 repeats of 1.
func clampedUniformKnots(n, degree int) []*big.Rat {
	spans := int64(n - degree)
	knots := make([]*big.Rat, 0, n+degree+1)
	for range degree + 1 {
		knots = append(knots, new(big.Rat))
	}
	for j := int64(1); j < spans; j++ {
		knots = append(knots, big.NewRat(j, spans))
	}
	for range degree + 1 {
		knots = append(knots, big.NewRat(1, 1))
	}
	return knots
}

// nurbsBezierSpans converts a NURBSSeg. Only a NON-RATIONAL one — every weight
// equal — is Tier A: its spans are polynomial Béziers and its moments integrate
// exactly. A genuinely rational NURBS is Tier C and refuses here, because
// converting it to a polynomial Bézier would integrate a DIFFERENT curve and
// report the result as exact.
func nurbsBezierSpans(seg NURBSSeg, work *freeformWork) ([]bezierSpan, error) {
	if err := requireFullFreeformRange(seg.TStart, seg.TEnd, "NURBS segment"); err != nil {
		return nil, err
	}
	if err := validateNURBSSegment(seg); err != nil {
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
	return clampedBezierSpans(seg.Degree, ctrl, knots, work)
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
	ctrl, err := ratPointsOf(seg.Control)
	if err != nil {
		return nil, err
	}
	n := len(ctrl)
	if n < 3 {
		return nil, fmt.Errorf(
			`%w: a closed cubic B-spline needs at least 3 control points, got %d`,
			ErrDegenerate, n,
		)
	}
	if err := work.step(uint64(n) * 4); err != nil {
		return nil, err
	}
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
// per-span Bézier form.
//
// Every arithmetic step is rational, so the extracted spans are the curve.
func clampedBezierSpans(degree int, ctrl []ratPoint, knots []*big.Rat, work *freeformWork) ([]bezierSpan, error) {
	if degree < 1 {
		return nil, fmt.Errorf(`%w: a B-spline degree must be at least 1, got %d`, ErrDegenerate, degree)
	}
	if want := len(ctrl) + degree + 1; len(knots) != want {
		return nil, fmt.Errorf(
			`%w: a degree-%d B-spline over %d control points needs %d knots, got %d`,
			ErrDegenerate, degree, len(ctrl), want, len(knots),
		)
	}
	for _, target := range interiorKnotValues(degree, len(ctrl), knots) {
		for knotMultiplicity(knots, target) < degree {
			if err := work.step(uint64(degree + 1)); err != nil {
				return nil, err
			}
			inserted, insertedKnots, err := insertKnot(degree, ctrl, knots, target)
			if err != nil {
				return nil, err
			}
			ctrl, knots = inserted, insertedKnots
		}
	}
	// Every interior knot now sits at multiplicity degree, so span j owns
	// control points [j*degree, j*degree+degree].
	if (len(ctrl)-1)%degree != 0 {
		return nil, fmt.Errorf(
			`%w: knot insertion left %d control points, which is not a whole number of degree-%d Bézier spans`,
			ErrDegenerate, len(ctrl), degree,
		)
	}
	count := (len(ctrl) - 1) / degree
	if count == 0 {
		return nil, fmt.Errorf(`%w: a B-spline with an empty knot domain bounds no curve`, ErrDegenerate)
	}
	spans := make([]bezierSpan, count)
	for j := range count {
		span := make(bezierSpan, degree+1)
		copy(span, ctrl[j*degree:j*degree+degree+1])
		spans[j] = span
	}
	return spans, nil
}

// interiorKnotValues returns each distinct knot strictly inside the clamped
// domain, in ascending order — the values insertion must raise.
func interiorKnotValues(degree, n int, knots []*big.Rat) []*big.Rat {
	lo, hi := knots[degree], knots[n]
	var out []*big.Rat
	for _, knot := range knots[degree+1 : n] {
		if knot.Cmp(lo) <= 0 || knot.Cmp(hi) >= 0 {
			continue
		}
		if len(out) > 0 && out[len(out)-1].Cmp(knot) == 0 {
			continue
		}
		out = append(out, knot)
	}
	return out
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
