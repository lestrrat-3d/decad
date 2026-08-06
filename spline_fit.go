package decad

import (
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/sketch/geom"
)

// This file is docs/spline-design.md §5.1.2's fit-spline reduction: a
// recorded FitSplineSeg is Tier A for the moments path (Table F) by consuming
// sketch's own EXPORTED interpolant — never by re-running the interpolation
// solve, which seam §2 and record.go's own FitSplineSeg doc comment forbid.
// Table R row R6 states only the BUILD-path refusal; nothing here retires it.
//
// It consumes geom.FitInterpolant's Params/Points/SecondDerivs triple, NEVER
// its Spans() monomial form. FitSpan's own doc comment states why: its
// coefficients are computed with two or three float roundings apiece
// (cubicSpanCoeffs), so the polynomial Spans() publishes is a DIFFERENT
// polynomial from the one FitInterpolant.Eval walks — displaced by an amount
// sketch states no bound on. Integrating that displaced polynomial exactly
// would publish a possibly-zero bound, hence a possible Exact, for a curve
// nobody defined. Params/Points/SecondDerivs carry the standing every other
// Tier A kind's defining data has: they are sketch's own computed floats,
// taken exactly, and the arithmetic FitInterpolant's own doc states over them
// — v(p) = a·v[i] + b·v[i+1] + ((a³−a)·m[i] + (b³−b)·m[i+1])·h²/6 — is the
// curve's definition, term for term the same arrangement evalCubicSpan
// computes. Spans() still earns its keep as an independent oracle for testing
// this conversion (spline_fit_internal_test.go), never as its input.
//
// geom.NewFitInterpolant COLLAPSES consecutive fit points closer than an
// absolute 1e-12, keeping the FIRST of each run (geom/fitspline.go's
// fitChordEps). So the chain's endpoints are Points[0] and Points[len-1], and
// the LAST one is not always Fit[len(Fit)-1]: a record whose last two fit
// points coincide within that threshold has a curve whose true end is the
// second-to-last recorded point. decad integrates exactly the curve
// FitSpline.Eval walks — Points, not Fit — which is the right answer; nothing
// here papers over the difference or adds a second threshold.
func fitSplineBezierSpans(seg FitSplineSeg, work *freeformWork) ([]bezierSpan, error) {
	if err := requireFullFreeformRange(seg.TStart, seg.TEnd, "fit spline segment"); err != nil {
		return nil, err
	}
	// An O(1) size refusal ahead of any content read. record.go's own validation
	// already floors Fit at 2 (validateSegmentPoints), so this is the guard for a
	// caller-built or hand-decoded record that bypassed that seam.
	if len(seg.Fit) < 2 {
		return nil, fmt.Errorf(
			`%w: a fit spline needs at least 2 fit points, got %d`,
			ErrDegenerate, len(seg.Fit),
		)
	}
	// Charged BEFORE geom.NewFitInterpolant allocates anything: the charge reads
	// nothing but the recorded slice's own length, exactly the discipline
	// chargeRationalLift and the other Tier A conversions already keep.
	if err := chargeFitInterpolant(work, len(seg.Fit)); err != nil {
		return nil, err
	}
	// A non-finite recorded fit coordinate is a non-finite INPUT (core §12), and
	// this is the ONLY gate that catches it for a caller-built record: record.go's
	// validateSegmentPoints runs at JSON decode, never for a ProfileRecord a
	// caller assembles directly in Go and hands straight to Area/Centroid/
	// SecondMoments. Scanning here — after the size charge, before
	// geom.NewFitInterpolant reads a single coordinate — keeps R16 below scoped
	// to what it actually claims: a FINITE fit set whose interpolant itself runs
	// off float64 range. Every element of seg.Fit is checked, not just the
	// first, so a non-finite point in an interior or terminal position is
	// ErrNotFinite too, never sketch's own dedup or naturalSecondDerivs
	// answering it as an unrelated ErrDegenerate.
	for i, point := range seg.Fit {
		if finiteSegmentValue(point.U) && finiteSegmentValue(point.V) {
			continue
		}
		return nil, fmt.Errorf(`%w: fit spline point %d is not finite`, ErrNotFinite, i)
	}
	interp, err := geom.NewFitInterpolant(fitCoords(seg.Fit))
	if err != nil {
		if errors.Is(err, geom.ErrNonFiniteFitInterpolant) {
			// Table R row R16: the fit points are finite (the scan above, ahead of
			// record.go's own gate for a JSON-decoded record) and therefore define a
			// curve, but its cumulative chord parameter or a span coefficient runs
			// off float64 — a range limit of this evaluator, not a claim that no such
			// curve exists, and not a non-finite INPUT (every coordinate reaching the
			// solve is finite). §5.1.2 draws the same distinction R15 does:
			// R15 refuses an arc-length enclosure past MaxFloat64 the identical way.
			return nil, fmt.Errorf(
				`%w: a fit spline's interpolant runs off the float64 range and cannot be described, though the fit points are finite`,
				ErrUnsupported,
			)
		}
		// geom.ErrTooFewFitPoints is unreachable here: record.go's own validation
		// floors Fit at 2, and the length check above re-asserts it for a record
		// that bypassed that seam. Map defensively to ErrDegenerate rather than
		// leak sketch's own error type.
		return nil, fmt.Errorf(`%w: %s`, ErrDegenerate, err)
	}

	k := len(interp.Points)
	params := make([]*big.Rat, k)
	points := make([]ratPoint, k)
	seconds := make([]ratPoint, k)
	for i := range k {
		p, ok := ratOf(interp.Params[i])
		if !ok {
			return nil, fmt.Errorf(`%w: a fit spline's cumulative chord parameter is not finite`, ErrNotFinite)
		}
		params[i] = p
		u, okU := ratOf(interp.Points[i][0])
		v, okV := ratOf(interp.Points[i][1])
		if !okU || !okV {
			return nil, fmt.Errorf(`%w: a fit spline's active point is not finite`, ErrNotFinite)
		}
		points[i] = ratPoint{u: u, v: v}
		mu, okMU := ratOf(interp.SecondDerivs[i][0])
		mv, okMV := ratOf(interp.SecondDerivs[i][1])
		if !okMU || !okMV {
			return nil, fmt.Errorf(`%w: a fit spline's second derivative is not finite`, ErrNotFinite)
		}
		seconds[i] = ratPoint{u: mu, v: mv}
	}

	// k-1 spans over k active points; k == 1 (every fit point collapsed
	// together) yields zero spans, which freeformEndpoints and freeformDegenerate
	// refuse on their own terms as R14 — the same answer the identical record's
	// length bracket gives.
	spans := make([]bezierSpan, k-1)
	for i := range spans {
		h := new(big.Rat).Sub(params[i+1], params[i])
		hSq := new(big.Rat).Mul(h, h)
		b1u, b2u := fitSpanControls(points[i].u, points[i+1].u, seconds[i].u, seconds[i+1].u, hSq)
		b1v, b2v := fitSpanControls(points[i].v, points[i+1].v, seconds[i].v, seconds[i+1].v, hSq)
		spans[i] = bezierSpan{
			points[i],
			ratPoint{u: b1u, v: b1v},
			ratPoint{u: b2u, v: b2v},
			points[i+1],
		}
	}
	return spans, nil
}

// fitInterpolantCostPerPoint is the conservative per-fit-point charge behind
// fitInterpolantCost. Linear, with NO quadratic term — unlike a knot
// insertion's clampedConversionCost, a natural cubic interpolant gives one
// span per interval directly and there is no insertion pass to charge for.
// The ~40 units the interpolant solve, the rational lift and this file's own
// closed form actually cost (dedup + chord accumulation, the tridiagonal
// solve's two Thomas passes, the interpolant export, decad's rational lift of
// the three returned slices, and the per-span closed form) round up to 64 as
// the conservative figure, per §5.2's rule that the constant is backed by a
// measured boundary regression rather than by this accounting alone.
//
// This charge is a FLOOR, not the binding constraint: for a record holding a
// lone fit spline, the reconstruction charge (freeformChords, already keyed on
// FitSplineSeg in spline_bezier.go's reconstructionChords) dominates well
// before this one does.
const fitInterpolantCostPerPoint = 64

// fitInterpolantCost is the size-derived charge for building and lifting one
// fit spline's interpolant, read from the recorded fit-point count alone —
// before geom.NewFitInterpolant allocates anything.
func fitInterpolantCost(n int) uint64 {
	return costMul(fitInterpolantCostPerPoint, uint64(n))
}

// chargeFitInterpolant levies fitInterpolantCost against the record's counter.
func chargeFitInterpolant(work *freeformWork, n int) error {
	return work.step(fitInterpolantCost(n))
}

// fitCoords restates the recorded fit points in the [][2]float64 shape
// geom.NewFitInterpolant takes. It copies nothing decad computes — every
// coordinate is the recorded float, taken as sketch's own solve reads it.
func fitCoords(points []Point2) [][2]float64 {
	out := make([][2]float64, len(points))
	for i, point := range points {
		out[i] = [2]float64{point.U, point.V}
	}
	return out
}

// fitSpanControls returns one coordinate's two INTERIOR Bézier control values
// for a fit-spline span, docs/spline-design.md §1.3's closed form:
//
//	b1 = (2·vi + vi1)/3 − hSq·(2·mi + mi1)/18
//	b2 = (vi + 2·vi1)/3 − hSq·(mi + 2·mi1)/18
//
// with vi/vi1 the span's endpoint values (its Bézier b0/b3, interpolated
// exactly), mi/mi1 the natural-cubic second derivatives at those same ends,
// and hSq the span width squared, all exact rationals. Every operand is a
// big.Rat, so nothing here rounds — cross-checked against the monomial route
// in spline_fit_internal_test.go, and against FitSpan's own independent
// conversion as an oracle.
func fitSpanControls(vi, vi1, mi, mi1, hSq *big.Rat) (b1, b2 *big.Rat) {
	three := big.NewRat(3, 1)
	eighteen := big.NewRat(18, 1)

	b1 = new(big.Rat).Add(new(big.Rat).Add(vi, vi), vi1)
	b1.Quo(b1, three)
	t1 := new(big.Rat).Add(new(big.Rat).Add(mi, mi), mi1)
	t1.Mul(t1, hSq)
	t1.Quo(t1, eighteen)
	b1.Sub(b1, t1)

	b2 = new(big.Rat).Add(vi, new(big.Rat).Add(vi1, vi1))
	b2.Quo(b2, three)
	t2 := new(big.Rat).Add(mi, new(big.Rat).Add(mi1, mi1))
	t2.Mul(t2, hSq)
	t2.Quo(t2, eighteen)
	b2.Sub(b2, t2)

	return b1, b2
}
