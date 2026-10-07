package freeform

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/polynomial"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/sketch/geom"
)

// RatPoint is a plane-local coordinate over exact rationals — Point2's exact
// counterpart, in millimetres by the same core §5.2 convention.
type RatPoint struct{ U, V *big.Rat }

// BezierSpan is one polynomial Bézier piece of a converted free-form curve:
// its control points in order, so its degree is len(BezierSpan)-1. A Bézier
// interpolates its first and last control point exactly, which is why
// consecutive spans join on a shared coordinate value and the chain's first and
// last control points ARE the recorded curve's own endpoints.
type BezierSpan []RatPoint

// ClampedUniformKnots is geom.ClampedKnots's OWN float knot vector lifted
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
func ClampedUniformKnots(n int) []*big.Rat {
	floats := geom.ClampedKnots(n)
	knots := make([]*big.Rat, len(floats))
	for i, knot := range floats {
		knots[i] = polynomial.MustRatOf(knot)
	}
	return knots
}

// RatWeighted returns (Σ wᵢ·pᵢ)/den exactly. Callers pass equal-length slices.
func RatWeighted(points []RatPoint, weights []int64, den int64) RatPoint {
	axis := func(get func(RatPoint) *big.Rat) *big.Rat {
		out := new(big.Rat)
		for i, point := range points {
			out.Add(out, new(big.Rat).Mul(get(point), big.NewRat(weights[i], 1)))
		}
		return out.Quo(out, big.NewRat(den, 1))
	}
	return RatPoint{
		U: axis(func(p RatPoint) *big.Rat { return p.U }),
		V: axis(func(p RatPoint) *big.Rat { return p.V }),
	}
}

// ClampedBezierSpans extracts the Bézier form of a clamped polynomial B-spline
// by Boehm knot insertion (docs/spline-design.md §5.1). Every interior knot is
// raised to multiplicity degree; the control points then split into consecutive
// blocks of degree+1 that share their boundary values, which is exactly the
// per-span Bézier form. BezierSliceCount proves that shape holds before a
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
func ClampedBezierSpans(degree int, ctrl []RatPoint, knots []*big.Rat) ([]BezierSpan, error) {
	if degree < 1 {
		return nil, fmt.Errorf(`%w: a B-spline degree must be at least 1, got %d`, decaderr.ErrDegenerate, degree)
	}
	if want := len(ctrl) + degree + 1; len(knots) != want {
		return nil, fmt.Errorf(
			`%w: a degree-%d B-spline over %d control points needs %d knots, got %d`,
			decaderr.ErrDegenerate, degree, len(ctrl), want, len(knots),
		)
	}
	targets, _, _ := InteriorKnotRuns(degree, len(ctrl), knots)
	for _, target := range targets {
		for KnotMultiplicity(knots, target) < degree {
			inserted, insertedKnots, err := InsertKnot(degree, ctrl, knots, target)
			if err != nil {
				return nil, err
			}
			ctrl, knots = inserted, insertedKnots
		}
	}
	count, err := BezierSliceCount(degree, ctrl, knots)
	if err != nil {
		return nil, err
	}
	spans := make([]BezierSpan, count)
	for j := range count {
		span := make(BezierSpan, degree+1)
		copy(span, ctrl[j*degree:j*degree+degree+1])
		spans[j] = span
	}
	return spans, nil
}

// BezierSliceCount establishes the precondition the stride-degree slicing above
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
func BezierSliceCount(degree int, ctrl []RatPoint, knots []*big.Rat) (int, error) {
	values, runs, starts := InteriorKnotRuns(degree, len(ctrl), knots)
	for i, run := range runs {
		if run == degree {
			continue
		}
		if run > degree && BrokenKnot(degree, ctrl, starts[i], run) {
			return 0, fmt.Errorf(
				`%w: interior knot %d repeats %d times at degree %d and its two one-sided limits are different control points, so the recorded curve breaks into disjoint pieces`,
				decaderr.ErrDegenerate, i, run, degree,
			)
		}
		return 0, fmt.Errorf(
			`%w: interior knot %d sits at multiplicity %d rather than %d, so consecutive Bézier spans share no boundary control point`,
			decaderr.ErrUnsupported, i, run, degree,
		)
	}
	if (len(ctrl)-1)%degree != 0 {
		return 0, fmt.Errorf(
			`%w: knot insertion left %d control points, which is not a whole number of degree-%d Bézier spans`,
			decaderr.ErrUnsupported, len(ctrl), degree,
		)
	}
	count := (len(ctrl) - 1) / degree
	if count == 0 {
		return 0, fmt.Errorf(`%w: a B-spline with an empty knot domain bounds no curve`, decaderr.ErrDegenerate)
	}
	if len(values) != count-1 {
		return 0, fmt.Errorf(
			`%w: a degree-%d B-spline over %d Bézier spans needs %d interior knots, got %d`,
			decaderr.ErrUnsupported, degree, count, count-1, len(values),
		)
	}
	return count, nil
}

// BrokenKnot reports whether the curve is discontinuous at an interior knot of
// multiplicity run beginning at knot index start. The two one-sided limits at a
// knot occupying indices j+1..j+m are the recorded control points P_j and
// P_{j+m−degree}; the curve breaks apart exactly when those two coordinates
// differ, which is an exact comparison over rationals.
func BrokenKnot(degree int, ctrl []RatPoint, start, run int) bool {
	left, right := start-1, start-1+run-degree
	if left < 0 || right < 0 || left >= len(ctrl) || right >= len(ctrl) {
		// No pair of recorded limits to compare, so nothing is proven broken.
		return false
	}
	return ctrl[left].U.Cmp(ctrl[right].U) != 0 || ctrl[left].V.Cmp(ctrl[right].V) != 0
}

// InteriorKnotRuns returns each distinct knot strictly inside the clamped
// domain — the values insertion must raise — beside the length of its
// contiguous run and the index that run starts at. The run is a SUBSET of the
// value's whole-vector multiplicity, so treating it as that multiplicity can
// only OVERSTATE the insertions still owed, which is what ClampedConversionCost
// needs to stay an upper bound. The insertion loop itself reads
// KnotMultiplicity, so an unsorted vector costs a wider estimate and never a
// wrong span.
func InteriorKnotRuns(degree, n int, knots []*big.Rat) ([]*big.Rat, []int, []int) {
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

// KnotInsertionDemand is what the insertion pass will owe a knot vector: the
// total single-knot insertions and the number of distinct interior targets it
// probes. It is stated separately from the vector itself because every caller
// can derive it WITHOUT lifting a knot into a rational, which is what lets the
// conversion be charged before it allocates.
type KnotInsertionDemand struct {
	Insertions uint64
	Targets    uint64
}

func (d *KnotInsertionDemand) Add(degree, run int) {
	d.Targets++
	if run < degree {
		d.Insertions = CostAdd(d.Insertions, uint64(degree-run))
	}
}

// UniformKnotDemand is the demand of geom.ClampedKnots(n) at the given degree,
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
func UniformKnotDemand(n, degree int) KnotInsertionDemand {
	if n <= degree+1 || degree < 1 {
		return KnotInsertionDemand{}
	}
	targets := uint64(n - degree - 1)
	return KnotInsertionDemand{
		Insertions: CostMul(targets, uint64(degree-1)),
		Targets:    targets,
	}
}

// FloatKnotDemand reads the demand off the RECORDED float knots, so a NURBS
// record can be charged for its conversion before it allocates one rational. It
// runs the same contiguous-run walk InteriorKnotRuns does; its callers have
// already proven the vector finite and non-decreasing, so the runs it counts are
// the multiplicities the lifted rational vector holds.
func FloatKnotDemand(degree, n int, knots []float64) KnotInsertionDemand {
	lo, hi := knots[degree], knots[n]
	var demand KnotInsertionDemand
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
			demand.Add(degree, run)
		}
		previous, run = knot, 1
	}
	if run > 0 {
		demand.Add(degree, run)
	}
	return demand
}

// ClampedConversionCost is the conservative preflight of the whole knot
// insertion pass, charged before a single control point is allocated.
//
// The charge is the work the pass actually pays, not the number of insertions:
// one InsertKnot scans every control point looking for the span and then COPIES
// both vectors, and the loop's own KnotMultiplicity condition rescans the knot
// vector once per attempt. So an insertion costs the length of the vectors it
// touches, and since each insertion lengthens both by one, the FINAL lengths
// bound every insertion in the pass. Quadratic total work therefore charges
// quadratically, which is what keeps a hundred-thousand-control degree-3
// record refusing at the ceiling instead of running for hours inside it.
//
// Every KnotMultiplicity PROBE is charged, not only the ones that go on to
// insert. The loop probes each target once more than it inserts at it, and a
// record already at degree multiplicity everywhere owes no insertion at all yet
// still pays one full knot-vector scan per target — quadratic work a charge
// counting insertions alone reads as nothing (a degree-1 record with thousands
// of distinct interior knots is that shape exactly).
func ClampedConversionCost(controls, knots int, demand KnotInsertionDemand) uint64 {
	finalControls := CostAdd(uint64(controls), demand.Insertions)
	finalKnots := CostAdd(uint64(knots), demand.Insertions)
	perInsertion := CostAdd(CostMul(2, finalControls), CostMul(3, finalKnots))
	probes := CostMul(demand.Targets, finalKnots)
	// The InteriorKnotRuns pass itself walks the knot vector once.
	return CostAdd(CostAdd(CostMul(demand.Insertions, perInsertion), probes), uint64(knots))
}

func KnotMultiplicity(knots []*big.Rat, target *big.Rat) int {
	count := 0
	for _, knot := range knots {
		if knot.Cmp(target) == 0 {
			count++
		}
	}
	return count
}

// InsertKnot inserts target once by Boehm's algorithm. The three control-point
// ranges are the standard ones: unchanged below the affected window, a rational
// convex blend inside it, and shifted above it.
func InsertKnot(degree int, ctrl []RatPoint, knots []*big.Rat, target *big.Rat) ([]RatPoint, []*big.Rat, error) {
	span := -1
	for i := degree; i < len(ctrl); i++ {
		if knots[i].Cmp(target) <= 0 && target.Cmp(knots[i+1]) < 0 {
			span = i
		}
	}
	if span < 0 {
		return nil, nil, fmt.Errorf(`%w: a knot to insert lies outside the B-spline's own domain`, decaderr.ErrDegenerate)
	}
	multiplicity := KnotMultiplicity(knots, target)

	out := make([]RatPoint, len(ctrl)+1)
	for i := 0; i <= span-degree; i++ {
		out[i] = ctrl[i]
	}
	for i := span - degree + 1; i <= span-multiplicity; i++ {
		denominator := new(big.Rat).Sub(knots[i+degree], knots[i])
		if denominator.Sign() == 0 {
			return nil, nil, fmt.Errorf(`%w: a B-spline knot window has zero width`, decaderr.ErrDegenerate)
		}
		alpha := new(big.Rat).Quo(new(big.Rat).Sub(target, knots[i]), denominator)
		beta := new(big.Rat).Sub(big.NewRat(1, 1), alpha)
		out[i] = RatPoint{
			U: new(big.Rat).Add(new(big.Rat).Mul(alpha, ctrl[i].U), new(big.Rat).Mul(beta, ctrl[i-1].U)),
			V: new(big.Rat).Add(new(big.Rat).Mul(alpha, ctrl[i].V), new(big.Rat).Mul(beta, ctrl[i-1].V)),
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
