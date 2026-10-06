package freeform

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/sketch/geom"
)

// FreeformWorkLimit is the fixed ceiling on ONE RECORD's free-form conversion,
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
const FreeformWorkLimit uint64 = 1 << 20

// ReconstructionWorkLimit is the separate fixed ceiling on one record's sketch
// topology reconstruction. Its cost is a record-wide quadratic in the chord
// total, not exact-rational conversion or integration work, so it must not
// consume the much smaller ceiling that bounds those passes. The public moment
// methods take no context, so this counter is still fixed and record-wide.
const ReconstructionWorkLimit uint64 = 1 << 26

// FreeformCostCeiling is where the conservative cost arithmetic below
// saturates. Any estimate that reaches it is already over budget on its own —
// step refuses at FreeformWorkLimit and this sits one unit above it — so the
// helpers never need to represent a larger number and can never wrap.
const FreeformCostCeiling = FreeformWorkLimit + 1

// ReconstructionCostCeiling is the corresponding saturation point for the
// sketch reconstruction charge. It is independent of FreeformCostCeiling so
// an ordinary analytic arrangement can use its own budget without widening the
// exact-rational conversion and integration budget.
const ReconstructionCostCeiling = ReconstructionWorkLimit + 1

// CostAdd and CostMul are the saturating arithmetic every preflight estimate is
// built from. A preflight is charged BEFORE the work it pays for is allocated,
// so it must be an UPPER bound: saturating (never wrapping) is what keeps an
// astronomically large shape refusing instead of charging a small number.
func CostAdd(a, b uint64) uint64 {
	if a >= FreeformCostCeiling || b >= FreeformCostCeiling || a > FreeformCostCeiling-b {
		return FreeformCostCeiling
	}
	return a + b
}

func CostMul(a, b uint64) uint64 {
	if a == 0 || b == 0 {
		return 0
	}
	if a >= FreeformCostCeiling || b >= FreeformCostCeiling || a > FreeformCostCeiling/b {
		return FreeformCostCeiling
	}
	return a * b
}

// ReconstructionCostAdd and ReconstructionCostMul are the saturating arithmetic
// for the sketch reconstruction charge. They use its own ceiling for the same
// reason CostAdd and CostMul use FreeformCostCeiling for exact-rational work.
func ReconstructionCostAdd(a, b uint64) uint64 {
	if a >= ReconstructionCostCeiling || b >= ReconstructionCostCeiling || a > ReconstructionCostCeiling-b {
		return ReconstructionCostCeiling
	}
	return a + b
}

func ReconstructionCostMul(a, b uint64) uint64 {
	if a == 0 || b == 0 {
		return 0
	}
	if a >= ReconstructionCostCeiling || b >= ReconstructionCostCeiling || a > ReconstructionCostCeiling/b {
		return ReconstructionCostCeiling
	}
	return a * b
}

// FreeformWork holds one record's exact-rational and reconstruction counters.
// They are separate because their cost models and safe ceilings are separate,
// but each counter spans the whole operation over that record.
type FreeformWork struct {
	Spent               uint64
	ReconstructionSpent uint64
}

// NewFreeformWork opens ONE record's work state. Minting is deliberately
// explicit and rare: each ceiling bounds that record's total work across the
// whole operation. Mint one where a record walk begins with no preflight state
// in hand; everywhere else, pass the state the record already has.
func NewFreeformWork() *FreeformWork { return &FreeformWork{} }

func (w *FreeformWork) Step(n uint64) error {
	if w == nil {
		return nil
	}
	if n > FreeformWorkLimit-w.Spent {
		w.Spent = FreeformWorkLimit
		return fmt.Errorf(
			`%w: free-form exact integration needs more than the fixed work budget of %d`,
			decaderr.ErrUnsupported, FreeformWorkLimit,
		)
	}
	w.Spent += n
	return nil
}

// reconstructionStep spends the record's sketch reconstruction counter before
// sketch arranges the scene.
func (w *FreeformWork) ReconstructionStep(n uint64) error {
	if w == nil {
		return nil
	}
	if n > ReconstructionWorkLimit-w.ReconstructionSpent {
		w.ReconstructionSpent = ReconstructionWorkLimit
		return fmt.Errorf(
			`%w: sketch reconstruction needs more than the fixed work budget of %d`,
			decaderr.ErrUnsupported, ReconstructionWorkLimit,
		)
	}
	w.ReconstructionSpent += n
	return nil
}

// RequireFiniteFreeformRange rejects a non-finite recorded range on ANY
// free-form kind. Core §12 gives [ErrNotFinite] for a non-finite input, which is
// what every other segment kind reports for exactly this field and what the
// free-form path already reports for a non-finite control coordinate. It is
// therefore decided ahead of the kind dispatch, never inside one kind's arm: a
// NaN fails both full-domain equality tests, so a kind that tested the range
// alone would report it as a trimmed range and a kind that never tested it at
// all would report its own staging reason instead. The test is O(1) on the
// recorded parameters, so it stands with the structural size refusals, ahead of
// any content scan.
func RequireFiniteFreeformRange(tStart, tEnd float64, what string) error {
	if FiniteMomentValues(tStart, tEnd) {
		return nil
	}
	return fmt.Errorf(
		`%w: a %s's recorded range is not finite (range [%v, %v])`,
		decaderr.ErrNotFinite, what, tStart, tEnd,
	)
}

// RequireFullFreeformRange rejects a recorded free-form range that is not the
// entity's full domain. spline design §2 proves none is recordable, so reaching
// this is a caller-built or decoded record that bypassed the seam — refuse
// rather than integrate a piece the conversion does not cover.
//
// It is the Tier A arms' own gate, and it stays there. Table R states R2
// unconditionally and carries no row for a trimmed range reaching the evaluator,
// so a kind refused for its own cause reports that cause whatever its range
// says. A FitSplineSeg carries no such unconditional refusal — it is Tier A
// for the moments path (Table F) — so it reaches this same gate instead of
// skipping it. Finiteness is the separate refusal above, already decided for
// every kind before this runs.
func RequireFullFreeformRange(tStart, tEnd float64, what string) error {
	if (tStart == 0 && tEnd == 1) || (tStart == 1 && tEnd == 0) {
		return nil
	}
	return fmt.Errorf(
		`%w: a %s must span its full domain; a trimmed free-form range is never recordable (range [%v, %v])`,
		decaderr.ErrUnsupported, what, tStart, tEnd,
	)
}

// ChargeRationalLift charges the rational lift of a recorded curve's own
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
// whole record's are at most K·FreeformWorkLimit however the record is split
// into segments. K is 4 today — the widest kind is a NURBSSeg, whose preflight
// walks its controls once (validateSegmentPoints), its knots three times (the
// finite/monotone scan, validateNURBSInteriorMultiplicity and FloatKnotDemand),
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
// (ClampedConversionCost).
func ChargeRationalLift(work *FreeformWork, controls, knots, weights int) error {
	return work.Step(RationalLiftCost(controls, knots, weights))
}

func RationalLiftCost(controls, knots, weights int) uint64 {
	return CostAdd(CostAdd(CostMul(2, uint64(controls)), uint64(knots)), uint64(weights))
}

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
		knots[i] = MustRatOf(knot)
	}
	return knots
}

// RatWeighted returns (Σ wᵢ·pᵢ)/den exactly. Callers pass equal-length slices.
func RatWeighted(points []survey2d.RatPoint, weights []int64, den int64) survey2d.RatPoint {
	axis := func(get func(survey2d.RatPoint) *big.Rat) *big.Rat {
		out := new(big.Rat)
		for i, point := range points {
			out.Add(out, new(big.Rat).Mul(get(point), big.NewRat(weights[i], 1)))
		}
		return out.Quo(out, big.NewRat(den, 1))
	}
	return survey2d.RatPoint{
		U: axis(func(p survey2d.RatPoint) *big.Rat { return p.U }),
		V: axis(func(p survey2d.RatPoint) *big.Rat { return p.V }),
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
func ClampedBezierSpans(degree int, ctrl []survey2d.RatPoint, knots []*big.Rat) ([]survey2d.BezierSpan, error) {
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
	spans := make([]survey2d.BezierSpan, count)
	for j := range count {
		span := make(survey2d.BezierSpan, degree+1)
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
func BezierSliceCount(degree int, ctrl []survey2d.RatPoint, knots []*big.Rat) (int, error) {
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
func BrokenKnot(degree int, ctrl []survey2d.RatPoint, start, run int) bool {
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
func InsertKnot(degree int, ctrl []survey2d.RatPoint, knots []*big.Rat, target *big.Rat) ([]survey2d.RatPoint, []*big.Rat, error) {
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

	out := make([]survey2d.RatPoint, len(ctrl)+1)
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
		out[i] = survey2d.RatPoint{
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

// The chord counts sketch's own reconstruction sampler produces, restated from
// geom/arrange.go's sampleParams. They are restated rather than read because the
// reconstruction charge below has to be levied before that sampler runs and no
// recorded field reports them: a free-form source is chorded
// FreeformChordsPerControl times per control point with FreeformChordFloor as
// its floor, and a curved analytic source AnalyticChordsPerTurn times per full
// turn. They are sketch's numbers, so a change upstream is a change here — and
// they are FLOORED, which is why a record of many tiny curves arranges far more
// chords than its control count suggests.
const (
	FreeformChordsPerControl = 16
	FreeformChordFloor       = 64
	AnalyticChordsPerTurn    = 256
)

// FreeformReconstruction is the whole-RECORD model of the sketch reconstruction
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
type FreeformReconstruction struct {
	// chords is the record-wide chord total the arrangement will hold. Once it
	// crosses ReconstructionChordCeiling the charge refuses whatever the rest of
	// the record holds, so reconstructionOf stops counting rather than spending
	// more work — or interning memory — on a record already refused.
	Chords uint64
	// arrangement is one whole-scene arrangement's charge: the ORDERED pair
	// count, twice the i<j pairs the intersection loop runs, so the doubling
	// stands for the rest of the pass — the vertex table, the chord splitting
	// and the region walk — rather than being modelled separately.
	Arrangement uint64
}

// ReconstructionChordCeiling is the largest chord total chargeReconstruction
// can admit for validation's first two whole-scene arrangements. One unit more
// exceeds ReconstructionWorkLimit, so the record refuses however the rest of it
// reads, and reconstructionOf stops counting there.
const ReconstructionChordCeiling uint64 = 5792

// FreeformChords is the per-control sample count with sketch's own floor. The
// floor is what makes a record of many three-control splines expensive: each one
// arranges FreeformChordFloor chords however few controls it holds.
func FreeformChords(count int) uint64 {
	if count <= 0 {
		return FreeformChordFloor
	}
	chords := CostMul(FreeformChordsPerControl, uint64(count))
	if chords < FreeformChordFloor {
		return FreeformChordFloor
	}
	return chords
}

// ChargeFreeformShift preflights the re-anchoring of a whole converted chain:
// two rational subtractions per control point of every span. It is levied at
// the record-level preflight beside the conversion and integration charges
// (moments_validate.go), never where the shift itself runs — a charge levied at
// the point of use lands after the sketch reconstruction the ceiling exists to
// precede, so a chain that fits the budget everywhere except its re-anchoring
// would run minutes of uncancellable work before refusing.
func ChargeFreeformShift(spans []survey2d.BezierSpan, work *FreeformWork) error {
	for _, span := range spans {
		if err := work.Step(CostMul(2, uint64(len(span)))); err != nil {
			return err
		}
	}
	return nil
}

// FreeformEndControls picks the converted chain's first and last CONTROL points
// in the recorded walk order. It is the single owner of that selection —
// freeformEndpoints rounds the pair it returns and freeformEndpointBounds
// measures that rounding, and the two readings must never disagree about which
// control point an end is.
func FreeformEndControls(spans []survey2d.BezierSpan, reversed bool) (survey2d.RatPoint, survey2d.RatPoint, bool) {
	if len(spans) == 0 || len(spans[0]) == 0 || len(spans[len(spans)-1]) == 0 {
		return survey2d.RatPoint{}, survey2d.RatPoint{}, false
	}
	first := spans[0][0]
	last := spans[len(spans)-1][len(spans[len(spans)-1])-1]
	if reversed {
		first, last = last, first
	}
	return first, last, true
}

// EndTangents is a walk's pair of end tangent directions, each with the proven
// error bound it carries on EITHER of its two components — the pair a
// survey2d.SegmentWalk copies into tanIn/tanInBound and tanOut/tanOutBound.
type EndTangents struct {
	InU, InV   float64
	InBound    float64
	OutU, OutV float64
	OutBound   float64
}

// FreeformEndTangents returns the walk's tangent directions at its start and
// end. A Bézier's derivative at an end is degree·(the adjacent control leg), so
// the DIRECTION is an exact fact of the control net — no sampling, and no
// normalization (a survey2d.SegmentWalk tangent is a direction, not a unit vector).
//
// The float64 the walk holds is not that exact fact, though: the leg is formed
// over big.Rat and then rounded once on the way out, and a control point of an
// ordinary rational curve is a ratio no float64 lands on. So each tangent
// STATES its bound — the gap from the exact rational leg to the float that
// stands for it, the wider component of the two — rather than passing the
// rounding off as exactness.
//
// A reversed walk enters where the curve leaves, so both the order and the sign
// of the two legs flip. IEEE negation is exact, so each bound rides along with
// the leg it belongs to.
func FreeformEndTangents(spans []survey2d.BezierSpan, reversed bool) (EndTangents, error) {
	if len(spans) == 0 {
		return EndTangents{}, fmt.Errorf(`%w: a converted free-form curve holds no span`, decaderr.ErrDegenerate)
	}
	first, last := spans[0], spans[len(spans)-1]
	if len(first) < 2 || len(last) < 2 {
		return EndTangents{}, fmt.Errorf(`%w: a converted free-form span holds fewer than two control points`, decaderr.ErrDegenerate)
	}
	leg := func(from, to survey2d.RatPoint, degree int) (float64, float64, float64, bool) {
		scale := big.NewRat(int64(degree), 1)
		du := new(big.Rat).Mul(scale, new(big.Rat).Sub(to.U, from.U))
		dv := new(big.Rat).Mul(scale, new(big.Rat).Sub(to.V, from.V))
		u, _ := du.Float64()
		v, _ := dv.Float64()
		if proofbound.IsNonFinite(u) || proofbound.IsNonFinite(v) {
			return 0, 0, 0, false
		}
		bound := math.Max(proofarith.RationalFloatError(du, u), proofarith.RationalFloatError(dv, v))
		return u, v, bound, true
	}
	inU, inV, inBound, okIn := leg(first[0], first[1], len(first)-1)
	outU, outV, outBound, okOut := leg(last[len(last)-2], last[len(last)-1], len(last)-1)
	if !okIn || !okOut {
		return EndTangents{}, fmt.Errorf(`%w: a free-form end tangent is not representable`, decaderr.ErrNotFinite)
	}
	if reversed {
		return EndTangents{
			InU: -outU, InV: -outV, InBound: outBound,
			OutU: -inU, OutV: -inV, OutBound: inBound,
		}, nil
	}
	return EndTangents{
		InU: inU, InV: inV, InBound: inBound,
		OutU: outU, OutV: outV, OutBound: outBound,
	}, nil
}

// FreeformControlExtent is an upper envelope on |u|+|v| over the curve, read
// off the control points. The convex hull property makes it a PROVEN envelope
// for the curve itself, not just for its control net.
func FreeformControlExtent(spans []survey2d.BezierSpan) float64 {
	extent := 0.0
	for _, span := range spans {
		for _, point := range span {
			u, _ := new(big.Rat).Abs(point.U).Float64()
			v, _ := new(big.Rat).Abs(point.V).Float64()
			if sum := proofbound.AbsSumUpper(u, v); sum > extent {
				extent = sum
			}
		}
	}
	return extent
}
