package freeform

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
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
//
// It is the DEFAULT ceiling. One operation raises it for its own record
// counters (FreeformWork.Limit): a loft raises it, once its station cap gate
// has passed, by a fixed charge per station its cap admits
// (docs/loft-gear-bounds-design.md §7). No other caller raises it.
const FreeformWorkLimit uint64 = 1 << 20

// ReconstructionWorkLimit is the separate fixed ceiling on one record's sketch
// topology reconstruction. Its cost is a record-wide quadratic in the chord
// total, not exact-rational conversion or integration work, so it must not
// consume the much smaller ceiling that bounds those passes. The public moment
// methods take no context, so this counter is still fixed and record-wide.
//
// It is decad's model of sketch's quadratic arranger: 1 << 28 admits a record
// of ReconstructionChordCeiling chords, which a forty-tooth gear outline's
// 6640 chords fit under (docs/loft-gear-bounds-design.md §7).
const ReconstructionWorkLimit uint64 = 1 << 28

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
//
// Limit is the exact-rational counter's ceiling. Zero reads as
// FreeformWorkLimit, so a counter minted by NewFreeformWork or as a literal
// keeps the default. RaiseLimit is the one way an operation changes it.
type FreeformWork struct {
	Spent               uint64
	ReconstructionSpent uint64
	Limit               uint64
}

// NewFreeformWork opens ONE record's work state. Minting is deliberately
// explicit and rare: each ceiling bounds that record's total work across the
// whole operation. Mint one where a record walk begins with no preflight state
// in hand; everywhere else, pass the state the record already has.
func NewFreeformWork() *FreeformWork { return &FreeformWork{} }

// WorkLimit is the exact-rational counter's ceiling: Limit, or
// FreeformWorkLimit where Limit is zero.
func (w *FreeformWork) WorkLimit() uint64 {
	if w.Limit == 0 {
		return FreeformWorkLimit
	}
	return w.Limit
}

// RaiseLimit sets the exact-rational counter's ceiling to limit when that is
// above the current one, and never lowers it.
func (w *FreeformWork) RaiseLimit(limit uint64) {
	if limit > w.WorkLimit() {
		w.Limit = limit
	}
}

// Step charges n units to the exact-rational counter and refuses with
// ErrUnsupported (Table R row R7) once the total would pass WorkLimit.
//
// A charge at or above FreeformCostCeiling refuses whatever the limit is.
// CostAdd and CostMul saturate there, so such a charge stands for an
// estimate of unknown size; under a raised limit it would otherwise be
// admitted as if it cost FreeformCostCeiling units.
func (w *FreeformWork) Step(n uint64) error {
	if w == nil {
		return nil
	}
	limit := w.WorkLimit()
	if n >= FreeformCostCeiling || n > limit-w.Spent {
		w.Spent = limit
		return fmt.Errorf(
			`%w: free-form exact integration needs more than the fixed work budget of %d`,
			decaderr.ErrUnsupported, limit,
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
// reads, and reconstructionOf stops counting there: 2·11585² is at most
// 1 << 28 and 2·11586² is above it.
const ReconstructionChordCeiling uint64 = 11585

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
func ChargeFreeformShift(spans []BezierSpan, work *FreeformWork) error {
	for _, span := range spans {
		if err := work.Step(CostMul(2, uint64(len(span)))); err != nil {
			return err
		}
	}
	return nil
}
