package momentinput

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentregion"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func exactnessOf(bound float64) measurement.Exactness {
	if bound == 0 {
		return measurement.Exact
	}
	return measurement.Approximate
}

// Area returns the recorded region's net area — the outer loop minus its
// holes — as a [measurement.Measurement] of Kind Area (mm²): a computed quantity carries
// its Exactness and Bound (docs/api-design.md §6). Each boundary segment
// contributes its Green's-theorem integral in walk order, so a hole's clockwise
// walk subtracts without a special case.
//
// Line and Tier A free-form contributions — a spline, a closed spline, a
// NURBS whose weights are all equal, and a fit spline (docs/spline-design.md
// Table F) — integrate to exact rationals. A region built only from them is reported as the whole
// region's rational rounded ONCE, so its bound is that single rounding: zero,
// hence Exact, exactly when the rational is representable in float64, and never
// unconditionally. A circular contribution has no exact rational and carries a
// proven float evaluation bound, which the whole region then inherits.
//
// A region whose exact area is strictly positive is measured even where no
// float64 holds it: a section scaled far enough down reports the zero its
// rational rounds to, with that rounding as the bound — Approximate, never a
// refusal. A region whose area is genuinely zero or negative is [ErrDegenerate].
//
// The remaining free-form kinds are [ErrUnsupported] (docs/spline-design.md
// Table R) — never approximated: an ellipse, a conic and a rational NURBS have
// no exact rational moments yet, and an elliptical arc's record is
// self-inconsistent. So is a Tier A
// boundary whose exact integration would exceed this evaluator's fixed work
// budget (Table R row R7). A malformed or open record is [ErrDegenerate].
// A circle radius of the wrong kind is [ErrUnitKind], a negative radius is
// [ErrNegativeMagnitude], and a non-finite field or arithmetic result is
// [ErrNotFinite]. No measurement is returned on error.
func (r Profile) Area() (measurement.Measurement, error) {
	ig, err := r.integralsTo(freeform.MomentAreaOrder)
	if err != nil {
		return measurement.Measurement{}, err
	}
	return measurement.Measurement{
		Value:     units.SquareMillimeters(ig.Area),
		Exactness: exactnessOf(ig.AreaBound),
		Bound:     units.SquareMillimeters(ig.AreaBound),
	}, nil
}

// Centroid returns the recorded region's centroid from its bounded first
// moments, as a [measurement.VecMeasurement] — a computed coordinate is a measurement
// (docs/api-design.md §6). The Value is PLANE-LOCAL: (u, v, 0) in the
// region's own plane coordinates, millimetres (§5.2), not a world position —
// lift it through the profile's PlaneRecord to place it in space.
//
// A region whose exact area and first moments are all rational — a boundary of
// line and Tier A free-form walks only — has its centroid taken over those
// rationals and each coordinate rounded ONCE, so the reported bound is that
// single rounding: zero, hence Exact, exactly when the quotient is
// representable. The region's exact area is already proven strictly positive
// there (see RequirePositiveArea), so the quotient exists however small the
// area's own float image is — a section scaled far enough down for its area to
// underflow still reports its centroid.
//
// Only where some contribution has NO exact rational — a circular walk, whose
// integral carries π — is the centroid divided in bounded floats. A region whose
// net area is zero has no centroid and is [ErrDegenerate], and one whose net
// area that float division cannot prove stays away from zero is
// [ErrUnsupported]: the division has no bounded result. Record validation and
// arithmetic errors match [Profile.Area].
func (r Profile) Centroid() (measurement.VecMeasurement, error) {
	ig, err := r.integralsTo(freeform.MomentFirstOrder)
	if err != nil {
		return measurement.VecMeasurement{}, err
	}
	if exact, ok := ig.ExactCentroid(); ok {
		return exact, nil
	}
	if ig.Area == 0 && ig.AreaBound == 0 {
		return measurement.VecMeasurement{}, fmt.Errorf(`%w: a region with zero net area has no centroid`, ErrDegenerate)
	}
	if math.Abs(ig.Area) <= ig.AreaBound {
		return measurement.VecMeasurement{}, fmt.Errorf(`%w: the evaluator cannot prove this region's net area stays away from zero`, ErrUnsupported)
	}
	u := proofbound.BoundedQuotient(ig.Mu, ig.MuBound, ig.Area, ig.AreaBound)
	v := proofbound.BoundedQuotient(ig.Mv, ig.MvBound, ig.Area, ig.AreaBound)
	bound := proofbound.Radius2D(u.Bound, v.Bound)
	geometryBound := proofbound.Radius2D(
		proofbound.UpRound(math.Abs(u.Value)+ig.CoordUpper),
		proofbound.UpRound(math.Abs(v.Value)+ig.CoordUpper),
	)
	bound = math.Min(bound, geometryBound)
	return measurement.VecMeasurement{
		Value:     r3.NewVec(u.Value, v.Value, 0),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// ExactCentroid divides the region's exact first moments by its exact area over
// rationals and rounds each quotient ONCE, which is the same single-rounding
// rule PublishExact applies to the moments themselves (docs/spline-design.md
// §3). It reports whether the region has such an accumulator at all.
//
// It is what keeps the answer from being refused when it is already in hand.
// RequirePositiveArea has PROVEN the exact area strictly positive before this
// runs, so the centroid exists; the float guards below it read the published
// area, whose float image can be zero for a region whose exact area underflows,
// and would report ErrUnsupported for a quotient that is perfectly
// representable. The float path stays for the regions this one cannot serve:
// those whose accumulator a circular contribution retired.
func (ig Integrals) ExactCentroid() (measurement.VecMeasurement, bool) {
	if ig.ExactDead || !ig.Exact.Complete() || ig.Exact.Area.Sign() <= 0 {
		return measurement.VecMeasurement{}, false
	}
	u := new(big.Rat).Quo(ig.Exact.Mu, ig.Exact.Area)
	v := new(big.Rat).Quo(ig.Exact.Mv, ig.Exact.Area)
	uHeld, _ := u.Float64()
	vHeld, _ := v.Float64()
	if proofbound.IsNonFinite(uHeld) || proofbound.IsNonFinite(vHeld) {
		// No float64 holds this centroid, so there is no single rounding to
		// publish; the bounded path answers, or refuses, on its own terms.
		return measurement.VecMeasurement{}, false
	}
	bound := proofbound.Radius2D(proofarith.RationalFloatError(u, uHeld), proofarith.RationalFloatError(v, vHeld))
	return measurement.VecMeasurement{
		Value:     r3.NewVec(uHeld, vHeld, 0),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, true
}

// SecondMoments is a recorded region's second moments of area about the
// plane origin, in the plane's own (u, v): every field is a measurement.Measurement of
// Kind SecondMomentOfArea (mm⁴), with the closed-form evaluation's proven
// rounding bound. They are what a revolve's solid centroid is computed from
// (docs/evaluator-design.md §4/§6); to re-reference them to another axis, use
// the parallel-axis theorem with the region's Area and Centroid.
type SecondMoments struct {
	// UU is ∫u² dA, VV is ∫v² dA, UV is the mixed ∫uv dA.
	UU measurement.Measurement
	UV measurement.Measurement
	VV measurement.Measurement
}

// SecondMoments returns the region's bounded second moments of area about the
// plane origin. The staging matches [Profile.Area]: a Tier A free-form
// boundary is integrated exactly and rounded once, every other free-form kind
// is [ErrUnsupported], and malformed or non-finite records are rejected before
// a measurement is constructed.
func (r Profile) SecondMoments() (SecondMoments, error) {
	ig, err := r.integralsTo(freeform.MomentSecondOrder)
	if err != nil {
		return SecondMoments{}, err
	}
	measured := func(x, bound float64) measurement.Measurement {
		return measurement.Measurement{
			Value:     units.QuarticMillimeters(x),
			Exactness: exactnessOf(bound),
			Bound:     units.QuarticMillimeters(bound),
		}
	}
	return SecondMoments{
		UU: measured(ig.Muu, ig.MuuBound),
		UV: measured(ig.Muv, ig.MuvBound),
		VV: measured(ig.Mvv, ig.MvvBound),
	}, nil
}

// Integrals accumulates the boundary integrals of one region: the net
// signed area, the first moments ∫u dA and ∫v dA, and the second moments
// ∫u² dA, ∫uv dA and ∫v² dA.
type Integrals struct {
	CoordUpper float64 // max |u|+|v| over the recorded material
	Area       float64
	AreaBound  float64
	Mu         float64 // ∫u dA
	MuBound    float64
	Mv         float64 // ∫v dA
	MvBound    float64
	Muu        float64 // ∫u² dA
	MuuBound   float64
	Muv        float64 // ∫uv dA
	MuvBound   float64
	Mvv        float64 // ∫v² dA
	MvvBound   float64

	// exact is the WHOLE region's moments as exact rationals — every line and
	// Tier A free-form contribution added into it, and the anchor
	// re-referencing applied over rationals too. It stays alive only while
	// every contribution so far has had an exact rational, and when it does the
	// published float is that region-level rational rounded ONCE
	// (docs/spline-design.md §3/§5.2). Rounding per segment instead would make
	// the held float a sum of roundings, so a multi-segment region would miss
	// the single-rounding property a one-segment region has.
	Exact freeform.ExactMoments
	// exactDead records that some contribution had no exact rational — a
	// circular walk's integral carries π and trig terms — so the region's own
	// sum is not exact and the per-segment float accumulation with its own
	// proven bounds is what gets published.
	ExactDead bool

	// third is the freeform.MomentThirdOrder sum (∫u³, ∫u²v, ∫uv², ∫v³ dA, in that
	// order) as rational enclosures about the PLANE ORIGIN rather than the walk
	// anchor. It has no float twin: its only consumer is the revolve mass path,
	// which composes rational intervals, so no float conditioning needs the
	// anchor and no re-referencing step follows the walk. A line or Tier A
	// span contributes a point interval, a circular walk its enclosure.
	// thirdDead records a contribution with no enclosure — a circular record
	// no rational states — after which the region has no third-order moments
	// at all.
	Third     [4]proofbound.RatInterval
	ThirdDead bool
}

func (ig *Integrals) state() momentregion.State {
	return momentregion.State{
		CoordUpper: &ig.CoordUpper,
		Fields: [6]momentregion.Field{
			{Value: &ig.Area, Bound: &ig.AreaBound},
			{Value: &ig.Mu, Bound: &ig.MuBound},
			{Value: &ig.Mv, Bound: &ig.MvBound},
			{Value: &ig.Muu, Bound: &ig.MuuBound},
			{Value: &ig.Muv, Bound: &ig.MuvBound},
			{Value: &ig.Mvv, Bound: &ig.MvvBound},
		},
		Exact: &ig.Exact, ExactDead: &ig.ExactDead,
		Third: &ig.Third, ThirdDead: &ig.ThirdDead,
	}
}

// ThirdMoments returns the region's third-order sum and whether every
// boundary contribution had an enclosure. It is only populated by an
// integration run at freeform.MomentThirdOrder.
func (ig Integrals) ThirdMoments() ([4]proofbound.RatInterval, bool) {
	return ig.state().ThirdMoments()
}

func (ig Integrals) IsFinite(order freeform.MomentIntegralOrder) bool {
	return ig.state().IsFinite(order)
}

// ExactArea returns the region's exact area when every contribution had one.
func (ig Integrals) ExactArea() *big.Rat {
	if ig.ExactDead || !ig.Exact.Complete() {
		return nil
	}
	return ig.Exact.Area
}

// AddFreeformTo accumulates a converted and charged freeform curve.
func (ig *Integrals) AddFreeformTo(spans []freeform.BezierSpan, reversed bool, order freeform.MomentIntegralOrder) {
	ig.state().AddFreeform(spans, reversed, order)
}

// IntegralsBudget integrates after a wall-budget field preflight.
func (r Profile) IntegralsBudget(budget *proofbound.WorkBudget) (Integrals, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return Integrals{}, err
	}
	pre, err := ValidateFieldsWithPoll(func() error { return survey2d.WallBudgetStep(budget) }, r, nil)
	if err != nil {
		return Integrals{}, err
	}
	return integrateMomentRecordBudget(pre, freeform.MomentSecondOrder, budget)
}

func (r Profile) integralsTo(order freeform.MomentIntegralOrder) (Integrals, error) {
	pre, err := ValidateRecord(r)
	if err != nil {
		return Integrals{}, err
	}
	return integrateMomentRecord(pre, order)
}

// IntegralsTo validates topology and integrates to the requested order.
func (r Profile) IntegralsTo(order freeform.MomentIntegralOrder) (Integrals, error) {
	return r.integralsTo(order)
}

// EvaluatorIntegrals integrates checked fields with the caller's work counter.
func (r Profile) EvaluatorIntegrals(order freeform.MomentIntegralOrder, work *freeform.FreeformWork) (Integrals, error) {
	pre, err := ValidateFieldsWithPoll(nil, r, work)
	if err != nil {
		return Integrals{}, err
	}
	return integrateMomentRecord(pre, order)
}

// EvaluatorIntegralsContext integrates fields while polling the caller's context.
func (r Profile) EvaluatorIntegralsContext(ctx context.Context, order freeform.MomentIntegralOrder, work *freeform.FreeformWork) (Integrals, error) {
	pre, err := ValidateFieldsWithPoll(ctx.Err, r, work)
	if err != nil {
		return Integrals{}, err
	}
	return integrateMomentRecordModeContext(ctx, pre, order, true)
}

// EvaluatorIntegralsUncheckedContext skips the finite check for unused higher-order fields.
func (r Profile) EvaluatorIntegralsUncheckedContext(ctx context.Context, order freeform.MomentIntegralOrder, work *freeform.FreeformWork) (Integrals, error) {
	pre, err := ValidateFieldsWithPoll(ctx.Err, r, work)
	if err != nil {
		return Integrals{}, err
	}
	return integrateMomentRecordUncheckedContext(ctx, pre, order)
}

func integrateMomentRecord(pre FieldPreflight, order freeform.MomentIntegralOrder) (Integrals, error) {
	return integrateMomentRecordBudget(pre, order, nil)
}

func integrateMomentRecordBudget(pre FieldPreflight, order freeform.MomentIntegralOrder, budget *proofbound.WorkBudget) (Integrals, error) {
	return integrateMomentRecordMode(pre, order, true, budget)
}

func integrateMomentRecordMode(pre FieldPreflight, order freeform.MomentIntegralOrder, checkFinite bool, budget *proofbound.WorkBudget) (Integrals, error) {
	return integrateMomentRecordWithPoll(func() error { return survey2d.WallBudgetStep(budget) }, pre, order, checkFinite)
}

func integrateMomentRecordUncheckedContext(ctx context.Context, pre FieldPreflight, order freeform.MomentIntegralOrder) (Integrals, error) {
	return integrateMomentRecordModeContext(ctx, pre, order, false)
}

func integrateMomentRecordModeContext(ctx context.Context, pre FieldPreflight, order freeform.MomentIntegralOrder, checkFinite bool) (Integrals, error) {
	return integrateMomentRecordWithPoll(ctx.Err, pre, order, checkFinite)
}

func integrateMomentRecordWithPoll(poll func() error, pre FieldPreflight, order freeform.MomentIntegralOrder, checkFinite bool) (Integrals, error) {
	var ig Integrals
	for loopIndex, loop := range append([]LoopRecord{pre.Record.Outer}, pre.Record.Holes...) {
		if poll != nil {
			if err := poll(); err != nil {
				return Integrals{}, err
			}
		}
		for segmentIndex, segment := range loop.Segments {
			if poll != nil {
				if err := poll(); err != nil {
					return Integrals{}, err
				}
			}
			if err := ig.AddFor(segment, pre.PlanAt(loopIndex, segmentIndex), pre.Anchor, order); err != nil {
				return Integrals{}, err
			}
			if checkFinite && !ig.IsFinite(order) {
				return Integrals{}, fmt.Errorf(`%w: mass-property integration overflowed at loop %d segment %d`, ErrNotFinite, loopIndex, segmentIndex)
			}
		}
		if loopIndex < len(pre.ends) {
			chargeLoopJunctions(&ig, loop, pre.ends[loopIndex], pre.Anchor, order)
		}
	}
	ig = translateMomentIntegrals(ig, pre.Anchor, order)
	ig.PublishExact()
	if err := ig.RequirePositiveArea(); err != nil {
		return Integrals{}, err
	}
	if checkFinite && !ig.IsFinite(order) {
		return Integrals{}, fmt.Errorf(`%w: mass-property integration overflowed while restoring the profile origin`, ErrNotFinite)
	}
	return ig, nil
}

// RequirePositiveArea refuses a region whose net area is not positive.
//
// The region's own EXACT rational decides wherever there is one, because a
// strictly positive area can have a float64 image of zero: a valid section
// scaled far enough down has an exact area of s²·A > 0 that underflows, and a
// gate reading the float accumulator would refuse the region rather than publish
// the bounded zero the accumulator already holds — value 0 with the rational
// rounded up as its bound, hence Approximate, which is the honest reading of a
// positive area no float64 can hold. Every exactly integrated boundary is
// covered: the line path's closed forms and the Tier A free-form chains alike.
// Only where a contribution has no exact rational at all — a circular walk,
// whose integral carries π — does the float sum decide, as it always has.
func (ig *Integrals) RequirePositiveArea() error {
	return ig.state().RequirePositiveArea()
}

// translateMomentIntegrals restores the profile origin on a copy of the sum.
func translateMomentIntegrals(ig Integrals, anchor Point2, order freeform.MomentIntegralOrder) Integrals {
	ig.state().Translate(anchor, order)
	return ig
}

// add accumulates one segment's boundary-integral contribution, in the
// segment's recorded walk direction. Circular contributions carry an outward
// evaluation bound.
//
// Every contribution is re-referenced to the walk anchor, and the two
// arithmetics do it differently ON PURPOSE. The FLOAT evaluation reads
// anchor-shifted float coordinates, which is what keeps it conditioned. Every
// EXACT rational — a line's closed form, an arc's proven area interval, a
// converted free-form chain — subtracts the anchor over rationals instead,
// from the recorded coordinates themselves. Shifting in float first would round
// the geometry before the exact pipeline ever saw it, so the rational would be
// the exact answer for a region the caller did not record, and PublishExact
// would round an already representable value and report Exact with a zero bound
// for it.
// A free-form segment arrives with the chain the record-level preflight already
// converted and charged by ValidateRecord, so this pass converts nothing and
// charges nothing.
func (ig *Integrals) add(segment CurveSegment, plan Plan, anchor Point2, order freeform.MomentIntegralOrder) error {
	return ig.state().AddSegment(segment, momentregion.Plan{Spans: plan.Spans, Reversed: plan.Reversed}, anchor, order)
}

// AddAnalytic accumulates one line, circle or arc segment about the given
// anchor. It is how the section audits take a loop's own signed area: their
// loops are proven walkable before any area is asked for — walkOf refuses every
// free-form kind — so no converted chain is involved and no work is charged.
func (ig *Integrals) AddAnalytic(segment CurveSegment, anchor Point2) error {
	return ig.add(segment, Plan{}, anchor, freeform.MomentSecondOrder)
}

// AddFor skips higher-order work when the caller needs only area or first moments.
func (ig *Integrals) AddFor(segment CurveSegment, plan Plan, anchor Point2, order freeform.MomentIntegralOrder) error {
	return ig.add(segment, plan, anchor, order)
}

// PublishExact replaces the per-segment float accumulation with the region's
// own exact rational rounded ONCE, and the bound with that single rounding —
// zero, hence Exact, exactly when the rational is representable
// (docs/spline-design.md §3). It is a no-op once any contribution lacked an
// exact rational.
//
// Each field is published INDEPENDENTLY, and it must be: a field's value and
// bound are self-contained — the value is fl(exact) and the bound |exact −
// value| over rationals — so a rational no float64 can hold costs only its own
// field its single rounding. Publishing the six together instead abandons every
// one of them, so a second moment overflowing at coordinates near 1e78 mm denies
// Area the rational already in hand and leaves it the SUM of its per-segment
// roundings, past the half ulp §3 promises unconditionally. The mixed result is
// sound because every consumer reads each field through its own (value, bound)
// pair, and all cross-field composition — a revolve's axisMoments, the cup mass
// properties, Centroid's bounded-quotient fallback — is interval arithmetic,
// which asks only that each input interval encloses the truth.
func (ig *Integrals) PublishExact() {
	ig.state().PublishExact()
}
