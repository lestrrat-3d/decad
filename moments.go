package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentline"
	"github.com/lestrrat-3d/decad/internal/polynomial"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the mass-property engine of docs/evaluator-design.md §4:
// decad integrating its OWN records. sketch decides topology and
// admissibility; once a region is recorded, its areas and moments are decad's
// job, computed by closed-form boundary integrals (Green's theorem) per
// segment kind. Line and Tier A free-form walks (docs/spline-design.md Table F)
// integrate to exact rationals, so a region built only from them is published
// as its own rational rounded ONCE and retains a zero bound wherever that
// rational is representable; circular evaluations have no exact rational and
// carry outward bounds instead. Every other free-form kind is unsupported.
//
// Three sibling files carry the machinery this engine integrates with, each
// with its own doc comment: internal/proofbound/bounded.go the bounded-scalar arithmetic every
// published reading is composed in, internal/proofbound/rat_interval.go the exact rational
// interval arithmetic the certified terms are proven in, and
// moments_circular.go the circular segment's own enclosures.

// Area returns the recorded region's net area — the outer loop minus its
// holes — as a [Measurement] of Kind Area (mm²): a computed quantity carries
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
func (r ProfileRecord) Area() (Measurement, error) {
	ig, err := r.integralsTo(freeform.MomentAreaOrder)
	if err != nil {
		return Measurement{}, err
	}
	return Measurement{
		Value:     units.SquareMillimeters(ig.area),
		Exactness: exactnessOf(ig.areaBound),
		Bound:     units.SquareMillimeters(ig.areaBound),
	}, nil
}

// Centroid returns the recorded region's centroid from its bounded first
// moments, as a [VecMeasurement] — a computed coordinate is a measurement
// (docs/api-design.md §6). The Value is PLANE-LOCAL: (u, v, 0) in the
// region's own plane coordinates, millimetres (§5.2), not a world position —
// lift it through the profile's PlaneRecord to place it in space.
//
// A region whose exact area and first moments are all rational — a boundary of
// line and Tier A free-form walks only — has its centroid taken over those
// rationals and each coordinate rounded ONCE, so the reported bound is that
// single rounding: zero, hence Exact, exactly when the quotient is
// representable. The region's exact area is already proven strictly positive
// there (see requirePositiveArea), so the quotient exists however small the
// area's own float image is — a section scaled far enough down for its area to
// underflow still reports its centroid.
//
// Only where some contribution has NO exact rational — a circular walk, whose
// integral carries π — is the centroid divided in bounded floats. A region whose
// net area is zero has no centroid and is [ErrDegenerate], and one whose net
// area that float division cannot prove stays away from zero is
// [ErrUnsupported]: the division has no bounded result. Record validation and
// arithmetic errors match [ProfileRecord.Area].
func (r ProfileRecord) Centroid() (VecMeasurement, error) {
	ig, err := r.integralsTo(freeform.MomentFirstOrder)
	if err != nil {
		return VecMeasurement{}, err
	}
	if exact, ok := ig.exactCentroid(); ok {
		return exact, nil
	}
	if ig.area == 0 && ig.areaBound == 0 {
		return VecMeasurement{}, fmt.Errorf(`%w: a region with zero net area has no centroid`, ErrDegenerate)
	}
	if math.Abs(ig.area) <= ig.areaBound {
		return VecMeasurement{}, fmt.Errorf(`%w: the evaluator cannot prove this region's net area stays away from zero`, ErrUnsupported)
	}
	u := proofbound.BoundedQuotient(ig.mu, ig.muBound, ig.area, ig.areaBound)
	v := proofbound.BoundedQuotient(ig.mv, ig.mvBound, ig.area, ig.areaBound)
	bound := proofbound.Radius2D(u.Bound, v.Bound)
	geometryBound := proofbound.Radius2D(
		proofbound.UpRound(math.Abs(u.Value)+ig.coordUpper),
		proofbound.UpRound(math.Abs(v.Value)+ig.coordUpper),
	)
	bound = math.Min(bound, geometryBound)
	return VecMeasurement{
		Value:     r3.NewVec(u.Value, v.Value, 0),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// exactCentroid divides the region's exact first moments by its exact area over
// rationals and rounds each quotient ONCE, which is the same single-rounding
// rule publishExact applies to the moments themselves (docs/spline-design.md
// §3). It reports whether the region has such an accumulator at all.
//
// It is what keeps the answer from being refused when it is already in hand.
// requirePositiveArea has PROVEN the exact area strictly positive before this
// runs, so the centroid exists; the float guards below it read the published
// area, whose float image can be zero for a region whose exact area underflows,
// and would report ErrUnsupported for a quotient that is perfectly
// representable. The float path stays for the regions this one cannot serve:
// those whose accumulator a circular contribution retired.
func (ig regionIntegrals) exactCentroid() (VecMeasurement, bool) {
	if ig.exactDead || !ig.exact.Complete() || ig.exact.Area.Sign() <= 0 {
		return VecMeasurement{}, false
	}
	u := new(big.Rat).Quo(ig.exact.Mu, ig.exact.Area)
	v := new(big.Rat).Quo(ig.exact.Mv, ig.exact.Area)
	uHeld, _ := u.Float64()
	vHeld, _ := v.Float64()
	if proofbound.IsNonFinite(uHeld) || proofbound.IsNonFinite(vHeld) {
		// No float64 holds this centroid, so there is no single rounding to
		// publish; the bounded path answers, or refuses, on its own terms.
		return VecMeasurement{}, false
	}
	bound := proofbound.Radius2D(proofarith.RationalFloatError(u, uHeld), proofarith.RationalFloatError(v, vHeld))
	return VecMeasurement{
		Value:     r3.NewVec(uHeld, vHeld, 0),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, true
}

// SecondMoments is a recorded region's second moments of area about the
// plane origin, in the plane's own (u, v): every field is a Measurement of
// Kind SecondMomentOfArea (mm⁴), with the closed-form evaluation's proven
// rounding bound. They are what a revolve's solid centroid is computed from
// (docs/evaluator-design.md §4/§6); to re-reference them to another axis, use
// the parallel-axis theorem with the region's Area and Centroid.
type SecondMoments struct {
	// UU is ∫u² dA, VV is ∫v² dA, UV is the mixed ∫uv dA.
	UU Measurement
	UV Measurement
	VV Measurement
}

// SecondMoments returns the region's bounded second moments of area about the
// plane origin. The staging matches [ProfileRecord.Area]: a Tier A free-form
// boundary is integrated exactly and rounded once, every other free-form kind
// is [ErrUnsupported], and malformed or non-finite records are rejected before
// a measurement is constructed.
func (r ProfileRecord) SecondMoments() (SecondMoments, error) {
	ig, err := r.integralsTo(freeform.MomentSecondOrder)
	if err != nil {
		return SecondMoments{}, err
	}
	measured := func(x, bound float64) Measurement {
		return Measurement{
			Value:     units.QuarticMillimeters(x),
			Exactness: exactnessOf(bound),
			Bound:     units.QuarticMillimeters(bound),
		}
	}
	return SecondMoments{
		UU: measured(ig.muu, ig.muuBound),
		UV: measured(ig.muv, ig.muvBound),
		VV: measured(ig.mvv, ig.mvvBound),
	}, nil
}

// regionIntegrals accumulates the boundary integrals of one region: the net
// signed area, the first moments ∫u dA and ∫v dA, and the second moments
// ∫u² dA, ∫uv dA and ∫v² dA.
type regionIntegrals struct {
	coordUpper float64 // max |u|+|v| over the recorded material
	area       float64
	areaBound  float64
	mu         float64 // ∫u dA
	muBound    float64
	mv         float64 // ∫v dA
	mvBound    float64
	muu        float64 // ∫u² dA
	muuBound   float64
	muv        float64 // ∫uv dA
	muvBound   float64
	mvv        float64 // ∫v² dA
	mvvBound   float64

	// exact is the WHOLE region's moments as exact rationals — every line and
	// Tier A free-form contribution added into it, and the anchor
	// re-referencing applied over rationals too. It stays alive only while
	// every contribution so far has had an exact rational, and when it does the
	// published float is that region-level rational rounded ONCE
	// (docs/spline-design.md §3/§5.2). Rounding per segment instead would make
	// the held float a sum of roundings, so a multi-segment region would miss
	// the single-rounding property a one-segment region has.
	exact freeform.ExactMoments
	// exactDead records that some contribution had no exact rational — a
	// circular walk's integral carries π and trig terms — so the region's own
	// sum is not exact and the per-segment float accumulation with its own
	// proven bounds is what gets published.
	exactDead bool

	// third is the freeform.MomentThirdOrder sum (∫u³, ∫u²v, ∫uv², ∫v³ dA, in that
	// order) as rational enclosures about the PLANE ORIGIN rather than the walk
	// anchor. It has no float twin: its only consumer is the revolve mass path,
	// which composes rational intervals, so no float conditioning needs the
	// anchor and no re-referencing step follows the walk. A line or Tier A
	// span contributes a point interval, a circular walk its enclosure.
	// thirdDead records a contribution with no enclosure — a trimmed ArcSeg
	// fragment — after which the region has no third-order moments at all.
	third     [4]proofbound.RatInterval
	thirdDead bool
}

// addThird folds one segment's third-order contribution into the region's
// sum, or retires the sum for good when the segment has none.
func (ig *regionIntegrals) addThird(terms [4]proofbound.RatInterval, ok bool) {
	if ig.thirdDead {
		return
	}
	if !ok {
		ig.thirdDead = true
		ig.third = [4]proofbound.RatInterval{}
		return
	}
	if ig.third[0].Lo == nil {
		ig.third = terms
		return
	}
	for i := range ig.third {
		ig.third[i] = proofbound.IntervalAdd(ig.third[i], terms[i])
	}
}

// thirdMoments returns the region's third-order sum and whether every
// boundary contribution had an enclosure. It is only populated by an
// integration run at freeform.MomentThirdOrder.
func (ig regionIntegrals) thirdMoments() ([4]proofbound.RatInterval, bool) {
	if ig.thirdDead || ig.third[0].Lo == nil {
		return [4]proofbound.RatInterval{}, false
	}
	return ig.third, true
}

func accumulateMoment(value, bound *float64, term, termBound float64) {
	next := *value + term
	*bound = proofbound.AbsSumUpper(*bound, termBound, proofarith.AddRoundError(*value, term, next))
	*value = next
}

func (ig regionIntegrals) isFinite(order freeform.MomentIntegralOrder) bool {
	switch order {
	case freeform.MomentAreaOrder:
		return freeform.FiniteMomentValues(ig.area)
	case freeform.MomentFirstOrder:
		return freeform.FiniteMomentValues(ig.area, ig.mu, ig.mv)
	default:
		return freeform.FiniteMomentValues(ig.area, ig.mu, ig.mv, ig.muu, ig.muv, ig.mvv)
	}
}

func (r ProfileRecord) integralsBudget(budget *proofbound.WorkBudget) (regionIntegrals, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return regionIntegrals{}, err
	}
	pre, err := validateMomentFieldsBudget(budget, r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecordBudget(pre, freeform.MomentSecondOrder, budget)
}

func (r ProfileRecord) integralsTo(order freeform.MomentIntegralOrder) (regionIntegrals, error) {
	pre, err := validateMomentRecord(r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecord(pre, order)
}

// evaluatorIntegrals supplies only the mass properties an evaluator needs.
// Public measurement methods use integralsTo and retain full topology and
// finiteness checks; evaluator construction must not let unused higher-order
// overflow prevent clearance verification from running.
//
// work is the record's free-form work counter (docs/spline-design.md §5.2). An
// evaluator that already spent part of this record's ceiling passes the same
// counter, so the preflight below continues it rather than open a second one.
func (r ProfileRecord) evaluatorIntegrals(order freeform.MomentIntegralOrder, work *freeform.FreeformWork) (regionIntegrals, error) {
	pre, err := validateMomentFieldsWork(work, r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecord(pre, order)
}

func (r ProfileRecord) evaluatorIntegralsContext(ctx context.Context, order freeform.MomentIntegralOrder, work *freeform.FreeformWork) (regionIntegrals, error) {
	pre, err := validateMomentFieldsContext(ctx, work, r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecordModeContext(ctx, pre, order, true)
}

func (r ProfileRecord) evaluatorIntegralsUncheckedContext(ctx context.Context, order freeform.MomentIntegralOrder, work *freeform.FreeformWork) (regionIntegrals, error) {
	pre, err := validateMomentFieldsContext(ctx, work, r)
	if err != nil {
		return regionIntegrals{}, err
	}
	return integrateMomentRecordUncheckedContext(ctx, pre, order)
}

func integrateMomentRecord(pre momentPreflight, order freeform.MomentIntegralOrder) (regionIntegrals, error) {
	return integrateMomentRecordBudget(pre, order, nil)
}

func integrateMomentRecordBudget(pre momentPreflight, order freeform.MomentIntegralOrder, budget *proofbound.WorkBudget) (regionIntegrals, error) {
	return integrateMomentRecordMode(pre, order, true, budget)
}

func integrateMomentRecordMode(pre momentPreflight, order freeform.MomentIntegralOrder, checkFinite bool, budget *proofbound.WorkBudget) (regionIntegrals, error) {
	return integrateMomentRecordWithPoll(func() error { return survey2d.WallBudgetStep(budget) }, pre, order, checkFinite)
}

func integrateMomentRecordUncheckedContext(ctx context.Context, pre momentPreflight, order freeform.MomentIntegralOrder) (regionIntegrals, error) {
	return integrateMomentRecordModeContext(ctx, pre, order, false)
}

func integrateMomentRecordModeContext(ctx context.Context, pre momentPreflight, order freeform.MomentIntegralOrder, checkFinite bool) (regionIntegrals, error) {
	return integrateMomentRecordWithPoll(ctx.Err, pre, order, checkFinite)
}

func integrateMomentRecordWithPoll(poll func() error, pre momentPreflight, order freeform.MomentIntegralOrder, checkFinite bool) (regionIntegrals, error) {
	var ig regionIntegrals
	for loopIndex, loop := range append([]LoopRecord{pre.record.Outer}, pre.record.Holes...) {
		if poll != nil {
			if err := poll(); err != nil {
				return regionIntegrals{}, err
			}
		}
		for segmentIndex, segment := range loop.Segments {
			if poll != nil {
				if err := poll(); err != nil {
					return regionIntegrals{}, err
				}
			}
			if err := ig.addFor(segment, pre.planAt(loopIndex, segmentIndex), pre.anchor, order); err != nil {
				return regionIntegrals{}, err
			}
			if checkFinite && !ig.isFinite(order) {
				return regionIntegrals{}, fmt.Errorf(`%w: mass-property integration overflowed at loop %d segment %d`, ErrNotFinite, loopIndex, segmentIndex)
			}
		}
	}
	ig = translateMomentIntegrals(ig, pre.anchor, order)
	ig.publishExact()
	if err := ig.requirePositiveArea(); err != nil {
		return regionIntegrals{}, err
	}
	if checkFinite && !ig.isFinite(order) {
		return regionIntegrals{}, fmt.Errorf(`%w: mass-property integration overflowed while restoring the profile origin`, ErrNotFinite)
	}
	return ig, nil
}

// requirePositiveArea refuses a region whose net area is not positive.
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
func (ig *regionIntegrals) requirePositiveArea() error {
	if !ig.exactDead && ig.exact.Complete() {
		if ig.exact.Area.Sign() > 0 {
			return nil
		}
		return fmt.Errorf(`%w: the recorded region encloses no positive net area`, ErrDegenerate)
	}
	if ig.area <= 0 {
		return fmt.Errorf(`%w: the recorded region encloses no positive net area`, ErrDegenerate)
	}
	return nil
}

// shiftPoint re-references one recorded coordinate to the walk anchor in
// float64. It conditions the FLOAT evaluation only: every exact rational
// subtracts the anchor over rationals instead, because fl(p−anchor) rounds and
// an exact result taken over rounded coordinates is the exact answer for a
// different region (see regionIntegrals.add).
func shiftPoint(point, anchor Point2) Point2 {
	return Point2{U: point.U - anchor.U, V: point.V - anchor.V}
}

// shiftPoints translates a control-point slice into a fresh slice, leaving the
// caller's recorded segment untouched.
func translateMomentIntegrals(ig regionIntegrals, anchor Point2, order freeform.MomentIntegralOrder) regionIntegrals {
	if !ig.exactDead {
		ig.exact = translateExactMoments(ig.exact, anchor, order)
	}
	if order == freeform.MomentAreaOrder {
		return ig
	}
	area := proofbound.MeasuredScalar(ig.area, ig.areaBound)
	mu := proofbound.MeasuredScalar(ig.mu, ig.muBound)
	mv := proofbound.MeasuredScalar(ig.mv, ig.mvBound)
	if order >= freeform.MomentSecondOrder {
		two := proofbound.ExactScalar(2)
		anchorU := proofbound.ExactScalar(anchor.U)
		anchorV := proofbound.ExactScalar(anchor.V)
		muu := proofbound.BoundedAdd(
			proofbound.MeasuredScalar(ig.muu, ig.muuBound),
			proofbound.BoundedAdd(
				proofbound.BoundedMul(proofbound.BoundedMul(two, anchorU), mu),
				proofbound.BoundedMul(proofbound.BoundedMul(anchorU, anchorU), area),
			),
		)
		muv := proofbound.BoundedAdd(
			proofbound.MeasuredScalar(ig.muv, ig.muvBound),
			proofbound.BoundedAdd(
				proofbound.BoundedMul(anchorV, mu),
				proofbound.BoundedAdd(
					proofbound.BoundedMul(anchorU, mv),
					proofbound.BoundedMul(proofbound.BoundedMul(anchorU, anchorV), area),
				),
			),
		)
		mvv := proofbound.BoundedAdd(
			proofbound.MeasuredScalar(ig.mvv, ig.mvvBound),
			proofbound.BoundedAdd(
				proofbound.BoundedMul(proofbound.BoundedMul(two, anchorV), mv),
				proofbound.BoundedMul(proofbound.BoundedMul(anchorV, anchorV), area),
			),
		)
		ig.muu, ig.muuBound = muu.Value, muu.Bound
		ig.muv, ig.muvBound = muv.Value, muv.Bound
		ig.mvv, ig.mvvBound = mvv.Value, mvv.Bound
	}
	mu = proofbound.BoundedAdd(mu, proofbound.BoundedMul(proofbound.ExactScalar(anchor.U), area))
	mv = proofbound.BoundedAdd(mv, proofbound.BoundedMul(proofbound.ExactScalar(anchor.V), area))
	ig.mu, ig.muBound = mu.Value, mu.Bound
	ig.mv, ig.mvBound = mv.Value, mv.Bound
	ig.coordUpper = math.Max(ig.coordUpper, proofbound.AbsSumUpper(anchor.U, anchor.V))
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
// the exact answer for a region the caller did not record, and publishExact
// would round an already representable value and report Exact with a zero bound
// for it.
// A free-form segment arrives with the chain the record-level preflight already
// converted and charged (moments_validate.go), so this pass converts nothing and
// charges nothing.
func (ig *regionIntegrals) add(segment CurveSegment, plan freeformPlan, anchor Point2, order freeform.MomentIntegralOrder) error {
	segment, err := normalizeSegment(segment)
	if err != nil {
		return err
	}
	if order == freeform.MomentThirdOrder {
		// Before the switch: the free-form arm below shifts its spans to the
		// anchor in place, and the third-order sum is kept about the origin.
		ig.addThird(segmentThirdMoments(segment, plan))
	}
	switch segment := segment.(type) {
	case LineSeg:
		ig.addLine(segment, anchor, order)
		return nil
	case CircleSeg:
		if segment.Radius.Kind() != units.Length {
			return fmt.Errorf(`%w: a circle segment's radius must be a %s, got %s`, ErrUnitKind, units.Length, segment.Radius.Kind())
		}
		radius, err := segment.Radius.In(units.Millimeter)
		if err != nil {
			return fmt.Errorf(`%w: a circle segment's radius is not representable: %s`, ErrNotFinite, err)
		}
		if segment.CCW != (segment.TStart < segment.TEnd) {
			return fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, ErrDegenerate)
		}
		areaProof, haveAreaProof := circularAreaInterval(segment, anchor)
		muProof, mvProof, haveMomentProof := circularFirstMomentInterval(segment, anchor)
		var muuProof, muvProof, mvvProof proofbound.RatInterval
		var haveSecondMomentProof bool
		if order >= freeform.MomentSecondOrder {
			muuProof, muvProof, mvvProof, haveSecondMomentProof = circularSecondMomentInterval(segment, anchor)
		}
		segment.Center = shiftPoint(segment.Center, anchor)
		// The arrangement's normalized t is the angle 2π·t from +u
		// (geom.BoundaryEdge); the recorded range order is the walk.
		ig.addCircular(
			segment.Center,
			radius,
			2*math.Pi*segment.TStart,
			2*math.Pi*segment.TEnd,
			math.Abs(radius),
			proofbound.CircularSweepUpper(segment.TStart, segment.TEnd),
			areaProof,
			haveAreaProof,
			muProof,
			mvProof,
			haveMomentProof,
			muuProof,
			muvProof,
			mvvProof,
			haveSecondMomentProof,
			order,
		)
		return nil
	case ArcSeg:
		areaProof, haveAreaProof := circularAreaInterval(segment, anchor)
		muProof, mvProof, haveMomentProof := circularFirstMomentInterval(segment, anchor)
		var muuProof, muvProof, mvvProof proofbound.RatInterval
		var haveSecondMomentProof bool
		if order >= freeform.MomentSecondOrder {
			muuProof, muvProof, mvvProof, haveSecondMomentProof = circularSecondMomentInterval(segment, anchor)
		}
		segment.Center = shiftPoint(segment.Center, anchor)
		segment.Start = shiftPoint(segment.Start, anchor)
		segment.End = shiftPoint(segment.End, anchor)
		radius := math.Hypot(segment.Start.U-segment.Center.U, segment.Start.V-segment.Center.V)
		a0 := math.Atan2(segment.Start.V-segment.Center.V, segment.Start.U-segment.Center.U)
		a1 := math.Atan2(segment.End.V-segment.Center.V, segment.End.U-segment.Center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		// normalized t maps to angle = a0 + t·sweep; the range order is the walk.
		ig.addCircular(
			segment.Center,
			radius,
			a0+segment.TStart*sweep,
			a0+segment.TEnd*sweep,
			arcRadiusUpper(segment),
			proofbound.CircularSweepUpper(segment.TStart, segment.TEnd),
			areaProof,
			haveAreaProof,
			muProof,
			mvProof,
			haveMomentProof,
			muuProof,
			muvProof,
			mvvProof,
			haveSecondMomentProof,
			order,
		)
		return nil
	default:
		// A free-form kind with no converted chain is one the preflight could
		// not convert, so this evaluator has no integral for it.
		if !isFreeformSegment(segment) || len(plan.spans) == 0 {
			return fmt.Errorf(`%w: this evaluator computes mass properties over line, arc, circle and Tier A free-form profile segments only; the profile has a %T segment`, ErrUnsupported, segment)
		}
		// The chain was converted from the RECORDED control points, so shifting
		// it here over rationals keeps the spans the recorded curve.
		if err := shiftFreeformSpans(plan.spans, anchor); err != nil {
			return err
		}
		ig.addFreeformTo(plan.spans, plan.reversed, order)
		return nil
	}
}

// segmentThirdMoments encloses one normalized segment's third-order
// contribution about the plane origin: a line and a converted Tier A chain
// exactly, from their recorded coordinates, and a circular walk through
// circularThirdMomentInterval. Any other segment, and a circular walk that
// enclosure does not admit, answers false.
func segmentThirdMoments(segment CurveSegment, plan freeformPlan) ([4]proofbound.RatInterval, bool) {
	var exact [4]*big.Rat
	switch segment := segment.(type) {
	case LineSeg:
		u0 := ratLerp(segment.Start.U, segment.End.U, segment.TStart)
		v0 := ratLerp(segment.Start.V, segment.End.V, segment.TStart)
		u1 := ratLerp(segment.Start.U, segment.End.U, segment.TEnd)
		v1 := ratLerp(segment.Start.V, segment.End.V, segment.TEnd)
		if u0 == nil || v0 == nil || u1 == nil || v1 == nil {
			return [4]proofbound.RatInterval{}, false
		}
		exact = freeform.PolyThirdMoments(
			polynomial.RatPoly{u0, new(big.Rat).Sub(u1, u0)},
			polynomial.RatPoly{v0, new(big.Rat).Sub(v1, v0)},
		)
	case CircleSeg, ArcSeg:
		return circularThirdMomentInterval(segment)
	default:
		if !isFreeformSegment(segment) || len(plan.spans) == 0 {
			return [4]proofbound.RatInterval{}, false
		}
		exact = freeform.FreeformThirdMoments(plan.spans, plan.reversed)
	}
	var out [4]proofbound.RatInterval
	for i, value := range exact {
		out[i] = proofbound.PointInterval(value)
	}
	return out, true
}

// addAnalytic accumulates one line, circle or arc segment about the given
// anchor. It is how the section audits take a loop's own signed area: their
// loops are proven walkable before any area is asked for — walkOf refuses every
// free-form kind — so no converted chain is involved and no work is charged.
func (ig *regionIntegrals) addAnalytic(segment CurveSegment, anchor Point2) error {
	return ig.add(segment, freeformPlan{}, anchor, freeform.MomentSecondOrder)
}

// addFor skips second-moment work when the caller needs only area or first moments.
func (ig *regionIntegrals) addFor(segment CurveSegment, plan freeformPlan, anchor Point2, order freeform.MomentIntegralOrder) error {
	return ig.add(segment, plan, anchor, order)
}

// addLine folds a recorded line's neutral integral into the region sums.
func (ig *regionIntegrals) addLine(seg LineSeg, anchor Point2, order freeform.MomentIntegralOrder) {
	shifted := seg
	shifted.Start = shiftPoint(seg.Start, anchor)
	shifted.End = shiftPoint(seg.End, anchor)
	u0, v0 := lerp2(shifted.Start, shifted.End, shifted.TStart)
	u1, v1 := lerp2(shifted.Start, shifted.End, shifted.TEnd)
	_, _, coordUpper := lineWalkBounds(shifted, math.Hypot(u1-u0, v1-v0))
	ig.coordUpper = math.Max(ig.coordUpper, coordUpper)

	line := momentline.Line{
		Start:  momentline.Point{U: seg.Start.U, V: seg.Start.V},
		End:    momentline.Point{U: seg.End.U, V: seg.End.V},
		TStart: seg.TStart, TEnd: seg.TEnd,
	}
	values, bounds, exact := momentline.Evaluate(line, momentline.Point{U: anchor.U, V: anchor.V}, order)
	for i, field := range ig.heldFields() {
		if order < freeform.MomentSecondOrder && i >= 3 {
			break
		}
		accumulateMoment(field.value, field.bound, values[i], bounds[i])
	}
	ig.addExact(exact)
}

func newExactMoments() freeform.ExactMoments {
	return freeform.ExactMoments{
		Area: new(big.Rat),
		Mu:   new(big.Rat),
		Mv:   new(big.Rat),
		Muu:  new(big.Rat),
		Muv:  new(big.Rat),
		Mvv:  new(big.Rat),
	}
}

// heldFields pairs each accumulated float moment with its bound, in the same
// order freeform.ExactMoments.fields uses, so a rational and its published float are
// never matched up by hand at a call site.
func (ig *regionIntegrals) heldFields() [6]struct{ value, bound *float64 } {
	return [6]struct{ value, bound *float64 }{
		{&ig.area, &ig.areaBound},
		{&ig.mu, &ig.muBound},
		{&ig.mv, &ig.mvBound},
		{&ig.muu, &ig.muuBound},
		{&ig.muv, &ig.muvBound},
		{&ig.mvv, &ig.mvvBound},
	}
}

// addExact folds one segment's exact contribution into the region-level
// rational accumulator.
func (ig *regionIntegrals) addExact(exact freeform.ExactMoments) {
	if ig.exactDead {
		return
	}
	if !exact.Complete() {
		ig.dropExact()
		return
	}
	if !ig.exact.Complete() {
		ig.exact = newExactMoments()
	}
	running := ig.exact.Fields()
	for i, term := range exact.Fields() {
		running[i].Add(running[i], term)
	}
}

// dropExact retires the region-level rational accumulator for good: once one
// contribution has no exact rational, the region's own sum has none either.
func (ig *regionIntegrals) dropExact() {
	ig.exactDead = true
	ig.exact = freeform.ExactMoments{}
}

// publishExact replaces the per-segment float accumulation with the region's
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
func (ig *regionIntegrals) publishExact() {
	if ig.exactDead || !ig.exact.Complete() {
		return
	}
	exact := ig.exact.Fields()
	for i, field := range ig.heldFields() {
		held, _ := exact[i].Float64()
		if proofbound.IsNonFinite(held) {
			// No float64 holds this moment, so it has no single rounding to
			// publish; its own float accumulation and proven bound stand, and a
			// non-finite accumulation is refused by the order's finiteness check
			// rather than reported.
			continue
		}
		*field.value = held
		*field.bound = proofarith.RationalFloatError(exact[i], held)
	}
}

// translateExactMoments shifts exact integrals to the profile origin.
func translateExactMoments(exact freeform.ExactMoments, anchor Point2, order freeform.MomentIntegralOrder) freeform.ExactMoments {
	return momentline.TranslateExactMoments(exact, momentline.Point{U: anchor.U, V: anchor.V}, order)
}

func ratScale(value *big.Rat, num, den int64) *big.Rat {
	return momentline.RatScale(value, num, den)
}

func ratLerp(start, end, t float64) *big.Rat {
	return momentline.RatLerp(start, end, t)
}

// addCircular folds a bounded circular integral into the region sums.
func (ig *regionIntegrals) addCircular(
	c Point2,
	r, th0, th1, radiusUpper, sweepUpper float64,
	areaProof proofbound.RatInterval,
	haveAreaProof bool,
	muProof, mvProof proofbound.RatInterval,
	haveMomentProof bool,
	muuProof, muvProof, mvvProof proofbound.RatInterval,
	haveSecondMomentProof bool,
	order freeform.MomentIntegralOrder,
) {
	// Keep the area expression here so the independent section audit and this
	// integrator produce the same float bits on every supported platform.
	sin0, cos0 := math.Sincos(th0)
	sin1, cos1 := math.Sincos(th1)
	dth := th1 - th0
	area := 0.5 * (r*r*dth + c.U*r*(sin1-sin0) - c.V*r*(cos1-cos0))
	held := circularbounds.EvaluateFloat(
		circularPoint(c), r, th0, th1, radiusUpper, sweepUpper, area,
		areaProof, haveAreaProof, muProof, mvProof, haveMomentProof,
		muuProof, muvProof, mvvProof, haveSecondMomentProof, order,
	)
	ig.coordUpper = math.Max(ig.coordUpper, held.CoordUpper)
	// Circular integrals have no exact rational because they contain π and trig terms.
	ig.dropExact()
	for i, field := range ig.heldFields() {
		if order < freeform.MomentSecondOrder && i >= 3 {
			break
		}
		accumulateMoment(field.value, field.bound, held.Values[i], held.Bounds[i])
	}
}

// lerp2 returns the point at parameter t on the segment start→end.
//
// At the two natural bounds the answer is the record's own coordinate: the
// parameterization is P(t) = start + t·(end − start), so P(0) is start and P(1)
// is end, exactly. Those two cases therefore return the endpoint verbatim
// instead of evaluating the formula, whose float rounding need not land back on
// it — start + (end − start) can miss end by an ulp whenever the difference
// itself rounds. That is not a repair of the input: it is the same value the
// exact-rational twin ratLerp already returns at both bounds, and the same rule
// seam.go's edgeJoin already applies when it reads an uncut bound
// (TStart == 0 or TEnd == 1) off the record rather than off sketch's node.
//
// Reproducing the endpoint matters to every consumer that rebuilds geometry
// from a walk and then compares it against the record: buildPrismScene
// (prism_boolean.go) creates one sketch point per walked endpoint, so a walk
// that missed a whole segment's own vertex by an ulp would hand sketch two
// distinct points where the record states one, and the region sketch then
// admits on its proximity threshold would fail the seam's loop-closure
// falsifier at RecordProfile.
func lerp2(start, end Point2, t float64) (float64, float64) {
	switch t {
	case 0:
		return start.U, start.V
	case 1:
		return end.U, end.V
	}
	return start.U + t*(end.U-start.U), start.V + t*(end.V-start.V)
}

func arcRadiusUpper(seg ArcSeg) float64 {
	// The exact coordinate differences can each be no larger than the sum of
	// their input magnitudes, and hypot is no larger than the L1 norm.
	return proofbound.AbsSumUpper(seg.Start.U, seg.Center.U, seg.Start.V, seg.Center.V)
}
