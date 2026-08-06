package decad

import (
	"context"
	"fmt"
	"math"
	"slices"
	"strings"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// momentPreflight is one whole ProfileRecord's preflight: the checked record,
// the walk anchor the integrator re-references its moments to, and the converted
// Bézier chain of every free-form segment.
//
// It is the record-level step docs/spline-design.md §5's work ceiling needs.
// ONE freeformWork counter runs through the entire record, and through every
// later phase of the operation that reads it — the ceiling bounds a RECORD's
// total free-form work, not each segment's separately and not each pass's, so a
// record of individually cheap segments whose aggregate is unaffordable refuses
// here rather than running to completion — and every charge is levied before
// the work it pays for: the rational lift and knot insertion of the
// conversion, the re-anchoring of each converted chain, the exact integration
// that runs only later in the moments pass, and the whole-scene arrangement the topology
// reconstruction runs. A charge levied where its work runs fires after the
// sketch reconstruction and the sampling that reconstruction does, which is
// precisely the work the ceiling exists to precede: the public ProfileRecord
// methods take no context, so a late refusal cannot be cancelled and bounds
// nothing.
//
// The converted chains are kept for the same reason. The moments pass
// integrates the chains this preflight already converted and paid for, rather
// than converting a second time on a counter that would have to be charged
// again.
type momentPreflight struct {
	record ProfileRecord
	anchor Point2
	// plans holds one entry per CONVERTED free-form segment, keyed by its
	// [loop, segment] index over the checked record's loops in
	// outer-then-holes order — the order the moments pass walks them in.
	//
	// It is sparse, and deliberately so. The key set is decided by the
	// segments that actually converted, never by the recorded loop lengths, so
	// an analytic record allocates no plan storage at all and a free-form
	// record pays only for the segments that already passed their own charge.
	// Storage sized by the recorded segment count would be forced by an
	// untrusted record ahead of the first per-segment charge, which is the one
	// thing that can refuse it.
	plans map[[2]int]freeformPlan
	// work is the record's own counter, still open — and it is the OPERATION's
	// where one was handed in, so a caller that spent part of the ceiling on
	// this record earlier reads what is left rather than a fresh one (§5.2). The
	// topology reconstruction re-arranges the whole scene once per candidate
	// profile it authenticates, so those arrangements are charged here as they
	// happen rather than predicted.
	work *freeformWork
	// arrangement is one whole-scene arrangement's charge. This preflight levies
	// it only for a record holding a free-form segment; a record holding none
	// leaves it zero here and is charged the same way by validateMomentRecord,
	// the one entry that actually runs the reconstruction. Either way the charge
	// counts EVERY source, analytic ones included, because the arrangement is
	// global (docs/spline-design.md §5.2).
	arrangement uint64
}

// freeformPlan is one free-form segment's converted Bézier chain beside the
// walk direction its recorded range order states. A zero plan (nil spans) means
// the segment is not a converted free-form one — a line, an arc or a circle,
// each integrated from its own closed form.
type freeformPlan struct {
	spans    []bezierSpan
	reversed bool
}

// planAt returns the plan the preflight converted for one segment. A segment
// the preflight recorded no plan for — every line, arc and circle — reads the
// zero plan, which is what the moments pass integrates from its own closed form.
func (p momentPreflight) planAt(loopIndex, segmentIndex int) freeformPlan {
	return p.plans[[2]int{loopIndex, segmentIndex}]
}

// validateMomentRecord normalizes and checks the fields the integrator reads,
// then asks sketch to decide whether those entities form the recorded region.
// decad does not carry a second planar-arrangement implementation.
func validateMomentRecord(record ProfileRecord) (momentPreflight, error) {
	pre, err := validateMomentFields(record)
	if err != nil {
		return momentPreflight{}, err
	}
	if circular, err := validateWholeCircleRegion(pre.record); circular {
		if err != nil {
			return momentPreflight{}, err
		}
		return pre, nil
	}
	if err := pre.chargeAnalyticReconstruction(); err != nil {
		return momentPreflight{}, err
	}
	matched, err := pre.matchesSketch(pre.record)
	if err != nil {
		return momentPreflight{}, err
	}
	if !matched {
		validationRecord, err := scaleMomentRecordForValidation(pre.record, pre.anchor)
		if err != nil {
			return momentPreflight{}, err
		}
		// The rescaled record names the same entities at unit scale, so it holds
		// the same chord total and each of its arrangements costs the same.
		matched, err := pre.matchesSketch(validationRecord)
		if err != nil {
			return momentPreflight{}, err
		}
		if !matched {
			return momentPreflight{}, fmt.Errorf(
				`%w: the recorded segments do not form the stated closed region`,
				ErrDegenerate,
			)
		}
	}
	return pre, nil
}

// matchesSketch runs one reconstruction pass on the record's own counter.
func (p momentPreflight) matchesSketch(record ProfileRecord) (bool, error) {
	return momentRecordMatchesSketch(record, p.work, p.arrangement)
}

// chargeAnalyticReconstruction levies the record-wide arrangement charge for a
// record the field preflight left uncharged, which is exactly the record holding
// no free-form segment. It is the SAME charge on the SAME record counter — the
// ceiling is one per record, never one per kind — and it is levied here because
// this is where an analytic record's reconstruction is decided: after the exact
// whole-circle certificate, which runs no arrangement and so owes none, and
// before sketch is asked anything at all.
//
// Without it an analytic record reaches the reconstruction uncharged, and its
// cost is the same global quadratic a free-form record's is: sketch chords every
// source in the scene — 256 chords for a whole turn, its own share of that for
// an arc — and then tests every PAIR of chords, once per candidate profile and
// again for the rescaled retry. The three public ProfileRecord methods take no
// context, so a record large enough to make that pass expensive occupies the
// caller uncancellably (docs/spline-design.md §5.2).
func (p *momentPreflight) chargeAnalyticReconstruction() error {
	if p.arrangement != 0 {
		return nil
	}
	arrangement, err := chargeFreeformReconstruction(p.record, p.work)
	if err != nil {
		return fmt.Errorf(`decad: profile record is invalid: %w`, err)
	}
	p.arrangement = arrangement
	return nil
}

func validateMomentFields(record ProfileRecord) (momentPreflight, error) {
	return validateMomentFieldsBudget(nil, record)
}

// validateMomentFieldsWork is the preflight an evaluator runs when it already
// holds this record's counter: the charges below continue it rather than start a
// second full ceiling on the same record (docs/spline-design.md §5.2).
func validateMomentFieldsWork(work *freeformWork, record ProfileRecord) (momentPreflight, error) {
	return validateMomentFieldsWithPoll(nil, record, work)
}

func validateMomentFieldsBudget(budget *workBudget, record ProfileRecord) (momentPreflight, error) {
	return validateMomentFieldsWithPoll(func() error { return wallBudgetStep(budget) }, record, newFreeformWork())
}

func validateMomentFieldsContext(ctx context.Context, work *freeformWork, record ProfileRecord) (momentPreflight, error) {
	return validateMomentFieldsWithPoll(ctx.Err, record, work)
}

// work is the counter this record spends against freeformWorkLimit. It is a
// parameter rather than a local because the ceiling is the RECORD's across a
// whole operation: an evaluator that already charged this record's conversion
// passes the same counter back in, so a later phase spends what is left instead
// of a fresh ceiling.
func validateMomentFieldsWithPoll(poll func() error, record ProfileRecord, work *freeformWork) (momentPreflight, error) {
	loops := append([]LoopRecord{record.Outer}, record.Holes...)
	normalized := make([]LoopRecord, len(loops))
	// Plan storage is minted on the first segment that converts, so a record
	// naming none allocates none (see momentPreflight.plans).
	var plans map[[2]int]freeformPlan
	if work == nil {
		// One counter for the whole record: the ceiling bounds the record's total
		// free-form work, never each segment's own.
		work = newFreeformWork()
	}
	var anchor Point2
	freeform := false
	for loopIndex := range normalized {
		if poll != nil {
			if err := poll(); err != nil {
				return momentPreflight{}, err
			}
		}
		loop := record.Outer
		if loopIndex > 0 {
			loop = record.Holes[loopIndex-1]
		}
		if len(loop.Segments) == 0 {
			return momentPreflight{}, fmt.Errorf(
				`decad: profile loop %d is invalid: %w: a recorded loop holds no segments`,
				loopIndex,
				ErrDegenerate,
			)
		}
		normalized[loopIndex].Segments = make([]CurveSegment, len(loop.Segments))
		for segmentIndex, segment := range loop.Segments {
			if poll != nil {
				if err := poll(); err != nil {
					return momentPreflight{}, err
				}
			}
			checked, start, plan, err := validateMomentSegment(segment, work)
			if err != nil {
				return momentPreflight{}, fmt.Errorf(
					`decad: profile loop %d segment %d is invalid: %w`,
					loopIndex,
					segmentIndex,
					err,
				)
			}
			normalized[loopIndex].Segments[segmentIndex] = checked
			if len(plan.spans) > 0 {
				if plans == nil {
					plans = make(map[[2]int]freeformPlan)
				}
				plans[[2]int{loopIndex, segmentIndex}] = plan
			}
			freeform = freeform || isFreeformSegment(checked)
			if loopIndex == 0 && segmentIndex == 0 {
				anchor = start
			}
		}
	}
	pre := momentPreflight{
		record: ProfileRecord{Outer: normalized[0], Holes: normalized[1:]},
		anchor: anchor,
		plans:  plans,
		work:   work,
	}
	if !freeform {
		return pre, nil
	}
	// The reconstruction's charge is the record's, so it is levied here — once,
	// over the whole scene that pass will arrange — rather than per segment. It
	// comes after the per-segment charges because those bound the conversion each
	// segment has ALREADY run above, and it comes before validateMomentRecord asks
	// sketch anything at all, which is what the ceiling is for: the public
	// ProfileRecord methods take no context, so a charge levied after the
	// reconstruction bounds nothing it was added to bound.
	//
	// The placement is checkable, and the claim is that every charge precedes the
	// arrangement it pays for. This record-wide charge sits ahead of both
	// arrangements validation always runs; each candidate's own re-arrangement is
	// charged immediately before its RecordProfile call
	// (momentRecordMatchesSketch); and every free-form conversion is charged
	// before its rational lift (spline_bezier.go). What runs AHEAD of this charge
	// is the loop above, and each segment in it levies its own size-derived linear
	// floor before scanning its own arrays, so that loop is bounded by this same
	// ceiling rather than excluded from it. Its ORDER is what
	// docs/spline-design.md §5.2 requires: a segment's tier is decided before its
	// CONVERSION charge, so hoisting a record-wide charge ahead of per-segment
	// validation would hand a valid rational NURBS the R7 ceiling instead of its
	// own Table R reason.
	//
	// A record with a large analytic prefix is expensive on its own account, not
	// because of this charge. 500,000 line segments plus one minimal spline take
	// 2.641 s and 1,976,749 KiB here, against 2.541 s and 1,961,167 KiB with this
	// charge removed, and either way the refusal lands at the same trailing
	// segment after the same full scan. That cost is extrude.go's lineWalkBounds
	// doing big.Rat arithmetic through walkOf, once per segment, which no charge
	// placement here changes. An analytic-only record returns above without
	// levying this charge, and validateMomentRecord levies the identical amount
	// on the identical counter instead. That split is not a second ceiling — only
	// one of the two ever fires for a given record. Its reason is the exact
	// whole-circle certificate, which validateMomentRecord answers from disk
	// containment alone and which runs no arrangement at all, so charging it here
	// would refuse a record no reconstruction ever reads. No free-form record can
	// reach that certificate, so the free-form charge stays here, where it also
	// covers an evaluator preflight that never calls validateMomentRecord.
	arrangement, err := chargeFreeformReconstruction(pre.record, work)
	if err != nil {
		return momentPreflight{}, fmt.Errorf(`decad: profile record is invalid: %w`, err)
	}
	pre.arrangement = arrangement
	return pre, nil
}

func scaleMomentRecordForValidation(record ProfileRecord, anchor Point2) (ProfileRecord, error) {
	scale := 0.0
	grow := func(point Point2) {
		scale = math.Max(scale, math.Abs(point.U-anchor.U))
		scale = math.Max(scale, math.Abs(point.V-anchor.V))
	}
	for _, loop := range append([]LoopRecord{record.Outer}, record.Holes...) {
		for _, segment := range loop.Segments {
			switch segment := segment.(type) {
			case LineSeg:
				grow(segment.Start)
				grow(segment.End)
			case CircleSeg:
				grow(segment.Center)
				radius, _ := segment.Radius.In(units.Millimeter)
				scale = math.Max(scale, radius)
			case ArcSeg:
				grow(segment.Center)
				grow(segment.Start)
				grow(segment.End)
			case SplineSeg:
				growAll(segment.Control, grow)
			case ClosedSplineSeg:
				growAll(segment.Control, grow)
			case NURBSSeg:
				growAll(segment.Control, grow)
			case FitSplineSeg:
				growAll(segment.Fit, grow)
			}
		}
	}
	if scale == 0 {
		return ProfileRecord{}, fmt.Errorf(`%w: a recorded region has no geometric extent`, ErrDegenerate)
	}
	if !finiteMomentValues(scale) {
		return ProfileRecord{}, fmt.Errorf(`%w: the recorded region's extent is not finite`, ErrNotFinite)
	}

	transform := func(point Point2) Point2 {
		return Point2{U: (point.U - anchor.U) / scale, V: (point.V - anchor.V) / scale}
	}
	loops := append([]LoopRecord{record.Outer}, record.Holes...)
	scaled := make([]LoopRecord, len(loops))
	for loopIndex, loop := range loops {
		scaled[loopIndex].Segments = make([]CurveSegment, len(loop.Segments))
		for segmentIndex, segment := range loop.Segments {
			switch segment := segment.(type) {
			case LineSeg:
				segment.Start = transform(segment.Start)
				segment.End = transform(segment.End)
				scaled[loopIndex].Segments[segmentIndex] = segment
			case CircleSeg:
				radius, _ := segment.Radius.In(units.Millimeter)
				segment.Center = transform(segment.Center)
				segment.Radius = units.Millimeters(radius / scale)
				scaled[loopIndex].Segments[segmentIndex] = segment
			case ArcSeg:
				segment.Center = transform(segment.Center)
				segment.Start = transform(segment.Start)
				segment.End = transform(segment.End)
				scaled[loopIndex].Segments[segmentIndex] = segment
			case SplineSeg:
				// A B-spline is affine invariant, so transforming its control
				// points transforms the curve — the rescaled record states the
				// same shape at unit scale.
				segment.Control = shiftPoints(segment.Control, transform)
				scaled[loopIndex].Segments[segmentIndex] = segment
			case ClosedSplineSeg:
				segment.Control = shiftPoints(segment.Control, transform)
				scaled[loopIndex].Segments[segmentIndex] = segment
			case NURBSSeg:
				segment.Control = shiftPoints(segment.Control, transform)
				scaled[loopIndex].Segments[segmentIndex] = segment
			case FitSplineSeg:
				// A chord-length natural cubic is equivariant under a similarity: this
				// transform is a UNIFORM scale plus a translation, so every chord
				// scales by the same factor, the parameterization is the same up to
				// that factor, and the interpolant is the image of the original curve.
				//
				// One caveat, a consequence of the 1e-12 fit-point dedup threshold
				// (spline_fit.go) being ABSOLUTE: a rescale can collapse a fit-point
				// pair the original record kept, or keep a pair it collapsed. That is
				// confined to THIS falsifier — decad always integrates the ORIGINAL
				// record's interpolant, never the rescaled one — so the worst it can
				// do is weaken this check into a no-match refusal, or into one that
				// falsifies less than it might have. It can never move a published
				// number.
				segment.Fit = shiftPoints(segment.Fit, transform)
				scaled[loopIndex].Segments[segmentIndex] = segment
			}
		}
	}
	return ProfileRecord{Outer: scaled[0], Holes: scaled[1:]}, nil
}

func growAll(points []Point2, grow func(Point2)) {
	for _, point := range points {
		grow(point)
	}
}

// validateMomentSegment normalizes one recorded segment and returns it beside
// the walk's own start point — the anchor the integrator re-references its
// moments to. A free-form segment resolves that point from its converted Bézier
// chain rather than through walkOf, which still refuses free-form kinds: the
// walk vocabulary carries no free-form state yet, and a free-form walk that
// merely read as non-circular would be built as a straight line.
func validateMomentSegment(segment CurveSegment, work *freeformWork) (CurveSegment, Point2, freeformPlan, error) {
	segment, err := normalizeSegment(segment)
	if err != nil {
		return nil, Point2{}, freeformPlan{}, err
	}
	if segment == nil {
		return nil, Point2{}, freeformPlan{}, errNilSegment
	}
	if isFreeformSegment(segment) {
		return validateFreeformMomentSegment(segment, work)
	}
	checked, start, err := validateAnalyticMomentSegment(segment, work)
	return checked, start, freeformPlan{}, err
}

// validateAnalyticMomentSegment normalizes and checks one line, circle or arc
// segment — the kinds integrated from their own closed forms, with no
// conversion and so no charge against the record's work counter.
func validateAnalyticMomentSegment(segment CurveSegment, work *freeformWork) (CurveSegment, Point2, error) {
	switch segment := segment.(type) {
	case LineSeg:
		if !finiteMomentValues(
			segment.Start.U,
			segment.Start.V,
			segment.End.U,
			segment.End.V,
			segment.TStart,
			segment.TEnd,
		) {
			return nil, Point2{}, fmt.Errorf(`%w: a line segment field is not finite`, ErrNotFinite)
		}
		if err := validateMomentRange(segment.TStart, segment.TEnd); err != nil {
			return nil, Point2{}, err
		}
	case CircleSeg:
		radius, err := magnitudeIn(segment.Radius, units.Length, units.Millimeter, "a circle segment's radius")
		if err != nil {
			return nil, Point2{}, err
		}
		if !finiteMomentValues(segment.Center.U, segment.Center.V, segment.TStart, segment.TEnd) {
			return nil, Point2{}, fmt.Errorf(`%w: a circle segment field is not finite`, ErrNotFinite)
		}
		if err := validateMomentRange(segment.TStart, segment.TEnd); err != nil {
			return nil, Point2{}, err
		}
		if segment.CCW != (segment.TStart < segment.TEnd) {
			return nil, Point2{}, fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, ErrDegenerate)
		}
		segment.Radius = units.Millimeters(radius)
		return validateMomentWalk(segment, work)
	case ArcSeg:
		if !finiteMomentValues(
			segment.Center.U,
			segment.Center.V,
			segment.Start.U,
			segment.Start.V,
			segment.End.U,
			segment.End.V,
			segment.TStart,
			segment.TEnd,
		) {
			return nil, Point2{}, fmt.Errorf(`%w: an arc segment field is not finite`, ErrNotFinite)
		}
		if err := validateMomentRange(segment.TStart, segment.TEnd); err != nil {
			return nil, Point2{}, err
		}
		startRadius := math.Hypot(segment.Start.U-segment.Center.U, segment.Start.V-segment.Center.V)
		endRadius := math.Hypot(segment.End.U-segment.Center.U, segment.End.V-segment.Center.V)
		if !finiteMomentValues(startRadius, endRadius) {
			return nil, Point2{}, fmt.Errorf(`%w: an arc segment's derived radius is not finite`, ErrNotFinite)
		}
		if !momentCoordinateJoins(startRadius, endRadius) {
			return nil, Point2{}, fmt.Errorf(
				`%w: an arc segment's pinned start and end radii differ (%g and %g)`,
				ErrDegenerate,
				startRadius,
				endRadius,
			)
		}
	default:
		return nil, Point2{}, fmt.Errorf(
			`%w: this evaluator computes mass properties over line, arc, circle and Tier A free-form profile segments only; the profile has a %T segment`,
			ErrUnsupported,
			segment,
		)
	}
	return validateMomentWalk(segment, work)
}

// validateFreeformMomentSegment checks the fields the exact integrator reads on
// a Tier A free-form segment and returns the walk's own start point beside the
// converted chain the moments pass will integrate. The conversion itself is the
// check: it rejects a non-finite range or control coordinate, a trimmed range, a
// rational NURBS and every non-Tier-A kind with its own reason
// (docs/spline-design.md Table R), so no field test is duplicated here.
//
// Every charge this SEGMENT owes is levied here, on the record's own counter:
// the conversion, the re-anchoring the moments pass performs, and the exact
// integration it then runs. The R7 ceiling exists because the public
// ProfileRecord methods take no context and so cannot be cancelled, and a
// ceiling consulted at the point of use fires after the work it is there to
// bound has already run. The sketch RECONSTRUCTION is charged separately, by
// the record-level preflight above: it arranges the whole scene at once, so its
// cost is a property of the record rather than of any segment in it.
func validateFreeformMomentSegment(segment CurveSegment, work *freeformWork) (CurveSegment, Point2, freeformPlan, error) {
	spans, reversed, err := freeformBezierSpans(segment, work)
	if err != nil {
		return nil, Point2{}, freeformPlan{}, err
	}
	if err := chargeFreeformShift(spans, work); err != nil {
		return nil, Point2{}, freeformPlan{}, err
	}
	if err := chargeFreeformSpans(spans, work); err != nil {
		return nil, Point2{}, freeformPlan{}, err
	}
	start, end, err := freeformEndpoints(spans, reversed)
	if err != nil {
		return nil, Point2{}, freeformPlan{}, err
	}
	if !finiteMomentValues(start.U, start.V) {
		return nil, Point2{}, freeformPlan{}, fmt.Errorf(`%w: a free-form segment's start point is not finite`, ErrNotFinite)
	}
	if freeformDegenerate(spans) {
		return nil, Point2{}, freeformPlan{}, fmt.Errorf(`%w: a free-form segment whose control points all coincide contributes no boundary`, ErrDegenerate)
	}
	if err := requireFitSplineTerminalJoins(segment, start, end, reversed); err != nil {
		return nil, Point2{}, freeformPlan{}, err
	}
	return segment, start, freeformPlan{spans: spans, reversed: reversed}, nil
}

// requireFitSplineTerminalJoins falsifies a FitSplineSeg whose converted chain
// does not actually reach the fit point its own record names as the walk's
// natural-end coordinate — the point a following segment's Start (or, for a
// loop's own last segment, the loop's first segment's Start) is recorded
// against. sketch's fit-point dedup (fitChordEps, geom/fitspline.go) collapses
// a run of fit points closer than 1e-12, keeping only the FIRST of each run,
// so a terminal fit point sitting that close to its predecessor is dropped and
// geom.NewFitInterpolant's own chain ends one point short of what the record
// still claims — while the record's own natural-START point (always the first
// of its own run) can never be dropped this way, so only the natural-end side
// needs checking. This is the one Tier A kind that can drift here at all
// (docs/spline-design.md Table F): a clamped SplineSeg/ClosedSplineSeg/
// NURBSSeg interpolates its own recorded end control point exactly, with no
// sketch-side collapsing on the way, so a general cross-kind join check would
// only add unearned cost for those three.
//
// sketch's own reconstruction (momentRecordMatchesSketch) cannot catch this on
// its own: it rebuilds the SAME entity from the SAME recorded fit points, so a
// collapsed terminal point is dropped identically on both sides and the round
// trip matches regardless.
func requireFitSplineTerminalJoins(segment CurveSegment, start, end Point2, reversed bool) error {
	fit, ok := segment.(FitSplineSeg)
	if !ok {
		return nil
	}
	// The walk's natural-end coordinate is the recorded chain's own LAST fit
	// point; freeformEndpoints already swapped start/end into walk order for a
	// reversed range, so the natural-end side is start there instead of end.
	want := fit.Fit[len(fit.Fit)-1]
	got := end
	if reversed {
		got = start
	}
	if got == want {
		return nil
	}
	return fmt.Errorf(
		`%w: the converted fit-spline curve's own boundary reaches (%v, %v), not the recorded terminal fit point (%v, %v) — sketch's fit-point dedup collapsed it`,
		ErrDegenerate,
		got.U,
		got.V,
		want.U,
		want.V,
	)
}

// freeformDegenerate reports whether every control point of the converted chain
// is the same coordinate. The convex hull property makes that the exact
// condition for the curve to be a single point, so the test is a proof rather
// than a tolerance.
//
// It is this path's half of Table R row R14. The length bracket refuses the
// same record on its own terms — a collapsed net is the one shape whose bracket
// has zero width (spline_length.go) — so the two paths agree.
func freeformDegenerate(spans []bezierSpan) bool {
	if len(spans) == 0 || len(spans[0]) == 0 {
		return true
	}
	first := spans[0][0]
	for _, span := range spans {
		for _, point := range span {
			if point.u.Cmp(first.u) != 0 || point.v.Cmp(first.v) != 0 {
				return false
			}
		}
	}
	return true
}

func validateMomentWalk(segment CurveSegment, work *freeformWork) (CurveSegment, Point2, error) {
	walk, err := walkOf(segment, work)
	if err != nil {
		return nil, Point2{}, err
	}
	if !finiteMomentValues(
		walk.startU,
		walk.startV,
		walk.endU,
		walk.endV,
		walk.tanInU,
		walk.tanInV,
		walk.tanOutU,
		walk.tanOutV,
		walk.length,
		walk.cU,
		walk.cV,
		walk.radius,
		walk.th0,
		walk.th1,
	) {
		return nil, Point2{}, fmt.Errorf(`%w: a segment's derived walk is not finite`, ErrNotFinite)
	}
	if walk.length <= 0 {
		return nil, Point2{}, fmt.Errorf(`%w: a zero-length segment contributes no boundary`, ErrDegenerate)
	}
	return segment, Point2{U: walk.startU, V: walk.startV}, nil
}

func validateMomentRange(start, end float64) error {
	if start < 0 || start > 1 || end < 0 || end > 1 {
		return fmt.Errorf(`%w: a segment range must stay within [0, 1]`, ErrDegenerate)
	}
	if start == end {
		return fmt.Errorf(`%w: a zero-length segment range contributes no boundary`, ErrDegenerate)
	}
	return nil
}

func momentCoordinateJoins(a, b float64) bool {
	scale := math.Max(math.Abs(a), math.Abs(b))
	if scale == 0 {
		return true
	}
	ulp := scale - math.Nextafter(scale, 0)
	return math.Abs(a-b) <= 1024*ulp
}

// validateWholeCircleRegion keeps exact whole-circle regions independent of
// sketch's proximity threshold. The topology is only containment of the outer
// disk and pairwise separation of the hole disks, so no arrangement machinery
// is needed.
func validateWholeCircleRegion(record ProfileRecord) (bool, error) {
	loops := append([]LoopRecord{record.Outer}, record.Holes...)
	circles := make([]CircleSeg, len(loops))
	for loopIndex, loop := range loops {
		if len(loop.Segments) != 1 {
			return false, nil
		}
		circle, ok := loop.Segments[0].(CircleSeg)
		if !ok {
			return false, nil
		}
		if circle.TStart != 0 || circle.TEnd != 1 {
			if circle.TStart != 1 || circle.TEnd != 0 {
				// Partial circle fragments are handled by the normal topology
				// reconstruction and integration path below.
				return false, nil
			}
		}
		wantCCW := loopIndex == 0
		if circle.CCW != wantCCW {
			return true, fmt.Errorf(`%w: profile loop %d has the wrong winding`, ErrDegenerate, loopIndex)
		}
		circles[loopIndex] = circle
	}

	outerRadius, _ := circles[0].Radius.In(units.Millimeter)
	for holeIndex, hole := range circles[1:] {
		holeRadius, _ := hole.Radius.In(units.Millimeter)
		distance := math.Hypot(hole.Center.U-circles[0].Center.U, hole.Center.V-circles[0].Center.V)
		if !finiteMomentValues(distance) {
			return true, fmt.Errorf(`%w: a circle separation is not finite`, ErrNotFinite)
		}
		if holeRadius >= outerRadius || distance >= outerRadius-holeRadius {
			return true, fmt.Errorf(`%w: profile hole %d is not contained by the outer circle`, ErrDegenerate, holeIndex)
		}
	}
	for a := 1; a < len(circles); a++ {
		radiusA, _ := circles[a].Radius.In(units.Millimeter)
		for b := a + 1; b < len(circles); b++ {
			radiusB, _ := circles[b].Radius.In(units.Millimeter)
			minimum := radiusA + radiusB
			distance := math.Hypot(
				circles[a].Center.U-circles[b].Center.U,
				circles[a].Center.V-circles[b].Center.V,
			)
			if !finiteMomentValues(minimum, distance) {
				return true, fmt.Errorf(`%w: a circle separation is not finite`, ErrNotFinite)
			}
			if distance <= minimum {
				return true, fmt.Errorf(`%w: profile holes overlap, touch or nest`, ErrDegenerate)
			}
		}
	}
	return true, nil
}

type momentEntityKey struct {
	kind   uint8
	first  Point2
	second Point2
	third  Point2
	radius float64
	// control identifies a free-form entity by its own defining data, which
	// no fixed number of Point2 fields can hold.
	control string
}

// analyticEntityKey is the interning key momentRecordScene builds an analytic
// segment's entity under, read off the segment's own defining data in constant
// time. Both that scene and reconstructionOf's chord count share it, so the
// charge cannot drift from the set of entities the arrangement actually holds.
//
// It reports false for every free-form kind. Those key on freeformEntityKey,
// which walks all the control points and allocates a string per segment — a pass
// the reconstruction charge must PRECEDE rather than run — so the chord count
// leaves them un-interned and counts each fragment for itself.
func analyticEntityKey(segment CurveSegment) (momentEntityKey, bool) {
	switch segment := segment.(type) {
	case LineSeg:
		return momentEntityKey{kind: 1, first: segment.Start, second: segment.End}, true
	case CircleSeg:
		radius, _ := segment.Radius.In(units.Millimeter)
		return momentEntityKey{kind: 2, first: segment.Center, radius: radius}, true
	case ArcSeg:
		return momentEntityKey{
			kind:   3,
			first:  segment.Center,
			second: segment.Start,
			third:  segment.End,
		}, true
	default:
		return momentEntityKey{}, false
	}
}

// freeformEntityKey renders a free-form segment's defining data as a key. It
// keys on the entity's OWN fields — control points, and a NURBS's degree, knots
// and weights — so two recorded segments dedupe exactly when they name the same
// entity.
func freeformEntityKey(kind uint8, points []Point2, extra ...float64) momentEntityKey {
	var b strings.Builder
	for _, point := range points {
		fmt.Fprintf(&b, "%v,%v;", point.U, point.V)
	}
	b.WriteByte('|')
	for _, value := range extra {
		fmt.Fprintf(&b, "%v;", value)
	}
	return momentEntityKey{kind: kind, control: b.String()}
}

// normalizeReconstructionWeights rewrites every all-equal NURBS weight vector
// in a record to ones, for the sketch reconstruction only.
//
// Equal weights CANCEL in the homogeneous quotient, so the normalized segment
// names the very same curve — the exact integration reads the recorded weights
// and is untouched by this. What changes is the magnitude sketch's own evaluator
// works in: it squares the homogeneous denominator to differentiate, so weights
// past about sqrt(MaxFloat64) overflow it and the reconstruction yields no valid
// profile at all. A record whose weights are all 1e300 is then refused as a
// region that does not close, while the identical curve at weight 1 measures
// fine. The record is Tier A and owes an answer, so the reconstruction is asked
// about the representable spelling of the same curve.
//
// The normalized record is what the comparison below runs against too: sketch
// records the entity it was given, so a candidate carries the normalized weights
// and must be matched against a record carrying them as well.
func normalizeReconstructionWeights(record ProfileRecord) ProfileRecord {
	loops := append([]LoopRecord{record.Outer}, record.Holes...)
	out := make([]LoopRecord, len(loops))
	changed := false
	for loopIndex, loop := range loops {
		out[loopIndex] = loop
		for segmentIndex, segment := range loop.Segments {
			seg, ok := segment.(NURBSSeg)
			if !ok || !equalNURBSWeights(seg.Weights) || seg.Weights[0] == 1 {
				continue
			}
			if !changed {
				for i, source := range loops {
					out[i].Segments = slices.Clone(source.Segments)
				}
				changed = true
			}
			ones := make([]float64, len(seg.Weights))
			for i := range ones {
				ones[i] = 1
			}
			seg.Weights = ones
			out[loopIndex].Segments[segmentIndex] = seg
		}
	}
	if !changed {
		return record
	}
	return ProfileRecord{Outer: out[0], Holes: out[1:]}
}

// equalNURBSWeights reports whether every weight is the same value — the
// non-rational (Tier A) condition the exact conversion also tests.
func equalNURBSWeights(weights []float64) bool {
	if len(weights) == 0 {
		return false
	}
	for _, weight := range weights {
		if weight != weights[0] {
			return false
		}
	}
	return true
}

// momentRecordMatchesSketch asks sketch whether the recorded segments form the
// recorded region. It builds the scene, arranges it once to list the candidate
// profiles, and authenticates each candidate through RecordProfile.
//
// arrangement is one whole-scene arrangement's charge and work is the record's
// own counter (spline_bezier.go). The preflight already paid for this pass's own
// arrangement; each candidate costs one MORE, because RecordProfile
// authenticates against a fresh Sketch.Profiles and that rebuilds the whole
// arrangement. Charging each of them here, before it runs, is what keeps the
// ceiling a bound on the pass rather than on a prediction of it. Every record
// reaching this pass carries a positive arrangement charge, whatever its kinds:
// an analytic-only record is charged by chargeAnalyticReconstruction, so the
// loop is never free.
func momentRecordMatchesSketch(record ProfileRecord, work *freeformWork, arrangement uint64) (bool, error) {
	record = normalizeReconstructionWeights(record)
	s, built := momentRecordScene(record)
	if !built {
		return false, nil
	}
	for _, profile := range s.Profiles() {
		if !profile.Valid {
			continue
		}
		// One more whole-scene arrangement, charged before it runs.
		if err := work.step(arrangement); err != nil {
			return false, err
		}
		candidate, _, err := RecordProfile(s, profile)
		if err == nil && momentRecordsEqual(record, candidate) {
			return true, nil
		}
	}
	return false, nil
}

// momentRecordScene builds the sketch entities the record names, deduplicating
// the ones several segments share. It reports whether every entity was created:
// an entity sketch declines to build is a record that does not reconstruct, so
// the answer above is a no-match rather than a failure to report.
func momentRecordScene(record ProfileRecord) (*sketch.Sketch, bool) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		return nil, false
	}

	interned := make(map[Point2]*sketch.Point)
	point := func(value Point2) *sketch.Point {
		if existing, ok := interned[value]; ok {
			return existing
		}
		created := s.CreatePoint(value.U, value.V)
		interned[value] = created
		return created
	}
	points := func(values []Point2) []*sketch.Point {
		out := make([]*sketch.Point, len(values))
		for i, value := range values {
			out[i] = point(value)
		}
		return out
	}
	entities := make(map[momentEntityKey]struct{})
	for _, loop := range append([]LoopRecord{record.Outer}, record.Holes...) {
		for _, segment := range loop.Segments {
			switch segment := segment.(type) {
			case LineSeg:
				key, _ := analyticEntityKey(segment)
				if _, ok := entities[key]; !ok {
					s.CreateLine(point(segment.Start), point(segment.End))
					entities[key] = struct{}{}
				}
			case CircleSeg:
				key, _ := analyticEntityKey(segment)
				if _, ok := entities[key]; !ok {
					s.CreateCircle(point(segment.Center), key.radius)
					entities[key] = struct{}{}
				}
			case ArcSeg:
				key, _ := analyticEntityKey(segment)
				if _, ok := entities[key]; !ok {
					s.CreateArc(point(segment.Center), point(segment.Start), point(segment.End))
					entities[key] = struct{}{}
				}
			case SplineSeg:
				key := freeformEntityKey(4, segment.Control)
				if _, ok := entities[key]; !ok {
					if _, err := s.CreateSpline(points(segment.Control)...); err != nil {
						return nil, false
					}
					entities[key] = struct{}{}
				}
			case ClosedSplineSeg:
				key := freeformEntityKey(5, segment.Control)
				if _, ok := entities[key]; !ok {
					if _, err := s.CreateClosedSpline(points(segment.Control)...); err != nil {
						return nil, false
					}
					entities[key] = struct{}{}
				}
			case NURBSSeg:
				extra := append([]float64{float64(segment.Degree)}, segment.Knots...)
				extra = append(extra, segment.Weights...)
				key := freeformEntityKey(6, segment.Control, extra...)
				if _, ok := entities[key]; !ok {
					if _, err := s.CreateNURBS(segment.Degree, points(segment.Control), segment.Weights, segment.Knots); err != nil {
						return nil, false
					}
					entities[key] = struct{}{}
				}
			case FitSplineSeg:
				key := freeformEntityKey(7, segment.Fit)
				if _, ok := entities[key]; !ok {
					if _, err := s.CreateFitSpline(points(segment.Fit)...); err != nil {
						return nil, false
					}
					entities[key] = struct{}{}
				}
			default:
				return nil, false
			}
		}
	}
	return s, true
}

func momentRecordsEqual(a, b ProfileRecord) bool {
	if !momentLoopsEqual(a.Outer, b.Outer) || len(a.Holes) != len(b.Holes) {
		return false
	}
	matched := make([]bool, len(b.Holes))
	for _, holeA := range a.Holes {
		found := false
		for holeIndex, holeB := range b.Holes {
			if !matched[holeIndex] && momentLoopsEqual(holeA, holeB) {
				matched[holeIndex] = true
				found = true
				break
			}
		}
		if !found {
			return false
		}
	}
	return true
}

func momentLoopsEqual(a, b LoopRecord) bool {
	if len(a.Segments) != len(b.Segments) {
		return false
	}
	for offset := range b.Segments {
		equal := true
		for segmentIndex, segmentA := range a.Segments {
			if !momentSegmentsEqual(segmentA, b.Segments[(segmentIndex+offset)%len(b.Segments)]) {
				equal = false
				break
			}
		}
		if equal {
			return true
		}
	}
	return false
}

func momentSegmentsEqual(a, b CurveSegment) bool {
	switch a := a.(type) {
	case LineSeg:
		b, ok := b.(LineSeg)
		return ok && a == b
	case CircleSeg:
		b, ok := b.(CircleSeg)
		if !ok {
			return false
		}
		radiusA, _ := a.Radius.In(units.Millimeter)
		radiusB, _ := b.Radius.In(units.Millimeter)
		return a.Center == b.Center &&
			radiusA == radiusB &&
			a.CCW == b.CCW &&
			a.TStart == b.TStart &&
			a.TEnd == b.TEnd
	case ArcSeg:
		b, ok := b.(ArcSeg)
		return ok && a == b
	case SplineSeg:
		b, ok := b.(SplineSeg)
		return ok && slices.Equal(a.Control, b.Control) && a.TStart == b.TStart && a.TEnd == b.TEnd
	case ClosedSplineSeg:
		b, ok := b.(ClosedSplineSeg)
		return ok && slices.Equal(a.Control, b.Control) &&
			a.CCW == b.CCW && a.TStart == b.TStart && a.TEnd == b.TEnd
	case NURBSSeg:
		b, ok := b.(NURBSSeg)
		return ok && a.Degree == b.Degree &&
			slices.Equal(a.Control, b.Control) &&
			slices.Equal(a.Knots, b.Knots) &&
			slices.Equal(a.Weights, b.Weights) &&
			a.TStart == b.TStart && a.TEnd == b.TEnd
	case FitSplineSeg:
		b, ok := b.(FitSplineSeg)
		return ok && slices.Equal(a.Fit, b.Fit) && a.TStart == b.TStart && a.TEnd == b.TEnd
	default:
		return false
	}
}
