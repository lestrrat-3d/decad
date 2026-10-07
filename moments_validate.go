package decad

import (
	"context"
	"fmt"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentvalidate"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// momentPreflight is one whole ProfileRecord's preflight: the checked record,
// the walk anchor the integrator re-references its moments to, and the converted
// Bézier chain of every free-form segment.
//
// It is the record-level step docs/spline-design.md §5's work ceilings need.
// ONE freeform.FreeformWork state runs through the entire record and every later phase
// that reads it. Its exact-rational and reconstruction counters each bound that
// record's aggregate work, rather than each segment or pass independently. The
// conversion, re-anchoring and exact integration charges use the former; the
// whole-scene arrangement uses the latter. Every charge is levied before its
// work runs. Public ProfileRecord methods take no context, so a late refusal
// cannot cancel or bound the work it was meant to prevent.
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
	// work is the record's own work state, still open — and it is the
	// OPERATION's where one was handed in, so a caller that spent part of either
	// ceiling on this record earlier reads what is left rather than a fresh one
	// (§5.2). The topology reconstruction re-arranges the whole scene once per
	// candidate profile it authenticates, so those arrangements are charged here
	// as they happen rather than predicted.
	work *freeform.FreeformWork
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
	spans    []survey2d.BezierSpan
	reversed bool
}

func momentProfile(record ProfileRecord) momentvalidate.Profile {
	return momentvalidate.Profile{Outer: record.Outer, Holes: record.Holes}
}

func profileFromMoment(record momentvalidate.Profile) ProfileRecord {
	return ProfileRecord{Outer: record.Outer, Holes: record.Holes}
}

func scaleMomentRecordForValidation(record ProfileRecord, anchor Point2) (ProfileRecord, error) {
	scaled, err := momentvalidate.ScaleForValidation(momentProfile(record), anchor)
	return profileFromMoment(scaled), err
}

func validateMomentSegment(segment CurveSegment, work *freeform.FreeformWork) (CurveSegment, Point2, freeformPlan, error) {
	checked, start, plan, err := momentvalidate.ValidateSegment(segment, work)
	return checked, start, freeformPlan{spans: plan.Spans, reversed: plan.Reversed}, err
}

func validateFreeformMomentSegment(segment CurveSegment, work *freeform.FreeformWork) (CurveSegment, Point2, freeformPlan, error) {
	checked, start, plan, err := momentvalidate.ValidateFreeformSegment(segment, work)
	return checked, start, freeformPlan{spans: plan.Spans, reversed: plan.Reversed}, err
}

func validateWholeCircleRegion(record ProfileRecord) (bool, error) {
	return momentvalidate.ValidateWholeCircleRegion(momentProfile(record))
}

func normalizeReconstructionWeights(record ProfileRecord) ProfileRecord {
	return profileFromMoment(momentvalidate.NormalizeReconstructionWeights(momentProfile(record)))
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
	work := freeform.NewFreeformWork()
	if err := chargeKnownOverBudgetAnalyticReconstruction(record, work); err != nil {
		return momentPreflight{}, fmt.Errorf(`decad: profile record is invalid: %w`, err)
	}
	pre, err := validateMomentFieldsWork(work, record)
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

// matchesSketch runs one reconstruction pass on the record's own reconstruction
// counter.
func (p momentPreflight) matchesSketch(record ProfileRecord) (bool, error) {
	return momentRecordMatchesSketch(record, p.work, p.arrangement)
}

// chargeAnalyticReconstruction levies the record-wide arrangement charge for a
// record the field preflight left uncharged, which is exactly the record holding
// no free-form segment. It is the SAME charge on the SAME record's
// reconstruction counter — the ceiling is one per record, never one per kind —
// and it is levied here because this is where an analytic record's
// reconstruction is decided: after the exact whole-circle certificate, which
// runs no arrangement and so owes none, and before sketch is asked anything at
// all.
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
	arrangement, err := chargeReconstruction(p.record, p.work)
	if err != nil {
		return fmt.Errorf(`decad: profile record is invalid: %w`, err)
	}
	p.arrangement = arrangement
	return nil
}

// chargeKnownOverBudgetAnalyticReconstruction refuses an analytic record whose
// reconstruction charge is already known to exceed its fixed ceiling before
// validateMomentFields derives each arc's exact interval. That interval work is
// part of a measurement, not the reconstruction charge, and a record that
// cannot reach reconstruction must not spend it first.
//
// The exact whole-circle certificate is exempt because it performs no sketch
// arrangement. Free-form records are also exempt: their conversion preflight
// owns their error precedence and charges the exact-rational counter before
// their record-level reconstruction charge is known.
func chargeKnownOverBudgetAnalyticReconstruction(record ProfileRecord, work *freeform.FreeformWork) error {
	if wholeCircleRecordShape(record) || !analyticRecord(record) {
		return nil
	}
	demand := reconstructionOf(record)
	if freeform.ReconstructionCostMul(2, demand.Arrangement) <= freeform.ReconstructionWorkLimit {
		return nil
	}
	_, err := chargeReconstruction(record, work)
	return err
}

// wholeCircleRecordShape reports whether record can take
// validateWholeCircleRegion's no-arrangement path after field validation.
func wholeCircleRecordShape(record ProfileRecord) bool {
	for _, loop := range append([]LoopRecord{record.Outer}, record.Holes...) {
		if len(loop.Segments) != 1 {
			return false
		}
		circle, ok := loop.Segments[0].(CircleSeg)
		if !ok {
			return false
		}
		if (circle.TStart == 0 && circle.TEnd == 1) || (circle.TStart == 1 && circle.TEnd == 0) {
			continue
		}
		return false
	}
	return true
}

// analyticRecord reports whether record contains only the fixed-size analytic
// segment kinds whose reconstruction demand can be read without conversion.
func analyticRecord(record ProfileRecord) bool {
	for _, loop := range append([]LoopRecord{record.Outer}, record.Holes...) {
		for _, segment := range loop.Segments {
			switch segment.(type) {
			case LineSeg, CircleSeg, ArcSeg:
			default:
				return false
			}
		}
	}
	return true
}

func validateMomentFields(record ProfileRecord) (momentPreflight, error) {
	return validateMomentFieldsBudget(nil, record)
}

// validateMomentFieldsWork is the preflight an evaluator runs when it already
// holds this record's work state: the charges below continue it rather than
// start fresh ceilings on the same record (docs/spline-design.md §5.2).
func validateMomentFieldsWork(work *freeform.FreeformWork, record ProfileRecord) (momentPreflight, error) {
	return validateMomentFieldsWithPoll(nil, record, work)
}

func validateMomentFieldsBudget(budget *proofbound.WorkBudget, record ProfileRecord) (momentPreflight, error) {
	return validateMomentFieldsWithPoll(func() error { return survey2d.WallBudgetStep(budget) }, record, freeform.NewFreeformWork())
}

func validateMomentFieldsContext(ctx context.Context, work *freeform.FreeformWork, record ProfileRecord) (momentPreflight, error) {
	return validateMomentFieldsWithPoll(ctx.Err, record, work)
}

// work holds the counters this record spends. It is a parameter rather than a
// local because each ceiling is the RECORD's across a whole operation: an
// evaluator that already charged this record's conversion passes the same work
// state back in, so a later phase spends what is left instead of a fresh ceiling.
func validateMomentFieldsWithPoll(poll func() error, record ProfileRecord, work *freeform.FreeformWork) (momentPreflight, error) {
	loops := append([]LoopRecord{record.Outer}, record.Holes...)
	normalized := make([]LoopRecord, len(loops))
	// Plan storage is minted on the first segment that converts, so a record
	// naming none allocates none (see momentPreflight.plans).
	var plans map[[2]int]freeformPlan
	if work == nil {
		// One work state for the whole record: each ceiling bounds the record's
		// total work in its own cost model, never each segment's own.
		work = freeform.NewFreeformWork()
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
	arrangement, err := chargeReconstruction(pre.record, work)
	if err != nil {
		return momentPreflight{}, fmt.Errorf(`decad: profile record is invalid: %w`, err)
	}
	pre.arrangement = arrangement
	return pre, nil
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

// momentRecordMatchesSketch asks sketch whether the recorded segments form the
// recorded region. It builds the scene, arranges it once to list the candidate
// profiles, and authenticates each candidate through RecordProfile.
//
// arrangement is one whole-scene arrangement's charge and work holds the
// record's own reconstruction counter (spline_bezier.go). The preflight already
// paid for this pass's own arrangement; each candidate costs one MORE, because RecordProfile
// authenticates against a fresh Sketch.Profiles and that rebuilds the whole
// arrangement. Charging each of them here, before it runs, is what keeps the
// ceiling a bound on the pass rather than on a prediction of it. Every record
// reaching this pass carries a positive arrangement charge, whatever its kinds:
// an analytic-only record is charged by chargeAnalyticReconstruction, so the
// loop is never free.
func momentRecordMatchesSketch(record ProfileRecord, work *freeform.FreeformWork, arrangement uint64) (bool, error) {
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
		if err := work.ReconstructionStep(arrangement); err != nil {
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
