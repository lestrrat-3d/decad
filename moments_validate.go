package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"
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
	plans map[[2]int]momentinput.Plan
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
	spans    []freeform.BezierSpan
	reversed bool
}

func momentProfile(record ProfileRecord) momentinput.Profile {
	return momentinput.Profile{Outer: record.Outer, Holes: record.Holes}
}

func profileFromMoment(record momentinput.Profile) ProfileRecord {
	return ProfileRecord{Outer: record.Outer, Holes: record.Holes}
}

func scaleMomentRecordForValidation(record ProfileRecord, anchor Point2) (ProfileRecord, error) {
	scaled, err := momentinput.ScaleForValidation(momentProfile(record), anchor)
	return profileFromMoment(scaled), err
}

func validateFreeformMomentSegment(segment CurveSegment, work *freeform.FreeformWork) (CurveSegment, Point2, freeformPlan, error) {
	checked, start, plan, err := momentinput.ValidateFreeformSegment(segment, work)
	return checked, start, freeformPlan{spans: plan.Spans, reversed: plan.Reversed}, err
}

func validateWholeCircleRegion(record ProfileRecord) (bool, error) {
	return momentinput.ValidateWholeCircleRegion(momentProfile(record))
}

func normalizeReconstructionWeights(record ProfileRecord) ProfileRecord {
	return profileFromMoment(momentinput.NormalizeReconstructionWeights(momentProfile(record)))
}

// planAt returns the plan the preflight converted for one segment. A segment
// the preflight recorded no plan for — every line, arc and circle — reads the
// zero plan, which is what the moments pass integrates from its own closed form.
func (p momentPreflight) planAt(loopIndex, segmentIndex int) freeformPlan {
	plan := p.plans[[2]int{loopIndex, segmentIndex}]
	return freeformPlan{spans: plan.Spans, reversed: plan.Reversed}
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

func validateMomentFieldsWithPoll(poll func() error, record ProfileRecord, work *freeform.FreeformWork) (momentPreflight, error) {
	pre, err := momentinput.ValidateFieldsWithPoll(poll, momentProfile(record), work)
	if err != nil {
		return momentPreflight{}, err
	}
	return momentPreflight{
		record:      profileFromMoment(pre.Record),
		anchor:      pre.Anchor,
		plans:       pre.Plans,
		work:        pre.Work,
		arrangement: pre.Arrangement,
	}, nil
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
	s, built := momentinput.RecordScene(momentProfile(record))
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
		if err == nil && momentinput.RecordsEqual(momentProfile(record), momentProfile(candidate)) {
			return true, nil
		}
	}
	return false, nil
}
