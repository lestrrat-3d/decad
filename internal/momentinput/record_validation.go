package momentinput

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sketchrecord"
)

// ValidateRecord checks fields and then asks sketch to authenticate the
// recorded region. Whole circles use their direct containment certificate.
func ValidateRecord(record Profile) (FieldPreflight, error) {
	work := freeform.NewFreeformWork()
	if err := chargeKnownOverBudgetAnalyticReconstruction(record, work); err != nil {
		return FieldPreflight{}, fmt.Errorf(`decad: profile record is invalid: %w`, err)
	}
	pre, err := ValidateFieldsWithPoll(nil, record, work)
	if err != nil {
		return FieldPreflight{}, err
	}
	if circular, err := validateWholeCircleRegion(pre.Record); circular {
		if err != nil {
			return FieldPreflight{}, err
		}
		return pre, nil
	}
	if err := pre.chargeAnalyticReconstruction(); err != nil {
		return FieldPreflight{}, err
	}
	matched, err := pre.matchesSketch(pre.Record)
	if err != nil {
		return FieldPreflight{}, err
	}
	if !matched {
		validationRecord, err := scaleMomentRecordForValidation(pre.Record, pre.Anchor)
		if err != nil {
			return FieldPreflight{}, err
		}
		matched, err = pre.matchesSketch(validationRecord)
		if err != nil {
			return FieldPreflight{}, err
		}
		if !matched {
			return FieldPreflight{}, fmt.Errorf(
				`%w: the recorded segments do not form the stated closed region`, ErrDegenerate)
		}
	}
	return pre, nil
}

// PlanAt returns the converted chain of one free-form segment, or the zero
// plan for an analytic segment.
func (p FieldPreflight) PlanAt(loopIndex, segmentIndex int) Plan {
	return p.Plans[[2]int{loopIndex, segmentIndex}]
}

func (p FieldPreflight) matchesSketch(record Profile) (bool, error) {
	record = normalizeReconstructionWeights(record)
	s, built := momentRecordScene(record)
	if !built {
		return false, nil
	}
	for _, profile := range s.Profiles() {
		if !profile.Valid {
			continue
		}
		if err := p.Work.ReconstructionStep(p.Arrangement); err != nil {
			return false, err
		}
		trusted, err := sketchrecord.AdmitProfile(s, profile)
		if err != nil {
			continue
		}
		outer, holes, err := sketchrecord.RecordProfileLoops(trusted)
		if err == nil && momentRecordsEqual(record, Profile{Outer: outer, Holes: holes}) {
			return true, nil
		}
	}
	return false, nil
}

func (p *FieldPreflight) chargeAnalyticReconstruction() error {
	if p.Arrangement != 0 {
		return nil
	}
	arrangement, err := chargeReconstruction(p.Record, p.Work)
	if err != nil {
		return fmt.Errorf(`decad: profile record is invalid: %w`, err)
	}
	p.Arrangement = arrangement
	return nil
}

func chargeKnownOverBudgetAnalyticReconstruction(record Profile, work *freeform.FreeformWork) error {
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

func wholeCircleRecordShape(record Profile) bool {
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

func analyticRecord(record Profile) bool {
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
