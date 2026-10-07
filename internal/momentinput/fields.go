package momentinput

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
)

// FieldPreflight carries one checked record and its converted free-form plans.
type FieldPreflight struct {
	Record      Profile
	Anchor      Point2
	Plans       map[[2]int]Plan
	Work        *freeform.FreeformWork
	Arrangement uint64
}

// ValidateFieldsWithPoll checks a whole record while spending its shared work
// state. Polling and charges keep their record, loop, and segment order.
func ValidateFieldsWithPoll(poll func() error, record Profile, work *freeform.FreeformWork) (FieldPreflight, error) {
	loops := append([]LoopRecord{record.Outer}, record.Holes...)
	normalized := make([]LoopRecord, len(loops))
	// Plan storage is minted on the first segment that converts, so a record
	// naming none allocates none.
	var plans map[[2]int]Plan
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
				return FieldPreflight{}, err
			}
		}
		loop := record.Outer
		if loopIndex > 0 {
			loop = record.Holes[loopIndex-1]
		}
		if len(loop.Segments) == 0 {
			return FieldPreflight{}, fmt.Errorf(
				`decad: profile loop %d is invalid: %w: a recorded loop holds no segments`,
				loopIndex,
				ErrDegenerate,
			)
		}
		normalized[loopIndex].Segments = make([]CurveSegment, len(loop.Segments))
		for segmentIndex, segment := range loop.Segments {
			if poll != nil {
				if err := poll(); err != nil {
					return FieldPreflight{}, err
				}
			}
			checked, start, plan, err := validateMomentSegment(segment, work)
			if err != nil {
				return FieldPreflight{}, fmt.Errorf(
					`decad: profile loop %d segment %d is invalid: %w`,
					loopIndex,
					segmentIndex,
					err,
				)
			}
			normalized[loopIndex].Segments[segmentIndex] = checked
			if len(plan.Spans) > 0 {
				if plans == nil {
					plans = make(map[[2]int]Plan)
				}
				plans[[2]int{loopIndex, segmentIndex}] = plan
			}
			freeform = freeform || splinebezier.IsFreeformSegment(checked)
			if loopIndex == 0 && segmentIndex == 0 {
				anchor = start
			}
		}
	}
	pre := FieldPreflight{
		Record: Profile{Outer: normalized[0], Holes: normalized[1:]},
		Anchor: anchor,
		Plans:  plans,
		Work:   work,
	}
	if !freeform {
		return pre, nil
	}
	// The reconstruction charge is levied once over the whole scene after
	// per-segment charges, and before any reconstruction. An analytic record
	// reaches the same charge through validateMomentRecord after its whole-circle
	// certificate. See docs/spline-design.md §5.2 for the charge order.
	arrangement, err := chargeReconstruction(pre.Record, work)
	if err != nil {
		return FieldPreflight{}, fmt.Errorf(`decad: profile record is invalid: %w`, err)
	}
	pre.Arrangement = arrangement
	return pre, nil
}
