package decad

import (
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
)

type freeformPlan = momentinput.Plan

func momentProfile(record ProfileRecord) momentinput.Profile { return record }

func validateFreeformMomentSegment(segment CurveSegment, work *freeform.FreeformWork) (CurveSegment, Point2, freeformPlan, error) {
	return momentinput.ValidateFreeformSegment(segment, work)
}

func validateMomentFields(record ProfileRecord) (momentinput.FieldPreflight, error) {
	return momentinput.ValidateFieldsWithPoll(nil, record, nil)
}
