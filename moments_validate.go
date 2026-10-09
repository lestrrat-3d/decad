package decad

import (
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
)

type freeformPlan = momentinput.Plan

func validateFreeformMomentSegment(segment curveSegment, work *freeform.FreeformWork) (curveSegment, Point2, freeformPlan, error) {
	return momentinput.ValidateFreeformSegment(segment, work)
}

func validateMomentFields(record profileRecord) (momentinput.FieldPreflight, error) {
	return momentinput.ValidateFieldsWithPoll(nil, record, nil)
}
