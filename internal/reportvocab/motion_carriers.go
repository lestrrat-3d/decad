package reportvocab

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/units"
)

// MotionRequest records the effective settings of a VerifyMotion call.
type MotionRequest struct {
	RelativeTolerance units.Value
	Resolution        units.Value
	MinClearance      *units.Value
}

// MotionInterval records the certificate between adjacent evaluated poses.
// Clearance is a proven lower bound only when Outcome is IntervalClear.
type MotionInterval struct {
	From, To  units.Value
	Outcome   IntervalOutcome
	Clearance *measurement.Measurement
}

// IntervalOutcome states what a MotionInterval proves.
type IntervalOutcome int

const (
	IntervalNotEvaluated IntervalOutcome = iota
	IntervalClear
	IntervalColliding
	IntervalUndecided
)

// String renders the stable lower-snake token, including unknown values.
func (o IntervalOutcome) String() string {
	switch o {
	case IntervalNotEvaluated:
		return tokenNotEvaluated
	case IntervalClear:
		return "clear"
	case IntervalColliding:
		return "colliding"
	case IntervalUndecided:
		return tokenUndecided
	default:
		return fmt.Sprintf("interval_outcome(%d)", int(o))
	}
}
