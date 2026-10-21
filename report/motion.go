package report

import (
	"fmt"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// MotionReportOf is VerifyMotion's path report (agent-docs/motion-check-design.md
// §4). B is the body reference type and M the motion type the call stated.
type MotionReportOf[B comparable, M any] struct {
	Request     MotionRequest
	Motion      M
	Moving      []B
	Against     []B
	Poses       []PoseResultOf[B]
	Intervals   []MotionInterval
	Collisions  []CollisionOf[B]
	Clearance   *ScalarReading
	Assessment  Assessment
	Diagnostics []DiagnosticOf[B, JointCell]
	Status      Status
}

// Passed reports whether the report is Sound. It returns false for nil.
func (r *MotionReportOf[B, M]) Passed() bool { return r != nil && r.Status == Sound }

// MotionRequest records the effective settings of a VerifyMotion call.
type MotionRequest struct {
	RelativeTolerance units.Value
	Resolution        units.Value
	MinClearance      *units.Value
}

// PoseResultOf records one evaluated pose and its pair findings.
type PoseResultOf[B comparable] struct {
	At            units.Value
	Pose          r3.Transform
	Interferences []InterferenceOf[B]
	Clearances    []ClearanceOf[B]
	Diagnostics   []DiagnosticOf[B, JointCell]
}

// MotionInterval records the certificate between adjacent poses.
type MotionInterval struct {
	From, To  units.Value
	Outcome   IntervalOutcome
	Clearance *Measurement
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
		return "not_evaluated"
	case IntervalClear:
		return "clear"
	case IntervalColliding:
		return "colliding"
	case IntervalUndecided:
		return "undecided"
	default:
		return fmt.Sprintf("interval_outcome(%d)", int(o))
	}
}

// CollisionOf is a proven overlap at one ideal pose.
type CollisionOf[B comparable] struct {
	At     units.Value
	Pose   r3.Transform
	Moving B
	Static B
	Volume Measurement
}
