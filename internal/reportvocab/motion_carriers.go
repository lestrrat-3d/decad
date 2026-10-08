package reportvocab

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// MotionReport records VerifyMotion's path, pair results, and verdict.
// Its body and cell parameters preserve the caller's own identities.
type MotionReport[BodyT comparable, CellT any] struct {
	Request     MotionRequest
	Motion      motionbound.Motion
	Moving      []BodyT
	Against     []BodyT
	Poses       []PoseResult[BodyT, CellT]
	Intervals   []MotionInterval
	Collisions  []Collision[BodyT]
	Clearance   *ScalarReading
	Assessment  Assessment
	Diagnostics []Diagnostic[BodyT, CellT]
	Status      Status
}

// Passed reports whether the report is Sound. It returns false for nil.
func (r *MotionReport[BodyT, CellT]) Passed() bool {
	return r != nil && r.Status == Sound
}

// MotionRequest records the effective settings of a VerifyMotion call.
type MotionRequest struct {
	RelativeTolerance units.Value
	Resolution        units.Value
	MinClearance      *units.Value
}

// PoseResult records one evaluated pose and its pair findings.
type PoseResult[BodyT comparable, CellT any] struct {
	At            units.Value
	Pose          r3.Transform
	Interferences []Interference[BodyT]
	Clearances    []Clearance[BodyT]
	Diagnostics   []Diagnostic[BodyT, CellT]
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

// Collision is a proven overlap at one ideal pose. Volume's lower end
// includes the allowance from the measured float pose to the ideal pose.
type Collision[BodyT comparable] struct {
	At     units.Value
	Pose   r3.Transform
	Moving BodyT
	Static BodyT
	Volume measurement.Measurement
}
