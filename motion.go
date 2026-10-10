package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/motionoption"
	"github.com/lestrrat-3d/decad/internal/reportvocab"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the motion vocabulary of docs/motion-check-design.md §2-§4:
// public aliases for the sealed Motion set, VerifyMotion's options, and
// root-owned MotionReport records. motion_verify.go runs the check, and
// internal/motionbound proves the bounds its interval certificate consumes.

// Motion is the sealed set of one-parameter rigid motions. PoseAt returns
// the rigid transform at a typed parameter value.
type Motion = motionbound.Motion

// Revolute rotates a moving set about the axis through Center. From and To
// are signed angles.
type Revolute = motionbound.Revolute

// Prismatic translates a moving set along Dir. From and To are signed
// lengths.
type Prismatic = motionbound.Prismatic

// Between joins From and To along the shorter rigid screw path. Its
// parameter is a dimensionless fraction.
type Between = motionbound.Between

// JointBoxOption configures VerifyJointBox.
type JointBoxOption = motionoption.JointBoxOption

// MotionOption configures VerifyMotion and VerifyLinkage.
type MotionOption = motionoption.MotionOption

// WithMotionTolerance sets the relative tolerance for motion readings.
func WithMotionTolerance(rel units.Value) MotionOption { return motionoption.WithMotionTolerance(rel) }

// WithResolution sets the finest parameter step for verdicts and readings.
func WithResolution(step units.Value) MotionOption { return motionoption.WithResolution(step) }

// WithMinClearance sets the minimum gap over the path.
func WithMinClearance(minimum units.Value) MotionOption {
	return motionoption.WithMinClearance(minimum)
}

type motionConfig = motionoption.Config

// MotionReport is VerifyMotion's path report (docs/motion-check-design.md §4).
type MotionReport struct {
	Request     MotionRequest
	Motion      Motion
	Moving      []*Body
	Against     []*Body
	Poses       []PoseResult
	Intervals   []MotionInterval
	Collisions  []Collision
	Clearance   *ScalarReading
	Assessment  Assessment
	Diagnostics []Diagnostic
	Status      Status
}

// Passed reports whether the report is Sound. It returns false for nil.
func (r *MotionReport) Passed() bool { return r != nil && r.Status == Sound }

// MotionRequest records the effective settings of a VerifyMotion call.
type MotionRequest struct {
	RelativeTolerance units.Value
	Resolution        units.Value
	MinClearance      *units.Value
}

// PoseResult records one evaluated pose and its pair findings.
type PoseResult struct {
	At            units.Value
	Pose          r3.Transform
	Interferences []Interference
	Clearances    []Clearance
	Diagnostics   []Diagnostic
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

func rootMotionRequest(r reportvocab.MotionRequest) MotionRequest {
	return MotionRequest{
		RelativeTolerance: r.RelativeTolerance,
		Resolution:        r.Resolution,
		MinClearance:      r.MinClearance,
	}
}

func rootMotionIntervals(intervals []reportvocab.MotionInterval) []MotionInterval {
	if intervals == nil {
		return nil
	}
	result := make([]MotionInterval, len(intervals))
	for i, interval := range intervals {
		result[i] = MotionInterval{
			From:      interval.From,
			To:        interval.To,
			Outcome:   IntervalOutcome(interval.Outcome),
			Clearance: interval.Clearance,
		}
	}
	return result
}

// Collision is a proven overlap at one ideal pose.
type Collision struct {
	At     units.Value
	Pose   r3.Transform
	Moving *Body
	Static *Body
	Volume Measurement
}
