package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/motionoption"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the motion vocabulary of docs/motion-check-design.md §2-§4:
// the sealed Motion set, root-owned option types and
// MotionReport records. internal/motionengine runs the check, and
// internal/motionbound proves the bounds its interval certificate consumes.

// Motion is the sealed set of one-parameter rigid motions. PoseAt returns
// the rigid transform at a typed parameter value.
type Motion interface {
	PoseAt(at units.Value) (r3.Transform, error)
	motion()
}

// Revolute rotates a moving set about the axis through Center. From and To
// are signed angles.
type Revolute struct {
	Center r3.Vec
	Axis   r3.Vec
	From   units.Value
	To     units.Value
}

func (Revolute) motion() {}

// PoseAt returns the rigid pose at the stated angle.
func (m Revolute) PoseAt(at units.Value) (r3.Transform, error) {
	return motionbound.Revolute(m).PoseAt(at)
}

// Prismatic translates a moving set along Dir. From and To are signed
// lengths.
type Prismatic struct {
	Dir  r3.Vec
	From units.Value
	To   units.Value
}

func (Prismatic) motion() {}

// PoseAt returns the rigid pose at the stated displacement.
func (m Prismatic) PoseAt(at units.Value) (r3.Transform, error) {
	return motionbound.Prismatic(m).PoseAt(at)
}

// Between joins From and To along the shorter rigid screw path. Its
// parameter is a dimensionless fraction.
type Between struct {
	From r3.Transform
	To   r3.Transform
}

func (Between) motion() {}

// PoseAt returns the rigid pose at the stated path fraction.
func (m Between) PoseAt(at units.Value) (r3.Transform, error) {
	return motionbound.Between(m).PoseAt(at)
}

func encodedMotion(m Motion) (motionbound.Motion, error) {
	switch value := m.(type) {
	case Revolute:
		return motionbound.Revolute(value), nil
	case *Revolute:
		if value == nil {
			return (*motionbound.Revolute)(nil), nil
		}
		encoded := motionbound.Revolute(*value)
		return &encoded, nil
	case Prismatic:
		return motionbound.Prismatic(value), nil
	case *Prismatic:
		if value == nil {
			return (*motionbound.Prismatic)(nil), nil
		}
		encoded := motionbound.Prismatic(*value)
		return &encoded, nil
	case Between:
		return motionbound.Between(value), nil
	case *Between:
		if value == nil {
			return (*motionbound.Between)(nil), nil
		}
		encoded := motionbound.Between(*value)
		return &encoded, nil
	case nil:
		return nil, fmt.Errorf(`%w: a nil motion names no path`, ErrDegenerate)
	default:
		return nil, fmt.Errorf(`%w: a motion of type %T is not one this evaluator checks`, ErrUnsupported, m)
	}
}

func resolveMotion(m Motion) (motionbound.Spec, error) {
	encoded, err := encodedMotion(m)
	if err != nil {
		return motionbound.Spec{}, err
	}
	return motionbound.ResolveMotion(encoded)
}

// JointBoxOption configures VerifyJointBox.
type JointBoxOption interface {
	option.Interface
	jointBoxOption()
}

// MotionOption configures VerifyMotion and VerifyLinkage.
type MotionOption interface {
	JointBoxOption
	motionOption()
}

type motionOptionValue struct{ motionoption.MotionOption }
type jointBoxOptionValue struct{ motionoption.JointBoxOption }

func (motionOptionValue) motionOption()     {}
func (motionOptionValue) jointBoxOption()   {}
func (jointBoxOptionValue) jointBoxOption() {}

// WithMotionTolerance sets the relative tolerance for motion readings.
func WithMotionTolerance(rel units.Value) MotionOption {
	return motionOptionValue{motionoption.WithMotionTolerance(rel)}
}

// WithResolution sets the finest parameter step for verdicts and readings.
func WithResolution(step units.Value) MotionOption {
	return motionOptionValue{motionoption.WithResolution(step)}
}

// WithMinClearance sets the minimum gap over the path.
func WithMinClearance(minimum units.Value) MotionOption {
	return motionOptionValue{motionoption.WithMinClearance(minimum)}
}

func decodeMotionOptions(opts []MotionOption) ([]motionoption.MotionOption, error) {
	encoded := make([]motionoption.MotionOption, len(opts))
	for i, opt := range opts {
		if opt == nil {
			continue
		}
		value, ok := opt.(motionOptionValue)
		if !ok {
			return nil, fmt.Errorf(`%w: the motion option is not a decad motion option (%T)`, ErrDegenerate, opt)
		}
		encoded[i] = value.MotionOption
	}
	return encoded, nil
}

type (
	// MotionReport is VerifyMotion's path report (docs/motion-check-design.md §4).
	MotionReport = report.MotionReportOf[*Body, Motion]
	// MotionRequest records the effective settings of a VerifyMotion call.
	MotionRequest = report.MotionRequest
	// PoseResult records one evaluated pose and its pair findings.
	PoseResult = report.PoseResultOf[*Body]
	// MotionInterval records the certificate between adjacent poses.
	MotionInterval = report.MotionInterval
	// IntervalOutcome states what a MotionInterval proves.
	IntervalOutcome = report.IntervalOutcome
	// Collision is a proven overlap at one ideal pose.
	Collision = report.CollisionOf[*Body]
)

const (
	IntervalNotEvaluated = report.IntervalNotEvaluated
	IntervalClear        = report.IntervalClear
	IntervalColliding    = report.IntervalColliding
	IntervalUndecided    = report.IntervalUndecided
)
