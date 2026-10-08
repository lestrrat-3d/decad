package decad

import (
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/motionoption"
	"github.com/lestrrat-3d/decad/internal/reportvocab"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the motion vocabulary of docs/motion-check-design.md §2-§4:
// public aliases for the sealed Motion set, VerifyMotion's options, and
// the MotionReport records. motion_verify.go runs the check, and motion_bound.go
// proves the bounds its interval certificate consumes.

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

func revolutePose(center, axis r3.Vec, at units.Value) (r3.Transform, error) {
	return motionbound.RevolutePose(center, axis, at)
}

func prismaticPose(dir r3.Vec, at units.Value) (r3.Transform, error) {
	return motionbound.PrismaticPose(dir, at)
}

func motionKinds(kind units.Kind, from, to units.Value) error {
	return motionbound.MotionKinds(kind, from, to)
}

func motionFinite(values ...units.Value) error { return motionbound.MotionFinite(values...) }

func motionValueValid(v units.Value, kind units.Kind, what string) error {
	return motionbound.MotionValueValid(v, kind, what)
}

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

func resolveMotionOptions(opts []MotionOption, spec motionSpec) (motionConfig, error) {
	d := spec.motionDomain
	return motionoption.Resolve(opts, motionoption.Domain{
		Quantity: d.quantity, From: d.from, To: d.to, FromP: d.fromP, ToP: d.toP,
	})
}

// MotionReport is VerifyMotion's path report (docs/motion-check-design.md §4).
type MotionReport = reportvocab.MotionReport[*Body, JointCell]

// MotionRequest records the effective settings of a VerifyMotion call.
type MotionRequest = reportvocab.MotionRequest

// PoseResult records one evaluated pose and its pair findings.
type PoseResult = reportvocab.PoseResult[*Body, JointCell]

// MotionInterval records the certificate between adjacent poses.
type MotionInterval = reportvocab.MotionInterval

// IntervalOutcome states what a MotionInterval proves.
type IntervalOutcome = reportvocab.IntervalOutcome

const (
	IntervalNotEvaluated = reportvocab.IntervalNotEvaluated
	IntervalClear        = reportvocab.IntervalClear
	IntervalColliding    = reportvocab.IntervalColliding
	IntervalUndecided    = reportvocab.IntervalUndecided
)

// Collision is a proven overlap at one ideal pose.
type Collision = reportvocab.Collision[*Body]

// sameMotionValue reports exact equality of two quantities of one Kind,
// compared as the exact rationals they denote (motionbound.MotionParam), so 0.5 m and
// 500 mm are one value and a degree is never mistaken for a radian.
func sameMotionValue(a, b units.Value) bool {
	pa, okA := motionbound.ExactMotionParam(a)
	pb, okB := motionbound.ExactMotionParam(b)
	return okA && okB && pa.Turn.Cmp(pb.Turn) == 0 && pa.Base.Cmp(pb.Base) == 0
}
