package decad

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
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

// JointBoxOption configures VerifyJointBox (docs/linkage-check-design.md
// §14.1). Every MotionOption is one, so WithMotionTolerance, WithResolution
// and WithMinClearance pass to VerifyJointBox unchanged; WithCellBudget is the
// box's own.
type JointBoxOption interface {
	option.Interface
	jointBoxOption()
}

// MotionOption configures VerifyMotion and VerifyLinkage. It embeds
// JointBoxOption, so a MotionOption also configures VerifyJointBox, while a
// JointBoxOption of the box's own, WithCellBudget, does not compile as a
// MotionOption.
type MotionOption interface {
	JointBoxOption
	motionOption()
}

type motionOption struct{ option.Interface }

func (motionOption) motionOption()   {}
func (motionOption) jointBoxOption() {}

type identMotionTolerance struct{}
type identResolution struct{}
type identMinClearance struct{}

// WithMotionTolerance is verification §2's relative tolerance under a name of
// its own (docs/motion-check-design.md §3): it governs every bounded reading
// a MotionReport carries, with the pair diameter as the reference.
// Dimensionless; the default is units.Scalar(1e-3). A wrong Kind is
// ErrUnitKind, a negative value ErrNegativeMagnitude, a non-finite one
// ErrNotFinite.
func WithMotionTolerance(rel units.Value) MotionOption {
	return motionOption{option.New(identMotionTolerance{}, rel)}
}

// WithResolution states the finest parameter step the check refines to, for
// the verdict and for the readings alike (docs/motion-check-design.md §3,
// §6): an interval at or below it that the certificate still cannot settle
// reads IntervalUndecided, and a path reading or margin the floor leaves
// coarse is published coarse. It is a magnitude of the motion's own Kind —
// an angle for a Revolute, a length for a Prismatic, a dimensionless fraction
// of the path for a Between and of the drive for VerifyLinkage. A wrong Kind
// is ErrUnitKind, a negative or zero value ErrNegativeMagnitude, a non-finite
// one ErrNotFinite. A resolution wider than the whole path (wider than 1 for
// a Between or a drive) evaluates the endpoints alone. The default is
// |To − From|/1024, units.Scalar(1.0/1024) for a Between or a drive. If that
// step underflows in From's unit or its base unit, the check uses and reports
// the smallest positive resolution in From's unit accepted by WithResolution.
// VerifyLinkage left without it refines its whole-drive reading further, to
// units.Scalar(1.0/16384) (LinkageReport.ReadingResolution); stated, it is
// the one floor of both calls.
func WithResolution(step units.Value) MotionOption {
	return motionOption{option.New(identResolution{}, step)}
}

// WithMinClearance states the spec that the moving set stays at least minimum
// from every static body over the whole path (docs/motion-check-design.md §3),
// and, for VerifyLinkage, that every evaluated pair stays that far apart,
// deciding the report's Assessment: met when every interval certifies the
// margin, violated when some pose proves a gap below it, undecided otherwise.
// Its magnitude rules are WithMinWallThickness's: a Length, finite and
// non-negative, and a zero minimum, which no gap can fall below, poses no
// question and is ErrDegenerate.
func WithMinClearance(minimum units.Value) MotionOption {
	return motionOption{option.New(identMinClearance{}, minimum)}
}

// motionConfig is the folded VerifyMotion option set. resolution and
// minimum are the stated values; resolutionP and minimumMM are the exact
// rationals every comparison reads them as.
type motionConfig struct {
	rel         float64
	resolution  units.Value
	resolutionP motionbound.MotionParam
	// stated: WithResolution set the resolution; false when it is the
	// default.
	stated bool
	// readingP is the floor of the whole-path reading's refinement when it
	// differs from resolutionP (docs/linkage-check-design.md §3); nil means
	// the one floor governs both, as for every VerifyMotion call.
	readingP  *motionbound.MotionParam
	minimum   *units.Value
	minimumMM *big.Rat
}

// resolveMotionOptions folds and validates the options against the resolved
// parameter domain — a Motion's own, or a linkage drive's fraction — and
// duplicates keep the last occurrence (verification §1.0).
func resolveMotionOptions(opts []MotionOption, spec motionSpec) (motionConfig, error) {
	cfg := motionConfig{rel: 1e-3}
	var resolution *units.Value
	for _, o := range opts {
		if o == nil {
			return motionConfig{}, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		v, ok := option.Get[units.Value](o)
		if !ok {
			return motionConfig{}, fmt.Errorf(`%w: a motion option carries no value`, ErrDegenerate)
		}
		switch o.Ident().(type) {
		case identMotionTolerance:
			rel, err := magnitudeIn(v, units.Dimensionless, units.One, "motion tolerance")
			if err != nil {
				return motionConfig{}, err
			}
			cfg.rel = rel
		case identResolution:
			kind := spec.paramKind()
			base, _ := units.BaseUnit(kind)
			step, err := magnitudeIn(v, kind, base, "resolution")
			if err != nil {
				return motionConfig{}, err
			}
			if step == 0 {
				return motionConfig{}, fmt.Errorf(`%w: a resolution must be positive, got %s`, ErrNegativeMagnitude, v)
			}
			resolution = &v
		case identMinClearance:
			minimum, err := magnitudeIn(v, units.Length, units.Millimeter, "minimum clearance")
			if err != nil {
				return motionConfig{}, err
			}
			if minimum == 0 {
				return motionConfig{}, fmt.Errorf(`%w: a zero minimum clearance poses no question`, ErrDegenerate)
			}
			cfg.minimum = &v
		}
	}
	if resolution == nil {
		// The published default is a float label for the exact one-1024th
		// step, unless that label underflows. Then use the reported positive
		// fallback as the floor itself.
		var clamped bool
		cfg.resolution, clamped = spec.defaultResolution()
		cfg.resolutionP = spec.defaultResolutionParam()
		if clamped {
			cfg.resolutionP, _ = motionbound.ExactMotionParam(cfg.resolution)
		}
	} else {
		cfg.resolution, cfg.stated = *resolution, true
		var ok bool
		if cfg.resolutionP, ok = motionbound.ExactMotionParam(cfg.resolution); !ok {
			return motionConfig{}, fmt.Errorf(`%w: the resolution is not representable`, ErrNotFinite)
		}
	}
	if cfg.minimum != nil {
		p, ok := motionbound.ExactMotionParam(*cfg.minimum)
		if !ok {
			return motionConfig{}, fmt.Errorf(`%w: the minimum clearance is not representable`, ErrNotFinite)
		}
		cfg.minimumMM = p.Base
	}
	return cfg, nil
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
