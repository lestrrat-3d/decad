package motionoption

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

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
// ErrUnitKind, a negative value decaderr.ErrNegativeMagnitude, a non-finite one
// decaderr.ErrNotFinite.
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
// is ErrUnitKind, a negative or zero value decaderr.ErrNegativeMagnitude, a non-finite
// one decaderr.ErrNotFinite. A resolution wider than the whole path (wider than 1 for
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
// question and is decaderr.ErrDegenerate.
func WithMinClearance(minimum units.Value) MotionOption {
	return motionOption{option.New(identMinClearance{}, minimum)}
}

// Config is the folded VerifyMotion option set. resolution and
// minimum are the stated values; resolutionP and minimumMM are the exact
// rationals every comparison reads them as.
type Config struct {
	Rel         float64
	Resolution  units.Value
	ResolutionP motionbound.MotionParam
	// Stated: WithResolution set the Resolution; false when it is the
	// default.
	Stated bool
	// ReadingP is the floor of the whole-path reading's refinement when it
	// differs from ResolutionP (docs/linkage-check-design.md §3); nil means
	// the one floor governs both, as for every VerifyMotion call.
	ReadingP  *motionbound.MotionParam
	Minimum   *units.Value
	MinimumMM *big.Rat
}

// Resolve folds and validates the options against the resolved
// parameter domain — a Motion's own, or a linkage drive's fraction — and
// duplicates keep the last occurrence (verification §1.0).
func Resolve(opts []MotionOption, spec motionbound.Domain) (Config, error) {
	cfg := Config{Rel: 1e-3}
	var resolution *units.Value
	for _, o := range opts {
		if o == nil {
			return Config{}, fmt.Errorf(`%w: a nil option names nothing to apply`, decaderr.ErrDegenerate)
		}
		v, ok := option.Get[units.Value](o)
		if !ok {
			return Config{}, fmt.Errorf(`%w: a motion option carries no value`, decaderr.ErrDegenerate)
		}
		switch o.Ident().(type) {
		case identMotionTolerance:
			rel, err := sectionrecord.MagnitudeIn(v, units.Dimensionless, units.One, "motion tolerance")
			if err != nil {
				return Config{}, err
			}
			cfg.Rel = rel
		case identResolution:
			kind := spec.Quantity
			base, _ := units.BaseUnit(kind)
			step, err := sectionrecord.MagnitudeIn(v, kind, base, "resolution")
			if err != nil {
				return Config{}, err
			}
			if step == 0 {
				return Config{}, fmt.Errorf(`%w: a resolution must be positive, got %s`, decaderr.ErrNegativeMagnitude, v)
			}
			resolution = &v
		case identMinClearance:
			minimum, err := sectionrecord.MagnitudeIn(v, units.Length, units.Millimeter, "minimum clearance")
			if err != nil {
				return Config{}, err
			}
			if minimum == 0 {
				return Config{}, fmt.Errorf(`%w: a zero minimum clearance poses no question`, decaderr.ErrDegenerate)
			}
			cfg.Minimum = &v
		}
	}
	if resolution == nil {
		// The published default is a float label for the exact one-1024th
		// step, unless that label underflows. Then use the reported positive
		// fallback as the floor itself.
		var clamped bool
		cfg.Resolution, clamped = motionbound.DefaultResolution(spec.Quantity, spec.From, spec.To, spec.FromP, spec.ToP)
		cfg.ResolutionP = motionbound.DefaultResolutionParam(spec.FromP, spec.ToP)
		if clamped {
			cfg.ResolutionP, _ = motionbound.ExactMotionParam(cfg.Resolution)
		}
	} else {
		cfg.Resolution, cfg.Stated = *resolution, true
		var ok bool
		if cfg.ResolutionP, ok = motionbound.ExactMotionParam(cfg.Resolution); !ok {
			return Config{}, fmt.Errorf(`%w: the resolution is not representable`, decaderr.ErrNotFinite)
		}
	}
	if cfg.Minimum != nil {
		p, ok := motionbound.ExactMotionParam(*cfg.Minimum)
		if !ok {
			return Config{}, fmt.Errorf(`%w: the minimum clearance is not representable`, decaderr.ErrNotFinite)
		}
		cfg.MinimumMM = p.Base
	}
	return cfg, nil
}

// IdentCellBudget identifies the JointBox-only cell budget option.
type IdentCellBudget struct{}

type cellBudgetOption struct{ option.Interface }

func (cellBudgetOption) jointBoxOption() {}

// WithCellBudget records the maximum number of joint-box centres.
func WithCellBudget(cells int) JointBoxOption {
	return cellBudgetOption{option.New(IdentCellBudget{}, cells)}
}
