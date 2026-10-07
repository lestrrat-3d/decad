package decad

import (
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the motion vocabulary of docs/motion-check-design.md §2-§4:
// the sealed Motion set and its PoseAt, VerifyMotion's options, and the
// MotionReport records. motion_verify.go runs the check, and motion_bound.go
// proves the bounds its interval certificate consumes.

// Motion is a one-parameter family of rigid motions, parameterised by a typed
// scalar whose Kind the variant fixes: an Angle for Revolute, a Length for
// Prismatic, a Dimensionless fraction of the path for Between. PoseAt returns
// the rigid motion at one parameter value; it is what a layer above calls to
// draw the moving set, and what VerifyMotion calls to evaluate a pose. The set
// is sealed: Revolute, Prismatic and Between are its only members.
type Motion interface {
	PoseAt(at units.Value) (r3.Transform, error)
	motion()
}

// Revolute rotates about the axis through Center along Axis, right-handed,
// from angle From to angle To. The angle 0 is the moving body as it currently
// sits; the motion composes onto each moving body's own placement. From and
// To are signed angles, not magnitudes: From > To traverses the path in the
// other sense.
type Revolute struct {
	Center r3.Vec      // a position, millimetres (core §5.2)
	Axis   r3.Vec      // a direction; only its direction is used
	From   units.Value // Kind Angle, signed
	To     units.Value // Kind Angle, signed
}

func (Revolute) motion() {}

// PoseAt returns r3.RotationAround(Center, Axis, at). A wrong-Kind From, To
// or at is ErrUnitKind; a non-finite From, To, at, Center or Axis component,
// or a rotation r3 refuses to build, is ErrNotFinite; the zero Axis is
// ErrDegenerate. PoseAt takes no range, so From == To is not its refusal.
func (m Revolute) PoseAt(at units.Value) (r3.Transform, error) {
	if err := m.validate(); err != nil {
		return r3.Transform{}, err
	}
	if err := motionValueValid(at, units.Angle, "the pose angle"); err != nil {
		return r3.Transform{}, err
	}
	return revolutePose(m.Center, m.Axis, at)
}

// revolutePose is the rotation by at about the axis through center along
// axis, the one pose builder a Revolute and a revolute joint share. Its
// inputs are validated; an r3 refusal maps through motionPoseError.
func revolutePose(center, axis r3.Vec, at units.Value) (r3.Transform, error) {
	pose, err := r3.RotationAround(center, axis, at)
	if err != nil {
		return r3.Transform{}, motionPoseError(err)
	}
	return pose, nil
}

// validate applies the Revolute's own field refusals in docs/motion-check-design.md
// §2's order: every Kind first, then finiteness, then the axis direction.
func (m Revolute) validate() error {
	if err := motionKinds(units.Angle, m.From, m.To); err != nil {
		return err
	}
	if err := motionFinite(m.From, m.To); err != nil {
		return err
	}
	if !proofbound.FiniteVec(m.Center) || !proofbound.FiniteVec(m.Axis) {
		return fmt.Errorf(`%w: a revolute's center and axis must be finite, got %v and %v`, ErrNotFinite, m.Center, m.Axis)
	}
	if zeroVec(m.Axis) {
		return fmt.Errorf(`%w: a zero rotation axis names no direction`, ErrDegenerate)
	}
	return nil
}

// Prismatic translates along Dir from displacement From to displacement To.
// The displacement 0 is the moving body as it currently sits. From and To
// are signed displacements, not magnitudes: From > To traverses the path in
// the other sense.
type Prismatic struct {
	Dir  r3.Vec      // a direction; only its direction is used
	From units.Value // Kind Length, signed
	To   units.Value // Kind Length, signed
}

func (Prismatic) motion() {}

// PoseAt returns r3.Translation(d·at), where d is Dir normalised by
// r3.Vec.Normalize. A wrong-Kind From, To or at is ErrUnitKind; a non-finite
// From, To, at or Dir component, or a translation r3 refuses to build, is
// ErrNotFinite; a Dir Normalize reports no direction for is ErrDegenerate.
// PoseAt takes no range, so From == To is not its refusal.
func (m Prismatic) PoseAt(at units.Value) (r3.Transform, error) {
	if err := m.validate(); err != nil {
		return r3.Transform{}, err
	}
	if err := motionValueValid(at, units.Length, "the pose displacement"); err != nil {
		return r3.Transform{}, err
	}
	return prismaticPose(m.Dir, at)
}

// prismaticPose is the translation by at along dir normalised by
// r3.Vec.Normalize, the one pose builder a Prismatic and a prismatic joint
// share. Its inputs are validated; an r3 refusal maps through
// motionPoseError.
func prismaticPose(dir r3.Vec, at units.Value) (r3.Transform, error) {
	unit, _ := dir.Normalize()
	dist, err := at.In(units.Millimeter)
	if err != nil {
		return r3.Transform{}, fmt.Errorf(`%w: the pose displacement is not representable: %w`, ErrNotFinite, err)
	}
	pose, err := r3.Translation(unit.Scale(dist))
	if err != nil {
		return r3.Transform{}, motionPoseError(err)
	}
	return pose, nil
}

// validate applies the Prismatic's own field refusals in docs/motion-check-design.md
// §2's order: every Kind first, then finiteness, then the direction.
func (m Prismatic) validate() error {
	if err := motionKinds(units.Length, m.From, m.To); err != nil {
		return err
	}
	if err := motionFinite(m.From, m.To); err != nil {
		return err
	}
	if !proofbound.FiniteVec(m.Dir) {
		return fmt.Errorf(`%w: a prismatic direction must be finite, got %v`, ErrNotFinite, m.Dir)
	}
	if _, ok := m.Dir.Normalize(); !ok {
		return fmt.Errorf(`%w: the prismatic direction %v names no direction`, ErrDegenerate, m.Dir)
	}
	return nil
}

// Between is the screw motion joining two rigid poses: the parameter s runs
// from 0, where each moving body sits under From composed onto its own
// placement, to 1, where it sits under To. The path rotates through the
// shorter arc between the two orientations and slides uniformly along the
// rotation axis (Chasles; r3's Transform.Screw). r3.Identity() as From is the
// moving set as it currently sits.
type Between struct {
	From r3.Transform // the pose at s = 0
	To   r3.Transform // the pose at s = 1
}

func (Between) motion() {}

// PoseAt returns From.Then(screw.At(s)), where screw is the relative motion
// From.Inverse().Then(To) read by r3's Transform.Screw: the rotation by s·θ
// about the screw axis and the slide s·d along it, applied after From. At
// s = 0 it returns From and at s = 1 it returns To, both as stated. at is a
// Dimensionless fraction; an angle or a length is ErrUnitKind, a non-finite
// one ErrNotFinite.
//
// From and To are refused in docs/motion-check-design.md §2's order: a
// non-finite component is ErrNotFinite; a transform that is not a rigid motion
// (Transform.IsValid false, the zero r3.Transform{} included) is
// ErrDegenerate; a reflection joined to a proper motion, which no rigid path
// connects, is ErrDegenerate; and a screw or pose r3 cannot represent is
// ErrNotFinite. PoseAt takes no range, so From == To is not its refusal.
func (m Between) PoseAt(at units.Value) (r3.Transform, error) {
	if err := m.validate(); err != nil {
		return r3.Transform{}, err
	}
	if err := motionValueValid(at, units.Dimensionless, "the pose fraction"); err != nil {
		return r3.Transform{}, err
	}
	sc, err := m.screw()
	if err != nil {
		return r3.Transform{}, err
	}
	s, err := at.In(units.One)
	if err != nil || proofbound.IsNonFinite(s) {
		return r3.Transform{}, fmt.Errorf(`%w: the pose fraction is not representable`, ErrNotFinite)
	}
	switch s {
	case 0:
		return m.From, nil
	case 1:
		return m.To, nil
	}
	step, err := sc.At(s)
	if err != nil {
		return r3.Transform{}, betweenError(err)
	}
	pose, err := m.From.Then(step)
	if err != nil {
		return r3.Transform{}, betweenError(err)
	}
	return pose, nil
}

// validate applies the Between's own field refusals in
// docs/motion-check-design.md §2's order: finiteness, then rigidity, then
// handedness. Degeneracy — From == To, or a zero screw — is VerifyMotion's.
func (m Between) validate() error {
	ends := [2]r3.Transform{m.From, m.To}
	for _, t := range ends {
		b := t.Basis()
		if !proofbound.FiniteVec(b.EX) || !proofbound.FiniteVec(b.EY) || !proofbound.FiniteVec(b.EZ) || !proofbound.FiniteVec(t.Translation()) {
			return fmt.Errorf(`%w: a between's From and To must be finite`, ErrNotFinite)
		}
	}
	for _, t := range ends {
		if !t.IsValid() {
			return fmt.Errorf(`%w: a between's From and To must each be a rigid motion`, ErrDegenerate)
		}
	}
	if m.From.IsReflection() != m.To.IsReflection() {
		return fmt.Errorf(`%w: no rigid path joins a reflection to a proper motion`, ErrDegenerate)
	}
	return nil
}

// screw reads the relative motion From.Inverse().Then(To) as a screw. It is
// a deterministic function of From's and To's bits, so every PoseAt call and
// VerifyMotion's exact frame read the same parameters.
func (m Between) screw() (r3.Screw, error) {
	inv, err := m.From.Inverse()
	if err != nil {
		return r3.Screw{}, betweenError(err)
	}
	rel, err := inv.Then(m.To)
	if err != nil {
		return r3.Screw{}, betweenError(err)
	}
	sc, err := rel.Screw()
	if err != nil {
		return r3.Screw{}, betweenError(err)
	}
	return sc, nil
}

// betweenError maps an r3 refusal met while reading or evaluating a Between
// onto the core §12 vocabulary: a reflection joined to a proper motion, or a
// result that is not a rigid motion, is ErrDegenerate; anything else r3
// refuses is a screw or pose it cannot represent, ErrNotFinite.
func betweenError(err error) error {
	if errors.Is(err, r3.ErrImproper) || errors.Is(err, r3.ErrNotOrthonormal) {
		return fmt.Errorf(`%w: the between's relative motion is not a rigid motion: %w`, ErrDegenerate, err)
	}
	return fmt.Errorf(`%w: the between's screw or pose is not representable: %w`, ErrNotFinite, err)
}

// motionKinds refuses a From or To of the wrong Kind.
func motionKinds(kind units.Kind, from, to units.Value) error {
	if from.Kind() != kind {
		return fmt.Errorf(`%w: a motion's From must be a %s, got %s`, ErrUnitKind, kind, from.Kind())
	}
	if to.Kind() != kind {
		return fmt.Errorf(`%w: a motion's To must be a %s, got %s`, ErrUnitKind, kind, to.Kind())
	}
	return nil
}

// motionFinite refuses a From or To whose magnitude, or whose magnitude in the
// Kind's base unit, is not finite.
func motionFinite(values ...units.Value) error {
	for _, v := range values {
		if err := motionValueFinite(v, "a motion endpoint"); err != nil {
			return err
		}
	}
	return nil
}

// motionValueValid refuses a pose parameter of the wrong Kind, then a
// non-finite one.
func motionValueValid(v units.Value, kind units.Kind, what string) error {
	if v.Kind() != kind {
		return fmt.Errorf(`%w: %s must be a %s, got %s`, ErrUnitKind, what, kind, v.Kind())
	}
	return motionValueFinite(v, what)
}

func motionValueFinite(v units.Value, what string) error {
	if proofbound.IsNonFinite(v.Mag()) {
		return fmt.Errorf(`%w: %s must be finite, got %s`, ErrNotFinite, what, v)
	}
	base, ok := units.BaseUnit(v.Kind())
	if !ok {
		return fmt.Errorf(`%w: %s has no base unit`, ErrUnitKind, what)
	}
	if _, err := v.In(base); err != nil {
		return fmt.Errorf(`%w: %s is not representable: %w`, ErrNotFinite, what, err)
	}
	return nil
}

// motionPoseError maps an r3 constructor's refusal onto the core §12
// vocabulary: an axis with no direction is ErrDegenerate, anything else the
// constructor refuses is a pose it could not represent, ErrNotFinite.
func motionPoseError(err error) error {
	if errors.Is(err, r3.ErrDegenerateAxis) {
		return fmt.Errorf(`%w: the rotation axis names no direction: %w`, ErrDegenerate, err)
	}
	return fmt.Errorf(`%w: the pose is not representable: %w`, ErrNotFinite, err)
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

// MotionReport is what VerifyMotion returns (docs/motion-check-design.md §4):
// Verify's own vocabulary plus the records that carry the path.
// Diagnostics lists interval findings in interval order, then pose findings
// in pose order, then the whole-path reading's own tolerance finding; it is
// empty exactly when Status is Sound, and Status is the worst
// Diagnostic.Status in it.
type MotionReport struct {
	Request     MotionRequest    // the validated effective settings this call used, including defaults
	Motion      Motion           // the motion as stated
	Moving      []*Body          // the rigid set, in the order given
	Against     []*Body          // every static body considered, in Document.Bodies() order
	Poses       []PoseResult     // every pose evaluated, in traversal order from From to To
	Intervals   []MotionInterval // the consecutive intervals between adjacent Poses, in the same order
	Collisions  []Collision      // every proven collision, in traversal order then pair order
	Clearance   *ScalarReading   // the minimum gap over the WHOLE path; nil unless every interval is IntervalClear
	Assessment  Assessment       // against WithMinClearance; AssessmentNotEvaluated when not requested
	Diagnostics []Diagnostic     // interval findings, then pose findings, then the path reading's
	Status      Status           // Unverified on a zero value; VerifyMotion always returns a decided status
}

// Passed reports whether the report is Sound. It returns false for a nil
// report and for any other Status.
func (r *MotionReport) Passed() bool {
	return r != nil && r.Status == Sound
}

// MotionRequest is one VerifyMotion call's effective settings, each value as
// the caller stated it or as the default was formed.
type MotionRequest struct {
	RelativeTolerance units.Value  // always present
	Resolution        units.Value  // always present, in the motion's Kind
	MinClearance      *units.Value // non-nil exactly when WithMinClearance was requested
}

// PoseResult is one evaluated pose: the parameter, the rigid motion applied to
// the moving set there, and the pair results at that pose in Verify's own
// shape. A Clearance row here is a measurement at this pose only; the
// continuous claim lives on the MotionInterval. Every row and diagnostic
// names the caller's own moving body as A, never the transient placement the
// check evaluated.
type PoseResult struct {
	At            units.Value    // the parameter; Kind Angle, Length or Dimensionless as the Motion fixes
	Pose          r3.Transform   // Motion.PoseAt(At): what composes onto each mover's own placement
	Interferences []Interference // A is the mover, B the static body; proven overlap, bounded volume
	Clearances    []Clearance    // A is the mover, B the static body; every pair proven disjoint or touching
	Diagnostics   []Diagnostic   // this pose's own undecided or unsupported pairs and invalid bodies, At set
}

// MotionInterval is the stretch of the path between two adjacent evaluated
// poses.
//
// Clearance is set only on an IntervalClear interval that holds at least one
// evaluated or swept-box-excluded pair. Its Value is a PROVEN LOWER BOUND on
// every (mover, static) gap over the whole closed interval — not an estimate
// of the gap — so it reads Approximate with a zero Bound: the number is the
// claim itself, and nothing about the true gap above it is stated.
type MotionInterval struct {
	From, To  units.Value // the two adjacent PoseResult.At values, in traversal order
	Outcome   IntervalOutcome
	Clearance *Measurement // a proven lower bound on the gap over the interval; nil unless Outcome is IntervalClear
}

// IntervalOutcome is what a MotionInterval proves (docs/motion-check-design.md
// §4).
type IntervalOutcome int

const (
	// IntervalNotEvaluated is the reserved zero value: VerifyMotion never
	// returns it.
	IntervalNotEvaluated IntervalOutcome = iota
	// IntervalClear — for EVERY parameter in the closed interval, every
	// (mover, static) pair has disjoint interiors at a proven positive gap.
	IntervalClear
	// IntervalColliding — a proven collision sits at one of its endpoints;
	// nothing is claimed about the interior.
	IntervalColliding
	// IntervalUndecided — neither of the above. It claims nothing.
	IntervalUndecided
)

// String renders the pinned lower-snake token. An out-of-range value renders
// "interval_outcome(<n>)", never a panic.
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

// Collision is a proven overlap at an evaluated pose, about the IDEAL pose
// the parameter names (docs/motion-check-design.md §5.1). The read-only proof
// measured the overlap with the mover under the float transform Pose composed
// onto its own placement; the collision is published only when that measured
// volume's proven lower end clears the volume the mover's boundary can sweep
// between the float pose and the ideal one, and Volume's Bound carries that
// allowance, so Volume.Value − Volume.Bound is a proven lower bound on the
// ideal pose's overlap. Nothing is claimed about the interval around it.
type Collision struct {
	At     units.Value  // the parameter of the pose
	Pose   r3.Transform // Motion.PoseAt(At)
	Moving *Body
	Static *Body
	Volume Measurement // the overlap volume, Value − Bound a proven lower bound on the ideal overlap
}

// sameMotionValue reports exact equality of two quantities of one Kind,
// compared as the exact rationals they denote (motionbound.MotionParam), so 0.5 m and
// 500 mm are one value and a degree is never mistaken for a radian.
func sameMotionValue(a, b units.Value) bool {
	pa, okA := motionbound.ExactMotionParam(a)
	pb, okB := motionbound.ExactMotionParam(b)
	return okA && okB && pa.Turn.Cmp(pb.Turn) == 0 && pa.Base.Cmp(pb.Base) == 0
}
