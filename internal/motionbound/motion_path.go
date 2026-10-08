package motionbound

import (
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

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
		return fmt.Errorf(`%w: a revolute's center and axis must be finite, got %v and %v`, decaderr.ErrNotFinite, m.Center, m.Axis)
	}
	if m.Axis == (r3.Vec{}) {
		return fmt.Errorf(`%w: a zero rotation axis names no direction`, decaderr.ErrDegenerate)
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
		return r3.Transform{}, fmt.Errorf(`%w: the pose displacement is not representable: %w`, decaderr.ErrNotFinite, err)
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
		return fmt.Errorf(`%w: a prismatic direction must be finite, got %v`, decaderr.ErrNotFinite, m.Dir)
	}
	if _, ok := m.Dir.Normalize(); !ok {
		return fmt.Errorf(`%w: the prismatic direction %v names no direction`, decaderr.ErrDegenerate, m.Dir)
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
		return r3.Transform{}, fmt.Errorf(`%w: the pose fraction is not representable`, decaderr.ErrNotFinite)
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
			return fmt.Errorf(`%w: a between's From and To must be finite`, decaderr.ErrNotFinite)
		}
	}
	for _, t := range ends {
		if !t.IsValid() {
			return fmt.Errorf(`%w: a between's From and To must each be a rigid motion`, decaderr.ErrDegenerate)
		}
	}
	if m.From.IsReflection() != m.To.IsReflection() {
		return fmt.Errorf(`%w: no rigid path joins a reflection to a proper motion`, decaderr.ErrDegenerate)
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
		return fmt.Errorf(`%w: the between's relative motion is not a rigid motion: %w`, decaderr.ErrDegenerate, err)
	}
	return fmt.Errorf(`%w: the between's screw or pose is not representable: %w`, decaderr.ErrNotFinite, err)
}

// motionKinds refuses a From or To of the wrong Kind.
func motionKinds(kind units.Kind, from, to units.Value) error {
	if from.Kind() != kind {
		return fmt.Errorf(`%w: a motion's From must be a %s, got %s`, decaderr.ErrUnitKind, kind, from.Kind())
	}
	if to.Kind() != kind {
		return fmt.Errorf(`%w: a motion's To must be a %s, got %s`, decaderr.ErrUnitKind, kind, to.Kind())
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
		return fmt.Errorf(`%w: %s must be a %s, got %s`, decaderr.ErrUnitKind, what, kind, v.Kind())
	}
	return motionValueFinite(v, what)
}

func motionValueFinite(v units.Value, what string) error {
	if proofbound.IsNonFinite(v.Mag()) {
		return fmt.Errorf(`%w: %s must be finite, got %s`, decaderr.ErrNotFinite, what, v)
	}
	base, ok := units.BaseUnit(v.Kind())
	if !ok {
		return fmt.Errorf(`%w: %s has no base unit`, decaderr.ErrUnitKind, what)
	}
	if _, err := v.In(base); err != nil {
		return fmt.Errorf(`%w: %s is not representable: %w`, decaderr.ErrNotFinite, what, err)
	}
	return nil
}

// motionPoseError maps an r3 constructor's refusal onto the core §12
// vocabulary: an axis with no direction is ErrDegenerate, anything else the
// constructor refuses is a pose it could not represent, ErrNotFinite.
func motionPoseError(err error) error {
	if errors.Is(err, r3.ErrDegenerateAxis) {
		return fmt.Errorf(`%w: the rotation axis names no direction: %w`, decaderr.ErrDegenerate, err)
	}
	return fmt.Errorf(`%w: the pose is not representable: %w`, decaderr.ErrNotFinite, err)
}

// ValidateRevolute applies the variant's field checks.
func ValidateRevolute(m Revolute) error { return m.validate() }

// ValidatePrismatic applies the variant's field checks.
func ValidatePrismatic(m Prismatic) error { return m.validate() }

// ValidateBetween applies the variant's field checks.
func ValidateBetween(m Between) error { return m.validate() }

// ScrewBetween reads the relative rigid motion's screw parameters.
func ScrewBetween(m Between) (r3.Screw, error) { return m.screw() }

// RevolutePose builds a validated revolute joint's pose.
func RevolutePose(center, axis r3.Vec, at units.Value) (r3.Transform, error) {
	return revolutePose(center, axis, at)
}

// PrismaticPose builds a validated prismatic joint's pose.
func PrismaticPose(dir r3.Vec, at units.Value) (r3.Transform, error) {
	return prismaticPose(dir, at)
}

// MotionKinds checks the kinds of two motion endpoints.
func MotionKinds(kind units.Kind, from, to units.Value) error { return motionKinds(kind, from, to) }

// MotionFinite checks the finite values of motion endpoints.
func MotionFinite(values ...units.Value) error { return motionFinite(values...) }

// MotionValueValid checks a pose parameter's kind and finiteness.
func MotionValueValid(v units.Value, kind units.Kind, what string) error {
	return motionValueValid(v, kind, what)
}
