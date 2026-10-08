package motionbound

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Domain is the parameter a check bisects: its quantity kind, its two endpoints
// as stated, and their exact denotations. A Motion's domain is its own From
// and To; a linkage drive's is the Dimensionless fraction [0, 1], exactly a
// Between's (FractionDomain).
type Domain struct {
	Quantity   units.Kind
	From, To   units.Value
	FromP, ToP MotionParam
}

// FractionDomain is the Dimensionless fraction s ∈ [0, 1] a Between and a
// linkage drive both run over.
func FractionDomain() Domain {
	return Domain{
		Quantity: units.Dimensionless,
		From:     units.Scalar(0),
		To:       units.Scalar(1),
		FromP:    MotionParam{Turn: new(big.Rat), Base: new(big.Rat)},
		ToP:      MotionParam{Turn: new(big.Rat), Base: big.NewRat(1, 1)},
	}
}

// Spec is a validated Motion read into the fields every pose and bound
// is built from. A Between's parameter runs over the dimensionless fraction
// [0, 1], so its From and To are units.Scalar(0) and units.Scalar(1), and
// Between and Screw carry its poses and the screw r3 reads off them.
type Spec struct {
	Domain
	Motion  Motion
	Kind    MotionKind
	Center  r3.Vec
	Axis    r3.Vec
	Dir     r3.Vec
	Between Between
	Screw   r3.Screw
	Frame   MotionFrame
}

// ResolveMotion validates m (docs/motion-check-design.md §2, §8): the
// variant's own field refusals, From == To, and both endpoint poses building
// under r3. A nil motion is ErrDegenerate.
func ResolveMotion(m Motion) (Spec, error) {
	var spec Spec
	switch mv := m.(type) {
	case Revolute:
		if err := ValidateRevolute(mv); err != nil {
			return Spec{}, err
		}
		spec = Spec{Kind: MotionRevolute, Center: mv.Center, Axis: mv.Axis,
			Domain: Domain{Quantity: units.Angle, From: mv.From, To: mv.To}}
	case *Revolute:
		if mv == nil {
			return Spec{}, fmt.Errorf(`%w: a nil motion names no path`, decaderr.ErrDegenerate)
		}
		return resolveMotionAs(m, *mv)
	case Prismatic:
		if err := ValidatePrismatic(mv); err != nil {
			return Spec{}, err
		}
		spec = Spec{Kind: MotionPrismatic, Dir: mv.Dir,
			Domain: Domain{Quantity: units.Length, From: mv.From, To: mv.To}}
	case *Prismatic:
		if mv == nil {
			return Spec{}, fmt.Errorf(`%w: a nil motion names no path`, decaderr.ErrDegenerate)
		}
		return resolveMotionAs(m, *mv)
	case Between:
		sc, err := resolveBetween(mv)
		if err != nil {
			return Spec{}, err
		}
		spec = Spec{Kind: MotionBetween, Between: mv, Screw: sc,
			Domain: Domain{Quantity: units.Dimensionless, From: units.Scalar(0), To: units.Scalar(1)}}
	case *Between:
		if mv == nil {
			return Spec{}, fmt.Errorf(`%w: a nil motion names no path`, decaderr.ErrDegenerate)
		}
		return resolveMotionAs(m, *mv)
	case nil:
		return Spec{}, fmt.Errorf(`%w: a nil motion names no path`, decaderr.ErrDegenerate)
	default:
		return Spec{}, fmt.Errorf(`%w: a motion of type %T is not one this evaluator checks`, decaderr.ErrUnsupported, m)
	}
	spec.Motion = m
	var okF, okT bool
	spec.FromP, okF = ExactMotionParam(spec.From)
	spec.ToP, okT = ExactMotionParam(spec.To)
	if !okF || !okT {
		return Spec{}, fmt.Errorf(`%w: a motion endpoint is not representable`, decaderr.ErrNotFinite)
	}
	if SameMotionValue(spec.From, spec.To) {
		return Spec{}, fmt.Errorf(`%w: From and To are both %s, which names no path`, decaderr.ErrDegenerate, spec.From)
	}
	for _, end := range []units.Value{spec.From, spec.To} {
		if _, err := m.PoseAt(end); err != nil {
			return Spec{}, err
		}
	}
	frame, ok := NewMotionFrame(spec)
	if !ok {
		return Spec{}, fmt.Errorf(`%w: the motion's axis is not representable`, decaderr.ErrNotFinite)
	}
	spec.Frame = frame
	return spec, nil
}

// Label is the published parameter of the pose at fraction f of the path:
// From and To exactly at the ends, and otherwise the float nearest the exact
// interpolation carried in From's unit — exact itself whenever From and To
// share a unit and the dyadic step is representable. It is a label: every
// bound is built from the exact parameter FromP.Lerp(ToP, f), and
// PoseDeviation charges whatever separates the pose PoseAt builds from this
// label and the ideal pose at f.
func (s Domain) Label(f *big.Rat) units.Value {
	return DomainLabel(s.From, s.To, f)
}

// resolveBetween validates a Between for VerifyMotion (docs/motion-check-design.md
// §2, §8): its own field refusals, then From == To, then the screw r3 reads
// off the relative motion, which must be representable and must not be the
// zero screw — a relative motion with neither angle nor slide names no path,
// since PoseAt is then From at every s.
func resolveBetween(m Between) (r3.Screw, error) {
	if err := ValidateBetween(m); err != nil {
		return r3.Screw{}, err
	}
	if m.From == m.To {
		return r3.Screw{}, fmt.Errorf(`%w: a between whose From equals its To names no path`, decaderr.ErrDegenerate)
	}
	sc, err := ScrewBetween(m)
	if err != nil {
		return r3.Screw{}, err
	}
	if sc.Angle.Mag() == 0 && sc.Slide == 0 {
		return r3.Screw{}, fmt.Errorf(`%w: a between whose relative motion is the zero screw names no path`, decaderr.ErrDegenerate)
	}
	return sc, nil
}

// resolveMotionAs resolves a dereferenced pointer motion while keeping the
// caller's own value as the stated motion the report echoes.
func resolveMotionAs(stated Motion, value Motion) (Spec, error) {
	spec, err := ResolveMotion(value)
	if err != nil {
		return Spec{}, err
	}
	spec.Motion = stated
	return spec, nil
}

// NewMotionFrame builds the exact motion frame for a validated specification.
func NewMotionFrame(spec Spec) (MotionFrame, bool) {
	dirVec := spec.Dir
	center := r3.Vec{}
	switch spec.Kind {
	case MotionRevolute:
		dirVec, center = spec.Axis, spec.Center
	case MotionBetween:
		dirVec, center = spec.Screw.Axis, spec.Screw.Point
	}
	axis, okA := RatVecOf(dirVec)
	pivot, okC := RatVecOf(center)
	if !okA || !okC {
		return MotionFrame{}, false
	}
	unit, ok := UnitScaleInterval(axis)
	if !ok {
		return MotionFrame{}, false
	}
	mf := MotionFrame{Kind: spec.Kind, Axis: axis, Unit: unit, Center: pivot}
	if spec.Kind != MotionBetween {
		return mf, true
	}
	theta, okT := ExactMotionParam(spec.Screw.Angle)
	slide := proofarith.FloatRat(spec.Screw.Slide)
	fromRot, fromT, okF := ExactTransform(spec.Between.From)
	toRot, toT, okTo := ExactTransform(spec.Between.To)
	if !okT || slide == nil || !okF || !okTo {
		return MotionFrame{}, false
	}
	mf.Theta, mf.Slide = theta, slide
	mf.FromRot, mf.FromT, mf.ToRot, mf.ToT = fromRot, fromT, toRot, toT
	return mf, true
}

// SameMotionValue reports exact equality of two quantities of one Kind,
// compared as the exact rationals they denote (MotionParam), so 0.5 m and
// 500 mm are one value and a degree is never mistaken for a radian.
func SameMotionValue(a, b units.Value) bool {
	pa, okA := ExactMotionParam(a)
	pb, okB := ExactMotionParam(b)
	return okA && okB && pa.Turn.Cmp(pb.Turn) == 0 && pa.Base.Cmp(pb.Base) == 0
}
