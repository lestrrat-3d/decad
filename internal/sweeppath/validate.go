package sweeppath

import (
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// AffinePath is the validated exact path used by sweep proofs.
type AffinePath struct {
	From, To  r3.Transform
	Delta     [3]proofarith.Dyadic // displacement over the full path, in millimetres
	Duration  *big.Rat             // exact seconds represented by the input value
	Drift     *RigidDriftSegment
	Screw     *r3.Screw
	Read      *r3.Screw // the read screw of a rotating PoseSegment, admitted or not
	Supported bool
}

func ExactBaseValue(v units.Value) (*big.Rat, bool) {
	m, f := proofarith.FloatRat(v.Mag()), proofarith.FloatRat(v.Unit().Factor())
	if m == nil || f == nil {
		return nil, false
	}
	return new(big.Rat).Mul(m, f), true
}

func Validate(path PairPath) (AffinePath, error) {
	var out AffinePath
	switch p := path.(type) {
	case PoseSegment:
		out.From, out.To = p.From, p.To
		if err := motionbound.ValidateBetween(motionbound.Between{From: p.From, To: p.To}); err != nil {
			return out, err
		}
		out.Duration, _ = ExactBaseValue(p.Duration)
		if err := DurationValid(p.Duration, out.Duration); err != nil {
			return out, err
		}
		if p.From.Basis() != p.To.Basis() {
			inverse, err := p.From.Inverse()
			if err != nil {
				return out, err
			}
			relative, err := inverse.Then(p.To)
			if err != nil {
				return out, err
			}
			screw, err := relative.Screw()
			if err != nil {
				return out, err
			}
			out.Read = &screw
			if _, _, ok := clearance.SignedAxis(screw.Axis); ok && screw.Angle.Base() > 0 &&
				allFinite(screw.Point.X, screw.Point.Y, screw.Point.Z, screw.Slide) {
				out.Screw, out.Supported = &screw, true
			}
			return out, nil
		}
		start, end := p.From.Translation(), p.To.Translation()
		out.Delta = [3]proofarith.Dyadic{proofarith.DySubScalar(proofarith.MustDyOf(end.X), proofarith.MustDyOf(start.X)),
			proofarith.DySubScalar(proofarith.MustDyOf(end.Y), proofarith.MustDyOf(start.Y)), proofarith.DySubScalar(proofarith.MustDyOf(end.Z), proofarith.MustDyOf(start.Z))}
		out.Supported = true
	case RigidDriftSegment:
		out.From = p.From
		if !p.From.IsValid() || !proofbound.FiniteVec(p.Center) {
			return out, fmt.Errorf("%w: invalid drift placement or center", decaderr.ErrDegenerate)
		}
		out.Duration, _ = ExactBaseValue(p.Duration)
		if err := DurationValid(p.Duration, out.Duration); err != nil {
			return out, err
		}
		for _, v := range []units.Value{p.LinearVelocity.X, p.LinearVelocity.Y, p.LinearVelocity.Z} {
			if err := motionbound.MotionValueValid(v, units.Velocity, "drift linear velocity"); err != nil {
				return out, err
			}
		}
		angularKind := units.Angle.Div(units.Time)
		rotating := false
		for _, v := range []units.Value{p.AngularVelocity.X, p.AngularVelocity.Y, p.AngularVelocity.Z} {
			if v.Kind() != angularKind {
				return out, fmt.Errorf("%w: drift angular velocity must have Angle/Time kind", decaderr.ErrUnitKind)
			}
			registered, ok := units.Lookup(v.Unit().Symbol())
			if !ok || registered != v.Unit() {
				return out, fmt.Errorf("%w: drift angular velocity requires a registered unit", decaderr.ErrUnitKind)
			}
			if !allFinite(v.Base()) {
				return out, fmt.Errorf("%w: nonfinite drift angular velocity", decaderr.ErrNotFinite)
			}
			if v.Base() != 0 {
				rotating = true
			}
		}
		if rotating {
			out.Drift = &p
			out.Supported = true
			return out, nil
		}
		for i, v := range []units.Value{p.LinearVelocity.X, p.LinearVelocity.Y, p.LinearVelocity.Z} {
			base, ok := ExactBaseValue(v)
			if !ok {
				return out, fmt.Errorf("%w: nonfinite drift velocity", decaderr.ErrNotFinite)
			}
			full := new(big.Rat).Mul(base, out.Duration)
			d, ok := proofarith.DyOfRat(full)
			if !ok {
				return out, fmt.Errorf("%w: drift displacement is not dyadic", decaderr.ErrUnsupported)
			}
			out.Delta[i] = d
		}
		out.Supported = true
	default:
		return out, fmt.Errorf("%w: unknown pair path", decaderr.ErrDegenerate)
	}
	return out, nil
}

func DurationValid(v units.Value, base *big.Rat) error {
	if err := motionbound.MotionValueValid(v, units.Time, "sweep duration"); err != nil {
		return err
	}
	if base == nil || base.Sign() <= 0 {
		return fmt.Errorf("%w: sweep duration must be positive", decaderr.ErrDegenerate)
	}
	return nil
}

func (p AffinePath) PoseAt(f *big.Rat) (r3.Transform, error) {
	if f.Sign() == 0 {
		return p.From, nil
	}
	if f.Cmp(big.NewRat(1, 1)) == 0 && p.To.IsValid() {
		return p.To, nil
	}
	d := r3.NewVec(RatFloatNearest(new(big.Rat).Mul(p.Delta[0].Rat(), f)),
		RatFloatNearest(new(big.Rat).Mul(p.Delta[1].Rat(), f)),
		RatFloatNearest(new(big.Rat).Mul(p.Delta[2].Rat(), f)))
	step, err := r3.Translation(d)
	if err != nil {
		return r3.Transform{}, err
	}
	return p.From.Then(step)
}

func RatFloatNearest(v *big.Rat) float64 { f, _ := v.Float64(); return f }

func allFinite(values ...float64) bool {
	return !slices.ContainsFunc(values, proofbound.IsNonFinite)
}
