package sweeppath

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Motion carries the exact path inputs and the full-path travel bound used by
// the continuous pair sweep.
type Motion struct {
	Path       AffinePath
	FullTravel *big.Rat
	FromRot    motionbound.IvMat
	FromT      motionbound.RatVec
	Velocity   motionbound.RatVec
	Frame      motionbound.MotionFrame
	OmegaLow   *big.Rat
	OmegaHigh  *big.Rat
}

// PrepareMotion reads a validated path and obtains its body's radius when the
// path rotates. radius receives the path's start pose, center, and exact axis.
func PrepareMotion(path AffinePath,
	radius func(r3.Transform, r3.Vec, motionbound.RatVec) (*big.Rat, bool),
) (Motion, bool) {
	if path.Screw != nil {
		axis := path.Screw.Axis
		angle, ok := ExactBaseValue(path.Screw.Angle)
		if !ok || path.Duration.Sign() <= 0 {
			return Motion{}, false
		}
		angular := new(big.Rat).Quo(angle, path.Duration)
		linear := new(big.Rat).Quo(proofarith.FloatRat(path.Screw.Slide), path.Duration)
		omega, speed := RatFloatNearest(angular), RatFloatNearest(linear)
		if !allFinite(omega, speed) {
			return Motion{}, false
		}
		path.Drift = &RigidDriftSegment{From: path.From, Center: path.Screw.Point,
			LinearVelocity: QuantityVec{X: units.MillimetersPerSecond(axis.X * speed),
				Y: units.MillimetersPerSecond(axis.Y * speed),
				Z: units.MillimetersPerSecond(axis.Z * speed)},
			AngularVelocity: QuantityVec{X: units.RadiansPerSecond(axis.X * omega),
				Y: units.RadiansPerSecond(axis.Y * omega),
				Z: units.RadiansPerSecond(axis.Z * omega)},
			Duration: units.Seconds(RatFloatNearest(path.Duration))}
	}
	fromRot, fromT, ok := motionbound.ExactTransform(path.From)
	if !ok {
		return Motion{}, false
	}
	prepared := Motion{Path: path, FromRot: fromRot, FromT: fromT}
	if path.Drift == nil {
		travel, ok := motionbound.SweepLinearTravel(path.Delta)
		if !ok {
			return Motion{}, false
		}
		prepared.FullTravel = travel
		return prepared, true
	}
	drift := path.Drift
	velocity := [3]units.Value{drift.LinearVelocity.X, drift.LinearVelocity.Y, drift.LinearVelocity.Z}
	angular := [3]units.Value{drift.AngularVelocity.X, drift.AngularVelocity.Y, drift.AngularVelocity.Z}
	omega := motionbound.RatVec{}
	for axis := range 3 {
		prepared.Velocity[axis], _ = ExactBaseValue(velocity[axis])
		omega[axis], _ = ExactBaseValue(angular[axis])
	}
	var vSquared *big.Rat
	prepared.Frame, prepared.OmegaLow, prepared.OmegaHigh, vSquared, ok =
		motionbound.SweepAngularFrame(prepared.Velocity, omega, drift.Center)
	if !ok {
		return Motion{}, false
	}
	rho, ok := radius(path.From, drift.Center, omega)
	if !ok {
		return Motion{}, false
	}
	prepared.FullTravel, ok = motionbound.SweepRotatingTravel(vSquared, prepared.OmegaHigh, rho, path.Duration)
	if !ok {
		return Motion{}, false
	}
	if path.Screw != nil {
		angle, _ := ExactBaseValue(path.Screw.Angle)
		angular := new(big.Rat).Quo(angle, path.Duration)
		linear := new(big.Rat).Quo(proofarith.FloatRat(path.Screw.Slide), path.Duration)
		for axis, component := range [3]float64{path.Screw.Axis.X, path.Screw.Axis.Y, path.Screw.Axis.Z} {
			prepared.Velocity[axis] = new(big.Rat).Mul(proofarith.FloatRat(component), linear)
		}
		prepared.OmegaLow, prepared.OmegaHigh = angular, angular
		prepared.FullTravel = new(big.Rat).Mul(new(big.Rat).Add(new(big.Rat).Abs(linear),
			new(big.Rat).Mul(rho, angular)), path.Duration)
	}
	return prepared, true
}

// RoundedPoseAt evaluates the float pose for the path's sample fraction.
// The screw endpoint uses the stated To transform exactly.
func (p AffinePath) RoundedPoseAt(f *big.Rat) (r3.Transform, error) {
	if p.Screw != nil {
		if f.Sign() == 0 {
			return p.From, nil
		}
		if f.Cmp(big.NewRat(1, 1)) == 0 {
			return p.To, nil
		}
		step, err := p.Screw.At(RatFloatNearest(f))
		if err != nil {
			return r3.Transform{}, err
		}
		return p.From.Then(step)
	}
	if p.Drift == nil {
		return p.PoseAt(f)
	}
	if f.Sign() == 0 {
		return p.From, nil
	}
	drift := p.Drift
	elapsed := RatFloatNearest(new(big.Rat).Mul(p.Duration, f))
	axis := r3.Vec{X: drift.AngularVelocity.X.Base(), Y: drift.AngularVelocity.Y.Base(),
		Z: drift.AngularVelocity.Z.Base()}
	norm := math.Hypot(axis.X, math.Hypot(axis.Y, axis.Z))
	turn, err := r3.RotationAround(drift.Center, axis, units.Radians(norm*elapsed))
	if err != nil {
		return r3.Transform{}, err
	}
	pose, err := p.From.Then(turn)
	if err != nil {
		return r3.Transform{}, err
	}
	shift, err := r3.Translation(r3.Vec{X: drift.LinearVelocity.X.Base() * elapsed,
		Y: drift.LinearVelocity.Y.Base() * elapsed, Z: drift.LinearVelocity.Z.Base() * elapsed})
	if err != nil {
		return r3.Transform{}, err
	}
	return pose.Then(shift)
}
