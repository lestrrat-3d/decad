package dynamics

import (
	"math/big"

	"github.com/lestrrat-3d/units"
)

// eventConservationFailure checks the numerical event separately from the
// full-step force kick. Each body's one mass interval is shared by its pre
// and post velocities, including in the kinetic-energy difference.
func (w *World) eventConservationFailure(event ContactEvent) string {
	if event.Pair != (BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body}) ||
		event.NormalImpulse.Kind() != units.Impulse ||
		validateQuantityVec(event.TangentImpulse, units.Impulse) != nil {
		return "contact event has invalid conservation inputs"
	}
	normalImpulse := exactBase(event.NormalImpulse)
	if normalImpulse == nil || normalImpulse.Sign() < 0 {
		return "contact event has invalid normal impulse"
	}
	var applied [3]*big.Rat
	for axis := range applied {
		applied[axis] = exactBase(velocityComponent(event.TangentImpulse, axis))
		if applied[axis] == nil {
			return "contact event has invalid tangent impulse"
		}
	}
	if event.Kind == ContactTransition {
		if normalImpulse.Sign() != 0 || len(event.Manifold.Points) != 0 {
			return "contact transition changes impulse or retains a manifold"
		}
		for _, component := range applied {
			if component.Sign() != 0 {
				return "contact transition carries a tangent impulse"
			}
		}
		for i := range w.parts {
			pre, post := eventBodyVelocities(event, i)
			if validateQuantityVec(pre, units.Velocity) != nil ||
				validateQuantityVec(post, units.Velocity) != nil || pre != post {
				return "contact transition changes velocity"
			}
		}
		return ""
	}
	if event.Kind != ContactImpact || normalImpulse.Sign() == 0 || len(event.Manifold.Points) == 0 {
		return "contact impact lacks an impulse or manifold"
	}
	point := event.Manifold.Points[0]
	if point.Normal.Bound.Base() != 0 || point.NormalAngle.Base() != 0 {
		return "contact event normal is not exact"
	}
	components := [3]float64{point.Normal.Value.X, point.Normal.Value.Y, point.Normal.Value.Z}
	for axis, component := range components {
		normal := new(big.Rat).SetFloat64(component)
		if normal == nil {
			return "contact event normal is not finite"
		}
		applied[axis].Add(applied[axis], new(big.Rat).Mul(normalImpulse, normal))
	}
	impulseLimit, velocityLimit := exactBase(w.step.ImpulseResidual), exactBase(w.step.VelocityResidual)
	if impulseLimit == nil || velocityLimit == nil {
		return "contact event residual limits are invalid"
	}
	energyUpper, energyAllowance := new(big.Rat), new(big.Rat)
	hasKinematic := false
	for i, part := range w.parts {
		if part.definition.Role == Kinematic {
			hasKinematic = true
		}
		if part.definition.Role != Dynamic {
			continue
		}
		mass, bound := exactBase(part.mass.Mass.Value), exactBase(part.mass.Mass.Bound)
		if mass == nil || bound == nil {
			return "contact event mass is not finite"
		}
		low, high := new(big.Rat).Sub(mass, bound), new(big.Rat).Add(mass, bound)
		if low.Sign() <= 0 {
			return "contact event mass is not positive"
		}
		pre, post := eventBodyVelocities(event, i)
		if validateQuantityVec(pre, units.Velocity) != nil ||
			validateQuantityVec(post, units.Velocity) != nil {
			return "contact event has invalid velocities"
		}
		sign := int64(1)
		if i == 0 {
			sign = -1
		}
		squaredChange := new(big.Rat)
		for axis := range applied {
			before, after := exactBase(velocityComponent(pre, axis)), exactBase(velocityComponent(post, axis))
			change := new(big.Rat).Sub(after, before)
			impulse := new(big.Rat).Mul(applied[axis], big.NewRat(sign, 1))
			atLow := new(big.Rat).Sub(new(big.Rat).Mul(low, change), impulse)
			atHigh := new(big.Rat).Sub(new(big.Rat).Mul(high, change), impulse)
			residual := absRat(atLow)
			if upper := absRat(atHigh); upper.Cmp(residual) > 0 {
				residual = upper
			}
			momentumAllowance := new(big.Rat).Add(impulseLimit,
				new(big.Rat).Mul(high, velocityLimit))
			if residual.Cmp(momentumAllowance) > 0 {
				return "contact event linear momentum exceeds impulse residual"
			}
			beforeSquared := new(big.Rat).Mul(before, before)
			afterSquared := new(big.Rat).Mul(after, after)
			squaredChange.Add(squaredChange, new(big.Rat).Sub(afterSquared, beforeSquared))
			speedSum := new(big.Rat).Add(absRat(new(big.Rat).Set(before)),
				absRat(new(big.Rat).Set(after)))
			speedSum.Add(speedSum, velocityLimit)
			axisAllowance := new(big.Rat).Mul(new(big.Rat).Add(
				new(big.Rat).Mul(high, velocityLimit), impulseLimit), speedSum)
			energyAllowance.Add(energyAllowance, axisAllowance)
		}
		// The sign of the whole squared-speed difference selects the mass
		// endpoint. Separate pre/post energy intervals would hide a gain.
		if squaredChange.Sign() > 0 {
			energyUpper.Add(energyUpper, new(big.Rat).Mul(high, squaredChange))
		} else {
			energyUpper.Add(energyUpper, new(big.Rat).Mul(low, squaredChange))
		}
	}
	if hasKinematic {
		return ""
	}
	energyUpper.Quo(energyUpper, big.NewRat(2, 1))
	if energyUpper.Cmp(energyAllowance) > 0 {
		return "contact event increases kinetic energy beyond numerical residual"
	}
	return ""
}

func eventBodyVelocities(event ContactEvent, index int) (QuantityVec, QuantityVec) {
	if index == 0 {
		return event.PreVelocityA, event.PostVelocityA
	}
	return event.PreVelocityB, event.PostVelocityB
}
