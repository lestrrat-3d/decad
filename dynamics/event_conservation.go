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
	applied, valid := eventAppliedImpulse(event)
	if !valid {
		return "contact event has invalid impulse or normal"
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
	energyUpper.Quo(energyUpper, big.NewRat(2, 1))
	if hasKinematic {
		work, workAllowance, ok := w.kinematicEventWork(event, applied, impulseLimit)
		if !ok {
			return "contact event has invalid kinematic work inputs"
		}
		energyUpper.Sub(energyUpper, work)
		energyAllowance.Add(energyAllowance, workAllowance)
	}
	if energyUpper.Cmp(energyAllowance) > 0 {
		if !hasKinematic {
			return "contact event increases kinetic energy beyond numerical residual"
		}
		return "contact event increases kinetic energy beyond work and numerical residual"
	}
	return ""
}

// eventAppliedImpulse returns the exact held aggregate impulse on B. A
// transition has no manifold and carries a typed zero impulse.
func eventAppliedImpulse(event ContactEvent) ([3]*big.Rat, bool) {
	var applied [3]*big.Rat
	for axis := range applied {
		applied[axis] = exactBase(velocityComponent(event.TangentImpulse, axis))
		if applied[axis] == nil {
			return applied, false
		}
	}
	if event.Kind == ContactTransition {
		return applied, true
	}
	if len(event.Manifold.Points) == 0 {
		return applied, false
	}
	point := event.Manifold.Points[0]
	if point.Normal.Bound.Base() != 0 || point.NormalAngle.Base() != 0 {
		return applied, false
	}
	normalImpulse := exactBase(event.NormalImpulse)
	if normalImpulse == nil {
		return applied, false
	}
	components := [3]float64{point.Normal.Value.X, point.Normal.Value.Y, point.Normal.Value.Z}
	for axis, component := range components {
		normal := new(big.Rat).SetFloat64(component)
		if normal == nil {
			return applied, false
		}
		applied[axis].Add(applied[axis], new(big.Rat).Mul(normalImpulse, normal))
	}
	return applied, true
}

// kinematicEventWork uses the reaction impulse on the driver to calculate
// work delivered to the dynamic body. The per-axis impulse residual widens
// the event energy gate without changing the reported numerical work.
func (w *World) kinematicEventWork(event ContactEvent, applied [3]*big.Rat,
	impulseLimit *big.Rat) (*big.Rat, *big.Rat, bool) {
	work, allowance := new(big.Rat), new(big.Rat)
	for i, part := range w.parts {
		if part.definition.Role != Kinematic {
			continue
		}
		pre, post := eventBodyVelocities(event, i)
		if validateQuantityVec(pre, units.Velocity) != nil || pre != post {
			return nil, nil, false
		}
		sign := int64(1)
		if i == 1 {
			sign = -1
		}
		for axis, impulse := range applied {
			speed := exactBase(velocityComponent(pre, axis))
			if speed == nil {
				return nil, nil, false
			}
			work.Add(work, new(big.Rat).Mul(new(big.Rat).Mul(impulse, speed), big.NewRat(sign, 1)))
			allowance.Add(allowance, new(big.Rat).Mul(absRat(new(big.Rat).Set(speed)), impulseLimit))
		}
	}
	return work, allowance, true
}

func eventBodyVelocities(event ContactEvent, index int) (QuantityVec, QuantityVec) {
	if index == 0 {
		return event.PreVelocityA, event.PostVelocityA
	}
	return event.PreVelocityB, event.PostVelocityB
}
