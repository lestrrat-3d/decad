package dynamics

import (
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
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
	angularPre, angularPost, eventPoses, hasAngular, valid := eventAngularReadings(event)
	if !valid {
		return "contact event has invalid angular inputs"
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
			if hasAngular && angularPre[i] != angularPost[i] {
				return "contact transition changes angular velocity"
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
		if hasAngular {
			_, spinUpper, ok := spinEnergyChange(part.mass.Inertia, eventPoses[i],
				angularPre[i], angularPost[i])
			if !ok {
				return "contact event rotational energy cannot be enclosed"
			}
			energyUpper.Add(energyUpper, spinUpper)
			if reason := w.eventAngularImpulseFailure(event, i, angularPre[i], angularPost[i],
				eventPoses[i], impulseLimit); reason != "" {
				return reason
			}
			// A velocity residual can change rotational energy by at most the
			// inertia row ceiling times its component speed sum.
			inertiaUpper := inertiaRowCeiling(part.mass.Inertia)
			angularLimit := exactBase(w.step.AngularVelocityResidual)
			if inertiaUpper == nil || angularLimit == nil {
				return "contact event angular residual inputs are invalid"
			}
			for axis := range 3 {
				speed := new(big.Rat).Add(absRat(exactBase(velocityComponent(angularPre[i], axis))),
					absRat(exactBase(velocityComponent(angularPost[i], axis))))
				speed.Add(speed, angularLimit)
				energyAllowance.Add(energyAllowance, new(big.Rat).Mul(
					new(big.Rat).Mul(big.NewRat(3, 1),
						new(big.Rat).Mul(inertiaUpper, angularLimit)), speed))
			}
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

func eventAngularReadings(event ContactEvent) ([2]QuantityVec, [2]QuantityVec,
	[2]r3.Transform, bool, bool) {
	pre := [2]QuantityVec{event.PreAngularVelocityA, event.PreAngularVelocityB}
	post := [2]QuantityVec{event.PostAngularVelocityA, event.PostAngularVelocityB}
	poses := [2]r3.Transform{event.PoseA, event.PoseB}
	if pre == ([2]QuantityVec{}) && post == ([2]QuantityVec{}) &&
		poses == ([2]r3.Transform{}) {
		return pre, post, poses, false, true
	}
	for i := range pre {
		if validateQuantityVec(pre[i], units.AngularVelocity) != nil ||
			validateQuantityVec(post[i], units.AngularVelocity) != nil ||
			!poses[i].IsValid() || poses[i].IsReflection() {
			return pre, post, poses, true, false
		}
	}
	return pre, post, poses, true, true
}

func inertiaRowCeiling(inertia decad.InertiaReading) *big.Rat {
	components := [3][3]decad.Measurement{
		{inertia.XX, inertia.XY, inertia.XZ},
		{inertia.XY, inertia.YY, inertia.YZ},
		{inertia.XZ, inertia.YZ, inertia.ZZ}}
	maximum := new(big.Rat)
	for _, row := range components {
		sum := new(big.Rat)
		for _, entry := range row {
			value, bound := exactBase(entry.Value), exactBase(entry.Bound)
			if value == nil || bound == nil || bound.Sign() < 0 {
				return nil
			}
			sum.Add(sum, absRat(value)).Add(sum, bound)
		}
		if sum.Cmp(maximum) > 0 {
			maximum = sum
		}
	}
	return maximum
}

// eventAngularImpulseFailure checks the published spin change against every
// contact point's signed torque about this body's world mass center.
func (w *World) eventAngularImpulseFailure(event ContactEvent, body int,
	pre, post QuantityVec, pose r3.Transform, impulseLimit *big.Rat) string {
	if len(event.PointImpulses) != len(event.Manifold.Points) {
		return "contact event lacks point impulses for angular response"
	}
	mass := w.parts[body].mass
	center, centerError, ok := worldCenterReading(pose, mass.Center)
	if !ok {
		return "contact event mass center cannot be enclosed"
	}
	beforeSpin, ok := spinReadings(mass.Inertia, pose, pre)
	if !ok {
		return "contact event angular momentum cannot be enclosed"
	}
	afterSpin, ok := spinReadings(mass.Inertia, pose, post)
	if !ok {
		return "contact event angular momentum cannot be enclosed"
	}
	var torque, torqueError [3]*big.Rat
	var aggregate [3]*big.Rat
	for axis := range torque {
		torque[axis], torqueError[axis] = new(big.Rat), new(big.Rat)
		aggregate[axis] = new(big.Rat)
	}
	sign := int64(1)
	if body == 0 {
		sign = -1
	}
	for i, point := range event.Manifold.Points {
		impulse := event.PointImpulses[i]
		if impulse.Normal.Kind() != units.Impulse || !finite(impulse.Normal.Base()) ||
			validateQuantityVec(impulse.Tangent, units.Impulse) != nil ||
			impulse.Normal.Base() < 0 || point.Normal.Bound.Base() != 0 ||
			point.NormalAngle.Base() != 0 {
			return "contact event has invalid point impulse"
		}
		witness := point.OnA
		if body == 1 {
			witness = point.OnB
		}
		pointBound := exactBase(witness.Bound)
		if pointBound == nil || pointBound.Sign() < 0 {
			return "contact event point bound is invalid"
		}
		var arm, action [3]*big.Rat
		coordinates := [3]float64{witness.Value.X, witness.Value.Y, witness.Value.Z}
		normal := [3]float64{point.Normal.Value.X, point.Normal.Value.Y, point.Normal.Value.Z}
		for axis := range arm {
			coordinate, direction := new(big.Rat).SetFloat64(coordinates[axis]),
				new(big.Rat).SetFloat64(normal[axis])
			if coordinate == nil || direction == nil {
				return "contact event point coordinates are invalid"
			}
			normalImpulse := exactBase(impulse.Normal)
			tangentImpulse := exactBase(velocityComponent(impulse.Tangent, axis))
			if normalImpulse == nil || tangentImpulse == nil {
				return "contact event has invalid point impulse"
			}
			arm[axis] = new(big.Rat).Sub(coordinate, center[axis])
			action[axis] = new(big.Rat).Add(
				new(big.Rat).Mul(normalImpulse, direction), tangentImpulse)
			aggregate[axis].Add(aggregate[axis], action[axis])
		}
		for axis := range torque {
			a, b := (axis+1)%3, (axis+2)%3
			cross := new(big.Rat).Sub(new(big.Rat).Mul(arm[a], action[b]),
				new(big.Rat).Mul(arm[b], action[a]))
			torque[axis].Add(torque[axis], cross.Mul(cross, big.NewRat(sign, 1)))
			uncertainty := new(big.Rat).Add(pointBound, centerError[a])
			torqueError[axis].Add(torqueError[axis], new(big.Rat).Mul(uncertainty,
				absRat(new(big.Rat).Set(action[b]))))
			uncertainty = new(big.Rat).Add(pointBound, centerError[b])
			torqueError[axis].Add(torqueError[axis], new(big.Rat).Mul(uncertainty,
				absRat(new(big.Rat).Set(action[a]))))
			torqueError[axis].Add(torqueError[axis], new(big.Rat).Mul(impulseLimit,
				new(big.Rat).Add(absRat(new(big.Rat).Set(arm[a])),
					absRat(new(big.Rat).Set(arm[b])))))
		}
	}
	applied, ok := eventAppliedImpulse(event)
	if !ok {
		return "contact event aggregate impulse cannot be enclosed"
	}
	for axis := range aggregate {
		if absRat(new(big.Rat).Sub(aggregate[axis], applied[axis])).Cmp(impulseLimit) > 0 {
			return "contact event point impulses do not match aggregate impulse"
		}
	}
	row := inertiaRowCeiling(mass.Inertia)
	angularLimit := exactBase(w.step.AngularVelocityResidual)
	if row == nil || angularLimit == nil {
		return "contact event inertia residual cannot be enclosed"
	}
	for axis := range torque {
		spin := new(big.Rat).Sub(afterSpin.value[axis], beforeSpin.value[axis])
		low := new(big.Rat).Sub(afterSpin.low[axis], beforeSpin.high[axis])
		high := new(big.Rat).Sub(afterSpin.high[axis], beforeSpin.low[axis])
		residual := absRat(new(big.Rat).Sub(spin, torque[axis]))
		allowed := new(big.Rat).Add(torqueError[axis], intervalDeviation(spin, low, high))
		allowed.Add(allowed, new(big.Rat).Mul(big.NewRat(3, 1),
			new(big.Rat).Mul(row, angularLimit)))
		if residual.Cmp(allowed) > 0 {
			return "contact event angular momentum exceeds point impulse residual"
		}
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
