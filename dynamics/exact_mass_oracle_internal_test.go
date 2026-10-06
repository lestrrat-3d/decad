package dynamics

import (
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// The functions in this file are the forms the island certificate, the
// conservation readings and the pre-solve classification computed before
// each dynamic body's mass reading was converted once per World
// (exact_mass.go) and before the zero shortcuts of the certificate and the
// readings. They convert every reading where they read it, through
// SetFloat64 and an unconditional unit-factor product, and take no
// shortcut. exact_mass_internal_test.go feeds them and the current forms
// the same inputs and requires exactly equal rationals and decisions. A
// deliberate change to what the certificate or a reading computes changes
// the matching form here with it.

func oldRatFloat(value float64) *big.Rat { return new(big.Rat).SetFloat64(value) }

func oldRatVec(v r3.Vec) ([3]*big.Rat, bool) {
	var out [3]*big.Rat
	for axis, x := range [3]float64{v.X, v.Y, v.Z} {
		out[axis] = oldRatFloat(x)
		if out[axis] == nil {
			return out, false
		}
	}
	return out, true
}

func oldExactBase(value units.Value) *big.Rat {
	mag := new(big.Rat).SetFloat64(value.Mag())
	factor := new(big.Rat).SetFloat64(value.Unit().Factor())
	if mag == nil || factor == nil {
		return nil
	}
	return new(big.Rat).Mul(mag, factor)
}

func oldQuantityRats(q QuantityVec) ([3]*big.Rat, bool) {
	var out [3]*big.Rat
	for axis := range out {
		out[axis] = oldExactBase(velocityComponent(q, axis))
		if out[axis] == nil {
			return out, false
		}
	}
	return out, true
}

func oldNewCertBody(w *World, index int, entry BodyState, post BodyState,
	drive map[int]driverMotion) (certBody, bool) {
	body := certBody{index: index, dynamic: w.bodies[index].definition.Role == Dynamic, pose: entry.Pose}
	zero := [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
	body.v, body.w, body.vPost, body.wPost = zero, zero, zero, zero
	if motion, ok := drive[index]; ok && w.bodies[index].definition.Role == Kinematic {
		body.kinematic = true
		body.v, body.vPost, body.w, body.wPost = motion.linear, motion.linear, motion.angular, motion.angular
		body.center = zeroIVec()
	}
	if !body.dynamic {
		return body, true
	}
	mass := w.bodies[index].mass
	m, bound := oldExactBase(mass.Mass.Value), oldExactBase(mass.Mass.Bound)
	if m == nil || bound == nil || bound.Sign() < 0 {
		return certBody{}, false
	}
	body.mass = proof.OwnedInterval(new(big.Rat).Sub(m, bound), new(big.Rat).Add(m, bound))
	if body.mass.Lo.Sign() <= 0 {
		return certBody{}, false
	}
	largest := new(big.Rat)
	components := [3][3]decad.Measurement{
		{mass.Inertia.XX, mass.Inertia.XY, mass.Inertia.XZ},
		{mass.Inertia.XY, mass.Inertia.YY, mass.Inertia.YZ},
		{mass.Inertia.XZ, mass.Inertia.YZ, mass.Inertia.ZZ}}
	for i, row := range components {
		for j, component := range row {
			value, componentBound := oldExactBase(component.Value), oldExactBase(component.Bound)
			if value == nil || componentBound == nil || componentBound.Sign() < 0 {
				return certBody{}, false
			}
			body.inertia[i][j] = proof.OwnedInterval(new(big.Rat).Sub(value, componentBound),
				new(big.Rat).Add(value, componentBound))
			if m := magnitude(body.inertia[i][j]); m.Cmp(largest) > 0 {
				largest = m
			}
		}
	}
	basis := entry.Pose.Basis()
	for column, axis := range [3]r3.Vec{basis.EX, basis.EY, basis.EZ} {
		values, ok := oldRatVec(axis)
		if !ok {
			return certBody{}, false
		}
		for row := range 3 {
			body.rotation[row][column] = values[row]
		}
	}
	d := new(big.Rat)
	for i := range 3 {
		for j := range 3 {
			entry := new(big.Rat)
			for k := range 3 {
				entry.Add(entry, new(big.Rat).Mul(body.rotation[k][i], body.rotation[k][j]))
			}
			if i == j {
				entry.Sub(entry, big.NewRat(1, 1))
			}
			d.Add(d, absRat(entry))
		}
	}
	body.defect = new(big.Rat).Mul(big.NewRat(3, 1), d)
	body.defect.Mul(body.defect, new(big.Rat).Add(big.NewRat(2, 1), d))
	body.defect.Mul(body.defect, largest)
	body.inertiaLower = certifiedInertiaFloor(mass)
	body.rowCeiling = inertiaRowCeiling(mass.Inertia)
	if body.inertiaLower == nil || body.inertiaLower.Sign() <= 0 || body.rowCeiling == nil {
		return certBody{}, false
	}
	center, centerError, ok := oldWorldCenterReading(entry.Pose, mass.Center)
	if !ok {
		return certBody{}, false
	}
	body.centerL1 = new(big.Rat)
	for axis := range 3 {
		body.center[axis] = proof.OwnedInterval(new(big.Rat).Sub(center[axis], centerError[axis]),
			new(big.Rat).Add(center[axis], centerError[axis]))
		body.centerL1.Add(body.centerL1, magnitude(body.center[axis]))
	}
	var valid [4]bool
	body.v, valid[0] = oldQuantityRats(entry.LinearVelocity)
	body.w, valid[1] = oldQuantityRats(entry.AngularVelocity)
	body.vPost, valid[2] = oldQuantityRats(post.LinearVelocity)
	body.wPost, valid[3] = oldQuantityRats(post.AngularVelocity)
	return body, valid[0] && valid[1] && valid[2] && valid[3]
}

func oldPointVelocity(body certBody, v, omega [3]*big.Rat, lever ivec) ivec {
	if !body.dynamic && !body.kinematic {
		return pointIVec(v)
	}
	return addIVec(pointIVec(v), proof.CrossInterval3(pointIVec(omega), lever))
}

func oldPreNormalSpeed(p certPoint, bodies []certBody) proof.RatInterval {
	relative := subIVec(oldPointVelocity(bodies[p.b], bodies[p.b].v, bodies[p.b].w, p.rB),
		oldPointVelocity(bodies[p.a], bodies[p.a].v, bodies[p.a].w, p.rA))
	return proof.DotInterval3(relative, p.normal)
}

func oldCertifyIsland(w *World, bodies []certBody, points []certPoint) islandCertificate {
	cert := islandCertificate{linear: new(big.Rat), angular: new(big.Rat), normal: new(big.Rat),
		energy: new(big.Rat), momentum: new(big.Rat), angularMomentum: new(big.Rat),
		cone: new(big.Rat), tangent: new(big.Rat), spin: new(big.Rat),
		witnessTorque: new(big.Rat), witnessSpin: new(big.Rat)}
	impulseLimit, velocityLimit := oldExactBase(w.step.ImpulseResidual), oldExactBase(w.step.VelocityResidual)
	angularLimit, impactSpeed := oldExactBase(w.step.AngularVelocityResidual), oldExactBase(w.step.ImpactSpeed)
	// The island's largest lever bound ρ.
	rho := new(big.Rat)
	for _, p := range points {
		if bodies[p.a].dynamic {
			raise(&rho, euclideanUpper(p.rA))
		}
		if bodies[p.b].dynamic {
			raise(&rho, euclideanUpper(p.rB))
		}
	}
	// Each point's impulse on B, λ·n + λt; A receives its negation.
	impulses := make([]ivec, len(points))
	for k, p := range points {
		impulses[k] = addIVec(scaleIVec(p.normal, p.lambda), pointIVec(p.tangent))
	}
	linearMomentum, angularMomentum := zeroIVec(), zeroIVec()
	linearMomentumLimit, angularMomentumLimit := new(big.Rat), new(big.Rat)
	energyUpper, energyAllowance, kinematicWork := new(big.Rat), new(big.Rat), new(big.Rat)
	for slot, body := range bodies {
		if !body.dynamic {
			continue
		}
		raise(&cert.spin, euclideanUpper(pointIVec(body.wPost)))
		var dv, dw [3]*big.Rat
		for axis := range 3 {
			dv[axis] = new(big.Rat).Sub(body.vPost[axis], body.v[axis])
			dw[axis] = new(big.Rat).Sub(body.wPost[axis], body.w[axis])
		}
		force, torque := zeroIVec(), zeroIVec()
		for k, p := range points {
			switch slot {
			case p.b:
				force = addIVec(force, impulses[k])
				torque = addIVec(torque, proof.CrossInterval3(p.rB, impulses[k]))
			case p.a:
				force = subIVec(force, impulses[k])
				torque = subIVec(torque, proof.CrossInterval3(p.rA, impulses[k]))
			}
		}
		// Linear law: m·(v' − v) − ΣJ over the mass interval.
		linearLimit := new(big.Rat).Add(impulseLimit, new(big.Rat).Mul(body.mass.Hi, velocityLimit))
		momentumChange := ivec{}
		for axis := range 3 {
			momentumChange[axis] = proof.MulInterval(body.mass, proof.PointInterval(dv[axis]))
			residual := magnitude(proof.SubInterval(momentumChange[axis], force[axis]))
			raise(&cert.linear, residual)
			if residual.Cmp(linearLimit) > 0 {
				cert.fail(gateLinearLaw, residual, linearLimit)
			}
		}
		// Angular law: I_world·(ω' − ω) − Σ r×J over the inertia and lever
		// intervals. The limit carries the body's witness torque T_β; the
		// mass-center ball, the normal ball and the inertia intervals stay in
		// the residual alone.
		spinChange := body.inertiaApply(dw)
		witnessTorque := bodyWitnessTorque(slot, points, impulses)
		raise(&cert.witnessTorque, witnessTorque)
		raise(&cert.witnessSpin, new(big.Rat).Quo(witnessTorque, body.inertiaLower))
		angularBodyLimit := new(big.Rat).Add(new(big.Rat).Mul(impulseLimit, rho),
			new(big.Rat).Mul(body.inertiaLower, angularLimit))
		angularBodyLimit.Add(angularBodyLimit, witnessTorque)
		for axis := range 3 {
			residual := magnitude(proof.SubInterval(spinChange[axis], torque[axis]))
			raise(&cert.angular, residual)
			if residual.Cmp(angularBodyLimit) > 0 {
				cert.fail(gateAngularLaw, residual, angularBodyLimit)
			}
		}
		// Island momentum about the world origin: Σ m·Δv and
		// Σ (I·Δω + c×m·Δv), against the impulses delivered by Fixed bodies.
		// The angular limit sums each body's angular-law limit, T_β included.
		linearMomentum = addIVec(linearMomentum, momentumChange)
		angularMomentum = addIVec(angularMomentum,
			addIVec(spinChange, proof.CrossInterval3(body.center, momentumChange)))
		linearMomentumLimit.Add(linearMomentumLimit, linearLimit)
		angularMomentumLimit.Add(angularMomentumLimit, angularBodyLimit)
		angularMomentumLimit.Add(angularMomentumLimit, new(big.Rat).Mul(body.centerL1, linearLimit))
		// Energy: the squared-speed difference with the mass endpoint that
		// maximises it, the rotational change over the inertia intervals, and
		// the defect widening of both rotational readings.
		squaredChange := new(big.Rat)
		for axis := range 3 {
			squaredChange.Add(squaredChange, new(big.Rat).Sub(new(big.Rat).Mul(body.vPost[axis], body.vPost[axis]),
				new(big.Rat).Mul(body.v[axis], body.v[axis])))
			speeds := new(big.Rat).Add(absRat(new(big.Rat).Set(body.v[axis])),
				absRat(new(big.Rat).Set(body.vPost[axis])))
			speeds.Add(speeds, velocityLimit)
			energyAllowance.Add(energyAllowance, new(big.Rat).Mul(linearLimit, speeds))
			spins := new(big.Rat).Add(absRat(new(big.Rat).Set(body.w[axis])),
				absRat(new(big.Rat).Set(body.wPost[axis])))
			spins.Add(spins, angularLimit)
			energyAllowance.Add(energyAllowance, new(big.Rat).Mul(
				new(big.Rat).Mul(big.NewRat(3, 1), new(big.Rat).Mul(body.rowCeiling, angularLimit)), spins))
		}
		if squaredChange.Sign() > 0 {
			energyUpper.Add(energyUpper, new(big.Rat).Mul(body.mass.Hi, squaredChange))
		} else {
			energyUpper.Add(energyUpper, new(big.Rat).Mul(body.mass.Lo, squaredChange))
		}
		spinUpper, ok := oldSpinEnergyChange(w.bodies[body.index].mass.Inertia, body.pose, oldRatQuantity(body.w), oldRatQuantity(body.wPost))
		if !ok {
			cert.fail(gateEnergy, new(big.Rat), new(big.Rat))
			continue
		}
		energyUpper.Add(energyUpper, spinUpper)
		for _, omega := range [2][3]*big.Rat{body.w, body.wPost} {
			l1 := new(big.Rat)
			for _, value := range omega {
				l1.Add(l1, absRat(new(big.Rat).Set(value)))
			}
			energyUpper.Add(energyUpper, new(big.Rat).Mul(body.defect, new(big.Rat).Mul(l1, l1)))
		}
	}
	for k, p := range points {
		// Normal sign: λn ≥ 0 exactly.
		if p.lambda.Sign() < 0 {
			cert.fail(gateNormalSign, new(big.Rat).Neg(p.lambda), new(big.Rat))
		}
		// Restitution target from the enclosed pre-solve normal speed.
		c := oldPreNormalSpeed(p, bodies)
		e, ok := restitutionTarget(c, p.restitution, impactSpeed)
		if !ok {
			cert.fail(gateRestitutionTarget, magnitude(c), impactSpeed)
			continue
		}
		// w'·n − target = ((v' + e·v) + (ω' + e·ω)×r)_B−A · n.
		combined := func(body certBody, r ivec) ivec {
			var u, omega [3]*big.Rat
			for axis := range 3 {
				u[axis] = new(big.Rat).Add(body.vPost[axis], new(big.Rat).Mul(e, body.v[axis]))
				omega[axis] = new(big.Rat).Add(body.wPost[axis], new(big.Rat).Mul(e, body.w[axis]))
			}
			return oldPointVelocity(body, u, omega, r)
		}
		q := proof.DotInterval3(subIVec(combined(bodies[p.b], p.rB), combined(bodies[p.a], p.rA)), p.normal)
		if below := new(big.Rat).Neg(q.Lo); below.Cmp(velocityLimit) > 0 {
			cert.fail(gateNonPenetration, below, velocityLimit)
		}
		if p.lambda.Sign() > 0 {
			residual := magnitude(q)
			raise(&cert.normal, residual)
			if residual.Cmp(velocityLimit) > 0 {
				cert.fail(gateComplementarity, residual, velocityLimit)
			}
		} else if q.Lo.Sign() < 0 {
			raise(&cert.normal, new(big.Rat).Neg(q.Lo))
		}
		oldCertifyFriction(&cert, p, bodies, impulseLimit, velocityLimit)
		// Kinematic work: a driver on side A delivers J·V to the island, one
		// on side B −J·V, with J = λ·n + λt the impulse on B and V the
		// driver's field at the contact point; the energy gate admits its
		// upper end.
		for side, slot := range [2]int{p.a, p.b} {
			if !bodies[slot].kinematic {
				continue
			}
			lever := p.rA
			if side == 1 {
				lever = p.rB
			}
			field := oldPointVelocity(bodies[slot], bodies[slot].v, bodies[slot].w, lever)
			work := proof.DotInterval3(impulses[k], field)
			if side == 1 {
				work = proof.NegInterval(work)
			}
			kinematicWork.Add(kinematicWork, work.Hi)
		}
		// External impulses: those a Fixed or Kinematic body delivers. A dynamic pair's
		// two sides read the same λ and normal interval and cancel exactly in
		// the linear sum; their torques about the origin use each side's own
		// witness and so stay in the angular sum.
		for side, slot := range [2]int{p.a, p.b} {
			if !bodies[slot].dynamic {
				continue
			}
			witness, sign := p.onA, big.NewRat(-1, 1)
			if side == 1 {
				witness, sign = p.onB, big.NewRat(1, 1)
			}
			angularMomentum = subIVec(angularMomentum, scaleIVec(proof.CrossInterval3(witness, impulses[k]), sign))
			other := p.b
			if side == 1 {
				other = p.a
			}
			if !bodies[other].dynamic {
				linearMomentum = subIVec(linearMomentum, scaleIVec(impulses[k], sign))
			}
		}
	}
	energyUpper.Quo(energyUpper, big.NewRat(2, 1))
	cert.energy = energyUpper
	// The island's kinetic energy may grow by the work its drivers deliver.
	energyAllowance.Add(energyAllowance, kinematicWork)
	if energyUpper.Cmp(energyAllowance) > 0 {
		cert.fail(gateEnergy, energyUpper, energyAllowance)
	}
	for axis := range 3 {
		residual := magnitude(linearMomentum[axis])
		raise(&cert.momentum, residual)
		if residual.Cmp(linearMomentumLimit) > 0 {
			cert.fail(gateLinearMomentum, residual, linearMomentumLimit)
		}
		residual = magnitude(angularMomentum[axis])
		raise(&cert.angularMomentum, residual)
		if residual.Cmp(angularMomentumLimit) > 0 {
			cert.fail(gateAngularMomentum, residual, angularMomentumLimit)
		}
	}
	return cert
}

func oldCertifyFriction(cert *islandCertificate, p certPoint, bodies []certBody,
	impulseLimit, velocityLimit *big.Rat) {
	square := new(big.Rat)
	for _, component := range p.tangent {
		square.Add(square, new(big.Rat).Mul(component, component))
	}
	coneLower := new(big.Rat).Mul(p.mu, p.lambda)
	allowed := new(big.Rat).Add(coneLower, impulseLimit)
	norm := ratSqrtUpper(square)
	if excess := new(big.Rat).Sub(norm, coneLower); excess.Sign() > 0 {
		raise(&cert.cone, excess)
	}
	if allowed.Sign() < 0 || square.Cmp(new(big.Rat).Mul(allowed, allowed)) > 0 {
		cert.fail(gateCone, new(big.Rat).Sub(norm, coneLower), impulseLimit)
	}
	relative := subIVec(oldPointVelocity(bodies[p.b], bodies[p.b].vPost, bodies[p.b].wPost, p.rB),
		oldPointVelocity(bodies[p.a], bodies[p.a].vPost, bodies[p.a].wPost, p.rA))
	normalSpeed := proof.DotInterval3(relative, p.normal)
	var slide ivec
	for axis := range slide {
		slide[axis] = proof.SubInterval(relative[axis], proof.MulInterval(normalSpeed, p.normal[axis]))
	}
	speedUpper := euclideanUpper(slide)
	threshold := new(big.Rat).Sub(coneLower, impulseLimit)
	if threshold.Sign() > 0 && square.Cmp(new(big.Rat).Mul(threshold, threshold)) < 0 {
		raise(&cert.tangent, speedUpper)
		if speedUpper.Cmp(velocityLimit) > 0 {
			cert.fail(gateStick, speedUpper, velocityLimit)
		}
		return
	}
	opposed := proof.DotInterval3(pointIVec(p.tangent), slide).Hi
	opposed = new(big.Rat).Add(opposed, new(big.Rat).Mul(norm, speedUpper))
	limit := new(big.Rat).Mul(impulseLimit, euclideanLower(slide))
	limit.Add(limit, new(big.Rat).Mul(velocityLimit, ratSqrtLower(square)))
	if opposed.Cmp(limit) > 0 {
		cert.fail(gateSlip, opposed, limit)
	}
}

func oldRatQuantity(x [3]*big.Rat) QuantityVec {
	var out QuantityVec
	for axis, value := range x {
		f, _ := value.Float64()
		setVelocityComponent(&out, axis, units.RadiansPerSecond(f))
	}
	return out
}

func oldConservationState(w *World, state State) (ConservationState, bool) {
	energy := new(big.Rat)
	energyLow := new(big.Rat)
	energyHigh := new(big.Rat)
	var momentum, low, high, angularValue, angularLow, angularHigh [3]*big.Rat
	for axis := range momentum {
		momentum[axis], low[axis], high[axis] = new(big.Rat), new(big.Rat), new(big.Rat)
		angularValue[axis], angularLow[axis], angularHigh[axis] = new(big.Rat), new(big.Rat), new(big.Rat)
	}
	for i, part := range w.bodies {
		if part.definition.Role != Dynamic {
			continue
		}
		mass, bound := oldExactBase(part.mass.Mass.Value), oldExactBase(part.mass.Mass.Bound)
		if mass == nil || bound == nil {
			return ConservationState{}, false
		}
		massLow := new(big.Rat).Sub(mass, bound)
		massHigh := new(big.Rat).Add(mass, bound)
		if massLow.Sign() <= 0 {
			return ConservationState{}, false
		}
		center, centerError, ok := oldWorldCenterReading(state.entries[i].Pose, part.mass.Center)
		if !ok {
			return ConservationState{}, false
		}
		speedSquared := new(big.Rat)
		var velocity [3]*big.Rat
		for axis := range momentum {
			v := oldExactBase(velocityComponent(state.entries[i].LinearVelocity, axis))
			if v == nil {
				return ConservationState{}, false
			}
			velocity[axis] = v
			momentum[axis].Add(momentum[axis], new(big.Rat).Mul(mass, v))
			if v.Sign() < 0 {
				low[axis].Add(low[axis], new(big.Rat).Mul(massHigh, v))
				high[axis].Add(high[axis], new(big.Rat).Mul(massLow, v))
			} else {
				low[axis].Add(low[axis], new(big.Rat).Mul(massLow, v))
				high[axis].Add(high[axis], new(big.Rat).Mul(massHigh, v))
			}
			speedSquared.Add(speedSquared, new(big.Rat).Mul(v, v))
		}
		for axis := range angularValue {
			j, k := (axis+1)%3, (axis+2)%3
			coefficient := new(big.Rat).Sub(new(big.Rat).Mul(center[j], velocity[k]),
				new(big.Rat).Mul(center[k], velocity[j]))
			uncertainty := new(big.Rat).Add(
				new(big.Rat).Mul(centerError[j], absRat(new(big.Rat).Set(velocity[k]))),
				new(big.Rat).Mul(centerError[k], absRat(new(big.Rat).Set(velocity[j]))))
			oldAddMassProduct(angularValue[axis], angularLow[axis], angularHigh[axis],
				mass, massLow, massHigh, coefficient,
				new(big.Rat).Sub(coefficient, uncertainty), new(big.Rat).Add(coefficient, uncertainty))
		}
		energy.Add(energy, new(big.Rat).Mul(mass, speedSquared))
		energyLow.Add(energyLow, new(big.Rat).Mul(massLow, speedSquared))
		energyHigh.Add(energyHigh, new(big.Rat).Mul(massHigh, speedSquared))
		spin, ok := oldSpinReadings(part.mass.Inertia, state.entries[i].Pose,
			state.entries[i].AngularVelocity)
		if !ok {
			return ConservationState{}, false
		}
		for axis := range angularValue {
			angularValue[axis].Add(angularValue[axis], spin.value[axis])
			angularLow[axis].Add(angularLow[axis], spin.low[axis])
			angularHigh[axis].Add(angularHigh[axis], spin.high[axis])
		}
		energy.Add(energy, spin.energy)
		energyLow.Add(energyLow, spin.energyLow)
		energyHigh.Add(energyHigh, spin.energyHigh)
	}
	half := big.NewRat(1, 2)
	energy.Mul(energy, half)
	energyLow.Mul(energyLow, half)
	energyHigh.Mul(energyHigh, half)
	kinetic, ok := boundedReading(energy, energyLow, energyHigh,
		units.KilogramSquareMillimeterPerSecondSquared)
	if !ok {
		return ConservationState{}, false
	}
	linear, ok := boundedMomentum(momentum, low, high)
	if !ok {
		return ConservationState{}, false
	}
	angularReading, ok := boundedVector(angularValue, angularLow, angularHigh,
		units.KilogramSquareMillimeterPerSecond)
	if !ok {
		return ConservationState{}, false
	}
	return ConservationState{KineticEnergy: kinetic, LinearMomentum: linear,
		AngularMomentum: angularReading}, true
}

func oldSpinReadings(inertia decad.InertiaReading, pose r3.Transform, omega QuantityVec) (spinReading, bool) {
	reading := spinReading{energy: new(big.Rat), energyLow: new(big.Rat), energyHigh: new(big.Rat)}
	for axis := range reading.value {
		reading.value[axis], reading.low[axis], reading.high[axis] =
			new(big.Rat), new(big.Rat), new(big.Rat)
	}
	rotation, local, ok := oldSpinBasis(pose, omega)
	if !ok {
		return spinReading{}, false
	}
	for _, component := range inertiaComponents(inertia) {
		quantity, errorBound := oldExactBase(component.reading.Value), oldExactBase(component.reading.Bound)
		if quantity == nil || errorBound == nil || errorBound.Sign() < 0 {
			return spinReading{}, false
		}
		coefficient := new(big.Rat).Mul(local[component.i], local[component.j])
		if component.i != component.j {
			coefficient.Mul(coefficient, big.NewRat(2, 1))
		}
		oldAddIntervalProduct(reading.energy, reading.energyLow, reading.energyHigh,
			quantity, errorBound, coefficient)
		for axis := range reading.value {
			coefficient = new(big.Rat).Mul(rotation[axis][component.i], local[component.j])
			if component.i != component.j {
				coefficient.Add(coefficient,
					new(big.Rat).Mul(rotation[axis][component.j], local[component.i]))
			}
			oldAddIntervalProduct(reading.value[axis], reading.low[axis], reading.high[axis],
				quantity, errorBound, coefficient)
		}
	}
	return reading, true
}

func oldSpinEnergyChange(inertia decad.InertiaReading, pose r3.Transform,
	before, after QuantityVec) (*big.Rat, bool) {
	_, initial, ok := oldSpinBasis(pose, before)
	if !ok {
		return nil, false
	}
	_, final, ok := oldSpinBasis(pose, after)
	if !ok {
		return nil, false
	}
	upper := new(big.Rat)
	for _, component := range inertiaComponents(inertia) {
		quantity, bound := oldExactBase(component.reading.Value), oldExactBase(component.reading.Bound)
		if quantity == nil || bound == nil || bound.Sign() < 0 {
			return nil, false
		}
		coefficient := new(big.Rat).Sub(
			new(big.Rat).Mul(final[component.i], final[component.j]),
			new(big.Rat).Mul(initial[component.i], initial[component.j]))
		if component.i != component.j {
			coefficient.Mul(coefficient, big.NewRat(2, 1))
		}
		contribution := new(big.Rat).Mul(quantity, coefficient)
		upper.Add(upper, new(big.Rat).Add(contribution,
			new(big.Rat).Mul(bound, absRat(coefficient))))
	}
	return upper, true
}

func oldSpinBasis(pose r3.Transform, omega QuantityVec) ([3][3]*big.Rat, [3]*big.Rat, bool) {
	var rotation [3][3]*big.Rat
	var local [3]*big.Rat
	basis := pose.Basis()
	columns := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	for i, column := range columns {
		local[i] = new(big.Rat)
		for axis, coordinate := range [3]float64{column.X, column.Y, column.Z} {
			rotation[axis][i] = new(big.Rat).SetFloat64(coordinate)
			velocity := oldExactBase(velocityComponent(omega, axis))
			if rotation[axis][i] == nil || velocity == nil {
				return rotation, local, false
			}
			local[i].Add(local[i], new(big.Rat).Mul(rotation[axis][i], velocity))
		}
	}
	return rotation, local, true
}

func oldAddIntervalProduct(value, low, high, nominal, uncertainty, coefficient *big.Rat) {
	value.Add(value, new(big.Rat).Mul(nominal, coefficient))
	width := new(big.Rat).Mul(uncertainty, absRat(new(big.Rat).Set(coefficient)))
	contribution := new(big.Rat).Mul(nominal, coefficient)
	low.Add(low, new(big.Rat).Sub(contribution, width))
	high.Add(high, new(big.Rat).Add(contribution, width))
}

func oldWorldCenterReading(pose r3.Transform, source decad.VecMeasurement) ([3]*big.Rat, [3]*big.Rat, bool) {
	var nominal, errorBound [3]*big.Rat
	center := source.Value
	world := pose.Apply(center)
	basis := pose.Basis()
	translation := pose.Translation()
	radius := oldExactBase(source.Bound)
	if radius == nil || radius.Sign() < 0 {
		return nominal, errorBound, false
	}
	columns := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	local := [3]float64{center.X, center.Y, center.Z}
	for axis, output := range [3]float64{world.X, world.Y, world.Z} {
		nominal[axis] = new(big.Rat).SetFloat64(output)
		if nominal[axis] == nil {
			return nominal, errorBound, false
		}
		coordinate := [3]float64{translation.X, translation.Y, translation.Z}[axis]
		exact := new(big.Rat).SetFloat64(coordinate)
		if exact == nil {
			return nominal, errorBound, false
		}
		rowSum := new(big.Rat)
		for j, column := range columns {
			basisComponent := [3]float64{column.X, column.Y, column.Z}[axis]
			factor := new(big.Rat).SetFloat64(basisComponent)
			term := new(big.Rat).SetFloat64(local[j])
			if factor == nil || term == nil {
				return nominal, errorBound, false
			}
			exact.Add(exact, new(big.Rat).Mul(factor, term))
			rowSum.Add(rowSum, absRat(new(big.Rat).Set(factor)))
		}
		errorBound[axis] = new(big.Rat).Add(absRat(new(big.Rat).Sub(exact, nominal[axis])),
			new(big.Rat).Mul(radius, rowSum))
	}
	return nominal, errorBound, true
}

func oldAddMassProduct(sum, low, high, mass, massLow, massHigh, coefficient, coefficientLow,
	coefficientHigh *big.Rat) {
	sum.Add(sum, new(big.Rat).Mul(mass, coefficient))
	products := [4]*big.Rat{
		new(big.Rat).Mul(massLow, coefficientLow),
		new(big.Rat).Mul(massLow, coefficientHigh),
		new(big.Rat).Mul(massHigh, coefficientLow),
		new(big.Rat).Mul(massHigh, coefficientHigh),
	}
	minimum, maximum := products[0], products[0]
	for _, product := range products[1:] {
		if product.Cmp(minimum) < 0 {
			minimum = product
		}
		if product.Cmp(maximum) > 0 {
			maximum = product
		}
	}
	low.Add(low, minimum)
	high.Add(high, maximum)
}

func oldPairActive(w *World, pair islandPair, state State, drive map[int]driverMotion) (bool, bool) {
	a, okA := oldNewCertBody(w, pair.a, state.entries[pair.a], state.entries[pair.a], drive)
	b, okB := oldNewCertBody(w, pair.b, state.entries[pair.b], state.entries[pair.b], drive)
	if !okA || !okB {
		return false, false
	}
	bodies := []certBody{a, b}
	limit := oldExactBase(w.step.VelocityResidual)
	for _, point := range pair.manifold.Points {
		p, ok := newCertPoint(0, 0, 1, point, bodies, new(big.Rat))
		if !ok {
			return false, false
		}
		if oldPreNormalSpeed(p, bodies).Lo.Cmp(limit) <= 0 {
			return true, true
		}
	}
	return false, true
}

func oldGrazeSpeedWithin(w *World, pair islandPair, state State, drive map[int]driverMotion) bool {
	a, okA := oldNewCertBody(w, pair.a, state.entries[pair.a], state.entries[pair.a], drive)
	b, okB := oldNewCertBody(w, pair.b, state.entries[pair.b], state.entries[pair.b], drive)
	if !okA || !okB {
		return false
	}
	bodies := []certBody{a, b}
	limit := oldExactBase(w.step.VelocityResidual)
	for _, point := range pair.manifold.Points {
		p, ok := newCertPoint(0, 0, 1, point, bodies, new(big.Rat))
		if !ok || magnitude(oldPreNormalSpeed(p, bodies)).Cmp(limit) > 0 {
			return false
		}
	}
	return true
}
