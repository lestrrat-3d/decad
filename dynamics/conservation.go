package dynamics

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// MomentumReading encloses each component of a linear impulse or momentum,
// or angular momentum when used by ConservationState.AngularMomentum.
// Contact impulse bounds refer to the published numerical event impulses.
type MomentumReading struct {
	Value QuantityVec
	Bound QuantityVec
}

// ConservationState sums readings over the world's dynamic bodies only.
type ConservationState struct {
	// KineticEnergy has units.Torque's dimensions, kg*mm^2/s^2.
	KineticEnergy   decad.Measurement
	LinearMomentum  MomentumReading
	AngularMomentum MomentumReading
}

// StepConservation describes the discrete input, kick, and completed step.
// Fixed and kinematic bodies contribute through contact impulse and driver
// work. No reading claims the physical impulse of unresolved contact.
type StepConservation struct {
	Input, AfterKick, Completion                ConservationState
	GravityImpulse, LoadImpulse, ContactImpulse MomentumReading
	// TorqueImpulse is the full-step world torque impulse on dynamic bodies.
	TorqueImpulse MomentumReading
	// KinematicWork is the signed work delivered by prescribed drivers at contact events.
	KinematicWork decad.Measurement
	// DriftChange excludes the kick, every contact impulse, and position correction.
	DriftChange ConservationState
}

func (w *World) torqueImpulse(loads []*BodyLoad, dt units.Value) (MomentumReading, bool) {
	var value [3]*big.Rat
	for axis := range value {
		value[axis] = new(big.Rat)
	}
	duration := exactBase(dt)
	if duration == nil || duration.Sign() <= 0 {
		return MomentumReading{}, false
	}
	for i, part := range w.bodies {
		if part.definition.Role != Dynamic || loads[i] == nil {
			continue
		}
		for axis := range value {
			torque := exactBase(velocityComponent(loads[i].Torque, axis))
			if torque == nil {
				return MomentumReading{}, false
			}
			value[axis].Add(value[axis], new(big.Rat).Mul(torque, duration))
		}
	}
	return boundedVector(value, value, value, units.KilogramSquareMillimeterPerSecond)
}

func (w *World) conservationState(state State) (ConservationState, bool) {
	sums := newConservationSums()
	for i, part := range w.bodies {
		if part.definition.Role != Dynamic {
			continue
		}
		exact := part.exact
		if exact.low == nil || exact.low.Sign() <= 0 {
			return ConservationState{}, false
		}
		velocity, ok := quantityRats(state.entries[i].LinearVelocity)
		if !ok {
			return ConservationState{}, false
		}
		if zeroRats(velocity) {
			// A body at rest adds nothing to the linear readings, and its
			// mass center enters them only through c×v; the center must
			// still read.
			if !centerReadable(state.entries[i].Pose, exact) {
				return ConservationState{}, false
			}
		} else if !sums.addLinear(state.entries[i].Pose, exact, velocity) {
			return ConservationState{}, false
		}
		spin, ok := spinReadings(&exact.components, state.entries[i].Pose,
			state.entries[i].AngularVelocity)
		if !ok {
			return ConservationState{}, false
		}
		for axis := range sums.angular {
			proof.AddRat(sums.angular[axis], sums.angular[axis], spin.value[axis])
			proof.AddRat(sums.angularLow[axis], sums.angularLow[axis], spin.low[axis])
			proof.AddRat(sums.angularHigh[axis], sums.angularHigh[axis], spin.high[axis])
		}
		proof.AddRat(sums.energy, sums.energy, spin.energy)
		proof.AddRat(sums.energyLow, sums.energyLow, spin.energyLow)
		proof.AddRat(sums.energyHigh, sums.energyHigh, spin.energyHigh)
	}
	return sums.readings()
}

// conservationSums accumulates conservationState's exact readings: twice
// the kinetic energy, the linear momentum and the angular momentum about the
// world origin, each with its enclosure.
type conservationSums struct {
	energy, energyLow, energyHigh    *big.Rat
	momentum, low, high              [3]*big.Rat
	angular, angularLow, angularHigh [3]*big.Rat
}

func newConservationSums() *conservationSums {
	sums := &conservationSums{energy: new(big.Rat), energyLow: new(big.Rat), energyHigh: new(big.Rat)}
	for axis := range sums.momentum {
		sums.momentum[axis], sums.low[axis], sums.high[axis] = new(big.Rat), new(big.Rat), new(big.Rat)
		sums.angular[axis], sums.angularLow[axis], sums.angularHigh[axis] = new(big.Rat), new(big.Rat), new(big.Rat)
	}
	return sums
}

// addLinear adds one moving body's linear momentum, its moment c×m·v about
// the world origin and its m·v², each over its mass interval, the moment
// over its mass center reading too.
func (sums *conservationSums) addLinear(pose r3.Transform, exact *exactMass, velocity [3]*big.Rat) bool {
	mass, massLow, massHigh := exact.mass, exact.low, exact.high
	center, centerError, ok := worldCenterReading(pose, exact)
	if !ok {
		return false
	}
	speedSquared := new(big.Rat)
	for axis, v := range velocity {
		proof.AddRat(sums.momentum[axis], sums.momentum[axis], proof.MulRat(new(big.Rat), mass, v))
		if v.Sign() < 0 {
			proof.AddRat(sums.low[axis], sums.low[axis], proof.MulRat(new(big.Rat), massHigh, v))
			proof.AddRat(sums.high[axis], sums.high[axis], proof.MulRat(new(big.Rat), massLow, v))
		} else {
			proof.AddRat(sums.low[axis], sums.low[axis], proof.MulRat(new(big.Rat), massLow, v))
			proof.AddRat(sums.high[axis], sums.high[axis], proof.MulRat(new(big.Rat), massHigh, v))
		}
		proof.AddRat(speedSquared, speedSquared, proof.MulRat(new(big.Rat), v, v))
	}
	for axis := range sums.angular {
		j, k := (axis+1)%3, (axis+2)%3
		coefficient := proof.SubRat(new(big.Rat), proof.MulRat(new(big.Rat), center[j], velocity[k]),
			proof.MulRat(new(big.Rat), center[k], velocity[j]))
		uncertainty := proof.AddRat(new(big.Rat),
			proof.MulRat(new(big.Rat), centerError[j], absRat(new(big.Rat).Set(velocity[k]))),
			proof.MulRat(new(big.Rat), centerError[k], absRat(new(big.Rat).Set(velocity[j]))))
		addMassProduct(sums.angular[axis], sums.angularLow[axis], sums.angularHigh[axis],
			mass, massLow, massHigh, coefficient,
			proof.SubRat(new(big.Rat), coefficient, uncertainty), proof.AddRat(new(big.Rat), coefficient, uncertainty))
	}
	proof.AddRat(sums.energy, sums.energy, proof.MulRat(new(big.Rat), mass, speedSquared))
	proof.AddRat(sums.energyLow, sums.energyLow, proof.MulRat(new(big.Rat), massLow, speedSquared))
	proof.AddRat(sums.energyHigh, sums.energyHigh, proof.MulRat(new(big.Rat), massHigh, speedSquared))
	return true
}

// readings halves the energy sums and publishes every reading.
func (sums *conservationSums) readings() (ConservationState, bool) {
	half := big.NewRat(1, 2)
	sums.energy.Mul(sums.energy, half)
	sums.energyLow.Mul(sums.energyLow, half)
	sums.energyHigh.Mul(sums.energyHigh, half)
	kinetic, ok := boundedReading(sums.energy, sums.energyLow, sums.energyHigh,
		units.KilogramSquareMillimeterPerSecondSquared)
	if !ok {
		return ConservationState{}, false
	}
	linear, ok := boundedMomentum(sums.momentum, sums.low, sums.high)
	if !ok {
		return ConservationState{}, false
	}
	angularReading, ok := boundedVector(sums.angular, sums.angularLow, sums.angularHigh,
		units.KilogramSquareMillimeterPerSecond)
	if !ok {
		return ConservationState{}, false
	}
	return ConservationState{KineticEnergy: kinetic, LinearMomentum: linear,
		AngularMomentum: angularReading}, true
}

// zeroRats reports whether every component is zero.
func zeroRats(x [3]*big.Rat) bool {
	return x[0].Sign() == 0 && x[1].Sign() == 0 && x[2].Sign() == 0
}

type spinReading struct {
	value, low, high              [3]*big.Rat
	energy, energyLow, energyHigh *big.Rat
}

// spinReadings evaluates I in the source axes. The held pose basis and angular
// velocity are exact rational inputs here; only the six inertia readings carry
// intervals. Rotating the result back to world axes preserves their signs.
func spinReadings(components *[6]exactComponent, pose r3.Transform, omega QuantityVec) (spinReading, bool) {
	reading := spinReading{energy: new(big.Rat), energyLow: new(big.Rat), energyHigh: new(big.Rat)}
	for axis := range reading.value {
		reading.value[axis], reading.low[axis], reading.high[axis] =
			new(big.Rat), new(big.Rat), new(big.Rat)
	}
	velocity, ok := quantityRats(omega)
	if !ok {
		return spinReading{}, false
	}
	if zeroRats(velocity) {
		// Every coefficient is zero, so every reading is: the basis and the
		// components must still read.
		basis := pose.Basis()
		if !finite(basis.EX.X, basis.EX.Y, basis.EX.Z, basis.EY.X, basis.EY.Y, basis.EY.Z,
			basis.EZ.X, basis.EZ.Y, basis.EZ.Z) {
			return spinReading{}, false
		}
		for _, component := range components {
			if component.value == nil || component.bound == nil || component.bound.Sign() < 0 {
				return spinReading{}, false
			}
		}
		return reading, true
	}
	rotation, local, ok := basisSpin(pose, velocity)
	if !ok {
		return spinReading{}, false
	}
	for _, component := range components {
		quantity, errorBound := component.value, component.bound
		if quantity == nil || errorBound == nil || errorBound.Sign() < 0 {
			return spinReading{}, false
		}
		coefficient := proof.MulRat(new(big.Rat), local[component.i], local[component.j])
		if component.i != component.j {
			proof.MulRat(coefficient, coefficient, big.NewRat(2, 1))
		}
		addIntervalProduct(reading.energy, reading.energyLow, reading.energyHigh,
			quantity, errorBound, coefficient)
		for axis := range reading.value {
			coefficient = proof.MulRat(new(big.Rat), rotation[axis][component.i], local[component.j])
			if component.i != component.j {
				proof.AddRat(coefficient, coefficient,
					proof.MulRat(new(big.Rat), rotation[axis][component.j], local[component.i]))
			}
			addIntervalProduct(reading.value[axis], reading.low[axis], reading.high[axis],
				quantity, errorBound, coefficient)
		}
	}
	return reading, true
}

type inertiaComponent struct {
	reading decad.Measurement
	i, j    int
}

func inertiaComponents(inertia decad.InertiaReading) [6]inertiaComponent {
	return [6]inertiaComponent{{inertia.XX, 0, 0}, {inertia.YY, 1, 1}, {inertia.ZZ, 2, 2},
		{inertia.XY, 0, 1}, {inertia.XZ, 0, 2}, {inertia.YZ, 1, 2}}
}

// spinBasis reads a pose basis as exact rationals, rotation[row][column],
// and the angular velocity in its axes, Rᵀω.
func spinBasis(pose r3.Transform, omega QuantityVec) ([3][3]*big.Rat, [3]*big.Rat, bool) {
	velocity, ok := quantityRats(omega)
	if !ok {
		return [3][3]*big.Rat{}, [3]*big.Rat{}, false
	}
	return basisSpin(pose, velocity)
}

// basisSpin is spinBasis for an exact angular velocity.
func basisSpin(pose r3.Transform, velocity [3]*big.Rat) ([3][3]*big.Rat, [3]*big.Rat, bool) {
	var rotation [3][3]*big.Rat
	var local [3]*big.Rat
	basis := pose.Basis()
	columns := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	for i, column := range columns {
		local[i] = new(big.Rat)
		for axis, coordinate := range [3]float64{column.X, column.Y, column.Z} {
			rotation[axis][i] = ratFloat(coordinate)
			if rotation[axis][i] == nil {
				return rotation, local, false
			}
			if velocity[axis].Sign() == 0 {
				continue
			}
			proof.AddRat(local[i], local[i], proof.MulRat(new(big.Rat), rotation[axis][i], velocity[axis]))
		}
	}
	return rotation, local, true
}

// addIntervalProduct adds nominal·coefficient to value, and its enclosure
// over nominal ± uncertainty to low and high. A zero coefficient, or a zero
// nominal with no uncertainty, adds nothing.
func addIntervalProduct(value, low, high, nominal, uncertainty, coefficient *big.Rat) {
	if coefficient.Sign() == 0 || (nominal.Sign() == 0 && uncertainty.Sign() == 0) {
		return
	}
	contribution := proof.MulRat(new(big.Rat), nominal, coefficient)
	proof.AddRat(value, value, contribution)
	if uncertainty.Sign() == 0 {
		proof.AddRat(low, low, contribution)
		proof.AddRat(high, high, contribution)
		return
	}
	width := proof.MulRat(new(big.Rat), uncertainty, absRat(new(big.Rat).Set(coefficient)))
	proof.AddRat(low, low, proof.SubRat(new(big.Rat), contribution, width))
	proof.AddRat(high, high, proof.AddRat(contribution, contribution, width))
}

// worldCenterReading uses r3 for the point transform, then encloses both its
// floating-point evaluation and the source mass center's ball uncertainty.
func worldCenterReading(pose r3.Transform, exact *exactMass) ([3]*big.Rat, [3]*big.Rat, bool) {
	var nominal, errorBound [3]*big.Rat
	world := pose.Apply(exact.center)
	basis := pose.Basis()
	translation := pose.Translation()
	radius := exact.radius
	if radius == nil || radius.Sign() < 0 {
		return nominal, errorBound, false
	}
	for _, term := range exact.local {
		if term == nil {
			return nominal, errorBound, false
		}
	}
	columns := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	for axis, output := range [3]float64{world.X, world.Y, world.Z} {
		nominal[axis] = ratFloat(output)
		if nominal[axis] == nil {
			return nominal, errorBound, false
		}
		coordinate := [3]float64{translation.X, translation.Y, translation.Z}[axis]
		exactValue := ratFloat(coordinate)
		if exactValue == nil {
			return nominal, errorBound, false
		}
		rowSum := new(big.Rat)
		for j, column := range columns {
			basisComponent := [3]float64{column.X, column.Y, column.Z}[axis]
			factor := ratFloat(basisComponent)
			if factor == nil {
				return nominal, errorBound, false
			}
			if factor.Sign() == 0 {
				continue
			}
			proof.AddRat(exactValue, exactValue, proof.MulRat(new(big.Rat), factor, exact.local[j]))
			proof.AddRat(rowSum, rowSum, absRat(factor))
		}
		errorBound[axis] = absRat(proof.SubRat(exactValue, exactValue, nominal[axis]))
		if radius.Sign() != 0 {
			proof.AddRat(errorBound[axis], errorBound[axis], proof.MulRat(rowSum, radius, rowSum))
		}
	}
	return nominal, errorBound, true
}

// centerReadable reports whether worldCenterReading reads a pose's mass
// center, without its arithmetic: it fails exactly when the center's bound
// does not convert or is negative, a source coordinate is not finite, or the
// transformed center, the translation or the basis is not finite.
func centerReadable(pose r3.Transform, exact *exactMass) bool {
	if exact.radius == nil || exact.radius.Sign() < 0 {
		return false
	}
	for _, term := range exact.local {
		if term == nil {
			return false
		}
	}
	world, translation, basis := pose.Apply(exact.center), pose.Translation(), pose.Basis()
	return finite(world.X, world.Y, world.Z, translation.X, translation.Y, translation.Z,
		basis.EX.X, basis.EX.Y, basis.EX.Z, basis.EY.X, basis.EY.Y, basis.EY.Z, basis.EZ.X, basis.EZ.Y, basis.EZ.Z)
}

// addMassProduct adds mass·coefficient to sum, and the smallest and largest
// corner products of the mass and coefficient intervals to low and high. An
// ordered positive mass interval against an ordered coefficient interval
// picks its two extreme corners by the coefficient's endpoint signs; every
// other input compares all four.
func addMassProduct(sum, low, high, mass, massLow, massHigh, coefficient, coefficientLow,
	coefficientHigh *big.Rat) {
	proof.AddRat(sum, sum, proof.MulRat(new(big.Rat), mass, coefficient))
	if massLow.Sign() > 0 && massLow.Cmp(massHigh) <= 0 && coefficientLow.Cmp(coefficientHigh) <= 0 {
		if coefficientLow.Sign() < 0 {
			proof.AddRat(low, low, proof.MulRat(new(big.Rat), massHigh, coefficientLow))
		} else {
			proof.AddRat(low, low, proof.MulRat(new(big.Rat), massLow, coefficientLow))
		}
		if coefficientHigh.Sign() > 0 {
			proof.AddRat(high, high, proof.MulRat(new(big.Rat), massHigh, coefficientHigh))
		} else {
			proof.AddRat(high, high, proof.MulRat(new(big.Rat), massLow, coefficientHigh))
		}
		return
	}
	products := [4]*big.Rat{
		proof.MulRat(new(big.Rat), massLow, coefficientLow),
		proof.MulRat(new(big.Rat), massLow, coefficientHigh),
		proof.MulRat(new(big.Rat), massHigh, coefficientLow),
		proof.MulRat(new(big.Rat), massHigh, coefficientHigh),
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
	proof.AddRat(low, low, minimum)
	proof.AddRat(high, high, maximum)
}

// driftConservationSlices reads every conservation reading it needs through
// work, which reuses the step's own readings.
func (w *World) driftConservationSlices(work *stepWork, slices [][2]State) (ConservationState, bool) {
	for _, slice := range slices {
		for i, part := range w.bodies {
			if part.definition.Role == Dynamic && !sameOrientation(
				slice[0].entries[i].Pose, slice[1].entries[i].Pose) {
				return w.rotatingDriftChange(work, slices)
			}
		}
	}
	var angular, low, high [3]*big.Rat
	for axis := range angular {
		angular[axis], low[axis], high[axis] = new(big.Rat), new(big.Rat), new(big.Rat)
	}
	for i, part := range w.bodies {
		if part.definition.Role != Dynamic {
			continue
		}
		mass, massLow, massHigh := part.exact.mass, part.exact.low, part.exact.high
		if massLow == nil || massLow.Sign() <= 0 {
			return ConservationState{}, false
		}
		var coefficient [3]*big.Rat
		for axis := range coefficient {
			coefficient[axis] = new(big.Rat)
		}
		for _, slice := range slices {
			before, after := slice[0].entries[i], slice[1].entries[i]
			if before.Body != after.Body || before.LinearVelocity != after.LinearVelocity ||
				before.AngularVelocity != after.AngularVelocity || !sameOrientation(before.Pose, after.Pose) {
				return ConservationState{}, false
			}
			from, to := before.Pose.Translation(), after.Pose.Translation()
			start := [3]float64{from.X, from.Y, from.Z}
			end := [3]float64{to.X, to.Y, to.Z}
			var displacement, velocity [3]*big.Rat
			for axis := range displacement {
				startValue, endValue := ratFloat(start[axis]),
					ratFloat(end[axis])
				velocity[axis] = exactBase(velocityComponent(before.LinearVelocity, axis))
				if startValue == nil || endValue == nil || velocity[axis] == nil {
					return ConservationState{}, false
				}
				displacement[axis] = new(big.Rat).Sub(endValue, startValue)
			}
			for axis := range coefficient {
				j, k := (axis+1)%3, (axis+2)%3
				coefficient[axis].Add(coefficient[axis], new(big.Rat).Sub(
					new(big.Rat).Mul(displacement[j], velocity[k]),
					new(big.Rat).Mul(displacement[k], velocity[j])))
			}
		}
		for axis := range angular {
			addMassProduct(angular[axis], low[axis], high[axis], mass, massLow, massHigh,
				coefficient[axis], coefficient[axis], coefficient[axis])
		}
	}
	zero, ok := boundedReading(new(big.Rat), new(big.Rat), new(big.Rat),
		units.KilogramSquareMillimeterPerSecondSquared)
	if !ok {
		return ConservationState{}, false
	}
	var zeros [3]*big.Rat
	for axis := range zeros {
		zeros[axis] = new(big.Rat)
	}
	linear, ok := boundedMomentum(zeros, zeros, zeros)
	if !ok {
		return ConservationState{}, false
	}
	angularReading, ok := boundedVector(angular, low, high, units.KilogramSquareMillimeterPerSecond)
	if !ok {
		return ConservationState{}, false
	}
	return ConservationState{KineticEnergy: zero, LinearMomentum: linear,
		AngularMomentum: angularReading}, true
}

// Rotating drift compares the independently enclosed endpoint readings.
// This can be wider than the translation-only cancellation above but includes
// world-frame inertia changes without attributing an event impulse to drift.
func (w *World) rotatingDriftChange(work *stepWork, slices [][2]State) (ConservationState, bool) {
	var energyValue, energyLow, energyHigh big.Rat
	var linearValue, linearLow, linearHigh, angularValue, angularLow, angularHigh [3]*big.Rat
	for axis := range linearValue {
		linearValue[axis], linearLow[axis], linearHigh[axis] = new(big.Rat), new(big.Rat), new(big.Rat)
		angularValue[axis], angularLow[axis], angularHigh[axis] = new(big.Rat), new(big.Rat), new(big.Rat)
	}
	for _, slice := range slices {
		for i, part := range w.bodies {
			if part.definition.Role != Dynamic {
				continue
			}
			before, after := slice[0].entries[i], slice[1].entries[i]
			if before.Body != after.Body || before.LinearVelocity != after.LinearVelocity ||
				before.AngularVelocity != after.AngularVelocity {
				return ConservationState{}, false
			}
		}
		before, ok := work.conservation(slice[0])
		if !ok {
			return ConservationState{}, false
		}
		after, ok := work.conservation(slice[1])
		if !ok {
			return ConservationState{}, false
		}
		addReadingDifference(&energyValue, &energyLow, &energyHigh,
			before.KineticEnergy, after.KineticEnergy)
		for axis := range linearValue {
			addReadingDifference(linearValue[axis], linearLow[axis], linearHigh[axis],
				componentReading(before.LinearMomentum, axis), componentReading(after.LinearMomentum, axis))
			addReadingDifference(angularValue[axis], angularLow[axis], angularHigh[axis],
				componentReading(before.AngularMomentum, axis), componentReading(after.AngularMomentum, axis))
		}
	}
	energy, ok := boundedReading(&energyValue, &energyLow, &energyHigh,
		units.KilogramSquareMillimeterPerSecondSquared)
	if !ok {
		return ConservationState{}, false
	}
	linear, ok := boundedMomentum(linearValue, linearLow, linearHigh)
	if !ok {
		return ConservationState{}, false
	}
	angular, ok := boundedVector(angularValue, angularLow, angularHigh,
		units.KilogramSquareMillimeterPerSecond)
	if !ok {
		return ConservationState{}, false
	}
	return ConservationState{KineticEnergy: energy, LinearMomentum: linear,
		AngularMomentum: angular}, true
}

func componentReading(vector MomentumReading, axis int) decad.Measurement {
	return decad.Measurement{Value: velocityComponent(vector.Value, axis),
		Bound: velocityComponent(vector.Bound, axis)}
}

func addReadingDifference(value, low, high *big.Rat, before, after decad.Measurement) {
	beforeValue, beforeBound := exactBase(before.Value), exactBase(before.Bound)
	afterValue, afterBound := exactBase(after.Value), exactBase(after.Bound)
	value.Add(value, new(big.Rat).Sub(afterValue, beforeValue))
	low.Add(low, new(big.Rat).Sub(new(big.Rat).Sub(afterValue, afterBound),
		new(big.Rat).Add(beforeValue, beforeBound)))
	high.Add(high, new(big.Rat).Sub(new(big.Rat).Add(afterValue, afterBound),
		new(big.Rat).Sub(beforeValue, beforeBound)))
}

func (w *World) forceImpulses(gravity QuantityVec, loads []*BodyLoad,
	dt units.Value) (MomentumReading, MomentumReading, bool) {
	var gValue, gLow, gHigh, fValue [3]*big.Rat
	for axis := range gValue {
		gValue[axis], gLow[axis], gHigh[axis], fValue[axis] =
			new(big.Rat), new(big.Rat), new(big.Rat), new(big.Rat)
	}
	duration := exactBase(dt)
	if duration == nil || duration.Sign() <= 0 {
		return MomentumReading{}, MomentumReading{}, false
	}
	for i, part := range w.bodies {
		if part.definition.Role != Dynamic {
			continue
		}
		mass, massLow, massHigh := part.exact.mass, part.exact.low, part.exact.high
		if massLow == nil {
			return MomentumReading{}, MomentumReading{}, false
		}
		for axis := range gValue {
			acceleration := exactBase(velocityComponent(gravity, axis))
			if acceleration == nil {
				return MomentumReading{}, MomentumReading{}, false
			}
			gdt := new(big.Rat).Mul(acceleration, duration)
			gValue[axis].Add(gValue[axis], new(big.Rat).Mul(mass, gdt))
			if gdt.Sign() < 0 {
				gLow[axis].Add(gLow[axis], new(big.Rat).Mul(massHigh, gdt))
				gHigh[axis].Add(gHigh[axis], new(big.Rat).Mul(massLow, gdt))
			} else {
				gLow[axis].Add(gLow[axis], new(big.Rat).Mul(massLow, gdt))
				gHigh[axis].Add(gHigh[axis], new(big.Rat).Mul(massHigh, gdt))
			}
			if loads[i] != nil {
				force := exactBase(velocityComponent(loads[i].Force, axis))
				if force == nil {
					return MomentumReading{}, MomentumReading{}, false
				}
				fValue[axis].Add(fValue[axis], new(big.Rat).Mul(force, duration))
			}
		}
	}
	gravityReading, ok := boundedMomentum(gValue, gLow, gHigh)
	if !ok {
		return MomentumReading{}, MomentumReading{}, false
	}
	loadReading, ok := boundedMomentum(fValue, fValue, fValue)
	return gravityReading, loadReading, ok
}

func boundedMomentum(value, low, high [3]*big.Rat) (MomentumReading, bool) {
	return boundedVector(value, low, high, units.KilogramMillimeterPerSecond)
}

func boundedVector(value, low, high [3]*big.Rat, unit units.Unit) (MomentumReading, bool) {
	reading := MomentumReading{}
	for axis := range value {
		component, ok := boundedReading(value[axis], low[axis], high[axis], unit)
		if !ok {
			return MomentumReading{}, false
		}
		setVelocityComponent(&reading.Value, axis, component.Value)
		setVelocityComponent(&reading.Bound, axis, component.Bound)
	}
	return reading, true
}

func boundedReading(nominal, low, high *big.Rat, unit units.Unit) (decad.Measurement, bool) {
	if nominal == nil || low == nil || high == nil || low.Cmp(nominal) > 0 || high.Cmp(nominal) < 0 {
		return decad.Measurement{}, false
	}
	value, _ := nominal.Float64()
	if !finite(value) {
		return decad.Measurement{}, false
	}
	actual := ratFloat(value)
	deviation := intervalDeviation(actual, low, high)
	bound, _ := deviation.Float64()
	if !finite(bound) || bound < 0 {
		return decad.Measurement{}, false
	}
	if ratFloat(bound).Cmp(deviation) < 0 {
		bound = math.Nextafter(bound, math.Inf(1))
	}
	if !finite(bound) {
		return decad.Measurement{}, false
	}
	exactness := decad.Exact
	if bound != 0 {
		exactness = decad.Approximate
	}
	return decad.Measurement{Value: units.New(value, unit), Bound: units.New(bound, unit),
		Exactness: exactness}, true
}

// inertiaRowCeiling is the largest row sum of the inertia tensor's
// component magnitudes plus their bounds: a ceiling on its largest
// eigenvalue.
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
