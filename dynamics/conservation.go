package dynamics

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
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
		mass, bound := exactBase(part.mass.Mass.Value), exactBase(part.mass.Mass.Bound)
		if mass == nil || bound == nil {
			return ConservationState{}, false
		}
		massLow := new(big.Rat).Sub(mass, bound)
		massHigh := new(big.Rat).Add(mass, bound)
		if massLow.Sign() <= 0 {
			return ConservationState{}, false
		}
		center, centerError, ok := worldCenterReading(state.entries[i].Pose, part.mass.Center)
		if !ok {
			return ConservationState{}, false
		}
		speedSquared := new(big.Rat)
		var velocity [3]*big.Rat
		for axis := range momentum {
			v := exactBase(velocityComponent(state.entries[i].LinearVelocity, axis))
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
			addMassProduct(angularValue[axis], angularLow[axis], angularHigh[axis],
				mass, massLow, massHigh, coefficient,
				new(big.Rat).Sub(coefficient, uncertainty), new(big.Rat).Add(coefficient, uncertainty))
		}
		energy.Add(energy, new(big.Rat).Mul(mass, speedSquared))
		energyLow.Add(energyLow, new(big.Rat).Mul(massLow, speedSquared))
		energyHigh.Add(energyHigh, new(big.Rat).Mul(massHigh, speedSquared))
		spin, ok := spinReadings(part.mass.Inertia, state.entries[i].Pose,
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

type spinReading struct {
	value, low, high              [3]*big.Rat
	energy, energyLow, energyHigh *big.Rat
}

// spinReadings evaluates I in the source axes. The held pose basis and angular
// velocity are exact rational inputs here; only the six inertia readings carry
// intervals. Rotating the result back to world axes preserves their signs.
func spinReadings(inertia decad.InertiaReading, pose r3.Transform, omega QuantityVec) (spinReading, bool) {
	reading := spinReading{energy: new(big.Rat), energyLow: new(big.Rat), energyHigh: new(big.Rat)}
	for axis := range reading.value {
		reading.value[axis], reading.low[axis], reading.high[axis] =
			new(big.Rat), new(big.Rat), new(big.Rat)
	}
	rotation, local, ok := spinBasis(pose, omega)
	if !ok {
		return spinReading{}, false
	}
	for _, component := range inertiaComponents(inertia) {
		quantity, errorBound := exactBase(component.reading.Value), exactBase(component.reading.Bound)
		if quantity == nil || errorBound == nil || errorBound.Sign() < 0 {
			return spinReading{}, false
		}
		coefficient := new(big.Rat).Mul(local[component.i], local[component.j])
		if component.i != component.j {
			coefficient.Mul(coefficient, big.NewRat(2, 1))
		}
		addIntervalProduct(reading.energy, reading.energyLow, reading.energyHigh,
			quantity, errorBound, coefficient)
		for axis := range reading.value {
			coefficient = new(big.Rat).Mul(rotation[axis][component.i], local[component.j])
			if component.i != component.j {
				coefficient.Add(coefficient,
					new(big.Rat).Mul(rotation[axis][component.j], local[component.i]))
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

// spinEnergyChange keeps each source inertia interval shared across the
// before/after squared-speed difference of one impulse. It returns the upper
// end of ωᵀIω's change, twice the rotational energy change.
func spinEnergyChange(inertia decad.InertiaReading, pose r3.Transform,
	before, after QuantityVec) (*big.Rat, bool) {
	_, initial, ok := spinBasis(pose, before)
	if !ok {
		return nil, false
	}
	_, final, ok := spinBasis(pose, after)
	if !ok {
		return nil, false
	}
	upper := new(big.Rat)
	for _, component := range inertiaComponents(inertia) {
		quantity, bound := exactBase(component.reading.Value), exactBase(component.reading.Bound)
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

func spinBasis(pose r3.Transform, omega QuantityVec) ([3][3]*big.Rat, [3]*big.Rat, bool) {
	var rotation [3][3]*big.Rat
	var local [3]*big.Rat
	basis := pose.Basis()
	columns := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	for i, column := range columns {
		local[i] = new(big.Rat)
		for axis, coordinate := range [3]float64{column.X, column.Y, column.Z} {
			rotation[axis][i] = new(big.Rat).SetFloat64(coordinate)
			velocity := exactBase(velocityComponent(omega, axis))
			if rotation[axis][i] == nil || velocity == nil {
				return rotation, local, false
			}
			local[i].Add(local[i], new(big.Rat).Mul(rotation[axis][i], velocity))
		}
	}
	return rotation, local, true
}

func addIntervalProduct(value, low, high, nominal, uncertainty, coefficient *big.Rat) {
	value.Add(value, new(big.Rat).Mul(nominal, coefficient))
	width := new(big.Rat).Mul(uncertainty, absRat(new(big.Rat).Set(coefficient)))
	contribution := new(big.Rat).Mul(nominal, coefficient)
	low.Add(low, new(big.Rat).Sub(contribution, width))
	high.Add(high, new(big.Rat).Add(contribution, width))
}

// worldCenterReading uses r3 for the point transform, then encloses both its
// floating-point evaluation and the source mass center's ball uncertainty.
func worldCenterReading(pose r3.Transform, source decad.VecMeasurement) ([3]*big.Rat, [3]*big.Rat, bool) {
	var nominal, errorBound [3]*big.Rat
	center := source.Value
	world := pose.Apply(center)
	basis := pose.Basis()
	translation := pose.Translation()
	radius := exactBase(source.Bound)
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

func addMassProduct(sum, low, high, mass, massLow, massHigh, coefficient, coefficientLow,
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

func (w *World) driftConservationSlices(slices [][2]State) (ConservationState, bool) {
	for _, slice := range slices {
		for i, part := range w.bodies {
			if part.definition.Role == Dynamic && !sameOrientation(
				slice[0].entries[i].Pose, slice[1].entries[i].Pose) {
				return w.rotatingDriftChange(slices)
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
		mass, bound := exactBase(part.mass.Mass.Value), exactBase(part.mass.Mass.Bound)
		if mass == nil || bound == nil {
			return ConservationState{}, false
		}
		massLow, massHigh := new(big.Rat).Sub(mass, bound), new(big.Rat).Add(mass, bound)
		if massLow.Sign() <= 0 {
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
				startValue, endValue := new(big.Rat).SetFloat64(start[axis]),
					new(big.Rat).SetFloat64(end[axis])
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
func (w *World) rotatingDriftChange(slices [][2]State) (ConservationState, bool) {
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
		before, ok := w.conservationState(slice[0])
		if !ok {
			return ConservationState{}, false
		}
		after, ok := w.conservationState(slice[1])
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
		mass, bound := exactBase(part.mass.Mass.Value), exactBase(part.mass.Mass.Bound)
		if mass == nil || bound == nil {
			return MomentumReading{}, MomentumReading{}, false
		}
		massLow := new(big.Rat).Sub(mass, bound)
		massHigh := new(big.Rat).Add(mass, bound)
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
	actual := new(big.Rat).SetFloat64(value)
	deviation := intervalDeviation(actual, low, high)
	bound, _ := deviation.Float64()
	if !finite(bound) || bound < 0 {
		return decad.Measurement{}, false
	}
	if new(big.Rat).SetFloat64(bound).Cmp(deviation) < 0 {
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
