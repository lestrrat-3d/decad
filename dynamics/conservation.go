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
	// KinematicWork is the signed work delivered by prescribed drivers at contact events.
	KinematicWork decad.Measurement
	// DriftChange excludes the kick, every contact impulse, and position correction.
	DriftChange ConservationState
}

func (w *World) conservationReadings(from, kicked, end State, trace Trace, events []ContactEvent,
	gravity QuantityVec, loads [2]*BodyLoad, dt units.Value) (StepConservation, bool) {
	input, ok := w.conservationState(from)
	if !ok {
		return StepConservation{}, false
	}
	afterKick, ok := w.conservationState(kicked)
	if !ok {
		return StepConservation{}, false
	}
	completion, ok := w.conservationState(end)
	if !ok {
		return StepConservation{}, false
	}
	gravityImpulse, loadImpulse, ok := w.forceImpulses(gravity, loads, dt)
	if !ok {
		return StepConservation{}, false
	}
	contactImpulse, ok := w.externalContactImpulse(events)
	if !ok {
		return StepConservation{}, false
	}
	kinematicWork, ok := w.kinematicWork(events)
	if !ok {
		return StepConservation{}, false
	}
	driftChange, ok := w.driftConservationChange(kicked, trace)
	if !ok {
		return StepConservation{}, false
	}
	return StepConservation{Input: input, AfterKick: afterKick, Completion: completion,
		GravityImpulse: gravityImpulse, LoadImpulse: loadImpulse, ContactImpulse: contactImpulse,
		KinematicWork: kinematicWork, DriftChange: driftChange}, true
}

func (w *World) kinematicWork(events []ContactEvent) (decad.Measurement, bool) {
	total := new(big.Rat)
	impulseLimit := exactBase(w.step.ImpulseResidual)
	if impulseLimit == nil {
		return decad.Measurement{}, false
	}
	for _, event := range events {
		applied, ok := eventAppliedImpulse(event)
		if !ok {
			return decad.Measurement{}, false
		}
		work, _, ok := w.kinematicEventWork(event, applied, impulseLimit)
		if !ok {
			return decad.Measurement{}, false
		}
		total.Add(total, work)
	}
	return boundedReading(total, total, total, units.KilogramSquareMillimeterPerSecondSquared)
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
	for i, part := range w.parts {
		if part.definition.Role != Dynamic {
			continue
		}
		angular := state.entries[i].AngularVelocity
		if angular.X.Mag() != 0 || angular.Y.Mag() != 0 || angular.Z.Mag() != 0 {
			return ConservationState{}, false
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

// driftConservationChange adds each body's drift coefficients before applying
// its one held mass interval. The source center cancels on a translation slice.
func (w *World) driftConservationChange(kicked State, trace Trace) (ConservationState, bool) {
	var angular, low, high [3]*big.Rat
	for axis := range angular {
		angular[axis], low[axis], high[axis] = new(big.Rat), new(big.Rat), new(big.Rat)
	}
	slices := [][2]State{{kicked, trace.end}}
	if trace.hasEvent {
		slices = [][2]State{{kicked, trace.pre}, {trace.post, trace.end}}
	}
	for i, part := range w.parts {
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

func (w *World) forceImpulses(gravity QuantityVec, loads [2]*BodyLoad,
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
	for i, part := range w.parts {
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

func (w *World) externalContactImpulse(events []ContactEvent) (MomentumReading, bool) {
	var components [3]*big.Rat
	for axis := range components {
		components[axis] = new(big.Rat)
	}
	if w.parts[0].definition.Role == Dynamic && w.parts[1].definition.Role == Dynamic {
		return boundedMomentum(components, components, components)
	}
	// A contact impulse acts along A-to-B on B and oppositely on A.
	sign := int64(1)
	if w.parts[0].definition.Role == Dynamic {
		sign = -1
	}
	for _, event := range events {
		if event.NormalImpulse.Kind() != units.Impulse ||
			validateQuantityVec(event.TangentImpulse, units.Impulse) != nil {
			return MomentumReading{}, false
		}
		normalImpulse := exactBase(event.NormalImpulse)
		if normalImpulse == nil {
			return MomentumReading{}, false
		}
		if event.Kind == ContactTransition && normalImpulse.Sign() == 0 &&
			event.TangentImpulse.X.Mag() == 0 && event.TangentImpulse.Y.Mag() == 0 &&
			event.TangentImpulse.Z.Mag() == 0 {
			continue
		}
		if len(event.Manifold.Points) == 0 {
			return MomentumReading{}, false
		}
		point := event.Manifold.Points[0]
		if point.Normal.Bound.Base() != 0 || point.NormalAngle.Base() != 0 {
			return MomentumReading{}, false
		}
		axis := [3]float64{point.Normal.Value.X, point.Normal.Value.Y, point.Normal.Value.Z}
		for i, value := range axis {
			normal := new(big.Rat).SetFloat64(value)
			tangent := exactBase(velocityComponent(event.TangentImpulse, i))
			if normal == nil || tangent == nil {
				return MomentumReading{}, false
			}
			applied := new(big.Rat).Add(new(big.Rat).Mul(normalImpulse, normal), tangent)
			components[i].Add(components[i], applied.Mul(applied, big.NewRat(sign, 1)))
		}
	}
	return boundedMomentum(components, components, components)
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
