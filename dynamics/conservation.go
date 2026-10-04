package dynamics

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// MomentumReading encloses each component of a linear impulse or momentum.
// Contact impulse bounds refer to the published numerical event impulses.
type MomentumReading struct {
	Value QuantityVec
	Bound QuantityVec
}

// ConservationState sums readings over the world's dynamic bodies only.
type ConservationState struct {
	// KineticEnergy has units.Torque's dimensions, kg*mm^2/s^2.
	KineticEnergy  decad.Measurement
	LinearMomentum MomentumReading
}

// StepConservation describes the discrete input, kick, and completed step.
// Fixed and kinematic bodies contribute only through ContactImpulse. No
// reading claims the physical impulse of an unresolved continuous contact.
type StepConservation struct {
	Input, AfterKick, Completion                ConservationState
	GravityImpulse, LoadImpulse, ContactImpulse MomentumReading
}

func (w *World) conservationReadings(from, kicked, end State, events []ContactEvent,
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
	return StepConservation{Input: input, AfterKick: afterKick, Completion: completion,
		GravityImpulse: gravityImpulse, LoadImpulse: loadImpulse, ContactImpulse: contactImpulse}, true
}

func (w *World) conservationState(state State) (ConservationState, bool) {
	energy := new(big.Rat)
	energyLow := new(big.Rat)
	energyHigh := new(big.Rat)
	var momentum, low, high [3]*big.Rat
	for axis := range momentum {
		momentum[axis], low[axis], high[axis] = new(big.Rat), new(big.Rat), new(big.Rat)
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
		speedSquared := new(big.Rat)
		for axis := range momentum {
			v := exactBase(velocityComponent(state.entries[i].LinearVelocity, axis))
			if v == nil {
				return ConservationState{}, false
			}
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
	return ConservationState{KineticEnergy: kinetic, LinearMomentum: linear}, true
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
	reading := MomentumReading{}
	for axis := range value {
		component, ok := boundedReading(value[axis], low[axis], high[axis],
			units.KilogramMillimeterPerSecond)
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
