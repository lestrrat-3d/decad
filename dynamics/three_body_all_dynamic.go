package dynamics

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// Each dynamic body receives the full-step kick once. Child worlds provide
// its admitted mass and inertia; only the selected body's result is copied.
func (w *World) stepThreeAllDynamic(ctx context.Context, from State, input StepInput,
	dt units.Value, reference *World) (*StepReport, error) {
	loads, err := w.validateThreeLoads(input.Loads)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	kicked := from
	for worldIndex, part := range w.three.parts {
		var pair *World
		pairKey := -1
		for key, indices := range threePairs {
			if indices[0] == worldIndex || indices[1] == worldIndex {
				pair, pairKey = w.three.pairs[key], key
				break
			}
		}
		if pair == nil {
			return w.threeUndecided(pairKey, "dynamic body has no validated pair"), nil
		}
		var pairLoads [2]*BodyLoad
		for side, member := range pair.parts {
			if member.definition.Body == part.Body {
				pairLoads[side] = loads[worldIndex]
			}
		}
		advanced, ok := pair.kickByLoads(pairState(from, pair), input.Gravity, pairLoads, dt)
		if !ok {
			return w.threeUndecided(pairKey, "force kick or torque kick exceeds its residual"), nil
		}
		for _, entry := range advanced.entries {
			if entry.Body == part.Body {
				kicked = withBodyState(kicked, entry)
			}
		}
	}
	if report, handled, err := w.stepThreeDynamicSphereFriction(ctx, from, kicked, input, dt); handled || err != nil {
		return report, err
	}
	if report, handled, err := w.stepThreeDynamicSphereClear(ctx, from, kicked, input, dt); handled || err != nil {
		return report, err
	}
	report, handled, err := w.stepThreeSequential(ctx, from, kicked, input,
		[2]*BodyLoad{}, dt, reference)
	if handled || err != nil {
		return report, err
	}
	return w.threeUndecided(-1, "three-dynamic response lacks a certified isolated pair path"), nil
}

// Each of the three dynamic bodies occurs in exactly two child pairs. Sum
// their bounded readings and divide by two, preserving the input intervals.
func (w *World) threeAllDynamicConservation(from, kicked, end State, trace Trace,
	events []ContactEvent, input StepInput, dt units.Value) (StepConservation, bool) {
	loads, err := w.validateThreeLoads(input.Loads)
	if err != nil {
		return StepConservation{}, false
	}
	var parts [3]StepConservation
	for key, pair := range w.three.pairs {
		if pair == nil {
			return StepConservation{}, false
		}
		var pairLoads [2]*BodyLoad
		for side, worldIndex := range threePairs[key] {
			pairLoads[side] = loads[worldIndex]
		}
		var ok bool
		parts[key].Input, ok = pair.conservationState(pairState(from, pair))
		if !ok {
			return StepConservation{}, false
		}
		parts[key].AfterKick, ok = pair.conservationState(pairState(kicked, pair))
		if !ok {
			return StepConservation{}, false
		}
		parts[key].Completion, ok = pair.conservationState(pairState(end, pair))
		if !ok {
			return StepConservation{}, false
		}
		parts[key].GravityImpulse, parts[key].LoadImpulse, ok =
			pair.forceImpulses(input.Gravity, pairLoads, dt)
		if !ok {
			return StepConservation{}, false
		}
		parts[key].TorqueImpulse, ok = pair.torqueImpulse(pairLoads, dt)
		if !ok {
			return StepConservation{}, false
		}
		slices := make([][2]State, 0, len(trace.threeSlices))
		for _, slice := range trace.threeSlices {
			slices = append(slices, [2]State{pairState(slice.from, pair), pairState(slice.to, pair)})
		}
		if len(slices) == 0 {
			if trace.hasEvent {
				slices = append(slices,
					[2]State{pairState(kicked, pair), pairState(trace.pre, pair)},
					[2]State{pairState(trace.post, pair), pairState(end, pair)})
			} else {
				slices = append(slices, [2]State{pairState(kicked, pair), pairState(end, pair)})
			}
		}
		parts[key].DriftChange, ok = pair.driftConservationSlices(slices)
		if !ok {
			return StepConservation{}, false
		}
	}
	inputState, ok := halfThreeConservation(parts[0].Input, parts[1].Input, parts[2].Input)
	if !ok {
		return StepConservation{}, false
	}
	kickState, ok := halfThreeConservation(parts[0].AfterKick, parts[1].AfterKick, parts[2].AfterKick)
	if !ok {
		return StepConservation{}, false
	}
	lastState, ok := halfThreeConservation(parts[0].Completion, parts[1].Completion, parts[2].Completion)
	if !ok {
		return StepConservation{}, false
	}
	driftState, ok := halfThreeConservation(parts[0].DriftChange, parts[1].DriftChange, parts[2].DriftChange)
	if !ok {
		return StepConservation{}, false
	}
	gravity, ok := halfThreeMomentum(parts[0].GravityImpulse, parts[1].GravityImpulse,
		parts[2].GravityImpulse)
	if !ok {
		return StepConservation{}, false
	}
	load, ok := halfThreeMomentum(parts[0].LoadImpulse, parts[1].LoadImpulse,
		parts[2].LoadImpulse)
	if !ok {
		return StepConservation{}, false
	}
	torque, ok := halfThreeMomentum(parts[0].TorqueImpulse, parts[1].TorqueImpulse,
		parts[2].TorqueImpulse)
	if !ok {
		return StepConservation{}, false
	}
	contact, ok := w.threeSequentialContactImpulse(events)
	if !ok {
		return StepConservation{}, false
	}
	zero, ok := boundedReading(new(big.Rat), new(big.Rat), new(big.Rat),
		units.KilogramSquareMillimeterPerSecondSquared)
	if !ok {
		return StepConservation{}, false
	}
	return StepConservation{Input: inputState, AfterKick: kickState, Completion: lastState,
		DriftChange: driftState, GravityImpulse: gravity, LoadImpulse: load,
		TorqueImpulse: torque, ContactImpulse: contact, KinematicWork: zero}, true
}

func halfThreeConservation(a, b, c ConservationState) (ConservationState, bool) {
	first, ok := sumConservationStates(a, b)
	if !ok {
		return ConservationState{}, false
	}
	total, ok := sumConservationStates(first, c)
	if !ok {
		return ConservationState{}, false
	}
	energy, ok := halfReading(total.KineticEnergy)
	if !ok {
		return ConservationState{}, false
	}
	linear, ok := halfMomentum(total.LinearMomentum)
	if !ok {
		return ConservationState{}, false
	}
	angular, ok := halfMomentum(total.AngularMomentum)
	if !ok {
		return ConservationState{}, false
	}
	return ConservationState{KineticEnergy: energy, LinearMomentum: linear,
		AngularMomentum: angular}, true
}

func halfThreeMomentum(a, b, c MomentumReading) (MomentumReading, bool) {
	total, ok := sumMomentum(a, b, c)
	if !ok {
		return MomentumReading{}, false
	}
	return halfMomentum(total)
}

func halfReading(reading decad.Measurement) (decad.Measurement, bool) {
	value, bound := exactBase(reading.Value), exactBase(reading.Bound)
	if value == nil || bound == nil || bound.Sign() < 0 {
		return decad.Measurement{}, false
	}
	half := big.NewRat(1, 2)
	low := new(big.Rat).Mul(new(big.Rat).Sub(value, bound), half)
	high := new(big.Rat).Mul(new(big.Rat).Add(value, bound), half)
	return boundedReading(new(big.Rat).Mul(value, half), low, high, reading.Value.Unit())
}

func halfMomentum(reading MomentumReading) (MomentumReading, bool) {
	var value, low, high [3]*big.Rat
	half := big.NewRat(1, 2)
	for axis := range value {
		v, b := exactBase(velocityComponent(reading.Value, axis)),
			exactBase(velocityComponent(reading.Bound, axis))
		if v == nil || b == nil || b.Sign() < 0 {
			return MomentumReading{}, false
		}
		value[axis] = new(big.Rat).Mul(v, half)
		low[axis] = new(big.Rat).Mul(new(big.Rat).Sub(v, b), half)
		high[axis] = new(big.Rat).Mul(new(big.Rat).Add(v, b), half)
	}
	return boundedVector(value, low, high, reading.Value.X.Unit())
}
