package dynamics

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// stepThreeTwoDynamic validates and kicks the whole world before the first
// pair sweep. A child response sees zero loads and never repeats the kick.
func (w *World) stepThreeTwoDynamic(ctx context.Context, from State, input StepInput,
	dt units.Value, reference *World) (*StepReport, error) {
	loads, err := w.validateThreeLoads(input.Loads)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	kicked := from
	fixed := -1
	for i, part := range w.bodies {
		if part.definition.Role == Fixed {
			fixed = i
			break
		}
	}
	for key, part := range w.bodies {
		if part.definition.Role != Dynamic {
			continue
		}
		pairKey := -1
		for i, indices := range threePairs {
			if indices == [2]int{key, fixed} || indices == [2]int{fixed, key} {
				pairKey = i
				break
			}
		}
		pair := w.three.pairs[pairKey]
		var pairLoads [2]*BodyLoad
		for side, item := range pair.bodies {
			if item.definition.Body == part.definition.Body {
				pairLoads[side] = loads[key]
			}
		}
		advanced, ok := pair.kickByLoads(pairState(from, pair), input.Gravity, pairLoads[:], dt)
		if !ok {
			return w.threeUndecided(pairKey, "force kick or torque kick exceeds its residual"), nil
		}
		for _, entry := range advanced.entries {
			if entry.Body == part.definition.Body {
				kicked = withBodyState(kicked, entry)
			}
		}
	}
	if report, handled, err := w.stepThreeBoxStack(ctx, from, kicked, input, dt); handled || err != nil {
		return report, err
	}
	report, handled, err := w.stepThreeSequential(ctx, from, kicked, input, [2]*BodyLoad{}, dt, reference)
	if handled || err != nil {
		return report, err
	}
	return w.threeUndecided(-1, "two-dynamic response lacks a certified single-event path"), nil
}

func (w *World) validateThreeLoads(entries []BodyLoad) ([3]*BodyLoad, error) {
	var loads [3]*BodyLoad
	for i := range entries {
		load := &entries[i]
		index := -1
		for j, part := range w.bodies {
			if load.Body == part.definition.Body {
				index = j
				break
			}
		}
		if index < 0 || w.bodies[index].definition.Role != Dynamic {
			return loads, fmt.Errorf("%w: load body is not a dynamic member of this world", ErrInvalidInput)
		}
		if loads[index] != nil {
			return loads, fmt.Errorf("%w: duplicate load body", ErrInvalidInput)
		}
		if err := validateQuantityVec(load.Force, units.Force); err != nil {
			return loads, err
		}
		if err := validateQuantityVec(load.Torque, units.Torque); err != nil {
			return loads, err
		}
		loads[index] = load
	}
	return loads, nil
}

// threeDriftState advances every body through its validated child state.
func (w *World) threeDriftState(start State, seconds float64) (State, error) {
	out := start
	seen := make(map[*decad.Body]struct{}, 3)
	for _, pair := range w.three.pairs {
		if pair == nil {
			continue
		}
		advanced, err := driftState(pairState(start, pair), seconds)
		if err != nil {
			return State{}, err
		}
		for _, entry := range advanced.entries {
			if _, present := seen[entry.Body]; present {
				held, _ := out.Body(entry.Body)
				if held.Pose != entry.Pose {
					return State{}, fmt.Errorf("%w: shared body drift poses disagree", ErrUnsupported)
				}
				continue
			}
			out = withBodyState(out, entry)
			seen[entry.Body] = struct{}{}
		}
	}
	if len(seen) != 3 {
		return State{}, fmt.Errorf("%w: three-body drift lacks a body", ErrUnsupported)
	}
	return out, nil
}

func sumReadings(readings ...decad.Measurement) (decad.Measurement, bool) {
	value, low, high := new(big.Rat), new(big.Rat), new(big.Rat)
	for _, reading := range readings {
		v, b := exactBase(reading.Value), exactBase(reading.Bound)
		if v == nil || b == nil || b.Sign() < 0 {
			return decad.Measurement{}, false
		}
		value.Add(value, v)
		low.Add(low, new(big.Rat).Sub(v, b))
		high.Add(high, new(big.Rat).Add(v, b))
	}
	return boundedReading(value, low, high, readings[0].Value.Unit())
}

func sumMomentum(readings ...MomentumReading) (MomentumReading, bool) {
	var value, low, high [3]*big.Rat
	for axis := range value {
		value[axis], low[axis], high[axis] = new(big.Rat), new(big.Rat), new(big.Rat)
		for _, reading := range readings {
			v, b := exactBase(velocityComponent(reading.Value, axis)),
				exactBase(velocityComponent(reading.Bound, axis))
			if v == nil || b == nil || b.Sign() < 0 {
				return MomentumReading{}, false
			}
			value[axis].Add(value[axis], v)
			low[axis].Add(low[axis], new(big.Rat).Sub(v, b))
			high[axis].Add(high[axis], new(big.Rat).Add(v, b))
		}
	}
	return boundedVector(value, low, high, readings[0].Value.X.Unit())
}

func sumConservationStates(a, b ConservationState) (ConservationState, bool) {
	energy, ok := sumReadings(a.KineticEnergy, b.KineticEnergy)
	if !ok {
		return ConservationState{}, false
	}
	linear, ok := sumMomentum(a.LinearMomentum, b.LinearMomentum)
	if !ok {
		return ConservationState{}, false
	}
	angular, ok := sumMomentum(a.AngularMomentum, b.AngularMomentum)
	if !ok {
		return ConservationState{}, false
	}
	return ConservationState{KineticEnergy: energy,
		LinearMomentum: linear, AngularMomentum: angular}, true
}

func (w *World) threeTwoDynamicConservation(from, kicked, end State, trace Trace,
	events []ContactEvent, input StepInput, dt units.Value) (StepConservation, bool) {
	loads, err := w.validateThreeLoads(input.Loads)
	if err != nil {
		return StepConservation{}, false
	}
	var individual [2]StepConservation
	count := 0
	for worldIndex, part := range w.bodies {
		if part.definition.Role != Dynamic {
			continue
		}
		var pair *World
		for key, indices := range threePairs {
			if indices[0] != worldIndex && indices[1] != worldIndex {
				continue
			}
			other := indices[0]
			if other == worldIndex {
				other = indices[1]
			}
			if w.bodies[other].definition.Role == Fixed {
				pair = w.three.pairs[key]
				break
			}
		}
		if pair == nil {
			return StepConservation{}, false
		}
		var pairLoads [2]*BodyLoad
		for side, entry := range pair.bodies {
			if entry.definition.Body == part.definition.Body {
				pairLoads[side] = loads[worldIndex]
			}
		}
		first, ok := pair.conservationState(pairState(from, pair))
		if !ok {
			return StepConservation{}, false
		}
		kick, ok := pair.conservationState(pairState(kicked, pair))
		if !ok {
			return StepConservation{}, false
		}
		last, ok := pair.conservationState(pairState(end, pair))
		if !ok {
			return StepConservation{}, false
		}
		gravity, load, ok := pair.forceImpulses(input.Gravity, pairLoads[:], dt)
		if !ok {
			return StepConservation{}, false
		}
		torque, ok := pair.torqueImpulse(pairLoads[:], dt)
		if !ok {
			return StepConservation{}, false
		}
		slices := make([][2]State, 0, len(trace.threeSlices))
		for _, slice := range trace.threeSlices {
			slices = append(slices, [2]State{pairState(slice.from, pair), pairState(slice.to, pair)})
		}
		drift, ok := pair.driftConservationSlices(slices)
		if !ok {
			return StepConservation{}, false
		}
		individual[count] = StepConservation{Input: first, AfterKick: kick, Completion: last,
			GravityImpulse: gravity, LoadImpulse: load, TorqueImpulse: torque,
			DriftChange: drift}
		count++
	}
	if count != 2 {
		return StepConservation{}, false
	}
	first, ok := sumConservationStates(individual[0].Input, individual[1].Input)
	if !ok {
		return StepConservation{}, false
	}
	kick, ok := sumConservationStates(individual[0].AfterKick, individual[1].AfterKick)
	if !ok {
		return StepConservation{}, false
	}
	last, ok := sumConservationStates(individual[0].Completion, individual[1].Completion)
	if !ok {
		return StepConservation{}, false
	}
	drift, ok := sumConservationStates(individual[0].DriftChange, individual[1].DriftChange)
	if !ok {
		return StepConservation{}, false
	}
	gravity, ok := sumMomentum(individual[0].GravityImpulse, individual[1].GravityImpulse)
	if !ok {
		return StepConservation{}, false
	}
	load, ok := sumMomentum(individual[0].LoadImpulse, individual[1].LoadImpulse)
	if !ok {
		return StepConservation{}, false
	}
	torque, ok := sumMomentum(individual[0].TorqueImpulse, individual[1].TorqueImpulse)
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
	return StepConservation{Input: first, AfterKick: kick, Completion: last,
		DriftChange: drift, GravityImpulse: gravity, LoadImpulse: load,
		TorqueImpulse: torque, ContactImpulse: contact, KinematicWork: zero}, true
}
