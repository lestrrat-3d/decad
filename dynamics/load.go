package dynamics

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func (w *World) validateLoads(entries []BodyLoad) ([2]*BodyLoad, error) {
	var loads [2]*BodyLoad
	for i := range entries {
		load := &entries[i]
		if load.Body == nil {
			return loads, fmt.Errorf("%w: nil load body", ErrInvalidInput)
		}
		index := -1
		for j, part := range w.parts {
			if part.definition.Body == load.Body {
				index = j
				break
			}
		}
		if index < 0 || w.parts[index].definition.Role != Dynamic {
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

// kickByLoads applies one bounded force, gravity, and torque kick before any sweep.
func (w *World) kickByLoads(from State, gravity QuantityVec, loads [2]*BodyLoad, dt units.Value) (State, bool) {
	out := from
	limit, duration := exactBase(w.step.VelocityResidual), exactBase(dt)
	if limit == nil || duration == nil {
		return State{}, false
	}
	for i, part := range w.parts {
		if part.definition.Role != Dynamic {
			continue
		}
		mass, bound := exactBase(part.mass.Mass.Value), exactBase(part.mass.Mass.Bound)
		if mass == nil || bound == nil {
			return State{}, false
		}
		massLow := new(big.Rat).Sub(mass, bound)
		massHigh := new(big.Rat).Add(mass, bound)
		if massLow.Sign() <= 0 {
			return State{}, false
		}
		for axis := range 3 {
			g := velocityComponent(gravity, axis)
			force := units.KilogramMillimetersPerSecondSquared(0)
			if loads[i] != nil {
				force = velocityComponent(loads[i].Force, axis)
			}
			// Base() can underflow even when a held load is nonzero.
			if g.Mag() == 0 && force.Mag() == 0 {
				continue
			}
			v := velocityComponent(from.entries[i].LinearVelocity, axis)
			forceValue := exactBase(force)
			if forceValue == nil {
				return State{}, false
			}
			var forceLow, forceHigh *big.Rat
			if forceValue.Sign() < 0 {
				forceLow = new(big.Rat).Quo(forceValue, massLow)
				forceHigh = new(big.Rat).Quo(forceValue, massHigh)
			} else {
				forceLow = new(big.Rat).Quo(forceValue, massHigh)
				forceHigh = new(big.Rat).Quo(forceValue, massLow)
			}
			gravityValue := exactBase(g)
			velocityValue := exactBase(v)
			if gravityValue == nil || velocityValue == nil {
				return State{}, false
			}
			low := new(big.Rat).Add(velocityValue,
				new(big.Rat).Mul(new(big.Rat).Add(gravityValue, forceLow), duration))
			high := new(big.Rat).Add(velocityValue,
				new(big.Rat).Mul(new(big.Rat).Add(gravityValue, forceHigh), duration))
			published := v.Base() + (g.Base()+force.Base()/part.mass.Mass.Value.Base())*dt.Base()
			read := new(big.Rat).SetFloat64(published)
			if !finite(published) || read == nil ||
				intervalDeviation(read, low, high).Cmp(limit) > 0 {
				return State{}, false
			}
			setVelocityComponent(&out.entries[i].LinearVelocity, axis, units.MillimetersPerSecond(published))
		}
		if !kickAngular(&out.entries[i], part.mass, loads[i], dt, w.step.AngularVelocityResidual) {
			return State{}, false
		}
	}
	return out, true
}

// kickAngular applies the initial-orientation Euler kick. The residual is
// checked over every admitted source-inertia component interval.
func kickAngular(entry *BodyState, mass decad.MassProperties, load *BodyLoad,
	dt, limit units.Value) bool {
	for _, component := range []units.Value{entry.AngularVelocity.X, entry.AngularVelocity.Y,
		entry.AngularVelocity.Z} {
		if component.Mag() != 0 && component.Base() == 0 {
			return false
		}
	}
	if load != nil {
		for _, component := range []units.Value{load.Torque.X, load.Torque.Y, load.Torque.Z} {
			if component.Mag() != 0 && component.Base() == 0 {
				return false
			}
		}
	}
	omega := r3.Vec{X: entry.AngularVelocity.X.Base(), Y: entry.AngularVelocity.Y.Base(),
		Z: entry.AngularVelocity.Z.Base()}
	torque := r3.Vec{}
	if load != nil {
		torque = r3.Vec{X: load.Torque.X.Base(), Y: load.Torque.Y.Base(), Z: load.Torque.Z.Base()}
	}
	if zeroAngularVelocity(entry.AngularVelocity) && torque == (r3.Vec{}) &&
		(load == nil || (load.Torque.X.Mag() == 0 && load.Torque.Y.Mag() == 0 && load.Torque.Z.Mag() == 0)) {
		return true
	}
	inertia := mass.Inertia
	tensor, err := r3.NewSymmetricTensor(inertia.XX.Value.Base(), inertia.YY.Value.Base(),
		inertia.ZZ.Value.Base(), inertia.XY.Value.Base(), inertia.XZ.Value.Base(), inertia.YZ.Value.Base())
	if err != nil {
		return false
	}
	world, err := tensor.Rotate(entry.Pose)
	if err != nil {
		return false
	}
	inverse, err := world.Inverse()
	if err != nil {
		return false
	}
	acceleration := inverse.Apply(torque.Sub(omega.Cross(world.Apply(omega))))
	updated := omega.Add(acceleration.Scale(dt.Base()))
	if !finite(updated.X, updated.Y, updated.Z) {
		return false
	}
	before := entry.AngularVelocity
	after := QuantityVec{X: units.RadiansPerSecond(updated.X), Y: units.RadiansPerSecond(updated.Y),
		Z: units.RadiansPerSecond(updated.Z)}
	if !angularKickWithin(inertia, entry.Pose, before, after, load, dt, limit) {
		return false
	}
	entry.AngularVelocity = after
	return true
}

// angularKickWithin bounds I*(omegaAfter-omegaBefore)/dt + omegaBefore×I*omegaBefore - torque.
// Radian is explicitly a scalar inside this physical-law calculation.
func angularKickWithin(inertia decad.InertiaReading, pose r3.Transform, before, after QuantityVec,
	load *BodyLoad, dt, limit units.Value) bool {
	duration, allowance, eigenLower := exactBase(dt), exactBase(limit),
		certifiedInertiaLower(decad.MassProperties{Inertia: inertia})
	if duration == nil || duration.Sign() <= 0 || allowance == nil || eigenLower == nil || eigenLower.Sign() <= 0 {
		return false
	}
	var omega [3]*big.Rat
	var derivative [3]*big.Rat
	for axis := range 3 {
		omega[axis] = exactBase(velocityComponent(before, axis))
		updated := exactBase(velocityComponent(after, axis))
		if omega[axis] == nil || updated == nil {
			return false
		}
		derivative[axis] = new(big.Rat).Quo(new(big.Rat).Sub(updated, omega[axis]), duration)
	}
	rotation, _, ok := spinBasis(pose, before)
	if !ok {
		return false
	}
	var local [3]*big.Rat
	for i := range local {
		local[i] = new(big.Rat)
		for axis := range 3 {
			local[i].Add(local[i], new(big.Rat).Mul(rotation[axis][i], derivative[axis]))
		}
	}
	var delta spinReading
	for axis := range 3 {
		delta.value[axis], delta.low[axis], delta.high[axis] =
			new(big.Rat), new(big.Rat), new(big.Rat)
	}
	for _, component := range inertiaComponents(inertia) {
		quantity, uncertainty := exactBase(component.reading.Value), exactBase(component.reading.Bound)
		if quantity == nil || uncertainty == nil || uncertainty.Sign() < 0 {
			return false
		}
		for axis := range 3 {
			coefficient := new(big.Rat).Mul(rotation[axis][component.i], local[component.j])
			if component.i != component.j {
				coefficient.Add(coefficient,
					new(big.Rat).Mul(rotation[axis][component.j], local[component.i]))
			}
			addIntervalProduct(delta.value[axis], delta.low[axis], delta.high[axis],
				quantity, uncertainty, coefficient)
		}
	}
	spin, ok := spinReadings(inertia, pose, before)
	if !ok {
		return false
	}
	totalResidual := new(big.Rat)
	for axis := range 3 {
		j, k := (axis+1)%3, (axis+2)%3
		value := new(big.Rat).Set(delta.value[axis])
		low, high := new(big.Rat).Set(delta.low[axis]), new(big.Rat).Set(delta.high[axis])
		addSignedSpinProduct(value, low, high, omega[j], spin, k)
		addSignedSpinProduct(value, low, high, new(big.Rat).Neg(omega[k]), spin, j)
		if load != nil {
			torque := exactBase(velocityComponent(load.Torque, axis))
			if torque == nil {
				return false
			}
			value.Sub(value, torque)
			low.Sub(low, torque)
			high.Sub(high, torque)
		}
		residual := mathRatMax(absRat(low), absRat(high))
		totalResidual.Add(totalResidual, residual)
	}
	// The L1 residual bounds its Euclidean norm. Divide by the certified
	// minimum eigenvalue, then multiply by duration to bound the kick error.
	totalResidual.Mul(totalResidual, duration)
	return totalResidual.Cmp(new(big.Rat).Mul(eigenLower, allowance)) <= 0
}

func addSignedSpinProduct(value, low, high, coefficient *big.Rat, spin spinReading, axis int) {
	value.Add(value, new(big.Rat).Mul(coefficient, spin.value[axis]))
	if coefficient.Sign() < 0 {
		low.Add(low, new(big.Rat).Mul(coefficient, spin.high[axis]))
		high.Add(high, new(big.Rat).Mul(coefficient, spin.low[axis]))
	} else {
		low.Add(low, new(big.Rat).Mul(coefficient, spin.low[axis]))
		high.Add(high, new(big.Rat).Mul(coefficient, spin.high[axis]))
	}
}

func mathRatMax(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) > 0 {
		return a
	}
	return b
}
