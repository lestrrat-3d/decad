package dynamics

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/units"
)

func (w *World) validateLoads(entries []BodyLoad) ([2]*BodyLoad, error) {
	var loads [2]*BodyLoad
	unsupportedTorque := false
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
		if load.Torque.X.Base() != 0 || load.Torque.Y.Base() != 0 || load.Torque.Z.Base() != 0 {
			unsupportedTorque = true
		}
		loads[index] = load
	}
	if unsupportedTorque {
		return loads, fmt.Errorf("%w: torque response is not implemented", ErrUnsupported)
	}
	return loads, nil
}

// kickByLoads applies one bounded force and gravity kick before any sweep.
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
	}
	return out, true
}
