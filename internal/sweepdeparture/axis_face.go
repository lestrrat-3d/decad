package sweepdeparture

import "math/big"

// AxisFace proves immediate departure when exact source faces share an axis
// plane and their relative motion opens its gap.
func AxisFace(paths [2]Path, axis, side int) (*big.Rat, bool) {
	velocity := func(path Path) (*big.Rat, bool) {
		if !path.Drift {
			return new(big.Rat).Quo(path.Delta[axis].Rat(), path.Duration), true
		}
		for other := range 3 {
			if other != axis && path.Axis[other].Sign() != 0 {
				return nil, false
			}
		}
		return path.Velocity[axis], true
	}
	vA, validA := velocity(paths[0])
	vB, validB := velocity(paths[1])
	if !validA || !validB {
		return nil, false
	}
	derivative := new(big.Rat).Sub(vB, vA)
	if side == 0 {
		derivative.Neg(derivative)
	}
	if derivative.Sign() <= 0 {
		return nil, false
	}
	return big.NewRat(1, 2), true
}
