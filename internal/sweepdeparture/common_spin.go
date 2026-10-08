package sweepdeparture

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// CommonSpin bounds the opening gap of two boxes with equal angular velocity
// after the caller verifies exact axis-face contact at their starting poses.
func CommonSpin(paths [2]Path, axis, side int) (*big.Rat, bool) {
	var omega, difference, velocity [3]*big.Rat
	for i := range 3 {
		if paths[0].Axis[i].Cmp(paths[1].Axis[i]) != 0 {
			return nil, false
		}
		omega[i] = paths[0].Axis[i]
		difference[i] = new(big.Rat).Sub(paths[1].Center[i], paths[0].Center[i])
		velocity[i] = new(big.Rat).Sub(paths[1].Velocity[i], paths[0].Velocity[i])
	}
	sign := int64(1)
	if side == 0 {
		sign = -1
	}
	derivative := new(big.Rat).Mul(velocity[axis], big.NewRat(sign, 1))
	normal := [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
	normal[axis] = big.NewRat(sign, 1)
	cross := [3]*big.Rat{
		new(big.Rat).Sub(new(big.Rat).Mul(omega[1], normal[2]),
			new(big.Rat).Mul(omega[2], normal[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(omega[2], normal[0]),
			new(big.Rat).Mul(omega[0], normal[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(omega[0], normal[1]),
			new(big.Rat).Mul(omega[1], normal[0])),
	}
	for i := range 3 {
		derivative.Add(derivative, new(big.Rat).Mul(cross[i], difference[i]))
	}
	if derivative.Sign() <= 0 {
		return nil, false
	}
	normUpper := func(vector [3]*big.Rat) *big.Rat {
		squared := new(big.Rat)
		for i := range 3 {
			squared.Add(squared, new(big.Rat).Mul(vector[i], vector[i]))
		}
		root := proofbound.RatSqrtUp(squared)
		if math.IsNaN(root) || math.IsInf(root, 0) {
			return nil
		}
		return proofarith.FloatRat(root)
	}
	dNorm, vNorm := normUpper(difference), normUpper(velocity)
	if dNorm == nil || vNorm == nil {
		return nil, false
	}
	omegaUpper := paths[0].OmegaUpper
	omegaSquared := new(big.Rat).Mul(omegaUpper, omegaUpper)
	curvature := new(big.Rat).Add(new(big.Rat).Mul(omegaSquared, dNorm),
		new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Mul(omegaUpper, vNorm)))
	curvature.Add(curvature, new(big.Rat).Mul(omegaSquared,
		new(big.Rat).Mul(vNorm, paths[0].Duration)))
	fraction := big.NewRat(1, 1)
	for range 60 {
		until := new(big.Rat).Mul(fraction, paths[0].Duration)
		if new(big.Rat).Mul(curvature, until).Cmp(derivative) < 0 {
			return fraction, true
		}
		fraction = new(big.Rat).Quo(fraction, big.NewRat(2, 1))
	}
	return nil, false
}
