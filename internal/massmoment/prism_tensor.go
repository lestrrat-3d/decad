package massmoment

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// PrismLocalInertia forms the centroidal tensor from one volume-moment enclosure.
func PrismLocalInertia(moments Moments, density units.Value) ([3][3]proofbound.RatInterval, *big.Rat, *big.Rat, error) {
	volume, first, second := moments.Volume, moments.First, moments.Second
	var central [3][3]proofbound.RatInterval
	for i := range central {
		for j := range central[i] {
			shift, _ := proofbound.IntervalQuo(proofbound.IntervalMul(first[i], first[j]), volume)
			central[i][j] = proofbound.IntervalSub(second[i][j], shift)
		}
	}
	trace := proofbound.IntervalAdd(proofbound.IntervalAdd(central[0][0], central[1][1]), central[2][2])
	rho := new(big.Rat).Mul(proofarith.FloatRat(density.Mag()), proofarith.FloatRat(density.Unit().Factor()))
	var local [3][3]proofbound.RatInterval
	for i := range local {
		for j := range local[i] {
			term := proofbound.IntervalNeg(central[i][j])
			if i == j {
				term = proofbound.IntervalSub(trace, central[i][j])
			}
			local[i][j] = proofbound.IntervalScale(term, rho)
		}
	}
	eigenLower := GershgorinLower(local)
	if eigenLower.Sign() <= 0 {
		return [3][3]proofbound.RatInterval{}, nil, nil,
			fmt.Errorf("%w: inertia interval does not prove positive definiteness", decaderr.ErrUnsupported)
	}
	return local, rho, eigenLower, nil
}

// RotatePrismInertia maps a local tensor to world axes and charges held-basis error.
func RotatePrismInertia(basis [3][3]*big.Rat, local [3][3]proofbound.RatInterval) [3][3]proofbound.RatInterval {
	world := RotateTensor(basis, local)
	widen := new(big.Rat).Mul(big.NewRat(3, 1), OrthonormalityDefect(basis))
	widen.Mul(widen, new(big.Rat).Add(big.NewRat(2, 1), OrthonormalityDefect(basis)))
	widen.Mul(widen, TensorMagnitude(local))
	for i := range world {
		for j := range world[i] {
			world[i][j] = proofbound.IntervalWiden(world[i][j], widen)
		}
	}
	return world
}
