package massmoment

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// RigidInertia forms the world inertia interval and exact density from local moments.
func RigidInertia(m Moments, rotation [3][3]*big.Rat, density units.Value) ([3][3]proofbound.RatInterval, *big.Rat, error) {
	volume, first, second := m.Volume, m.First, m.Second
	if volume.Lo.Sign() <= 0 {
		return [3][3]proofbound.RatInterval{}, nil, fmt.Errorf("%w: volume interval does not prove positive volume", decaderr.ErrUnsupported)
	}
	var centroidal [3][3]proofbound.RatInterval
	for i := range 3 {
		for j := i; j < 3; j++ {
			shift, ok := proofbound.IntervalQuo(proofbound.IntervalMul(first[i], first[j]), volume)
			if !ok {
				return [3][3]proofbound.RatInterval{}, nil, fmt.Errorf("%w: volume interval does not prove positive volume", decaderr.ErrUnsupported)
			}
			centroidal[i][j] = proofbound.IntervalSub(second[i][j], shift)
			centroidal[j][i] = centroidal[i][j]
		}
	}

	rho := new(big.Rat).Mul(proofarith.FloatRat(density.Mag()), proofarith.FloatRat(density.Unit().Factor()))
	trace := proofbound.IntervalAdd(proofbound.IntervalAdd(centroidal[0][0], centroidal[1][1]), centroidal[2][2])
	var local [3][3]proofbound.RatInterval
	for i := range 3 {
		for j := range 3 {
			if i == j {
				local[i][j] = proofbound.IntervalScale(proofbound.IntervalSub(trace, centroidal[i][i]), rho)
				continue
			}
			local[i][j] = proofbound.IntervalScale(centroidal[i][j], new(big.Rat).Neg(rho))
		}
	}
	world := RotateTensor(rotation, local)
	defect := OrthonormalityDefect(rotation)
	widen := new(big.Rat).Mul(big.NewRat(3, 1), defect)
	widen.Mul(widen, new(big.Rat).Add(big.NewRat(2, 1), defect))
	widen.Mul(widen, TensorMagnitude(local))
	for i := range world {
		for j := range world[i] {
			world[i][j] = proofbound.IntervalWiden(world[i][j], widen)
		}
	}
	return world, rho, nil
}
