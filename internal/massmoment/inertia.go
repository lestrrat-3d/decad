package massmoment

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// AffineInertia forms the world inertia interval and mass interval of the
// image of the local solid under the exact linear map linear (its column k
// the world image of local axis k), with the image's own density. An affine
// map scales every volume element by |det linear|, so the image's mass is
// ρ·|det|·V and its centroidal second moment is |det|·M·S·Mᵀ with S the local
// one, both exact for the image whatever linear's departure from orthonormal.
// Its inertia is ρ·(trace(S′)·1 − S′) of that second moment S′. An exactly
// orthonormal map answers the rigid rotation's tensor exactly.
func AffineInertia(m Moments, linear [3][3]*big.Rat, density units.Value) ([3][3]proofbound.RatInterval, proofbound.RatInterval, error) {
	volume, first, second := m.Volume, m.First, m.Second
	if volume.Lo.Sign() <= 0 {
		return [3][3]proofbound.RatInterval{}, proofbound.RatInterval{}, fmt.Errorf("%w: volume interval does not prove positive volume", decaderr.ErrUnsupported)
	}
	var centroidal [3][3]proofbound.RatInterval
	for i := range 3 {
		for j := i; j < 3; j++ {
			shift, ok := proofbound.IntervalQuo(proofbound.IntervalMul(first[i], first[j]), volume)
			if !ok {
				return [3][3]proofbound.RatInterval{}, proofbound.RatInterval{}, fmt.Errorf("%w: volume interval does not prove positive volume", decaderr.ErrUnsupported)
			}
			centroidal[i][j] = proofbound.IntervalSub(second[i][j], shift)
			centroidal[j][i] = centroidal[i][j]
		}
	}
	det := Determinant(linear)
	det.Abs(det)
	rho := new(big.Rat).Mul(proofarith.FloatRat(density.Mag()), proofarith.FloatRat(density.Unit().Factor()))
	weight := new(big.Rat).Mul(rho, det)
	world := RotateTensor(linear, centroidal)
	trace := proofbound.IntervalAdd(proofbound.IntervalAdd(world[0][0], world[1][1]), world[2][2])
	var out [3][3]proofbound.RatInterval
	for i := range 3 {
		for j := range 3 {
			if i == j {
				out[i][j] = proofbound.IntervalScale(proofbound.IntervalSub(trace, world[i][i]), weight)
				continue
			}
			out[i][j] = proofbound.IntervalScale(world[i][j], new(big.Rat).Neg(weight))
		}
	}
	return out, proofbound.IntervalScale(volume, weight), nil
}
