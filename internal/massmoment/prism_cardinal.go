package massmoment

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// CardinalPrismInertia integrates an undisplaced section and maps its tensor through signed-permutation axes.
func CardinalPrismInertia(section [6]proofbound.RatInterval, z0, z1 float64, frame r3.Frame,
	xform r3.Transform, density units.Value) (proofbound.RatInterval, [3][3]proofbound.RatInterval, error) {
	a, mu, mv := section[0], section[1], section[2]
	if a.Lo.Sign() <= 0 {
		return proofbound.RatInterval{}, [3][3]proofbound.RatInterval{},
			fmt.Errorf("%w: section area interval does not prove positive volume", decaderr.ErrUnsupported)
	}
	h := new(big.Rat).Sub(proofarith.FloatRat(z1), proofarith.FloatRat(z0))
	if h.Sign() <= 0 {
		return proofbound.RatInterval{}, [3][3]proofbound.RatInterval{},
			fmt.Errorf("%w: axial interval does not prove positive volume", decaderr.ErrUnsupported)
	}
	rho := new(big.Rat).Mul(proofarith.FloatRat(density.Mag()), proofarith.FloatRat(density.Unit().Factor()))
	rhoH := new(big.Rat).Mul(rho, h)
	massIv := proofbound.IntervalScale(a, rhoH)
	mu2OverA, _ := proofbound.IntervalQuo(proofbound.IntervalMul(mu, mu), a)
	mv2OverA, _ := proofbound.IntervalQuo(proofbound.IntervalMul(mv, mv), a)
	mumvOverA, _ := proofbound.IntervalQuo(proofbound.IntervalMul(mu, mv), a)
	cuu := proofbound.IntervalSub(section[3], mu2OverA)
	cuv := proofbound.IntervalSub(section[4], mumvOverA)
	cvv := proofbound.IntervalSub(section[5], mv2OverA)
	h2Over12 := new(big.Rat).Quo(new(big.Rat).Mul(h, h), big.NewRat(12, 1))
	axial := proofbound.IntervalScale(a, new(big.Rat).Mul(rhoH, h2Over12))
	local := [3][3]proofbound.RatInterval{}
	local[0][0] = proofbound.IntervalAdd(proofbound.IntervalScale(cvv, rhoH), axial)
	local[1][1] = proofbound.IntervalAdd(proofbound.IntervalScale(cuu, rhoH), axial)
	local[2][2] = proofbound.IntervalScale(proofbound.IntervalAdd(cuu, cvv), rhoH)
	local[0][1] = proofbound.IntervalScale(cuv, new(big.Rat).Neg(rhoH))
	local[1][0] = local[0][1]
	zero := proofbound.PointInterval(new(big.Rat))
	local[0][2], local[2][0], local[1][2], local[2][1] = zero, zero, zero, zero

	world := [3][3]proofbound.RatInterval{}
	var worldAxis [3]int
	var sign [3]int64
	for k, axis := range []r3.Vec{
		xform.ApplyDir(frame.U()),
		xform.ApplyDir(frame.V()),
		xform.ApplyDir(frame.N()),
	} {
		switch {
		case math.Abs(axis.X) == 1 && axis.Y == 0 && axis.Z == 0:
			worldAxis[k], sign[k] = 0, int64(axis.X)
		case axis.X == 0 && math.Abs(axis.Y) == 1 && axis.Z == 0:
			worldAxis[k], sign[k] = 1, int64(axis.Y)
		case axis.X == 0 && axis.Y == 0 && math.Abs(axis.Z) == 1:
			worldAxis[k], sign[k] = 2, int64(axis.Z)
		default:
			return proofbound.RatInterval{}, [3][3]proofbound.RatInterval{},
				fmt.Errorf("%w: prism orientation has no exact cardinal axes", decaderr.ErrUnsupported)
		}
	}
	for i := range local {
		for j := range local[i] {
			world[worldAxis[i]][worldAxis[j]] = proofbound.IntervalScale(local[i][j], big.NewRat(sign[i]*sign[j], 1))
		}
	}
	for i := range world {
		lower := new(big.Rat).Set(world[i][i].Lo)
		for j := range world[i] {
			if i == j {
				continue
			}
			magnitude := new(big.Rat).Abs(world[i][j].Lo)
			if upper := new(big.Rat).Abs(world[i][j].Hi); upper.Cmp(magnitude) > 0 {
				magnitude = upper
			}
			lower.Sub(lower, magnitude)
		}
		if lower.Sign() <= 0 {
			return proofbound.RatInterval{}, [3][3]proofbound.RatInterval{},
				fmt.Errorf("%w: inertia interval does not prove positive definiteness", decaderr.ErrUnsupported)
		}
	}
	return massIv, world, nil
}
