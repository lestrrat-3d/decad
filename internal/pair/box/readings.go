package box

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// RationalPointReading rounds an exact world point and bounds its 3D error.
func RationalPointReading(point [3]*big.Rat) (PointReading, bool) {
	var coordinate [3]float64
	maxError := new(big.Rat)
	for axis := range 3 {
		coordinate[axis], _ = point[axis].Float64()
		if !finite(coordinate[axis]) {
			return PointReading{}, false
		}
		deviation := new(big.Rat).Sub(point[axis], proofarith.FloatRat(coordinate[axis]))
		deviation.Abs(deviation)
		if deviation.Cmp(maxError) > 0 {
			maxError = deviation
		}
	}
	bound := proofbound.Radius3D(proofbound.RatFloatUp(maxError))
	if !finite(bound) {
		return PointReading{}, false
	}
	return PointReading{Value: r3.Vec{X: coordinate[0], Y: coordinate[1], Z: coordinate[2]},
		BoundMM: bound}, true
}

// NormalReading encloses a unit normal and its angular error.
type NormalReading struct {
	Value        r3.Vec
	Bound, Angle float64
}

// DyadicNormalReading rounds a dyadic axis into a bounded unit normal.
func DyadicNormalReading(axis proofarith.DyV3) (NormalReading, bool) {
	squared := proofarith.DvDot(axis, axis).Rat()
	if squared.Sign() <= 0 {
		return NormalReading{}, false
	}
	low, high := proofbound.RatSqrtDown(squared), proofbound.RatSqrtUp(squared)
	if low <= 0 || !finite(low) || !finite(high) {
		return NormalReading{}, false
	}
	raw := r3.Vec{}
	raw.X, _ = axis[0].Float64()
	raw.Y, _ = axis[1].Float64()
	raw.Z, _ = axis[2].Float64()
	value, ok := raw.Normalize()
	if !ok || !proofbound.FiniteVec(value) {
		return NormalReading{}, false
	}
	components := [3]float64{value.X, value.Y, value.Z}
	maxError := new(big.Rat)
	for k := range 3 {
		first := new(big.Rat).Quo(axis[k].Rat(), proofarith.FloatRat(low))
		second := new(big.Rat).Quo(axis[k].Rat(), proofarith.FloatRat(high))
		for _, endpoint := range []*big.Rat{first, second} {
			deviation := new(big.Rat).Sub(proofarith.FloatRat(components[k]), endpoint)
			deviation.Abs(deviation)
			if deviation.Cmp(maxError) > 0 {
				maxError = deviation
			}
		}
	}
	bound := proofbound.Radius3D(proofbound.RatFloatUp(maxError))
	angle := proofbound.UpRound(4 * bound)
	if !finite(bound) || !finite(angle) || angle >= math.Pi {
		return NormalReading{}, false
	}
	return NormalReading{Value: value, Bound: bound, Angle: angle}, true
}
