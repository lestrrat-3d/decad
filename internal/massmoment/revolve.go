package massmoment

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// RevolveAngular holds the certified integrals of the angular factors.
type RevolveAngular struct {
	Width, Cos, Sin    proofbound.RatInterval
	Cos2, SinCos, Sin2 proofbound.RatInterval
}

// AngularFactors encloses the angular integrals from a sweep's width and endpoint values.
func AngularFactors(width, s0, c0, s1, c1 proofbound.RatInterval) RevolveAngular {
	half := big.NewRat(1, 2)
	halfWidth := proofbound.IntervalScale(width, half)
	doubleAngle := proofbound.IntervalScale(proofbound.IntervalSub(proofbound.IntervalMul(s1, c1), proofbound.IntervalMul(s0, c0)), half)
	return RevolveAngular{
		Width:  width,
		Cos:    proofbound.IntervalSub(s1, s0),
		Sin:    proofbound.IntervalSub(c0, c1),
		Cos2:   proofbound.IntervalAdd(halfWidth, doubleAngle),
		Sin2:   proofbound.IntervalSub(halfWidth, doubleAngle),
		SinCos: proofbound.IntervalScale(proofbound.IntervalSub(proofbound.IntervalMul(s1, s1), proofbound.IntervalMul(s0, s0)), half),
	}
}

// RevolveMoments integrates the section moments about the local axis anchor.
func RevolveMoments(plane [4][4]proofbound.RatInterval, aU, aV, dU, dV float64, angular RevolveAngular) (Moments, error) {
	axisMoment := AxisMoments(plane, aU, aV, dU, dV)
	r1 := axisMoment(0, 1)
	zr := axisMoment(1, 1)
	r2 := axisMoment(0, 2)
	z2r := axisMoment(2, 1)
	zr2 := axisMoment(1, 2)
	r3m := axisMoment(0, 3)

	volume := proofbound.IntervalMul(angular.Width, r1)
	if volume.Lo.Sign() <= 0 {
		return Moments{}, fmt.Errorf("%w: revolve volume interval does not prove positive volume", decaderr.ErrUnsupported)
	}
	first := [3]proofbound.RatInterval{
		proofbound.IntervalMul(angular.Width, zr),
		proofbound.IntervalMul(angular.Cos, r2),
		proofbound.IntervalMul(angular.Sin, r2),
	}
	var second [3][3]proofbound.RatInterval
	second[0][0] = proofbound.IntervalMul(angular.Width, z2r)
	second[0][1] = proofbound.IntervalMul(angular.Cos, zr2)
	second[0][2] = proofbound.IntervalMul(angular.Sin, zr2)
	second[1][1] = proofbound.IntervalMul(angular.Cos2, r3m)
	second[1][2] = proofbound.IntervalMul(angular.SinCos, r3m)
	second[2][2] = proofbound.IntervalMul(angular.Sin2, r3m)
	second[1][0], second[2][0], second[2][1] = second[0][1], second[0][2], second[1][2]
	return Moments{Volume: volume, First: first, Second: second}, nil
}
