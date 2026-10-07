package motionbound

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// SweepIdealPoseAt encloses a prepared sweep path's ideal affine pose at f.
// Translation uses the exact staged displacement. Rotation uses both rate
// bounds and the source transform's exact rational reading.
func SweepIdealPoseAt(fromRot IvMat, fromT RatVec, delta [3]proofarith.Dyadic,
	frame MotionFrame, velocity RatVec, omegaLow, omegaHigh, duration, f *big.Rat,
	rotating bool) (ScaledIvMat, IvVec, bool) {
	if !rotating {
		shift := PointVec(fromT)
		for axis := range 3 {
			shift[axis] = proofbound.IntervalAdd(shift[axis],
				proofbound.PointInterval(new(big.Rat).Mul(delta[axis].Rat(), f)))
		}
		return NewScaledIvMat(fromRot), shift, true
	}
	elapsed := new(big.Rat).Mul(duration, f)
	angleLow := new(big.Rat).Mul(omegaLow, elapsed)
	angleHigh := new(big.Rat).Mul(omegaHigh, elapsed)
	sin, cos := RadianSinCos(angleLow)
	width := new(big.Rat).Sub(angleHigh, angleLow)
	sin = proofbound.IntervalOwned(new(big.Rat).Sub(sin.Lo, width), new(big.Rat).Add(sin.Hi, width))
	cos = proofbound.IntervalOwned(new(big.Rat).Sub(cos.Lo, width), new(big.Rat).Add(cos.Hi, width))
	rot := frame.ScaledRotation(sin, cos)
	offset := make([]*big.Rat, 3)
	for axis := range 3 {
		offset[axis] = new(big.Rat).Sub(fromT[axis], frame.Center[axis])
	}
	q := proofarith.CommonDenom(offset...)
	lo, hi := rot.ApplyScaled([3]*big.Int{proofarith.ScaledNum(offset[0], q),
		proofarith.ScaledNum(offset[1], q), proofarith.ScaledNum(offset[2], q)})
	rotDen := new(big.Int).Mul(rot.Den, q)
	var shift IvVec
	for axis := range 3 {
		pivot := new(big.Rat).Add(frame.Center[axis], new(big.Rat).Mul(velocity[axis], elapsed))
		den := proofarith.LcmInt(rotDen, pivot.Denom())
		multiplier := new(big.Int).Quo(den, rotDen)
		pivotN := proofarith.ScaledNum(pivot, den)
		low := lo[axis].Mul(lo[axis], multiplier)
		high := hi[axis].Mul(hi[axis], multiplier)
		shift[axis] = proofbound.IntervalOwned(new(big.Rat).SetFrac(low.Add(low, pivotN), den),
			new(big.Rat).SetFrac(high.Add(high, pivotN), den))
	}
	linear, ok := rot.MulPoints(fromRot)
	return linear, shift, ok
}

// SweepTransferCharge bounds the displacement of a point within delta of a
// source point when the rounded pose basis differs from the ideal enclosure.
func SweepTransferCharge(pose r3.Transform, rot ScaledIvMat, delta proofarith.Dyadic) (*big.Rat, bool) {
	if delta.Sign() == 0 {
		return new(big.Rat), true
	}
	rounded, _, ok := ExactTransform(pose)
	if !ok {
		return nil, false
	}
	values := make([]*big.Rat, 0, 9)
	for i := range 3 {
		for k := range 3 {
			values = append(values, rounded[i][k].Lo)
		}
	}
	den := proofarith.LcmInt(proofarith.CommonDenom(values...), rot.Den)
	scale := new(big.Int).Quo(den, rot.Den)
	squared := new(big.Int)
	for i := range 3 {
		for k := range 3 {
			r := proofarith.ScaledNum(values[3*i+k], den)
			below := new(big.Int).Mul(rot.Hi[i][k], scale)
			below.Sub(r, below)
			above := new(big.Int).Mul(rot.Lo[i][k], scale)
			above.Sub(r, above)
			farther := below.Abs(below)
			if above.Abs(above).Cmp(farther) > 0 {
				farther = above
			}
			squared.Add(squared, farther.Mul(farther, farther))
		}
	}
	norm := proofbound.RatSqrtUp(new(big.Rat).SetFrac(squared, new(big.Int).Mul(den, den)))
	if proofbound.IsNonFinite(norm) {
		return nil, false
	}
	return new(big.Rat).Mul(proofarith.FloatRat(norm), delta.Rat()), true
}

// SweepPointDeviationSquared bounds the farthest source point's squared gap
// between its staged rounded position and its ideal pose enclosure.
func SweepPointDeviationSquared(source, actual []proofarith.DyV3, rot ScaledIvMat, shift IvVec) *big.Rat {
	// Dyadic coordinates share one power-of-two denominator.
	exp := 0
	for i, point := range source {
		for axis := range 3 {
			exp = max(exp, dyDenominatorExp(point[axis]), dyDenominatorExp(actual[i][axis]))
		}
	}
	q := new(big.Int).Lsh(big.NewInt(1), uint(exp))
	rotDen := new(big.Int).Mul(rot.Den, q)
	var den, rotMultiplier, observedMultiplier, shiftLo, shiftHi, toWhole [3]*big.Int
	whole := big.NewInt(1)
	for axis := range 3 {
		den[axis] = proofarith.LcmInt(proofarith.LcmInt(rotDen, shift[axis].Lo.Denom()),
			shift[axis].Hi.Denom())
		rotMultiplier[axis] = new(big.Int).Quo(den[axis], rotDen)
		observedMultiplier[axis] = new(big.Int).Quo(den[axis], q)
		shiftLo[axis], shiftHi[axis] = proofarith.ScaledNum(shift[axis].Lo, den[axis]),
			proofarith.ScaledNum(shift[axis].Hi, den[axis])
		whole = proofarith.LcmInt(whole, den[axis])
	}
	for axis := range 3 {
		toWhole[axis] = new(big.Int).Quo(whole, den[axis])
	}
	maxSquared := new(big.Int)
	for i := range source {
		var point [3]*big.Int
		for axis := range 3 {
			point[axis] = dyScaledNum(source[i][axis], exp)
		}
		lo, hi := rot.ApplyScaled(point)
		squared := new(big.Int)
		for axis := range 3 {
			observed := dyScaledNum(actual[i][axis], exp)
			observed.Mul(observed, observedMultiplier[axis])
			low := lo[axis].Mul(lo[axis], rotMultiplier[axis])
			low.Add(low, shiftLo[axis])
			high := hi[axis].Mul(hi[axis], rotMultiplier[axis])
			high.Add(high, shiftHi[axis])
			below := high.Sub(observed, high)
			above := low.Sub(observed, low)
			maximum := below.Abs(below)
			if above.Abs(above).Cmp(maximum) > 0 {
				maximum = above
			}
			maximum.Mul(maximum, toWhole[axis])
			squared.Add(squared, maximum.Mul(maximum, maximum))
		}
		if squared.Cmp(maxSquared) > 0 {
			maxSquared = squared
		}
	}
	return new(big.Rat).SetFrac(maxSquared, new(big.Int).Mul(whole, whole))
}

func dyDenominatorExp(d proofarith.Dyadic) int {
	if d.Sign() == 0 {
		return 0
	}
	return max(0, -d.Exp())
}

func dyScaledNum(d proofarith.Dyadic, shift int) *big.Int {
	if d.Sign() == 0 {
		return new(big.Int)
	}
	mant := d.MantInto(new(big.Int))
	return mant.Lsh(mant, uint(d.Exp()+shift))
}
