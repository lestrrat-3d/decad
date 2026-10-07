package motionbound

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// SweepSpans encloses each source point over a sweep fraction interval.
// Each axis stores integer numerators over one shared denominator.
type SweepSpans struct {
	Den    [3]*big.Int
	Lo, Hi [][3]*big.Int
}

// SweepCornerSpan encloses the path of every source point from fraction from
// through to. Rotating paths grow their midpoint enclosure by axis speed.
func SweepCornerSpan(points []proofarith.DyV3, delta [3]proofarith.Dyadic,
	frame MotionFrame, velocity RatVec, omegaLow, omegaHigh, duration, from, to *big.Rat,
	rotating bool) SweepSpans {
	count := len(points)
	output := SweepSpans{Lo: make([][3]*big.Int, count), Hi: make([][3]*big.Int, count)}
	if !rotating {
		starts := make([]*big.Rat, count)
		for axis := range 3 {
			lo := new(big.Rat).Mul(delta[axis].Rat(), from)
			hi := new(big.Rat).Mul(delta[axis].Rat(), to)
			if lo.Cmp(hi) > 0 {
				lo, hi = hi, lo
			}
			for index, point := range points {
				starts[index] = point[axis].Rat()
			}
			den := proofarith.LcmInt(proofarith.LcmInt(proofarith.CommonDenom(starts...), lo.Denom()), hi.Denom())
			output.Den[axis] = den
			loN, hiN := proofarith.ScaledNum(lo, den), proofarith.ScaledNum(hi, den)
			for index, start := range starts {
				startN := proofarith.ScaledNum(start, den)
				output.Lo[index][axis] = new(big.Int).Add(startN, loN)
				output.Hi[index][axis] = startN.Add(startN, hiN)
			}
		}
		return output
	}
	lowTime := new(big.Rat).Mul(duration, from)
	highTime := new(big.Rat).Mul(duration, to)
	lowAngle := new(big.Rat).Mul(omegaLow, lowTime)
	highAngle := new(big.Rat).Mul(omegaHigh, highTime)
	sin, cos := RotationalSinCosSpan(lowAngle, highAngle)
	rotationSpan := frame.ScaledRotation(sin, cos)
	midTime := new(big.Rat).Quo(new(big.Rat).Add(lowTime, highTime), big.NewRat(2, 1))
	angleAtMidLow := new(big.Rat).Mul(omegaLow, midTime)
	angleAtMidHigh := new(big.Rat).Mul(omegaHigh, midTime)
	midSin, midCos := RotationalSinCosSpan(angleAtMidLow, angleAtMidHigh)
	rotationMid := frame.ScaledRotation(midSin, midCos)
	halfDuration := new(big.Rat).Quo(new(big.Rat).Sub(highTime, lowTime), big.NewRat(2, 1))

	relative := make([]*big.Rat, 0, 3*count)
	for _, point := range points {
		for axis := range 3 {
			relative = append(relative, new(big.Rat).Sub(point[axis].Rat(), frame.Center[axis]))
		}
	}
	q := proofarith.CommonDenom(relative...)
	spanLo, spanHi := make([][3]*big.Int, count), make([][3]*big.Int, count)
	midLo, midHi := make([][3]*big.Int, count), make([][3]*big.Int, count)
	for index := range count {
		var offset [3]*big.Int
		for axis := range 3 {
			offset[axis] = proofarith.ScaledNum(relative[3*index+axis], q)
		}
		spanLo[index], spanHi[index] = rotationSpan.ApplyScaled(offset)
		midLo[index], midHi[index] = rotationMid.ApplyScaled(offset)
	}
	spanDen := new(big.Int).Mul(rotationSpan.Den, q)
	midDen := new(big.Int).Mul(rotationMid.Den, q)
	for axis := range 3 {
		following, preceding := (axis+1)%3, (axis+2)%3
		v := velocity[axis]
		scaleF, scaleP := frame.Axis[following], frame.Axis[preceding]
		scaleDen := proofarith.LcmInt(new(big.Int).Set(scaleF.Denom()), scaleP.Denom())
		derivativeDen := proofarith.LcmInt(new(big.Int).Mul(spanDen, scaleDen), v.Denom())
		multiplierF := new(big.Int).Quo(derivativeDen, new(big.Int).Mul(spanDen, scaleF.Denom()))
		multiplierF.Mul(multiplierF, scaleF.Num())
		multiplierP := new(big.Int).Quo(derivativeDen, new(big.Int).Mul(spanDen, scaleP.Denom()))
		multiplierP.Mul(multiplierP, scaleP.Num())
		velocityN := proofarith.ScaledNum(v, derivativeDen)
		shift := new(big.Rat).Add(frame.Center[axis], new(big.Rat).Mul(v, midTime))
		travelDen := new(big.Int).Mul(derivativeDen, halfDuration.Denom())
		den := proofarith.LcmInt(proofarith.LcmInt(midDen, shift.Denom()), travelDen)
		output.Den[axis] = den
		midMultiplier := new(big.Int).Quo(den, midDen)
		travelMultiplier := new(big.Int).Quo(den, travelDen)
		travelMultiplier.Mul(travelMultiplier, halfDuration.Num())
		shiftN := proofarith.ScaledNum(shift, den)
		for index := range count {
			loF := new(big.Int).Mul(spanLo[index][preceding], multiplierF)
			hiF := new(big.Int).Mul(spanHi[index][preceding], multiplierF)
			if multiplierF.Sign() < 0 {
				loF, hiF = hiF, loF
			}
			loP := new(big.Int).Mul(spanLo[index][following], multiplierP)
			hiP := new(big.Int).Mul(spanHi[index][following], multiplierP)
			if multiplierP.Sign() < 0 {
				loP, hiP = hiP, loP
			}
			low := loF.Sub(loF, hiP)
			low.Add(low, velocityN)
			high := hiF.Sub(hiF, loP)
			high.Add(high, velocityN)
			maximum := low.Abs(low)
			if high.Abs(high).Cmp(maximum) > 0 {
				maximum = high
			}
			travel := maximum.Mul(maximum, travelMultiplier)
			outLo := new(big.Int).Mul(midLo[index][axis], midMultiplier)
			outLo.Add(outLo, shiftN)
			output.Lo[index][axis] = outLo.Sub(outLo, travel)
			outHi := new(big.Int).Mul(midHi[index][axis], midMultiplier)
			outHi.Add(outHi, shiftN)
			output.Hi[index][axis] = outHi.Add(outHi, travel)
		}
	}
	return output
}

// RotationalSinCosSpan encloses sine and cosine across a bounded angle span.
func RotationalSinCosSpan(low, high *big.Rat) (proofbound.RatInterval, proofbound.RatInterval) {
	loSin, loCos := RadianSinCos(low)
	hiSin, hiCos := loSin, loCos
	if low.Cmp(high) != 0 {
		hiSin, hiCos = RadianSinCos(high)
	}
	if low.Sign() >= 0 && high.Cmp(proofbound.HalfPiInterval().Lo) <= 0 {
		return proofbound.Interval(loSin.Lo, hiSin.Hi), proofbound.Interval(hiCos.Lo, loCos.Hi)
	}
	width := new(big.Rat).Sub(high, low)
	return proofbound.IntervalOwned(new(big.Rat).Sub(loSin.Lo, width),
			new(big.Rat).Add(loSin.Hi, width)),
		proofbound.IntervalOwned(new(big.Rat).Sub(loCos.Lo, width), new(big.Rat).Add(loCos.Hi, width))
}
