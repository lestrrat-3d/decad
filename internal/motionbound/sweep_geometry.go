package motionbound

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// SweepRadiusDyadic rounds up the farthest perpendicular distance of the
// transformed exact corners from the stated dyadic rotation axis.
func SweepRadiusDyadic(corners []proofarith.DyV3, from r3.Transform, center r3.Vec,
	axis proofarith.DyV3) (*big.Rat, bool) {
	axisSquared := proofarith.DvDot(axis, axis)
	if axisSquared.Sign() <= 0 {
		return nil, false
	}
	pivot := proofarith.DyVec(center)
	place := newSweepPointMap(from)
	best := proofarith.DyZero()
	for _, corner := range corners {
		cross := proofarith.DvCross(proofarith.DvSub(place.apply(corner), pivot), axis)
		if squared := proofarith.DvDot(cross, cross); proofarith.DyCmp(squared, best) > 0 {
			best = squared
		}
	}
	return sweepRadiusOf(new(big.Rat).Quo(best.Rat(), axisSquared.Rat()))
}

// SweepRadiusRational handles an axis with a non-dyadic component.
func SweepRadiusRational(corners []proofarith.DyV3, from r3.Transform, center r3.Vec,
	axis RatVec) (*big.Rat, bool) {
	axisSquared := new(big.Rat)
	for k := range 3 {
		axisSquared.Add(axisSquared, new(big.Rat).Mul(axis[k], axis[k]))
	}
	if axisSquared.Sign() <= 0 {
		return nil, false
	}
	pivot := proofarith.DyVec(center)
	place := newSweepPointMap(from)
	best := new(big.Rat)
	for _, corner := range corners {
		delta := proofarith.DvSub(place.apply(corner), pivot)
		squared := new(big.Rat)
		for k := range 3 {
			i, j := (k+1)%3, (k+2)%3
			component := new(big.Rat).Sub(new(big.Rat).Mul(delta[i].Rat(), axis[j]),
				new(big.Rat).Mul(delta[j].Rat(), axis[i]))
			squared.Add(squared, component.Mul(component, component))
		}
		if squared.Cmp(best) > 0 {
			best = squared
		}
	}
	return sweepRadiusOf(best.Quo(best, axisSquared))
}

// SweepLinearTravel rounds up one affine path's Euclidean displacement.
func SweepLinearTravel(delta [3]proofarith.Dyadic) (*big.Rat, bool) {
	travelSquared := new(big.Rat)
	for _, component := range delta {
		value := component.Rat()
		travelSquared.Add(travelSquared, new(big.Rat).Mul(value, value))
	}
	bound := proofbound.RatSqrtUp(travelSquared)
	if proofbound.IsNonFinite(bound) {
		return nil, false
	}
	return proofarith.FloatRat(bound), true
}

// SweepAngularFrame encloses a drift's angular speed and prepares the exact
// axis and center used by its ideal pose and travel proofs.
func SweepAngularFrame(velocity, omega RatVec, center r3.Vec) (MotionFrame, *big.Rat, *big.Rat,
	*big.Rat, bool) {
	vSquared, omegaSquared := new(big.Rat), new(big.Rat)
	for axis := range 3 {
		vSquared.Add(vSquared, new(big.Rat).Mul(velocity[axis], velocity[axis]))
		omegaSquared.Add(omegaSquared, new(big.Rat).Mul(omega[axis], omega[axis]))
	}
	if omegaSquared.Sign() <= 0 {
		return MotionFrame{}, nil, nil, nil, false
	}
	omegaLow := proofarith.FloatRat(proofbound.RatSqrtDown(omegaSquared))
	omegaHigh := proofarith.FloatRat(proofbound.RatSqrtUp(omegaSquared))
	if omegaLow == nil || omegaHigh == nil || omegaHigh.Sign() <= 0 {
		return MotionFrame{}, nil, nil, nil, false
	}
	pivot, ok := RatVecOf(center)
	if !ok {
		return MotionFrame{}, nil, nil, nil, false
	}
	unit, ok := UnitScaleInterval(omega)
	if !ok {
		return MotionFrame{}, nil, nil, nil, false
	}
	return MotionFrame{Kind: MotionRevolute, Axis: omega, Unit: unit, Center: pivot},
		omegaLow, omegaHigh, vSquared, true
}

// SweepRotatingTravel bounds the full drift path from its linear and angular
// speed bounds and the axis radius.
func SweepRotatingTravel(vSquared, omegaHigh, radius, duration *big.Rat) (*big.Rat, bool) {
	vUp := proofbound.RatSqrtUp(vSquared)
	if proofbound.IsNonFinite(vUp) {
		return nil, false
	}
	speed := new(big.Rat).Add(proofarith.FloatRat(vUp), new(big.Rat).Mul(radius, omegaHigh))
	return new(big.Rat).Mul(speed, duration), true
}

// SweepPointDeviation stages each source point through the rounded pose and
// bounds its squared gap from the same ideal pose as the sweep.
func SweepPointDeviation(source []proofarith.DyV3, pose r3.Transform, rot ScaledIvMat,
	shift IvVec, poll func() error) ([]proofarith.DyV3, *big.Rat, error) {
	actual := make([]proofarith.DyV3, len(source))
	place := newSweepPointMap(pose)
	for i, point := range source {
		if err := poll(); err != nil {
			return nil, nil, err
		}
		actual[i] = place.apply(point)
	}
	return actual, SweepPointDeviationSquared(source, actual, rot, shift), nil
}

type sweepPointMap struct {
	translation, ex, ey, ez proofarith.DyV3
}

func newSweepPointMap(t r3.Transform) sweepPointMap {
	b := t.Basis()
	return sweepPointMap{translation: proofarith.DyVec(t.Translation()),
		ex: proofarith.DyVec(b.EX), ey: proofarith.DyVec(b.EY), ez: proofarith.DyVec(b.EZ)}
}

func (m sweepPointMap) apply(p proofarith.DyV3) proofarith.DyV3 {
	return proofarith.DvAdd(m.translation, proofarith.DvAdd(sweepScaleVec(m.ex, p[0]),
		proofarith.DvAdd(sweepScaleVec(m.ey, p[1]), sweepScaleVec(m.ez, p[2]))))
}

func sweepScaleVec(v proofarith.DyV3, s proofarith.Dyadic) proofarith.DyV3 {
	return proofarith.DyV3{proofarith.DyMul(v[0], s), proofarith.DyMul(v[1], s), proofarith.DyMul(v[2], s)}
}

func sweepRadiusOf(squared *big.Rat) (*big.Rat, bool) {
	radius := proofbound.RatSqrtUp(squared)
	if proofbound.IsNonFinite(radius) {
		return nil, false
	}
	return proofarith.FloatRat(radius), true
}
