package box

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair"
	proof "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/r3"
)

// OrientedSphereWitness binds an isolated box face to bounded points on both
// source surfaces and a normal oriented by body order.
type OrientedSphereWitness struct {
	Face                  FaceSlot
	SpherePoint, BoxPoint PointReading
	Normal                NormalReading
	Separation            pair.ScalarReading
}

// OrientedSphereResult carries the complete occupied-set relation and an
// optional response witness without public topology.
type OrientedSphereResult struct {
	Relation pair.Relation
	Reason   pair.Reason
	Gap      *pair.ScalarReading
	Witness  *OrientedSphereWitness
}

// ClassifyOrientedSphere compares a complete source ball with an exactly
// orthogonal source box at its read pose. The normal points from the box to
// the sphere unless sphereFirst reverses body order.
func ClassifyOrientedSphere(sphere AxisSphere, box OrientedBox, sphereFirst bool,
	pointResolutionMM, normalResolutionRad float64) OrientedSphereResult {
	result := OrientedSphereResult{}
	if !OrthogonalSourceBox(box) {
		result.Reason = pair.PayloadUnsupported
		return result
	}
	distance2, ok := ClosestOrientedBoxPoint(sphere.Center, box)
	if !ok {
		result.Reason = pair.PayloadUnsupported
		return result
	}
	radius2 := proof.DyMul(sphere.Radius, sphere.Radius).Rat()
	switch distance2.Cmp(radius2) {
	case 1:
		gap, valid := orientedSphereSignedReading(distance2, sphere.Radius)
		if !valid || gap.ValueMM-gap.BoundMM <= 0 {
			result.Reason = pair.NoGapProof
			return result
		}
		result.Relation, result.Gap = pair.Separated, &gap
		return result
	case 0:
		zero := pair.ScalarReading{}
		result.Relation, result.Gap = pair.Touching, &zero
	default:
		result.Relation = pair.Overlapping
	}
	axis, side, outward, coordinate, faceDistance2, faceOK := OrientedSphereFace(sphere.Center, sphere.Radius, box)
	if !faceOK || faceDistance2.Cmp(distance2) != 0 {
		result.Reason = pair.AmbiguousFeature
		return result
	}
	// A shallow overlap must still leave the opposite face outside the ball.
	opposite := new(big.Rat).Set(coordinate[axis])
	if side == 1 {
		opposite = new(big.Rat).Mul(coordinate[axis], OrientedDenominator(box, axis).Rat())
	} else {
		opposite.Sub(big.NewRat(1, 1), opposite)
		opposite.Mul(opposite, OrientedDenominator(box, axis).Rat())
	}
	if new(big.Rat).Mul(opposite, opposite).Cmp(new(big.Rat).Mul(radius2,
		proof.DvDot(outward, outward).Rat())) <= 0 {
		result.Reason = pair.AmbiguousFeature
		return result
	}
	normalAxis := outward
	if sphereFirst {
		for k := range 3 {
			normalAxis[k] = proof.DyNeg(normalAxis[k])
		}
	}
	normal, ok := DyadicNormalReading(normalAxis)
	if !ok || normal.Angle > normalResolutionRad {
		result.Reason = pair.NoNormalProof
		return result
	}
	foot := [3]*big.Rat{}
	for k := range 3 {
		foot[k] = box.Corner[0][k].Rat()
		for edge := range 3 {
			value := coordinate[edge]
			if edge == axis {
				value = big.NewRat(int64(side), 1)
			}
			foot[k].Add(foot[k], new(big.Rat).Mul(value, box.Edge[edge][k].Rat()))
		}
	}
	boxPoint, boxOK := RationalPointReading(foot)
	spherePoint, sphereOK := orientedSpherePoint(sphere, outward)
	if !boxOK || !sphereOK || boxPoint.BoundMM > pointResolutionMM ||
		spherePoint.BoundMM > pointResolutionMM {
		result.Reason = pair.PointTooCoarse
		return result
	}
	separation, valid := orientedSphereSignedReading(faceDistance2, sphere.Radius)
	if !valid {
		result.Reason = pair.PointTooCoarse
		return result
	}
	result.Witness = &OrientedSphereWitness{Face: FaceSlot{Axis: axis, Side: side},
		SpherePoint: spherePoint, BoxPoint: boxPoint, Normal: normal, Separation: separation}
	return result
}

func orientedSphereSignedReading(distance2 *big.Rat, radius proof.Dyadic) (pair.ScalarReading, bool) {
	lower, upper := proofbound.RatSqrtDown(distance2), proofbound.RatSqrtUp(distance2)
	if !finite(lower) || !finite(upper) {
		return pair.ScalarReading{}, false
	}
	low := new(big.Rat).Sub(proof.FloatRat(lower), radius.Rat())
	high := new(big.Rat).Sub(proof.FloatRat(upper), radius.Rat())
	value := sweeppath.RatFloatNearest(new(big.Rat).Quo(new(big.Rat).Add(low, high), big.NewRat(2, 1)))
	bound := proofbound.RatFloatUp(proofbound.RatMax(new(big.Rat).Sub(proof.FloatRat(value), low),
		new(big.Rat).Sub(high, proof.FloatRat(value))))
	if !finite(value) || !finite(bound) || bound < 0 {
		return pair.ScalarReading{}, false
	}
	return pair.ScalarReading{ValueMM: value, BoundMM: bound}, true
}

func orientedSpherePoint(sphere AxisSphere, outward proof.DyV3) (PointReading, bool) {
	squared := proof.DvDot(outward, outward).Rat()
	low, high := proofbound.RatSqrtDown(squared), proofbound.RatSqrtUp(squared)
	if low <= 0 || !finite(low) || !finite(high) {
		return PointReading{}, false
	}
	var value [3]float64
	maxError := new(big.Rat)
	for k := range 3 {
		a := new(big.Rat).Quo(outward[k].Rat(), proof.FloatRat(low))
		b := new(big.Rat).Quo(outward[k].Rat(), proof.FloatRat(high))
		first := new(big.Rat).Sub(sphere.Center[k].Rat(), new(big.Rat).Mul(sphere.Radius.Rat(), a))
		second := new(big.Rat).Sub(sphere.Center[k].Rat(), new(big.Rat).Mul(sphere.Radius.Rat(), b))
		value[k] = sweeppath.RatFloatNearest(new(big.Rat).Quo(new(big.Rat).Add(first, second), big.NewRat(2, 1)))
		if !finite(value[k]) {
			return PointReading{}, false
		}
		for _, endpoint := range []*big.Rat{first, second} {
			deviation := new(big.Rat).Sub(endpoint, proof.FloatRat(value[k]))
			deviation.Abs(deviation)
			if deviation.Cmp(maxError) > 0 {
				maxError = deviation
			}
		}
	}
	bound := proofbound.Radius3D(proofbound.RatFloatUp(maxError))
	if !finite(bound) {
		return PointReading{}, false
	}
	return PointReading{Value: r3.Vec{X: value[0], Y: value[1], Z: value[2]}, BoundMM: bound}, true
}
