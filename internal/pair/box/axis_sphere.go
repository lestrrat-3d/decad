package box

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// AxisSphere is a complete occupied ball admitted by its source record.
type AxisSphere struct {
	Center proof.DyV3
	Radius proof.Dyadic
}

// AxisSphereWitness identifies the supporting box face and both bounded points.
type AxisSphereWitness struct {
	Face                   FaceSlot
	NormalAxis, NormalSign int
	SpherePoint, BoxPoint  PointReading
	Separation             pair.ScalarReading
}

// AxisSphereResult carries the exact occupied-set relation and optional face witness.
type AxisSphereResult struct {
	Relation pair.Relation
	Reason   pair.Reason
	Gap      *pair.ScalarReading
	Witness  *AxisSphereWitness
}

// ClassifyAxisSphere compares the complete ball with the complete axis box.
// A witness requires one exterior support and a projected disk strictly inside
// the other two face intervals. The caller binds FaceSlot to original topology.
func ClassifyAxisSphere(sphere AxisSphere, box AxisBox, pointResolutionMM float64) AxisSphereResult {
	var nearest proof.DyV3
	distance2 := proof.DyZero()
	outsideAxis, outsideSide, outsideCount := 0, 0, 0
	for i := range 3 {
		nearest[i] = dyMax(box.Lo[i], dyMin(sphere.Center[i], box.Hi[i]))
		d := proof.DySubScalar(sphere.Center[i], nearest[i])
		distance2 = proof.DyAdd(distance2, proof.DyMul(d, d))
		if d.Sign() != 0 {
			outsideAxis, outsideCount = i, outsideCount+1
			if d.Sign() > 0 {
				outsideSide = 1
			}
		}
	}
	radius2 := proof.DyMul(sphere.Radius, sphere.Radius)
	result := AxisSphereResult{}
	switch proof.DyCmp(distance2, radius2) {
	case 1:
		gap, ok := axisSphereGap(distance2, sphere.Radius)
		if !ok {
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
	if outsideCount != 1 {
		result.Reason = pair.AmbiguousFeature
		return result
	}
	axis := outsideAxis
	for i := range 3 {
		if i == axis {
			continue
		}
		if proof.DyCmp(proof.DySubScalar(sphere.Center[i], sphere.Radius), box.Lo[i]) <= 0 ||
			proof.DyCmp(proof.DyAdd(sphere.Center[i], sphere.Radius), box.Hi[i]) >= 0 {
			result.Reason = pair.AmbiguousFeature
			return result
		}
	}
	if result.Relation == pair.Overlapping {
		opposite := box.Lo[axis]
		if outsideSide == 0 {
			opposite = box.Hi[axis]
		}
		if proof.DyCmp(proof.DyAbs(proof.DySubScalar(sphere.Center[axis], opposite)), sphere.Radius) <= 0 {
			result.Reason = pair.AmbiguousFeature
			return result
		}
	}
	sideSign := -1
	if outsideSide == 1 {
		sideSign = 1
	}
	witnessSphere := sphere.Center
	witnessSphere[axis] = proof.DySubScalar(sphere.Center[axis],
		proof.DyMul(proof.MustDyOf(float64(sideSign)), sphere.Radius))
	onSphere, okSphere := ReadPointAt(&witnessSphere)
	onBox, okBox := ReadPointAt(&nearest)
	if !okSphere || !okBox || onSphere.BoundMM > pointResolutionMM || onBox.BoundMM > pointResolutionMM {
		result.Reason = pair.PointTooCoarse
		return result
	}
	distance := proof.DyAbs(proof.DySubScalar(sphere.Center[axis], nearest[axis]))
	separation, ok := SignedReading(proof.DySubScalar(distance, sphere.Radius))
	if !ok {
		result.Reason = pair.PointTooCoarse
		return result
	}
	result.Witness = &AxisSphereWitness{
		Face: FaceSlot{Axis: axis, Side: outsideSide}, NormalAxis: axis, NormalSign: sideSign,
		SpherePoint: onSphere, BoxPoint: onBox, Separation: separation,
	}
	return result
}

func axisSphereGap(distance2, radius proof.Dyadic) (pair.ScalarReading, bool) {
	lo := proofbound.RatFloatDown(new(big.Rat).Sub(proof.FloatRat(proof.DySqrtDown(distance2)), radius.Rat()))
	hi := proofbound.RatFloatUp(new(big.Rat).Sub(proof.FloatRat(proof.DySqrtUp(distance2)), radius.Rat()))
	if !finite(lo) || !finite(hi) || lo <= 0 || hi < lo {
		return pair.ScalarReading{}, false
	}
	value := lo + (hi-lo)/2
	left := new(big.Rat).Sub(proof.FloatRat(value), proof.FloatRat(lo))
	right := new(big.Rat).Sub(proof.FloatRat(hi), proof.FloatRat(value))
	if right.Cmp(left) > 0 {
		left = right
	}
	bound := proofbound.RatFloatUp(left)
	if !finite(value) || !finite(bound) ||
		new(big.Rat).Sub(proof.FloatRat(value), proof.FloatRat(bound)).Sign() <= 0 {
		return pair.ScalarReading{}, false
	}
	return pair.ScalarReading{ValueMM: value, BoundMM: bound}, true
}
