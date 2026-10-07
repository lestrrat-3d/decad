package decad

import (
	"math/big"

	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func closestOrientedBoxPoint(center proofarith.DyV3, box orientedSourceBox) (*big.Rat, bool) {
	return pairbox.ClosestOrientedBoxPoint(center, box.pairBox())
}

func orientedSphereFace(sphere sourceSphereContactProof, box orientedSourceBox) (
	int, int, proofarith.DyV3, [3]*big.Rat, *big.Rat, bool) {
	return pairbox.OrientedSphereFace(sphere.center, sphere.radius, box.pairBox())
}

func orientedDual(box orientedSourceBox, axis int) proofarith.DyV3 {
	return pairbox.OrientedDual(box.pairBox(), axis)
}

func orientedDenominator(box orientedSourceBox, axis int) proofarith.Dyadic {
	return pairbox.OrientedDenominator(box.pairBox(), axis)
}

func classifySourceSphereOrientedBox(report *ContactReport, sphere sourceSphereContactProof,
	box orientedSourceBox, sphereFirst bool) {
	if !orthogonalSourceBox(box) {
		report.Reason = ContactPayloadUnsupported
		return
	}
	distance2, ok := closestOrientedBoxPoint(sphere.center, box)
	if !ok {
		report.Reason = ContactPayloadUnsupported
		return
	}
	radius2 := proofarith.DyMul(sphere.radius, sphere.radius).Rat()
	switch distance2.Cmp(radius2) {
	case 1:
		gap, valid := orientedSphereSignedReading(distance2, sphere.radius)
		if !valid || gap.Value.Base()-gap.Bound.Base() <= 0 {
			report.Reason = ContactNoGapProof
			return
		}
		report.Relation, report.Gap = ContactSeparated, &gap
		return
	case 0:
		report.Relation = ContactTouching
		report.Gap = &Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: Exact}
	default:
		report.Relation = ContactOverlapping
	}
	axis, side, outward, coordinate, faceDistance2, faceOK := orientedSphereFace(sphere, box)
	if !faceOK || faceDistance2.Cmp(distance2) != 0 {
		report.Reason = ContactAmbiguousFeature
		return
	}
	// A shallow overlap must still leave the opposite face outside the ball.
	opposite := new(big.Rat).Set(coordinate[axis])
	if side == 1 {
		opposite = new(big.Rat).Mul(coordinate[axis], orientedDenominator(box, axis).Rat())
	} else {
		opposite.Sub(big.NewRat(1, 1), opposite)
		opposite.Mul(opposite, orientedDenominator(box, axis).Rat())
	}
	if new(big.Rat).Mul(opposite, opposite).Cmp(new(big.Rat).Mul(radius2,
		proofarith.DvDot(outward, outward).Rat())) <= 0 {
		report.Reason = ContactAmbiguousFeature
		return
	}
	face := box.faces[axis][side]
	if face == nil {
		report.Reason = ContactNoNormalProof
		return
	}
	normalAxis := outward
	if sphereFirst {
		for k := range 3 {
			normalAxis[k] = proofarith.DyNeg(normalAxis[k])
		}
	}
	normal, angle, ok := orientedBoxNormal(normalAxis)
	if !ok || angle.Base() > report.Request.NormalResolution.Base() {
		report.Reason = ContactNoNormalProof
		return
	}
	foot := [3]*big.Rat{}
	for k := range 3 {
		foot[k] = box.corner[0][k].Rat()
		for edge := range 3 {
			value := coordinate[edge]
			if edge == axis {
				value = big.NewRat(int64(side), 1)
			}
			foot[k].Add(foot[k], new(big.Rat).Mul(value, box.edge[edge][k].Rat()))
		}
	}
	boxPoint, boxOK := orientedBoxPoint(foot)
	spherePoint, sphereOK := orientedSpherePoint(sphere, outward)
	if !boxOK || !sphereOK || boxPoint.Bound.Base() > report.Request.PointResolution.Base() ||
		spherePoint.Bound.Base() > report.Request.PointResolution.Base() {
		report.Reason = ContactPointTooCoarse
		return
	}
	separation, valid := orientedSphereSignedReading(faceDistance2, sphere.radius)
	if !valid {
		report.Reason = ContactPointTooCoarse
		return
	}
	point := ContactPoint{Normal: normal, NormalAngle: angle, Separation: separation}
	if sphereFirst {
		point.OnA, point.OnB = spherePoint, boxPoint
		point.FeatureA, point.FeatureB = ContactFeature{Face: sphere.face}, ContactFeature{Face: face}
	} else {
		point.OnA, point.OnB = boxPoint, spherePoint
		point.FeatureA, point.FeatureB = ContactFeature{Face: face}, ContactFeature{Face: sphere.face}
	}
	point.FaceA, point.FaceB = point.FeatureA.Face, point.FeatureB.Face
	report.Manifold = &ContactManifold{Points: []ContactPoint{point}}
	report.Reason = ContactNoReason
}

func orthogonalSourceBox(box orientedSourceBox) bool {
	return pairbox.OrthogonalSourceBox(box.pairBox())
}

func orientedSphereSignedReading(distance2 *big.Rat, radius proofarith.Dyadic) (Measurement, bool) {
	lower, upper := proofbound.RatSqrtDown(distance2), proofbound.RatSqrtUp(distance2)
	if !finiteMeasurementValues(lower, upper) {
		return Measurement{}, false
	}
	low := new(big.Rat).Sub(proofarith.FloatRat(lower), radius.Rat())
	high := new(big.Rat).Sub(proofarith.FloatRat(upper), radius.Rat())
	value := ratFloatNearest(new(big.Rat).Quo(new(big.Rat).Add(low, high), big.NewRat(2, 1)))
	bound := proofbound.RatFloatUp(proofbound.RatMax(new(big.Rat).Sub(proofarith.FloatRat(value), low),
		new(big.Rat).Sub(high, proofarith.FloatRat(value))))
	if !finiteMeasurementValues(value, bound) || bound < 0 {
		return Measurement{}, false
	}
	return Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
		Exactness: exactnessOf(bound)}, true
}

func orientedSpherePoint(sphere sourceSphereContactProof, outward proofarith.DyV3) (VecMeasurement, bool) {
	squared := proofarith.DvDot(outward, outward).Rat()
	low, high := proofbound.RatSqrtDown(squared), proofbound.RatSqrtUp(squared)
	if low <= 0 || !finiteMeasurementValues(low, high) {
		return VecMeasurement{}, false
	}
	var value [3]float64
	maxError := new(big.Rat)
	for k := range 3 {
		a := new(big.Rat).Quo(outward[k].Rat(), proofarith.FloatRat(low))
		b := new(big.Rat).Quo(outward[k].Rat(), proofarith.FloatRat(high))
		first := new(big.Rat).Sub(sphere.center[k].Rat(), new(big.Rat).Mul(sphere.radius.Rat(), a))
		second := new(big.Rat).Sub(sphere.center[k].Rat(), new(big.Rat).Mul(sphere.radius.Rat(), b))
		value[k] = ratFloatNearest(new(big.Rat).Quo(new(big.Rat).Add(first, second), big.NewRat(2, 1)))
		if !finiteMeasurementValues(value[k]) {
			return VecMeasurement{}, false
		}
		for _, endpoint := range []*big.Rat{first, second} {
			deviation := new(big.Rat).Sub(endpoint, proofarith.FloatRat(value[k]))
			deviation.Abs(deviation)
			if deviation.Cmp(maxError) > 0 {
				maxError = deviation
			}
		}
	}
	bound := proofbound.Radius3D(proofbound.RatFloatUp(maxError))
	if !finiteMeasurementValues(bound) {
		return VecMeasurement{}, false
	}
	return VecMeasurement{Value: r3.Vec{X: value[0], Y: value[1], Z: value[2]},
		Bound: units.Millimeters(bound), Exactness: exactnessOf(bound)}, true
}
