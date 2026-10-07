package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// closestOrientedBoxPoint minimizes squared Euclidean distance over the exact
// parallelotope. Each coordinate is free or fixed at either endpoint; the
// twenty-seven resulting stationary candidates cover every face, edge and
// vertex of the compact convex box.
func closestOrientedBoxPoint(center proofarith.DyV3, box orientedSourceBox) (*big.Rat, bool) {
	var best *big.Rat
	for code := range 27 {
		status, value := code, [3]*big.Rat{}
		var free [3]int
		count := 0
		point := [3]*big.Rat{}
		for k := range 3 {
			point[k] = box.corner[0][k].Rat()
		}
		for axis := range 3 {
			side := status % 3
			status /= 3
			if side == 2 {
				free[count], count = axis, count+1
				continue
			}
			value[axis] = big.NewRat(int64(side), 1)
			if side == 1 {
				for k := range 3 {
					point[k].Add(point[k], box.edge[axis][k].Rat())
				}
			}
		}
		if count > 0 {
			matrix := make([][]*big.Rat, count)
			for i := range count {
				matrix[i] = make([]*big.Rat, count+1)
				for j := range count {
					matrix[i][j] = proofarith.DvDot(box.edge[free[i]], box.edge[free[j]]).Rat()
				}
				matrix[i][count] = new(big.Rat)
				for k := range 3 {
					term := new(big.Rat).Sub(center[k].Rat(), point[k])
					matrix[i][count].Add(matrix[i][count],
						new(big.Rat).Mul(box.edge[free[i]][k].Rat(), term))
				}
			}
			if !solvePositiveGram(matrix) {
				return nil, false
			}
			valid := true
			for i := range count {
				candidate := matrix[i][count]
				if candidate.Sign() < 0 || candidate.Cmp(big.NewRat(1, 1)) > 0 {
					valid = false
					break
				}
				value[free[i]] = candidate
			}
			if !valid {
				continue
			}
		}
		distance2 := new(big.Rat)
		for k := range 3 {
			coordinate := box.corner[0][k].Rat()
			for axis := range 3 {
				coordinate.Add(coordinate, new(big.Rat).Mul(value[axis], box.edge[axis][k].Rat()))
			}
			delta := new(big.Rat).Sub(center[k].Rat(), coordinate)
			distance2.Add(distance2, new(big.Rat).Mul(delta, delta))
		}
		if best == nil || distance2.Cmp(best) < 0 {
			best = distance2
		}
	}
	return best, best != nil
}

func solvePositiveGram(matrix [][]*big.Rat) bool {
	n := len(matrix)
	for pivot := range n {
		if matrix[pivot][pivot].Sign() <= 0 {
			return false
		}
		denominator := new(big.Rat).Set(matrix[pivot][pivot])
		for column := pivot; column <= n; column++ {
			matrix[pivot][column] = new(big.Rat).Quo(matrix[pivot][column], denominator)
		}
		for row := pivot + 1; row < n; row++ {
			factor := new(big.Rat).Set(matrix[row][pivot])
			for column := pivot; column <= n; column++ {
				matrix[row][column] = new(big.Rat).Sub(matrix[row][column],
					new(big.Rat).Mul(factor, matrix[pivot][column]))
			}
		}
	}
	for row := n - 1; row >= 0; row-- {
		for column := row + 1; column < n; column++ {
			matrix[row][n] = new(big.Rat).Sub(matrix[row][n],
				new(big.Rat).Mul(matrix[row][column], matrix[column][n]))
		}
	}
	return true
}

// orientedSphereFace identifies one isolated exterior support. The strict
// dual-coordinate margins hold the entire projected ball inside the face.
func orientedSphereFace(sphere sourceSphereContactProof, box orientedSourceBox) (
	int, int, proofarith.DyV3, [3]*big.Rat, *big.Rat, bool) {
	for axis := range 3 {
		i, j := (axis+1)%3, (axis+2)%3
		dual := proofarith.DvCross(box.edge[i], box.edge[j])
		denom := proofarith.DvDot(box.edge[axis], dual)
		if denom.Sign() == 0 {
			return 0, 0, proofarith.DyV3{}, [3]*big.Rat{}, nil, false
		}
		if denom.Sign() < 0 {
			for k := range 3 {
				dual[k] = proofarith.DyNeg(dual[k])
			}
			denom = proofarith.DyNeg(denom)
		}
		var coordinate [3]*big.Rat
		for k := range 3 {
			denominator := orientedDenominator(box, k).Rat()
			denominator.Abs(denominator)
			coordinate[k] = new(big.Rat).Quo(proofarith.DvDot(proofarith.DvSub(sphere.center, box.corner[0]),
				orientedDual(box, k)).Rat(), denominator)
		}
		side := -1
		if coordinate[axis].Sign() < 0 {
			side = 0
		} else if coordinate[axis].Cmp(big.NewRat(1, 1)) > 0 {
			side = 1
		}
		if side < 0 {
			continue
		}
		norm2 := proofarith.DvDot(dual, dual).Rat()
		plane := new(big.Rat).Mul(coordinate[axis], denom.Rat())
		if side == 1 {
			plane.Sub(plane, denom.Rat())
		} else {
			plane.Neg(plane)
		}
		if plane.Sign() <= 0 {
			continue
		}
		for k := range 3 {
			if k != axis && !orientedSphereMargin(sphere.radius, box, k, coordinate[k]) {
				return 0, 0, proofarith.DyV3{}, [3]*big.Rat{}, nil, false
			}
		}
		if side == 0 {
			for k := range 3 {
				dual[k] = proofarith.DyNeg(dual[k])
			}
		}
		planeDistance2 := new(big.Rat).Quo(new(big.Rat).Mul(plane, plane), norm2)
		return axis, side, dual, coordinate, planeDistance2, true
	}
	return 0, 0, proofarith.DyV3{}, [3]*big.Rat{}, nil, false
}

func orientedDual(box orientedSourceBox, axis int) proofarith.DyV3 {
	i, j := (axis+1)%3, (axis+2)%3
	dual := proofarith.DvCross(box.edge[i], box.edge[j])
	if orientedDenominator(box, axis).Sign() < 0 {
		for k := range 3 {
			dual[k] = proofarith.DyNeg(dual[k])
		}
	}
	return dual
}

func orientedDenominator(box orientedSourceBox, axis int) proofarith.Dyadic {
	i, j := (axis+1)%3, (axis+2)%3
	return proofarith.DvDot(box.edge[axis], proofarith.DvCross(box.edge[i], box.edge[j]))
}

func orientedSphereMargin(radius proofarith.Dyadic, box orientedSourceBox, axis int, coordinate *big.Rat) bool {
	if coordinate.Sign() <= 0 || coordinate.Cmp(big.NewRat(1, 1)) >= 0 {
		return false
	}
	dual := orientedDual(box, axis)
	denom := orientedDenominator(box, axis).Rat()
	margin := new(big.Rat).Set(coordinate)
	remaining := new(big.Rat).Sub(big.NewRat(1, 1), coordinate)
	if remaining.Cmp(margin) < 0 {
		margin = remaining
	}
	left := new(big.Rat).Mul(margin, margin)
	left.Mul(left, new(big.Rat).Mul(denom, denom))
	right := new(big.Rat).Mul(radius.Rat(), radius.Rat())
	right.Mul(right, proofarith.DvDot(dual, dual).Rat())
	return left.Cmp(right) > 0
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
	for i := range 3 {
		for j := i + 1; j < 3; j++ {
			if !proofarith.DvDot(box.edge[i], box.edge[j]).IsZero() {
				return false
			}
		}
	}
	return true
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
