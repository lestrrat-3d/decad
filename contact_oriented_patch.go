package decad

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// publishOrientedBoxPatch handles opposed faces of boxes with parallel source
// edge directions. Their complete contact set is a rectangle in A's exact
// dual basis, even when the read rotation's dyadic entries are slightly skew.
func publishOrientedBoxPatch(report *ContactReport, a, b orientedSourceBox) {
	var dual [3]proofarith.DyV3
	var denominator [3]*big.Rat
	var bAxis [3]int
	for axis := range 3 {
		i, j := (axis+1)%3, (axis+2)%3
		dual[axis] = proofarith.DvCross(a.edge[i], a.edge[j])
		denominator[axis] = proofarith.DvDot(a.edge[axis], dual[axis]).Rat()
		if denominator[axis].Sign() == 0 {
			report.Reason = ContactNoNormalProof
			return
		}
		if denominator[axis].Sign() < 0 {
			for k := range 3 {
				dual[axis][k] = proofarith.DyNeg(dual[axis][k])
			}
			denominator[axis].Neg(denominator[axis])
		}
		bAxis[axis] = -1
		for candidate := range 3 {
			if !proofarith.DvIsZero(proofarith.DvCross(a.edge[axis], b.edge[candidate])) {
				continue
			}
			if bAxis[axis] >= 0 {
				report.Reason = ContactAmbiguousFeature
				return
			}
			bAxis[axis] = candidate
		}
		if bAxis[axis] < 0 {
			report.Reason = ContactNoNormalProof
			return
		}
	}
	if bAxis[0] == bAxis[1] || bAxis[1] == bAxis[2] || bAxis[0] == bAxis[2] {
		report.Reason = ContactAmbiguousFeature
		return
	}
	var alo, ahi, blo, bhi [3]*big.Rat
	for axis := range 3 {
		alo[axis], ahi[axis] = big.NewRat(0, 1), big.NewRat(1, 1)
		for _, corner := range b.corner {
			coordinate := new(big.Rat).Quo(proofarith.DvDot(proofarith.DvSub(corner, a.corner[0]), dual[axis]).Rat(),
				denominator[axis])
			if blo[axis] == nil || coordinate.Cmp(blo[axis]) < 0 {
				blo[axis] = coordinate
			}
			if bhi[axis] == nil || coordinate.Cmp(bhi[axis]) > 0 {
				bhi[axis] = coordinate
			}
		}
	}
	contactAxis, sign := -1, 0
	for axis := range 3 {
		switch {
		case ahi[axis].Cmp(blo[axis]) == 0:
			if contactAxis >= 0 {
				report.Reason = ContactAmbiguousFeature
				return
			}
			contactAxis, sign = axis, 1
		case bhi[axis].Cmp(alo[axis]) == 0:
			if contactAxis >= 0 {
				report.Reason = ContactAmbiguousFeature
				return
			}
			contactAxis, sign = axis, -1
		default:
			if alo[axis].Cmp(bhi[axis]) >= 0 || blo[axis].Cmp(ahi[axis]) >= 0 {
				report.Reason = ContactAmbiguousFeature
				return
			}
		}
	}
	if contactAxis < 0 {
		report.Reason = ContactAmbiguousFeature
		return
	}
	var limits [3][2]*big.Rat
	var tangents [2]int
	n := 0
	for axis := range 3 {
		if axis == contactAxis {
			value := alo[axis]
			if sign > 0 {
				value = ahi[axis]
			}
			limits[axis] = [2]*big.Rat{value, value}
			continue
		}
		limits[axis] = [2]*big.Rat{survey2d.RatMax(alo[axis], blo[axis]), survey2d.RatMin(ahi[axis], bhi[axis])}
		if limits[axis][0].Cmp(limits[axis][1]) >= 0 {
			report.Reason = ContactAmbiguousFeature
			return
		}
		tangents[n] = axis
		n++
	}
	aSide := 0
	if sign > 0 {
		aSide = 1
	}
	bSide := 0
	// Copy the contact axis into a named local before the call below, rather
	// than reading dual[contactAxis] on both sides of it: Go's compiler
	// (CSE of OpLocalAddr, Go 1.24 through 1.27) reuses the element pointer
	// it took for the first use after the call, while dual is no longer live
	// and is not a stack object, so the collector frees the *big.Int
	// mantissas that only dual reached.
	normalAxis := dual[contactAxis]
	if proofarith.DvDot(b.edge[bAxis[contactAxis]], normalAxis).Sign()*sign > 0 {
		bSide = 0
	} else {
		bSide = 1
	}
	faceA, faceB := a.faces[contactAxis][aSide], b.faces[bAxis[contactAxis]][bSide]
	if faceA == nil || faceB == nil {
		report.Reason = ContactNoNormalProof
		return
	}
	if sign < 0 {
		for k := range 3 {
			normalAxis[k] = proofarith.DyNeg(normalAxis[k])
		}
	}
	normal, angle, ok := orientedBoxNormal(normalAxis)
	if !ok || angle.Base() > report.Request.NormalResolution.Base() {
		report.Reason = ContactNoNormalProof
		return
	}
	points := make([]ContactPoint, 0, 4)
	for _, corner := range [][2]int{{0, 0}, {1, 0}, {1, 1}, {0, 1}} {
		coordinates := [3]*big.Rat{}
		for axis := range 3 {
			coordinates[axis] = limits[axis][0]
		}
		for i, axis := range tangents {
			coordinates[axis] = limits[axis][corner[i]]
		}
		var point [3]*big.Rat
		for k := range 3 {
			point[k] = a.corner[0][k].Rat()
			for axis := range 3 {
				point[k] = new(big.Rat).Add(point[k],
					new(big.Rat).Mul(a.edge[axis][k].Rat(), coordinates[axis]))
			}
		}
		witness, valid := orientedBoxPoint(point)
		if !valid || witness.Bound.Base() > report.Request.PointResolution.Base() {
			report.Reason = ContactPointTooCoarse
			return
		}
		points = append(points, ContactPoint{OnA: witness, OnB: witness, Normal: normal,
			NormalAngle: angle, Separation: Measurement{Value: units.Millimeters(0),
				Bound: units.Millimeters(0), Exactness: Exact}, FaceA: faceA, FaceB: faceB,
			FeatureA: ContactFeature{Face: faceA}, FeatureB: ContactFeature{Face: faceB}})
	}
	report.Manifold = &ContactManifold{Points: points}
	report.Reason = ContactNoReason
}

func orientedBoxPoint(point [3]*big.Rat) (VecMeasurement, bool) {
	var coordinate [3]float64
	maxError := new(big.Rat)
	for axis := range 3 {
		coordinate[axis] = ratFloatNearest(point[axis])
		if !finiteMeasurementValues(coordinate[axis]) {
			return VecMeasurement{}, false
		}
		deviation := new(big.Rat).Sub(point[axis], proofarith.FloatRat(coordinate[axis]))
		deviation.Abs(deviation)
		if deviation.Cmp(maxError) > 0 {
			maxError = deviation
		}
	}
	bound := proofbound.Radius3D(proofbound.RatFloatUp(maxError))
	if !finiteMeasurementValues(bound) {
		return VecMeasurement{}, false
	}
	return VecMeasurement{Value: r3.Vec{X: coordinate[0], Y: coordinate[1], Z: coordinate[2]},
		Bound: units.Millimeters(bound), Exactness: exactnessOf(bound)}, true
}

func orientedBoxNormal(axis proofarith.DyV3) (VecMeasurement, units.Value, bool) {
	squared := proofarith.DvDot(axis, axis).Rat()
	if squared.Sign() <= 0 {
		return VecMeasurement{}, units.Value{}, false
	}
	low, high := proofbound.RatSqrtDown(squared), proofbound.RatSqrtUp(squared)
	if low <= 0 || !finiteMeasurementValues(low, high) {
		return VecMeasurement{}, units.Value{}, false
	}
	raw := r3.Vec{}
	raw.X, _ = axis[0].Float64()
	raw.Y, _ = axis[1].Float64()
	raw.Z, _ = axis[2].Float64()
	value, ok := raw.Normalize()
	if !ok || !proofbound.FiniteVec(value) {
		return VecMeasurement{}, units.Value{}, false
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
	if !finiteMeasurementValues(bound, angle) || angle >= math.Pi {
		return VecMeasurement{}, units.Value{}, false
	}
	return VecMeasurement{Value: value, Bound: units.Scalar(bound),
		Exactness: exactnessOf(bound)}, units.Radians(angle), true
}
