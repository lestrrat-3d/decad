package decad

import (
	"math/big"

	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/units"
)

// publishOrientedBoxPatch handles opposed faces of boxes with parallel source
// edge directions. Their complete contact set is a rectangle in A's exact
// dual basis, even when the read rotation's dyadic entries are slightly skew.
func publishOrientedBoxPatch(report *ContactReport, a, b orientedSourceBox) {
	geometry, failure := pairbox.OrientedFacePatch(a.pairBox(), b.pairBox())
	switch failure {
	case pairbox.OrientedPatchNoNormal:
		report.Reason = ContactNoNormalProof
		return
	case pairbox.OrientedPatchAmbiguous:
		report.Reason = ContactAmbiguousFeature
		return
	}
	faceA := a.faces[geometry.FaceA.Axis][geometry.FaceA.Side]
	faceB := b.faces[geometry.FaceB.Axis][geometry.FaceB.Side]
	if faceA == nil || faceB == nil {
		report.Reason = ContactNoNormalProof
		return
	}
	normal, angle, ok := orientedBoxNormal(geometry.Normal)
	if !ok || angle.Base() > report.Request.NormalResolution.Base() {
		report.Reason = ContactNoNormalProof
		return
	}
	points := make([]ContactPoint, 0, 4)
	for _, corner := range [][2]int{{0, 0}, {1, 0}, {1, 1}, {0, 1}} {
		point := geometry.Point(a.pairBox(), corner)
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
	reading, ok := pairbox.RationalPointReading(point)
	if !ok {
		return VecMeasurement{}, false
	}
	return VecMeasurement{Value: reading.Value, Bound: units.Millimeters(reading.BoundMM),
		Exactness: exactnessOf(reading.BoundMM)}, true
}

func orientedBoxNormal(axis proofarith.DyV3) (VecMeasurement, units.Value, bool) {
	reading, ok := pairbox.DyadicNormalReading(axis)
	if !ok {
		return VecMeasurement{}, units.Value{}, false
	}
	return VecMeasurement{Value: reading.Value, Bound: units.Scalar(reading.Bound),
		Exactness: exactnessOf(reading.Bound)}, units.Radians(reading.Angle), true
}
