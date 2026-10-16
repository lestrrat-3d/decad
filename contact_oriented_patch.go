package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/units"
)

// publishOrientedBoxPatch handles opposed faces of boxes with parallel source
// edge directions. Their complete contact set is a rectangle in A's exact
// dual basis, even when the read rotation's dyadic entries are slightly skew.
func publishOrientedBoxPatch(report *reportvocab.ContactReport, a, b orientedSourceBox) {
	geometry, failure := pairbox.OrientedFacePatch(a.OrientedBox, b.OrientedBox)
	switch failure {
	case pairbox.OrientedPatchNoNormal:
		report.Reason = reportvocab.ContactNoNormalProof
		return
	case pairbox.OrientedPatchAmbiguous:
		report.Reason = reportvocab.ContactAmbiguousFeature
		return
	}
	faceA := a.faces[geometry.FaceA.Axis][geometry.FaceA.Side]
	faceB := b.faces[geometry.FaceB.Axis][geometry.FaceB.Side]
	if faceA == nil || faceB == nil {
		report.Reason = reportvocab.ContactNoNormalProof
		return
	}
	normal, angle, ok := orientedBoxNormal(geometry.Normal)
	if !ok || angle.Base() > report.Request.NormalResolution.Base() {
		report.Reason = reportvocab.ContactNoNormalProof
		return
	}
	points := make([]reportvocab.ContactPoint, 0, 4)
	for _, corner := range [][2]int{{0, 0}, {1, 0}, {1, 1}, {0, 1}} {
		point := geometry.Point(a.OrientedBox, corner)
		witness, valid := orientedBoxPoint(point)
		onBoth := vecMeasurementToInternal(witness)
		if !valid || witness.Bound.Base() > report.Request.PointResolution.Base() {
			report.Reason = reportvocab.ContactPointTooCoarse
			return
		}
		points = append(points, reportvocab.ContactPoint{OnA: onBoth, OnB: onBoth, Normal: normal,
			NormalAngle: angle, Separation: measurement.Measurement{Value: units.Millimeters(0),
				Bound: units.Millimeters(0), Exactness: measurement.Exact}, FaceA: faceNode(faceA), FaceB: faceNode(faceB),
			FeatureA: reportvocab.ContactFeature{Face: faceNode(faceA)}, FeatureB: reportvocab.ContactFeature{Face: faceNode(faceB)}})
	}
	report.Manifold = &reportvocab.ContactManifold{Points: points}
	report.Reason = reportvocab.ContactNoReason
}

func orientedBoxPoint(point [3]*big.Rat) (VecMeasurement, bool) {
	reading, ok := pairbox.RationalPointReading(point)
	if !ok {
		return VecMeasurement{}, false
	}
	return VecMeasurement{Value: reading.Value, Bound: units.Millimeters(reading.BoundMM),
		Exactness: exactnessOf(reading.BoundMM)}, true
}

func orientedBoxNormal(axis proofarith.DyV3) (measurement.VecMeasurement, units.Value, bool) {
	reading, ok := pairbox.DyadicNormalReading(axis)
	if !ok {
		return measurement.VecMeasurement{}, units.Value{}, false
	}
	return measurement.VecMeasurement{Value: reading.Value, Bound: units.Scalar(reading.Bound),
		Exactness: measurement.ExactnessOf(reading.Bound)}, units.Radians(reading.Angle), true
}
