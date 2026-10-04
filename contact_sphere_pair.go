package decad

import (
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// classifySourceSpherePair compares the complete occupied balls. A response
// witness needs a unique cardinal center axis and two crossing sphere faces.
func classifySourceSpherePair(report *ContactReport, a, b sourceSphereContactProof) {
	var delta dyV3
	distance2 := dyZero()
	axis, nonzero := 0, 0
	for i := range 3 {
		delta[i] = dySubScalar(b.center[i], a.center[i])
		distance2 = dyAdd(distance2, dyMul(delta[i], delta[i]))
		if delta[i].sign() != 0 {
			axis, nonzero = i, nonzero+1
		}
	}
	radius := dyAdd(a.radius, b.radius)
	radius2 := dyMul(radius, radius)
	switch dyCmp(distance2, radius2) {
	case 1:
		lower := new(big.Rat).Sub(floatRat(dySqrtDown(distance2)), radius.rat())
		upper := new(big.Rat).Sub(floatRat(dySqrtUp(distance2)), radius.rat())
		lo, hi := ratFloatDown(lower), ratFloatUp(upper)
		if !finiteMeasurementValues(lo, hi) || lo <= 0 || hi < lo {
			report.Reason = ContactNoGapProof
			return
		}
		value := lo + (hi-lo)/2
		left := new(big.Rat).Sub(floatRat(value), floatRat(lo))
		right := new(big.Rat).Sub(floatRat(hi), floatRat(value))
		if right.Cmp(left) > 0 {
			left = right
		}
		bound := ratFloatUp(left)
		if !finiteMeasurementValues(value, bound) ||
			new(big.Rat).Sub(floatRat(value), floatRat(bound)).Sign() <= 0 {
			report.Reason = ContactNoGapProof
			return
		}
		gap := Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
			Exactness: exactnessOf(bound)}
		report.Relation, report.Gap = ContactSeparated, &gap
		return
	case 0:
		report.Relation = ContactTouching
		gap := Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: Exact}
		report.Gap = &gap
	default:
		report.Relation = ContactOverlapping
	}
	if nonzero != 1 {
		report.Reason = ContactAmbiguousFeature
		return
	}
	distance := dyAbs(delta[axis])
	if dyCmp(distance, dyAbs(dySubScalar(a.radius, b.radius))) <= 0 {
		report.Reason = ContactAmbiguousFeature
		return
	}
	sign := 1.0
	if delta[axis].sign() < 0 {
		sign = -1
	}
	normal := r3.Vec{}
	switch axis {
	case 0:
		normal.X = sign
	case 1:
		normal.Y = sign
	case 2:
		normal.Z = sign
	}
	onAExact, onBExact := a.center, b.center
	onAExact[axis] = dyAdd(a.center[axis], dyMul(mustDyOf(sign), a.radius))
	onBExact[axis] = dySubScalar(b.center[axis], dyMul(mustDyOf(sign), b.radius))
	onA, okA := sourceBoxPoint(onAExact)
	onB, okB := sourceBoxPoint(onBExact)
	if !okA || !okB || onA.Bound.Base() > report.Request.PointResolution.Base() ||
		onB.Bound.Base() > report.Request.PointResolution.Base() {
		report.Reason = ContactPointTooCoarse
		return
	}
	sep, ok := sourceBoxSignedReading(dySubScalar(distance, radius))
	if !ok {
		report.Reason = ContactPointTooCoarse
		return
	}
	featureA, featureB := ContactFeature{Face: a.face}, ContactFeature{Face: b.face}
	report.Manifold = &ContactManifold{Points: []ContactPoint{{
		OnA: onA, OnB: onB,
		Normal:      VecMeasurement{Value: normal, Bound: units.Scalar(0), Exactness: Exact},
		NormalAngle: units.Radians(0), Separation: sep,
		FaceA: a.face, FaceB: b.face, FeatureA: featureA, FeatureB: featureB,
	}}}
}
