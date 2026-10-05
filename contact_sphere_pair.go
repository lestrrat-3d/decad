package decad

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// classifySourceSpherePair compares the complete occupied balls. A response
// witness needs a nonzero center line and two crossing sphere faces.
func classifySourceSpherePair(report *ContactReport, a, b sourceSphereContactProof) {
	var delta proofarith.DyV3
	distance2 := proofarith.DyZero()
	axis, nonzero := 0, 0
	for i := range 3 {
		delta[i] = proofarith.DySubScalar(b.center[i], a.center[i])
		distance2 = proofarith.DyAdd(distance2, proofarith.DyMul(delta[i], delta[i]))
		if delta[i].Sign() != 0 {
			axis, nonzero = i, nonzero+1
		}
	}
	radius := proofarith.DyAdd(a.radius, b.radius)
	radius2 := proofarith.DyMul(radius, radius)
	switch proofarith.DyCmp(distance2, radius2) {
	case 1:
		lower := new(big.Rat).Sub(proofarith.FloatRat(proofarith.DySqrtDown(distance2)), radius.Rat())
		upper := new(big.Rat).Sub(proofarith.FloatRat(proofarith.DySqrtUp(distance2)), radius.Rat())
		lo, hi := ratFloatDown(lower), ratFloatUp(upper)
		if !finiteMeasurementValues(lo, hi) || lo <= 0 || hi < lo {
			report.Reason = ContactNoGapProof
			return
		}
		value := lo + (hi-lo)/2
		left := new(big.Rat).Sub(proofarith.FloatRat(value), proofarith.FloatRat(lo))
		right := new(big.Rat).Sub(proofarith.FloatRat(hi), proofarith.FloatRat(value))
		if right.Cmp(left) > 0 {
			left = right
		}
		bound := ratFloatUp(left)
		if !finiteMeasurementValues(value, bound) ||
			new(big.Rat).Sub(proofarith.FloatRat(value), proofarith.FloatRat(bound)).Sign() <= 0 {
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
	if nonzero == 0 {
		report.Reason = ContactAmbiguousFeature
		return
	}
	radiusDifference := proofarith.DyAbs(proofarith.DySubScalar(a.radius, b.radius))
	if proofarith.DyCmp(distance2, proofarith.DyMul(radiusDifference, radiusDifference)) <= 0 {
		report.Reason = ContactAmbiguousFeature
		return
	}
	if nonzero != 1 {
		publishObliqueSpherePair(report, a, b, delta, distance2, radius)
		return
	}
	distance := proofarith.DyAbs(delta[axis])
	sign := 1.0
	if delta[axis].Sign() < 0 {
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
	onAExact[axis] = proofarith.DyAdd(a.center[axis], proofarith.DyMul(proofarith.MustDyOf(sign), a.radius))
	onBExact[axis] = proofarith.DySubScalar(b.center[axis], proofarith.DyMul(proofarith.MustDyOf(sign), b.radius))
	onA, okA := sourceBoxPoint(onAExact)
	onB, okB := sourceBoxPoint(onBExact)
	if !okA || !okB || onA.Bound.Base() > report.Request.PointResolution.Base() ||
		onB.Bound.Base() > report.Request.PointResolution.Base() {
		report.Reason = ContactPointTooCoarse
		return
	}
	sep, ok := sourceBoxSignedReading(proofarith.DySubScalar(distance, radius))
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

// publishObliqueSpherePair retains the exact center-line direction until the
// final bounded float witness conversion. Each point ball includes the normal
// conversion error multiplied by its source radius.
func publishObliqueSpherePair(report *ContactReport, a, b sourceSphereContactProof,
	delta proofarith.DyV3, distance2, radius proofarith.Dyadic) {
	normal, angle, ok := orientedBoxNormal(delta)
	if !ok || angle.Base() > report.Request.NormalResolution.Base() {
		report.Reason = ContactNoNormalProof
		return
	}
	components := [3]float64{normal.Value.X, normal.Value.Y, normal.Value.Z}
	var pointA, pointB [3]*big.Rat
	for i, component := range components {
		offsetA := new(big.Rat).Mul(a.radius.Rat(), proofarith.FloatRat(component))
		offsetB := new(big.Rat).Mul(b.radius.Rat(), proofarith.FloatRat(component))
		pointA[i] = new(big.Rat).Add(a.center[i].Rat(), offsetA)
		pointB[i] = new(big.Rat).Sub(b.center[i].Rat(), offsetB)
	}
	onA, okA := orientedBoxPoint(pointA)
	onB, okB := orientedBoxPoint(pointB)
	if !okA || !okB {
		report.Reason = ContactPointTooCoarse
		return
	}
	for _, witness := range []struct {
		point  *VecMeasurement
		radius proofarith.Dyadic
	}{{&onA, a.radius}, {&onB, b.radius}} {
		radiusFloat := ratFloatUp(witness.radius.Rat())
		bound := provenUpRound(witness.point.Bound.Base() +
			provenUpRound(radiusFloat*normal.Bound.Base()))
		if !finiteMeasurementValues(bound) || bound > report.Request.PointResolution.Base() {
			report.Reason = ContactPointTooCoarse
			return
		}
		witness.point.Bound = units.Millimeters(bound)
		witness.point.Exactness = exactnessOf(bound)
	}
	low := new(big.Rat).Sub(proofarith.FloatRat(proofarith.DySqrtDown(distance2)), radius.Rat())
	high := new(big.Rat).Sub(proofarith.FloatRat(proofarith.DySqrtUp(distance2)), radius.Rat())
	value := ratFloatNearest(new(big.Rat).Quo(new(big.Rat).Add(low, high), big.NewRat(2, 1)))
	if !finiteMeasurementValues(value) {
		report.Reason = ContactPointTooCoarse
		return
	}
	left := new(big.Rat).Sub(low, proofarith.FloatRat(value))
	right := new(big.Rat).Sub(high, proofarith.FloatRat(value))
	boundExact := ratMax(left.Abs(left), right.Abs(right))
	boundExact.Add(boundExact, proofarith.FloatRat(onA.Bound.Base()))
	boundExact.Add(boundExact, proofarith.FloatRat(onB.Bound.Base()))
	bound := ratFloatUp(boundExact)
	if !finiteMeasurementValues(bound) {
		report.Reason = ContactPointTooCoarse
		return
	}
	sep := Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
		Exactness: exactnessOf(bound)}
	featureA, featureB := ContactFeature{Face: a.face}, ContactFeature{Face: b.face}
	report.Manifold = &ContactManifold{Points: []ContactPoint{{
		OnA: onA, OnB: onB, Normal: normal, NormalAngle: angle, Separation: sep,
		FaceA: a.face, FaceB: b.face, FeatureA: featureA, FeatureB: featureB,
	}}}
}
