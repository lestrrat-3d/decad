package dynamics

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func sphereIslandTestFloor(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), -10)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rectangle := s.CreateRectangle(-20, -20, 20, 20)
	s.Fix(rectangle.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func sphereIslandTestSphere(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	left, right, center := s.CreatePoint(-5, 0), s.CreatePoint(5, 0), s.CreatePoint(0, 0)
	s.Fix(left)
	s.CreateLine(left, right)
	s.CreateArc(center, right, left)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Revolve(s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

func TestSphereIslandCombinedSpinChargesAlignedContactTorques(t *testing.T) {
	doc := decad.New()
	floor := sphereIslandTestFloor(t, doc)
	ball, upper := sphereIslandTestSphere(t, doc), sphereIslandTestSphere(t, doc)
	ballPose, err := r3.Translation(r3.Vec{Z: 5})
	require.NoError(t, err)
	upperPose, err := r3.Translation(r3.Vec{X: 6, Z: 13})
	require.NoError(t, err)
	zeroLinear := decad.QuantityVec{X: units.MillimetersPerSecond(0),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	zeroAngular := decad.QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0)}
	path := func(pose r3.Transform) decad.RigidDriftSegment {
		return decad.RigidDriftSegment{From: pose, LinearVelocity: zeroLinear,
			AngularVelocity: zeroAngular, Duration: units.Seconds(.1)}
	}
	request := decad.SweepRequest{ContactRequest: decad.ContactRequest{
		PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)},
		TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128}
	floorSweep, err := doc.SweepPair(t.Context(), floor, ball,
		path(r3.Identity()), path(ballPose), request)
	require.NoError(t, err)
	require.Equal(t, decad.SweepInitiallyTouching, floorSweep.Outcome)
	upperSweep, err := doc.SweepPair(t.Context(), ball, upper,
		path(ballPose), path(upperPose), request)
	require.NoError(t, err)
	require.Equal(t, decad.SweepInitiallyTouching, upperSweep.Outcome)
	floorPoint := floorSweep.Event.Manifold.Points[0]
	upperPoint := upperSweep.Event.Manifold.Points[0]
	center := r3.Vec{X: .6, Z: 6.8}
	floorTorque := floorPoint.OnB.Value.Sub(center).Cross(floorPoint.Normal.Value)
	upperTorque := upperPoint.OnA.Value.Sub(center).Cross(upperPoint.Normal.Value.Scale(-1))
	require.Greater(t, floorTorque.Y, 0.0)
	require.Greater(t, upperTorque.Y, 0.0)
	reading := func(value units.Value) decad.Measurement {
		return decad.Measurement{Value: value, Bound: units.New(0, value.Unit()), Exactness: decad.Exact}
	}
	inertia := reading(units.KilogramSquareMillimeters(1e9))
	zeroInertia := reading(units.KilogramSquareMillimeters(0))
	mass := decad.MassProperties{
		Mass: reading(units.Kilograms(1)),
		Center: decad.VecMeasurement{Value: r3.Vec{X: .6, Z: 1.8},
			Bound: units.Millimeters(0), Exactness: decad.Exact},
		Inertia: decad.InertiaReading{XX: inertia, YY: inertia, ZZ: inertia,
			XY: zeroInertia, XZ: zeroInertia, YZ: zeroInertia},
	}
	step := StepConfig{ImpulseResidual: units.KilogramMillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-3)}
	sphere, ok := floorPoint.FaceB.Surface().(decad.Sphere)
	require.True(t, ok)
	floorSpin, ok := sphereOmittedSpinBounds(floorPoint, floorPoint, 1, ballPose,
		mass, sphere.Radius, 1100.0/3, units.Seconds(.1), step)
	require.True(t, ok)
	upperSpin, ok := sphereOmittedSpinBounds(upperPoint, upperPoint, 0, ballPose,
		mass, sphere.Radius, 1000.0/3, units.Seconds(.1), step)
	require.True(t, ok)
	angular, combined, ok := sphereIslandCombinedSpin(mass, []sphereOmittedBounds{floorSpin, upperSpin})
	require.True(t, ok)
	require.Equal(t, 0, angular.Cmp(new(big.Rat).Add(floorSpin.angularSpeed, upperSpin.angularSpeed)))
	separate := new(big.Rat).Add(floorSpin.twiceEnergy, upperSpin.twiceEnergy)
	require.Greater(t, combined.Cmp(separate), 0)
	cross := new(big.Rat).Mul(floorSpin.angularSpeed, upperSpin.angularSpeed)
	cross.Mul(cross, inertiaRowCeiling(mass.Inertia))
	cross.Mul(cross, big.NewRat(2, 1))
	require.Equal(t, 0, new(big.Rat).Sub(combined, separate).Cmp(cross))
}
