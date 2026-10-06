package dynamics

import (
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

// A ball rests on the floor under a fixed upper sphere and strikes both at
// (200, 0, −100) mm/s. Its mass center sits at (0.6, 0, 1.8) in its frame,
// off the line of either contact, so both contact impulses turn it the same
// way about Y. The island applies both torques to the one body together:
// the ball's spin is the sum of both torque impulses over I_yy, larger than
// either alone, and the certificate's AngularUpper covers it.
func TestSphereIslandCombinedSpinChargesAlignedContactTorques(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	floor := sphereIslandTestFloor(t, doc)
	ball, upper := sphereIslandTestSphere(t, doc), sphereIslandTestSphere(t, doc)
	ballPose, err := r3.Translation(r3.Vec{Z: 5})
	require.NoError(t, err)
	upperPose, err := r3.Translation(r3.Vec{X: 6, Z: 13})
	require.NoError(t, err)
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
	material := Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	cfg := StepConfig{Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-3),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 4, MaxPairSweeps: 4096}
	w, err := NewWorld(t.Context(), doc, WorldConfig{Bodies: []RigidBody{
		{Body: floor, Role: Fixed, Material: material},
		{Body: ball, Role: Dynamic, Supplied: &mass, Material: material},
		{Body: upper, Role: Fixed, Material: material},
	}, Step: cfg})
	require.NoError(t, err)
	still := QuantityVec{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(0)}
	zeroW := QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
		Z: units.RadiansPerSecond(0)}
	start, err := w.NewState([]BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: still, AngularVelocity: zeroW},
		{Body: ball, Pose: ballPose, LinearVelocity: QuantityVec{X: units.MillimetersPerSecond(200),
			Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroW},
		{Body: upper, Pose: upperPose, LinearVelocity: still, AngularVelocity: zeroW},
	})
	require.NoError(t, err)
	gravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(0)}
	report, err := w.Step(t.Context(), start, StepInput{Gravity: gravity}, units.Seconds(.1))
	require.NoError(t, err)
	require.NotEmpty(t, report.Events, "%+v", report.Diagnostics)
	floorEvent, upperEvent := report.Events[0], report.Events[1]
	require.Equal(t, BodyPair{A: floor, B: ball}, floorEvent.Pair)
	require.Equal(t, BodyPair{A: ball, B: upper}, upperEvent.Pair)
	require.Equal(t, floorEvent.Time, upperEvent.Time)
	center := ballPose.Apply(mass.Center.Value)
	floorPoint, upperPoint := floorEvent.Manifold.Points[0], upperEvent.Manifold.Points[0]
	// The floor's impulse acts on the ball (B) along the normal; the upper
	// sphere's acts on the ball (A) against it.
	floorTorque := floorPoint.OnB.Value.Sub(center).Cross(floorPoint.Normal.Value).Scale(floorEvent.NormalImpulse.Base())
	upperTorque := upperPoint.OnA.Value.Sub(center).Cross(upperPoint.Normal.Value.Scale(-1)).Scale(upperEvent.NormalImpulse.Base())
	require.Positive(t, floorTorque.Y)
	require.Positive(t, upperTorque.Y)
	combined := (floorTorque.Y + upperTorque.Y) / inertia.Value.Base()
	spin := upperEvent.PostAngularVelocityA.Y.Base()
	require.InDelta(t, combined, spin, 1e-15)
	require.Greater(t, spin, floorTorque.Y/inertia.Value.Base())
	require.Greater(t, spin, upperTorque.Y/inertia.Value.Base())
	require.GreaterOrEqual(t, upperEvent.Solver.AngularUpper.Base(), spin)
}
