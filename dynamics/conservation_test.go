package dynamics_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestConservationBoundsEncloseSuppliedMassInterval(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	density := units.KilogramsPerCubicMillimeter(0.001)
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	mass.Mass.Bound = units.Kilograms(0.01)
	mass.Mass.Exactness = decad.Approximate
	mass.Center.Bound = units.Millimeters(0.02)
	mass.Center.Exactness = decad.Approximate
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(10),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Conservation)
	reading := report.Conservation.Input
	require.Equal(t, units.Torque, reading.KineticEnergy.Value.Kind())
	require.Equal(t, units.Torque, reading.KineticEnergy.Bound.Kind())
	require.Equal(t, decad.Approximate, reading.KineticEnergy.Exactness)
	require.InDelta(t, 50, reading.KineticEnergy.Value.Base(), 1e-12)
	require.GreaterOrEqual(t, reading.KineticEnergy.Bound.Base(), 0.5)
	require.Equal(t, units.Impulse, reading.LinearMomentum.Value.X.Kind())
	require.Equal(t, units.Impulse, reading.LinearMomentum.Bound.X.Kind())
	require.InDelta(t, 10, reading.LinearMomentum.Value.X.Base(), 1e-12)
	require.GreaterOrEqual(t, reading.LinearMomentum.Bound.X.Base(), 0.1)
	require.InDelta(t, 10, report.Conservation.Completion.LinearMomentum.Value.X.Base(), 1e-12)
	require.Zero(t, report.Conservation.ContactImpulse.Value.X.Base())
	require.Equal(t, units.AngularMomentum, reading.AngularMomentum.Value.Y.Kind())
	require.Equal(t, units.AngularMomentum, reading.AngularMomentum.Bound.Y.Kind())
	require.InDelta(t, 150, reading.AngularMomentum.Value.Y.Base(), 1e-12)
	require.GreaterOrEqual(t, reading.AngularMomentum.Bound.Y.Base(), 1.5)
	require.GreaterOrEqual(t, reading.AngularMomentum.Bound.Z.Base(), 0.198)

	// The translated endpoint holds rounded coordinates. Applying the same mass
	// interval once to their combined drift coefficient keeps its bound tight.
	velocity = dynamics.QuantityVec{X: units.MillimetersPerSecond(0.1),
		Y: units.MillimetersPerSecond(0.3), Z: units.MillimetersPerSecond(0)}
	start, err = w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err = w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	drift := report.Conservation.DriftChange
	require.Equal(t, units.Torque, drift.KineticEnergy.Value.Kind())
	require.Equal(t, units.Torque, drift.KineticEnergy.Bound.Kind())
	require.Zero(t, drift.KineticEnergy.Value.Base())
	require.Zero(t, drift.KineticEnergy.Bound.Base())
	require.Equal(t, units.Impulse, drift.LinearMomentum.Value.X.Kind())
	require.Zero(t, drift.LinearMomentum.Value.X.Base())
	require.Zero(t, drift.LinearMomentum.Bound.X.Base())
	require.Equal(t, units.AngularMomentum, drift.AngularMomentum.Value.Z.Kind())
	require.Equal(t, units.AngularMomentum, drift.AngularMomentum.Bound.Z.Kind())
	require.InDelta(t, 4.163336342344337e-19, drift.AngularMomentum.Value.Z.Base(), 1e-21)
	require.Less(t, drift.AngularMomentum.Bound.Z.Base(), 1e-18)
	require.Less(t, math.Abs(drift.AngularMomentum.Value.Z.Base()), 1e-16)
}

func TestKinematicImpactCorrectionIsExcludedFromDriftChange(t *testing.T) {
	doc := decad.New()
	driver := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 20, -0.125, 30, 9.875, 0, 10)
	w := kinematicImpactWorld(t, doc, driver, box, false, 2)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(0),
		Y: units.MillimetersPerSecond(1), Z: units.MillimetersPerSecond(0)}
	start, err := w.NewState([]dynamics.BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	endDriver, err := r3.Translation(r3.Vec{X: 20})
	require.NoError(t, err)
	duration := units.Seconds(.25)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: r3.Identity(), To: endDriver, Duration: duration}}}}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.NotNil(t, report.Conservation)
	event := report.Events[0]
	require.NotZero(t, event.PositionChangeB.X)
	checkpoint, err := report.Trace.Sample(event.Time)
	require.NoError(t, err)
	post, ok := checkpoint.Body(box)
	require.True(t, ok)
	end, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, post.LinearVelocity, end.LinearVelocity)

	dx := new(big.Rat).Sub(new(big.Rat).SetFloat64(end.Pose.Translation().X),
		new(big.Rat).SetFloat64(post.Pose.Translation().X))
	dy := new(big.Rat).Sub(new(big.Rat).SetFloat64(end.Pose.Translation().Y),
		new(big.Rat).SetFloat64(post.Pose.Translation().Y))
	expected := new(big.Rat).Sub(new(big.Rat).Mul(dx,
		new(big.Rat).SetFloat64(post.LinearVelocity.Y.Base())),
		new(big.Rat).Mul(dy, new(big.Rat).SetFloat64(post.LinearVelocity.X.Base())))
	want, _ := expected.Float64()
	actual := report.Conservation.DriftChange.AngularMomentum.Value.Z.Base()
	require.InDelta(t, want, actual, 1e-16)
	require.Less(t, math.Abs(actual-want), math.Abs(event.PositionChangeB.X)*0.1)
	require.Zero(t, report.Conservation.DriftChange.KineticEnergy.Value.Base())
	require.Zero(t, report.Conservation.DriftChange.LinearMomentum.Value.Y.Base())
}
