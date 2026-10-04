package dynamics_test

import (
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
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2,
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
}
