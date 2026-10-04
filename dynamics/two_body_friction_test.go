package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestTwoDynamicFrictionRefusesMissingSpinConservation(t *testing.T) {
	doc := decad.New()
	a := makeBox(t, doc, -5, -5, 5, 5, -10, 10)
	b := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 4)

	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(.5)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: b, Role: dynamics.Dynamic, Density: &density, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact: request, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 2,
		},
	})
	require.NoError(t, err)
	zeroW := zeroAngular(t)
	start, err := world.NewState([]dynamics.BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroW},
		{Body: b, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroW},
	})
	require.NoError(t, err)
	report, err := world.Step(t.Context(), start,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	require.Contains(t, report.Diagnostics[0].Reason, "conservation readings cannot be represented")
}
