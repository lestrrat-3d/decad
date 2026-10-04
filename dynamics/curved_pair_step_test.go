package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSourceSpherePairTransverseEndpointImpact(t *testing.T) {
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	pa, err := r3.Translation(r3.Vec{X: -10})
	require.NoError(t, err)
	pb, err := r3.Translation(r3.Vec{X: 10, Y: 4})
	require.NoError(t, err)
	req := decad.ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), a, b, pa, pb, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
	require.InDelta(t, 10.39607805437114, contact.Gap.Value.Base(), 1e-12)
	va := decad.QuantityVec{X: units.MillimetersPerSecond(40), Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	vb := decad.QuantityVec{X: units.MillimetersPerSecond(-40), Y: units.MillimetersPerSecond(-32), Z: units.MillimetersPerSecond(0)}
	zero := decad.QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0)}
	sa := decad.RigidDriftSegment{From: pa, LinearVelocity: va, AngularVelocity: zero, Duration: units.Seconds(.125)}
	sb := decad.RigidDriftSegment{From: pb, LinearVelocity: vb, AngularVelocity: zero, Duration: units.Seconds(.125)}
	sweep, err := doc.SweepPair(t.Context(), a, b, sa, sb, decad.SweepRequest{ContactRequest: req,
		TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome, "cause=%v", sweep.Cause)
	require.True(t, sweep.BracketEndsAtDuration())
	require.Len(t, sweep.Event.Manifold.Points, 1)
	point := sweep.Event.Manifold.Points[0]
	require.Equal(t, r3.Vec{X: 1}, point.Normal.Value)
	require.InDelta(t, 0, point.Separation.Value.Base(), 1e-12)
	floatPoint := sweep.Samples[len(sweep.Samples)-1].FloatContact.Manifold.Points[0]
	require.Equal(t, floatPoint.OnA.Value, point.OnA.Value)
	require.Equal(t, floatPoint.OnB.Value, point.OnB.Value)
	require.GreaterOrEqual(t, point.OnA.Bound.Base(), floatPoint.OnA.Bound.Base())
	require.GreaterOrEqual(t, point.OnB.Bound.Base(), floatPoint.OnB.Bound.Base())
	require.LessOrEqual(t, point.OnA.Bound.Base(), req.PointResolution.Base())
	require.LessOrEqual(t, point.OnB.Bound.Base(), req.PointResolution.Base())
	require.Less(t, sweep.Bracket.From.Elapsed.Value.Base(), .125)
	require.Equal(t, .125, sweep.Bracket.To.Elapsed.Value.Base())
	mass := exactSphereMass()
	mat := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: mat},
			{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: mat}},
		Step: dynamics.StepConfig{Contact: req, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0), MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2},
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{{Body: a, Pose: pa, LinearVelocity: va,
		AngularVelocity: zero}, {Body: b, Pose: pb, LinearVelocity: vb,
		AngularVelocity: zero}})
	require.NoError(t, err)
	step, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.125))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
	require.Len(t, step.Events, 1)
	require.InDelta(t, 60, step.Events[0].NormalImpulse.Base(), 1e-5)
	finalA, ok := step.Next.Body(a)
	require.True(t, ok)
	finalB, ok := step.Next.Body(b)
	require.True(t, ok)
	require.InDelta(t, -20, finalA.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 20, finalB.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, -32, finalB.LinearVelocity.Y.Base(), 1e-6)
	require.InDelta(t, -5, finalA.Pose.Translation().X, 1e-6)
	require.InDelta(t, 5, finalB.Pose.Translation().X, 1e-6)
	require.InDelta(t, 0, finalB.Pose.Translation().Y, 1e-6)
}
