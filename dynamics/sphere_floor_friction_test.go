package dynamics_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSphereFloorSlidingFrictionUsesContactSweepStepAndTrace(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	ball := makeBall(t, doc)
	pose, err := r3.Translation(r3.Vec{Z: 5})
	require.NoError(t, err)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), floor, ball, r3.Identity(), pose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 1)
	flatEnd, err := r3.Translation(r3.Vec{X: 5, Z: 5})
	require.NoError(t, err)
	path, err := doc.SweepPair(t.Context(), floor, ball,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(.1)},
		decad.PoseSegment{From: pose, To: flatEnd, Duration: units.Seconds(.1)},
		decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128, StartPolicy: decad.ContinueCertifiedTouch})
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, path.Outcome)
	_, err = path.ContactTrack.ManifoldAt(units.Scalar(.5))
	require.NoError(t, err)
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(.5)}
	cfg := dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 2, MaxPairSweeps: 4096}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		}, Step: cfg,
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: pose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	gravity := dynamics.QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(-1000)}
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: gravity}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Len(t, report.Events[0].Manifold.Points, 1)
	require.InDelta(t, 100, report.Events[0].NormalImpulse.Base(), 1e-6)
	require.InDelta(t, -200.0/7, report.Events[0].TangentImpulse.X.Base(), 1e-6)
	require.Len(t, report.Events[0].PointImpulses, 1)
	require.LessOrEqual(t, math.Abs(report.Events[0].TangentImpulse.X.Base()),
		material.Friction.Base()*report.Events[0].NormalImpulse.Base())
	require.NotNil(t, report.Conservation)
	require.InDelta(t, -200.0/7, report.Conservation.ContactImpulse.Value.X.Base(), 1e-6)
	require.InDelta(t, 100, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	final, ok := report.Next.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 500.0/7, final.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 100.0/7, final.AngularVelocity.Y.Base(), 1e-6)
	require.GreaterOrEqual(t, report.Events[0].Solver.AngularUpper.Base(),
		math.Abs(final.AngularVelocity.Y.Base()))
	require.InDelta(t, 50.0/7, final.Pose.Translation().X, 1e-6)
	for _, sampleAt := range []units.Value{units.Seconds(.05), units.Seconds(.1)} {
		sample, sampleErr := report.Trace.Sample(sampleAt)
		require.NoError(t, sampleErr)
		ballState, exists := sample.Body(ball)
		require.True(t, exists)
		relation, contactErr := doc.ContactPair(t.Context(), floor, ball,
			r3.Identity(), ballState.Pose, request)
		require.NoError(t, contactErr)
		require.Equal(t, decad.ContactTouching, relation.Relation)
	}
	second, err := w.Step(t.Context(), *report.Next, dynamics.StepInput{Gravity: gravity}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, second.Status, "%+v", second.Diagnostics)
	require.Len(t, second.Events, 1)
	_, err = second.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	reverse, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: ball, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: floor, Role: dynamics.Fixed, Material: material},
		}, Step: cfg,
	})
	require.NoError(t, err)
	reverseStart, err := reverse.NewState([]dynamics.BodyState{
		{Body: ball, Pose: pose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroAngular(t)},
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	reversed, err := reverse.Step(t.Context(), reverseStart,
		dynamics.StepInput{Gravity: gravity}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, reversed.Status, "%+v", reversed.Diagnostics)
	require.InDelta(t, 200.0/7, reversed.Events[0].TangentImpulse.X.Base(), 1e-6)
	_, err = reversed.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	slipMaterial := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(.1)}
	slipWorld, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: slipMaterial},
			{Body: ball, Role: dynamics.Dynamic, Supplied: &mass, Material: slipMaterial},
		}, Step: cfg,
	})
	require.NoError(t, err)
	slipStart, err := slipWorld.NewState(start.Entries())
	require.NoError(t, err)
	slipping, err := slipWorld.Step(t.Context(), slipStart,
		dynamics.StepInput{Gravity: gravity}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, slipping.Status, "%+v", slipping.Diagnostics)
	require.InDelta(t, -10, slipping.Events[0].TangentImpulse.X.Base(), 1e-6)
	slipFinal, ok := slipping.Next.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 90, slipFinal.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 5, slipFinal.AngularVelocity.Y.Base(), 1e-6)
	_, err = slipping.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	// With restitution 0.5 the initial touch bounces: the floor delivers
	// 1.5·100 kg·mm/s and the ball leaves at 50 mm/s, while friction −200/7
	// kg·mm/s, inside the 75 kg·mm/s cone, sets it rolling.
	t.Run("restitution", func(t *testing.T) {
		bouncy := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(.5)}
		world, worldErr := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
			Bodies: []dynamics.RigidBody{
				{Body: floor, Role: dynamics.Fixed, Material: bouncy},
				{Body: ball, Role: dynamics.Dynamic, Supplied: &mass, Material: bouncy},
			}, Step: cfg,
		})
		require.NoError(t, worldErr)
		bouncyStart, stateErr := world.NewState(start.Entries())
		require.NoError(t, stateErr)
		result, stepErr := world.Step(t.Context(), bouncyStart, dynamics.StepInput{Gravity: gravity},
			units.Seconds(.1))
		require.NoError(t, stepErr)
		require.Equal(t, dynamics.Advanced, result.Status, "%+v", result.Diagnostics)
		require.Len(t, result.Events, 1)
		require.InDelta(t, 150, result.Events[0].NormalImpulse.Base(), 1e-6)
		require.InDelta(t, -200.0/7, result.Events[0].TangentImpulse.X.Base(), 1e-6)
		rolling, ok := result.Next.Body(ball)
		require.True(t, ok)
		require.InDelta(t, 500.0/7, rolling.LinearVelocity.X.Base(), 1e-6)
		require.InDelta(t, 50, rolling.LinearVelocity.Z.Base(), 1e-6)
		require.InDelta(t, 100.0/7, rolling.AngularVelocity.Y.Base(), 1e-6)
	})
	for _, tc := range []struct {
		name     string
		material dynamics.Material
		mass     decad.MassProperties
	}{
		{name: "bounded mass", material: material, mass: func() decad.MassProperties {
			changed := mass
			changed.Mass.Exactness = decad.Approximate
			changed.Mass.Bound = units.Kilograms(.01)
			return changed
		}()},
	} {
		t.Run(tc.name, func(t *testing.T) {
			refusal, worldErr := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
				Bodies: []dynamics.RigidBody{
					{Body: floor, Role: dynamics.Fixed, Material: tc.material},
					{Body: ball, Role: dynamics.Dynamic, Supplied: &tc.mass, Material: tc.material},
				}, Step: cfg,
			})
			require.NoError(t, worldErr)
			refusalStart, stateErr := refusal.NewState(start.Entries())
			require.NoError(t, stateErr)
			result, stepErr := refusal.Step(t.Context(), refusalStart,
				dynamics.StepInput{Gravity: gravity}, units.Seconds(.1))
			require.NoError(t, stepErr)
			require.Equal(t, dynamics.Undecided, result.Status)
			require.Nil(t, result.Next)
		})
	}
}
