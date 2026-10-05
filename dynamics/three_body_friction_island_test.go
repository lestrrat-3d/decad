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

func TestThreeBodyFrictionIslandRealPath(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	wall := makeBox(t, doc, -10, -20, 0, 20, -10, 30)
	ball := makeBall(t, doc)
	pose, err := r3.Translation(r3.Vec{X: 5, Z: 5})
	require.NoError(t, err)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	duration := units.Seconds(.1)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(-100),
		Y: units.MillimetersPerSecond(20), Z: units.MillimetersPerSecond(-100)}
	for _, fixed := range []*decad.Body{floor, wall} {
		contact, contactErr := doc.ContactPair(t.Context(), fixed, ball, r3.Identity(), pose, request)
		require.NoError(t, contactErr)
		require.Equal(t, decad.ContactTouching, contact.Relation)
		require.Len(t, contact.Manifold.Points, 1)
		sweep, sweepErr := doc.SweepPair(t.Context(), fixed, ball,
			decad.RigidDriftSegment{From: r3.Identity(), LinearVelocity: zeroVelocity(),
				AngularVelocity: zeroAngular(t), Duration: duration},
			decad.RigidDriftSegment{From: pose, LinearVelocity: velocity,
				AngularVelocity: zeroAngular(t), Duration: duration},
			decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
				MaxPoseEvaluations: 128, StartPolicy: decad.StopAtInitialContact})
		require.NoError(t, sweepErr)
		require.Equal(t, decad.SweepInitiallyTouching, sweep.Outcome)
		require.Len(t, sweep.Event.Manifold.Points, 1)
	}
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(.5)}
	step := dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(100),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 4, MaxPairSweeps: 4096}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: wall, Role: dynamics.Fixed, Material: material}},
		Excluded: []dynamics.BodyPair{{A: floor, B: wall}}, Step: step})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: pose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
		{Body: wall, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 2)
	require.Equal(t, dynamics.BodyPair{A: floor, B: ball}, report.Events[0].Pair)
	require.Equal(t, dynamics.BodyPair{A: ball, B: wall}, report.Events[1].Pair)
	for _, event := range report.Events {
		require.Len(t, event.Manifold.Points, 1)
		require.Len(t, event.PointImpulses, 1)
		require.InDelta(t, 100, event.NormalImpulse.Base(), 1e-9)
		require.InDelta(t, 40.0/9, math.Abs(event.TangentImpulse.Y.Base()), 1e-9)
		require.NotNil(t, event.Solver)
		require.LessOrEqual(t, event.Solver.TangentResidual.Base(), step.VelocityResidual.Base())
		require.LessOrEqual(t, math.Abs(event.TangentImpulse.Y.Base()),
			material.Friction.Base()*event.NormalImpulse.Base())
	}
	final, found := report.Next.Body(ball)
	require.True(t, found)
	require.Equal(t, 0.0, final.LinearVelocity.X.Base())
	require.InDelta(t, 100.0/9, final.LinearVelocity.Y.Base(), 1e-9)
	require.Equal(t, 0.0, final.LinearVelocity.Z.Base())
	require.InDelta(t, -20.0/9, final.AngularVelocity.X.Base(), 1e-9)
	require.Equal(t, 0.0, final.AngularVelocity.Y.Base())
	require.InDelta(t, 20.0/9, final.AngularVelocity.Z.Base(), 1e-9)
	require.InDelta(t, 5, final.Pose.Translation().X, 1e-12)
	require.InDelta(t, 10.0/9, final.Pose.Translation().Y, 1e-12)
	require.InDelta(t, 5, final.Pose.Translation().Z, 1e-12)
	require.NotNil(t, report.Conservation)
	require.InDelta(t, 100, report.Conservation.ContactImpulse.Value.X.Base(), 1e-9)
	require.InDelta(t, -80.0/9, report.Conservation.ContactImpulse.Value.Y.Base(), 1e-9)
	require.InDelta(t, 100, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-9)
	require.Less(t, report.Conservation.Completion.KineticEnergy.Value.Base(),
		report.Conservation.AfterKick.KineticEnergy.Value.Base())
	for _, sampleAt := range []units.Value{units.Seconds(.025), units.Seconds(.05), units.Seconds(.075)} {
		sample, sampleErr := report.Trace.Sample(sampleAt)
		require.NoError(t, sampleErr)
		ballAt, present := sample.Body(ball)
		require.True(t, present)
		require.Equal(t, final.LinearVelocity, ballAt.LinearVelocity)
		require.Equal(t, final.AngularVelocity, ballAt.AngularVelocity)
		for _, fixed := range []*decad.Body{floor, wall} {
			contact, contactErr := doc.ContactPair(t.Context(), fixed, ball,
				r3.Identity(), ballAt.Pose, request)
			require.NoError(t, contactErr)
			require.Equal(t, decad.ContactTouching, contact.Relation)
		}
	}
	replayed, err := report.Trace.Sample(duration)
	require.NoError(t, err)
	require.Equal(t, report.Next.Entries(), replayed.Entries())

	reversed, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: wall, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: floor, Role: dynamics.Fixed, Material: material}},
		Excluded: []dynamics.BodyPair{{A: floor, B: wall}}, Step: step})
	require.NoError(t, err)
	reversedStart, err := reversed.NewState(start.Entries())
	require.NoError(t, err)
	reversedReport, err := reversed.Step(t.Context(), reversedStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, reversedReport.Status, "%+v", reversedReport.Diagnostics)
	reversedBall, found := reversedReport.Next.Body(ball)
	require.True(t, found)
	require.Equal(t, final, reversedBall)
	_, err = reversedReport.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)

	for _, tc := range []struct {
		name     string
		material dynamics.Material
		mass     decad.MassProperties
		step     dynamics.StepConfig
	}{
		{name: "friction cone", material: dynamics.Material{Restitution: units.Scalar(0),
			Friction: units.Scalar(.01)}, mass: mass, step: step},
		{name: "restitution", material: dynamics.Material{Restitution: units.Scalar(.5),
			Friction: units.Scalar(.5)}, mass: mass, step: step},
		{name: "bounded mass", material: material, mass: func() decad.MassProperties {
			other := mass
			other.Mass.Exactness = decad.Approximate
			other.Mass.Bound = units.Kilograms(.01)
			return other
		}(), step: step},
		{name: "event limit", material: material, mass: mass, step: func() dynamics.StepConfig {
			other := step
			other.MaxEvents = 2
			return other
		}()},
	} {
		t.Run(tc.name, func(t *testing.T) {
			refusal, worldErr := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
				Bodies: []dynamics.RigidBody{{Body: floor, Role: dynamics.Fixed, Material: tc.material},
					{Body: ball, Role: dynamics.Dynamic, Supplied: &tc.mass, Material: tc.material},
					{Body: wall, Role: dynamics.Fixed, Material: tc.material}},
				Excluded: []dynamics.BodyPair{{A: floor, B: wall}}, Step: tc.step})
			require.NoError(t, worldErr)
			refusalStart, stateErr := refusal.NewState(start.Entries())
			require.NoError(t, stateErr)
			stopped, stepErr := refusal.Step(t.Context(), refusalStart,
				dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
			require.NoError(t, stepErr)
			require.Equal(t, dynamics.Undecided, stopped.Status)
			require.Nil(t, stopped.Next)
		})
	}
}
