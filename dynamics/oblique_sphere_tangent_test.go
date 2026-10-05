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

func TestRotatedSphereFrictionlessTangentialImpact(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 20)
	ball := makeBall(t, doc)
	turn, err := r3.Rotation(r3.Vec{Y: 1}, units.Degrees(45))
	require.NoError(t, err)
	normal := turn.ApplyDir(r3.Vec{Z: 1})
	startPose, err := r3.Translation(normal.Scale(20))
	require.NoError(t, err)
	endPose, err := r3.Translation(normal.Scale(14).Add(r3.Vec{Y: .6}))
	require.NoError(t, err)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), floor, ball, turn, startPose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
	require.InDelta(t, 5, contact.Gap.Value.Base(), 1e-12)
	sweep, err := doc.SweepPair(t.Context(), floor, ball,
		decad.PoseSegment{From: turn, To: turn, Duration: units.Seconds(.06)},
		decad.PoseSegment{From: startPose, To: endPose, Duration: units.Seconds(.06)},
		decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome, "cause=%v", sweep.Cause)
	require.NotNil(t, sweep.Event.Manifold)
	require.Len(t, sweep.Event.Manifold.Points, 1)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	mass := exactSphereMass()
	cfg := dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Supplied: &mass, Material: material}},
		Step: dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096},
	}
	w, err := dynamics.NewWorld(t.Context(), doc, cfg)
	require.NoError(t, err)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(-100 * normal.X),
		Y: units.MillimetersPerSecond(10), Z: units.MillimetersPerSecond(-100 * normal.Z)}
	state, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: startPose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	step, err := w.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.06))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
	require.Len(t, step.Events, 1)
	require.NotNil(t, step.Conservation)
	require.InDelta(t, 150, step.Events[0].NormalImpulse.Base(), 1e-6)
	require.Equal(t, velocity, step.Events[0].PreVelocity)
	require.Zero(t, step.Events[0].TangentImpulse.X.Base())
	require.Zero(t, step.Events[0].TangentImpulse.Y.Base())
	require.Zero(t, step.Events[0].TangentImpulse.Z.Base())
	final, ok := step.Next.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 50*normal.X, final.LinearVelocity.X.Base(), 1e-6)
	require.Equal(t, 10.0, final.LinearVelocity.Y.Base())
	require.InDelta(t, 50*normal.Z, final.LinearVelocity.Z.Base(), 1e-6)
	require.InDelta(t, .6, final.Pose.Translation().Y, 1e-9)
	require.Equal(t, final.LinearVelocity, step.Events[0].PostVelocity)
	atImpact, err := step.Trace.Sample(step.Events[0].Time)
	require.NoError(t, err)
	impactBall, ok := atImpact.Body(ball)
	require.True(t, ok)
	require.Equal(t, final.LinearVelocity, impactBall.LinearVelocity)
	before, err := step.Trace.Sample(units.Seconds(.03))
	require.NoError(t, err)
	beforeBall, ok := before.Body(ball)
	require.True(t, ok)
	require.Equal(t, velocity, beforeBall.LinearVelocity)
	require.InDelta(t, .3, beforeBall.Pose.Translation().Y, 1e-9)
	after, err := step.Trace.Sample(units.Seconds(.055))
	require.NoError(t, err)
	afterBall, ok := after.Body(ball)
	require.True(t, ok)
	require.Equal(t, final.LinearVelocity, afterBall.LinearVelocity)
	require.InDelta(t, .55, afterBall.Pose.Translation().Y, 1e-9)

	reverseCfg := cfg
	reverseCfg.Bodies = []dynamics.RigidBody{cfg.Bodies[1], cfg.Bodies[0]}
	reverseWorld, err := dynamics.NewWorld(t.Context(), doc, reverseCfg)
	require.NoError(t, err)
	reverseStart, err := reverseWorld.NewState([]dynamics.BodyState{
		{Body: ball, Pose: startPose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
		{Body: floor, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	reverseStep, err := reverseWorld.Step(t.Context(), reverseStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.06))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, reverseStep.Status, "%+v", reverseStep.Diagnostics)
	require.Len(t, reverseStep.Events, 1)
	reverseFinal, ok := reverseStep.Next.Body(ball)
	require.True(t, ok)
	require.InDelta(t, final.LinearVelocity.X.Base(), reverseFinal.LinearVelocity.X.Base(), 1e-6)
	require.Equal(t, 10.0, reverseFinal.LinearVelocity.Y.Base())
	require.InDelta(t, final.LinearVelocity.Z.Base(), reverseFinal.LinearVelocity.Z.Base(), 1e-6)
	reverseAfter, err := reverseStep.Trace.Sample(units.Seconds(.055))
	require.NoError(t, err)
	reverseBall, ok := reverseAfter.Body(ball)
	require.True(t, ok)
	require.Equal(t, reverseFinal.LinearVelocity, reverseBall.LinearVelocity)

	density := units.KilogramsPerCubicMillimeter(.001)
	densityMass, err := ball.MassProperties(t.Context(), density)
	require.NoError(t, err)
	densityCfg := cfg
	densityCfg.Bodies = []dynamics.RigidBody{cfg.Bodies[0],
		{Body: ball, Role: dynamics.Dynamic, Density: &density, Material: material}}
	densityWorld, err := dynamics.NewWorld(t.Context(), doc, densityCfg)
	require.NoError(t, err)
	densityStart, err := densityWorld.NewState([]dynamics.BodyState{
		{Body: floor, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: startPose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	densityStep, err := densityWorld.Step(t.Context(), densityStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.06))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, densityStep.Status, "%+v", densityStep.Diagnostics)
	require.Len(t, densityStep.Events, 1)
	require.InDelta(t, 150*densityMass.Mass.Value.Base(), densityStep.Events[0].NormalImpulse.Base(), 1e-6)
	densityFinal, ok := densityStep.Next.Body(ball)
	require.True(t, ok)
	require.Equal(t, 10.0, densityFinal.LinearVelocity.Y.Base())
	_, err = densityStep.Trace.Sample(units.Seconds(.055))
	require.NoError(t, err)

	thresholdCfg := cfg
	thresholdCfg.Step.ImpactSpeed = units.MillimetersPerSecond(math.Nextafter(100, math.Inf(-1)))
	thresholdWorld, err := dynamics.NewWorld(t.Context(), doc, thresholdCfg)
	require.NoError(t, err)
	thresholdStart, err := thresholdWorld.NewState([]dynamics.BodyState{
		{Body: floor, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: startPose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	thresholdStep, err := thresholdWorld.Step(t.Context(), thresholdStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.06))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, thresholdStep.Status)
	require.Nil(t, thresholdStep.Next)
	require.Empty(t, thresholdStep.Events)
	require.Contains(t, thresholdStep.Diagnostics[0].Reason, "threshold")

	tightCfg := cfg
	tightCfg.Step.VelocityResidual = units.MillimetersPerSecond(1e-15)
	tightWorld, err := dynamics.NewWorld(t.Context(), doc, tightCfg)
	require.NoError(t, err)
	tightStart, err := tightWorld.NewState([]dynamics.BodyState{
		{Body: floor, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: startPose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	tightStep, err := tightWorld.Step(t.Context(), tightStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.06))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, tightStep.Status)
	require.Nil(t, tightStep.Next)
	require.Empty(t, tightStep.Events)
	require.Contains(t, tightStep.Diagnostics[0].Reason, "residual")
}
