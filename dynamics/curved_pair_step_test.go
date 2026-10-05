package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSourceSpherePairStationaryTouchStep(t *testing.T) {
	doc := decad.New()
	fixed, moving := makeBall(t, doc), makeBall(t, doc)
	pose, err := r3.Translation(r3.Vec{X: 10})
	require.NoError(t, err)
	mass := exactSphereMass()
	mat := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	req := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: fixed, Role: dynamics.Fixed, Material: mat},
			{Body: moving, Role: dynamics.Dynamic, Supplied: &mass, Material: mat}},
		Step: dynamics.StepConfig{Contact: req, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0), MaxPoseEvaluations: 128,
			MaxIterations: 8, MaxEvents: 2},
	})
	require.NoError(t, err)
	initial, err := w.NewState([]dynamics.BodyState{{Body: fixed, Pose: r3.Identity(),
		LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: moving, Pose: pose, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)}})
	require.NoError(t, err)
	step, err := w.Step(t.Context(), initial, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
	require.Empty(t, step.Events)
	require.Equal(t, pose, step.Next.Entries()[1].Pose)
	interior, err := step.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	interiorBody, ok := interior.Body(moving)
	require.True(t, ok)
	require.Equal(t, pose, interiorBody.Pose)
	contact, err := doc.ContactPair(t.Context(), fixed, moving, r3.Identity(),
		interiorBody.Pose, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
}

func TestSourceSpherePairGrazingStep(t *testing.T) {
	doc := decad.New()
	fixed, moving := makeBall(t, doc), makeBall(t, doc)
	startPose, err := r3.Translation(r3.Vec{X: 20, Y: 10})
	require.NoError(t, err)
	touchPose, err := r3.Translation(r3.Vec{Y: 10})
	require.NoError(t, err)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(-40),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	zero := zeroAngular(t)
	duration := units.Seconds(1)
	req := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), fixed, moving, r3.Identity(), touchPose, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 1)
	stationary := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration}
	path := decad.RigidDriftSegment{From: startPose, Center: startPose.Translation(),
		LinearVelocity: velocity, AngularVelocity: zero, Duration: duration}
	sweep, err := doc.SweepPair(t.Context(), fixed, moving, stationary, path,
		decad.SweepRequest{ContactRequest: req, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepGrazingTouch, sweep.Outcome, "cause=%v", sweep.Cause)
	require.Equal(t, units.Scalar(.5), sweep.Event.At.Fraction)
	require.Nil(t, sweep.Bracket)

	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	config := dynamics.StepConfig{Contact: req, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: fixed, Role: dynamics.Fixed, Material: material},
			{Body: moving, Role: dynamics.Dynamic, Supplied: &mass, Material: material}},
		Step: config,
	})
	require.NoError(t, err)
	initial, err := w.NewState([]dynamics.BodyState{
		{Body: fixed, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zero},
		{Body: moving, Pose: startPose, LinearVelocity: velocity, AngularVelocity: zero},
	})
	require.NoError(t, err)
	step, err := w.Step(t.Context(), initial, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
	require.Len(t, step.Events, 1)
	require.Equal(t, dynamics.ContactGraze, step.Events[0].Kind)
	require.Equal(t, units.Seconds(.5), step.Events[0].Time)
	require.Equal(t, units.KilogramMillimetersPerSecond(0), step.Events[0].NormalImpulse)
	final, ok := step.Next.Body(moving)
	require.True(t, ok)
	require.Equal(t, velocity, final.LinearVelocity)
	require.Equal(t, r3.Vec{X: -20, Y: 10}, final.Pose.Translation())
	for _, query := range []struct {
		time units.Value
		want r3.Vec
	}{{units.Seconds(.25), r3.Vec{X: 10, Y: 10}},
		{units.Seconds(.5), r3.Vec{Y: 10}},
		{units.Seconds(.75), r3.Vec{X: -10, Y: 10}}} {
		sampled, err := step.Trace.Sample(query.time)
		require.NoError(t, err)
		body, ok := sampled.Body(moving)
		require.True(t, ok)
		require.Equal(t, query.want, body.Pose.Translation())
		require.Equal(t, velocity, body.LinearVelocity)
	}
	reversed, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: moving, Role: dynamics.Dynamic,
			Supplied: &mass, Material: material},
			{Body: fixed, Role: dynamics.Dynamic, Supplied: &mass, Material: material}},
		Step: config,
	})
	require.NoError(t, err)
	reversedInitial, err := reversed.NewState([]dynamics.BodyState{
		{Body: moving, Pose: startPose, LinearVelocity: velocity, AngularVelocity: zero},
		{Body: fixed, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zero},
	})
	require.NoError(t, err)
	reversedStep, err := reversed.Step(t.Context(), reversedInitial,
		dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, reversedStep.Status, "%+v", reversedStep.Diagnostics)
	require.Len(t, reversedStep.Events, 1)
	require.Equal(t, dynamics.ContactGraze, reversedStep.Events[0].Kind)
	require.Equal(t, r3.Vec{Y: -1}, reversedStep.Events[0].Manifold.Points[0].Normal.Value)
	reversedMoving, ok := reversedStep.Next.Body(moving)
	require.True(t, ok)
	require.Equal(t, velocity, reversedMoving.LinearVelocity)
	rotatedFixed, err := r3.FromBasis(r3.Basis{EX: r3.Vec{X: -1},
		EY: r3.Vec{Y: -1}, EZ: r3.Vec{Z: 1}}, r3.Vec{})
	require.NoError(t, err)
	rotatedMoving, err := r3.FromBasis(rotatedFixed.Basis(), r3.Vec{X: 20, Y: 10})
	require.NoError(t, err)
	rotatedWorld, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: fixed, Role: dynamics.Fixed, Material: material},
			{Body: moving, Role: dynamics.Dynamic, Supplied: &mass, Material: material}},
		Step: config,
	})
	require.NoError(t, err)
	rotatedState, err := rotatedWorld.NewState([]dynamics.BodyState{
		{Body: fixed, Pose: rotatedFixed, LinearVelocity: zeroVelocity(), AngularVelocity: zero},
		{Body: moving, Pose: rotatedMoving, LinearVelocity: velocity, AngularVelocity: zero},
	})
	require.NoError(t, err)
	rotatedStep, err := rotatedWorld.Step(t.Context(), rotatedState,
		dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, rotatedStep.Status, "%+v", rotatedStep.Diagnostics)
	require.Len(t, rotatedStep.Events, 1)
	require.Equal(t, dynamics.ContactGraze, rotatedStep.Events[0].Kind)
	rotatedSample, err := rotatedStep.Trace.Sample(units.Seconds(.75))
	require.NoError(t, err)
	rotatedBody, ok := rotatedSample.Body(moving)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: -10, Y: 10}, rotatedBody.Pose.Translation())
}

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
