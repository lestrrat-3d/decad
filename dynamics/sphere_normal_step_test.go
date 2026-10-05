package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSourceSpherePairDiagonalImpact(t *testing.T) {
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	pa, err := r3.Translation(r3.Vec{X: -6, Y: -8})
	require.NoError(t, err)
	pb, err := r3.Translation(r3.Vec{X: 6, Y: 8})
	require.NoError(t, err)
	req := decad.ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), a, b, pa, pb, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
	va := decad.QuantityVec{X: units.MillimetersPerSecond(30), Y: units.MillimetersPerSecond(40),
		Z: units.MillimetersPerSecond(0)}
	vb := decad.QuantityVec{X: units.MillimetersPerSecond(-30), Y: units.MillimetersPerSecond(-40),
		Z: units.MillimetersPerSecond(0)}
	zero := decad.QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
		Z: units.RadiansPerSecond(0)}
	sa := decad.RigidDriftSegment{From: pa, LinearVelocity: va, AngularVelocity: zero,
		Duration: units.Seconds(.1)}
	sb := decad.RigidDriftSegment{From: pb, LinearVelocity: vb, AngularVelocity: zero,
		Duration: units.Seconds(.1)}
	sweep, err := doc.SweepPair(t.Context(), a, b, sa, sb, decad.SweepRequest{ContactRequest: req,
		TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome, "cause=%v", sweep.Cause)
	require.True(t, sweep.BracketEndsAtDuration())
	require.Len(t, sweep.Event.Manifold.Points, 1)
	point := sweep.Event.Manifold.Points[0]
	require.InDelta(t, .6, point.Normal.Value.X, 1e-14)
	require.InDelta(t, .8, point.Normal.Value.Y, 1e-14)
	require.Positive(t, point.Normal.Bound.Base())
	require.LessOrEqual(t, point.NormalAngle.Base(), req.NormalResolution.Base())
	require.LessOrEqual(t, point.OnA.Bound.Base(), req.PointResolution.Base())
	require.LessOrEqual(t, point.OnB.Bound.Base(), req.PointResolution.Base())
	require.LessOrEqual(t, point.Separation.Bound.Base(), req.PointResolution.Base())
	endpointA, err := r3.Translation(r3.Vec{X: -3, Y: -4})
	require.NoError(t, err)
	endpointB, err := r3.Translation(r3.Vec{X: 3, Y: 4})
	require.NoError(t, err)
	endpoint, err := doc.ContactPair(t.Context(), a, b, endpointA, endpointB, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, endpoint.Relation)
	require.Len(t, endpoint.Manifold.Points, 1)
	require.Equal(t, point.FaceA, endpoint.Manifold.Points[0].FaceA)
	reversed, err := doc.ContactPair(t.Context(), b, a, endpointB, endpointA, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, reversed.Relation)
	require.InDelta(t, -.6, reversed.Manifold.Points[0].Normal.Value.X, 1e-14)
	require.InDelta(t, -.8, reversed.Manifold.Points[0].Normal.Value.Y, 1e-14)
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
			ImpactSpeed:             units.MillimetersPerSecond(0), MaxPoseEvaluations: 128,
			MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096},
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{{Body: a, Pose: pa, LinearVelocity: va,
		AngularVelocity: zero}, {Body: b, Pose: pb, LinearVelocity: vb, AngularVelocity: zero}})
	require.NoError(t, err)
	step, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
	require.Len(t, step.Events, 1)
	require.InDelta(t, 75, step.Events[0].NormalImpulse.Base(), 1e-5)
	finalA, ok := step.Next.Body(a)
	require.True(t, ok)
	finalB, ok := step.Next.Body(b)
	require.True(t, ok)
	require.InDelta(t, -15, finalA.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, -20, finalA.LinearVelocity.Y.Base(), 1e-6)
	require.InDelta(t, 15, finalB.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 20, finalB.LinearVelocity.Y.Base(), 1e-6)
	require.NotNil(t, step.Conservation)
	require.InDelta(t, 0, step.Conservation.Completion.LinearMomentum.Value.X.Base(), 1e-6)
	require.InDelta(t, 0, step.Conservation.Completion.LinearMomentum.Value.Y.Base(), 1e-6)
	sa.Duration, sb.Duration = units.Seconds(.2), units.Seconds(.2)
	interiorSweep, err := doc.SweepPair(t.Context(), a, b, sa, sb, decad.SweepRequest{ContactRequest: req,
		TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, interiorSweep.Outcome, "cause=%v", interiorSweep.Cause)
	require.True(t, interiorSweep.HasAffineReplayProof())
	require.Len(t, interiorSweep.Event.Manifold.Points, 1)
	require.Greater(t, interiorSweep.Event.Manifold.Points[0].Normal.Bound.Base(), 0.0)
	replayA, replayB, err := interiorSweep.CertifiedPosesAt(units.Seconds(.05))
	require.NoError(t, err)
	require.InDelta(t, -4.5, replayA.Translation().X, 1e-9)
	require.InDelta(t, 6, replayB.Translation().Y, 1e-9)
	_, _, err = interiorSweep.CertifiedPosesAt(units.Seconds(.15))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	long, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, long.Status, "%+v", long.Diagnostics)
	require.Len(t, long.Events, 1)
	longA, ok := long.Next.Body(a)
	require.True(t, ok)
	longB, ok := long.Next.Body(b)
	require.True(t, ok)
	require.InDelta(t, -4.5, longA.Pose.Translation().X, 2e-6)
	require.InDelta(t, -6, longA.Pose.Translation().Y, 2e-6)
	require.InDelta(t, 4.5, longB.Pose.Translation().X, 2e-6)
	require.InDelta(t, 6, longB.Pose.Translation().Y, 2e-6)
	require.InDelta(t, 625, long.Conservation.Completion.KineticEnergy.Value.Base(), 1e-4)
	beforeImpact, err := long.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	beforeA, ok := beforeImpact.Body(a)
	require.True(t, ok)
	require.InDelta(t, -4.5, beforeA.Pose.Translation().X, 1e-9)
	require.Equal(t, va, beforeA.LinearVelocity)
	afterImpact, err := long.Trace.Sample(units.Seconds(.15))
	require.NoError(t, err)
	afterA, ok := afterImpact.Body(a)
	require.True(t, ok)
	require.InDelta(t, -3.75, afterA.Pose.Translation().X, 2e-6)
	require.InDelta(t, -5, afterA.Pose.Translation().Y, 2e-6)
	require.InDelta(t, -15, afterA.LinearVelocity.X.Base(), 1e-6)
	endReplay, err := long.Trace.Sample(units.Seconds(.2))
	require.NoError(t, err)
	endA, ok := endReplay.Body(a)
	require.True(t, ok)
	require.Equal(t, longA.Pose, endA.Pose)
	require.Equal(t, longA.LinearVelocity, endA.LinearVelocity)
	continued, err := w.Step(t.Context(), *long.Next,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, continued.Status, "%+v", continued.Diagnostics)
	require.Empty(t, continued.Events)
	clearReplay, err := continued.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	clearA, ok := clearReplay.Body(a)
	require.True(t, ok)
	require.InDelta(t, longA.Pose.Translation().X-.75, clearA.Pose.Translation().X, 2e-6)
	wReverse, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: mat},
			{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: mat}},
		Step: dynamics.StepConfig{Contact: req, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0), MaxPoseEvaluations: 128,
			MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096},
	})
	require.NoError(t, err)
	backward, err := wReverse.NewState([]dynamics.BodyState{{Body: b, Pose: pb, LinearVelocity: vb,
		AngularVelocity: zero}, {Body: a, Pose: pa, LinearVelocity: va, AngularVelocity: zero}})
	require.NoError(t, err)
	reverseStep, err := wReverse.Step(t.Context(), backward,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, reverseStep.Status, "%+v", reverseStep.Diagnostics)
	require.Len(t, reverseStep.Events, 1)
	require.InDelta(t, -.6, reverseStep.Events[0].Manifold.Points[0].Normal.Value.X, 1e-14)
	require.InDelta(t, -.8, reverseStep.Events[0].Manifold.Points[0].Normal.Value.Y, 1e-14)
	require.InDelta(t, long.Events[0].NormalImpulse.Base(), reverseStep.Events[0].NormalImpulse.Base(), 1e-6)
	backA, ok := reverseStep.Next.Body(a)
	require.True(t, ok)
	backB, ok := reverseStep.Next.Body(b)
	require.True(t, ok)
	require.InDelta(t, longA.Pose.Translation().X, backA.Pose.Translation().X, 1e-8)
	require.InDelta(t, longB.Pose.Translation().Y, backB.Pose.Translation().Y, 1e-8)
	noncentral := mass
	noncentral.Center.Value = r3.Vec{X: .25}
	wNoncentral, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: mat},
			{Body: b, Role: dynamics.Dynamic, Supplied: &noncentral, Material: mat}},
		Step: dynamics.StepConfig{Contact: req, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0), MaxPoseEvaluations: 128,
			MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096},
	})
	require.NoError(t, err)
	shifted, err := wNoncentral.NewState([]dynamics.BodyState{{Body: a, Pose: pa, LinearVelocity: va,
		AngularVelocity: zero}, {Body: b, Pose: pb, LinearVelocity: vb, AngularVelocity: zero}})
	require.NoError(t, err)
	unsupported, err := wNoncentral.Step(t.Context(), shifted,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, unsupported.Status)
	require.Nil(t, unsupported.Next)
}

func TestSourceSpherePairDiagonalRefusals(t *testing.T) {
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	req := decad.ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)}
	coincident, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), r3.Identity(), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, coincident.Relation)
	require.Nil(t, coincident.Manifold)
	start, err := r3.Translation(r3.Vec{X: 10, Y: -1})
	require.NoError(t, err)
	zeroV := decad.QuantityVec{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(0)}
	moveV := decad.QuantityVec{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(3),
		Z: units.MillimetersPerSecond(0)}
	zeroA := decad.QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
		Z: units.RadiansPerSecond(0)}
	sweep, err := doc.SweepPair(t.Context(), a, b,
		decad.RigidDriftSegment{From: r3.Identity(), LinearVelocity: zeroV, AngularVelocity: zeroA,
			Duration: units.Seconds(1)},
		decad.RigidDriftSegment{From: start, LinearVelocity: moveV, AngularVelocity: zeroA,
			Duration: units.Seconds(1)},
		decad.SweepRequest{ContactRequest: req, TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepUndecided, sweep.Outcome)
	require.Equal(t, decad.SweepEventUnrepresentable, sweep.Cause)
	require.NotNil(t, sweep.Unresolved)
	require.Nil(t, sweep.Event)
}

func TestSourceSpherePairInitialDiagonalImpact(t *testing.T) {
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	poseB, err := r3.Translation(r3.Vec{X: 6, Y: 8})
	require.NoError(t, err)
	req := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), poseB, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 1)
	require.InDelta(t, .6, contact.Manifold.Points[0].Normal.Value.X, 1e-14)
	require.InDelta(t, .8, contact.Manifold.Points[0].Normal.Value.Y, 1e-14)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(-30),
		Y: units.MillimetersPerSecond(-40), Z: units.MillimetersPerSecond(0)}
	zero := zeroVelocity()
	angular := zeroAngular(t)
	dt := units.Seconds(.1)
	sweep, err := doc.SweepPair(t.Context(), a, b,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: dt},
		decad.RigidDriftSegment{From: poseB, Center: poseB.Translation(),
			LinearVelocity: velocity, AngularVelocity: angular, Duration: dt},
		decad.SweepRequest{ContactRequest: req, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepInitiallyTouching, sweep.Outcome)
	require.Len(t, sweep.Event.Manifold.Points, 1)
	mass := exactSphereMass()
	mat := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	cfg := dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: mat},
			{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: mat}},
		Step: dynamics.StepConfig{Contact: req, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096},
	}
	w, err := dynamics.NewWorld(t.Context(), doc, cfg)
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: zero, AngularVelocity: angular},
		{Body: b, Pose: poseB, LinearVelocity: velocity, AngularVelocity: angular},
	})
	require.NoError(t, err)
	step, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, dt)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
	require.Len(t, step.Events, 1)
	require.Equal(t, dynamics.ContactImpact, step.Events[0].Kind)
	require.Equal(t, units.Seconds(0), step.Events[0].Time)
	require.Equal(t, step.Events[0].Bracket.From, step.Events[0].Bracket.To)
	require.InDelta(t, 37.5, step.Events[0].NormalImpulse.Base(), 1e-6)
	require.NotNil(t, step.Conservation)
	require.InDelta(t, -30, step.Conservation.Completion.LinearMomentum.Value.X.Base(), 1e-6)
	require.InDelta(t, -40, step.Conservation.Completion.LinearMomentum.Value.Y.Base(), 1e-6)
	postAtZero, err := step.Trace.Sample(units.Seconds(0))
	require.NoError(t, err)
	postStateA, found := postAtZero.Body(a)
	require.True(t, found)
	postStateB, found := postAtZero.Body(b)
	require.True(t, found)
	ideal, err := doc.SweepPair(t.Context(), a, b,
		decad.RigidDriftSegment{From: postStateA.Pose,
			LinearVelocity: postStateA.LinearVelocity, AngularVelocity: angular, Duration: dt},
		decad.RigidDriftSegment{From: postStateB.Pose,
			LinearVelocity: postStateB.LinearVelocity, AngularVelocity: angular, Duration: dt},
		decad.SweepRequest{ContactRequest: req, TimeResolution: units.Seconds(1e-9),
			StartPolicy: decad.ContinueSeparatingTouch, MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, ideal.Outcome)
	finalA, found := step.Next.Body(a)
	require.True(t, found)
	finalB, found := step.Next.Body(b)
	require.True(t, found)
	rounded, err := doc.SweepPair(t.Context(), a, b,
		decad.PoseSegment{From: postStateA.Pose, To: finalA.Pose, Duration: dt},
		decad.PoseSegment{From: postStateB.Pose, To: finalB.Pose, Duration: dt},
		decad.SweepRequest{ContactRequest: req, TimeResolution: units.Seconds(1e-9),
			StartPolicy: decad.ContinueSeparatingTouch, MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, rounded.Outcome)
	for _, sample := range []struct {
		at        float64
		positionA r3.Vec
		positionB r3.Vec
	}{{0, r3.Vec{}, r3.Vec{X: 6, Y: 8}},
		{.05, r3.Vec{X: -1.125, Y: -1.5}, r3.Vec{X: 5.625, Y: 7.5}},
		{.1, r3.Vec{X: -2.25, Y: -3}, r3.Vec{X: 5.25, Y: 7}}} {
		state, err := step.Trace.Sample(units.Seconds(sample.at))
		require.NoError(t, err)
		stateA, ok := state.Body(a)
		require.True(t, ok)
		stateB, ok := state.Body(b)
		require.True(t, ok)
		require.InDelta(t, sample.positionA.X, stateA.Pose.Translation().X, 1e-8)
		require.InDelta(t, sample.positionA.Y, stateA.Pose.Translation().Y, 1e-8)
		require.InDelta(t, sample.positionB.X, stateB.Pose.Translation().X, 1e-8)
		require.InDelta(t, sample.positionB.Y, stateB.Pose.Translation().Y, 1e-8)
		require.InDelta(t, -22.5, stateA.LinearVelocity.X.Base(), 1e-6)
		require.InDelta(t, -30, stateA.LinearVelocity.Y.Base(), 1e-6)
		require.InDelta(t, -7.5, stateB.LinearVelocity.X.Base(), 1e-6)
		require.InDelta(t, -10, stateB.LinearVelocity.Y.Base(), 1e-6)
		if sample.at > 0 {
			pair, err := doc.ContactPair(t.Context(), a, b, stateA.Pose, stateB.Pose, req)
			require.NoError(t, err)
			require.Equal(t, decad.ContactSeparated, pair.Relation)
			if sample.at == .1 {
				require.InDelta(t, 2.5, pair.Gap.Value.Base(), 1e-8)
			}
		}
	}
	require.Equal(t, []*decad.Body{a, b}, doc.Bodies())
	cfg.Bodies[0], cfg.Bodies[1] = cfg.Bodies[1], cfg.Bodies[0]
	reversed, err := dynamics.NewWorld(t.Context(), doc, cfg)
	require.NoError(t, err)
	reversedStart, err := reversed.NewState([]dynamics.BodyState{
		{Body: b, Pose: poseB, LinearVelocity: velocity, AngularVelocity: angular},
		{Body: a, Pose: r3.Identity(), LinearVelocity: zero, AngularVelocity: angular},
	})
	require.NoError(t, err)
	reversedStep, err := reversed.Step(t.Context(), reversedStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, dt)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, reversedStep.Status, "%+v", reversedStep.Diagnostics)
	require.Len(t, reversedStep.Events, 1)
	require.InDelta(t, -.6, reversedStep.Events[0].Manifold.Points[0].Normal.Value.X, 1e-14)
	require.InDelta(t, -.8, reversedStep.Events[0].Manifold.Points[0].Normal.Value.Y, 1e-14)
	reversedSample, err := reversedStep.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	reversedA, ok := reversedSample.Body(a)
	require.True(t, ok)
	require.InDelta(t, -1.125, reversedA.Pose.Translation().X, 1e-8)
	shiftedMass := mass
	shiftedMass.Center.Value = r3.Vec{X: .25}
	cfg.Bodies[0].Supplied = &shiftedMass
	unproved, err := dynamics.NewWorld(t.Context(), doc, cfg)
	require.NoError(t, err)
	unprovedStart, err := unproved.NewState([]dynamics.BodyState{
		{Body: b, Pose: poseB, LinearVelocity: velocity, AngularVelocity: angular},
		{Body: a, Pose: r3.Identity(), LinearVelocity: zero, AngularVelocity: angular},
	})
	require.NoError(t, err)
	unprovedStep, err := unproved.Step(t.Context(), unprovedStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, dt)
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, unprovedStep.Status)
	require.Nil(t, unprovedStep.Next)
	require.Empty(t, unprovedStep.Events)
	cfg.Bodies[0].Supplied = &mass
	cfg.Bodies[0].Material.Restitution = units.Scalar(0)
	cfg.Bodies[1].Material.Restitution = units.Scalar(0)
	withoutRebound, err := dynamics.NewWorld(t.Context(), doc, cfg)
	require.NoError(t, err)
	withoutReboundStart, err := withoutRebound.NewState([]dynamics.BodyState{
		{Body: b, Pose: poseB, LinearVelocity: velocity, AngularVelocity: angular},
		{Body: a, Pose: r3.Identity(), LinearVelocity: zero, AngularVelocity: angular},
	})
	require.NoError(t, err)
	withoutReboundStep, err := withoutRebound.Step(t.Context(), withoutReboundStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, dt)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, withoutReboundStep.Status, "%+v", withoutReboundStep.Diagnostics)
	require.Len(t, withoutReboundStep.Events, 1)
	require.InDelta(t, 25, withoutReboundStep.Events[0].NormalImpulse.Base(), 1e-6)
	for _, body := range []*decad.Body{a, b} {
		end, ok := withoutReboundStep.Next.Body(body)
		require.True(t, ok)
		require.InDelta(t, -15, end.LinearVelocity.X.Base(), 1e-9)
		require.InDelta(t, -20, end.LinearVelocity.Y.Base(), 1e-9)
	}
	withoutReboundMid, err := withoutReboundStep.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	midA, ok := withoutReboundMid.Body(a)
	require.True(t, ok)
	midB, ok := withoutReboundMid.Body(b)
	require.True(t, ok)
	midContact, err := doc.ContactPair(t.Context(), a, b, midA.Pose, midB.Pose, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, midContact.Relation)
}
