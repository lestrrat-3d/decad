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

func TestObliqueBoxGeometryIntegration(t *testing.T) {
	doc := decad.New()
	fixed := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	moving := makeBox(t, doc, 10, 0, 20, 10, 0, 10)
	turn, err := r3.Rotation(r3.Vec{Y: 1}, units.Degrees(45))
	require.NoError(t, err)
	contact := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	pair, err := doc.ContactPair(t.Context(), fixed, moving, turn, turn, contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, pair.Relation)
	require.Equal(t, decad.ContactNoReason, pair.Reason)
	require.NotNil(t, pair.Manifold)
	require.Len(t, pair.Manifold.Points, 4)
	for _, point := range pair.Manifold.Points {
		require.Same(t, point.FaceA, point.FeatureA.Face)
		require.Same(t, point.FaceB, point.FeatureB.Face)
		require.Contains(t, fixed.Faces(), point.FaceA)
		require.Contains(t, moving.Faces(), point.FaceB)
		require.Equal(t, point.OnA.Value, point.OnB.Value)
		require.Greater(t, point.OnA.Bound.Base(), 0.0)
		require.LessOrEqual(t, point.OnA.Bound.Base(), contact.PointResolution.Base())
		require.Greater(t, point.Normal.Value.X, 0.0)
		require.Less(t, point.Normal.Value.Z, 0.0)
		require.InDelta(t, 1, point.Normal.Value.X*point.Normal.Value.X+
			point.Normal.Value.Z*point.Normal.Value.Z, 1e-14)
		require.LessOrEqual(t, point.NormalAngle.Base(), contact.NormalResolution.Base())
		require.Zero(t, point.Separation.Value.Base())
	}
	reversed, err := doc.ContactPair(t.Context(), moving, fixed, turn, turn, contact)
	require.NoError(t, err)
	require.Len(t, reversed.Manifold.Points, 4)
	require.Less(t, reversed.Manifold.Points[0].Normal.Value.X, 0.0)
	require.Greater(t, reversed.Manifold.Points[0].Normal.Value.Z, 0.0)
	tightPoint := contact
	tightPoint.PointResolution = units.Millimeters(1e-17)
	pointRefusal, err := doc.ContactPair(t.Context(), fixed, moving, turn, turn, tightPoint)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, pointRefusal.Relation)
	require.Nil(t, pointRefusal.Manifold)
	require.Equal(t, decad.ContactPointTooCoarse, pointRefusal.Reason)
	tightNormal := contact
	tightNormal.NormalResolution = units.Radians(1e-17)
	normalRefusal, err := doc.ContactPair(t.Context(), fixed, moving, turn, turn, tightNormal)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, normalRefusal.Relation)
	require.Nil(t, normalRefusal.Manifold)
	require.Equal(t, decad.ContactNoNormalProof, normalRefusal.Reason)
	edge := makeBox(t, doc, 10, 10, 20, 20, 0, 10)
	edgeContact, err := doc.ContactPair(t.Context(), fixed, edge, turn, turn, contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, edgeContact.Relation)
	require.Nil(t, edgeContact.Manifold)
	require.Equal(t, decad.ContactAmbiguousFeature, edgeContact.Reason)
	axisAligned, err := doc.ContactPair(t.Context(), fixed, moving, r3.Identity(), r3.Identity(), contact)
	require.NoError(t, err)
	require.Equal(t, decad.Exact, axisAligned.Manifold.Points[0].OnA.Exactness)
	require.Zero(t, axisAligned.Manifold.Points[0].OnA.Bound.Base())
	path := decad.PoseSegment{From: turn, To: turn, Duration: units.Seconds(0.01)}
	firstTouch, err := doc.SweepPair(t.Context(), fixed, moving, path, path, decad.SweepRequest{
		ContactRequest: contact, TimeResolution: units.Seconds(1e-9),
		MaxPoseEvaluations: 128, StartPolicy: decad.StopAtInitialContact,
	})
	require.NoError(t, err)
	require.Equal(t, decad.SweepInitiallyTouching, firstTouch.Outcome)
	require.NotNil(t, firstTouch.Event.Manifold)
	sweep, err := doc.SweepPair(t.Context(), fixed, moving, path, path, decad.SweepRequest{
		ContactRequest: contact, TimeResolution: units.Seconds(1e-9),
		MaxPoseEvaluations: 128, StartPolicy: decad.ContinueCertifiedTouch,
	})
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, sweep.Outcome)
	require.NotNil(t, sweep.ContactTrack)
	midpoint, err := sweep.ContactTrack.ManifoldAt(units.Scalar(0.5))
	require.NoError(t, err)
	require.Len(t, midpoint.Points, 4)
	require.Equal(t, pair.Manifold.Points[0].OnA.Value, midpoint.Points[0].OnA.Value)
	shift, err := r3.Translation(r3.Vec{Y: 0.5})
	require.NoError(t, err)
	to, err := turn.Then(shift)
	require.NoError(t, err)
	commonPath := decad.PoseSegment{From: turn, To: to, Duration: units.Seconds(0.01)}
	common, err := doc.SweepPair(t.Context(), fixed, moving, commonPath, commonPath, decad.SweepRequest{
		ContactRequest: contact, TimeResolution: units.Seconds(1e-9),
		MaxPoseEvaluations: 128, StartPolicy: decad.ContinueCertifiedTouch,
	})
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, common.Outcome, "cause=%v", common.Cause)
	require.True(t, common.HasAffineReplayProof())
	commonA, commonB, err := common.CertifiedPosesAtInterval(units.Seconds(.003),
		units.Seconds(0), units.Seconds(.01))
	require.NoError(t, err)
	require.Equal(t, commonA, commonB)
	require.InDelta(t, turn.Translation().Y+.15, commonA.Translation().Y, 1e-15)
	midpoint, err = common.ContactTrack.ManifoldAt(units.Scalar(0.5))
	require.NoError(t, err)
	require.InDelta(t, pair.Manifold.Points[0].OnA.Value.Y+0.25, midpoint.Points[0].OnA.Value.Y,
		midpoint.Points[0].OnA.Bound.Base()+1e-14)
	w := fixedBoxContactWorld(t, doc, fixed, moving, 0)
	incoming := dynamics.QuantityVec{X: units.MillimetersPerSecond(-50),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(50)}
	approach, err := doc.SweepPair(t.Context(), fixed, moving,
		decad.PoseSegment{From: turn, To: turn, Duration: units.Seconds(0.01)},
		decad.RigidDriftSegment{From: turn, Center: turn.Apply(r3.Vec{X: 15, Y: 5, Z: 5}),
			LinearVelocity: incoming, AngularVelocity: zeroAngular(t), Duration: units.Seconds(0.01)},
		decad.SweepRequest{ContactRequest: contact, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128, StartPolicy: decad.StopAtInitialContact})
	require.NoError(t, err)
	require.Equal(t, decad.SweepInitiallyTouching, approach.Outcome)
	require.NotNil(t, approach.Event.Manifold)
	_, _, err = approach.CertifiedPosesAtInterval(units.Seconds(.005),
		units.Seconds(0), units.Seconds(.01))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	outgoing := dynamics.QuantityVec{X: units.MillimetersPerSecond(50),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(-50)}
	departure, err := doc.SweepPair(t.Context(), fixed, moving,
		decad.PoseSegment{From: turn, To: turn, Duration: units.Seconds(0.01)},
		decad.RigidDriftSegment{From: turn, Center: turn.Apply(r3.Vec{X: 15, Y: 5, Z: 5}),
			LinearVelocity: outgoing, AngularVelocity: zeroAngular(t), Duration: units.Seconds(0.01)},
		decad.SweepRequest{ContactRequest: contact, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128, StartPolicy: decad.ContinueSeparatingTouch})
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, departure.Outcome, "cause=%v", departure.Cause)
	require.NotNil(t, departure.Departure)
	require.Greater(t, departure.Departure.GapAtUntil.Value.Base()-
		departure.Departure.GapAtUntil.Bound.Base(), 0.0)
	support, err := doc.SweepPair(t.Context(), fixed, moving,
		decad.PoseSegment{From: turn, To: turn, Duration: units.Seconds(0.01)},
		decad.RigidDriftSegment{From: turn, Center: turn.Apply(r3.Vec{X: 15, Y: 5, Z: 5}),
			LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t), Duration: units.Seconds(0.01)},
		decad.SweepRequest{ContactRequest: contact, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128, StartPolicy: decad.ContinueCertifiedTouch})
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, support.Outcome, "cause=%v", support.Cause)
	require.True(t, support.HasAffineReplayProof())
	poseA, poseB, err := support.CertifiedPosesAtInterval(units.Seconds(.003),
		units.Seconds(0), units.Seconds(.01))
	require.NoError(t, err)
	require.Equal(t, turn, poseA)
	require.Equal(t, turn, poseB)
	state, err := w.NewState([]dynamics.BodyState{
		{Body: fixed, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: moving, Pose: turn, LinearVelocity: incoming, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	step, err := w.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.01))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
	require.Len(t, step.Events, 1)
	require.InDelta(t, math.Sqrt(5000), step.Events[0].NormalImpulse.Base(), 1e-6)
	require.NotNil(t, step.Conservation)
	require.InDelta(t, 50, step.Conservation.ContactImpulse.Value.X.Base(), 1e-6)
	final, ok := step.Next.Body(moving)
	require.True(t, ok)
	require.Equal(t, turn, final.Pose)
	require.Equal(t, zeroVelocity(), final.LinearVelocity)
	replayed, err := step.Trace.Sample(units.Seconds(0.01))
	require.NoError(t, err)
	replayedBox, ok := replayed.Body(moving)
	require.True(t, ok)
	require.Equal(t, final.Pose, replayedBox.Pose)
	require.Equal(t, final.LinearVelocity, replayedBox.LinearVelocity)
	for _, elapsed := range []units.Value{units.Seconds(0), units.Seconds(.003),
		units.Seconds(.005), units.Seconds(.01)} {
		sample, sampleErr := step.Trace.Sample(elapsed)
		require.NoError(t, sampleErr)
		body, found := sample.Body(moving)
		require.True(t, found)
		require.Equal(t, turn, body.Pose)
		require.Equal(t, zeroVelocity(), body.LinearVelocity)
	}
	repeated, err := w.Step(t.Context(), *step.Next,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.01))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, repeated.Status, "%+v", repeated.Diagnostics)
	require.Empty(t, repeated.Events)
	require.Greater(t, step.Conservation.ContactImpulse.Bound.X.Base(), 0.0)
}

func TestObliqueSupportRefusesUnresolvedMotion(t *testing.T) {
	doc := decad.New()
	fixed := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	moving := makeBox(t, doc, 10, 0, 20, 10, 0, 10)
	turn, err := r3.Rotation(r3.Vec{Y: 1}, units.Degrees(45))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{Y: 1})
	require.NoError(t, err)
	offCenter, err := turn.Then(shift)
	require.NoError(t, err)
	incoming := dynamics.QuantityVec{X: units.MillimetersPerSecond(-50),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(50)}
	// A tangent velocity along the tilted face keeps going: restitution 0.5
	// reverses the normal approach, 1.5·√5000 kg·mm/s, and the Y slide stays.
	t.Run("tangent velocity", func(t *testing.T) {
		w := fixedBoxContactWorld(t, doc, fixed, moving, .5)
		state, stateErr := w.NewState([]dynamics.BodyState{
			{Body: fixed, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
			{Body: moving, Pose: turn, LinearVelocity: dynamics.QuantityVec{
				X: incoming.X, Y: units.MillimetersPerSecond(10), Z: incoming.Z}, AngularVelocity: zeroAngular(t)},
		})
		require.NoError(t, stateErr)
		step, stepErr := w.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()},
			units.Seconds(.01))
		require.NoError(t, stepErr)
		require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
		require.Len(t, step.Events, 1)
		require.InDelta(t, 1.5*math.Sqrt(5000), step.Events[0].NormalImpulse.Base(), 1e-6)
		end, ok := step.Next.Body(moving)
		require.True(t, ok)
		require.InDelta(t, 25, end.LinearVelocity.X.Base(), 1e-6)
		require.Equal(t, units.MillimetersPerSecond(10), end.LinearVelocity.Y)
		require.InDelta(t, -25, end.LinearVelocity.Z.Base(), 1e-6)
		require.Equal(t, zeroAngular(t), end.AngularVelocity)
	})
	for _, fixture := range []struct {
		name     string
		pose     r3.Transform
		velocity dynamics.QuantityVec
		spin     bool
	}{
		// Shifted 1 mm along the face, the box's center still lies over the
		// patch: the impulses shift toward it and no spin results.
		{name: "off-center face", pose: offCenter, velocity: incoming},
		// Spin about Y, which lies in the face, gives the face points normal
		// speeds that vary linearly across it; restitution 0.5 at every
		// point reverses both the approach and the spin by half.
		{name: "spinning face", pose: turn, velocity: incoming, spin: true},
	} {
		t.Run(fixture.name, func(t *testing.T) {
			w := fixedBoxContactWorld(t, doc, fixed, moving, .5)
			angular := zeroAngular(t)
			if fixture.spin {
				angular.Y = units.RadiansPerSecond(1)
			}
			state, stateErr := w.NewState([]dynamics.BodyState{
				{Body: fixed, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
				{Body: moving, Pose: fixture.pose, LinearVelocity: fixture.velocity,
					AngularVelocity: angular},
			})
			require.NoError(t, stateErr)
			step, stepErr := w.Step(t.Context(), state,
				dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.01))
			require.NoError(t, stepErr)
			require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
			require.Len(t, step.Events, 1)
			require.InDelta(t, 1.5*math.Sqrt(5000), step.Events[0].NormalImpulse.Base(), 1e-6)
			end, ok := step.Next.Body(moving)
			require.True(t, ok)
			require.InDelta(t, 25, end.LinearVelocity.X.Base(), 1e-6)
			require.InDelta(t, -25, end.LinearVelocity.Z.Base(), 1e-6)
			wantSpin := 0.0
			if fixture.spin {
				wantSpin = -.5
			}
			require.InDelta(t, wantSpin, end.AngularVelocity.Y.Base(), 1e-9)
		})
	}
	edge := makeBox(t, doc, 10, 10, 20, 20, 0, 10)
	w := fixedBoxContactWorld(t, doc, fixed, edge, .5)
	state, err := w.NewState([]dynamics.BodyState{
		{Body: fixed, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: edge, Pose: turn, LinearVelocity: incoming, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), state,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.01))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
}

func TestObliqueCenteredReboundAndNextStep(t *testing.T) {
	doc := decad.New()
	fixed := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	moving := makeBox(t, doc, 10, 0, 20, 10, 0, 10)
	turn, err := r3.Rotation(r3.Vec{Y: 1}, units.Degrees(45))
	require.NoError(t, err)
	w := fixedBoxContactWorld(t, doc, fixed, moving, .5)
	incoming := dynamics.QuantityVec{X: units.MillimetersPerSecond(-50),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(50)}
	state, err := w.NewState([]dynamics.BodyState{
		{Body: fixed, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: moving, Pose: turn, LinearVelocity: incoming, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.01))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 1.5*math.Sqrt(5000), report.Events[0].NormalImpulse.Base(), 1e-6)
	require.Equal(t, 4, len(report.Events[0].PointImpulses))
	end, ok := report.Next.Body(moving)
	require.True(t, ok)
	require.InDelta(t, 25, end.LinearVelocity.X.Base(), 1e-9)
	require.InDelta(t, -25, end.LinearVelocity.Z.Base(), 1e-9)
	require.InDelta(t, .25, end.Pose.Translation().X, 1e-9)
	require.InDelta(t, -.25, end.Pose.Translation().Z, 1e-9)
	require.NotNil(t, report.Conservation)
	require.InDelta(t, 75, report.Conservation.ContactImpulse.Value.X.Base(), 1e-6)
	require.InDelta(t, -75, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	require.InDelta(t, 625, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-6)
	second, err := w.Step(t.Context(), *report.Next, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.01))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, second.Status, "%+v", second.Diagnostics)
	require.Empty(t, second.Events)
	end, ok = second.Next.Body(moving)
	require.True(t, ok)
	require.InDelta(t, .5, end.Pose.Translation().X, 1e-9)
	require.InDelta(t, -.5, end.Pose.Translation().Z, 1e-9)
}

func TestObliqueSupportReverseWorldOrder(t *testing.T) {
	doc := decad.New()
	dynamic := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	fixed := makeBox(t, doc, 10, 0, 20, 10, 0, 10)
	turn, err := r3.Rotation(r3.Vec{Y: 1}, units.Degrees(45))
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: dynamic, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: fixed, Role: dynamics.Fixed, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0),
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	state, err := w.NewState([]dynamics.BodyState{
		{Body: dynamic, Pose: turn, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(50), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-50)}, AngularVelocity: zeroAngular(t)},
		{Body: fixed, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()},
		units.Seconds(.01))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 1.5*math.Sqrt(5000), report.Events[0].NormalImpulse.Base(), 1e-6)
	final, ok := report.Next.Body(dynamic)
	require.True(t, ok)
	require.InDelta(t, -25, final.LinearVelocity.X.Base(), 1e-9)
	require.InDelta(t, 25, final.LinearVelocity.Z.Base(), 1e-9)
	require.InDelta(t, -75, report.Conservation.ContactImpulse.Value.X.Base(), 1e-6)
	second, err := w.Step(t.Context(), *report.Next,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.01))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, second.Status, "%+v", second.Diagnostics)
	require.Empty(t, second.Events)
}
