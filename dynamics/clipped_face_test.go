package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestClippedRotatedFaceProductionPath(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -5, -5, 5, 5, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(45))
	require.NoError(t, err)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), turn, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.NotNil(t, contact.Manifold)
	require.Len(t, contact.Manifold.Points, 8)
	for _, point := range contact.Manifold.Points {
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
		require.InDelta(t, 0, point.OnA.Value.Z, point.OnA.Bound.Base())
		require.InDelta(t, 0, point.OnB.Value.Z, point.OnB.Bound.Base())
		require.Contains(t, floor.Faces(), point.FaceA)
		require.Contains(t, box.Faces(), point.FaceB)
	}
	reversed, err := doc.ContactPair(t.Context(), box, floor, turn, r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, reversed.Relation)
	require.Len(t, reversed.Manifold.Points, 8)
	require.Equal(t, r3.Vec{Z: -1}, reversed.Manifold.Points[0].Normal.Value)
	pathA := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(.01)}
	pathB := decad.PoseSegment{From: turn, To: turn, Duration: units.Seconds(.01)}
	sweep, err := doc.SweepPair(t.Context(), floor, box, pathA, pathB, decad.SweepRequest{
		ContactRequest: request, TimeResolution: units.Seconds(1e-9),
		MaxPoseEvaluations: 128, StartPolicy: decad.StopAtInitialContact})
	require.NoError(t, err)
	require.Equal(t, decad.SweepInitiallyTouching, sweep.Outcome)
	require.NotNil(t, sweep.Event.Manifold)
	reverseSweep, err := doc.SweepPair(t.Context(), box, floor, pathB, pathA, decad.SweepRequest{
		ContactRequest: request, TimeResolution: units.Seconds(1e-9),
		MaxPoseEvaluations: 128, StartPolicy: decad.StopAtInitialContact})
	require.NoError(t, err)
	require.Equal(t, decad.SweepInitiallyTouching, reverseSweep.Outcome)
	require.Len(t, reverseSweep.Event.Manifold.Points, 8)
	persistent, err := doc.SweepPair(t.Context(), floor, box, pathA, pathB, decad.SweepRequest{
		ContactRequest: request, TimeResolution: units.Seconds(1e-9),
		MaxPoseEvaluations: 128, StartPolicy: decad.ContinueCertifiedTouch})
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, persistent.Outcome, "cause=%v", persistent.Cause)
	require.True(t, persistent.HasAffineReplayProof())
	manifold, err := persistent.ContactTrack.ManifoldAt(units.Scalar(.5))
	require.NoError(t, err)
	require.Len(t, manifold.Points, 8)
	replayA, replayB, err := persistent.CertifiedPosesAtInterval(units.Seconds(.005),
		units.Seconds(0), units.Seconds(.01))
	require.NoError(t, err)
	require.Equal(t, r3.Identity(), replayA)
	require.Equal(t, turn, replayB)
	w := fixedBoxContactWorld(t, doc, floor, box, 0)
	state, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: turn, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-50)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	step, err := w.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.01))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
	require.Len(t, step.Events, 1)
	require.Len(t, step.Events[0].Manifold.Points, 8)
	require.InDelta(t, 50, step.Events[0].NormalImpulse.Base(), 1e-8)
	require.NotNil(t, step.Conservation)
	require.InDelta(t, 50, step.Conservation.ContactImpulse.Value.Z.Base(), 1e-8)
	end, ok := step.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, zeroVelocity(), end.LinearVelocity)
	require.Equal(t, turn, end.Pose)
	interior, err := step.Trace.Sample(units.Seconds(.005))
	require.NoError(t, err)
	interiorBox, ok := interior.Body(box)
	require.True(t, ok)
	require.Equal(t, turn, interiorBox.Pose)
	require.Equal(t, zeroVelocity(), interiorBox.LinearVelocity)
	second, err := w.Step(t.Context(), *step.Next,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.01))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, second.Status, "%+v", second.Diagnostics)
	require.Empty(t, second.Events)
}

func TestClippedRotatedFaceRefusesUnprovedContacts(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -5, -5, 5, 5, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(45))
	require.NoError(t, err)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	tight := request
	tight.PointResolution = units.Millimeters(1e-20)
	tooCoarse, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), turn, tight)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, tooCoarse.Relation)
	require.Nil(t, tooCoarse.Manifold)
	require.Equal(t, decad.ContactPointTooCoarse, tooCoarse.Reason)

	vertexFloor := makeBox(t, doc, -10, -10, 0, 0, -10, 10)
	vertexBox := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	vertex, err := doc.ContactPair(t.Context(), vertexFloor, vertexBox,
		r3.Identity(), turn, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, vertex.Relation)
	require.Nil(t, vertex.Manifold)

	shift, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	offCenter, err := turn.Then(shift)
	require.NoError(t, err)
	w := fixedBoxContactWorld(t, doc, floor, box, 0)
	state, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: offCenter, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-50)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	response, err := w.Step(t.Context(), state,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.01))
	require.NoError(t, err)
	// The turned, shifted box lands on the clipped eight-point patch, whose
	// pressure center sits under its mass center at x = 1: with no
	// restitution the patch stops it with 50 kg·mm/s and no spin.
	require.Equal(t, dynamics.Advanced, response.Status, "%+v", response.Diagnostics)
	require.Len(t, response.Events, 1)
	require.Len(t, response.Events[0].PointImpulses, 8)
	require.InDelta(t, 50, response.Events[0].NormalImpulse.Base(), 1e-6)
	moment := 0.0
	for i, point := range response.Events[0].PointImpulses {
		moment += point.Normal.Base() * response.Events[0].Manifold.Points[i].OnB.Value.X
	}
	require.InDelta(t, 50, moment, 1e-6)
	landed, ok := response.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, zeroVelocity(), landed.LinearVelocity)
	require.Equal(t, zeroAngular(t), landed.AngularVelocity)
	require.Equal(t, offCenter, landed.Pose)
}
