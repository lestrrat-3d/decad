package dynamics_test

import (
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
	midpoint, err = common.ContactTrack.ManifoldAt(units.Scalar(0.5))
	require.NoError(t, err)
	require.InDelta(t, pair.Manifold.Points[0].OnA.Value.Y+0.25, midpoint.Points[0].OnA.Value.Y,
		midpoint.Points[0].OnA.Bound.Base()+1e-14)
	w := fixedBoxContactWorld(t, doc, fixed, moving, 0)
	state, err := w.NewState([]dynamics.BodyState{
		{Body: fixed, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: moving, Pose: turn, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(-50), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(50)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	step, err := w.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.01))
	require.Nil(t, step)
	require.ErrorIs(t, err, dynamics.ErrUnsupported)
}
