package decad_test

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func contactRequest() decad.ContactRequest {
	return decad.ContactRequest{
		PointResolution:  units.Millimeters(1e-6),
		NormalResolution: units.Degrees(1),
	}
}

func contactPose(t *testing.T, v r3.Vec) r3.Transform {
	t.Helper()
	pose, err := r3.Translation(v)
	require.NoError(t, err)
	return pose
}

func TestContactPairSourceBoxes(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 0, 0, 10, 10, 10)
	before := doc.Bodies()
	id := r3.Identity()
	req := contactRequest()

	separated, err := doc.ContactPair(t.Context(), a, b, id, contactPose(t, r3.Vec{Z: 13}), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, separated.Relation)
	require.NotNil(t, separated.Gap)
	require.Equal(t, decad.Exact, separated.Gap.Exactness)
	require.Equal(t, 3.0, separated.Gap.Value.Base())
	require.Nil(t, separated.Manifold)
	diagonal, err := doc.ContactPair(t.Context(), a, b, id,
		contactPose(t, r3.Vec{X: 13, Y: 14}), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, diagonal.Relation)
	require.NotNil(t, diagonal.Gap)
	require.LessOrEqual(t, diagonal.Gap.Value.Base()-diagonal.Gap.Bound.Base(), 5.0)
	require.GreaterOrEqual(t, diagonal.Gap.Value.Base()+diagonal.Gap.Bound.Base(), 5.0)
	require.Nil(t, diagonal.Manifold)

	touching, err := doc.ContactPair(t.Context(), a, b, id, contactPose(t, r3.Vec{Z: 10}), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, touching.Relation)
	require.Equal(t, decad.Exact, touching.Gap.Exactness)
	require.Equal(t, 0.0, touching.Gap.Value.Base())
	require.NotNil(t, touching.Manifold)
	require.Len(t, touching.Manifold.Points, 4)
	want := []r3.Vec{{X: 0, Y: 0, Z: 10}, {X: 10, Y: 0, Z: 10},
		{X: 10, Y: 10, Z: 10}, {X: 0, Y: 10, Z: 10}}
	for i, point := range touching.Manifold.Points {
		require.Equal(t, want[i], point.OnA.Value)
		require.Equal(t, want[i], point.OnB.Value)
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
		require.Equal(t, 0.0, point.Normal.Bound.Base())
		require.Equal(t, 0.0, point.Separation.Value.Base())
		require.NotNil(t, point.FaceA)
		require.NotNil(t, point.FaceB)
		require.Contains(t, a.Faces(), point.FaceA)
		require.Contains(t, b.Faces(), point.FaceB)
	}
	reversed, err := doc.ContactPair(t.Context(), b, a, contactPose(t, r3.Vec{Z: 10}), id, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, reversed.Relation)
	for i, point := range reversed.Manifold.Points {
		require.Equal(t, touching.Manifold.Points[i].OnB.Value, point.OnA.Value)
		require.Equal(t, touching.Manifold.Points[i].OnA.Value, point.OnB.Value)
		require.Equal(t, r3.Vec{Z: -1}, point.Normal.Value)
	}
	edge, err := doc.ContactPair(t.Context(), a, b, id,
		contactPose(t, r3.Vec{X: 10, Z: 10}), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, edge.Relation)
	require.Nil(t, edge.Manifold)
	require.Equal(t, decad.ContactAmbiguousFeature, edge.Reason)
	require.Equal(t, before, doc.Bodies())

	fractional := boxBody(t, doc, 0.2, 0, 10.2, 10, 10)
	beforeFractional := doc.Bodies()
	coarseReq := req
	coarseReq.PointResolution = units.Millimeters(1e-20)
	coarse, err := doc.ContactPair(t.Context(), a, fractional, id,
		contactPose(t, r3.Vec{X: 0.1, Z: 10}), coarseReq)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, coarse.Relation)
	require.Nil(t, coarse.Manifold)
	require.Equal(t, decad.ContactPointTooCoarse, coarse.Reason)
	require.Equal(t, beforeFractional, doc.Bodies())
}

func TestContactPairSourceSphereAndBox(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -20, -20, 20, 20, -10, 10)
	ball := ballBody(t, doc, 5)
	before := doc.Bodies()
	req := contactRequest()
	for _, tc := range []struct {
		z        float64
		relation decad.ContactRelation
		gap      float64
		sep      float64
	}{
		{15, decad.ContactSeparated, 10, 0},
		{5, decad.ContactTouching, 0, 0},
		{4.5, decad.ContactOverlapping, 0, -0.5},
	} {
		pose := contactPose(t, r3.Vec{Z: tc.z})
		report, err := doc.ContactPair(t.Context(), floor, ball, r3.Identity(), pose, req)
		require.NoError(t, err)
		require.Equal(t, tc.relation, report.Relation, "z=%v, reason=%v", tc.z, report.Reason)
		if tc.relation == decad.ContactSeparated {
			require.NotNil(t, report.Gap)
			require.InDelta(t, tc.gap, report.Gap.Value.Base(), 1e-12)
			continue
		}
		require.NotNil(t, report.Manifold, "z=%v, reason=%v", tc.z, report.Reason)
		require.Len(t, report.Manifold.Points, 1)
		point := report.Manifold.Points[0]
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
		require.InDelta(t, tc.sep, point.Separation.Value.Base(), 1e-12)
		require.Contains(t, floor.Faces(), point.FaceA)
		require.Same(t, ball.Faces()[0], point.FaceB)
	}
	turn, err := r3.Rotation(r3.Vec{Y: 1}, units.Radians(1))
	require.NoError(t, err)
	spinning, err := r3.FromBasis(turn.Basis(), r3.Vec{Z: 5})
	require.NoError(t, err)
	rotated, err := doc.ContactPair(t.Context(), floor, ball, r3.Identity(), spinning, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, rotated.Relation)
	require.Len(t, rotated.Manifold.Points, 1)
	edge, err := doc.ContactPair(t.Context(), floor, ball, r3.Identity(),
		contactPose(t, r3.Vec{X: 25}), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, edge.Relation)
	require.Nil(t, edge.Manifold)
	require.Equal(t, decad.ContactAmbiguousFeature, edge.Reason)
	diagonal, err := doc.ContactPair(t.Context(), floor, ball, r3.Identity(),
		contactPose(t, r3.Vec{X: 23, Z: 5}), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, diagonal.Relation)
	require.NotNil(t, diagonal.Gap)
	require.LessOrEqual(t, diagonal.Gap.Value.Base()-diagonal.Gap.Bound.Base(), 0.8309518948453005)
	require.GreaterOrEqual(t, diagonal.Gap.Value.Base()+diagonal.Gap.Bound.Base(), 0.8309518948453005)
	require.Equal(t, before, doc.Bodies())
}

func TestContactPairSourceSpherePair(t *testing.T) {
	doc := decad.New()
	a, b := ballBody(t, doc, 5), ballBody(t, doc, 5)
	before := doc.Bodies()
	for _, tc := range []struct {
		x        float64
		relation decad.ContactRelation
		gap      float64
		sep      float64
	}{
		{12, decad.ContactSeparated, 2, 0},
		{10, decad.ContactTouching, 0, 0},
		{9.5, decad.ContactOverlapping, 0, -0.5},
	} {
		pose := contactPose(t, r3.Vec{X: tc.x})
		report, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), pose, contactRequest())
		require.NoError(t, err)
		require.Equal(t, tc.relation, report.Relation, "x=%v, reason=%v", tc.x, report.Reason)
		if tc.relation == decad.ContactSeparated {
			require.NotNil(t, report.Gap)
			require.InDelta(t, tc.gap, report.Gap.Value.Base(), 1e-12)
			require.Nil(t, report.Manifold)
			continue
		}
		require.NotNil(t, report.Manifold, "x=%v, reason=%v", tc.x, report.Reason)
		require.Len(t, report.Manifold.Points, 1)
		point := report.Manifold.Points[0]
		require.Equal(t, r3.Vec{X: 1}, point.Normal.Value)
		require.InDelta(t, tc.sep, point.Separation.Value.Base(), 1e-12)
		require.Same(t, a.Faces()[0], point.FaceA)
		require.Same(t, b.Faces()[0], point.FaceB)
		require.Equal(t, r3.Vec{X: 5}, point.OnA.Value)
		require.Equal(t, r3.Vec{X: tc.x - 5}, point.OnB.Value)
		reversed, err := doc.ContactPair(t.Context(), b, a, pose, r3.Identity(), contactRequest())
		require.NoError(t, err)
		require.Equal(t, tc.relation, reversed.Relation)
		require.Equal(t, r3.Vec{X: -1}, reversed.Manifold.Points[0].Normal.Value)
		require.Equal(t, point.OnB.Value, reversed.Manifold.Points[0].OnA.Value)
	}
	corner, err := doc.ContactPair(t.Context(), a, b, r3.Identity(),
		contactPose(t, r3.Vec{X: 6, Y: 8}), contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, corner.Relation)
	require.Len(t, corner.Manifold.Points, 1)
	require.InDelta(t, .6, corner.Manifold.Points[0].Normal.Value.X, 1e-14)
	require.InDelta(t, .8, corner.Manifold.Points[0].Normal.Value.Y, 1e-14)
	require.LessOrEqual(t, corner.Manifold.Points[0].NormalAngle.Base(), contactRequest().NormalResolution.Base())
	tight := contactRequest()
	tight.NormalResolution = units.Radians(1e-18)
	tooTight, err := doc.ContactPair(t.Context(), a, b, r3.Identity(),
		contactPose(t, r3.Vec{X: 6, Y: 8}), tight)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, tooTight.Relation)
	require.Nil(t, tooTight.Manifold)
	require.Equal(t, decad.ContactNoNormalProof, tooTight.Reason)
	coincident, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), r3.Identity(), contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, coincident.Relation)
	require.Nil(t, coincident.Manifold)
	require.Equal(t, before, doc.Bodies())
}

func TestContactPairAnalyticPrismGap(t *testing.T) {
	doc := decad.New()
	a := rodBody(t, doc, 0, 0, 2, 5)
	b := rodBody(t, doc, 20, 0, 2, 5)
	before := doc.Bodies()
	id := r3.Identity()

	report, err := doc.ContactPair(t.Context(), a, b, id, id, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, report.Relation)
	require.NotNil(t, report.Gap)
	require.LessOrEqual(t, report.Gap.Value.Base()-report.Gap.Bound.Base(), 16.0)
	require.GreaterOrEqual(t, report.Gap.Value.Base()+report.Gap.Bound.Base(), 16.0)
	require.Nil(t, report.Manifold)
	require.Equal(t, decad.ContactNoReason, report.Reason)
	require.Equal(t, before, doc.Bodies())
}

func TestContactPairAnalyticPrismTouch(t *testing.T) {
	doc := decad.New()
	a := rodBody(t, doc, 0, 0, 2, 5)
	b := boxBodyAtZ(t, doc, -5, -5, 5, 5, 5, 10)
	before := doc.Bodies()
	id := r3.Identity()

	report, err := doc.ContactPair(t.Context(), a, b, id, id, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, report.Relation)
	require.NotNil(t, report.Gap)
	require.Equal(t, decad.Exact, report.Gap.Exactness)
	require.Zero(t, report.Gap.Value.Base())
	require.NotNil(t, report.Manifold)
	require.Len(t, report.Manifold.Points, 1)
	require.Equal(t, r3.Vec{Z: 1}, report.Manifold.Points[0].Normal.Value)
	require.NotNil(t, report.Manifold.Points[0].FaceA)
	require.NotNil(t, report.Manifold.Points[0].FaceB)
	require.Equal(t, decad.ContactNoReason, report.Reason)
	require.Equal(t, before, doc.Bodies())
}

func TestContactPairAnalyticPrismOverlap(t *testing.T) {
	doc := decad.New()
	a := rodBody(t, doc, 0, 0, 2, 5)
	b := boxBodyAtZ(t, doc, -1, -1, 1, 1, -2, 10)
	id := r3.Identity()

	report, err := doc.ContactPair(t.Context(), a, b, id, id, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, report.Relation)
	require.Nil(t, report.Gap)
	require.Nil(t, report.Manifold)
	require.Equal(t, decad.ContactNoNormalProof, report.Reason)
}

func TestContactPairShallowOverlapAndRefusals(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 0, 0, 10, 10, 10)
	req := contactRequest()
	id := r3.Identity()

	overlap, err := doc.ContactPair(t.Context(), a, b, id, contactPose(t, r3.Vec{Z: 9.5}), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, overlap.Relation)
	require.Nil(t, overlap.Gap)
	require.NotNil(t, overlap.Manifold)
	require.Len(t, overlap.Manifold.Points, 4)
	for _, point := range overlap.Manifold.Points {
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
		require.Equal(t, 10.0, point.OnA.Value.Z)
		require.Equal(t, 9.5, point.OnB.Value.Z)
		require.Equal(t, -0.5, point.Separation.Value.Base())
	}

	tied, err := doc.ContactPair(t.Context(), a, b, id, contactPose(t, r3.Vec{X: 5, Z: 5}), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, tied.Relation)
	require.Nil(t, tied.Manifold)
	require.Equal(t, decad.ContactAmbiguousFeature, tied.Reason)

	contained := boxBody(t, doc, 2, 2, 8, 8, 6)
	inside, err := doc.ContactPair(t.Context(), a, contained, id, contactPose(t, r3.Vec{Z: 2}), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, inside.Relation)
	require.Nil(t, inside.Manifold)

	_, err = doc.ContactPair(t.Context(), a, a, id, id, req)
	require.True(t, errors.Is(err, decad.ErrDegenerate))
	_, err = doc.ContactPair(t.Context(), a, b, id, id,
		decad.ContactRequest{PointResolution: units.Millimeters(1), NormalResolution: units.Millimeters(1)})
	require.True(t, errors.Is(err, decad.ErrUnitKind))
}

func TestContactPairSignedAxisPose(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 0, 0, 10, 10, 10)
	turn, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{Y: 1}, EY: r3.Vec{X: -1}, EZ: r3.Vec{Z: 1},
	}, r3.Vec{X: 10, Z: 10})
	require.NoError(t, err)
	report, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), turn, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, report.Relation)
	require.Len(t, report.Manifold.Points, 4)
}

func TestContactPairPlacedBodyAndInputGates(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 0, 0, 10, 10, 10)
	placed, err := b.Placed(t.Context(), contactPose(t, r3.Vec{X: 20}))
	require.NoError(t, err)
	req := contactRequest()
	id := r3.Identity()
	report, err := doc.ContactPair(t.Context(), a, placed, id,
		contactPose(t, r3.Vec{X: -20, Z: 10}), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, report.Relation)
	require.Len(t, report.Manifold.Points, 4)

	edge, err := doc.ContactPair(t.Context(), a, placed, id,
		contactPose(t, r3.Vec{X: -10, Z: 10}), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, edge.Relation)
	require.Nil(t, edge.Manifold)
	require.Equal(t, decad.ContactAmbiguousFeature, edge.Reason)

	_, err = doc.ContactPair(t.Context(), a, b, id, id, req)
	require.ErrorIs(t, err, decad.ErrRetiredBody)
	other := decad.New()
	foreign := boxBody(t, other, 0, 0, 10, 10, 10)
	_, err = doc.ContactPair(t.Context(), a, foreign, id, id, req)
	require.ErrorIs(t, err, decad.ErrForeignBody)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	got, err := doc.ContactPair(ctx, a, placed, id, id, req)
	require.Nil(t, got)
	require.ErrorIs(t, err, context.Canceled)
}
