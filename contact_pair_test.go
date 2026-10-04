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
	require.Equal(t, before, doc.Bodies())
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
