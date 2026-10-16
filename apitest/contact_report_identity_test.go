package apitest_test

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// requireMember requires p to be one of the pointers in owned, compared with
// ==, so an equal copy of an element fails. Its failure names counts only.
func requireMember[T any](t *testing.T, owned []*T, p *T, what string) {
	t.Helper()
	if slices.Contains(owned, p) {
		return
	}
	require.Failf(t, "not the body's own topology", "%s is none of the body's %d elements", what, len(owned))
}

// requireFeatureMember requires every non-nil member of f to be one of the
// body's own faces, edges or vertices.
func requireFeatureMember(t *testing.T, body *decad.Body, f decad.ContactFeature, what string) {
	t.Helper()
	require.True(t, f.Face != nil || f.Edge != nil || f.Vertex != nil, "%s names a feature", what)
	if f.Face != nil {
		requireMember(t, body.Faces(), f.Face, what+".Face")
	}
	if f.Edge != nil {
		requireMember(t, body.Edges(), f.Edge, what+".Edge")
	}
	if f.Vertex != nil {
		requireMember(t, body.Vertices(), f.Vertex, what+".Vertex")
	}
}

// requirePointsOwned requires every point's faces and features to be the two
// bodies' own topology: A's for the A side, B's for the B side.
func requirePointsOwned(t *testing.T, a, b *decad.Body, points []decad.ContactPoint) {
	t.Helper()
	require.NotEmpty(t, points)
	for _, point := range points {
		if point.FaceA != nil {
			requireMember(t, a.Faces(), point.FaceA, "FaceA")
		}
		if point.FaceB != nil {
			requireMember(t, b.Faces(), point.FaceB, "FaceB")
		}
		requireFeatureMember(t, a, point.FeatureA, "FeatureA")
		requireFeatureMember(t, b, point.FeatureB, "FeatureB")
	}
}

// TestContactPairReportKeepsTopologyIdentity requires a report's bodies to be
// the bodies passed in, and every face, edge and vertex it names to be the
// very pointer the body's own Faces, Edges or Vertices returns: a face patch
// of two touching boxes, and a vertex and an edge touch of a turned box on a
// hexagonal prism.
func TestContactPairReportKeepsTopologyIdentity(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 0, 0, 10, 10, 10)
	touching, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), contactPose(t, r3.Vec{Z: 10}), contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, touching.Relation)
	require.Same(t, a, touching.A)
	require.Same(t, b, touching.B)
	require.NotNil(t, touching.Manifold)
	require.Len(t, touching.Manifold.Points, 4)
	requirePointsOwned(t, a, b, touching.Manifold.Points)
	for _, point := range touching.Manifold.Points {
		require.NotNil(t, point.FaceA)
		require.NotNil(t, point.FaceB)
		require.Same(t, point.FaceA, point.FeatureA.Face)
		require.Same(t, point.FaceB, point.FeatureB.Face)
	}

	hex := hexPrismBody(t, doc)
	box := boxBody(t, doc, 0, 0, 4, 4, 4)
	top := r3.Vec{X: 10, Y: -2, Z: 12}
	vertex, err := doc.ContactPair(t.Context(), hex, box, r3.Identity(),
		rotationPose(t, r3.Vec{X: 1, Y: -1}, 50, top), contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, vertex.Relation)
	require.Same(t, hex, vertex.A)
	require.Same(t, box, vertex.B)
	require.NotNil(t, vertex.Manifold)
	require.Len(t, vertex.Manifold.Points, 1)
	require.NotNil(t, vertex.Manifold.Points[0].FeatureB.Vertex, "premise: a vertex touch")
	requirePointsOwned(t, hex, box, vertex.Manifold.Points)

	c, s := math.Cos(math.Pi/6), math.Sin(math.Pi/6)
	edgePose := basisPose(t, r3.Vec{X: 1}, r3.Vec{Y: c, Z: s}, r3.Vec{Y: -s, Z: c}, top)
	edge, err := doc.ContactPair(t.Context(), hex, box, r3.Identity(), edgePose, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, edge.Relation)
	require.NotNil(t, edge.Manifold)
	require.Len(t, edge.Manifold.Points, 2)
	for _, point := range edge.Manifold.Points {
		require.NotNil(t, point.FeatureB.Edge, "premise: an edge touch")
	}
	requirePointsOwned(t, hex, box, edge.Manifold.Points)
}

// TestContactPairKeepsAbsentResultsNil requires a result a report does not
// carry to stay nil: a separated pair's manifold and overlap, and the
// manifold of a touch whose contact is an ambiguous edge.
func TestContactPairKeepsAbsentResultsNil(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 0, 0, 10, 10, 10)
	separated, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), contactPose(t, r3.Vec{Z: 13}), contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, separated.Relation)
	require.NotNil(t, separated.Gap)
	require.Nil(t, separated.Manifold)
	require.Nil(t, separated.Overlap)
	edge, err := doc.ContactPair(t.Context(), a, b, r3.Identity(),
		contactPose(t, r3.Vec{X: 10, Z: 10}), contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, edge.Relation)
	require.Equal(t, decad.ContactAmbiguousFeature, edge.Reason)
	require.Nil(t, edge.Manifold)
	require.Nil(t, edge.Overlap)
}

// TestContactPairMemoHitKeepsTopologyIdentity repeats a touching query twice: the
// memo serves a distinct report with a distinct manifold, whose bodies and
// faces are still the very pointers the first report and the bodies hold.
func TestContactPairMemoHitKeepsTopologyIdentity(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 0, 0, 10, 10, 10)
	pose := contactPose(t, r3.Vec{Z: 10})
	first, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), pose, contactRequest())
	require.NoError(t, err)
	second, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), pose, contactRequest())
	require.NoError(t, err)
	require.NotSame(t, first, second)
	require.NotNil(t, second.Manifold)
	require.NotSame(t, first.Manifold, second.Manifold)
	require.Len(t, second.Manifold.Points, len(first.Manifold.Points))
	require.NotSame(t, &first.Manifold.Points[0], &second.Manifold.Points[0])
	require.Same(t, a, second.A)
	require.Same(t, b, second.B)
	for i, point := range second.Manifold.Points {
		require.Same(t, first.Manifold.Points[i].FaceA, point.FaceA)
		require.Same(t, first.Manifold.Points[i].FaceB, point.FaceB)
	}
	requirePointsOwned(t, a, b, second.Manifold.Points)
	third, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), pose, contactRequest())
	require.NoError(t, err)
	require.NotSame(t, second, third, "two memo hits share no report")
	require.NotSame(t, second.Manifold, third.Manifold)
	require.NotSame(t, &second.Manifold.Points[0], &third.Manifold.Points[0])
	requirePointsOwned(t, a, b, third.Manifold.Points)
}

// TestSweepPairTrackKeepsTopologyIdentity requires a persistent slide's
// samples, contact track features and track manifolds to name the two
// bodies' own faces.
func TestSweepPairTrackKeepsTopologyIdentity(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	floor := boxBody(t, doc, 0, -5, 20, 15, 10)
	top := boxBodyAtZ(t, doc, 5, 0, 15, 10, 10, 10)
	req := sweepRequest()
	req.StartPolicy = decad.ContinueCertifiedTouch
	report, err := doc.SweepPair(t.Context(), floor, top,
		sweepDrift(r3.Vec{}, 1), sweepDrift(r3.Vec{X: 1}, 1), req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, report.Outcome)
	require.Same(t, floor, report.A)
	require.Same(t, top, report.B)
	require.NotNil(t, report.ContactTrack)
	featureA, featureB := report.ContactTrack.Features()
	require.NotNil(t, featureA.Face)
	require.NotNil(t, featureB.Face)
	requireFeatureMember(t, floor, featureA, "track FeatureA")
	requireFeatureMember(t, top, featureB, "track FeatureB")
	manifold, err := report.ContactTrack.ManifoldAt(units.Scalar(0.5))
	require.NoError(t, err)
	requirePointsOwned(t, floor, top, manifold.Points)
	for _, point := range manifold.Points {
		require.Same(t, featureA.Face, point.FaceA)
		require.Same(t, featureB.Face, point.FaceB)
	}
	sampled := 0
	for _, sample := range report.Samples {
		if sample.Ideal.Manifold == nil {
			continue
		}
		requirePointsOwned(t, floor, top, sample.Ideal.Manifold.Points)
		sampled++
	}
	require.Positive(t, sampled, "premise: a sample carries an ideal manifold")
}
