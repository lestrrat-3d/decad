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

// The Phase H slice (HS) probe: bodies built through the internal node builder
// keep pointer identity and nil-versus-empty behavior wherever a public report
// or accessor hands topology back. Identity uses slices.Contains (==), never
// require.Contains, whose deep-equality fallback would pass for a different
// pointer to equal data.

func TestPhaseHSliceContactIdentity(t *testing.T) {
	doc := decad.New()
	hex := hexPrismBody(t, doc)
	box := boxBody(t, doc, 0, 0, 4, 4, 4)
	id := r3.Identity()
	req := contactRequest()
	top := r3.Vec{X: 10, Y: -2, Z: 12}

	vertex, err := doc.ContactPair(t.Context(), hex, box, id, rotationPose(t, r3.Vec{X: 1, Y: -1}, 50, top), req)
	require.NoError(t, err)
	require.NotNil(t, vertex.Manifold)
	require.Len(t, vertex.Manifold.Points, 1)
	point := vertex.Manifold.Points[0]
	require.NotNil(t, point.FaceA)
	require.True(t, point.FaceA == point.FeatureA.Face)
	require.True(t, slices.Contains(hex.Faces(), point.FaceA), "FaceA is one of hex's own faces")
	require.Nil(t, point.FaceB)
	require.Nil(t, point.FeatureB.Face)
	require.Nil(t, point.FeatureB.Edge)
	require.NotNil(t, point.FeatureB.Vertex)
	require.True(t, slices.Contains(box.Vertices(), point.FeatureB.Vertex), "FeatureB.Vertex is one of box's own vertices")

	c, s := math.Cos(math.Pi/6), math.Sin(math.Pi/6)
	edgePose := basisPose(t, r3.Vec{X: 1}, r3.Vec{Y: c, Z: s}, r3.Vec{Y: -s, Z: c}, top)
	edge, err := doc.ContactPair(t.Context(), hex, box, id, edgePose, req)
	require.NoError(t, err)
	require.NotNil(t, edge.Manifold)
	require.NotEmpty(t, edge.Manifold.Points)
	for _, point := range edge.Manifold.Points {
		require.Nil(t, point.FaceB)
		require.NotNil(t, point.FeatureB.Edge)
		require.True(t, slices.Contains(box.Edges(), point.FeatureB.Edge), "FeatureB.Edge is one of box's own edges")
		require.True(t, slices.Contains(hex.Faces(), point.FaceA))
	}

	rim, err := doc.ContactPair(t.Context(), hex, box, id,
		rotationPose(t, r3.Vec{X: 1, Y: -1}, 50, r3.Vec{X: 10, Y: -10, Z: 12}), req)
	require.NoError(t, err)
	require.Nil(t, rim.Manifold)
	require.Equal(t, decad.ContactAmbiguousFeature, rim.Reason)
}

func TestPhaseHSliceVerifyIdentity(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 4, 4, 4)
	b := boxBody(t, doc, 10, 0, 14, 4, 4)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	row, err := report.ForBody(a)
	require.NoError(t, err)
	require.True(t, row.Body == a)
	row, err = report.ForBody(b)
	require.NoError(t, err)
	require.True(t, row.Body == b)
	require.NotNil(t, report.Interferences)
	require.Empty(t, report.Interferences)
	require.Nil(t, report.Clearances)

	overlap := decad.New()
	c := boxBody(t, overlap, 0, 0, 4, 4, 4)
	d := boxBody(t, overlap, 2, 0, 6, 4, 4)
	report, err = overlap.Verify(t.Context())
	require.NoError(t, err)
	require.NotEmpty(t, report.Interferences)
	for _, in := range report.Interferences {
		require.True(t, (in.A == c && in.B == d) || (in.A == d && in.B == c), "an interference names both bodies by identity")
	}
}

func TestPhaseHSliceMeshIdentity(t *testing.T) {
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 4, 4, 4)
	mesh, err := box.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	faces := box.Faces()
	sources := mesh.SourceFaces()
	require.NotEmpty(t, sources)
	for _, f := range sources {
		require.True(t, slices.Contains(faces, f), "every source face is one of the body's own faces")
	}
	again, err := box.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Same(t, mesh, again)
}

func TestPhaseHSliceTransformAndCopy(t *testing.T) {
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 4, 4, 4)
	move, err := r3.Translation(r3.NewVec(10, -3, 2))
	require.NoError(t, err)
	before := box.Vertices()
	volume, err := box.Volume()
	require.NoError(t, err)

	moved, err := decad.Transform(t.Context(), box, move)
	require.NoError(t, err)
	got, err := moved.Volume()
	require.NoError(t, err)
	require.Equal(t, volume, got)
	after := moved.Vertices()
	require.Len(t, after, len(before))
	for i, v := range after {
		require.Equal(t, move.Apply(before[i].Position().Value), v.Position().Value)
	}
	require.True(t, moved.Document() == doc)
	live := doc.Bodies()
	require.True(t, slices.Contains(live, moved))
	require.False(t, slices.Contains(live, box))
	_, err = decad.Transform(t.Context(), box, move)
	require.ErrorIs(t, err, decad.ErrRetiredBody)

	copied, err := decad.Copy(t.Context(), moved, decad.WithCopyPlacement(move))
	require.NoError(t, err)
	live = doc.Bodies()
	require.True(t, slices.Contains(live, moved), "Copy leaves its source live")
	require.True(t, slices.Contains(live, copied))
	require.Equal(t, moved.Origin().Role, copied.Origin().Role)
	for _, f := range copied.Faces() {
		require.True(t, f.Loops()[0].CoEdges()[0].Edge() != nil)
		require.False(t, slices.Contains(moved.Faces(), f), "a copy has its own faces")
	}
}

func TestPhaseHSliceNilAndEmpty(t *testing.T) {
	var zero decad.CoEdge
	require.Nil(t, zero.Edge())
	require.Nil(t, zero.Start())
	require.Nil(t, zero.End())

	doc := decad.New()
	require.Nil(t, doc.Bodies())
	box := boxBody(t, doc, 0, 0, 4, 4, 4)
	for _, f := range box.Faces() {
		for _, l := range f.Loops() {
			for _, ce := range l.CoEdges() {
				e := ce.Edge()
				require.True(t, slices.Contains(box.Edges(), e))
				start, end := e.Start(), e.End()
				if !ce.IsForward() {
					start, end = end, start
				}
				require.True(t, ce.Start() == start)
				require.True(t, ce.End() == end)
			}
		}
	}
	require.NoError(t, doc.Remove(box))
	require.Nil(t, doc.Bodies())

	s, profile := polygonSketch(t, [][2]float64{{0, 0}, {4, 0}, {4, 4}, {0, 4}})
	sheet, err := decad.Extrude(t.Context(), doc, s, profile,
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, sheet.Kind())
	_, err = sheet.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	free := 0
	for _, e := range sheet.Edges() {
		if e.IsFree() {
			require.Len(t, e.Faces(), 1)
			free++
		}
	}
	require.Equal(t, 8, free, "a four-wall sheet tube has four free rim edges at each end")
	require.True(t, sheet.Shells()[0].IsOpen())
}
