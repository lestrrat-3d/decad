package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestStitchStagedSurveyNamesPayloadClass pins the class name a stitched
// solid's staged wall survey prints: the wall survey has no stitch arm, so
// the message names the payload class, not the Go type that holds it.
//
// Shown to fail first: with SurveyClass deleted from internal/stitchrecord's
// Payload, the message named Payload.
func TestStitchStagedSurveyNamesPayloadClass(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	walls, bottom, top := stitchBoxSheets(t, doc, 10)
	body, err := decad.Stitch(walls, bottom, top).Do(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, body.Kind())
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	reading, err := report.ForBody(body)
	require.NoError(t, err)
	diag, found := findDiagnostic(reading.Diagnostics, decad.DiagUnsupportedSurveyPayload)
	require.True(t, found)
	require.Equal(t, "stitchPayload wall survey support is not implemented; use an analytic body or wait for wider "+
		"stitchPayload survey support", diag.Message)
}

// TestUnstitchedPlanarMeshVerifies checks the source face and its reflected copy.
func TestUnstitchedPlanarMeshVerifies(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := plateSketch(t)
	box, err := decad.Extrude(doc, s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	pieces, err := decad.Unstitch(box).Do(t.Context())
	require.NoError(t, err)
	require.NotEmpty(t, pieces)
	check := func(body *decad.Body) {
		mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.5), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		vertices, sources := mesh.Vertices(), mesh.SourceFaces()
		for i, tri := range mesh.Triangles() {
			a, b, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
			normal, err := sources[i].NormalAt(a.Add(b).Add(c).Scale(1.0 / 3))
			require.NoError(t, err)
			require.Positive(t, b.Sub(a).Cross(c.Sub(a)).Dot(normal.Value))
		}
		draft, err := body.Tessellate(t.Context(), units.Millimeters(0.5), decad.WithVerification(decad.VerifyNone))
		require.NoError(t, err)
		require.False(t, draft.BoundaryVerified())
	}
	check(pieces[0])
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	reflection, err := r3.Reflection(frame)
	require.NoError(t, err)
	placed, err := decad.Transform(pieces[0], reflection).Do(t.Context())
	require.NoError(t, err)
	check(placed)
}
