package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
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
	body, err := decad.Stitch(t.Context(), walls, bottom, top)
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

// TestTessellateRefusalNamesUnstitchPayload pins the payload class the
// unsupported-payload refusal prints for one piece of an unstitched box.
//
// Shown to fail first: with SurveyClass deleted from internal/stitchrecord's
// UnstitchPayload, the message named stitchrecord.UnstitchPayload.
func TestTessellateRefusalNamesUnstitchPayload(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := plateSketch(t)
	box, err := decad.Extrude(t.Context(), doc, s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	pieces, err := decad.Unstitch(t.Context(), box)
	require.NoError(t, err)
	require.NotEmpty(t, pieces)
	_, err = pieces[0].Tessellate(t.Context(), units.Millimeters(0.5))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, refusalText("decad.unstitchPayload"), err.Error())
}
