package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-wall-survey-design.md's public-surface tests for
// the wall survey of a faceted body, on two crossing plates.

// crossedPlates unions a 30 x 30 plate of the given thickness with a copy of it
// turned 45 degrees about the X axis through its middle, which is a mesh
// Boolean result.
func crossedPlates(t *testing.T, thickness float64) *decad.Document {
	t.Helper()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 30, 30, thickness)
	turned, err := decad.Copy(t.Context(), plate,
		decad.WithCopyRotation(r3.NewVec(15, 15, thickness/2), r3.NewVec(1, 0, 0), units.Degrees(45)))
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), plate, turned)
	require.NoError(t, err)
	require.Equal(t, "decad.facetedPayload", decadtest.PayloadClass(joined), "the fixture takes the mesh path")
	return doc
}

func wallDiagnostic(report *decad.Report, code decad.DiagnosticCode) (decad.Diagnostic, bool) {
	for _, d := range report.Diagnostics {
		if d.Code == code {
			return d, true
		}
	}
	return decad.Diagnostic{}, false
}

func TestFacetedWallSurveyProvesAThinPlateTooThin(t *testing.T) {
	t.Parallel()
	doc := crossedPlates(t, 1)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(2)))
	require.NoError(t, err)
	d, ok := wallDiagnostic(report, decad.DiagWallTooThin)
	require.True(t, ok, "a 1 mm plate is proven thinner than a 2 mm tool: %+v", report.Diagnostics)
	require.Equal(t, decad.Violating, d.Status)
	require.Equal(t, decad.Violating, report.Status)
	require.NotNil(t, d.Observed)
	require.LessOrEqual(t, d.Observed.Value.Base()+d.Observed.Bound.Base(), 2.0, "the reading's upper end is below the tool")
	require.GreaterOrEqual(t, d.Observed.Value.Base()+d.Observed.Bound.Base(), 1.0, "and at or above the true 1 mm wall")
}

func TestFacetedWallSurveyLeavesAThickEnoughPlateUndecided(t *testing.T) {
	t.Parallel()
	doc := crossedPlates(t, 1)
	// A 0.5 mm tool is thinner than the 1 mm plate. The rays only find walls, so
	// they cannot prove the plate thick enough: the survey stays undecided.
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(0.5)))
	require.NoError(t, err)
	_, thin := wallDiagnostic(report, decad.DiagWallTooThin)
	require.False(t, thin, "no wall is proven below 0.5 mm")
	_, undecided := wallDiagnostic(report, decad.DiagUndecidedWall)
	require.True(t, undecided, "the wall question stays undecided: %+v", report.Diagnostics)
	require.NotEqual(t, decad.Sound, report.Status)
	require.NotEqual(t, decad.Violating, report.Status)
}

func TestFacetedWallSurveyDoesNotCallAThickBodyThin(t *testing.T) {
	t.Parallel()
	doc := crossedPlates(t, 8)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(2)))
	require.NoError(t, err)
	_, thin := wallDiagnostic(report, decad.DiagWallTooThin)
	require.False(t, thin, "an 8 mm plate is not below a 2 mm tool: %+v", report.Diagnostics)
	require.NotEqual(t, decad.Violating, report.Status)
}
