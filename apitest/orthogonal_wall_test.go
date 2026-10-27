package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-wall-design.md's public-surface tests for Verify's wall survey of a faceted
// solid every face of which squares to a world axis.

func wallReport(t *testing.T, doc *decad.Document, tool float64) *decad.Report {
	t.Helper()
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(tool)))
	require.NoError(t, err)
	return report
}

func TestOrthogonalWallSurveyProvesAThickEnoughWall(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 4, 4))
	b := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(4, 10, 4))
	_, err := decad.Union(t.Context(), a, b) // an L whose legs are 4 mm thick: the thinnest wall is 4 mm
	require.NoError(t, err)
	// A 2 mm tool is under the 4 mm wall: the survey proves it, where the rays could only leave it undecided.
	report := wallReport(t, doc, 2)
	_, thin := wallDiagnostic(report, decad.DiagWallTooThin)
	_, undecided := wallDiagnostic(report, decad.DiagUndecidedWall)
	require.False(t, thin, "%+v", report.Diagnostics)
	require.False(t, undecided, "the wall is proven thick enough: %+v", report.Diagnostics)
	require.Equal(t, decad.Sound, report.Status, "nothing is left undecided: %+v", report.Diagnostics)
	// A 5 mm tool is over it: proven too thin, with the exact 4 mm reading.
	report = wallReport(t, doc, 5)
	d, thin := wallDiagnostic(report, decad.DiagWallTooThin)
	require.True(t, thin, "%+v", report.Diagnostics)
	require.Equal(t, decad.Violating, d.Status)
	require.InDelta(t, 4, d.Observed.Value.Base(), 1e-9)
	require.Less(t, d.Observed.Bound.Base(), 1e-9, "the reading is exact, not an interval from zero")
	// A tool of exactly the wall's thickness is not proven either way.
	report = wallReport(t, doc, 4)
	_, undecided = wallDiagnostic(report, decad.DiagUndecidedWall)
	_, thin = wallDiagnostic(report, decad.DiagWallTooThin)
	require.True(t, undecided || !thin, "a wall equal to the tool is not below it")
}

func TestOrthogonalWallSurveyFindsAThinFinAndAShelledWall(t *testing.T) {
	t.Parallel()
	// A 1 mm wall of a cup: the shell's wall is the thinnest.
	cupDoc := decad.New()
	block := blockIn(t, cupDoc, r3.NewVec(0, 0, 0), r3.NewVec(40, 30, 20))
	_, err := decad.Shell(t.Context(), block, decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(1))
	require.NoError(t, err)
	report := wallReport(t, cupDoc, 2)
	d, thin := wallDiagnostic(report, decad.DiagWallTooThin)
	require.True(t, thin, "%+v", report.Diagnostics)
	require.InDelta(t, 1, d.Observed.Value.Base(), 1e-9)
	report = wallReport(t, cupDoc, 0.5)
	_, thin = wallDiagnostic(report, decad.DiagWallTooThin)
	_, undecided := wallDiagnostic(report, decad.DiagUndecidedWall)
	require.False(t, thin || undecided, "1 mm walls are proven thicker than a 0.5 mm tool: %+v", report.Diagnostics)
	// A thick block with a 0.5 mm fin: the fin is the thinnest wall, which a centre ray might miss.
	finDoc := decad.New()
	base := blockIn(t, finDoc, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 20))
	fin := blockIn(t, finDoc, r3.NewVec(20, 3, 3), r3.NewVec(40, 3.5, 17))
	_, err = decad.Union(t.Context(), base, fin)
	require.NoError(t, err)
	report = wallReport(t, finDoc, 2)
	d, thin = wallDiagnostic(report, decad.DiagWallTooThin)
	require.True(t, thin, "%+v", report.Diagnostics)
	require.InDelta(t, 0.5, d.Observed.Value.Base(), 1e-9)
}
