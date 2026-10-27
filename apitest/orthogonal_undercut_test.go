package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-wall-design.md's public-surface tests for Verify's undercut survey of a faceted
// solid every face of which squares to a world axis, against a pull along that axis.

// tCap is a 10 x 10 x 4 base, a 2 x 2 post on it and a 10 x 10 cap on the post: a pull along z is blocked by the cap, a
// pull along x or y is free.
func tCap(t *testing.T) *decad.Document {
	t.Helper()
	doc := decad.New()
	base := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 4))
	post := blockIn(t, doc, r3.NewVec(4, 4, 4), r3.NewVec(6, 6, 10))
	lid := blockIn(t, doc, r3.NewVec(0, 0, 10), r3.NewVec(10, 10, 14))
	_, err := decad.Union(t.Context(), base, post, lid)
	require.NoError(t, err)
	return doc
}

func TestOrthogonalUndercutSurveyProvesAPullFree(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	_ = blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	for _, pull := range []r3.Vec{r3.NewVec(0, 0, 1), r3.NewVec(0, 0, -1), r3.NewVec(1, 0, 0), r3.NewVec(0, 3, 0)} {
		report, err := doc.Verify(t.Context(), decad.WithPullDirection(pull))
		require.NoError(t, err)
		_, undecided := wallDiagnostic(report, decad.DiagUndecidedUndercut)
		_, blocked := wallDiagnostic(report, decad.DiagUndercut)
		require.False(t, undecided || blocked, "a block is free to pull along %v: %+v", pull, report.Diagnostics)
		require.Equal(t, decad.Sound, report.Status, "%v", pull)
		require.NotNil(t, report.Bodies[0].Undercut.Faces)
		require.Empty(t, report.Bodies[0].Undercut.Faces)
	}
	// The T-shaped part pulls free along x and y.
	t1 := tCap(t)
	for _, pull := range []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, -1, 0)} {
		report, err := t1.Verify(t.Context(), decad.WithPullDirection(pull))
		require.NoError(t, err)
		require.Equal(t, decad.Sound, report.Status, "%v: %+v", pull, report.Diagnostics)
	}
}

func TestOrthogonalUndercutSurveyListsEveryBlockedFace(t *testing.T) {
	t.Parallel()
	doc := tCap(t)
	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	d, ok := wallDiagnostic(report, decad.DiagUndercut)
	require.True(t, ok, "%+v", report.Diagnostics)
	require.Equal(t, decad.Violating, d.Status)
	_, undecided := wallDiagnostic(report, decad.DiagUndecidedUndercut)
	require.False(t, undecided, "every face is decided")
	// The cap's underside and the base's top, each shadowed by the other, are the two blocked faces.
	blocked := report.Bodies[0].Undercut.Faces
	require.Len(t, blocked, 2)
	for _, f := range blocked {
		plane, isPlane := f.Surface().(decad.Plane)
		require.True(t, isPlane)
		n, err := f.NormalAt(plane.Frame.Origin())
		require.NoError(t, err)
		require.InDelta(t, 1, n.Value.Z*n.Value.Z, 1e-12, "a face square to the pull")
	}
	// A shelled box with a closed cavity: the cavity's floor and ceiling shadow each other along z.
	cavityDoc := decad.New()
	block := blockIn(t, cavityDoc, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 20))
	_, err = decad.Shell(t.Context(), block, nil, units.Millimeters(2), decad.WithNoOpenings())
	require.NoError(t, err)
	hollow, err := cavityDoc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	_, closed := wallDiagnostic(hollow, decad.DiagUndercut)
	require.True(t, closed, "a closed cavity cannot be pulled: %+v", hollow.Diagnostics)
}
