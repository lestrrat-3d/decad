package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is the undercut survey over draft bodies (docs/draft-design.md
// Table DD row DD7, the PR 4 fixtures of §11). A wall drafted at α leans its
// outward normal n̂·cos α + e·sin α, so against a pull along the sweep e its
// component is sin α: positive for a narrowing taper, which clears, and
// negative for a flare, which opposes. A cap's component is ±1 or 0 and is
// decided exactly.
//
// Shown to fail first: with the draftPayload arm removed from runSurveys
// (survey.go), every fixture here read Coverage Unavailable with
// DiagUnsupportedSurveyPayload and no listed face.

// draftSurvey verifies doc under the pull and returns body's report.
func draftSurvey(t *testing.T, doc *decad.Document, body *decad.Body, pull r3.Vec) (*decad.Report, *decad.BodyReport) {
	t.Helper()
	report, err := doc.Verify(t.Context(), decad.WithPullDirection(pull))
	require.NoError(t, err)
	return report, decadtest.FindBodyReport(t, report, body)
}

// draftWalls is every face of b carrying a side(i, j) role.
func draftWalls(b *decad.Body) []*decad.Face {
	var out []*decad.Face
	for _, f := range b.Faces() {
		if isWall(f) {
			out = append(out, f)
		}
	}
	return out
}

// requireDraftClear asserts the proven all-clear: every face decided and none
// opposing.
func requireDraftClear(t *testing.T, report *decad.Report, br *decad.BodyReport) {
	t.Helper()
	require.Equal(t, decad.CoverageComplete, br.Undercut.Coverage)
	require.NotNil(t, br.Undercut.Faces)
	require.Empty(t, br.Undercut.Faces)
	require.False(t, hasDiagnostic(report, decad.DiagUndercut))
	require.False(t, hasDiagnostic(report, decad.DiagUndecidedUndercut))
	require.False(t, hasDiagnostic(report, decad.DiagUnsupportedSurveyPayload))
}

// requireDraftWallsListed asserts that exactly the drafted walls of b are
// proven undercuts and nothing is left undecided: the two caps lie along the
// pull and clear.
func requireDraftWallsListed(t *testing.T, report *decad.Report, br *decad.BodyReport, b *decad.Body) {
	t.Helper()
	walls := draftWalls(b)
	require.NotEmpty(t, walls)
	require.Equal(t, decad.CoverageComplete, br.Undercut.Coverage)
	require.ElementsMatch(t, walls, br.Undercut.Faces)
	require.True(t, hasDiagnostic(report, decad.DiagUndercut))
	require.False(t, hasDiagnostic(report, decad.DiagUndecidedUndercut))
	require.Equal(t, decad.Violating, br.Status)
}

// TestDraftUndercutBox is F1 under the pull along the sweep and against it:
// along +e every wall reads sin 5° > 0 and the body is clear; along −e every
// wall reads −sin 5° and is listed, while the far cap faces exactly away
// from the pull and the near cap exactly along it.
func TestDraftUndercutBox(t *testing.T) {
	t.Parallel()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, boxSide))
	doc := decad.New()
	b := taperExtrude(t, doc, s, p, boxHeight, 5, decad.Along)
	require.Len(t, draftWalls(b), 4)

	report, br := draftSurvey(t, doc, b, r3.NewVec(0, 0, 1))
	requireDraftClear(t, report, br)
	require.Equal(t, decad.Sound, br.Status)

	report, br = draftSurvey(t, doc, b, r3.NewVec(0, 0, -1))
	requireDraftWallsListed(t, report, br, b)
}

// TestDraftUndercutFlare is F5: a negative taper widens the body away from
// the sketch plane, so the readings of F1 swap.
func TestDraftUndercutFlare(t *testing.T) {
	t.Parallel()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, boxSide))
	doc := decad.New()
	b := taperExtrude(t, doc, s, p, boxHeight, -5, decad.Along)

	report, br := draftSurvey(t, doc, b, r3.NewVec(0, 0, 1))
	requireDraftWallsListed(t, report, br, b)

	report, br = draftSurvey(t, doc, b, r3.NewVec(0, 0, -1))
	requireDraftClear(t, report, br)
}

// TestDraftUndercutSlot is F3: the two Cone walls are decided outright in both
// directions, through the patch's whole window, never left undecided.
func TestDraftUndercutSlot(t *testing.T) {
	t.Parallel()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSlot(0, 0))
	doc := decad.New()
	b := taperExtrude(t, doc, s, p, 8, 3, decad.Along)
	planes, cones := facesByKind(b)
	require.Len(t, planes, 2)
	require.Len(t, cones, 2)

	report, br := draftSurvey(t, doc, b, r3.NewVec(0, 0, 1))
	requireDraftClear(t, report, br)

	report, br = draftSurvey(t, doc, b, r3.NewVec(0, 0, -1))
	requireDraftWallsListed(t, report, br, b)
	for _, c := range cones {
		require.Contains(t, br.Undercut.Faces, c)
	}
}

// TestDraftUndercutRing is F7: the hole widens toward the far end, so its
// wall leans its normal, which points into the hole, toward the sweep e and
// clears along it like the narrowing outer wall does. A second, wider ring
// swept Against (F6's direction) reads the same about e = −z.
func TestDraftUndercutRing(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		r, hole float64
		dir     decad.Direction
		e       r3.Vec
	}{
		{"F7 along", 10, 4, decad.Along, r3.NewVec(0, 0, 1)},
		{"wide ring against", 25, 6, decad.Against, r3.NewVec(0, 0, -1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, p := taperSketch(t, r3.NewVec(0, 0, 0), 1, drawRing(tc.r, tc.hole))
			doc := decad.New()
			b := taperExtrude(t, doc, s, p, 10, 5, tc.dir)
			_, cones := facesByKind(b)
			require.Len(t, cones, 2, "the outer wall and the hole wall")

			report, br := draftSurvey(t, doc, b, tc.e)
			requireDraftClear(t, report, br)

			report, br = draftSurvey(t, doc, b, tc.e.Scale(-1))
			requireDraftWallsListed(t, report, br, b)
		})
	}
}

// TestDraftUndercutPlaced is F1 under F9's placement: the survey reads each
// wall and cap through the placed frame, so the placed sweep direction clears
// and its negation lists every wall.
func TestDraftUndercutPlaced(t *testing.T) {
	t.Parallel()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, boxSide))
	doc := decad.New()
	b := taperExtrude(t, doc, s, p, boxHeight, 5, decad.Along)
	motion := generalMotion(t, 23, -7.5)
	placed, err := b.Placed(t.Context(), motion)
	require.NoError(t, err)
	e := motion.ApplyDir(r3.NewVec(0, 0, 1))

	report, br := draftSurvey(t, doc, placed, e)
	requireDraftClear(t, report, br)

	report, br = draftSurvey(t, doc, placed, e.Scale(-1))
	requireDraftWallsListed(t, report, br, placed)
}

// TestDraftUndercutBodyDraft reads the survey on Body.Draft's result, whose
// roles are minted under its own producer: D1's drafted box clears along the
// sweep and lists every wall against it.
func TestDraftUndercutBodyDraft(t *testing.T) {
	t.Parallel()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, boxSide))
	doc := decad.New()
	box, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(boxHeight), Dir: decad.Along})
	require.NoError(t, err)
	b, err := box.Draft(t.Context(), decad.Faces(decad.Walls(box)), capNeutral(box, decad.CapStart), units.Degrees(5))
	require.NoError(t, err)

	report, br := draftSurvey(t, doc, b, r3.NewVec(0, 0, 1))
	requireDraftClear(t, report, br)

	report, br = draftSurvey(t, doc, b, r3.NewVec(0, 0, -1))
	requireDraftWallsListed(t, report, br, b)
}

// TestDraftUndercutTangentPullUndecided keeps the survey reject-only. The
// pull lies in the plane of one drafted wall, perpendicular to its published
// normal, so that wall's component is zero to within the bound its reading
// carries and is neither a proven undercut nor proven clear.
//
// Shown to fail first: with capPatchUndercuts deciding every patch on its
// sampled range alone (the allowance capPatchNormalRange returns dropped),
// the wall read a sign of its own rounding and the survey reported no
// DiagUndecidedUndercut.
func TestDraftUndercutTangentPullUndecided(t *testing.T) {
	t.Parallel()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, boxSide))
	doc := decad.New()
	b := taperExtrude(t, doc, s, p, boxHeight, 5, decad.Along)
	walls := draftWalls(b)
	require.NotEmpty(t, walls)
	wall := walls[0]
	n, err := wall.NormalAt(wall.Loops()[0].CoEdges()[0].Start().Position().Value)
	require.NoError(t, err)
	bound, err := n.Bound.In(units.One)
	require.NoError(t, err)
	require.Positive(t, bound, "a drafted wall's normal carries the taper's enclosure")

	// Horizontal and perpendicular to the wall's published normal.
	pull := r3.NewVec(n.Value.Y, -n.Value.X, 0)
	unit, ok := pull.Normalize()
	require.True(t, ok)
	require.Less(t, math.Abs(n.Value.Dot(unit)), bound,
		"the pull lies within the wall's own bound of its tangent plane")

	report, br := draftSurvey(t, doc, b, pull)
	require.NotContains(t, br.Undercut.Faces, wall)
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedUndercut))
}
