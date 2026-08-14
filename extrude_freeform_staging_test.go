package decad_test

import (
	"bytes"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/spline-design.md P4b's Task 4 (.tmp/a3-p4b-plan.md): now
// that a Tier A free-form section builds through the public Extrude (#179),
// a dozen OTHER capabilities reach a free-form-walled body for the first
// time. Every one of them still stages behind its own P5-or-later increment
// (docs/spline-design.md §8 Table C), and this file pins each refusal so a
// later change cannot widen one silently. Every test asserts the exact
// sentinel and the identifying part of the message, never merely "it ran"
// (CLAUDE.md's hard rules).

// freeformArchBody extrudes the fit-spline arch profile ((0,0), (4,3), (8,0)
// closed by a chord) used throughout extrude_freeform_test.go, into doc, by
// 10 mm. Its one free-form wall is the fit-spline span.
func freeformArchBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	s, p := fitSplineArchSketch(t)
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// straightNURBSPrismBody extrudes a 10x10 rectangle by 5 mm; its bottom edge
// is recorded as a degree-1 unit-weight NURBSSeg rather than a LineSeg — the
// straight-walk case (docs/spline-design.md §6.5's Table K, "K identically
// zero"). The refusal every capability below stages on is keyed to the
// RECORDED kind, never the degree or the geometric straightness, so this
// fixture must trip the identical refusal a curved free-form wall does.
func straightNURBSPrismBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	p0 := s.CreatePoint(0, 0)
	p1 := s.CreatePoint(10, 0)
	_, err = s.CreateNURBS(1, []*sketch.Point{p0, p1}, []float64{1, 1}, []float64{0, 0, 1, 1})
	require.NoError(t, err)
	p2 := s.CreatePoint(10, 10)
	p3 := s.CreatePoint(0, 10)
	s.CreateLine(p1, p2)
	s.CreateLine(p2, p3)
	s.CreateLine(p3, p0)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	body, err := doc.Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestFreeformPrismTessellateRefuses pins the Tessellate row: chording a
// free-form-walled prism is P5's own increment, so a Tier A free-form
// section that now builds still refuses at chordLoop (tessellate.go).
func TestFreeformPrismTessellateRefuses(t *testing.T) {
	doc := decad.New()
	body := freeformArchBody(t, doc)

	_, err := body.Tessellate(units.Millimeters(0.5))

	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "free-form boundary segment")
}

// TestFreeformPrismSTLRefusesUnchanged and TestFreeformPrismOBJRefusesUnchanged
// pin that STL/OBJ surface Tessellate's refusal unmodified (export.go).
func TestFreeformPrismSTLRefusesUnchanged(t *testing.T) {
	doc := decad.New()
	body := freeformArchBody(t, doc)

	var buf bytes.Buffer
	err := body.STL(&buf)

	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "free-form boundary segment")
	require.Zero(t, buf.Len(), "a refused export writes nothing")
}

func TestFreeformPrismOBJRefusesUnchanged(t *testing.T) {
	doc := decad.New()
	body := freeformArchBody(t, doc)

	var buf bytes.Buffer
	err := body.OBJ(&buf)

	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "free-form boundary segment")
	require.Zero(t, buf.Len(), "a refused export writes nothing")
}

// TestFreeformPrismBooleanStagesNotContact pins Union/Cut/Intersect: a
// free-form operand fails G4 (prismProfileIsAnalytic, prism_boolean.go)
// silently, then the mesh path's own tessellation of that operand refuses —
// a capability/staging limit reached BEFORE any contact is examined, so it
// must surface as the plain ErrUnsupported sentinel asBooleanError leaves
// unwrapped, never a *decad.BooleanError (mirrors
// TestUnionOfRevolveBodiesStagesNotContact in boolean_test.go).
func TestFreeformPrismBooleanStagesNotContact(t *testing.T) {
	for _, tc := range []struct {
		name string
		op   func(a, b *decad.Body) (*decad.Body, error)
	}{
		{"Union", decad.Union},
		{"Cut", decad.Cut},
		{"Intersect", decad.Intersect},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			a := freeformArchBody(t, doc)
			b := boxBody(t, doc, 20, 0, 30, 10, 10)

			_, err := tc.op(a, b)

			require.ErrorIs(t, err, decad.ErrUnsupported)
			var be *decad.BooleanError
			require.False(t, errors.As(err, &be),
				"a pre-contact staging limit is a plain ErrUnsupported, not a BooleanError")
		})
	}
}

// TestFreeformPrismInterferenceUndecided pins Verify's default pairwise
// interference proof over a free-form operand: DiagUnsupportedPairPayload
// (the cause) and the deprecated broad DiagUnsupportedPair both appear, both
// Suspect, and the pair never reports a proven Interference row nor a Sound
// status (interference.go / verify.go).
func TestFreeformPrismInterferenceUndecided(t *testing.T) {
	doc := decad.New()
	freeformArchBody(t, doc)
	// Bounding boxes overlap (u in [2,6] subsets the arch's u in [0,8], v in
	// [-1,1] meets the arch's v in [0,~3], z in [0,10] matches) so the pair
	// is never box-disjoint and the interference measurement actually runs.
	boxBody(t, doc, 2, -1, 6, 1, 10)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)

	require.True(t, hasDiagnostic(report, decad.DiagUnsupportedPairPayload))
	require.True(t, hasDiagnostic(report, decad.DiagUnsupportedPair))
	require.Empty(t, report.Interferences, "an undecided pair never publishes a proven overlap")
	require.NotEqual(t, decad.Sound, report.Status)
	for _, d := range report.Diagnostics {
		switch d.Code {
		case decad.DiagUnsupportedPairPayload, decad.DiagUnsupportedPair:
			require.Equal(t, decad.Suspect, d.Status)
		}
	}
}

// TestFreeformPrismClearanceUndecided pins WithClearances on a box-disjoint
// free-form pair: DiagUndecidedClearance, never a fabricated Gap and never a
// silent Sound pass (clearance.go / verify.go).
func TestFreeformPrismClearanceUndecided(t *testing.T) {
	doc := decad.New()
	freeformArchBody(t, doc)
	boxBody(t, doc, 100, 100, 110, 110, 10)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)

	require.True(t, hasDiagnostic(report, decad.DiagUndecidedClearance))
	require.Empty(t, report.Clearances, "an undecided gap never publishes a proven measurement")
	require.NotEqual(t, decad.Sound, report.Status)
}

// TestFreeformPrismMinWallThicknessUndecided pins WithMinWallThickness: a
// nil BodyReport.MinWallThickness with DiagUndecidedWall, never a silent
// pass (survey.go's errFreeformSection through survey.go's DiagUndecidedWall).
func TestFreeformPrismMinWallThicknessUndecided(t *testing.T) {
	doc := decad.New()
	freeformArchBody(t, doc)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)

	br := report.Bodies[0]
	require.Nil(t, br.MinWallThickness)
	require.NotEqual(t, decad.Sound, br.Status)
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedWall))
}

// TestFreeformPrismUndercutsUndecided pins WithPullDirection: an EMPTY
// BodyReport.Undercuts is the proven all-clear only inside a Sound report
// (verify.go's BodyReport doc comment); here it must come with
// DiagUndecidedUndercut and a non-Sound status, never read as a pass.
func TestFreeformPrismUndercutsUndecided(t *testing.T) {
	doc := decad.New()
	freeformArchBody(t, doc)

	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)

	br := report.Bodies[0]
	require.Empty(t, br.Undercuts)
	require.NotEqual(t, decad.Sound, br.Status)
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedUndercut))
}

// TestFreeformPrismMinRadiusUndecided pins WithMinRadius: a nil
// BodyReport.MinRadius with DiagUndecidedMinRadius, never a silent "no
// concave feature" pass (survey.go).
func TestFreeformPrismMinRadiusUndecided(t *testing.T) {
	doc := decad.New()
	freeformArchBody(t, doc)

	report, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)

	br := report.Bodies[0]
	require.Nil(t, br.MinRadius)
	require.NotEqual(t, decad.Sound, br.Status)
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedMinRadius))
}

// TestFreeformPrismFilletChamferRefuse pins Fillet and Chamfer's per-corner
// rewrite (fillet.go's prismCornerLoopsBudget): it walks every segment of
// every loop regardless of which corner is selected, so a free-form section
// refuses even when the selected edges are themselves the analytic vertical
// junctions. Both a curved wall and a straight-walk (degree-1) NURBSSeg wall
// must trip the SAME refusal — it is keyed on the recorded kind, not the
// degree (docs/spline-design.md §5.1/§6.5).
func TestFreeformPrismFilletChamferRefuse(t *testing.T) {
	for _, fixture := range []struct {
		name  string
		build func(t *testing.T, doc *decad.Document) *decad.Body
	}{
		{"curved fit-spline wall", func(t *testing.T, doc *decad.Document) *decad.Body {
			return freeformArchBody(t, doc)
		}},
		{"straight degree-1 NURBSSeg wall", func(t *testing.T, doc *decad.Document) *decad.Body {
			return straightNURBSPrismBody(t, doc)
		}},
	} {
		t.Run(fixture.name, func(t *testing.T) {
			t.Run("Fillet", func(t *testing.T) {
				doc := decad.New()
				body := fixture.build(t, doc)
				_, err := body.Fillet(verticalEdges(), units.Millimeters(1))
				require.ErrorIs(t, err, decad.ErrUnsupported)
				require.ErrorContains(t, err, "free-form boundary segment")
			})
			t.Run("Chamfer", func(t *testing.T) {
				doc := decad.New()
				body := fixture.build(t, doc)
				_, err := body.Chamfer(verticalEdges(), units.Millimeters(1))
				require.ErrorIs(t, err, decad.ErrUnsupported)
				require.ErrorContains(t, err, "free-form boundary segment")
			})
		})
	}
}

// TestFreeformPrismShellRefuses pins Shell (fillet.go's audit and
// shell_offset.go's own offset loop reversal), over both a curved and a
// straight-walk (degree-1) free-form wall.
func TestFreeformPrismShellRefuses(t *testing.T) {
	for _, fixture := range []struct {
		name  string
		build func(t *testing.T, doc *decad.Document) *decad.Body
	}{
		{"curved fit-spline wall", func(t *testing.T, doc *decad.Document) *decad.Body {
			return freeformArchBody(t, doc)
		}},
		{"straight degree-1 NURBSSeg wall", func(t *testing.T, doc *decad.Document) *decad.Body {
			return straightNURBSPrismBody(t, doc)
		}},
	} {
		t.Run(fixture.name, func(t *testing.T) {
			doc := decad.New()
			body := fixture.build(t, doc)
			_, err := body.Shell(bothCaps(), units.Millimeters(1))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, "free-form boundary segment")
		})
	}
}

// TestFreeformPrismCapLoopChamferRefuses pins the complete-cap-loop chamfer
// path (capblend_geom.go's oneLoopCornerLoop), reached by selecting every
// edge of one cap rather than a single corner.
func TestFreeformPrismCapLoopChamferRefuses(t *testing.T) {
	doc := decad.New()
	body := freeformArchBody(t, doc)

	_, err := body.Chamfer(capLoopEdges(body), units.Millimeters(1))

	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "free-form boundary segment")
}

// TestFreeformPrismRevolveRefuses pins Document.Revolve of a free-form
// section: rejectInteriorContact (revolve.go) walks every segment before
// deciding circularity, so it refuses on the free-form span regardless of
// where the axis sits relative to the profile.
func TestFreeformPrismRevolveRefuses(t *testing.T) {
	s, p := fitSplineArchSketch(t)
	doc := decad.New()

	_, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})

	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "free-form boundary segment")
	require.Empty(t, doc.Bodies(), "a refused Revolve registers no body")
}

// TestFreeformPrismSelectorNeverMatchesFace pins that a type-keyed FacePredicate
// (Planar, whose match is an `ok` type assertion on Surface) never counts the
// free-form NURBSSurface wall among its matches: the arch profile's two caps
// AND its straight chord wall are all analytic Plane faces, so Exactly(3)
// fails the moment the free-form wall's NURBSSurface leaks in too.
func TestFreeformPrismSelectorNeverMatchesFace(t *testing.T) {
	doc := decad.New()
	body := freeformArchBody(t, doc)
	require.Len(t, body.Faces(), 4, "two caps plus the spline wall plus the chord wall")

	faces, err := decad.Faces(decad.Planar()).Exactly(3).SelectFaces(body)
	require.NoError(t, err)
	for _, f := range faces {
		_, ok := f.Surface().(decad.NURBSSurface)
		require.False(t, ok, "the free-form wall must never satisfy a Surface type assertion it fails")
	}
}

// TestFreeformPrismSelectorNeverMatchesEdge pins that a type-keyed
// EdgePredicate (ParallelTo, whose match is an `ok` type assertion on Curve)
// never counts a free-form NURBSCurve rim: only the two analytic vertical
// junction edges are Line3, so Exactly(2) fails the moment a rim leaks in.
func TestFreeformPrismSelectorNeverMatchesEdge(t *testing.T) {
	doc := decad.New()
	body := freeformArchBody(t, doc)

	edges, err := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(2).SelectEdges(body)
	require.NoError(t, err)
	for _, e := range edges {
		_, ok := e.Curve().(decad.NURBSCurve)
		require.False(t, ok, "a free-form rim must never satisfy a Curve type assertion it fails")
	}
}
