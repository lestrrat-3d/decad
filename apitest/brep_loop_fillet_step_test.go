package apitest_test

import (
	"bytes"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// filletSTEP writes body to STEP text through the public export API.
func filletSTEP(t *testing.T, body *decad.Body) string {
	t.Helper()
	var buf bytes.Buffer
	require.NoError(t, export.STEP(t.Context(), &buf, body, units.Millimeters(0.1),
		export.WithSTEPName("fillet"), export.WithSTEPAuthor("test"), export.WithSTEPOrganization("test")))
	return buf.String()
}

// countEntity counts the STEP entity instances of one type: the name opens a
// parenthesised instance after the instance's '='.
func countEntity(step, name string) int {
	return strings.Count(step, "="+name+"(") + strings.Count(step, "= "+name+"(")
}

// TestBrepLoopFilletPublicSTEP exports loop-fillet bodies through the public
// API (loop-fillet DF4, DF10): P3's root fillet writes its whole-turn torus as
// one TOROIDAL_SURFACE, P1's top-loop fillet (r = 2) writes four ELLIPSE edges
// for its four mitres, and P2's pocket-mouth fillet, whose horn tori close on
// their axes, still exports and writes no TOROIDAL_SURFACE.
func TestBrepLoopFilletPublicSTEP(t *testing.T) {
	t.Parallel()
	t.Run("P3 root", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		plate := boxBody(t, doc, 0, 0, 40, 40, 10)
		w := sketch.NewWorld()
		plane, err := w.CreateOffsetPlane(w.XY(), 10)
		require.NoError(t, err)
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		c := s.CreatePoint(20, 20)
		s.Fix(c)
		s.CreateCircle(c, 5)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		boss, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(15), Dir: decad.Along})
		require.NoError(t, err)
		part, err := decad.Union(t.Context(), plate, boss)
		require.NoError(t, err)
		top := planeFacing(t, part, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 10))
		got, err := part.Fillet(t.Context(), loopEdgeQuery(top.Loops()[1]), units.Millimeters(1))
		require.NoError(t, err)
		step := filletSTEP(t, got)
		require.Equal(t, 1, countEntity(step, "TOROIDAL_SURFACE"))
	})
	t.Run("P1 top loop", func(t *testing.T) {
		t.Parallel()
		_, bar := crossDrilledBar(t)
		top := planeFacing(t, bar, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 20))
		got, err := bar.Fillet(t.Context(), loopEdgeQuery(top.Loops()[0]), units.Millimeters(2))
		require.NoError(t, err)
		step := filletSTEP(t, got)
		require.Equal(t, 4, countEntity(step, "ELLIPSE"))
	})
	t.Run("P2 mouth", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		plate := boxBody(t, doc, 0, 0, 40, 40, 10)
		pocket, err := decad.Cut(t.Context(), plate, boxBodyAtZ(t, doc, 10, 15, 30, 25, 5, 5))
		require.NoError(t, err)
		top := planeFacing(t, pocket, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 10))
		got, err := pocket.Fillet(t.Context(), loopEdgeQuery(top.Loops()[1]), units.Millimeters(1.5))
		require.NoError(t, err)
		step := filletSTEP(t, got)
		require.Zero(t, countEntity(step, "TOROIDAL_SURFACE"))
	})
}
