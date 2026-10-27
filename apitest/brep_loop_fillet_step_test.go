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
		boss, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(15), Dir: decad.Along})
		require.NoError(t, err)
		part, err := decad.Union(t.Context(), plate, boss)
		require.NoError(t, err)
		top := planeFacing(t, part, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 10))
		got, err := decad.Fillet(t.Context(), part, loopEdgeQuery(top.Loops()[1]), units.Millimeters(1))
		require.NoError(t, err)
		step := filletSTEP(t, got)
		require.Equal(t, 1, countEntity(step, "TOROIDAL_SURFACE"))
	})
	t.Run("P1 top loop", func(t *testing.T) {
		t.Parallel()
		_, bar := crossDrilledBar(t)
		top := planeFacing(t, bar, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 20))
		got, err := decad.Fillet(t.Context(), bar, loopEdgeQuery(top.Loops()[0]), units.Millimeters(2))
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
		got, err := decad.Fillet(t.Context(), pocket, loopEdgeQuery(top.Loops()[1]), units.Millimeters(1.5))
		require.NoError(t, err)
		step := filletSTEP(t, got)
		require.Zero(t, countEntity(step, "TOROIDAL_SURFACE"))
	})
}

// TestBrepLoopFilletPublicRoundedPlateAnalyticSTEP fillets P8's top loop by 2
// (loop-fillet DF10) and exports it through the public API. The loop's eight
// corners are tangent joins of a line and a quarter arc, and the fillet holds
// each join's foot at the exact point the record denotes, so every vertex of
// the body is Exact and every arc of the four torus patches holds both its
// vertices in its own plane, by exact rational arithmetic on the held
// coordinates. STEP then writes the body analytic, the four patches as
// TOROIDAL_SURFACE. ReadSTEP reads no torus face, so the file is checked by
// its surfaces. Shown to fail with filletband's exactFeet G1 arm deleted (the
// meridian at the (0, 0) corner held its Center at y = 2.9999999999999996
// beside its end vertex at y = 3, eight contour vertices read Approximate, and
// the body was written faceted).
func TestBrepLoopFilletPublicRoundedPlateAnalyticSTEP(t *testing.T) {
	t.Parallel()
	plate := roundedDrilledPlate(t)
	top := planeFacing(t, plate, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 20))
	got, err := decad.Fillet(t.Context(), plate, loopEdgeQuery(top.Loops()[0]), units.Millimeters(2))
	require.NoError(t, err)
	for _, v := range got.Vertices() {
		require.Equal(t, decad.Exact, v.Position().Exactness, "the vertex at %v", v.Position().Value)
	}
	arcs := 0
	for _, f := range got.Faces() {
		if _, ok := f.Surface().(decad.Torus); !ok {
			continue
		}
		for _, ce := range f.Loops()[0].CoEdges() {
			arc, ok := ce.Edge().Curve().(decad.Arc3)
			require.True(t, ok)
			c, axis := arcRatOf(arc.Center), arcRatOf(arc.Axis)
			for _, p := range []r3.Vec{ce.Edge().Start().Position().Value, ce.Edge().End().Position().Value} {
				require.Zero(t, arcRatOf(p).sub(c).dot(axis).Sign(), "the arc at %v about %v: the vertex %v is off its plane", arc.Center, arc.Axis, p)
			}
			arcs++
		}
	}
	require.Equal(t, 16, arcs, "four torus patches of four arcs")
	step := filletSTEP(t, got)
	require.Contains(t, step, "analytic decad solid")
	require.Equal(t, 4, countEntity(step, "TOROIDAL_SURFACE"))
}
