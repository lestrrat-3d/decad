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

// bracketHeel is the L bracket's outer heel edge, the line x = 0, z = 0 along y.
func bracketHeel() *decad.EdgeQuery {
	return decad.Edges(decad.ParallelTo(r3.NewVec(0, 1, 0)), decad.EndpointAt(r3.NewVec(0, 0, 0))).Exactly(1)
}

// bracketInnerCorner is the L bracket's inner corner, the line x = 8, z = 8 along y.
func bracketInnerCorner() *decad.EdgeQuery {
	return decad.Edges(decad.ParallelTo(r3.NewVec(0, 1, 0)), decad.EndpointAt(r3.NewVec(8, 0, 8))).Exactly(1)
}

// stackedLBracket unions a 60 x 40 x 8 base with an 8 x 40 x 50 upright. The
// upright's wall meets the base's side at x = 8, a slab-merge cut at 2/15 of
// the 60 mm side, yet no recorded face coordinate reads that parameter, so the
// result's faces are exact. Its volume is 60·40·8 + 8·40·42 = 32640.
func stackedLBracket(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	doc := decad.New()
	base := boxBody(t, doc, 0, 0, 60, 40, 8)
	upright := boxBody(t, doc, 0, 0, 8, 40, 50)
	bracket, err := decad.Union(t.Context(), base, upright)
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, bracket, units.CubicMillimeters(32640), decadtest.Exactly())
	return doc, bracket
}

// bracketBaseTop names the base's top face, the plane z = 8 facing +z, by an
// origin role that selects it alone: the upright's top at z = 50 faces the
// same way, so Facing alone matches both.
func bracketBaseTop(t *testing.T, bracket *decad.Body) decad.FaceSelector {
	t.Helper()
	for _, f := range bracket.Faces() {
		plane, ok := f.Surface().(decad.Plane)
		if !ok || plane.Frame.N().Sub(r3.NewVec(0, 0, 1)).Len() > 1e-9 || math.Abs(plane.Frame.Origin().Z-8) > 1e-9 {
			continue
		}
		for _, origin := range f.Origins() {
			sel := decad.Faces(decad.FaceCreatedBy(origin))
			if faces, err := sel.SelectFaces(bracket); err == nil && len(faces) == 1 {
				return sel
			}
		}
	}
	require.FailNow(t, "no origin role selects the base top alone")
	return nil
}

func requireBracketSound(t *testing.T, doc *decad.Document) {
	t.Helper()
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

// TestStackedUnionBracketShell checks that an inward Shell of the L bracket
// with its y = 0 end open builds. The bracket's section is the 816 mm^2 L; its
// inward offset by t = 2 keeps both strips (56 x 4 and 4 x 46, overlapping in
// a 4 x 4 square, area 392) plus the reflex corner's (6,8]^2 square less the
// quarter disc of radius 2 about (8,8), 4 - pi, so 396 - pi. The cavity runs
// from the open end to the closed end's inner face, 38 mm, and the wall keeps
// 816·40 - 38·(396 - pi) = 17592 + 38·pi.
func TestStackedUnionBracketShell(t *testing.T) {
	t.Parallel()
	doc, bracket := stackedLBracket(t)
	out, err := decad.Shell(t.Context(), bracket, decad.Faces(decad.Facing(r3.NewVec(0, -1, 0))).Exactly(1),
		units.Millimeters(2))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(17592+38*math.Pi), decadtest.WithinRel(units.Scalar(1e-12)))
	requireBracketSound(t, doc)
}

// TestStackedUnionBracketChamfers checks that a chamfer on the straight edge
// of an exact stacked Union builds and measures what the cut removes or adds.
func TestStackedUnionBracketChamfers(t *testing.T) {
	t.Parallel()

	t.Run("two distances", func(t *testing.T) {
		t.Parallel()
		doc, bracket := stackedLBracket(t)
		out, err := decad.Chamfer(t.Context(), bracket, bracketHeel(), units.Millimeters(3),
			decad.WithAsymmetricChamfer(decad.Faces(decad.Facing(r3.NewVec(-1, 0, 0))), units.Millimeters(5)))
		require.NoError(t, err)
		// The bevel removes a right triangle with legs 3 and 5 along 40 mm.
		decadtest.MeasuresVolume(t, out, units.CubicMillimeters(32640-0.5*3*5*40), decadtest.Exactly())
		requireBracketSound(t, doc)
	})

	t.Run("angle", func(t *testing.T) {
		t.Parallel()
		doc, bracket := stackedLBracket(t)
		out, err := decad.ChamferAngle(t.Context(), bracket, bracketInnerCorner(), units.Millimeters(3),
			bracketBaseTop(t, bracket), units.Degrees(30))
		require.NoError(t, err)
		// The inner corner is concave, so the bevel adds a wedge with legs 3
		// and 3·tan(30°) = √3 along 40 mm: 60√3.
		decadtest.MeasuresVolume(t, out, units.CubicMillimeters(32640+60*math.Sqrt(3)),
			decadtest.WithinRel(units.Scalar(1e-12)))
		requireBracketSound(t, doc)
	})

	// The equal chamfer builds through route P along y with or without this
	// change; it is a regression guard for that route. Before the change its
	// volume read Approximate (about 1.9e-9 bound) and now reads Exact.
	t.Run("equal distance", func(t *testing.T) {
		t.Parallel()
		doc, bracket := stackedLBracket(t)
		out, err := decad.Chamfer(t.Context(), bracket, bracketHeel(), units.Millimeters(3))
		require.NoError(t, err)
		decadtest.MeasuresVolume(t, out, units.CubicMillimeters(32640-0.5*3*3*40), decadtest.Exactly())
		requireBracketSound(t, doc)
	})
}
