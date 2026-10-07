package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/general-boolean-design.md §3 A4's public-API suite: the
// analytic booleans and Verify's interference reading with a reflected
// operand. Reflected bodies are built with PlacedCopy/Placed under
// r3.Reflection. The L is docs/mirror-pattern-design.md §2's: the union of
// [0, 20]×[0, 5] and [0, 5]×[5, 20], 175 mm², swept 10 mm. The re-wound
// record's mechanics are prism_boolean_reflected_internal_test.go's.

// reflectedLPts is the mirror design's L, drawn as one outer loop.
var reflectedLPts = [][2]float64{{0, 0}, {20, 0}, {20, 5}, {5, 5}, {5, 20}, {0, 20}}

// reflectedHeight is every fixture's sweep height in this file.
const reflectedHeight = 10.0

// mirrorAcrossX30 is the reflection across the world plane x = 30, which
// takes [0, 20] in x to [40, 60].
func mirrorAcrossX30(t *testing.T) r3.Transform {
	t.Helper()
	mirror, err := r3.NewFrame(r3.NewVec(30, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	refl, err := r3.Reflection(mirror)
	require.NoError(t, err)
	return refl
}

// reflectedDiscBody extrudes a circle of radius r centred at (cx, cy).
func reflectedDiscBody(t *testing.T, doc *decad.Document, cx, cy, r float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(cx, cy)
	s.Fix(center)
	s.CreateCircle(center, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(reflectedHeight), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// quarterDiscBody extrudes the quarter disc of radius r about (cx, cy) that
// spans the first quadrant: an arc counter-clockwise from (cx+r, cy) to
// (cx, cy+r) and the two radii back to the centre.
func quarterDiscBody(t *testing.T, doc *decad.Document, cx, cy, r float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(cx, cy)
	start := s.CreatePoint(cx+r, cy)
	end := s.CreatePoint(cx, cy+r)
	s.Fix(center)
	s.Fix(start)
	s.Fix(end)
	s.CreateArc(center, start, end)
	s.CreateLine(end, center)
	s.CreateLine(center, start)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profs := s.Profiles()
	require.Len(t, profs, 1)
	body, err := doc.Extrude(s, profs[0], decad.Distance{D: units.Millimeters(reflectedHeight), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestPrismCutReflectedTargetByCylinder is mirror §2's M5: the L mirrored
// across x = 30 (now [40, 60]×[0, 20]) cut by a same-plane Ø3 cylinder
// standing inside the image's leg. The pair builds analytically through the
// reflected re-expression: the cylinder's CircleSeg is re-wound into the
// mirrored L's frame, the clean-nesting match adds it as one hole, and the
// volume is 1750 − π·1.5²·10 within its bound. Shown to fail with G2's
// refusal restored (the mesh path's coplanar refusal), and with the
// re-wound CircleSeg's CCW flipped (walkOf refuses the contradicted range).
func TestPrismCutReflectedTargetByCylinder(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	l := polyPrismBody(t, doc, reflectedLPts, reflectedHeight)
	image, err := l.PlacedCopy(t.Context(), mirrorAcrossX30(t))
	require.NoError(t, err)
	tool := reflectedDiscBody(t, doc, 57.5, 12, 1.5)

	got, err := decad.Cut(t.Context(), image, tool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got), "the analytic reduction must own a reflected target")
	cyl, err := decad.Faces(decad.Cylindrical()).Exactly(1).SelectFaces(got)
	require.NoError(t, err)
	require.Len(t, cyl, 1)

	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Less(t, vol.Bound.Base(), 1e-6)
	decadtest.Measures(t, "reflected cut volume", vol, units.CubicMillimeters(1750-math.Pi*1.5*1.5*reflectedHeight))
	c, err := got.Centroid()
	require.NoError(t, err)
	require.Greater(t, c.Value.X, 40.0, "the result sits on the image side of the mirror")
}

// TestPrismCutReflectedTargetByArcTool cuts a 20×20 box mirrored across
// x = 30 (now [40, 60]×[0, 20]) by a same-plane quarter disc of radius 8
// about (45, 5). The tool's ArcSeg is re-wound as {m(C), m(E), m(S)}: the
// reflection turns the quarter arc clockwise, and the swap names the same
// quarter counter-clockwise again. The volume is 4000 − 16π·10 within its
// bound. Shown to fail with the ArcSeg re-wound as {m(C), m(S), m(E)}: the
// scene then holds the three-quarter arc, and the cut does not build.
func TestPrismCutReflectedTargetByArcTool(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 20, 20, reflectedHeight)
	image, err := box.PlacedCopy(t.Context(), mirrorAcrossX30(t))
	require.NoError(t, err)
	tool := quarterDiscBody(t, doc, 45, 5, 8)

	got, err := decad.Cut(t.Context(), image, tool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Less(t, vol.Bound.Base(), 1e-6)
	decadtest.Measures(t, "reflected arc cut volume", vol, units.CubicMillimeters(4000-16*math.Pi*reflectedHeight))
}

// TestPrismCutBothReflectedSharedAxis is mirror §2's T1: the L and a Ø3
// cylinder inside its leg, both placed by one reflection across x = 30.
// The two placements are bit-identical, so G3's shared-axis arm admits the
// pair with no re-expression, and the cut builds analytically with a bound
// of π's own rounding alone (its sectionDelta of exactly 0 is the internal
// suite's assertion). Shown to fail with G2's refusal restored.
func TestPrismCutBothReflectedSharedAxis(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	mirror := mirrorAcrossX30(t)
	l := polyPrismBody(t, doc, reflectedLPts, reflectedHeight)
	disc := reflectedDiscBody(t, doc, 2.5, 12, 1.5)
	ml, err := l.Placed(t.Context(), mirror)
	require.NoError(t, err)
	md, err := disc.Placed(t.Context(), mirror)
	require.NoError(t, err)

	got, err := decad.Cut(t.Context(), ml, md)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Less(t, vol.Bound.Base(), 1e-9)
	decadtest.Measures(t, "both-reflected cut volume", vol, units.CubicMillimeters(1750-math.Pi*1.5*1.5*reflectedHeight))
}

// TestPrismUnionReflectedOperandInsideBox unions a 30×30 box on [35, 65]×[−5, 25]
// with the L mirrored across x = 30, which lies strictly inside it. Union's
// select-all merge runs over the re-wound L and keeps the box's own outer
// loop, so the result is the box: 6 faces and 9000 mm³ within its bound,
// Approximate because the re-expression rounds the L's coordinates. Shown
// to fail with G2's refusal restored (the mesh path's coplanar refusal).
func TestPrismUnionReflectedOperandInsideBox(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 35, -5, 65, 25, reflectedHeight)
	l := polyPrismBody(t, doc, reflectedLPts, reflectedHeight)
	image, err := l.PlacedCopy(t.Context(), mirrorAcrossX30(t))
	require.NoError(t, err)

	got, err := decad.Union(t.Context(), box, image)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	require.Len(t, got.Faces(), 6)
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Less(t, vol.Bound.Base(), 1e-6)
	decadtest.Measures(t, "reflected union volume", vol, units.CubicMillimeters(9000))
}

// TestVerifyReflectedOperandMeasuresInterference is the interference twin's
// share of A4: the L mirrored across x = 30 beside a same-plane Ø3 cylinder
// standing inside the image's leg. evaluateAnalyticIntersect admits the
// reflected pair, so Verify reports one Interference row whose volume is
// π·1.5²·10 within its bound, where the mesh path's coplanar refusal left
// the pair Suspect with unsupported_pair_contact. Shown to fail with G2's
// refusal restored.
func TestVerifyReflectedOperandMeasuresInterference(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	l := polyPrismBody(t, doc, reflectedLPts, reflectedHeight)
	image, err := l.PlacedCopy(t.Context(), mirrorAcrossX30(t))
	require.NoError(t, err)
	require.NoError(t, doc.Remove(l))
	pin := reflectedDiscBody(t, doc, 57.5, 12, 1.5)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Interferences, 1)
	decadtest.MeasuresInterference(t, report, image, pin, units.CubicMillimeters(math.Pi*1.5*1.5*reflectedHeight))
	for _, d := range report.Diagnostics {
		require.NotEqual(t, decad.DiagUnsupportedPairContact, d.Code)
	}
}
