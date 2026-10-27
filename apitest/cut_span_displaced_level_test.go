package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// twoSidedRect extrudes the rectangle x0..x1 x y0..y1 on XY up by up and
// down by down.
func twoSidedRect(t *testing.T, doc *decad.Document, x0, y0, x1, y1 float64, up, down units.Value) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.TwoSided{
		One: decad.DistanceSide{D: up}, Two: decad.DistanceSide{D: down}})
	require.NoError(t, err)
	return body
}

// TestCutSpanDisplacedLevelFixtures pins what the tests below depend on: 0.2
// in holds the float 5.08 but denotes a rational below it, so a tool ending
// at 0.2 in stops short of a plate top at 5.08 mm.
func TestCutSpanDisplacedLevelFixtures(t *testing.T) {
	t.Parallel()
	held, err := units.Inches(0.2).In(units.Millimeter)
	require.NoError(t, err)
	require.Equal(t, 5.08, held, "both tops hold one float")
	require.Negative(t, denotedDepth(units.Inches(0.2)).Cmp(denotedDepth(units.Millimeters(5.08))),
		"0.2 in denotes a level below 5.08 mm")
}

// TestCutSpanDisplacedLevelKeepsTheSkin cuts a 20x20 plate over z -3..5.08 mm
// with a 4x4 tool over z -20 mm..0.2 in. The tool's top holds the plate top's
// float but denotes a level below it, so a skin about 7e-17 mm thick remains
// over the hole. A clean through hole would claim the skin is gone; the
// result must instead carry the denoted volume inside its bound.
func TestCutSpanDisplacedLevelKeepsTheSkin(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := twoSidedRect(t, doc, 0, 0, 20, 20, units.Millimeters(5.08), units.Millimeters(3))
	tool := twoSidedRect(t, doc, 8, 8, 12, 12, units.Inches(0.2), units.Millimeters(20))
	part, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	top, toolTop := denotedDepth(units.Millimeters(5.08)), denotedDepth(units.Inches(0.2))
	// The plate is 400 mm² over -3..top; the hole is 16 mm² over -3..toolTop.
	want := new(big.Rat).Mul(big.NewRat(400, 1), new(big.Rat).Add(top, big.NewRat(3, 1)))
	want.Sub(want, new(big.Rat).Mul(big.NewRat(16, 1), new(big.Rat).Add(toolTop, big.NewRat(3, 1))))
	requireRationalVolumeCovered(t, part, want)
	require.True(t, tookMeshPath(part), "a tool top within its displacement of the plate top is no proven span")
}

// TestCutSpanDisplacedLevelDenotedSpan cuts the same plate with a tool whose
// top denotes 0.2 in over a plate whose top also denotes 0.2 in. The two
// displaced levels denote one rational, so the span is proven and the cut is
// the analytic through hole.
func TestCutSpanDisplacedLevelDenotedSpan(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := twoSidedRect(t, doc, 0, 0, 20, 20, units.Inches(0.2), units.Millimeters(3))
	tool := twoSidedRect(t, doc, 8, 8, 12, 12, units.Inches(0.2), units.Millimeters(20))
	part, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	require.False(t, tookMeshPath(part), "equal denoted tops prove the span")
	require.Len(t, part.Faces(), 10)
	top := new(big.Rat).Add(denotedDepth(units.Inches(0.2)), big.NewRat(3, 1))
	requireRationalVolumeCovered(t, part, new(big.Rat).Mul(big.NewRat(384, 1), top))
}

// skinnedPlateVolume is a 20x20 plate over -3..top less n 4x4 tools over
// -3..toolTop each.
func skinnedPlateVolume(top, toolTop *big.Rat, n int64) *big.Rat {
	three := big.NewRat(3, 1)
	want := new(big.Rat).Mul(big.NewRat(400, 1), new(big.Rat).Add(top, three))
	return want.Sub(want, new(big.Rat).Mul(big.NewRat(16*n, 1), new(big.Rat).Add(toolTop, three)))
}

// TestCutSpanDisplacedLevelUnion unions two overlapping plates over z -3..5.08
// mm and -3 mm..0.2 in. Their tops hold one float but denote two levels, so
// no single top plane is proven; with both tops at 0.2 in it is.
func TestCutSpanDisplacedLevelUnion(t *testing.T) {
	t.Parallel()
	mm, inch := units.Millimeters(5.08), units.Inches(0.2)
	three := big.NewRat(3, 1)
	for _, tc := range []struct {
		name               string
		x0, x1             float64
		aTop, bTop         units.Value
		analytic           bool
		faces, lumps, area int64
	}{
		{"overlapping, different denoted tops", 10, 30, mm, inch, false, 0, 1, 200},
		{"overlapping, both tops 5.08 mm", 10, 30, mm, mm, true, 6, 1, 200},
		{"overlapping, both tops 0.2 in", 10, 30, inch, inch, true, 6, 1, 200},
		{"disjoint, different denoted tops", 25, 45, mm, inch, false, 0, 2, 400},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			a := twoSidedRect(t, doc, 0, 0, 20, 20, tc.aTop, units.Millimeters(3))
			b := twoSidedRect(t, doc, tc.x0, 0, tc.x1, 20, tc.bTop, units.Millimeters(3))
			got, err := decad.Union(t.Context(), a, b)
			require.NoError(t, err)
			require.Equal(t, !tc.analytic, tookMeshPath(got))
			require.Len(t, got.Lumps(), int(tc.lumps))
			if tc.analytic {
				require.Len(t, got.Faces(), int(tc.faces))
			}
			// A covers 400 mm² to its top; B adds area mm² beyond A to its own.
			want := new(big.Rat).Mul(big.NewRat(400, 1), new(big.Rat).Add(denotedDepth(tc.aTop), three))
			want.Add(want, new(big.Rat).Mul(big.NewRat(tc.area, 1), new(big.Rat).Add(denotedDepth(tc.bTop), three)))
			requireRationalVolumeCovered(t, got, want)
		})
	}
}

// TestCutSpanDisplacedLevelGroupCut cuts the 5.08 mm plate with a prism group
// of two 4x4 tools over -20 mm..0.2 in. The group's tops sit within their
// displacement of the plate top, so the group cut proves no span.
func TestCutSpanDisplacedLevelGroupCut(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := twoSidedRect(t, doc, 0, 0, 20, 20, units.Millimeters(5.08), units.Millimeters(3))
	left := twoSidedRect(t, doc, 2, 8, 6, 12, units.Inches(0.2), units.Millimeters(20))
	right := twoSidedRect(t, doc, 14, 8, 18, 12, units.Inches(0.2), units.Millimeters(20))
	group, err := decad.Union(t.Context(), left, right)
	require.NoError(t, err)
	require.False(t, tookMeshPath(group), "equal denoted tops build one group")
	require.Len(t, group.Lumps(), 2)
	part, err := decad.Cut(t.Context(), plate, group)
	require.NoError(t, err)
	require.True(t, tookMeshPath(part), "a group top within its displacement of the plate top is no proven span")
	requireRationalVolumeCovered(t, part,
		skinnedPlateVolume(denotedDepth(units.Millimeters(5.08)), denotedDepth(units.Inches(0.2)), 2))
}

// TestCutSpanDisplacedLevelStackedThroughCut pockets a plate from its top,
// then cuts it with a 4x4 tool over -20 mm..0.2 in beside the pocket. Over a
// 5.08 mm top the tool's top denotes a lower level, so the stacked through
// cut proves no span; over a 0.2 in top both denote one level and the cut
// stays analytic.
func TestCutSpanDisplacedLevelStackedThroughCut(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name     string
		top      units.Value
		analytic bool
	}{
		{"millimetre plate top", units.Millimeters(5.08), false},
		{"inch plate top", units.Inches(0.2), true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := twoSidedRect(t, doc, 0, 0, 20, 20, tc.top, units.Millimeters(3))
			// The pocket runs from z=-2 to z=10, past either plate top.
			pocket := twoSidedRect(t, doc, 2, 2, 6, 6, units.Millimeters(10), units.Millimeters(2))
			stacked, err := decad.Cut(t.Context(), plate, pocket)
			require.NoError(t, err)
			require.False(t, tookMeshPath(stacked), "the blind pocket builds a stack")
			tool := twoSidedRect(t, doc, 12, 12, 16, 16, units.Inches(0.2), units.Millimeters(20))
			part, err := decad.Cut(t.Context(), stacked, tool)
			require.NoError(t, err)
			require.Equal(t, !tc.analytic, tookMeshPath(part))
			want := skinnedPlateVolume(denotedDepth(tc.top), denotedDepth(units.Inches(0.2)), 1)
			// The pocket removes 16 mm² from z=-2 up to the plate top.
			want.Sub(want, new(big.Rat).Mul(big.NewRat(16, 1), new(big.Rat).Add(denotedDepth(tc.top), big.NewRat(2, 1))))
			requireRationalVolumeCovered(t, part, want)
		})
	}
}

// TestCutSpanDisplacedLevelSurfaceTrim trims a sheet swept over -3..5.08 mm
// with a tool over -20 mm..0.2 in. The tool's top denotes a level below the
// sheet's, so S6's span is unproven and Trim refuses; a sheet swept to 0.2
// in is spanned.
func TestCutSpanDisplacedLevelSurfaceTrim(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		top     units.Value
		refused bool
	}{
		{"millimetre sheet top", units.Millimeters(5.08), true},
		{"inch sheet top", units.Inches(0.2), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			s, p := plateSketch(t)
			sheet, err := decad.Extrude(t.Context(), doc, s, p, decad.TwoSided{
				One: decad.DistanceSide{D: tc.top}, Two: decad.DistanceSide{D: units.Millimeters(3)}},
				decad.WithSurfaceResult())
			require.NoError(t, err)
			tool := twoSidedRect(t, doc, 40, -10, 60, 70, units.Inches(0.2), units.Millimeters(20))
			trimmed, err := decad.Trim(t.Context(), sheet, tool, decad.KeepOutside)
			if tc.refused {
				require.ErrorIs(t, err, decad.ErrUnsupported)
				require.ErrorContains(t, err, "the tool does not span the receiver over the sweep parameter")
				return
			}
			require.NoError(t, err)
			require.Len(t, trimmed.Lumps(), 2)
		})
	}
}

// TestCutSpanDisplacedLevelIntersect intersects a plate over -3 mm..0.2 in
// with a block from fl(5e-17) below z=5.08 mm upward. The block's bottom lies
// below the plate top's float but above the level 0.2 in denotes, so the
// solids are disjoint and no overlap is proven.
func TestCutSpanDisplacedLevelIntersect(t *testing.T) {
	t.Parallel()
	bottom := new(big.Rat).Sub(denotedDepth(units.Millimeters(5.08)), denotedDepth(units.Millimeters(5e-17)))
	require.Positive(t, bottom.Cmp(denotedDepth(units.Inches(0.2))), "the block starts above the plate top")
	doc := decad.New()
	plate := twoSidedRect(t, doc, 0, 0, 20, 20, units.Inches(0.2), units.Millimeters(3))
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 5.08)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(4, 4, 16, 16)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	block, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(10)}, Two: decad.DistanceSide{D: units.Millimeters(5e-17)}})
	require.NoError(t, err)
	_, err = decad.Intersect(t.Context(), plate, block)
	var be *decad.BooleanError
	require.ErrorAs(t, err, &be)
	require.Equal(t, decad.BooleanEmpty, be.Code)
}
