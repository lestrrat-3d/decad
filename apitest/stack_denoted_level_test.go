package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The tests in this file reach a denoted level that a stack, a stacked union,
// a chain sheet, a Split piece, a tube shell or a pattern group records
// (agent-docs/stacked-prism-design.md §2.1). Each fixture places the
// undenoted side so that the held floats alone cannot decide it: 0.2 in
// denotes a rational below its held 5.08, and 0.3 in one above its held
// 7.619999999999999.

const (
	stackedPrismClass = "decad.stackedPrismPayload"
	prismClass        = "decad.prismPayload"
	brepClass         = "decad.brepPayload"
	sweepSpanMiss     = "the tool does not span the target over the sweep parameter"
)

// lowStack unions a 20x20 plate over -3..2 mm with a 10x10 boss over -3 mm..0.2
// in at 5..15, which builds a stack whose top denotes 0.2 in.
func lowStack(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	plate := twoSidedRect(t, doc, 0, 0, 20, 20, units.Millimeters(2), units.Millimeters(3))
	boss := twoSidedRect(t, doc, 5, 5, 15, 15, units.Inches(0.2), units.Millimeters(3))
	stack, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	require.Equal(t, stackedPrismClass, decadtest.PayloadClass(stack), "the union builds a stack")
	return stack
}

// stackTopTool extrudes the 4x4 square 38..42 x 8..12 on XY up to stack's top
// cap and down 20 mm.
func stackTopTool(t *testing.T, doc *decad.Document, stack *decad.Body) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(38, 8, 42, 12)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.TwoSided{
		One: decad.ToFace{Body: stack, Face: decad.Faces(decad.FaceCreatedBy(decad.CapEnd(stack)))},
		Two: decad.DistanceSide{D: units.Millimeters(20)}})
	require.NoError(t, err)
	return tool
}

// scaled is k·(v + add).
func scaled(k int64, v *big.Rat, add int64) *big.Rat {
	out := new(big.Rat).Add(v, big.NewRat(add, 1))
	return out.Mul(out, big.NewRat(k, 1))
}

// TestStackedUnionToFaceDenotedLevel cuts a plate whose top is 0.2 in with a
// tool that stops at a stacked union's top, which denotes 0.2 in. The stack's
// top cap records its level's denotation, so the stop proves the span and
// the cut is the analytic through hole. A plate top at 5.08 mm holds the same
// float but denotes another plane, so no span is proven.
func TestStackedUnionToFaceDenotedLevel(t *testing.T) {
	t.Parallel()
	d := denotedDepth(units.Inches(0.2))
	t.Run("0.2 in top", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		stack := lowStack(t, doc)
		plate := twoSidedRect(t, doc, 30, 0, 50, 20, units.Inches(0.2), units.Millimeters(3))
		part, err := decad.Cut(t.Context(), plate, stackTopTool(t, doc, stack))
		require.NoError(t, err)
		require.Equal(t, prismClass, decadtest.PayloadClass(part), "the stop denotes the plate's top")
		require.Len(t, part.Lumps(), 1)
		require.Len(t, part.Faces(), 10)
		requireRationalVolumeCovered(t, part, scaled(384, d, 3))
	})
	t.Run("5.08 mm top", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		stack := lowStack(t, doc)
		plate := twoSidedRect(t, doc, 30, 0, 50, 20, units.Millimeters(5.08), units.Millimeters(3))
		part, err := decad.Cut(t.Context(), plate, stackTopTool(t, doc, stack))
		requireDeclinedSpan(t, part, err, skinnedPlateVolume(denotedDepth(units.Millimeters(5.08)), d, 1), false)
	})
}

// TestStackedUnionChainedDenotedLevel unions lowStack with a second boss at
// 12..18 x 8..12 whose top ties the stack's top. Both tops denote 0.2 in, so
// the tie merges and the result is a stack: 20x20x5 below z=2, and the two
// bosses' 112 mm² union above it. With the second top at 5.08 mm the tops hold
// one float and denote two planes, so the union takes the mesh path.
func TestStackedUnionChainedDenotedLevel(t *testing.T) {
	t.Parallel()
	d, mm := denotedDepth(units.Inches(0.2)), denotedDepth(units.Millimeters(5.08))
	t.Run("0.2 in top", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		stack := lowStack(t, doc)
		boss2 := twoSidedRect(t, doc, 12, 8, 18, 12, units.Inches(0.2), units.Millimeters(3))
		joined, err := decad.Union(t.Context(), stack, boss2)
		require.NoError(t, err)
		require.Equal(t, stackedPrismClass, decadtest.PayloadClass(joined), "the tied denoted tops merge")
		require.Len(t, joined.Lumps(), 1)
		want := new(big.Rat).Add(big.NewRat(2000, 1), scaled(112, d, -2))
		requireRationalVolumeCovered(t, joined, want)
		requireBodyWatertight(t, joined)
	})
	t.Run("5.08 mm top", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		stack := lowStack(t, doc)
		boss2 := twoSidedRect(t, doc, 12, 8, 18, 12, units.Millimeters(5.08), units.Millimeters(3))
		joined, err := decad.Union(t.Context(), stack, boss2)
		require.NoError(t, err)
		require.True(t, tookMeshPath(joined), "two denoted tops are no tie")
		// The first boss less the 12 mm² overlap stands to 0.2 in; the second
		// boss stands to 5.08 mm, the higher plane.
		want := new(big.Rat).Add(big.NewRat(2000, 1), scaled(88, d, -2))
		want.Add(want, scaled(24, mm, -2))
		requireRationalVolumeCovered(t, joined, want)
	})
}

// TestStackedUnionBrepChainedDenotedLevel starts from an A1 union whose boss
// overhangs the plate's edge, so the interface crosses and the union builds a
// BRep over its stack. A second boss whose top ties the first's at 0.2 in
// merges through that stack's denoted levels. Below z=2 the footprint is
// plate 400 + boss overhang 50 + second-boss overhang 12 = 462 mm² over 5 mm;
// above it is the bosses' 112 mm² union.
func TestStackedUnionBrepChainedDenotedLevel(t *testing.T) {
	t.Parallel()
	d, mm := denotedDepth(units.Inches(0.2)), denotedDepth(units.Millimeters(5.08))
	a1 := func(t *testing.T, doc *decad.Document) *decad.Body {
		t.Helper()
		plate := twoSidedRect(t, doc, 0, 0, 20, 20, units.Millimeters(2), units.Millimeters(3))
		boss := twoSidedRect(t, doc, 15, 5, 25, 15, units.Inches(0.2), units.Millimeters(3))
		body, err := decad.Union(t.Context(), plate, boss)
		require.NoError(t, err)
		require.Equal(t, brepClass, decadtest.PayloadClass(body), "the crossing interface builds an A1 BRep")
		return body
	}
	t.Run("0.2 in top", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		first := a1(t, doc)
		boss2 := twoSidedRect(t, doc, 20, 8, 28, 12, units.Inches(0.2), units.Millimeters(3))
		joined, err := decad.Union(t.Context(), first, boss2)
		require.NoError(t, err)
		require.Equal(t, brepClass, decadtest.PayloadClass(joined), "the tied denoted tops merge")
		want := new(big.Rat).Add(big.NewRat(2310, 1), scaled(112, d, -2))
		requireRationalVolumeCovered(t, joined, want)
	})
	t.Run("5.08 mm top", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		first := a1(t, doc)
		boss2 := twoSidedRect(t, doc, 20, 8, 28, 12, units.Millimeters(5.08), units.Millimeters(3))
		joined, err := decad.Union(t.Context(), first, boss2)
		require.NoError(t, err)
		require.True(t, tookMeshPath(joined), "two denoted tops are no tie")
		// The first boss less the 20 mm² overlap stands to 0.2 in; the second
		// boss stands to 5.08 mm.
		want := new(big.Rat).Add(big.NewRat(2310, 1), scaled(80, d, -2))
		want.Add(want, scaled(32, mm, -2))
		requireRationalVolumeCovered(t, joined, want)
	})
}

// splitSheet chain-extrudes the line x=10 from y=-5 to y=25 on XY by e.
func splitSheet(t *testing.T, doc *decad.Document, e decad.Extent) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(10, -5)
	s.Fix(a)
	s.CreateLine(a, s.CreatePoint(10, 25))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	sheet, err := decad.Extrude(t.Context(), doc, s, s.Chains()[0], e)
	require.NoError(t, err)
	return sheet
}

// TestSurfaceSplitChainSheetDenotedLevel splits a 20x20 plate along x=10 with
// a chain sheet whose top denotes the plate's. The sheet records its sweep's
// denoted levels, so S6 proves it spans the plate and each piece is 10x20 over
// the plate's height. A 0.2 in sheet against a 5.08 mm plate really stops
// below the plate's top.
func TestSurfaceSplitChainSheetDenotedLevel(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name  string
		top   units.Value
		sheet decad.Extent
	}{
		{"0.3 in by 0.3 in", units.Inches(0.3), decad.Distance{D: units.Inches(0.3), Dir: decad.Along}},
		{"0.2 in by symmetric 0.4 in", units.Inches(0.2), decad.Symmetric{D: units.Inches(0.4), FullLength: true}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := upPlate(t, doc, tc.top)
			pieces, err := decad.Split(t.Context(), plate, splitSheet(t, doc, tc.sheet))
			require.NoError(t, err)
			require.Len(t, pieces, 2)
			for _, piece := range pieces {
				require.Equal(t, prismClass, decadtest.PayloadClass(piece))
				requireRationalVolumeCovered(t, piece, scaled(200, denotedDepth(tc.top), 0))
			}
		})
	}
	t.Run("sheet stops short", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		plate := upPlate(t, doc, units.Millimeters(5.08))
		_, err := decad.Split(t.Context(), plate, splitSheet(t, doc, decad.Distance{D: units.Inches(0.2), Dir: decad.Along}))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, sweepSpanMiss)
	})
}

// TestSurfaceSplitPieceKeepsDenotedLevel splits a 0.2 in plate along x=10,
// then cuts the piece over x 0..10 with a 4x4 tool to 0.2 in. The piece keeps
// the plate's denoted top, so the cut is the analytic through hole: 184 mm²
// over the plate's height.
func TestSurfaceSplitPieceKeepsDenotedLevel(t *testing.T) {
	t.Parallel()
	inch := units.Inches(0.2)
	doc := decad.New()
	plate := upPlate(t, doc, inch)
	pieces, err := decad.Split(t.Context(), plate, splitSheet(t, doc, decad.TwoSided{
		One: decad.DistanceSide{D: inch}, Two: decad.DistanceSide{D: units.Millimeters(3)}}))
	require.NoError(t, err)
	require.Len(t, pieces, 2)
	var left *decad.Body
	for _, piece := range pieces {
		box, err := piece.Bounds()
		require.NoError(t, err)
		if box.Max.X < 15 {
			left = piece
		}
	}
	require.NotNil(t, left, "one piece lies over x 0..10")
	part, err := decad.Cut(t.Context(), left, twoSidedRect(t, doc, 2, 8, 6, 12, inch, units.Millimeters(2)))
	require.NoError(t, err)
	require.Equal(t, prismClass, decadtest.PayloadClass(part), "the piece keeps the denoted top")
	requireRationalVolumeCovered(t, part, scaled(184, denotedDepth(inch), 0))
}

// TestShellTubeKeepsDenotedLevel shells a 0.2 in plate open at both caps into
// a 2 mm tube of 144 mm² section, then cuts a 1x1 tool to 0.2 in through its
// wall. The tube keeps the plate's denoted levels, so the cut is analytic.
func TestShellTubeKeepsDenotedLevel(t *testing.T) {
	t.Parallel()
	inch := units.Inches(0.2)
	doc := decad.New()
	plate := upPlate(t, doc, inch)
	tube, err := decad.Shell(t.Context(), plate,
		decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))).Or(decad.FaceCreatedBy(decad.CapStart(plate))),
		units.Millimeters(2))
	require.NoError(t, err)
	require.Equal(t, prismClass, decadtest.PayloadClass(tube))
	requireRationalVolumeCovered(t, tube, scaled(144, denotedDepth(inch), 0))
	part, err := decad.Cut(t.Context(), tube, twoSidedRect(t, doc, 0.5, 8, 1.5, 9, inch, units.Millimeters(2)))
	require.NoError(t, err)
	require.Equal(t, prismClass, decadtest.PayloadClass(part), "the tube keeps the denoted top")
	requireRationalVolumeCovered(t, part, scaled(143, denotedDepth(inch), 0))
}

// TestPatternedGroupKeepsDenotedLevel patterns a 4x4 peg over -2 mm..0.3 in
// twice, 8 mm apart, and cuts a 0.3 in plate with the group. The group keeps
// the peg's denoted top, so the cut is the plate with two analytic through
// holes: 368 mm² over the plate's height.
func TestPatternedGroupKeepsDenotedLevel(t *testing.T) {
	t.Parallel()
	inch := units.Inches(0.3)
	doc := decad.New()
	plate := upPlate(t, doc, inch)
	peg := twoSidedRect(t, doc, 2, 8, 6, 12, inch, units.Millimeters(2))
	group, err := decad.Patterned(t.Context(), peg, decad.LinearPattern{Dir: r3.NewVec(1, 0, 0),
		Step: units.Millimeters(8), Count: 2})
	require.NoError(t, err)
	require.Equal(t, stackedPrismClass, decadtest.PayloadClass(group))
	part, err := decad.Cut(t.Context(), plate, group)
	require.NoError(t, err)
	require.Equal(t, prismClass, decadtest.PayloadClass(part), "the group keeps the denoted top")
	requireRationalVolumeCovered(t, part, scaled(368, denotedDepth(inch), 0))
}

// extendRibbon chain-extrudes the segment x 0..40 of the line y=0, which a
// crossing line at x=40 splits, by top.
func extendRibbon(t *testing.T, doc *decad.Document, top units.Value) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	s.Fix(a)
	line := s.CreateLine(a, s.CreatePoint(100, 0))
	crossing := s.CreatePoint(40, -10)
	s.Fix(crossing)
	s.CreateLine(crossing, s.CreatePoint(40, 10))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, chain := range s.Chains() {
		if len(chain.Edges) != 1 || chain.Edges[0].Entity != line || chain.Edges[0].TStart != 0 {
			continue
		}
		ribbon, err := decad.Extrude(t.Context(), doc, s, chain, decad.Distance{D: top, Dir: decad.Along})
		require.NoError(t, err)
		return ribbon
	}
	require.FailNow(t, "the sketch has no chain starting at the origin")
	return nil
}

// TestSurfaceExtendChainDenotedLevel extends a 0.2 in ribbon's free edge at
// x=40 to a 0.2 in tool at x 70..90. The ribbon records its sweep's denoted
// levels, so S6 proves the tool spans it and the ribbon grows to x 0..70. A
// 5.08 mm ribbon rises above the 0.2 in tool's top.
func TestSurfaceExtendChainDenotedLevel(t *testing.T) {
	t.Parallel()
	inch := units.Inches(0.2)
	query := func() *decad.EdgeQuery {
		return decad.Edges(decad.Free(), decad.ParallelTo(r3.NewVec(0, 0, 1)),
			decad.EndpointAt(r3.NewVec(40, 0, 0))).Exactly(1)
	}
	t.Run("0.2 in ribbon", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ribbon := extendRibbon(t, doc, inch)
		tool := twoSidedRect(t, doc, 70, -10, 90, 10, inch, units.Millimeters(0))
		got, err := decad.Extend(t.Context(), ribbon, query(), tool)
		require.NoError(t, err)
		area, err := got.Area()
		require.NoError(t, err)
		residual := new(big.Rat).Sub(new(big.Rat).SetFloat64(areaMM2(t, area)), scaled(70, denotedDepth(inch), 0))
		bound, err := area.Bound.In(units.SquareMillimeter)
		require.NoError(t, err)
		require.LessOrEqual(t, residual.Abs(residual).Cmp(new(big.Rat).SetFloat64(bound)), 0,
			"the area bound covers the denoted area")
	})
	t.Run("5.08 mm ribbon", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ribbon := extendRibbon(t, doc, units.Millimeters(5.08))
		tool := twoSidedRect(t, doc, 70, -10, 90, 10, inch, units.Millimeters(0))
		_, err := decad.Extend(t.Context(), ribbon, query(), tool)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "the tool does not span the receiver over the sweep parameter")
	})
}
