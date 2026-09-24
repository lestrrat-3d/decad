package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md's T170-T174 and T181:
// docs/surface-intersection-design.md's PR1, Body.Trim over the prism
// family. Every fixture reuses plateSketch (extrude_test.go) for T1's own
// 100x60 rectangle sheet.

// trimRectSheet builds T1's own fixture: a 100x60 mm rectangle
// surface-extruded 10 mm Along, on doc.
func trimRectSheet(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	s, p := plateSketch(t)
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	return sheet
}

// trimSquareTool builds a solid extruded from the axis-aligned square
// (u0,v0)-(u1,v1) over the signed z interval [z0, z1], on doc's own XY plane
// (the identical frame trimRectSheet's own sheet uses, on the identity
// placement — S4/S7's shared-generator requirement).
func trimSquareTool(t *testing.T, doc *decad.Document, u0, v0, u1, v1, z0, z1 float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(u0, v0, u1, v1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(z1)},
		Two: decad.DistanceSide{D: units.Millimeters(-z0)},
	})
	require.NoError(t, err)
	return body
}

// trimSpanningTool is trimSquareTool's own shorthand for a tool that spans
// T1's sheet axially (S6): z in [-5, 15] covers the sheet's own [0, 10].
func trimSpanningTool(t *testing.T, doc *decad.Document, u0, v0, u1, v1 float64) *decad.Body {
	t.Helper()
	return trimSquareTool(t, doc, u0, v0, u1, v1, -5, 15)
}

// requireLumpFaceCounts asserts b has len(want) lumps, each with exactly
// want[i] faces, matched by lump order — sketch's own arrangement order is
// deterministic for a fixed input, but this helper does not rely on which
// physical piece lands first: every fixture below uses it only where every
// lump shares one face count.
func requireLumpFaceCounts(t *testing.T, b *decad.Body, faceCount int) {
	t.Helper()
	for i, l := range b.Lumps() {
		shells := l.Shells()
		require.Len(t, shells, 1, "lump %d", i)
		require.Len(t, shells[0].Faces(), faceCount, "lump %d", i)
	}
}

// planeNormalValue reads a face's own outward normal value, asserting it
// resolves — every wall this file's fixtures build is planar (Table G).
func planeNormalValue(t *testing.T, f *decad.Face) r3.Vec {
	t.Helper()
	n, err := f.NormalAt(r3.NewVec(0, 0, 0)) // ignored by a Plane surface
	require.NoError(t, err)
	return n.Value
}

// TestSurfaceTrimKeepOutsideSplitsIntoTwoRibbons is T170: T1's sheet trimmed
// KeepOutside by a tool spanning it axially and cutting its bottom and top
// walls at x = 40 and x = 60.
func TestSurfaceTrimKeepOutsideSplitsIntoTwoRibbons(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	sheet := trimRectSheet(t, doc)
	wantBounds, err := sheet.Bounds()
	require.NoError(t, err)
	tool := trimSpanningTool(t, doc, 40, -10, 60, 70)

	trimmed, err := sheet.Trim(t.Context(), tool, decad.KeepOutside)
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, trimmed.Kind())
	require.Len(t, trimmed.Lumps(), 2)
	requireLumpFaceCounts(t, trimmed, 3)

	free, err := decad.Edges(decad.Free()).Exactly(16).SelectEdges(trimmed)
	require.NoError(t, err)
	require.Len(t, free, 16)

	gotBounds, err := trimmed.Bounds()
	require.NoError(t, err)
	require.Equal(t, wantBounds, gotBounds, "both surviving runs still reach every extreme of the untrimmed sheet")

	_, err = trimmed.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)

	area, err := trimmed.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	// The bound must clear the incidental float-summation noise a two-level
	// per-lump-then-across-lumps boundedAdd carries even at zero
	// sectionDelta (measured at 2.8e-13 mm² for this fixture, forcing
	// resolveTrim's own cutDelta to zero) — otherwise this assertion would
	// pass on that noise alone and prove nothing about δ_cut. The proven
	// bound charging cutDelta measures 4.0e-10 mm² here, three orders above
	// that floor. Shown-to-fail: forcing resolveTrim's returned cutDelta to
	// 0 drops this bound under 1e-11 and turns this assertion red.
	require.Greater(t, area.Bound.Base(), 1e-11)
	decadtest.MeasuresArea(t, trimmed, units.SquareMillimeters(2800))
}

// TestSurfaceTrimKeepInsideKeepsTheCutStrips is T170's own pair, T171:
// KeepInside.
func TestSurfaceTrimKeepInsideKeepsTheCutStrips(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	sheet := trimRectSheet(t, doc)
	untrimmedFaces := sheet.Faces()
	untrimmedNormals := make(map[r3.Vec]*decad.Face, len(untrimmedFaces))
	for _, f := range untrimmedFaces {
		untrimmedNormals[planeNormalValue(t, f)] = f
	}
	tool := trimSpanningTool(t, doc, 40, -10, 60, 70)

	trimmed, err := sheet.Trim(t.Context(), tool, decad.KeepInside)
	require.NoError(t, err)

	require.Len(t, trimmed.Lumps(), 2)
	requireLumpFaceCounts(t, trimmed, 1)

	free, err := decad.Edges(decad.Free()).Exactly(8).SelectEdges(trimmed)
	require.NoError(t, err)
	require.Len(t, free, 8)

	area, err := trimmed.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	decadtest.MeasuresArea(t, trimmed, units.SquareMillimeters(400))

	for _, l := range trimmed.Lumps() {
		face := l.Shells()[0].Faces()[0]
		n := planeNormalValue(t, face)
		orig, ok := untrimmedNormals[n]
		require.True(t, ok, "no untrimmed wall shares this trimmed wall's own normal %v", n)
		wantN := planeNormalValue(t, orig)
		require.Equal(t, wantN, n, "a trim moves no wall")
	}
}

// TestSurfaceTrimDegenerateWhenNothingSeparates is T172: a tool wholly
// outside the receiver's section keeps every fragment, and one wholly
// containing it keeps none — both ErrDegenerate (R30), and neither operand
// changes.
func TestSurfaceTrimDegenerateWhenNothingSeparates(t *testing.T) {
	t.Parallel()

	t.Run("tool wholly outside", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		sheet := trimRectSheet(t, doc)
		tool := trimSpanningTool(t, doc, 200, -10, 220, 70)

		_, err := sheet.Trim(t.Context(), tool, decad.KeepOutside)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Len(t, doc.Bodies(), 2)
		require.Contains(t, doc.Bodies(), sheet)
		require.Contains(t, doc.Bodies(), tool)
	})

	t.Run("tool wholly contains", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		sheet := trimRectSheet(t, doc)
		tool := trimSpanningTool(t, doc, -10, -20, 110, 80)

		_, err := sheet.Trim(t.Context(), tool, decad.KeepOutside)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Len(t, doc.Bodies(), 2)
		require.Contains(t, doc.Bodies(), sheet)
		require.Contains(t, doc.Bodies(), tool)
	})
}

// TestSurfaceTrimRefusesASecondTrim is T173: T170's own trimmed result
// refuses a second Trim on S7's own section-displacement clause, while the
// SAME second tool succeeds against the untrimmed sheet.
func TestSurfaceTrimRefusesASecondTrim(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	sheet := trimRectSheet(t, doc)
	firstTool := trimSpanningTool(t, doc, 40, -10, 60, 70)
	trimmed, err := sheet.Trim(t.Context(), firstTool, decad.KeepOutside)
	require.NoError(t, err)

	secondTool := trimSpanningTool(t, doc, 10, -10, 20, 70)
	_, err = trimmed.Trim(t.Context(), secondTool, decad.KeepOutside)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "section displacement")

	sheet2 := trimRectSheet(t, doc)
	secondToolOnUntrimmed := trimSpanningTool(t, doc, 10, -10, 20, 70)
	_, err = sheet2.Trim(t.Context(), secondToolOnUntrimmed, decad.KeepOutside)
	require.NoError(t, err)
}

// TestSurfaceTrimS4IsExactFloatEquality is T174: a co-directional tool placed
// through r3.RotationAround at an inexact step count refuses on S4's own
// float== test, a tool on a plane parallel to the sheet's (an in-plane
// origin offset that keeps S4's worldNormal/worldOrigin test exact but
// leaves the re-expression non-identity) refuses on S7, and the identical
// rotated model built through r3.FromBasis instead is admitted.
func TestSurfaceTrimS4IsExactFloatEquality(t *testing.T) {
	t.Parallel()

	t.Run("a co-directional-looking tool refuses on S4", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		sheet := trimRectSheet(t, doc)
		tool := trimSpanningTool(t, doc, 40, -10, 60, 70)
		// docs/prism-boolean-design.md §3.3's own probe swept RotationAround
		// about the exact sweep axis and found specific step counts whose
		// composed normal misses (0,0,1) by an ulp through the Rodrigues
		// evaluation. The currently pinned r3 keeps that axis-aligned case
		// exact at every count from 3 to 60 (probed directly against this
		// module's own go.mod version), so this fixture instead rotates
		// about an axis one part in a thousand off the sweep direction:
		// still visibly "the same generator" to a caller reading the
		// placement, but its composed normal is provably not the stored
		// (0,0,1) Go == demands.
		xform, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0.001, 0, 1), units.Degrees(21.176470588235293))
		require.NoError(t, err)
		placedTool, err := tool.Placed(t.Context(), xform)
		require.NoError(t, err)

		_, err = sheet.Trim(t.Context(), placedTool, decad.KeepOutside)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "same generator")
	})

	t.Run("in-plane offset refuses on S7", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		sheet := trimRectSheet(t, doc)
		tool := trimSpanningTool(t, doc, 40, -10, 60, 70)
		xform, err := r3.Translation(r3.NewVec(5, 0, 0))
		require.NoError(t, err)
		placedTool, err := tool.Placed(t.Context(), xform)
		require.NoError(t, err)

		_, err = sheet.Trim(t.Context(), placedTool, decad.KeepOutside)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "identity")
	})

	t.Run("FromBasis rebuild is admitted", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		sheet := trimRectSheet(t, doc)
		// A tool drawn directly at the receiver's own rotated footprint —
		// exact by construction, never rotated through a placement.
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		rect := s.CreateRectangle(40, -10, 60, 70)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		tool, err := doc.Extrude(s, s.Profiles()[0], decad.TwoSided{
			One: decad.DistanceSide{D: units.Millimeters(15)},
			Two: decad.DistanceSide{D: units.Millimeters(5)},
		})
		require.NoError(t, err)
		basis, err := r3.FromBasis(r3.Basis{EX: r3.NewVec(1, 0, 0), EY: r3.NewVec(0, 1, 0), EZ: r3.NewVec(0, 0, 1)}, r3.NewVec(0, 0, 0))
		require.NoError(t, err)
		placedTool, err := tool.Placed(t.Context(), basis)
		require.NoError(t, err)

		_, err = sheet.Trim(t.Context(), placedTool, decad.KeepOutside)
		require.NoError(t, err)
	})
}

// trimRevolveCylinderSheet builds a revolve-family cylindrical sheet — a
// single meridian line, radius 5, spun a full turn about the sketch's own V
// axis — through RevolveChain, since a bare line forms no closed profile
// (docs/surface-design.md §13, revolve_chain_test.go's own pattern).
func trimRevolveCylinderSheet(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(5, 0)
	b := s.CreatePoint(5, 20)
	s.Fix(a)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Empty(t, s.Profiles())
	chains := s.Chains()
	require.Len(t, chains, 1)

	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}
	body, err := doc.RevolveChain(s, chains[0], axis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// TestSurfaceTrimRefusesAMixedFamilyPair is T181: a revolve-family sheet
// against a prism-family solid refuses both ways on S1, reading the payload
// family rather than the shape, and the message names the two generators —
// two independently built revolve-family cylinders against the SAME prism
// tool, so the refusal is S1's own family dispatch rather than a fact about
// one particular fixture.
func TestSurfaceTrimRefusesAMixedFamilyPair(t *testing.T) {
	t.Parallel()

	prismSolid := func(t *testing.T, doc *decad.Document) *decad.Body {
		t.Helper()
		return trimSquareTool(t, doc, -20, -20, 20, 20, 0, 20)
	}

	t.Run("T180's own revolve sheet against a prism solid", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		sheet := trimRevolveCylinderSheet(t, doc)
		tool := prismSolid(t, doc)

		_, err := sheet.Trim(t.Context(), tool, decad.KeepOutside)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "a revolve")
		require.ErrorContains(t, err, "a straight sweep")
	})

	t.Run("a separately built revolve cylinder against the same prism", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		sheet := trimRevolveCylinderSheet(t, doc)
		tool := prismSolid(t, doc)

		_, err := sheet.Trim(t.Context(), tool, decad.KeepOutside)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "a revolve")
		require.ErrorContains(t, err, "a straight sweep")
	})
}
