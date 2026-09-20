package decad_test

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

// This file is docs/surface-design.md's T1/T9/T13-shaped public-surface tests
// for WithSurfaceResult() on Extrude, its Table R row R1 refusal on Revolve,
// Sweep and Loft, and Table X's sheet-operand refusals. Every fixture reuses
// plateSketch/regularNGonSketch (extrude_test.go/extrude_bounds_test.go) and
// annularSketch/uAxis/loftSquares (revolve_test.go/loft_test.go).

// rectWithHoleSketch builds a solved 100x60 rectangle with a circular hole,
// same fixture TestExtrudePlateWithHole (extrude_test.go) builds inline.
func rectWithHoleSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	return s, prof
}

// requireSheetWithFreeEdges asserts b is a sheet with exactly n free edges.
func requireSheetWithFreeEdges(t *testing.T, b *decad.Body, n int) {
	t.Helper()
	require.Equal(t, decad.BodySheet, b.Kind())
	free, err := decad.Edges(decad.Free()).Exactly(n).SelectEdges(b)
	require.NoError(t, err)
	require.Len(t, free, n)
}

// TestSurfaceExtrudePlateIsASheet is docs/surface-design.md's T1: a 100x60 mm
// rectangle surface-extruded 10 mm Along.
func TestSurfaceExtrudePlateIsASheet(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.False(t, sheet.IsSolid())

	_, err = sheet.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	_, err = sheet.Centroid()
	require.ErrorIs(t, err, decad.ErrNotSolid)

	decadtest.MeasuresArea(t, sheet, units.SquareMillimeters(3200), decadtest.Exactly())

	faces := sheet.Faces()
	require.Len(t, faces, 4)
	for _, f := range faces {
		_, ok := f.Surface().(decad.Plane)
		require.True(t, ok, "every wall of a rectangular surface extrude is planar")
	}

	free, err := decad.Edges(decad.Free()).Exactly(8).SelectEdges(sheet)
	require.NoError(t, err)
	for _, e := range free {
		require.True(t, e.IsFree())
		require.Len(t, e.Faces(), 1)
	}

	var vertical int
	for _, e := range sheet.Edges() {
		if e.IsFree() {
			continue
		}
		vertical++
		require.Len(t, e.Faces(), 2)
	}
	require.Equal(t, 4, vertical)

	require.Len(t, sheet.Lumps(), 1)
	shells := sheet.Shells()
	require.Len(t, shells, 1)
	require.True(t, shells[0].IsOpen())
	require.False(t, shells[0].IsVoid())

	// Bounds (§4.3): unchanged from what the same profile builds as a solid,
	// value AND bound.
	solidDoc := decad.New()
	solid, err := solidDoc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	solidBox, err := solid.Bounds()
	require.NoError(t, err)
	sheetBox, err := sheet.Bounds()
	require.NoError(t, err)
	require.Equal(t, solidBox, sheetBox)
}

// TestSurfaceExtrudeAreaComposesCapSubtraction proves the area-bound
// composition on an inexact fixture: T1's rectangle is all-zero-bound and
// proves nothing about it.
func TestSurfaceExtrudeAreaComposesCapSubtraction(t *testing.T) {
	t.Parallel()
	const r = 10.0
	const h = 5.0
	const n = 12
	side := 2 * r * math.Sin(math.Pi/n)
	perimeter := float64(n) * side
	exactWallArea := perimeter * h

	s, p := regularNGonSketch(t, n, r)
	solidDoc := decad.New()
	solid, err := solidDoc.Extrude(s, p, decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	solidArea, err := solid.Area()
	require.NoError(t, err)

	capStartFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(solid))).Exactly(1).SelectFaces(solid)
	require.NoError(t, err)
	capStartArea, err := capStartFaces[0].Area()
	require.NoError(t, err)
	capEndFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(solid))).Exactly(1).SelectFaces(solid)
	require.NoError(t, err)
	capEndArea, err := capEndFaces[0].Area()
	require.NoError(t, err)

	s2, p2 := regularNGonSketch(t, n, r)
	sheetDoc := decad.New()
	sheet, err := sheetDoc.Extrude(s2, p2, decad.Distance{D: units.Millimeters(h), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	sheetArea, err := sheet.Area()
	require.NoError(t, err)

	// Value: the same subtraction the build performs, bit for bit.
	wantValue := solidArea.Value.Base()
	wantValue += -capStartArea.Value.Base()
	wantValue += -capEndArea.Value.Base()
	require.Equal(t, wantValue, sheetArea.Value.Base())

	// Bound: the composed sum, up to the tiny rounding term boundedAdd charges
	// on top of it.
	wantBound := solidArea.Bound.Base() + capStartArea.Bound.Base() + capEndArea.Bound.Base()
	require.GreaterOrEqual(t, sheetArea.Bound.Base(), wantBound)
	require.InDelta(t, wantBound, sheetArea.Bound.Base(), 1e-9)

	// Soundness against the closed-form perimeter times height.
	require.LessOrEqual(t, math.Abs(sheetArea.Value.Base()-exactWallArea), sheetArea.Bound.Base())
}

// TestSurfaceExtrudeRolesResolveThroughFaceCreatedBy is §4.2: a rectangle
// with a circular hole keeps its side(i, j) roles resolvable, while
// CapStart/CapEnd mint refs that match nothing.
func TestSurfaceExtrudeRolesResolveThroughFaceCreatedBy(t *testing.T) {
	t.Parallel()
	s, p := rectWithHoleSketch(t)
	doc := decad.New()
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(8), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)

	faces := sheet.Faces()
	require.NotEmpty(t, faces)
	for _, f := range faces {
		origins := f.Origins()
		require.Len(t, origins, 1)
		matched, err := decad.Faces(decad.FaceCreatedBy(origins[0])).Exactly(1).SelectFaces(sheet)
		require.NoError(t, err)
		require.Same(t, f, matched[0])
	}

	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapStart(sheet))).SelectFaces(sheet)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapEnd(sheet))).SelectFaces(sheet)
	require.ErrorIs(t, err, decad.ErrNoMatch)
}

// TestSurfaceExtrudeStaysASheetThroughPlacement is §4.2's Placed/Duplicate/
// PlacedCopy claim: each re-evaluates the payload and reproduces the sheet
// with no further code, and Placed's box is the sheet's own translated.
func TestSurfaceExtrudeStaysASheetThroughPlacement(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	sheetBox, err := sheet.Bounds()
	require.NoError(t, err)

	offset := r3.NewVec(50, 0, 0)
	motion, err := r3.Translation(offset)
	require.NoError(t, err)

	placed, err := sheet.Placed(motion)
	require.NoError(t, err)
	requireSheetWithFreeEdges(t, placed, 8)
	placedBox, err := placed.Bounds()
	require.NoError(t, err)
	require.Equal(t, sheetBox.Min.Add(offset), placedBox.Min)
	require.Equal(t, sheetBox.Max.Add(offset), placedBox.Max)

	dup, err := placed.Duplicate()
	require.NoError(t, err)
	requireSheetWithFreeEdges(t, dup, 8)

	copyMotion, err := r3.Translation(r3.NewVec(0, 50, 0))
	require.NoError(t, err)
	copied, err := dup.PlacedCopy(copyMotion)
	require.NoError(t, err)
	requireSheetWithFreeEdges(t, copied, 8)
}

// TestSurfaceResultRefusedByRevolveSweepAndLoft is Table R row R1: each
// feature this evaluator does not yet build as a surface refuses
// WithSurfaceResult() outright, and the document is unchanged.
func TestSurfaceResultRefusedByRevolveSweepAndLoft(t *testing.T) {
	t.Parallel()

	t.Run("revolve", func(t *testing.T) {
		t.Parallel()
		s, p := annularSketch(t)
		doc := decad.New()
		before := doc.Bodies()
		_, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, before, doc.Bodies())
	})

	t.Run("sweep", func(t *testing.T) {
		t.Parallel()
		s, p := plateSketch(t)
		path, err := decad.NewPath(r3.NewVec(0, 0, 0), decad.LineTo{End: r3.NewVec(0, 0, 10)})
		require.NoError(t, err)
		doc := decad.New()
		before := doc.Bodies()
		_, err = doc.Sweep(s, p, path, decad.WithSurfaceResult())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, before, doc.Bodies())
	})

	t.Run("loft", func(t *testing.T) {
		t.Parallel()
		s0, p0, s1, p1 := loftSquares(t, 20, 20)
		doc := decad.New()
		before := doc.Bodies()
		_, err := doc.Loft(s0, p0, s1, p1, decad.WithSurfaceResult())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, before, doc.Bodies())
	})
}

// newSheetAndSolid builds a document holding one sheet and one solid, both
// live, from the same plate profile.
func newSheetAndSolid(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
	t.Helper()
	doc := decad.New()
	s1, p1 := plateSketch(t)
	sheet, err := doc.Extrude(s1, p1, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	s2, p2 := plateSketch(t)
	solid, err := doc.Extrude(s2, p2, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return doc, sheet, solid
}

// TestSheetRefusedByBooleanAndModifyOperations is Table X: Union/Cut/
// Intersect with a sheet in either operand position, and Fillet/Chamfer/
// Shell on a sheet receiver, are each ErrUnsupported, and every body stays
// live with the document unchanged.
func TestSheetRefusedByBooleanAndModifyOperations(t *testing.T) {
	t.Parallel()

	t.Run("union sheet first", func(t *testing.T) {
		t.Parallel()
		doc, sheet, solid := newSheetAndSolid(t)
		before := doc.Bodies()
		_, err := decad.Union(sheet, solid)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, before, doc.Bodies())
	})
	t.Run("union solid first", func(t *testing.T) {
		t.Parallel()
		doc, sheet, solid := newSheetAndSolid(t)
		before := doc.Bodies()
		_, err := decad.Union(solid, sheet)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, before, doc.Bodies())
	})
	t.Run("cut", func(t *testing.T) {
		t.Parallel()
		doc, sheet, solid := newSheetAndSolid(t)
		before := doc.Bodies()
		_, err := decad.Cut(solid, sheet)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, before, doc.Bodies())
	})
	t.Run("intersect", func(t *testing.T) {
		t.Parallel()
		doc, sheet, solid := newSheetAndSolid(t)
		before := doc.Bodies()
		_, err := decad.Intersect(sheet, solid)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, before, doc.Bodies())
	})
	t.Run("fillet", func(t *testing.T) {
		t.Parallel()
		doc, sheet, _ := newSheetAndSolid(t)
		before := doc.Bodies()
		_, err := sheet.Fillet(decad.Edges(), units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, before, doc.Bodies())
	})
	t.Run("chamfer", func(t *testing.T) {
		t.Parallel()
		doc, sheet, _ := newSheetAndSolid(t)
		before := doc.Bodies()
		_, err := sheet.Chamfer(decad.Edges(), units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, before, doc.Bodies())
	})
	t.Run("shell", func(t *testing.T) {
		t.Parallel()
		doc, sheet, _ := newSheetAndSolid(t)
		before := doc.Bodies()
		_, err := sheet.Shell(decad.Faces(), units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, before, doc.Bodies())
	})
}

// TestThroughAllIgnoresASheetBody is Table X's ThroughAll row: a sheet in the
// document is skipped when resolving stops, so a ThroughAll sweep resolves
// against the solids alone.
func TestThroughAllIgnoresASheetBody(t *testing.T) {
	t.Parallel()
	doc := decad.New()

	sheetSketch, sheetProfile := plateSketch(t)
	sheet, err := doc.Extrude(sheetSketch, sheetProfile, decad.Distance{D: units.Millimeters(20), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, sheet.Kind())

	blockSketch, blockProfile := plateSketch(t)
	_, err = doc.Extrude(blockSketch, blockProfile, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)

	thirdSketch, thirdProfile := plateSketch(t)
	swept, err := doc.Extrude(thirdSketch, thirdProfile, decad.ThroughAll{Dir: decad.Along})
	require.NoError(t, err)

	// If the sheet were not skipped, its far side at z=20 would win over the
	// block's at z=5, and the volume below would be four times too large.
	decadtest.MeasuresVolume(t, swept, units.CubicMillimeters(100*60*5))
}

// TestSurfaceExtrudeShellOpenAgreesWithFreeEdgeDerivation catches a builder
// that mints a free edge and forgets the Shell.open flag: the stored flag
// must agree with the derivation on every surface-extruded body.
func TestSurfaceExtrudeShellOpenAgreesWithFreeEdgeDerivation(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)

	shells := sheet.Shells()
	require.Len(t, shells, 1)
	hasFree := false
	for _, e := range sheet.Edges() {
		if e.IsFree() {
			hasFree = true
			break
		}
	}
	require.Equal(t, hasFree, shells[0].IsOpen())
	require.True(t, shells[0].IsOpen(), "a surface extrude's rim edges must be free")
}

// TestSurfaceExtrudeSheetVerifiesUndecided is the verify.go holding fix
// (docs/surface-design.md §9.1, §14): the manifold-with-boundary audit lands
// later, so a sheet reads ValidityUndecided rather than proven invalid, and
// carries no Region and no DiagInvalidBody.
func TestSurfaceExtrudeSheetVerifiesUndecided(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br := decadtest.FindBodyReport(t, report, sheet)
	require.Equal(t, decad.ValidityUndecided, br.Validity.Outcome)
	require.Nil(t, br.Region)
	for _, d := range br.Diagnostics {
		require.NotEqual(t, decad.DiagInvalidBody, d.Code)
	}
}
