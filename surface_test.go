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
// for WithSurfaceResult() on Extrude and Table X's sheet-operand refusals.
// WithSurfaceResult() on Revolve, Sweep and Loft each have their own file,
// surface_revolve_test.go, surface_sweep_test.go and surface_loft_test.go.
// Every fixture reuses plateSketch/regularNGonSketch
// (extrude_test.go/extrude_bounds_test.go) and annularSketch/uAxis/
// loftSquares (revolve_test.go/loft_test.go).

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
// Every call site across the package's surface-result placement tests wants
// the same 8-edge shape (a rectangular profile's two rims), so n is a
// literal at the call rather than a threaded parameter, and adding a call
// with a different shape is exactly what would earn this back its own
// argument.
func requireSheetWithFreeEdges(t *testing.T, b *decad.Body) {
	t.Helper()
	const n = 8
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
	requireSheetWithFreeEdges(t, placed)
	placedBox, err := placed.Bounds()
	require.NoError(t, err)
	require.Equal(t, sheetBox.Min.Add(offset), placedBox.Min)
	require.Equal(t, sheetBox.Max.Add(offset), placedBox.Max)

	dup, err := placed.Duplicate()
	require.NoError(t, err)
	requireSheetWithFreeEdges(t, dup)

	copyMotion, err := r3.Translation(r3.NewVec(0, 50, 0))
	require.NoError(t, err)
	copied, err := dup.PlacedCopy(copyMotion)
	require.NoError(t, err)
	requireSheetWithFreeEdges(t, copied)
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

// TestSurfaceExtrudeHoledProfileReportsDisconnectedLumps is decision A
// (docs/surface-design.md §2.2): a holed profile's surface extrude has an
// outer wall tube and a hole wall tube that touch nowhere once their shared
// caps are omitted, so Body.Lumps() must report both rather than the one a
// single hardcoded shell used to. The solid built from the same profile keeps
// its one lump: its cap faces join both tubes into one connected shell.
func TestSurfaceExtrudeHoledProfileReportsDisconnectedLumps(t *testing.T) {
	t.Parallel()
	s, p := rectWithHoleSketch(t)
	doc := decad.New()
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(8), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	require.Len(t, sheet.Lumps(), 2)
	for _, l := range sheet.Lumps() {
		require.Len(t, l.Shells(), 1)
		require.True(t, l.Shells()[0].IsOpen())
	}

	s2, p2 := rectWithHoleSketch(t)
	solidDoc := decad.New()
	solid, err := solidDoc.Extrude(s2, p2, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	require.Len(t, solid.Lumps(), 1)
}

// TestSurfaceExtrudeSheetVerifiesValid is docs/surface-design.md's sheet
// validity audit (§9.1): a surface-extruded rectangle's boundary reads clean
// under all three structural legs, and its payload — a prismPayload with
// surfaceResult true and a zero sectionDelta — admits the fourth,
// non-self-intersection, by construction. The sheet reads ValidityValid
// outright, with no Region (a sheet encloses none) and both the body and the
// whole report Sound with no diagnostics.
func TestSurfaceExtrudeSheetVerifiesValid(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br := decadtest.FindBodyReport(t, report, sheet)
	require.Equal(t, decad.ValidityValid, br.Validity.Outcome)
	require.Empty(t, br.Validity.Diagnostics)
	require.Nil(t, br.Region)
	require.Equal(t, 1, br.Topology.Lumps)
	require.Equal(t, 0, br.Topology.Voids)
	require.Equal(t, decad.Sound, br.Status)
	require.Empty(t, br.Diagnostics)
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Diagnostics)
}

// TestSheetWallAndConcaveRadiusKeepTheSurveyPrerequisiteRefusal is
// docs/surface-design.md §9.1's survey row: a sound sheet has no material for
// a wall or a concave question, so each reads Unavailable with its own
// DiagSurveyPrerequisite naming the blocked survey and dropping "pull" from
// its message (§9.1's undercut clause is not this survey's cause any more).
// The pull requested alongside them is axial — every wall is exactly
// perpendicular to it — so Undercut reads a real CoverageComplete with an
// empty Faces rather than a third prerequisite refusal, and the report's
// Suspect comes from the wall and concave-radius refusals alone.
func TestSheetWallAndConcaveRadiusKeepTheSurveyPrerequisiteRefusal(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)

	// The tool (50 mm) exceeds the plate's own 10 mm spanning wall on
	// purpose: publishWallResult's kind gate must refuse before that
	// reading is ever consulted, so an unguarded wall survey run on the
	// sheet would otherwise surface a DiagWallTooThin the published result
	// must never carry.
	report, err := doc.Verify(t.Context(),
		decad.WithMinWallThickness(units.Millimeters(50)),
		decad.WithPullDirection(r3.NewVec(0, 0, 1)),
		decad.WithConcaveRadius(),
	)
	require.NoError(t, err)
	br := decadtest.FindBodyReport(t, report, sheet)

	require.Equal(t, decad.ScalarUnavailable, br.Wall.Outcome)
	require.Equal(t, decad.CoverageComplete, br.Undercut.Coverage)
	require.NotNil(t, br.Undercut.Faces)
	require.Empty(t, br.Undercut.Faces)
	require.Equal(t, decad.ScalarUnavailable, br.ConcaveRadius.Outcome)

	requireOnePrerequisite := func(diags []decad.Diagnostic, survey decad.SurveyKind) {
		t.Helper()
		require.Len(t, diags, 1)
		require.Equal(t, decad.DiagSurveyPrerequisite, diags[0].Code)
		require.Equal(t, survey, diags[0].Survey)
		require.NotContains(t, diags[0].Message, "pull")
	}
	requireOnePrerequisite(br.Wall.Diagnostics, decad.SurveyWall)
	requireOnePrerequisite(br.ConcaveRadius.Diagnostics, decad.SurveyConcaveRadius)
	require.Empty(t, br.Undercut.Diagnostics)

	require.Len(t, br.Diagnostics, 2)
	require.Equal(t, decad.Suspect, report.Status)
}

// TestSheetUndercutsListTheWallThatOpposesThePull is
// TestUndercutsTiltedPull's own fixture (survey_test.go) with
// WithSurfaceResult() added, which is what makes the comparison legible: the
// solid lists two faces, the −X wall and the bottom cap, and the sheet —
// which builds no cap face at all — must list exactly the one wall.
func TestSheetUndercutsListTheWallThatOpposesThePull(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(1, 0, 1)))
	require.NoError(t, err)
	br := decadtest.FindBodyReport(t, report, sheet)

	require.Equal(t, decad.CoverageComplete, br.Undercut.Coverage)
	require.Len(t, br.Undercut.Faces, 1)
	f := br.Undercut.Faces[0]
	pt := f.Loops()[0].CoEdges()[0].Start().Position().Value
	n, err := f.NormalAt(pt)
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(-1, 0, 0), n.Value)
	bound, err := n.Bound.In(units.One)
	require.NoError(t, err)
	require.Equal(t, 0.0, bound)
	require.Equal(t, decad.AssessmentViolated, br.Undercut.Assessment)
	require.Equal(t, decad.Violating, br.Status)
	require.Len(t, br.Undercut.Diagnostics, 1)
	require.Equal(t, decad.DiagUndercut, br.Undercut.Diagnostics[0].Code)
	require.Equal(t, decad.SurveyUndercut, br.Undercut.Diagnostics[0].Survey)
	require.False(t, report.Passed())

	require.Nil(t, br.Region)
	_, err = sheet.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
}

// TestSheetUndercutsProveTheAllClearUnderAnAxialPull is the same sheet under
// the axial pull TestUndercutsPrismClear uses for the equivalent solid:
// every wall is exactly perpendicular, the proven all-clear §6 carves out,
// and the report reaches Sound with an empty (not nil) Faces.
func TestSheetUndercutsProveTheAllClearUnderAnAxialPull(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	br := decadtest.FindBodyReport(t, report, sheet)

	require.Equal(t, decad.CoverageComplete, br.Undercut.Coverage)
	require.NotNil(t, br.Undercut.Faces)
	require.Empty(t, br.Undercut.Faces)
	require.Equal(t, decad.AssessmentMet, br.Undercut.Assessment)
	require.Equal(t, decad.Sound, br.Status)
	require.Empty(t, br.Diagnostics)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Passed())
}

// TestSheetUndercutsReadUnavailableOnALoftSheet is the family this increment
// leaves closed: a surface-result Loft sheet's own ValidityValid
// (surface_loft_test.go's TestSurfaceLoftSheetVerifiesSound) does not by
// itself admit the undercut survey, because runSurveys names no loftPayload
// arm, so the refusal moves from DiagSurveyPrerequisite to
// DiagUnsupportedSurveyPayload rather than being lifted.
func TestSheetUndercutsReadUnavailableOnALoftSheet(t *testing.T) {
	t.Parallel()
	s0, p0, s1, p1 := loftSquares(t, 20, 10)
	doc := decad.New()
	sheet, err := doc.Loft(s0, p0, s1, p1, decad.WithSurfaceResult())
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	br := decadtest.FindBodyReport(t, report, sheet)

	require.Equal(t, decad.CoverageUnavailable, br.Undercut.Coverage)
	require.Len(t, br.Undercut.Diagnostics, 1)
	require.Equal(t, decad.DiagUnsupportedSurveyPayload, br.Undercut.Diagnostics[0].Code)
	require.Equal(t, decad.SurveyUndercut, br.Undercut.Diagnostics[0].Survey)
	require.Contains(t, br.Undercut.Diagnostics[0].Message, "loftPayload")
	require.Equal(t, decad.AssessmentUndecided, br.Undercut.Assessment)
	require.Equal(t, decad.Suspect, br.Status)
}

// TestSheetUndercutsRefusedOnARevolveSheet pins the survey-aware message
// split: a surface-result Revolve sheet's own fourth leg is undecided
// (surface_revolve_test.go's TestSurfaceRevolveSheetVerifiesUndecided), so
// the undercut refusal is DiagSurveyPrerequisite as before, but its message
// must now name the validity cause rather than the sheet cause — the
// undercut publisher's kind gate no longer fires for this body at all.
func TestSheetUndercutsRefusedOnARevolveSheet(t *testing.T) {
	t.Parallel()
	s, p := annularSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, quarterTurn, decad.WithSurfaceResult())
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	br := decadtest.FindBodyReport(t, report, sheet)

	require.Equal(t, decad.CoverageUnavailable, br.Undercut.Coverage)
	require.Len(t, br.Undercut.Diagnostics, 1)
	require.Equal(t, decad.DiagSurveyPrerequisite, br.Undercut.Diagnostics[0].Code)
	require.Contains(t, br.Undercut.Diagnostics[0].Message, "validity")
	require.NotContains(t, br.Undercut.Diagnostics[0].Message, "sheet")
}

// sheetBoxBody surface-extrudes an axis-aligned rectangle into doc, the sheet
// counterpart of clearance_test.go's boxBody.
func sheetBoxBody(t *testing.T, doc *decad.Document, x0, y0, x1, y1, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	return body
}

// TestSheetSolidPairCrossingBoxesReadInterfering is docs/surface-design.md's
// T7, rewritten for increment 2's decision procedure: the sheet's wall at
// x=10 (y∈[0,10], z∈[0,5]) and the solid's face at y=5 (x∈[5,15], z∈[0,5])
// are transversal planes whose intersection line — x=10, y=5, z∈[0,5] — sits
// inside both faces' trims, an admitted crossing the candidate enumeration
// proves. Increment 1 read this same fixture as DiagUnsupportedPairSheet and
// Suspect (the box rule alone, with no decision procedure behind it yet);
// increment 2 decides it as a proven crossing instead, so this test is
// rewritten in place rather than repointed to new geometry.
func TestSheetSolidPairCrossingBoxesReadInterfering(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	sheet := sheetBoxBody(t, doc, 0, 0, 10, 10, 5)
	solid := boxBody(t, doc, 5, 5, 15, 15, 5)

	for _, opts := range [][]decad.VerifyOption{nil, {decad.WithClearances()}} {
		report, err := doc.Verify(t.Context(), opts...)
		require.NoError(t, err)
		require.Equal(t, decad.Interfering, report.Status)
		require.False(t, report.Passed())

		diags := decadtest.FindDiagnostics(t, report, decad.DiagSheetSolidCrossing)
		require.Len(t, diags, 1)
		d := diags[0]
		require.NotNil(t, d.Pair)
		require.Same(t, sheet, d.Pair.A)
		require.Same(t, solid, d.Pair.B)
		require.Equal(t, decad.Interfering, d.Status)
		require.Equal(t, decad.ReadingNone, d.Reading)
		require.Nil(t, d.Observed)
		require.Nil(t, d.ObservedVec)
		require.Nil(t, d.ObservedBox)
		require.Nil(t, d.Required)
		require.Nil(t, d.Body)

		// A sheet encloses no region: the crossing is proven, but there is
		// no overlap volume to report, so no Interference row is ever
		// emitted for it — that absence is not a proven non-overlap here
		// (docs/surface-design.md §9.3).
		require.Empty(t, report.Interferences)
		require.Empty(t, report.Clearances)
		require.Len(t, doc.Bodies(), 2)
	}
}

// TestSheetSolidPairSeparatedBoxesVerifySound is docs/surface-design.md's T8:
// the same pair, moved apart until the boxes separate, contributes nothing
// under the default call. Under WithClearances() the kernel still measures
// the gap, exactly as it does for a box-separated solid-solid pair
// (docs/interference-design.md §3.1: box separation does not measure the
// true gap, and the analytic kernel still runs when a gap is asked for) — a
// sheet-solid pair honours the same rule rather than staying silent for the
// same question the solid path answers.
func TestSheetSolidPairSeparatedBoxesVerifySound(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	sheet := sheetBoxBody(t, doc, 0, 0, 10, 10, 5)
	solid := boxBody(t, doc, 100, 100, 110, 110, 5)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Diagnostics)
	require.Empty(t, report.Interferences)
	require.Empty(t, report.Clearances, "no gap row without WithClearances()")

	report, err = doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Diagnostics)
	require.Empty(t, report.Interferences)
	requireExactGap(t, report, 90*math.Sqrt(2))
	require.Same(t, sheet, report.Clearances[0].A)
	require.Same(t, solid, report.Clearances[0].B)

	require.Len(t, doc.Bodies(), 2)
}

// TestSheetSolidPairOutsideProvenSound is docs/surface-design.md §9.3's
// increment-2 decision procedure, outside branch: the sheet is a 10×10 mm
// wall frame (walls only, no caps) around a 4×4 mm solid centered inside it,
// both spanning z∈[0,5]. Their bounds-inflated boxes MEET (the solid's box
// sits wholly inside the sheet's), so box separation alone cannot settle the
// pair, but the sheet's material never comes closer than the frame-to-block
// gap of 3 mm on every side — a closed-form Plane×Plane reading, Exact. The
// deterministic witness (a sheet corner) casts outside the solid, proving the
// sheet lies wholly outside it: Sound, with the gap row under
// WithClearances() and no diagnostic.
func TestSheetSolidPairOutsideProvenSound(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	sheet := sheetBoxBody(t, doc, 0, 0, 10, 10, 5)
	solid := boxBody(t, doc, 3, 3, 7, 7, 5)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Diagnostics)
	require.Empty(t, report.Interferences)
	require.Empty(t, report.Clearances, "no gap row without WithClearances()")

	report, err = doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Diagnostics)
	require.Empty(t, report.Interferences)
	requireExactGap(t, report, 3)
	require.Same(t, sheet, report.Clearances[0].A)
	require.Same(t, solid, report.Clearances[0].B)

	require.Len(t, doc.Bodies(), 2)
}

// TestSheetSolidPairContainedProvenSound is docs/surface-design.md §9.3's
// increment-2 decision procedure, containment branch: a 10×10×10 mm sheet
// box sits centered 5 mm inside a 20×20×20 mm solid cube on every side. The
// boxes meet (the sheet's box sits wholly inside the solid's), the boundary
// distance is a closed-form 5 mm on every side, Exact, and the deterministic
// witness casts inside the solid, proving containment: Sound, with the SAME
// shape of gap row under WithClearances() as the outside case — one
// consequence of the decision procedure is that this row never asserts which
// side the sheet is on (docs/api-design.md §6.2) — and no diagnostic.
func TestSheetSolidPairContainedProvenSound(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	solid := boxBody(t, doc, 0, 0, 20, 20, 20)
	raw := sheetBoxBody(t, doc, 5, 5, 15, 15, 10)
	shift, err := r3.Translation(r3.NewVec(0, 0, 5))
	require.NoError(t, err)
	sheet, err := raw.Placed(shift)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Diagnostics)
	require.Empty(t, report.Interferences)
	require.Empty(t, report.Clearances, "no gap row without WithClearances()")

	report, err = doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Diagnostics)
	require.Empty(t, report.Interferences)
	// The sheet arrives via Placed, so its own frame/placement rounding
	// (bodyGeom.delta) now widens the row to honest-Approximate.
	requireBoundedGapContains(t, report, 5)
	require.Same(t, solid, report.Clearances[0].A)
	require.Same(t, sheet, report.Clearances[0].B)

	require.Len(t, doc.Bodies(), 2)
}

// TestSheetSolidPairKernelCannotSettleStaysSuspect exercises the "missing
// body model" leg of docs/surface-design.md §9.3's decision procedure: a
// surface-extruded fit-spline arch is ValidityValid — the sheet audit's
// fourth leg is proven by construction for ANY surface-result prism with no
// section displacement, regardless of wall kind (verify.go's
// auditSheetBoundary) — but its one free-form wall has no analytic carrier
// face the clearance kernel can build (walkElem, clearance_geom.go), the same
// gap extrude_freeform_test.go already pins for the wall/undercut/
// concave-radius surveys on the solid built from the same profile. With no
// model to decide against, the pair stays undecided even though its boxes
// MEET: DiagUnsupportedPairSheet, Suspect — its message now names the
// kernel's own failure to settle the pair, not an unsupported sheet operand.
func TestSheetSolidPairKernelCannotSettleStaysSuspect(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	solid := boxBody(t, doc, 2, 0, 6, 2, 10)
	s, p := fitSplineArchSketch(t)
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)

	diags := decadtest.FindDiagnostics(t, report, decad.DiagUnsupportedPairSheet)
	require.Len(t, diags, 1)
	d := diags[0]
	require.NotNil(t, d.Pair)
	require.Same(t, solid, d.Pair.A)
	require.Same(t, sheet, d.Pair.B)
	require.Contains(t, d.Message, "could not settle")
	require.NotContains(t, d.Message, "unsupported",
		"the message must name the kernel's own failure to settle the pair, not an unsupported sheet operand")

	require.Empty(t, report.Interferences)
	require.Empty(t, report.Clearances)
	require.Len(t, doc.Bodies(), 2)
}

// TestSheetSheetPairStaysUndecided is docs/surface-design.md §9.3's
// sheet-sheet rule: neither operand offers a closed boundary to cast the
// other against, so the decision procedure never runs at all — the pair
// keeps DiagUnsupportedPairSheet and Suspect regardless of whether the true
// geometry would have crossed, exactly as the fixture in
// TestSheetSolidPairCrossingBoxesReadInterfering would have, had either
// operand been a solid.
func TestSheetSheetPairStaysUndecided(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	sheet1 := sheetBoxBody(t, doc, 0, 0, 10, 10, 5)
	sheet2 := sheetBoxBody(t, doc, 5, 5, 15, 15, 5)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)

	diags := decadtest.FindDiagnostics(t, report, decad.DiagUnsupportedPairSheet)
	require.Len(t, diags, 1)
	d := diags[0]
	require.NotNil(t, d.Pair)
	require.Same(t, sheet1, d.Pair.A)
	require.Same(t, sheet2, d.Pair.B)

	require.Empty(t, report.Interferences)
	require.Empty(t, report.Clearances)
	require.Len(t, doc.Bodies(), 2)
}

// TestSheetSolidPairDiagnosticFollowsDocumentOrder proves the crossing
// diagnostic's Pair fields follow Document.Bodies() order rather than
// "sheet always first": the solid is created before the sheet here, the
// reverse of TestSheetSolidPairCrossingBoxesReadInterfering, and the
// diagnostic's Pair.A/B swap to match.
func TestSheetSolidPairDiagnosticFollowsDocumentOrder(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	solid := boxBody(t, doc, 5, 5, 15, 15, 5)
	sheet := sheetBoxBody(t, doc, 0, 0, 10, 10, 5)
	require.Equal(t, []*decad.Body{solid, sheet}, doc.Bodies())

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)

	diags := decadtest.FindDiagnostics(t, report, decad.DiagSheetSolidCrossing)
	require.Len(t, diags, 1)
	require.Same(t, solid, diags[0].Pair.A)
	require.Same(t, sheet, diags[0].Pair.B)
	require.Len(t, doc.Bodies(), 2)
}
