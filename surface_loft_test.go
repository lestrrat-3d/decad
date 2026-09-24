package decad_test

import (
	"bytes"
	"math"
	"math/big"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md's Table W and T-shaped public-surface
// tests for WithSurfaceResult() on Loft: a lofted box's face/edge/area/bounds
// obligations, the area-bound composition on a fixture whose caps are
// genuinely inexact, role resolution, placement reproduction, a holed
// profile's disconnected lumps, the sheet's own validity reading at a zero
// and a positive section displacement, sheet tessellation, and the
// shared IsOpen obligation. WithSurfaceResult() on Extrude/Revolve has its own
// file (surface_test.go/surface_revolve_test.go). Every fixture reuses
// loftSquares/loftSquaresAt (loft_test.go) or builds its own n-gon, wedge or
// holed pair, since those fixtures need either a positive cap bound or a
// same-kind curved correspondence loft_test.go's own fixtures do not build.

// loftNGonAt builds two congruent regular n-gon profiles of circumradius r on
// parallel planes height apart: a LineSeg-only correspondence, so sectionDelta
// stays zero, whose cap area is irrational for n != 4 — unlike loftSquares'
// own square caps, which are exactly representable and would prove nothing
// about the area-bound composition (docs/surface-design.md §4.3).
func loftNGonAt(t *testing.T, n int, r, height float64) (*sketch.Sketch, *sketch.Profile, *sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	base := w.XY()
	top, err := w.CreateOffsetPlane(base, height)
	require.NoError(t, err)

	build := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		pts := make([]*sketch.Point, n)
		for k := range n {
			theta := 2 * math.Pi * float64(k) / float64(n)
			pts[k] = s.CreatePoint(r*math.Cos(theta), r*math.Sin(theta))
		}
		s.Fix(pts[0])
		for i := range pts {
			s.CreateLine(pts[i], pts[(i+1)%len(pts)])
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		profiles := s.Profiles()
		require.Len(t, profiles, 1)
		return s, profiles[0]
	}
	s0, p0 := build(base)
	s1, p1 := build(top)
	return s0, p0, s1, p1
}

// loftHoledSquares builds two congruent profiles, each a 40x40 mm square with
// a smaller 10x10 mm square hole, on parallel planes 10 mm apart — every
// correspondence LineSeg-to-LineSeg, so the build is cheap and sectionDelta
// stays zero. Every call site wants this one shape, so the height is a
// literal here rather than a threaded parameter (loftSquares' own doc
// comment states the identical convention for the fixture it wraps).
func loftHoledSquares(t *testing.T) (*sketch.Sketch, *sketch.Profile, *sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	base := w.XY()
	top, err := w.CreateOffsetPlane(base, 10)
	require.NoError(t, err)

	build := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		outer := s.CreateRectangle(-20, -20, 20, 20)
		s.Fix(outer.A)
		s.CreateRectangle(-5, -5, 5, 5)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, p := range s.Profiles() {
			if len(p.Holes) == 1 {
				return s, p
			}
		}
		t.Fatal("no holed profile among solved profiles")
		return nil, nil
	}
	s0, p0 := build(base)
	s1, p1 := build(top)
	return s0, p0, s1, p1
}

// loftWedgePlanes and loftWedgeSketch build the same cheap quarter-disc wedge
// (two radial lines plus one connecting arc) loft_arc_pairs_internal_test.go's
// own wedgeArcSketch builds, reproduced here since that helper is unexported
// in the internal package: a same-kind ArcSeg pairing, admitted by Table P row
// P5, whose station chording makes sectionDelta positive (§5.1).
func loftWedgePlanes(t *testing.T, height float64) (*sketch.World, *sketch.Plane, *sketch.Plane) {
	t.Helper()
	w := sketch.NewWorld()
	top, err := w.CreateOffsetPlane(w.XY(), height)
	require.NoError(t, err)
	return w, w.XY(), top
}

func loftWedgeSketch(t *testing.T, w *sketch.World, plane *sketch.Plane, radius float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	px := s.CreatePoint(radius, 0)
	py := s.CreatePoint(0, radius)
	s.CreateLine(origin, px)
	s.CreateLine(py, origin)
	s.CreateArc(origin, px, py)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// TestSurfaceLoftBoxIsASheet is Table W's loft row over loft_test.go's own
// congruent-squares fixture: 40x40 mm squares, 10 mm apart. The solid's own
// TestLoftBuildsCongruentSquares proves 10 faces (8 walls + 2 caps); the sheet
// keeps the 8 walls alone. Perimeter*height = 160*10 = 1600 mm^2 by hand.
func TestSurfaceLoftBoxIsASheet(t *testing.T) {
	t.Parallel()
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	sheet, err := doc.Loft(t.Context(), s0, p0, s1, p1, decad.WithSurfaceResult())
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.False(t, sheet.IsSolid())
	_, err = sheet.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	_, err = sheet.Centroid()
	require.ErrorIs(t, err, decad.ErrNotSolid)

	decadtest.MeasuresArea(t, sheet, units.SquareMillimeters(1600))
	requireSheetWithFreeEdges(t, sheet)

	t0, tp0, t1, tp1 := loftSquares(t, 20, 20)
	solidDoc := decad.New()
	solid, err := solidDoc.Loft(t.Context(), t0, tp0, t1, tp1)
	require.NoError(t, err)
	require.Len(t, solid.Faces(), len(sheet.Faces())+2)

	solidBox, err := solid.Bounds()
	require.NoError(t, err)
	sheetBox, err := sheet.Bounds()
	require.NoError(t, err)
	require.Equal(t, solidBox, sheetBox)

	require.Len(t, sheet.Lumps(), 1)
	shells := sheet.Shells()
	require.Len(t, shells, 1)
	require.True(t, shells[0].IsOpen())
	require.False(t, shells[0].IsVoid())
}

// TestSurfaceLoftCapBoundsComposeOnInexactFixture proves the area-bound
// composition (§4.3) on a fixture whose caps are NOT exactly representable: a
// regular pentagon, unlike loftSquares' own square caps, which are exact and
// would prove nothing about composition.
func TestSurfaceLoftCapBoundsComposeOnInexactFixture(t *testing.T) {
	t.Parallel()
	const n, r, height = 5, 10.0, 10.0

	s0, p0, s1, p1 := loftNGonAt(t, n, r, height)
	solidDoc := decad.New()
	solid, err := solidDoc.Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)

	capStartFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(solid))).Exactly(1).SelectFaces(solid)
	require.NoError(t, err)
	capStartArea, err := capStartFaces[0].Area()
	require.NoError(t, err)
	capEndFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(solid))).Exactly(1).SelectFaces(solid)
	require.NoError(t, err)
	capEndArea, err := capEndFaces[0].Area()
	require.NoError(t, err)

	// Both caps must actually be inexact, or this fixture proves nothing
	// about the composition — a square's cap area is exactly representable
	// and would pass this test vacuously.
	require.Greater(t, capStartArea.Bound.Base(), 0.0)
	require.Greater(t, capEndArea.Bound.Base(), 0.0)

	s2, p2, s3, p3 := loftNGonAt(t, n, r, height)
	sheetDoc := decad.New()
	sheet, err := sheetDoc.Loft(t.Context(), s2, p2, s3, p3, decad.WithSurfaceResult())
	require.NoError(t, err)

	solidArea, err := solid.Area()
	require.NoError(t, err)
	sheetArea, err := sheet.Area()
	require.NoError(t, err)

	wantValue := solidArea.Value.Base() - capStartArea.Value.Base() - capEndArea.Value.Base()
	require.Equal(t, wantValue, sheetArea.Value.Base())

	wantBound := solidArea.Bound.Base() + capStartArea.Bound.Base() + capEndArea.Bound.Base()
	t.Logf("cap bounds: start=%g end=%g solid=%g sheet=%g want=%g",
		capStartArea.Bound.Base(), capEndArea.Bound.Base(), solidArea.Bound.Base(), sheetArea.Bound.Base(), wantBound)
	require.GreaterOrEqual(t, sheetArea.Bound.Base(), wantBound)
	require.InDelta(t, wantBound, sheetArea.Bound.Base(), 1e-9)
	// The two float subtractions add their own exact rounding gaps beside all
	// three input bounds. Compare as rationals, so this assertion does not
	// depend on the host's FMA behavior or a pinned decimal threshold.
	rat := func(v float64) *big.Rat { return new(big.Rat).SetFloat64(v) }
	mid := solidArea.Value.Base() - capStartArea.Value.Base()
	exactMid := new(big.Rat).Sub(rat(solidArea.Value.Base()), rat(capStartArea.Value.Base()))
	firstRound := new(big.Rat).Sub(exactMid, rat(mid))
	firstRound.Abs(firstRound)
	end := mid - capEndArea.Value.Base()
	exactEnd := new(big.Rat).Sub(rat(mid), rat(capEndArea.Value.Base()))
	secondRound := new(big.Rat).Sub(exactEnd, rat(end))
	secondRound.Abs(secondRound)
	needed := new(big.Rat).Add(rat(solidArea.Bound.Base()), rat(capStartArea.Bound.Base()))
	needed.Add(needed, rat(capEndArea.Bound.Base()))
	needed.Add(needed, firstRound)
	needed.Add(needed, secondRound)
	require.GreaterOrEqual(t, rat(sheetArea.Bound.Base()).Cmp(needed), 0,
		"the sheet area bound must include both cap bounds and both subtraction roundings")
}

// TestSurfaceLoftRolesResolveThroughFaceCreatedBy is §4.2: every wall face
// keeps its side(i,j,k) role resolvable, while CapStart/CapEnd mint refs that
// match nothing on a sheet.
func TestSurfaceLoftRolesResolveThroughFaceCreatedBy(t *testing.T) {
	t.Parallel()
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	sheet, err := doc.Loft(t.Context(), s0, p0, s1, p1, decad.WithSurfaceResult())
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

// TestSurfaceLoftStaysASheetThroughPlacement is §4.2's Placed/Duplicate/
// PlacedCopy claim for Loft: each re-evaluates the payload and reproduces the
// sheet with no further code.
func TestSurfaceLoftStaysASheetThroughPlacement(t *testing.T) {
	t.Parallel()
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	sheet, err := doc.Loft(t.Context(), s0, p0, s1, p1, decad.WithSurfaceResult())
	require.NoError(t, err)

	motion, err := r3.Translation(r3.NewVec(50, 0, 0))
	require.NoError(t, err)
	placed, err := sheet.Placed(t.Context(), motion)
	require.NoError(t, err)
	requireSheetWithFreeEdges(t, placed)

	dup, err := placed.Duplicate(t.Context())
	require.NoError(t, err)
	requireSheetWithFreeEdges(t, dup)

	copyMotion, err := r3.Translation(r3.NewVec(0, 50, 0))
	require.NoError(t, err)
	copied, err := dup.PlacedCopy(t.Context(), copyMotion)
	require.NoError(t, err)
	requireSheetWithFreeEdges(t, copied)
}

// TestSurfaceLoftHoledProfileReportsDisconnectedLumps is the loft analogue of
// decision A (docs/surface-design.md §2.2, TestSurfaceRevolveHoledProfileReport
// sDisconnectedLumps): a holed profile lofted as a surface has an outer wall
// tube and a hole wall tube that touch nowhere once their shared cap is
// omitted, so Body.Lumps() must report both. The solid built from the same
// profile keeps its one lump, joined through its two caps.
func TestSurfaceLoftHoledProfileReportsDisconnectedLumps(t *testing.T) {
	t.Parallel()
	s0, p0, s1, p1 := loftHoledSquares(t)
	doc := decad.New()
	sheet, err := doc.Loft(t.Context(), s0, p0, s1, p1, decad.WithSurfaceResult())
	require.NoError(t, err)
	require.Len(t, sheet.Lumps(), 2)
	for _, l := range sheet.Lumps() {
		require.Len(t, l.Shells(), 1)
		require.False(t, l.Shells()[0].IsVoid())
		require.True(t, l.Shells()[0].IsOpen())
	}

	s2, p2, s3, p3 := loftHoledSquares(t)
	solidDoc := decad.New()
	solid, err := solidDoc.Loft(t.Context(), s2, p2, s3, p3)
	require.NoError(t, err)
	require.Len(t, solid.Lumps(), 1)
}

// TestSurfaceLoftSheetVerifiesSound is docs/loft-design.md §9 Table D row D6's
// own zero-displacement case: a LineSeg-only surface-result loft's crossing
// audit already ran over its complete triangle set, so its walls alone earn
// ValidityValid and the report reads Sound.
func TestSurfaceLoftSheetVerifiesSound(t *testing.T) {
	t.Parallel()
	s0, p0, s1, p1 := loftSquares(t, 20, 10)
	doc := decad.New()
	sheet, err := doc.Loft(t.Context(), s0, p0, s1, p1, decad.WithSurfaceResult())
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Passed())
	require.Empty(t, report.Diagnostics)

	br := decadtest.FindBodyReport(t, report, sheet)
	require.Equal(t, decad.ValidityValid, br.Validity.Outcome)
	require.Nil(t, br.Region)
}

// TestSurfaceLoftChordedPairVerifiesUndecided is the honest answer at a
// positive section displacement: a same-kind ArcSeg pairing chords its walls,
// so sectionDelta is positive and the sheet's own fourth leg has nothing to
// stand on (docs/surface-design.md §9.1).
func TestSurfaceLoftChordedPairVerifiesUndecided(t *testing.T) {
	t.Parallel()
	w, base, top := loftWedgePlanes(t, 10)
	s0, p0 := loftWedgeSketch(t, w, base, 5)
	s1, p1 := loftWedgeSketch(t, w, top, 5)
	doc := decad.New()
	sheet, err := doc.Loft(t.Context(), s0, p0, s1, p1, decad.WithSurfaceResult())
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br := decadtest.FindBodyReport(t, report, sheet)
	require.Equal(t, decad.ValidityUndecided, br.Validity.Outcome)
	require.Nil(t, br.Region)
	require.Len(t, br.Validity.Diagnostics, 1)
	require.Equal(t, decad.DiagUndecidedValidity, br.Validity.Diagnostics[0].Code)
	for _, d := range br.Diagnostics {
		require.NotEqual(t, decad.DiagInvalidBody, d.Code)
	}
}

// TestSurfaceLoftTessellationRestatesWalls checks that a surface-result loft
// publishes its wall triangles with the free section rims intact.
func TestSurfaceLoftTessellationRestatesWalls(t *testing.T) {
	t.Parallel()
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	solidDoc := decad.New()
	solid, err := solidDoc.Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	solidMesh, err := solid.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	doc := decad.New()
	sheet, err := doc.Loft(t.Context(), s0, p0, s1, p1, decad.WithSurfaceResult())
	require.NoError(t, err)

	mesh, err := sheet.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.Len(t, sheet.Faces(), 8)
	free, err := decad.Edges(decad.Free()).SelectEdges(sheet)
	require.NoError(t, err)
	require.Len(t, free, 8)
	require.Len(t, solidMesh.Triangles(), 12)
	require.Len(t, mesh.Triangles(), 8)
	require.Equal(t, solidMesh.Triangles()[:8], mesh.Triangles())
	require.Equal(t, solidMesh.Vertices(), mesh.Vertices())
	require.Equal(t, 8, directedEdgeCensus(t, mesh))
	require.Len(t, mesh.SourceFaces(), 8)
	live := map[*decad.Face]struct{}{}
	for _, f := range sheet.Faces() {
		live[f] = struct{}{}
	}
	for _, f := range mesh.SourceFaces() {
		require.Contains(t, live, f)
	}
	require.InDelta(t, 1600.0, meshTriangleArea(mesh), 1e-9)
	area, err := sheet.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-1600), area.Bound.Base())
	require.False(t, mesh.VolumeVerified())
	t.Logf("sheet mesh: faces=%d free_edges=%d triangles=%d area=%g bound=%g",
		len(sheet.Faces()), len(free), len(mesh.Triangles()), area.Value.Base(), area.Bound.Base())

	for _, format := range []string{"STL", "OBJ"} {
		var plain, verified bytes.Buffer
		if format == "STL" {
			require.NoError(t, sheet.STL(&plain))
			require.NoError(t, sheet.STL(&verified, decad.WithVerification(decad.VerifyAll)))
			require.Equal(t, 8, strings.Count(plain.String(), "  facet normal "))
		} else {
			require.NoError(t, sheet.OBJ(&plain))
			require.NoError(t, sheet.OBJ(&verified, decad.WithVerification(decad.VerifyAll)))
			require.Equal(t, 8, strings.Count(plain.String(), "\nf "))
		}
		require.Equal(t, plain.Bytes(), verified.Bytes())
	}
	rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	placed, err := sheet.PlacedCopy(t.Context(), rotation)
	require.NoError(t, err)
	placedArea, err := placed.Area()
	require.NoError(t, err)
	require.Positive(t, placedArea.Bound.Base())
	require.LessOrEqual(t, math.Abs(placedArea.Value.Base()-1600), placedArea.Bound.Base())
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	require.Len(t, placedMesh.Triangles(), 8)
	require.Positive(t, placedMesh.Bound().Base())
	require.False(t, placedMesh.VolumeVerified())
	t.Logf("placed sheet area=%g bound=%g mesh_bound=%g", placedArea.Value.Base(),
		placedArea.Bound.Base(), placedMesh.Bound().Base())
}

// TestSurfaceLoftShellOpenAgreesWithFreeEdgeDerivation catches a builder that
// mints a free edge and forgets the Shell.open flag — or marks a shell open
// with none — across a plain box, a holed box, and each one's solid control.
func TestSurfaceLoftShellOpenAgreesWithFreeEdgeDerivation(t *testing.T) {
	t.Parallel()

	t.Run("box/sheet", func(t *testing.T) {
		t.Parallel()
		s0, p0, s1, p1 := loftSquares(t, 20, 20)
		doc := decad.New()
		body, err := doc.Loft(t.Context(), s0, p0, s1, p1, decad.WithSurfaceResult())
		require.NoError(t, err)
		requireOpenAgreesWithFreeEdges(t, body)
	})
	t.Run("box/solid", func(t *testing.T) {
		t.Parallel()
		s0, p0, s1, p1 := loftSquares(t, 20, 20)
		doc := decad.New()
		body, err := doc.Loft(t.Context(), s0, p0, s1, p1)
		require.NoError(t, err)
		requireOpenAgreesWithFreeEdges(t, body)
	})
	t.Run("holed/sheet", func(t *testing.T) {
		t.Parallel()
		s0, p0, s1, p1 := loftHoledSquares(t)
		doc := decad.New()
		body, err := doc.Loft(t.Context(), s0, p0, s1, p1, decad.WithSurfaceResult())
		require.NoError(t, err)
		requireOpenAgreesWithFreeEdges(t, body)
	})
	t.Run("holed/solid", func(t *testing.T) {
		t.Parallel()
		s0, p0, s1, p1 := loftHoledSquares(t)
		doc := decad.New()
		body, err := doc.Loft(t.Context(), s0, p0, s1, p1)
		require.NoError(t, err)
		requireOpenAgreesWithFreeEdges(t, body)
	})
}

// TestSurfaceLoftWithSurfaceResultIsIdempotent is loft.go's own repeat rule: a
// repeated WithSurfaceResult() is accepted, deliberately unlike
// WithLoftAlignment's repeat-is-ErrDegenerate rule.
func TestSurfaceLoftWithSurfaceResultIsIdempotent(t *testing.T) {
	t.Parallel()
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	sheet, err := doc.Loft(t.Context(), s0, p0, s1, p1, decad.WithSurfaceResult(), decad.WithSurfaceResult())
	require.NoError(t, err)
	requireSheetWithFreeEdges(t, sheet)
}
