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

// This file is docs/surface-design.md's Table W and T6-shaped public-surface
// tests for WithSurfaceResult() on Revolve: one test per Table W row (a
// partial sweep clear of the axis, a partial sweep meeting it, and a full
// revolution, each on or off the axis), the design's T6 (a half-disc revolved
// a full turn into a closed sphere sheet), a bound-composition proof on an
// inexact fixture, and the shared IsOpen/lump-count obligations decisions A
// and B added. Every fixture reuses annularSketch/solidSketch/
// semicircleSketch/uAxis (revolve_test.go) or builds its own off-axis
// polygon, since those fixtures sit on or straddle the axis.

// ngonOffAxisSketch builds a solved regular n-gon of circumradius r centered
// at (u, v) = (0, vCenter), wound CCW: every side length off a small set of
// special angles is irrational, the same bound-composition fixture
// regularNGonSketch (extrude_bounds_test.go) is for extrude, offset clear of
// the revolve axis (v = 0) rather than centered on it.
func ngonOffAxisSketch(t *testing.T, n int, r, vCenter float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	sp := make([]*sketch.Point, n)
	for k := range n {
		theta := 2 * math.Pi * float64(k) / float64(n)
		sp[k] = s.CreatePoint(r*math.Cos(theta), vCenter+r*math.Sin(theta))
	}
	s.Fix(sp[0])
	for i := range sp {
		s.CreateLine(sp[i], sp[(i+1)%len(sp)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// annularHoledSketch builds annularSketch's own rectangle (u∈[0,10], v∈[5,15])
// with a circular hole entirely inside it and clear of the axis, so a full
// revolution's outer and hole loops each sweep their own closed surface that
// touches the other nowhere.
func annularHoledSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 5, 10, 15)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(5, 10), 2)
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

// quarterTurn is the 90° AngleExtent every partial-sweep row below uses: a
// degree-stated extent denotes an exact fraction of a turn
// (docs/evaluator-design.md §6), so it is the sweep every hand computation in
// this file is checked against.
var quarterTurn = decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along}

// TestSurfaceRevolvePartialClearOfAxisIsASheet is Table W's partial-sweep,
// clear-of-axis row: annularSketch's rectangle (u∈[0,10], v∈[5,15]) revolved
// a quarter turn. Pappus by hand: the four walls' first moments about the
// axis sum to 10·(15+5+10+10) = 400, times the quarter-turn angle π/2, so
// Area is 200π mm².
func TestSurfaceRevolvePartialClearOfAxisIsASheet(t *testing.T) {
	t.Parallel()
	s, p := annularSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, quarterTurn, decad.WithSurfaceResult())
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.False(t, sheet.IsSolid())
	_, err = sheet.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	_, err = sheet.Centroid()
	require.ErrorIs(t, err, decad.ErrNotSolid)

	decadtest.MeasuresArea(t, sheet, units.SquareMillimeters(200*math.Pi))

	require.Len(t, sheet.Faces(), 4)
	free, err := decad.Edges(decad.Free()).Exactly(8).SelectEdges(sheet)
	require.NoError(t, err)
	for _, e := range free {
		require.True(t, e.IsFree())
	}
	var interior int
	for _, e := range sheet.Edges() {
		if !e.IsFree() {
			interior++
		}
	}
	require.Equal(t, 4, interior, "one junction edge per pair of adjacent walls")

	require.Len(t, sheet.Lumps(), 1)
	shells := sheet.Shells()
	require.Len(t, shells, 1)
	require.True(t, shells[0].IsOpen())
	require.False(t, shells[0].IsVoid())

	// Bounds (§4.3): unchanged from what the same profile builds as a solid,
	// value AND bound — the extent reading never consults surfaceResult.
	s2, p2 := annularSketch(t)
	solidDoc := decad.New()
	solid, err := solidDoc.Revolve(s2, p2, uAxis, quarterTurn)
	require.NoError(t, err)
	solidBox, err := solid.Bounds()
	require.NoError(t, err)
	sheetBox, err := sheet.Bounds()
	require.NoError(t, err)
	require.Equal(t, solidBox, sheetBox)
}

// TestSurfaceRevolvePartialMeetingAxisOmitsOnAxisEdge is Table W's
// partial-sweep, meets-axis row: solidSketch's rectangle (u∈[0,10], v∈[0,8]),
// whose bottom edge lies on the axis, revolved a quarter turn. Pappus by
// hand: the cylinder wall's moment is 10·8 = 80 and each radial wall's is
// 8·4 = 32, so the three walls sum to 144, times π/2, giving 72π mm². Unlike
// the extrude/prism case, the solid's on-axis edge (shared by both its caps)
// has no wall of its own to keep it alive once the caps are omitted, so it is
// not merely free — it is gone from the sheet entirely.
func TestSurfaceRevolvePartialMeetingAxisOmitsOnAxisEdge(t *testing.T) {
	t.Parallel()
	s, p := solidSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, quarterTurn, decad.WithSurfaceResult())
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, sheet.Kind())
	decadtest.MeasuresArea(t, sheet, units.SquareMillimeters(72*math.Pi))
	require.Len(t, sheet.Faces(), 3)

	free, err := decad.Edges(decad.Free()).Exactly(6).SelectEdges(sheet)
	require.NoError(t, err)
	require.Len(t, free, 6)

	// The free boundary of a manifold-with-boundary sheet is always a union
	// of CLOSED curves, never a chain with loose ends: every vertex the free
	// edges touch is incident to exactly two of them, here forming one
	// six-edge loop through the two on-axis corners and the two off-axis
	// ones (docs/surface-design.md §2.2).
	degree := map[*decad.Vertex]int{}
	for _, e := range free {
		degree[e.Start()]++
		degree[e.End()]++
	}
	require.Len(t, degree, 6)
	for v, d := range degree {
		require.Equalf(t, 2, d, "vertex %p has free-edge degree %d, want 2", v, d)
	}

	// The solid built from the same profile has exactly one Line3 edge with
	// both endpoints on the axis — the edge shared by its two caps
	// (docs/evaluator-design.md §6) — and the sheet has none at all.
	s2, p2 := solidSketch(t)
	solidDoc := decad.New()
	solid, err := solidDoc.Revolve(s2, p2, uAxis, quarterTurn)
	require.NoError(t, err)
	require.Equal(t, 1, onAxisLine3Count(solid))
	require.Equal(t, 0, onAxisLine3Count(sheet))

	// Bounds (§4.3): unchanged from the solid's, value AND bound.
	solidBox, err := solid.Bounds()
	require.NoError(t, err)
	sheetBox, err := sheet.Bounds()
	require.NoError(t, err)
	require.Equal(t, solidBox, sheetBox)
}

// onAxisLine3Count counts b's Line3 edges whose both endpoints sit on the
// revolve axis (the u axis: y == 0, z == 0 in world space, uAxis's own line).
func onAxisLine3Count(b *decad.Body) int {
	n := 0
	for _, e := range b.Edges() {
		if _, ok := e.Curve().(decad.Line3); !ok {
			continue
		}
		start, end := e.Start().Position().Value, e.End().Position().Value
		if start.Y == 0 && start.Z == 0 && end.Y == 0 && end.Z == 0 {
			n++
		}
	}
	return n
}

// TestSurfaceRevolveFullTurnClearOfAxisIsClosed is Table W's full-revolution,
// clear-of-axis row: a full revolution mints no closing face to omit, so
// WithSurfaceResult() there changes no face at all — the result is a CLOSED
// sheet, not a refusal (§2.1, §4.1).
func TestSurfaceRevolveFullTurnClearOfAxisIsClosed(t *testing.T) {
	t.Parallel()
	s, p := annularSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.Len(t, sheet.Faces(), 4)
	shells := sheet.Shells()
	require.Len(t, shells, 1)
	require.False(t, shells[0].IsOpen())
	require.False(t, shells[0].IsVoid())

	_, err = decad.Edges(decad.Free()).SelectEdges(sheet)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	// A full revolution's face set is identical whether or not the caps are
	// omitted — there are none to omit — so Area is bit-identical to the
	// solid's, not merely within its bound.
	s2, p2 := annularSketch(t)
	solidDoc := decad.New()
	solid, err := solidDoc.Revolve(s2, p2, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	solidArea, err := solid.Area()
	require.NoError(t, err)
	sheetArea, err := sheet.Area()
	require.NoError(t, err)
	require.Equal(t, solidArea.Value.Base(), sheetArea.Value.Base())
	require.Equal(t, solidArea.Bound.Base(), sheetArea.Bound.Base())

	// Bounds (§4.3): unchanged from the solid's, value AND bound.
	solidBox, err := solid.Bounds()
	require.NoError(t, err)
	sheetBox, err := sheet.Bounds()
	require.NoError(t, err)
	require.Equal(t, solidBox, sheetBox)
}

// TestSurfaceRevolveHalfDiscFullTurnIsClosedSphereSheet is docs/surface-design
// .md's T6: a half-disc revolved a full turn about its diameter is a closed
// sphere sheet.
func TestSurfaceRevolveHalfDiscFullTurnIsClosedSphereSheet(t *testing.T) {
	t.Parallel()
	s, p := semicircleSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.Len(t, sheet.Faces(), 1)
	_, ok := sheet.Faces()[0].Surface().(decad.Sphere)
	require.True(t, ok)

	_, err = decad.Edges(decad.Free()).SelectEdges(sheet)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	shells := sheet.Shells()
	require.Len(t, shells, 1)
	require.False(t, shells[0].IsOpen())

	_, err = sheet.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	_, err = sheet.Centroid()
	require.ErrorIs(t, err, decad.ErrNotSolid)

	// Bounds (§4.3): unchanged from the solid's, value AND bound.
	s2, p2 := semicircleSketch(t)
	solidDoc := decad.New()
	solid, err := solidDoc.Revolve(s2, p2, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	solidBox, err := solid.Bounds()
	require.NoError(t, err)
	sheetBox, err := sheet.Bounds()
	require.NoError(t, err)
	require.Equal(t, solidBox, sheetBox)
}

// TestSurfaceRevolveCapBoundsComposeOnInexactFixture proves the area-bound
// composition (§4.3) on a fixture whose caps are NOT exactly representable:
// a regular 12-gon off the axis, unlike this file's other fixtures, whose
// caps are exact rectangles or circles and prove nothing about composition.
func TestSurfaceRevolveCapBoundsComposeOnInexactFixture(t *testing.T) {
	t.Parallel()
	const n, r, vCenter = 12, 3.0, 10.0

	s, p := ngonOffAxisSketch(t, n, r, vCenter)
	solidDoc := decad.New()
	solid, err := solidDoc.Revolve(s, p, uAxis, quarterTurn)
	require.NoError(t, err)

	capStartFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(solid))).Exactly(1).SelectFaces(solid)
	require.NoError(t, err)
	capStartArea, err := capStartFaces[0].Area()
	require.NoError(t, err)
	capEndFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(solid))).Exactly(1).SelectFaces(solid)
	require.NoError(t, err)
	capEndArea, err := capEndFaces[0].Area()
	require.NoError(t, err)

	// Both caps must actually be inexact, or this fixture proves nothing.
	require.Greater(t, capStartArea.Bound.Base(), 0.0)
	require.Greater(t, capEndArea.Bound.Base(), 0.0)

	s2, p2 := ngonOffAxisSketch(t, n, r, vCenter)
	sheetDoc := decad.New()
	sheet, err := sheetDoc.Revolve(s2, p2, uAxis, quarterTurn, decad.WithSurfaceResult())
	require.NoError(t, err)

	solidArea, err := solid.Area()
	require.NoError(t, err)
	sheetArea, err := sheet.Area()
	require.NoError(t, err)

	wantValue := solidArea.Value.Base() - capStartArea.Value.Base() - capEndArea.Value.Base()
	require.Equal(t, wantValue, sheetArea.Value.Base())

	wantBound := solidArea.Bound.Base() + capStartArea.Bound.Base() + capEndArea.Bound.Base()
	require.GreaterOrEqual(t, sheetArea.Bound.Base(), wantBound)
	require.InDelta(t, wantBound, sheetArea.Bound.Base(), 1e-9)
}

// revolveOpenCase is one Table W row's fixture plus the extent to build it
// with, shared by the IsOpen-derivation table and the Placed test.
type revolveOpenCase struct {
	name   string
	sketch func(t *testing.T) (*sketch.Sketch, *sketch.Profile)
	extent decad.AngularExtent
}

func revolveOpenCases() []revolveOpenCase {
	return []revolveOpenCase{
		{"partial clear of axis", annularSketch, quarterTurn},
		{"partial meeting axis", solidSketch, quarterTurn},
		{"full turn clear of axis", annularSketch, decad.FullRevolution{}},
		{"full turn half-disc", semicircleSketch, decad.FullRevolution{}},
	}
}

// TestSurfaceRevolveShellOpenAgreesWithFreeEdgeDerivation catches a builder
// that mints a free edge and forgets the Shell.open flag — or marks a shell
// open with none — across every Table W row and its solid control.
func TestSurfaceRevolveShellOpenAgreesWithFreeEdgeDerivation(t *testing.T) {
	t.Parallel()
	for _, tc := range revolveOpenCases() {
		t.Run(tc.name+"/sheet", func(t *testing.T) {
			t.Parallel()
			s, p := tc.sketch(t)
			doc := decad.New()
			body, err := doc.Revolve(s, p, uAxis, tc.extent, decad.WithSurfaceResult())
			require.NoError(t, err)
			requireOpenAgreesWithFreeEdges(t, body)
		})
		t.Run(tc.name+"/solid", func(t *testing.T) {
			t.Parallel()
			s, p := tc.sketch(t)
			doc := decad.New()
			body, err := doc.Revolve(s, p, uAxis, tc.extent)
			require.NoError(t, err)
			requireOpenAgreesWithFreeEdges(t, body)
		})
	}
}

func requireOpenAgreesWithFreeEdges(t *testing.T, b *decad.Body) {
	t.Helper()
	hasFree := false
	for _, e := range b.Edges() {
		if e.IsFree() {
			hasFree = true
			break
		}
	}
	for _, sh := range b.Shells() {
		require.Equal(t, hasFree, sh.IsOpen())
	}
}

// TestSurfaceRevolveStaysASheetThroughPlacement is §4.2's Placed/Duplicate/
// PlacedCopy claim for Revolve: each re-evaluates the payload and reproduces
// the sheet with no further code.
func TestSurfaceRevolveStaysASheetThroughPlacement(t *testing.T) {
	t.Parallel()
	s, p := annularSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, quarterTurn, decad.WithSurfaceResult())
	require.NoError(t, err)

	offset := r3.NewVec(50, 0, 0)
	motion, err := r3.Translation(offset)
	require.NoError(t, err)

	placed, err := sheet.Placed(motion)
	require.NoError(t, err)
	requireRevolveSheetWithFreeEdges(t, placed, 8)

	dup, err := placed.Duplicate()
	require.NoError(t, err)
	requireRevolveSheetWithFreeEdges(t, dup, 8)

	copyMotion, err := r3.Translation(r3.NewVec(0, 50, 0))
	require.NoError(t, err)
	copied, err := dup.PlacedCopy(copyMotion)
	require.NoError(t, err)
	requireRevolveSheetWithFreeEdges(t, copied, 8)
}

func requireRevolveSheetWithFreeEdges(t *testing.T, b *decad.Body, n int) {
	t.Helper()
	require.Equal(t, decad.BodySheet, b.Kind())
	free, err := decad.Edges(decad.Free()).Exactly(n).SelectEdges(b)
	require.NoError(t, err)
	require.Len(t, free, n)
}

// TestSurfaceRevolveRolesResolveThroughFaceCreatedBy is §4.2: every wall face
// keeps its side(i, j) role resolvable, while CapStart/CapEnd mint refs that
// match nothing on a sheet.
func TestSurfaceRevolveRolesResolveThroughFaceCreatedBy(t *testing.T) {
	t.Parallel()
	s, p := annularSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, quarterTurn, decad.WithSurfaceResult())
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

// TestSurfaceRevolveSheetVerifiesUndecided mirrors extrude's holding fix
// (docs/surface-design.md §9.1, §14) for Revolve: the manifold-with-boundary
// audit lands later, so a sheet reads ValidityUndecided rather than proven
// invalid.
func TestSurfaceRevolveSheetVerifiesUndecided(t *testing.T) {
	t.Parallel()
	s, p := annularSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, quarterTurn, decad.WithSurfaceResult())
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

// TestSurfaceRevolveHoledProfileReportsDisconnectedLumps is decision A: a
// holed profile revolved a full turn as a surface has an outer wall tube and
// an inner (hole) wall tube that touch nowhere once the (never-built, for a
// full sweep) closing faces are out of the picture — Body.Lumps() must report
// both. The solid built from the same profile keeps its one lump, the hole's
// own closed surface a void SHELL of it rather than a second lump.
func TestSurfaceRevolveHoledProfileReportsDisconnectedLumps(t *testing.T) {
	t.Parallel()
	s, p := annularHoledSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
	require.NoError(t, err)
	require.Len(t, sheet.Lumps(), 2)
	for _, l := range sheet.Lumps() {
		require.Len(t, l.Shells(), 1)
		require.False(t, l.Shells()[0].IsVoid())
	}

	s2, p2 := annularHoledSketch(t)
	solidDoc := decad.New()
	solid, err := solidDoc.Revolve(s2, p2, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	require.Len(t, solid.Lumps(), 1)
	require.Len(t, solid.Lumps()[0].Shells(), 2)
}
