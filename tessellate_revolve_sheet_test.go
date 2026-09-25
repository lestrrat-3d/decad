package decad_test

import (
	"bytes"
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md §10's T10, run against the REVOLVE
// sheet path: WithSurfaceResult()'s own manifold-with-boundary mesh and
// export, one test per Table W row (a partial sweep clear of the axis, a
// partial sweep meeting it, and a full turn clear of it) plus the design's
// T6 half-disc sphere. Every fixture reuses annularSketch/solidSketch/
// semicircleSketch/uAxis/quarterTurn (revolve_test.go, surface_revolve_test
// .go) or meshTriangleArea/directedEdgeCensus (tessellate_sheet_test.go).
// NEVER call decadtest.IsManifold on a sheet mesh here — see that file's own
// note.

// revolveWallTriangleCount reads a revolve solid's own mesh back for the
// count Table W's sheet row must match, rather than hardcoding it: every
// triangle whose source face is NOT one of the solid's two caps is a wall
// triangle, and a sheet omits exactly the caps (Table W).
func revolveWallTriangleCount(t *testing.T, solid *decad.Body, solidMesh *decad.Mesh) int {
	t.Helper()
	capSet := map[*decad.Face]struct{}{}
	if fs, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(solid))).SelectFaces(solid); err == nil {
		for _, f := range fs {
			capSet[f] = struct{}{}
		}
	}
	if fs, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(solid))).SelectFaces(solid); err == nil {
		for _, f := range fs {
			capSet[f] = struct{}{}
		}
	}
	n := 0
	for _, f := range solidMesh.SourceFaces() {
		if _, ok := capSet[f]; !ok {
			n++
		}
	}
	return n
}

// meshFreeChainCount counts the connected components a mesh's own free
// directed edges form over its vertex indices — the boundary CHAIN count
// docs/tessellation-design.md §1.2 reads by connected component, never by
// edge count. This is the exact fixture the sheet chain-count fix
// (PR #292, requireSheetVertexLinks) was required before: a revolve wall
// meeting the axis shares an interned pole vertex between two rims that are
// nonetheless ONE chain.
func meshFreeChainCount(t *testing.T, mesh *decad.Mesh) int {
	t.Helper()
	directed := map[[2]int]int{}
	for _, tri := range mesh.Triangles() {
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	parent := map[int]int{}
	for e, n := range directed {
		if n != 1 || directed[[2]int{e[1], e[0]}] != 0 {
			continue // not a free directed edge
		}
		a, b := meshFreeChainRoot(parent, e[0]), meshFreeChainRoot(parent, e[1])
		if a != b {
			parent[a] = b
		}
	}
	roots := map[int]struct{}{}
	for v := range parent {
		roots[meshFreeChainRoot(parent, v)] = struct{}{}
	}
	return len(roots)
}

// meshFreeChainRoot is meshFreeChainCount's own path-compressing find, a
// top-level function rather than a closure over parent: a self-referential
// closure cannot be declared and assigned in one statement (staticcheck
// S1021 does not account for the recursion), the same reason
// tessellate_sheet.go's own chainRoot/vertexChainRoot are top-level
// functions instead of closures.
func meshFreeChainRoot(parent map[int]int, v int) int {
	if _, ok := parent[v]; !ok {
		parent[v] = v
	}
	for parent[v] != v {
		parent[v] = parent[parent[v]]
		v = parent[v]
	}
	return v
}

// TestSurfaceRevolvePartialClearOfAxisSheetMeshMatchesSolid is Table W's
// partial-sweep, clear-of-axis row: annularSketch's rectangle revolved a
// quarter turn. Pappus by hand (surface_revolve_test.go): Area is 200π mm².
func TestSurfaceRevolvePartialClearOfAxisSheetMeshMatchesSolid(t *testing.T) {
	t.Parallel()
	tol := units.Millimeters(0.5)

	solidDoc := decad.New()
	s1, p1 := annularSketch(t)
	solid, err := solidDoc.Revolve(s1, p1, uAxis, quarterTurn)
	require.NoError(t, err)
	solidMesh, err := solid.Tessellate(t.Context(), tol)
	require.NoError(t, err)

	sheetDoc := decad.New()
	s2, p2 := annularSketch(t)
	sheet, err := sheetDoc.Revolve(s2, p2, uAxis, quarterTurn, decad.WithSurfaceResult())
	require.NoError(t, err)
	sheetMesh, err := sheet.Tessellate(t.Context(), tol)
	require.NoError(t, err)

	// The source-face set carries no cap role: every mesh facet's source is
	// one of the sheet's own 4 live faces, and neither cap role resolves.
	liveFaces := map[*decad.Face]struct{}{}
	for _, f := range sheet.Faces() {
		liveFaces[f] = struct{}{}
	}
	require.Len(t, liveFaces, 4)
	for _, f := range sheetMesh.SourceFaces() {
		_, ok := liveFaces[f]
		require.True(t, ok, `every facet's source face is one the sheet actually carries`)
	}
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapStart(sheet))).SelectFaces(sheet)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapEnd(sheet))).SelectFaces(sheet)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	// The triangle count against the solid's, read back from the solid's own
	// mesh rather than hardcoded, so a chording change cannot silently pass.
	wantTris := revolveWallTriangleCount(t, solid, solidMesh)
	require.Len(t, sheetMesh.Triangles(), wantTris)

	// A vertex table bit-equal to the solid's: both builds chord the exact
	// same record identically.
	require.Equal(t, solidMesh.Vertices(), sheetMesh.Vertices())

	// The triangle list equal to the solid's LEADING wall triangles: caps
	// are appended last (docs/tessellation-design.md §9).
	require.Equal(t, solidMesh.Triangles()[:wantTris], sheetMesh.Triangles())

	// A summed area matching the closed form (within a bound comfortably
	// above the chord deficit a quarter-turn wall must carry, and well
	// below the gap a wrong closed form would open) and converging on the
	// analytic value as the tolerance shrinks.
	const analytic = 200 * math.Pi
	coarseArea := meshTriangleArea(sheetMesh)
	require.InDelta(t, analytic, coarseArea, 15.0)
	coarseDiff := math.Abs(coarseArea - analytic)
	require.Positive(t, coarseDiff, `a chorded quarter-turn wall must fall strictly short of the analytic curved area`)

	finerMesh, err := sheet.Tessellate(t.Context(), units.Millimeters(0.05))
	require.NoError(t, err)
	finerDiff := math.Abs(meshTriangleArea(finerMesh) - analytic)
	require.Less(t, finerDiff, coarseDiff, `a finer tolerance must converge closer on the analytic area`)
}

// TestSurfaceRevolvePartialMeetingAxisSheetFreeBoundaryIsOneCycle is Table
// W's partial-sweep, meets-axis row: solidSketch's rectangle revolved a
// quarter turn. This is the fixture that fails without PR #292's chain-count
// fix — the two rims sharing the interned pole vertex are ONE connected
// chain, not two.
func TestSurfaceRevolvePartialMeetingAxisSheetFreeBoundaryIsOneCycle(t *testing.T) {
	t.Parallel()
	s, p := solidSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, quarterTurn, decad.WithSurfaceResult())
	require.NoError(t, err)

	mesh, err := sheet.Tessellate(t.Context(), units.Millimeters(0.5))
	require.NoError(t, err)

	free := directedEdgeCensus(t, mesh)
	require.Positive(t, free)
	require.Equal(t, 1, meshFreeChainCount(t, mesh),
		`the free boundary through the two shared pole vertices is ONE connected chain, not two`)
}

// TestSurfaceRevolveFullTurnClearOfAxisSheetMeshBitEqualsSolid is Table W's
// full-revolution, clear-of-axis row: a full turn mints no cap in either
// kind, so the sheet mesh is bit-identical to the solid's and carries no
// free edge at all.
func TestSurfaceRevolveFullTurnClearOfAxisSheetMeshBitEqualsSolid(t *testing.T) {
	t.Parallel()
	tol := units.Millimeters(0.5)

	solidDoc := decad.New()
	s1, p1 := annularSketch(t)
	solid, err := solidDoc.Revolve(s1, p1, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	solidMesh, err := solid.Tessellate(t.Context(), tol)
	require.NoError(t, err)

	sheetDoc := decad.New()
	s2, p2 := annularSketch(t)
	sheet, err := sheetDoc.Revolve(s2, p2, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
	require.NoError(t, err)
	sheetMesh, err := sheet.Tessellate(t.Context(), tol)
	require.NoError(t, err)

	require.Equal(t, solidMesh.Vertices(), sheetMesh.Vertices())
	require.Equal(t, solidMesh.Triangles(), sheetMesh.Triangles())
	require.Zero(t, directedEdgeCensus(t, sheetMesh),
		`a full revolution mints no cap in either kind, so its sheet mesh carries no free edge at all`)
}

// TestSurfaceRevolveHalfDiscFullTurnSheetMeshMatchesSolid is docs/surface-
// design.md's T6, tessellated: a half-disc revolved a full turn about its
// diameter is a closed sphere sheet whose mesh is bit-identical to the
// solid's.
func TestSurfaceRevolveHalfDiscFullTurnSheetMeshMatchesSolid(t *testing.T) {
	t.Parallel()
	tol := units.Millimeters(0.5)

	solidDoc := decad.New()
	s1, p1 := semicircleSketch(t)
	solid, err := solidDoc.Revolve(s1, p1, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	solidMesh, err := solid.Tessellate(t.Context(), tol)
	require.NoError(t, err)

	sheetDoc := decad.New()
	s2, p2 := semicircleSketch(t)
	sheet, err := sheetDoc.Revolve(s2, p2, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
	require.NoError(t, err)
	sheetMesh, err := sheet.Tessellate(t.Context(), tol)
	require.NoError(t, err)

	require.Equal(t, solidMesh.Vertices(), sheetMesh.Vertices())
	require.Equal(t, solidMesh.Triangles(), sheetMesh.Triangles())
	require.Zero(t, directedEdgeCensus(t, sheetMesh))

	// The bound matches the solid's and falls at a finer tolerance. Never
	// assert a nonzero Bound literal — FMA differs amd64 vs arm64.
	require.True(t, sheetMesh.Bound().Equal(solidMesh.Bound(), 1e-12))
	require.Positive(t, sheetMesh.Bound().Mag())

	finerMesh, err := sheet.Tessellate(t.Context(), units.Millimeters(0.05))
	require.NoError(t, err)
	require.Less(t, finerMesh.Bound().Mag(), sheetMesh.Bound().Mag())

	var buf1, buf2 bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &buf1, sheet, units.Millimeters(0.1)))
	require.NoError(t, export.STL(t.Context(), &buf2, sheet, units.Millimeters(0.1)))
	require.Equal(t, buf1.String(), buf2.String())
}

// TestSurfaceRevolveSheetTessellationDeterminism is T10's determinism leg:
// the one-entry cache, two fresh documents producing equal meshes and
// byte-identical STL/OBJ, and a canceled context returning ctx.Err()
// unchanged.
func TestSurfaceRevolveSheetTessellationDeterminism(t *testing.T) {
	t.Parallel()
	build := func(t *testing.T) *decad.Body {
		t.Helper()
		s, p := annularSketch(t)
		doc := decad.New()
		sheet, err := doc.Revolve(s, p, uAxis, quarterTurn, decad.WithSurfaceResult())
		require.NoError(t, err)
		return sheet
	}

	sheetA := build(t)
	tol := units.Millimeters(0.5)
	meshA1, err := sheetA.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	meshA2, err := sheetA.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	require.Equal(t, meshA1.Vertices(), meshA2.Vertices(), `the one-entry cache returns the same mesh on a repeat call`)
	require.Equal(t, meshA1.Triangles(), meshA2.Triangles())

	sheetB := build(t)
	meshB, err := sheetB.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	require.Equal(t, meshA1.Vertices(), meshB.Vertices())
	require.Equal(t, meshA1.Triangles(), meshB.Triangles())

	var stlA, stlB bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &stlA, sheetA, units.Millimeters(0.1)))
	require.NoError(t, export.STL(t.Context(), &stlB, sheetB, units.Millimeters(0.1)))
	require.Equal(t, stlA.String(), stlB.String())

	var objA, objB bytes.Buffer
	require.NoError(t, export.OBJ(t.Context(), &objA, sheetA, units.Millimeters(0.1)))
	require.NoError(t, export.OBJ(t.Context(), &objB, sheetB, units.Millimeters(0.1)))
	require.Equal(t, objA.String(), objB.String())

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = sheetA.Tessellate(ctx, tol)
	require.ErrorIs(t, err, context.Canceled)
}

// TestSurfaceRevolveSheetReachesNoBoolean is docs/surface-design.md Table
// X's boolean row for the revolve path specifically: a revolve sheet mesh
// publishes no occupied-volume proof, so Union refuses it outright.
func TestSurfaceRevolveSheetReachesNoBoolean(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s1, p1 := annularSketch(t)
	sheet, err := doc.Revolve(s1, p1, uAxis, quarterTurn, decad.WithSurfaceResult())
	require.NoError(t, err)

	s2, p2 := solidSketch(t)
	solid, err := doc.Revolve(s2, p2, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	before := doc.Bodies()
	_, err = decad.Union(t.Context(), sheet, solid)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
}

// offAxisSemicircleSketch builds semicircleSketch's own half-disc shifted
// entirely clear of the revolve axis (diameter at v=10, arc bulging to
// v=15): a two-walk loop — one straight, one circular — whose CIRCULAR
// walk's revolveMeridianMin is 1 (unlike semicircleSketch's own 2, since
// neither of ITS endpoints sits on the axis here), so a coarse tolerance can
// chord it to a single chord with no interior station.
func offAxisSemicircleSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 10)
	s.Fix(o)
	end := s.CreatePoint(10, 10)
	c := s.CreatePoint(5, 10)
	s.CreateLine(o, end)
	s.CreateArc(c, end, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// TestSurfaceRevolveSheetRefusesTooFewMeridianSamples re-adds, for the
// sheet path, the minimum-sample refusal the omitted cap triangulation used
// to carry (triangulate.go's own "a cap needs at least three boundary
// samples"): a wall loop chording to fewer than three meridian samples is
// still ErrDegenerate even though no cap is built to catch it.
func TestSurfaceRevolveSheetRefusesTooFewMeridianSamples(t *testing.T) {
	t.Parallel()
	s, p := offAxisSemicircleSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, quarterTurn, decad.WithSurfaceResult())
	require.NoError(t, err)

	_, err = sheet.Tessellate(t.Context(), units.Millimeters(20))
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
