package decad_test

import (
	"bytes"
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md §10's T10, run against the PRISM sheet
// path: WithSurfaceResult()'s own manifold-with-boundary mesh and export.
// Every fixture reuses plateSketch/rectWithHoleSketch (extrude_test.go,
// surface_test.go). NEVER call decadtest.IsManifold on a sheet mesh here — it
// requires every edge to bound exactly two faces, which a sheet's own free
// rims never do.

// meshTriangleArea sums a mesh's triangle areas in the mesh's own coordinates.
func meshTriangleArea(mesh *decad.Mesh) float64 {
	verts := mesh.Vertices()
	total := 0.0
	for _, tri := range mesh.Triangles() {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		total += b.Sub(a).Cross(c.Sub(a)).Len() / 2
	}
	return total
}

// directedEdgeCensus asserts a mesh's directed edges never occur twice and
// returns the count of FREE directed edges (whose reverse is absent).
func directedEdgeCensus(t *testing.T, mesh *decad.Mesh) int {
	t.Helper()
	directed := map[[2]int]int{}
	for _, tri := range mesh.Triangles() {
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	free := 0
	for e, n := range directed {
		require.LessOrEqual(t, n, 1, `directed edge %v must never occur twice`, e)
		if n == 1 && directed[[2]int{e[1], e[0]}] == 0 {
			free++
		}
	}
	return free
}

// TestSurfaceExtrudeSheetTessellates is docs/surface-design.md's T10 for the
// prism sheet path: a 100x60 mm rectangle surface-extruded 10 mm produces a
// mesh that is the solid's own minus its two caps.
func TestSurfaceExtrudeSheetTessellates(t *testing.T) {
	t.Parallel()
	solidDoc := decad.New()
	s1, p1 := plateSketch(t)
	solid, err := solidDoc.Extrude(s1, p1, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	solidMesh, err := solid.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Len(t, solidMesh.Triangles(), 12)

	sheetDoc := decad.New()
	s2, p2 := plateSketch(t)
	sheet, err := sheetDoc.Extrude(s2, p2, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	sheetMesh, err := sheet.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)

	require.Len(t, sheetMesh.Triangles(), 8, `8 wall triangles against the solid's 12 — the two omitted caps' 4`)
	require.Equal(t, solidMesh.Vertices(), sheetMesh.Vertices(), `the sheet reuses the exact same chording, so the vertex table is identical`)
	require.Len(t, sheetMesh.SourceFaces(), len(sheetMesh.Triangles()))
	require.InDelta(t, 3200.0, meshTriangleArea(sheetMesh), 1e-9, `4 walls of 100x10/60x10/100x10/60x10 mm sum to 3200 mm^2`)

	// docs/tessellation-design.md §1.2: the shell's positive-side orientation
	// is never audited, only asserted here — it holds by construction, so the
	// sheet's own wall triangles are IDENTICAL, index for index, to the
	// solid's first 8 (its walls, emitted before its two caps).
	require.Equal(t, solidMesh.Triangles()[:8], sheetMesh.Triangles(), `a sheet's walls carry the same winding the solid's own walls would`)

	// No cap role among the source faces: the sheet's own live face set has
	// none, and every mesh facet's source is one of them.
	liveFaces := map[*decad.Face]struct{}{}
	for _, f := range sheet.Faces() {
		liveFaces[f] = struct{}{}
	}
	require.Len(t, liveFaces, 4)
	seen := map[*decad.Face]struct{}{}
	for _, f := range sheetMesh.SourceFaces() {
		_, ok := liveFaces[f]
		require.True(t, ok, `every facet's source face is one the sheet actually carries`)
		seen[f] = struct{}{}
	}
	require.Len(t, seen, 4, `every one of the sheet's 4 wall faces is covered`)

	// Directed-edge census: no edge twice, and exactly 8 free directed edges
	// — 2 per wall (its own top and bottom rim), which is what an omitted
	// cap leaves behind.
	free := directedEdgeCensus(t, sheetMesh)
	require.Equal(t, 8, free)

	// Every mesh free edge is attributed to a wall face carrying exactly two
	// free body edges (docs/tessellation-design.md §1.2's free-edge
	// attribution).
	src := sheetMesh.SourceFaces()
	tris := sheetMesh.Triangles()
	directed := map[[2]int]int{}
	for _, tri := range tris {
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	for i, tri := range tris {
		for k := range 3 {
			e := [2]int{tri[k], tri[(k+1)%3]}
			rev := [2]int{e[1], e[0]}
			if directed[rev] != 0 {
				continue
			}
			face := src[i]
			var faceFree int
			for _, fe := range face.Edges() {
				if fe.IsFree() {
					faceFree++
				}
			}
			require.Equal(t, 2, faceFree, `every wall face carries exactly two free body edges`)
		}
	}

	// STL: 8 facets, and a second write is byte-identical.
	var buf1, buf2 bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &buf1, sheet, units.Millimeters(0.1)))
	require.NoError(t, export.STL(t.Context(), &buf2, sheet, units.Millimeters(0.1)))
	require.Equal(t, buf1.String(), buf2.String())
	require.Equal(t, 8, countSTLFacets(buf1.String()))
}

// TestSurfaceExtrudeSheetBoundMatchesSolid is the bound leg of T10 over the
// holed fixture, where the bound is not trivially zero.
func TestSurfaceExtrudeSheetBoundMatchesSolid(t *testing.T) {
	t.Parallel()
	tol := units.Millimeters(0.5)

	solidDoc := decad.New()
	s1, p1 := rectWithHoleSketch(t)
	solid, err := solidDoc.Extrude(s1, p1, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	solidMesh, err := solid.Tessellate(t.Context(), tol)
	require.NoError(t, err)

	sheetDoc := decad.New()
	s2, p2 := rectWithHoleSketch(t)
	sheet, err := sheetDoc.Extrude(s2, p2, decad.Distance{D: units.Millimeters(8), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	sheetMesh, err := sheet.Tessellate(t.Context(), tol)
	require.NoError(t, err)

	require.True(t, sheetMesh.Bound().Equal(solidMesh.Bound(), 1e-12))
	require.Positive(t, sheetMesh.Bound().Mag())
	require.LessOrEqual(t, sheetMesh.Bound().Mag(), tol.Mag())

	finer := units.Millimeters(0.05)
	finerMesh, err := sheet.Tessellate(t.Context(), finer)
	require.NoError(t, err)
	require.Less(t, finerMesh.Bound().Mag(), sheetMesh.Bound().Mag())
}

// TestSurfaceExtrudeSheetDeterminism is T10's determinism leg: the one-entry
// cache, two fresh documents producing equal meshes and byte-identical
// STL/OBJ, and a cancelled context returning ctx.Err() unchanged.
func TestSurfaceExtrudeSheetDeterminism(t *testing.T) {
	t.Parallel()
	build := func(t *testing.T) *decad.Body {
		t.Helper()
		s, p := plateSketch(t)
		doc := decad.New()
		sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
		require.NoError(t, err)
		return sheet
	}

	sheetA := build(t)
	tol := units.Millimeters(0.1)
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

// TestSurfaceExtrudeHoledSheetMultiLumpTessellates is new since the plan was
// written: a holed profile's surface extrude splits into two disconnected
// lumps (docs/surface-design.md §2.2), and both the outer wall tube and the
// hole wall tube must tessellate and pass the manifold-with-boundary audit.
func TestSurfaceExtrudeHoledSheetMultiLumpTessellates(t *testing.T) {
	t.Parallel()
	s, p := rectWithHoleSketch(t)
	doc := decad.New()
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(8), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	require.Len(t, sheet.Lumps(), 2, `an outer wall tube and a hole wall tube, disconnected once their shared caps are omitted`)

	mesh, err := sheet.Tessellate(t.Context(), units.Millimeters(0.5))
	require.NoError(t, err)
	require.NotEmpty(t, mesh.Triangles())
	free := directedEdgeCensus(t, mesh)
	require.Positive(t, free)
}

// countSTLFacets counts "endfacet" occurrences in an ASCII STL text.
func countSTLFacets(stl string) int {
	n := 0
	for i := 0; i+len("endfacet") <= len(stl); i++ {
		if stl[i:i+len("endfacet")] == "endfacet" {
			n++
		}
	}
	return n
}
