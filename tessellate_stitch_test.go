package decad_test

import (
	"bytes"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestStitchCurvedRevolveSheetTessellates(t *testing.T) {
	t.Parallel()
	_, sheet := annularRevolveSheet(t, 10, 5, 15)
	tol := units.Millimeters(0.1)
	source, err := sheet.Tessellate(t.Context(), tol)
	require.NoError(t, err)

	stitched, err := decad.Stitch(t.Context(), sheet)
	require.NoError(t, err)
	mesh, err := stitched.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	require.Equal(t, source.Vertices(), mesh.Vertices())
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
	require.Equal(t, source.Bound(), mesh.Bound())
	require.Greater(t, mesh.Bound().Base(), 0.0)
	require.True(t, mesh.BoundaryVerified())
	require.False(t, mesh.VolumeVerified())
	require.Zero(t, directedEdgeCensus(t, mesh))
	live := map[*decad.Face]struct{}{}
	for _, f := range stitched.Faces() {
		live[f] = struct{}{}
	}
	for _, f := range mesh.SourceFaces() {
		require.Contains(t, live, f)
	}
	volume := anchoredMeshVolume(mesh)
	require.Less(t, math.Abs(volume-2000*math.Pi), 0.03*2000*math.Pi)
	coarse, err := stitched.Tessellate(t.Context(), units.Millimeters(0.4))
	require.NoError(t, err)
	coarseVolume := anchoredMeshVolume(coarse)
	require.Less(t, math.Abs(volume-2000*math.Pi), math.Abs(coarseVolume-2000*math.Pi))
	t.Logf("fine volume %.12g mm3; coarse volume %.12g mm3; analytic %.12g mm3", volume, coarseVolume, 2000*math.Pi)
}

func TestStitchOpenCurvedRevolveSheetKeepsFreeRims(t *testing.T) {
	t.Parallel()
	s, p := annularSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, quarterTurn, decad.WithSurfaceResult())
	require.NoError(t, err)
	stitched, err := decad.Stitch(t.Context(), sheet)
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, stitched.Kind())
	mesh, err := stitched.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, 2, meshFreeChainCount(t, mesh))
	require.False(t, mesh.VolumeVerified())
	free, err := decad.Edges(decad.Free()).SelectEdges(stitched)
	require.NoError(t, err)
	require.Len(t, free, 8)
}

func TestStitchCurvedMeshInheritsSourcePlacementBound(t *testing.T) {
	t.Parallel()
	_, sheet := annularRevolveSheet(t, 10, 5, 15)
	motion, err := r3.Translation(r3.NewVec(1000, -500, 200))
	require.NoError(t, err)
	placedSource, err := sheet.Placed(t.Context(), motion)
	require.NoError(t, err)
	source, err := placedSource.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	stitched, err := decad.Stitch(t.Context(), placedSource)
	require.NoError(t, err)
	mesh, err := stitched.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, source.Vertices(), mesh.Vertices())
	require.Equal(t, source.Bound(), mesh.Bound())
	require.Positive(t, mesh.Bound().Base())
}

func TestStitchCurvedRevolveSurfaceKindsTessellate(t *testing.T) {
	t.Parallel()
	frustumVolume, _, _ := frustumShellAnalytics(10, 5, 15, 8, 12)
	torusVolume, _ := halfTorusAnalytics(10, 5)
	cases := []struct {
		name        string
		build       func(*testing.T) *decad.Body
		buildDirect func(*testing.T) *decad.Body
		want        float64
	}{
		{"cone", func(t *testing.T) *decad.Body {
			_, sheet := frustumSheet(t, 10, 5, 15, 8, 12)
			return sheet
		}, func(t *testing.T) *decad.Body {
			s, p := trapezoidFrustumSketch(t, 10, 5, 15, 8, 12)
			solid, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
			require.NoError(t, err)
			return solid
		}, frustumVolume},
		{"sphere", func(t *testing.T) *decad.Body {
			_, sheet := sphereRevolveSheet(t, 0, 10)
			return sheet
		}, func(t *testing.T) *decad.Body {
			s, p := semicircleSketchAt(t, 0, 10)
			solid, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
			require.NoError(t, err)
			return solid
		}, 4.0 / 3.0 * math.Pi * 125},
		{"torus", func(t *testing.T) *decad.Body {
			s, p := offAxisSemicircleSketch(t)
			doc := decad.New()
			sheet, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
			require.NoError(t, err)
			return sheet
		}, func(t *testing.T) *decad.Body {
			s, p := offAxisSemicircleSketch(t)
			solid, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
			require.NoError(t, err)
			return solid
		}, torusVolume},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			sheet := tc.build(t)
			tol := units.Millimeters(0.1)
			source, err := sheet.Tessellate(t.Context(), tol)
			require.NoError(t, err)
			stitched, err := decad.Stitch(t.Context(), sheet)
			require.NoError(t, err)
			mesh, err := stitched.Tessellate(t.Context(), tol)
			require.NoError(t, err)
			direct, err := tc.buildDirect(t).Tessellate(t.Context(), tol)
			require.NoError(t, err)
			require.True(t, direct.VolumeVerified())
			require.Equal(t, source.Vertices(), mesh.Vertices())
			require.Equal(t, direct.Vertices(), mesh.Vertices())
			require.Equal(t, source.Bound(), mesh.Bound())
			require.Zero(t, directedEdgeCensus(t, mesh))
			require.Positive(t, meshTriangleArea(mesh))
			require.InDelta(t, anchoredMeshVolume(direct), anchoredMeshVolume(mesh), 1e-9)
			sourceKinds := map[decad.SurfaceKind]int{}
			for _, f := range source.SourceFaces() {
				sourceKinds[f.Surface().Kind()]++
			}
			stitchedKinds := map[decad.SurfaceKind]int{}
			for _, f := range mesh.SourceFaces() {
				stitchedKinds[f.Surface().Kind()]++
			}
			require.Equal(t, sourceKinds, stitchedKinds)
			require.InDelta(t, anchoredMeshVolume(source), anchoredMeshVolume(mesh), 1e-9)
			got := anchoredMeshVolume(mesh)
			require.Less(t, math.Abs(got-tc.want), 0.05*tc.want)
			t.Logf("mesh volume %.12g mm3; analytic %.12g mm3; bound %.12g mm", got, tc.want, mesh.Bound().Base())
		})
	}
}

func TestStitchCurvedRevolveMeshVerificationAndExport(t *testing.T) {
	t.Parallel()
	_, sheet := annularRevolveSheet(t, 10, 5, 15)
	stitched, err := decad.Stitch(t.Context(), sheet)
	require.NoError(t, err)
	tol := units.Millimeters(0.1)
	for _, tc := range []struct {
		level decad.Verification
		bound bool
	}{
		{decad.VerifyNone, false},
		{decad.VerifyBoundary, true},
		{decad.VerifyAll, true},
	} {
		mesh, err := stitched.Tessellate(t.Context(), tol, decad.WithVerification(tc.level))
		require.NoError(t, err)
		require.Equal(t, tc.bound, mesh.BoundaryVerified())
		require.False(t, mesh.VolumeVerified())
		require.Greater(t, mesh.Bound().Base(), 0.0)
	}
	var stl, obj bytes.Buffer
	require.NoError(t, stitched.STL(&stl, decad.WithChordTolerance(tol)))
	require.NoError(t, stitched.OBJ(&obj, decad.WithChordTolerance(tol)))
	require.NotEmpty(t, stl.Bytes())
	require.NotEmpty(t, obj.Bytes())
}

func TestStitchCurvedMeshRefusesUnsharedChording(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateCircle(s.CreatePoint(0, 0), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	sheet, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	stitched, err := decad.Stitch(t.Context(), sheet)
	require.NoError(t, err)
	_, err = stitched.Tessellate(t.Context(), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "Cylinder")

	_, source := annularRevolveSheet(t, 10, 5, 15)
	curved, err := decad.Stitch(t.Context(), source)
	require.NoError(t, err)
	motion, err := r3.Translation(r3.NewVec(100, 0, 0))
	require.NoError(t, err)
	placed, err := curved.Placed(t.Context(), motion)
	require.NoError(t, err)
	_, err = placed.Tessellate(t.Context(), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "Cylinder")
}

// This file is docs/surface-design.md §14 Table D row 5's own test list
// (§15 T58-T61, T63, T64): an all-planar stitched body's mesh, CLOSED or
// OPEN, is an exact restatement of the triangle set Stitch's own build
// already assembled and audited, over T3's stitched-box fixture (CLOSED) and
// T4's displaced-patch fixture (OPEN).

// stitchedBox stitches T3's box (stitchBoxSheets) into one BodySolid.
func stitchedBox(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	walls, bottom, top := stitchBoxSheets(t, doc, 10)
	box, err := decad.Stitch(t.Context(), walls, bottom, top)
	require.NoError(t, err)
	return box
}

// boxCorners is stitchedBox's own 8 corners, the 100x60x10 mm box
// stitchBoxSheets builds.
func boxCorners() map[r3.Vec]struct{} {
	corners := map[r3.Vec]struct{}{}
	for _, x := range []float64{0, 100} {
		for _, y := range []float64{0, 60} {
			for _, z := range []float64{0, 10} {
				corners[r3.NewVec(x, y, z)] = struct{}{}
			}
		}
	}
	return corners
}

// TestStitchSolidTessellateRecordsThePostSignFixTriangleSet is not one of
// §15's own numbered rows, but closes a gap none of them reach: T58 and T59
// both stitch stitchBoxSheets' walls first, and that operand order happens
// to land the arbitrary root face outward from the start, so
// evalStitchContext's global sign-fix (stitch.go, "if acc.vol6.Sign() < 0")
// never triggers for either. Stitching the SAME three sheets patch-first
// picks bottom's own face as the root instead, which DOES trigger the fix,
// and is what would catch M1's own payload recording the triangle set built
// BEFORE that fix rather than the final, re-triangulated one after it: doing
// so flips this test's own mesh volume to -60000 while leaving every
// permanent test in this package green, since none of them happens to pick
// an inward-starting root.
func TestStitchSolidTessellateRecordsThePostSignFixTriangleSet(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	walls, bottom, top := stitchBoxSheets(t, doc, 10)
	box, err := decad.Stitch(t.Context(), bottom, top, walls)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, box.Kind())

	mesh, err := box.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.InDelta(t, 60000.0, anchoredMeshVolume(mesh), 1e-9,
		"the recorded triangle set must be the one AFTER the global sign fix, never the stale pre-fix one")
}

// TestStitchSolidTessellatesItsOwnTriangleSet is docs/surface-design.md's
// T58: T3's stitched box, the flagship all-planar closed case, restates
// exactly the triangle set Stitch's own build assembled — no chording, so
// every vertex is bit-equal to one of the box's own 8 corners and the
// tetrahedron sum over the mesh equals the box's own 60000 mm^3 to the bit.
func TestStitchSolidTessellatesItsOwnTriangleSet(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := stitchedBox(t, doc)

	mesh, err := box.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.NotNil(t, mesh)

	require.Len(t, mesh.Triangles(), 12, "6 planar faces, 2 triangles apiece")
	require.Len(t, mesh.Vertices(), 8)
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))

	corners := boxCorners()
	for _, v := range mesh.Vertices() {
		require.Containsf(t, corners, v, "mesh vertex %v is not one of the box's own 8 corners", v)
	}

	distinct := map[*decad.Face]int{}
	for _, f := range mesh.SourceFaces() {
		require.NotNil(t, f)
		distinct[f]++
	}
	require.Len(t, distinct, 6)
	for f, n := range distinct {
		require.Equalf(t, 2, n, "face %v carries %d triangles, want 2", f, n)
	}

	require.Zero(t, mesh.Bound().Base(), "every held vertex is exact, so the restatement adds no displacement")

	requireWatertight(t, mesh)
	require.InDelta(t, 60000.0, meshVolume(mesh), 1e-9, "the exact tetrahedron sum over the mesh equals the box's own volume to the bit")
}

// TestStitchPlacedSolidTessellateBoundReadsVertexBound is docs/surface-design.md's
// T59: T58's box, Placed by a translation far from the origin and then a
// rotation, is the placement-delta route to a nonzero per-face bound — the
// leg TestStitchTessellateClassBoundIsIndependentOfPlacement (T60,
// tessellate_stitch_internal_test.go) shows is NOT the only route.
func TestStitchPlacedSolidTessellateBoundReadsVertexBound(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := stitchedBox(t, doc)

	shift, err := r3.Translation(r3.NewVec(1e6, -7e5, 3e5))
	require.NoError(t, err)
	rot, err := r3.RotationAround(r3.NewVec(1e6, -7e5, 3e5), r3.NewVec(0, 0, 1), units.Degrees(41))
	require.NoError(t, err)
	motion, err := shift.Then(rot)
	require.NoError(t, err)

	placed, err := box.Placed(t.Context(), motion)
	require.NoError(t, err)

	mesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Len(t, mesh.Triangles(), 12)

	require.Positive(t, mesh.Bound().Base())

	wantBound := 0.0
	for _, v := range placed.Vertices() {
		wantBound = max(wantBound, v.Position().Bound.Base())
	}
	require.Positive(t, wantBound)
	require.Equal(t, wantBound, mesh.Bound().Base(), "the mesh's own bound is the body's own largest Vertex.Bound()")

	vol, err := placed.Volume()
	require.NoError(t, err)
	value, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	// meshVolume's own raw triple product would lose precision at these
	// far-from-origin coordinates for reasons that have nothing to do with
	// the proof under test, so the volume is integrated anchored at the
	// mesh's own first vertex instead, exactly as the evaluator's own
	// tetrahedron sum anchors at verts[0].
	require.InDelta(t, value, anchoredMeshVolume(mesh), vol.Bound.Base()+1e-9,
		"the mesh's integrated volume must lie within the body's own published volume bound")
}

// TestStitchDisplacedPatchSheetTessellatesItsOwnTriangleSet is
// docs/surface-design.md's T61: T4's displaced-patch stitched sheet — open,
// every face planar — restates its own triangle set exactly as a closed
// stitched solid's mesh does (TestStitchSolidTessellatesItsOwnTriangleSet,
// above), running docs/tessellation-design.md §1.2's manifold-with-boundary
// audit (requireSheetMesh, requireSheetVertexLinks) in the closed-mesh
// audit's place. The free-boundary attribution needs no role at all: the
// mesh side and the body side agree by the identical live face pointer
// stitchPayload records and Body.Edges() reads back
// (docs/tessellation-design.md §1.2), which meshFreeChainsByFace and
// bodyFreeChainsByFace below check independently of tessellate_sheet.go's
// own audit, over the SAME connected-component-by-chain reading that audit
// uses.
func TestStitchDisplacedPatchSheetTessellatesItsOwnTriangleSet(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	walls, bottom, top := stitchBoxSheets(t, doc, 10+1e-9)
	sheet, err := decad.Stitch(t.Context(), walls, bottom, top)
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, sheet.Kind())

	mesh, err := sheet.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.NotNil(t, mesh)

	meshFree := meshFreeChainsByFace(mesh)
	bodyFree := bodyFreeChainsByFace(sheet)
	require.Len(t, bodyFree, 5, "the 4 wall faces and the displaced patch")
	require.Equal(t, bodyFree, meshFree,
		"the mesh's free boundary must attribute to exactly the faces, with the same chain count per face, the body reports free Edges on")
	for face, n := range meshFree {
		require.Equalf(t, 1, n, "face %v carries %d free boundary chain(s), want 1", face, n)
	}

	_, err = sheet.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)

	block := boxBody(t, doc, 200, 0, 300, 60, 10)
	_, err = decad.Union(t.Context(), sheet, block)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// meshFreeChainsByFace groups a mesh's own free directed edges — one whose
// reverse is absent — by the face SourceFaces names for the triangle it
// belongs to, then counts the connected components (chains) each face's own
// free edges form over the mesh's vertex indices. It is an independent
// reading of the same shape tessellate_sheet.go's freeSheetEdgesByFace and
// countChains compute internally, built from Mesh's public accessors alone.
func meshFreeChainsByFace(mesh *decad.Mesh) map[*decad.Face]int {
	directed := map[[2]int]int{}
	for _, tri := range mesh.Triangles() {
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	byFace := map[*decad.Face]map[int]int{}
	for i, tri := range mesh.Triangles() {
		face := mesh.SourceFaces()[i]
		for k := range 3 {
			e := [2]int{tri[k], tri[(k+1)%3]}
			if directed[[2]int{e[1], e[0]}] != 0 {
				continue // interior: shared with the facet across it
			}
			parent, ok := byFace[face]
			if !ok {
				parent = map[int]int{}
				byFace[face] = parent
			}
			unionVertices(parent, e[0], e[1])
		}
	}
	counts := make(map[*decad.Face]int, len(byFace))
	for face, parent := range byFace {
		counts[face] = countRoots(parent)
	}
	return counts
}

// bodyFreeChainsByFace groups a body's own recorded free Edges by their
// single adjacent face, then counts the connected components (chains) each
// face's own free Edges form over their own Vertex pointers — an
// independent reading of tessellate_sheet.go's freeChainCountsByFace, built
// from Body's public accessors alone.
func bodyFreeChainsByFace(body *decad.Body) map[*decad.Face]int {
	byFace := map[*decad.Face]map[*decad.Vertex]*decad.Vertex{}
	for _, e := range body.Edges() {
		if !e.IsFree() {
			continue
		}
		faces := e.Faces()
		require1Face(faces)
		face := faces[0]
		parent, ok := byFace[face]
		if !ok {
			parent = map[*decad.Vertex]*decad.Vertex{}
			byFace[face] = parent
		}
		unionVertexPointers(parent, e.Start(), e.End())
	}
	counts := make(map[*decad.Face]int, len(byFace))
	for face, parent := range byFace {
		roots := map[*decad.Vertex]struct{}{}
		for v := range parent {
			roots[findVertexPointer(parent, v)] = struct{}{}
		}
		counts[face] = len(roots)
	}
	return counts
}

func require1Face(faces []*decad.Face) {
	if len(faces) != 1 {
		panic("a free Edge must bound exactly one face")
	}
}

func unionVertices(parent map[int]int, a, b int) {
	if _, ok := parent[a]; !ok {
		parent[a] = a
	}
	if _, ok := parent[b]; !ok {
		parent[b] = b
	}
	ra, rb := findVertex(parent, a), findVertex(parent, b)
	if ra != rb {
		parent[ra] = rb
	}
}

func findVertex(parent map[int]int, v int) int {
	for parent[v] != v {
		parent[v] = parent[parent[v]]
		v = parent[v]
	}
	return v
}

func countRoots(parent map[int]int) int {
	roots := map[int]struct{}{}
	for v := range parent {
		roots[findVertex(parent, v)] = struct{}{}
	}
	return len(roots)
}

func unionVertexPointers(parent map[*decad.Vertex]*decad.Vertex, a, b *decad.Vertex) {
	if _, ok := parent[a]; !ok {
		parent[a] = a
	}
	if _, ok := parent[b]; !ok {
		parent[b] = b
	}
	ra, rb := findVertexPointer(parent, a), findVertexPointer(parent, b)
	if ra != rb {
		parent[ra] = rb
	}
}

func findVertexPointer(parent map[*decad.Vertex]*decad.Vertex, v *decad.Vertex) *decad.Vertex {
	for parent[v] != v {
		parent[v] = parent[parent[v]]
		v = parent[v]
	}
	return v
}

// TestStitchSolidTessellateIsDeterministic is docs/surface-design.md's T63:
// the same body tessellated at two different tolerances (the restatement
// takes no chord tolerance) publishes byte-for-byte identical output, and
// the returned slices never alias the held mesh.
func TestStitchSolidTessellateIsDeterministic(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := stitchedBox(t, doc)

	m1, err := box.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	m2, err := box.Tessellate(t.Context(), units.Millimeters(5))
	require.NoError(t, err)

	require.Equal(t, m1.Vertices(), m2.Vertices())
	require.Equal(t, m1.Triangles(), m2.Triangles())
	require.Equal(t, m1.SourceFaces(), m2.SourceFaces())

	var stl1, stl2 stringWriter
	require.NoError(t, box.STL(&stl1))
	require.NoError(t, box.STL(&stl2))
	require.Equal(t, stl1.s, stl2.s)

	var obj1, obj2 stringWriter
	require.NoError(t, box.OBJ(&obj1))
	require.NoError(t, box.OBJ(&obj2))
	require.Equal(t, obj1.s, obj2.s)

	verts := m1.Vertices()
	verts[0] = r3.NewVec(999, 999, 999)
	tris := m1.Triangles()
	tris[0] = [3]int{9, 9, 9}
	m3, err := box.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, m2.Vertices(), m3.Vertices(), "mutating a returned slice must not reach the held mesh")
	require.Equal(t, m2.Triangles(), m3.Triangles())
}

// TestStitchSolidUnionComposesTheCorrectVolume is docs/surface-design.md's
// T71: a CLOSED, all-planar, zero-vertex-bound stitched solid now publishes
// an occupied-volume proof of exactly zero, so requireVolumeProvingPayload's
// stitchPayload arm no longer refuses it and Union composes the two disjoint
// operands' exact volumes — the mesh-boolean analog of TestUnionDisjointCubes
// (boolean_test.go), over a stitched operand instead of a plain one. This
// replaces TestStitchSolidUnionRefusesOnTheVolumeProof, whose whole subject
// (a stitched solid ever entering a boolean) this row retires; T64's own row
// keeps the refusal only for a stitched solid this proof does not cover
// (T73, T74).
func TestStitchSolidUnionComposesTheCorrectVolume(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := stitchedBox(t, doc)
	block := boxBody(t, doc, 200, 0, 300, 60, 10)

	got, err := decad.Union(t.Context(), box, block)
	require.NoError(t, err)

	// Disjoint: the union is exactly the sum, two lumps, nothing chorded,
	// nothing rounded — Exact with a zero bound, the identical reading
	// TestUnionDisjointCubes gets for two plain boxes.
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	volMM3, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Equal(t, 120000.0, volMM3)
	boundMM3, err := vol.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Zero(t, boundMM3)
	require.Len(t, got.Lumps(), 2)
}

// TestStitchPlacedSolidUnionStillRefusesOnTheVolumeProof is
// docs/surface-design.md's T73: T58's own zero-bound box, Placed under a
// non-identity rigid motion, still refuses a boolean — the placement's own
// rigidRoundAllow widens every vertex bound (stitch.go), so
// stitchZeroVertexBound no longer holds and requireVolumeProvingPayload
// keeps refusing this operand exactly as it did before this increment. This
// reaches the gate by the PLACEMENT route; T74 reaches it by the
// CERTIFICATE-WELD route, and neither alone would prove the gate reads the
// vertex bound itself rather than one particular cause of it (T68/T69's
// identical pairing, for the clearance-kernel gate instead of this one).
func TestStitchPlacedSolidUnionStillRefusesOnTheVolumeProof(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := stitchedBox(t, doc)
	motion, err := r3.Translation(r3.NewVec(500, 500, 500))
	require.NoError(t, err)
	placed, err := box.Placed(t.Context(), motion)
	require.NoError(t, err)
	block := boxBody(t, doc, 700, 500, 800, 560, 10)

	before := len(doc.Bodies())
	_, err = decad.Union(t.Context(), placed, block)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "no proof of the volume")
	require.NotContains(t, err.Error(), "does not support payload")
	require.Len(t, doc.Bodies(), before, "a refused boolean leaves the document unchanged")
}

// TestStitchCertificateWeldedSolidUnionStillRefusesOnTheVolumeProof is
// docs/surface-design.md's T74: T42's own certificate-welded stitched
// solid — every vertex bound nonzero even at identity, no placement in
// play — still refuses a boolean by the CERTIFICATE-WELD route rather than
// the placement route T73 exercises.
func TestStitchCertificateWeldedSolidUnionStillRefusesOnTheVolumeProof(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := offAxisPlateSketch(t)
	wall, err := doc.Extrude(s, p, decad.Symmetric{D: units.Inches(2.5)}, decad.WithSurfaceResult())
	require.NoError(t, err)
	capped, err := wall.Patch(t.Context(), decad.Edges(decad.Free()).Exactly(8))
	require.NoError(t, err)
	solid, err := decad.Stitch(t.Context(), capped)
	require.NoError(t, err)
	for _, v := range solid.Vertices() {
		require.Greater(t, v.Position().Bound.Mag(), 0.0,
			"every vertex of the CURVE-welded solid carries a nonzero bound, at identity")
	}

	bb, err := solid.Bounds()
	require.NoError(t, err)
	far := bb.Max.Add(r3.NewVec(50, 50, 50))
	block := boxBody(t, doc, far.X, far.Y, far.X+10, far.Y+10, 10)

	before := len(doc.Bodies())
	_, err = decad.Union(t.Context(), solid, block)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "no proof of the volume")
	require.NotContains(t, err.Error(), "does not support payload")
	require.Len(t, doc.Bodies(), before, "a refused boolean leaves the document unchanged")
}

// anchoredMeshVolume is meshVolume's own tetrahedron sum, anchored at the
// mesh's first vertex rather than at the coordinate origin, exactly as the
// evaluator's own newLoftMassAccumulator anchors at verts[0]: a triple
// product of far-from-origin coordinates loses precision for reasons that
// have nothing to do with the proof under test.
func anchoredMeshVolume(mesh *decad.Mesh) float64 {
	verts := mesh.Vertices()
	if len(verts) == 0 {
		return 0
	}
	anchor := verts[0]
	total := 0.0
	for _, tri := range mesh.Triangles() {
		a := verts[tri[0]].Sub(anchor)
		b := verts[tri[1]].Sub(anchor)
		c := verts[tri[2]].Sub(anchor)
		total += a.Dot(b.Cross(c)) / 6
	}
	return total
}

// stringWriter is an io.Writer collecting everything written to it as a
// string, standing in for a real sink in a determinism check.
type stringWriter struct{ s string }

func (w *stringWriter) Write(p []byte) (int, error) {
	w.s += string(p)
	return len(p), nil
}
