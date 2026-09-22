package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// internalSheetBoxBody surface-extrudes an axis-aligned rectangle, the sheet
// counterpart of interference_internal_test.go's internalBoxBody.
func internalSheetBoxBody(t *testing.T, doc *Document, x0, y0, x1, y1, h float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along}, WithSurfaceResult())
	require.NoError(t, err)
	return body
}

// internalHoledSheetBody is internalHoledPlateBody's (tessellate_proof_internal_test.go)
// own sheet counterpart: the same 100×60 plate with a 10 mm-radius hole at
// (70, 30), surface-extruded 8 mm — the fixture whose circular hole gives a
// non-trivial bound and area slack to compare against the solid's.
func internalHoledSheetBody(t *testing.T, doc *Document) *Body {
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
	body, err := doc.Extrude(s, prof, Distance{D: units.Millimeters(8), Dir: Along}, WithSurfaceResult())
	require.NoError(t, err)
	return body
}

// TestSheetMeshPublishesNoOccupiedVolumeProof is docs/surface-design.md §10's
// own claim: a sheet mesh leaves symDiffOK false and volSymDiff zero, while
// the solid built from the same holed record — whose circular hole gives it
// a genuinely positive occupied-volume bound — publishes both.
func TestSheetMeshPublishesNoOccupiedVolumeProof(t *testing.T) {
	t.Parallel()
	solid := internalHoledPlateBody(t, New())
	solidMesh, err := tessellateContext(t.Context(), solid, units.Millimeters(0.5))
	require.NoError(t, err)
	require.True(t, solidMesh.symDiffOK)
	require.Positive(t, solidMesh.volSymDiff)

	sheet := internalHoledSheetBody(t, New())
	sheetMesh, err := tessellateContext(t.Context(), sheet, units.Millimeters(0.5))
	require.NoError(t, err)
	require.False(t, sheetMesh.symDiffOK)
	require.Zero(t, sheetMesh.volSymDiff)

	_, err = operandSymDiff(sheetMesh)
	require.ErrorIs(t, err, ErrUnsupported)
}

// TestSheetMeshFaceBoundCoversWallsOnly is docs/tessellation-design.md §2's
// per-face proof record, read for a sheet: one faceBound entry per wall and
// none for a cap role, while the solid built from the same record carries the
// wall entries plus the two caps'.
func TestSheetMeshFaceBoundCoversWallsOnly(t *testing.T) {
	t.Parallel()
	solid := internalHoledPlateBody(t, New())
	solidMesh, err := tessellateContext(t.Context(), solid, units.Millimeters(0.5))
	require.NoError(t, err)

	sheet := internalHoledSheetBody(t, New())
	sheetMesh, err := tessellateContext(t.Context(), sheet, units.Millimeters(0.5))
	require.NoError(t, err)

	require.Equal(t, len(sheet.Faces()), len(sheetMesh.faceBound), `one faceBound entry per wall, and a sheet carries no cap face`)
	for _, f := range sheet.Faces() {
		_, ok := sheetMesh.faceBound[f]
		require.True(t, ok, `every wall face has a published sourceBound`)
	}
	require.Equal(t, len(solid.Faces()), len(solidMesh.faceBound), `the solid's own faceBound additionally covers its two caps`)
}

// TestSheetAreaSlackIsBelowTheSolidsOnTheHoledFixture is the decision that a
// sheet's areaSlack drops the cap terms it no longer carries: on the holed
// fixture, whose circular hole gives every cap a genuinely positive
// chord-versus-arc deficit, the sheet's own areaSlack must be strictly below
// the solid's built from the same record.
func TestSheetAreaSlackIsBelowTheSolidsOnTheHoledFixture(t *testing.T) {
	t.Parallel()
	solid := internalHoledPlateBody(t, New())
	solidMesh, err := tessellateContext(t.Context(), solid, units.Millimeters(0.5))
	require.NoError(t, err)

	sheet := internalHoledSheetBody(t, New())
	sheetMesh, err := tessellateContext(t.Context(), sheet, units.Millimeters(0.5))
	require.NoError(t, err)

	require.Positive(t, solidMesh.areaSlack)
	// The gap is the two omitted caps' own circular-segment deficit, not mere
	// float noise from the smaller triangle count: require a gap wide enough
	// that only dropping the cap terms explains it.
	require.Greater(t, solidMesh.areaSlack-sheetMesh.areaSlack, 1.0)
}

// TestRequireSheetMeshRejectsBrokenFreeEdgeAttribution is the free-edge
// attribution's own falsifier. The public API can never construct a mesh
// whose free boundary is attributed to the wrong face — chordLoop always
// attributes a wall's own triangles to its own face — so this test hand-
// corrupts a real mesh's SourceFaces to prove requireSheetMesh actually
// refuses the mismatch it exists to catch, rather than passing vacuously.
func TestRequireSheetMeshRejectsBrokenFreeEdgeAttribution(t *testing.T) {
	t.Parallel()
	sheet := internalSheetBoxBody(t, New(), 0, 0, 10, 10, 5)
	mesh, err := tessellateContext(t.Context(), sheet, units.Millimeters(1))
	require.NoError(t, err)
	require.NoError(t, requireSheetMesh(t.Context(), sheet, mesh), `premise: the real mesh passes the audit as built`)

	faceA := mesh.source[0]
	var faceB *Face
	for _, f := range mesh.source {
		if f != faceA {
			faceB = f
			break
		}
	}
	require.NotNil(t, faceB, `premise: a box sheet's mesh names at least two distinct source faces`)

	broken := *mesh
	broken.source = append([]*Face(nil), mesh.source...)
	broken.source[0] = faceB
	err = requireSheetMesh(t.Context(), sheet, &broken)
	require.Error(t, err, `reassigning one triangle's source face unbalances the chain count on both faces`)
	require.ErrorIs(t, err, ErrDegenerate)
}

// TestRequireSheetMeshCatchesADuplicatedDirectedEdge is the manifold leg's
// own falsifier, for the same reason: the public API never builds a mesh with
// a repeated directed edge, so this hand-builds one triangle pair that shares
// a directed edge in the SAME direction — a fold rather than a fair
// back-to-back pairing — and confirms the audit refuses it.
func TestRequireSheetMeshCatchesADuplicatedDirectedEdge(t *testing.T) {
	t.Parallel()
	sheet := internalSheetBoxBody(t, New(), 0, 0, 10, 10, 5)
	mesh, err := tessellateContext(t.Context(), sheet, units.Millimeters(1))
	require.NoError(t, err)

	broken := *mesh
	broken.triangles = append([][3]int(nil), mesh.triangles...)
	// Force the first two triangles to share the directed edge (t0, t1) in
	// the same direction, which no valid mesh construction ever produces.
	broken.triangles[1] = [3]int{broken.triangles[0][0], broken.triangles[0][1], broken.triangles[0][2]}
	broken.source = append([]*Face(nil), mesh.source...)

	err = requireSheetMesh(t.Context(), sheet, &broken)
	require.Error(t, err)
	require.ErrorIs(t, err, ErrDegenerate)
}

// TestFreeChainCountsByFaceCountsConnectedComponentsNotEdges is defect 1's
// own falsifier: a face whose two free Edges share an interned Vertex — a
// revolve wall's two rims sharing a pole is the shipped case that reaches
// this, but no shipped body does yet, so the fixture is hand-built — is ONE
// connected chain, never two. Before the fix, freeChainCountsByFace counted
// one per free Edge object regardless of sharing, so this reports 2 and the
// test goes red.
func TestFreeChainCountsByFaceCountsConnectedComponentsNotEdges(t *testing.T) {
	t.Parallel()
	face := &Face{}
	vA, vB, vC := &Vertex{}, &Vertex{}, &Vertex{}
	e1 := &Edge{start: vA, end: vB, faces: []*Face{face}}
	e2 := &Edge{start: vB, end: vC, faces: []*Face{face}}
	face.loops = []*Loop{{coedges: []coedge{{edge: e1}, {edge: e2}}}}
	body := &Body{lumps: []*Lump{{shells: []*Shell{{faces: []*Face{face}}}}}}

	counts := freeChainCountsByFace(body)
	require.Equal(t, 1, counts[face], `two free edges sharing an interned vertex form one connected chain, not two`)
}

// TestRequireSheetVertexLinksAdmitsAnOpenPathFan is defect 2's own positive
// case: a fan of triangles around a center vertex, none of them closing the
// fan into a full disk, gives that center an OPEN-PATH link — exactly the
// shape every boundary vertex of a sound open sheet has, and exactly what
// requireVertexLinks' cycle-only rule would refuse.
func TestRequireSheetVertexLinksAdmitsAnOpenPathFan(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{}, {X: 1}, {X: 0, Y: 1}, {X: -1}, {X: 0, Y: -1}}
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 4}}
	m := &Mesh{vertices: verts, triangles: tris}
	require.NoError(t, requireSheetVertexLinks(t.Context(), m), `a fan's center vertex has a sound open-path link`)
}

// TestRequireSheetVertexLinksRefusesAForkedLink adds one triangle to the fan
// above that gives link vertex 2 a THIRD link edge — a fork, neither a cycle
// nor a path — which requireSheetVertexLinks must still refuse.
func TestRequireSheetVertexLinksRefusesAForkedLink(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{}, {X: 1}, {X: 0, Y: 1}, {X: -1}, {X: 0, Y: -1}, {Y: 2}}
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 2, 5}}
	m := &Mesh{vertices: verts, triangles: tris}
	err := requireSheetVertexLinks(t.Context(), m)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "forked link")
}

// TestRequireSheetVertexLinksRefusesTwoConesSharingAnApex reuses
// tessellate_revolve_internal_test.go's own pinched-apex fixture (two
// tetrahedral cones meeting at vertex 0): each cone alone has a sound
// closed-cycle link, but together apex 0's link is two disjoint cycles, one
// connected component short of the one this audit requires.
func TestRequireSheetVertexLinksRefusesTwoConesSharingAnApex(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{
		{X: 0, Y: 0, Z: 0},
		{X: 1, Y: 0, Z: 1}, {X: -1, Y: 1, Z: 1}, {X: -1, Y: -1, Z: 1},
		{X: 1, Y: 0, Z: -1}, {X: -1, Y: 1, Z: -1}, {X: -1, Y: -1, Z: -1},
	}
	tris := [][3]int{
		{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {1, 3, 2},
		{0, 5, 4}, {0, 6, 5}, {0, 4, 6}, {4, 5, 6},
	}
	m := &Mesh{vertices: verts, triangles: tris}
	err := requireSheetVertexLinks(t.Context(), m)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "more than one connected component")
}

// TestRequireSheetMeshAndVertexLinksAdmitAClosedSheet is
// docs/tessellation-design.md §1.2's closed-sheet sentence: with no free
// edge, every directed edge has its reverse, so the manifold-with-boundary
// audit is what a closed sheet runs and it passes for the same reason the
// closed-mesh audit would — and the vertex-link audit passes too, since
// every link is a sound single cycle.
func TestRequireSheetMeshAndVertexLinksAdmitAClosedSheet(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {1, 3, 2}}
	mesh := &Mesh{vertices: verts, triangles: tris, source: []*Face{{}, {}, {}, {}}}
	require.NoError(t, requireClosedMesh(mesh), `premise: a tetrahedron is a closed mesh`)

	faceA, faceB := &Face{}, &Face{}
	sharedEdge := &Edge{faces: []*Face{faceA, faceB}}
	faceA.loops = []*Loop{{coedges: []coedge{{edge: sharedEdge}}}}
	faceB.loops = []*Loop{{coedges: []coedge{{edge: sharedEdge}}}}
	body := &Body{lumps: []*Lump{{shells: []*Shell{{faces: []*Face{faceA, faceB}}}}}}
	require.Empty(t, freeChainCountsByFace(body), `premise: this body records no free edge`)

	require.NoError(t, requireSheetMesh(t.Context(), body, mesh), `no free edge on either side: the manifold-with-boundary audit passes for the same reason the closed-mesh audit would`)
	require.NoError(t, requireSheetVertexLinks(t.Context(), mesh))
}
