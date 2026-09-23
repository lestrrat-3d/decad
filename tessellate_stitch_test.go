package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md §14 Table D row 5's own test list
// (§15 T58-T60, T63, T64): a CLOSED, all-planar stitched body's mesh is an
// exact restatement of the triangle set Stitch's own build already
// assembled and audited, over T3's stitched-box fixture.

// stitchedBox stitches T3's box (stitchBoxSheets) into one BodySolid.
func stitchedBox(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	walls, bottom, top := stitchBoxSheets(t, doc, 10)
	box, err := decad.Stitch(walls, bottom, top)
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
	box, err := decad.Stitch(bottom, top, walls)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, box.Kind())

	mesh, err := box.Tessellate(units.Millimeters(0.1))
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

	mesh, err := box.Tessellate(units.Millimeters(0.1))
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

	placed, err := box.Placed(motion)
	require.NoError(t, err)

	mesh, err := placed.Tessellate(units.Millimeters(0.1))
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

// TestStitchSolidTessellateIsDeterministic is docs/surface-design.md's T63:
// the same body tessellated at two different tolerances (the restatement
// takes no chord tolerance) publishes byte-for-byte identical output, and
// the returned slices never alias the held mesh.
func TestStitchSolidTessellateIsDeterministic(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := stitchedBox(t, doc)

	m1, err := box.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err)
	m2, err := box.Tessellate(units.Millimeters(5))
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
	m3, err := box.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, m2.Vertices(), m3.Vertices(), "mutating a returned slice must not reach the held mesh")
	require.Equal(t, m2.Triangles(), m3.Triangles())
}

// TestStitchSolidUnionRefusesOnTheVolumeProof is docs/surface-design.md's
// T64: a stitched solid's mesh publishes no occupied-volume proof in this
// increment, so a boolean refuses it at requireVolumeProvingPayload's own
// stitchPayload arm — the volume-proof wording, never the payload-class
// wording tessellation's own dispatch would otherwise report — before
// either operand is touched.
func TestStitchSolidUnionRefusesOnTheVolumeProof(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := stitchedBox(t, doc)
	block := boxBody(t, doc, 200, 0, 300, 60, 10)

	before := len(doc.Bodies())
	_, err := decad.Union(box, block)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "no proof of the volume")
	require.NotContains(t, err.Error(), "does not support payload")

	require.Len(t, doc.Bodies(), before, "a refused boolean leaves the document unchanged")
	require.True(t, box.IsSolid())
	require.True(t, block.IsSolid())
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
