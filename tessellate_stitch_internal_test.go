package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md §15's T60 and T70, over
// tessellate_stitch.go's own private proof terms: T60 needs areaSlack, which
// the public API does not expose, and T70 needs a hand-built triangle set no
// public seam can construct (checkStitchClosure already catches it at build
// time).

// stitchInternalOffAxisPlateSketch is stitch_test.go's offAxisPlateSketch
// (package decad_test), duplicated here because this file's package (decad)
// cannot import the exported test package that helper lives in — the same
// reach stitch_internal_test.go's own sphereRevolveFacesForInternalTest
// duplication already sets the precedent for. An off-axis, non-origin
// sketch plane is what keeps this test's own bound term from silently
// reading zero the way an axis-aligned or origin-centred fixture would.
func stitchInternalOffAxisPlateSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	axis, ok := r3.NewVec(1, 2, 3).Normalize()
	require.True(t, ok)
	ref := r3.NewVec(1, 0, 0)
	u, ok := ref.Sub(axis.Scale(ref.Dot(axis))).Normalize()
	require.True(t, ok)
	frame, err := r3.NewFrame(r3.NewVec(7, -3, 11), u, axis.Cross(u))
	require.NoError(t, err)

	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// TestStitchTessellateClassBoundIsIndependentOfPlacement is docs/surface-design.md's
// T60: T42's bounded-rim stitched solid (a Symmetric surface-extruded wall
// on an off-axis, non-origin sketch plane, Body.Patch-capped on both rims,
// then stitched at IDENTITY — the same fixture stitch_test.go's
// TestStitchClosesABoundedPatchedWallWithChargedVolumeBound builds). No
// placement is in play at all, so a positive mesh Bound here can only be
// read from the weld's own CLASS bound (stitchVertexTable.boundByClass),
// never from a placement delta — the leg
// TestStitchPlacedSolidTessellateBoundReadsVertexBound (T59,
// tessellate_stitch_test.go) cannot distinguish on its own.
func TestStitchTessellateClassBoundIsIndependentOfPlacement(t *testing.T) {
	t.Parallel()
	s, p := stitchInternalOffAxisPlateSketch(t)
	doc := New()
	wall, err := doc.Extrude(s, p, Symmetric{D: units.Inches(2.5)}, WithSurfaceResult())
	require.NoError(t, err)

	capped, err := wall.Patch(Edges(Free()).Exactly(8))
	require.NoError(t, err)

	solid, err := Stitch(capped)
	require.NoError(t, err)
	require.Equal(t, BodySolid, solid.Kind())

	sp, ok := solid.payload.(stitchPayload)
	require.True(t, ok)
	require.Zero(t, sp.delta, "no placement is in play: the identity transform's own fast path")

	wantBound := 0.0
	for _, v := range solid.Vertices() {
		wantBound = max(wantBound, v.Position().Bound.Base())
	}
	require.Positive(t, wantBound, "the fixture's own premise: the welded class bound is nonzero even at identity")

	mesh, err := solid.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, wantBound, mesh.Bound().Base())
	require.Positive(t, mesh.areaSlack)
}

// TestTessellateStitchRefusesARepeatedDirectedEdge is docs/surface-design.md's
// T70: a hand-built triangle set whose two triangles repeat the SAME
// directed edge (rather than tracing it in the opposite sense the way two
// triangles sharing an edge must) drives tessellateStitch directly.
// checkStitchClosure already catches this shape at Stitch's own build time
// (Table R row R7), so no public seam can ever construct a stitchPayload
// this broken; the fixture is fed straight to the restatement instead.
func TestTessellateStitchRefusesARepeatedDirectedEdge(t *testing.T) {
	t.Parallel()
	f := stitchTestSquareFace(r3.Vec{})
	verts := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)}
	sp := stitchPayload{
		verts:     verts,
		vertBound: []float64{0, 0, 0},
		tris:      [][3]int{{0, 1, 2}, {0, 1, 2}},
		triFaces:  []*Face{f, f},
	}
	b := &Body{kind: BodySolid, solid: true}

	mesh, err := tessellateStitch(t.Context(), b, sp)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Nil(t, mesh)
}
