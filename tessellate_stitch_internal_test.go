package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md §15's T60, T70 and T87, over
// tessellate_stitch.go's own private proof terms: T60 needs areaSlack, which
// the public API does not expose, and T70 needs a hand-built triangle set no
// public seam can construct (checkStitchClosure already catches it at build
// time).

func TestStitchCurvedMeshNamesUnsupportedSurface(t *testing.T) {
	t.Parallel()
	plane := &Face{surface: Plane{}}
	freeform := &Face{surface: NURBSSurface{}}
	sp := stitchPayload{
		faces:     []*Face{plane, freeform},
		liveFaces: []*Face{{surface: Plane{}}, {surface: NURBSSurface{}}},
		plan:      &stitchWeldPlan{table: newStitchVertexTable()},
	}
	_, err := tessellateStitchCurved(t.Context(), &Body{kind: BodySheet}, sp, 0.1, VerifyAll)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "NURBSSurface")
}

// internalAnnularRevolveSheet is T31's own annular full-turn revolve sheet
// (annularRevolveSheet, stitch_flux_test.go: 2 Cylinder walls, 2 Plane
// annuli), duplicated here because this file's package (decad) cannot import
// the exported test package that helper lives in — the same reach
// stitchInternalOffAxisPlateSketch's own duplication above sets the
// precedent for.
func internalAnnularRevolveSheet(t *testing.T) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 5, 10, 15)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	sheet, err := New().Revolve(s, s.Profiles()[0], revolveAxisU, FullRevolution{}, WithSurfaceResult())
	require.NoError(t, err)
	return sheet
}

// TestStitchCurvedMeshInheritsEverySourceFaceBound checks T88's per-face
// relation over T31's own annular full-turn sheet — docs/surface-design.md
// §15 calls it "the T83 mesh", the same fixture T83 stitches. It lives in
// the internal test package because sourceBound is a private proof term;
// Mesh.Bound alone cannot detect a missing face bound.
func TestStitchCurvedMeshInheritsEverySourceFaceBound(t *testing.T) {
	t.Parallel()
	sheet := internalAnnularRevolveSheet(t)
	tol := units.Millimeters(0.1)
	source, err := sheet.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	stitched, err := Stitch(t.Context(), sheet)
	require.NoError(t, err)
	mesh, err := stitched.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	sp, ok := stitched.payload.(stitchPayload)
	require.True(t, ok)
	require.Len(t, sp.faces, 4)
	require.Equal(t, source.Bound(), mesh.Bound())
	require.Positive(t, mesh.Bound().Base(),
		"the fixture's own premise: the source mesh's overall bound must be positive, or a missing per-face charge would go unnoticed")

	for i, sourceFace := range sp.faces {
		want, ok := source.sourceBound(sourceFace)
		require.True(t, ok)
		require.Positivef(t, want, "source face %d's own bound must be positive, or the equality check below proves nothing", i)
		got, ok := mesh.sourceBound(sp.liveFaces[i])
		require.Truef(t, ok, "stitched face %d has no inherited source bound", i)
		require.Equalf(t, want, got, "stitched face %d lost its source bound", i)
	}
}

func TestStitchCurvedWeldedRevolveSiblingRejectsWrongEdgeAncestry(t *testing.T) {
	t.Parallel()
	sheet := internalOffAxisArcBody(t, true)
	pieces, err := sheet.Unstitch(t.Context())
	require.NoError(t, err)
	require.Len(t, pieces, 2)
	stitched, err := Stitch(t.Context(), pieces...)
	require.NoError(t, err)
	sp, ok := stitched.payload.(stitchPayload)
	require.True(t, ok)
	require.GreaterOrEqual(t, sp.plan.groups, 2)

	// Keep the real welded body, but claim one copied edge belongs to a
	// different original edge's weld group. The ancestry gate must refuse.
	plan := *sp.plan
	plan.group = make(map[*Edge]int, len(sp.plan.group))
	var first *Edge
	firstGroup := -1
	for edge, group := range sp.plan.group {
		plan.group[edge] = group
		if first == nil {
			first, firstGroup = edge, group
		}
	}
	for _, group := range sp.plan.group {
		if group != firstGroup {
			plan.group[first] = group
			break
		}
	}
	sp.plan = &plan
	_, err = tessellateStitchCurved(t.Context(), stitched, sp, 0.1, VerifyAll)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "a weld joins different original edges")
}

// TestStitchCurvedWeldedRevolveSiblingsInheritEverySourceFaceBound restates
// T88's per-face relation for the sibling-weld route (§10.2): each live
// face's bound must trace back to the ORIGINAL revolve sheet's own source
// mesh face, reached here through unstitchPayload.face, since sp.faces on
// this route holds the intermediate unstitched COPY, not the original.
// TestStitchCurvedMeshInheritsEverySourceFaceBound covers the direct route
// (§10.1) by reading sp.faces straight; a regression confined to
// stitchSiblingRevolveSource's own paired construction leaves that test
// green, which is why the sibling route needs this assertion of its own.
func TestStitchCurvedWeldedRevolveSiblingsInheritEverySourceFaceBound(t *testing.T) {
	t.Parallel()
	sheet := internalOffAxisArcBody(t, true)
	tol := units.Millimeters(0.1)
	source, err := sheet.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	pieces, err := sheet.Unstitch(t.Context())
	require.NoError(t, err)
	require.Len(t, pieces, 2)
	stitched, err := Stitch(t.Context(), pieces...)
	require.NoError(t, err)
	mesh, err := stitched.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	sp, ok := stitched.payload.(stitchPayload)
	require.True(t, ok)
	require.Len(t, sp.faces, 2)

	for i, copied := range sp.faces {
		up, ok := copied.body.payload.(unstitchPayload)
		require.True(t, ok)
		want, ok := source.sourceBound(up.face)
		require.True(t, ok)
		require.Positivef(t, want, "original face %d's own bound must be positive, or the equality check below proves nothing", i)
		got, ok := mesh.sourceBound(sp.liveFaces[i])
		require.Truef(t, ok, "stitched sibling face %d has no inherited source bound", i)
		require.Equalf(t, want, got, "stitched sibling face %d lost its original source bound", i)
	}
}

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

	capped, err := wall.Patch(t.Context(), Edges(Free()).Exactly(8))
	require.NoError(t, err)

	solid, err := Stitch(t.Context(), capped)
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

	mesh, err := solid.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, wantBound, mesh.Bound().Base())
	require.Positive(t, mesh.areaSlack)
}

// internalStitchedBox builds T3's own worked box example (stitchBoxSheets,
// stitch_test.go) inside package decad, since that helper lives in the
// external test package this file cannot import.
func internalStitchedBox(t *testing.T, doc *Document) *Body {
	t.Helper()
	w := sketch.NewWorld()

	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	walls, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along}, WithSurfaceResult())
	require.NoError(t, err)

	bs, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	brect := bs.CreateRectangle(0, 0, 100, 60)
	bs.Fix(brect.A)
	_, err = bs.Solve(t.Context())
	require.NoError(t, err)
	bottom, err := doc.Patch(t.Context(), bs, bs.Profiles()[0])
	require.NoError(t, err)

	topPlane, err := w.CreateOffsetPlane(w.XY(), 10)
	require.NoError(t, err)
	ts, err := w.CreateSketch(topPlane)
	require.NoError(t, err)
	trect := ts.CreateRectangle(0, 0, 100, 60)
	ts.Fix(trect.A)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	top, err := doc.Patch(t.Context(), ts, ts.Profiles()[0])
	require.NoError(t, err)

	box, err := Stitch(t.Context(), walls, bottom, top)
	require.NoError(t, err)
	return box
}

// TestTessellateStitchPublishesZeroSymDiffForClosedZeroBoundBody pins the
// gate this increment adds, directly against the mesh's own private proof
// fields the public API does not expose: a CLOSED, all-planar,
// zero-vertex-bound stitched body publishes volSymDiff == 0 with
// symDiffOK == true. Shown-to-fail: forcing symDiffOK to false
// unconditionally (this increment's own publication step) turns this red —
// the leg boolean_test.go's own T71 exercises end to end through Union.
func TestTessellateStitchPublishesZeroSymDiffForClosedZeroBoundBody(t *testing.T) {
	t.Parallel()
	box := internalStitchedBox(t, New())
	mesh, err := box.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, mesh.symDiffOK)
	require.Zero(t, mesh.volSymDiff)
}

// TestTessellateStitchDoesNotPublishSymDiffForAPlacedBody is the ZERO-BOUND
// gate's own leg: a placement widens every vertex bound (rigidRoundAllow),
// so stitchZeroVertexBound no longer holds even though the body stays
// CLOSED and all-planar. Shown-to-fail: skipping the zeroBound check (always
// publishing symDiffOK true for a BodySolid stitchPayload) turns this red.
func TestTessellateStitchDoesNotPublishSymDiffForAPlacedBody(t *testing.T) {
	t.Parallel()
	box := internalStitchedBox(t, New())
	motion, err := r3.Translation(r3.NewVec(5, 5, 5))
	require.NoError(t, err)
	placed, err := box.Placed(t.Context(), motion)
	require.NoError(t, err)

	mesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.False(t, mesh.symDiffOK)
}

// TestTessellateStitchDoesNotPublishSymDiffForAnOpenSheet is the
// CLOSED-body gate's own leg: the walls alone, stitched as one operand
// (Table C's first row — free top/bottom rims, so the result stays a
// BodySheet), are all-planar and zero-bound, but an open body has no
// occupied volume to prove (docs/surface-design.md §10). Shown-to-fail:
// dropping the b.Kind() == BodySolid guard (running the zero-bound check
// regardless of kind) turns this red.
func TestTessellateStitchDoesNotPublishSymDiffForAnOpenSheet(t *testing.T) {
	t.Parallel()
	doc := New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	walls, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along}, WithSurfaceResult())
	require.NoError(t, err)

	sheet, err := Stitch(t.Context(), walls)
	require.NoError(t, err)
	require.Equal(t, BodySheet, sheet.Kind())

	mesh, err := sheet.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.False(t, mesh.symDiffOK)
}

// stitchTestSquareFaceWithHole builds a hand-built planar face at z =
// corner.Z, an outer 10x10 square (CCW, stitchTestSquareFace's own winding)
// with a 2x2 hole square centred at (5, 5) — the identical hand-authoring
// style stitchTestSquareFace (stitch_internal_test.go) uses, extended with a
// second loop, so triangulateStitchFaces's own hole-bridging path
// (bridgeHole/earClip, triangulate.go) can be exercised directly without the
// full Extrude/Patch/Stitch pipeline. classOf and the returned positions
// slice let the caller check the triangulated set's own summed area against
// the analytic 100 - 4 = 96 mm^2 the two loops denote.
func stitchTestSquareFaceWithHole(corner r3.Vec) (f *Face, classOf map[*Vertex]int, positions []r3.Vec) {
	corners := [8]r3.Vec{
		corner,
		corner.Add(r3.NewVec(10, 0, 0)),
		corner.Add(r3.NewVec(10, 10, 0)),
		corner.Add(r3.NewVec(0, 10, 0)),
		corner.Add(r3.NewVec(4, 4, 0)),
		corner.Add(r3.NewVec(4, 6, 0)),
		corner.Add(r3.NewVec(6, 6, 0)),
		corner.Add(r3.NewVec(6, 4, 0)),
	}
	verts := make([]*Vertex, 8)
	classOf = map[*Vertex]int{}
	positions = make([]r3.Vec, 8)
	for i, p := range corners {
		verts[i] = &Vertex{position: p}
		classOf[verts[i]] = i
		positions[i] = p
	}
	outerEdges := make([]*Edge, 4)
	for i := range 4 {
		outerEdges[i] = &Edge{curve: Line3{}, start: verts[i], end: verts[(i+1)%4]}
	}
	holeEdges := make([]*Edge, 4)
	for i := range 4 {
		holeEdges[i] = &Edge{curve: Line3{}, start: verts[4+i], end: verts[4+(i+1)%4]}
	}
	frame, err := r3.NewFrame(corner, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	if err != nil {
		panic(err)
	}
	f = &Face{
		surface: Plane{Frame: frame},
		loops: []*Loop{
			{outer: true, coedges: []coedge{
				{edge: outerEdges[0], forward: true},
				{edge: outerEdges[1], forward: true},
				{edge: outerEdges[2], forward: true},
				{edge: outerEdges[3], forward: true},
			}},
			{outer: false, coedges: []coedge{
				{edge: holeEdges[0], forward: true},
				{edge: holeEdges[1], forward: true},
				{edge: holeEdges[2], forward: true},
				{edge: holeEdges[3], forward: true},
			}},
		},
		area: 96,
	}
	for _, e := range append(outerEdges, holeEdges...) {
		e.faces = []*Face{f}
	}
	return f, classOf, positions
}

// triAreaXY is twice a planar XY triangle's signed area's own absolute
// value, halved: the shoelace formula over the two edge vectors from a. Every
// coordinate stitchTestSquareFaceWithHole builds is a small exactly
// representable integer, so this sum is exact — no rounding for a tiling
// check to hide behind.
func triAreaXY(a, b, c r3.Vec) float64 {
	return math.Abs((b.X-a.X)*(c.Y-a.Y)-(b.Y-a.Y)*(c.X-a.X)) / 2
}

// TestTriangulateStitchFacesTilesAHoledFaceExactly is docs/surface-design.md's
// T75, the tiling leg of this increment's own three-part exactness argument
// (tessellate_stitch.go's own volSymDiff comment, step 3), narrowed to the
// HOLED case: hole-bridging
// ahead of ear clipping (triangulate.go) must not merely avoid an error, it
// must tile the outer-minus-hole region with no gap and no overlap. Every
// triangle's own area is strictly positive (no degenerate sliver a bridge
// stub could have left behind), every triangle belongs to the one face
// handed in, and the triangles' own summed area is bit-exactly the analytic
// 96 mm^2 — a gap would undershoot it, an overlap would overshoot it, and
// only an exact tiling lands on it exactly.
func TestTriangulateStitchFacesTilesAHoledFaceExactly(t *testing.T) {
	t.Parallel()
	f, classOf, positions := stitchTestSquareFaceWithHole(r3.Vec{})

	tris, triFaces, err := triangulateStitchFaces(t.Context(), []*Face{f}, classOf)
	require.NoError(t, err)
	require.NotEmpty(t, tris)

	total := 0.0
	for i, tri := range tris {
		require.Same(t, f, triFaces[i])
		a, b, c := positions[tri[0]], positions[tri[1]], positions[tri[2]]
		area := triAreaXY(a, b, c)
		require.Positive(t, area, "no zero-area sliver survives triangulation")
		total += area
	}
	require.Equal(t, 96.0, total, "the triangulated set tiles the outer-minus-hole region exactly")
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
