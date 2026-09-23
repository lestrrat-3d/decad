package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md's T11-shaped internal coverage: the
// two hand-built fixtures decad's public seam admits no way to author
// directly, because every real operand Stitch can be handed is already
// locally orientable and manifold by its own construction. Both fixtures
// call the package-private step they exercise directly, on a hand-built
// face set, rather than faking one through Stitch's public entry — the same
// treatment the design brief requires of the non-orientable case, extended
// here to its non-manifold sibling and to a self-overlapping closed
// assembly (R9).

// TestStitchOrientationRefusesMobiusAssembly is Table R row R7's
// orientation-contradiction path: three faces glued pairwise along three
// edges, each shared edge walked FORWARD by both its adjacent faces. That
// is the combinatorial shape of a Möbius strip's simplicial boundary: no
// assignment of a per-face flip can make every shared edge see one forward
// and one backward use, so deriveStitchOrientation must refuse rather than
// publish an inconsistent choice.
func TestStitchOrientationRefusesMobiusAssembly(t *testing.T) {
	t.Parallel()
	e12 := &Edge{curve: Line3{}, start: &Vertex{}, end: &Vertex{}}
	e23 := &Edge{curve: Line3{}, start: &Vertex{}, end: &Vertex{}}
	e31 := &Edge{curve: Line3{}, start: &Vertex{}, end: &Vertex{}}

	f1 := &Face{loops: []*Loop{{outer: true, coedges: []coedge{
		{edge: e12, forward: true}, {edge: e31, forward: true},
	}}}}
	f2 := &Face{loops: []*Loop{{outer: true, coedges: []coedge{
		{edge: e12, forward: true}, {edge: e23, forward: true},
	}}}}
	f3 := &Face{loops: []*Loop{{outer: true, coedges: []coedge{
		{edge: e23, forward: true}, {edge: e31, forward: true},
	}}}}
	e12.faces = []*Face{f1, f2}
	e23.faces = []*Face{f2, f3}
	e31.faces = []*Face{f3, f1}

	err := deriveStitchOrientation([]*Face{f1, f2, f3})
	require.ErrorIs(t, err, ErrDegenerate)
}

// TestStitchClosureRefusesNonManifoldEdge is Table R row R7's other path:
// one edge shared by three faces. Every pairwise contact test the reused
// crossing audit runs would pass for three triangles meeting only along one
// shared edge — the audit decides CONTACT, never adjacency counts — so this
// is exactly the gap the explicit directed-edge parity leg closes
// (correction 1, docs/surface-design.md §6.4).
func TestStitchClosureRefusesNonManifoldEdge(t *testing.T) {
	t.Parallel()
	e := &Edge{curve: Line3{}, start: &Vertex{}, end: &Vertex{}}
	f1 := &Face{loops: []*Loop{{outer: true, coedges: []coedge{{edge: e, forward: true}}}}}
	f2 := &Face{loops: []*Loop{{outer: true, coedges: []coedge{{edge: e, forward: false}}}}}
	f3 := &Face{loops: []*Loop{{outer: true, coedges: []coedge{{edge: e, forward: true}}}}}
	e.faces = []*Face{f1, f2, f3}

	err := checkStitchClosure([]*Face{f1, f2, f3})
	require.ErrorIs(t, err, ErrDegenerate)
}

// stitchTestSquareFace builds one free-standing, self-contained planar
// square face — its own four fresh vertices and edges, all zero-bound, one
// outer loop walked counter-clockwise as viewed from +Z with reversed
// false, exactly the invariant every real wall or patch face already
// satisfies (prism_build.go, patch.go): CCW-as-stored viewed from the
// face's own +frame.N(), reversed only ever toggled by a LATER derivation,
// never set true at first build. corner is the loop's first vertex, and the
// square runs corner -> (corner+10,0,0) -> (corner+10,10,0) -> (corner,10,0)
// in the z=corner.Z plane.
func stitchTestSquareFace(corner r3.Vec) *Face {
	c := [4]r3.Vec{
		corner,
		corner.Add(r3.NewVec(10, 0, 0)),
		corner.Add(r3.NewVec(10, 10, 0)),
		corner.Add(r3.NewVec(0, 10, 0)),
	}
	verts := make([]*Vertex, 4)
	for i, p := range c {
		verts[i] = &Vertex{position: p}
	}
	edges := make([]*Edge, 4)
	for i := range 4 {
		edges[i] = &Edge{curve: Line3{}, start: verts[i], end: verts[(i+1)%4]}
	}
	frame, err := r3.NewFrame(c[0], r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	if err != nil {
		panic(err)
	}
	f := &Face{
		surface: Plane{Frame: frame},
		loops: []*Loop{{outer: true, coedges: []coedge{
			{edge: edges[0], forward: true},
			{edge: edges[1], forward: true},
			{edge: edges[2], forward: true},
			{edge: edges[3], forward: true},
		}}},
		area: 100,
	}
	for _, e := range edges {
		e.faces = []*Face{f}
	}
	return f
}

// TestStitchAuditRefusesOverlappingAssembly is Table R row R9: two
// free-standing squares built at the identical location (stitchTestSquareFace
// twice over the same corner, so every one of the four vertex pairs is
// bit-identical and zero-bound) weld along all four edges into a
// combinatorially valid, consistently orientable closed shell — exactly the
// shape deriveStitchOrientation and checkStitchClosure both admit — whose
// two faces nonetheless occupy the SAME plane region. The reused crossing
// audit is what catches this: every pairwise contact it decides for these
// two faces' triangles is either a full vertex-set match or a coplanar
// overlap, neither of which is the pair's own expected contact, so
// evalStitchContext must surface the audit's ErrDegenerate unchanged rather
// than publish a zero-volume "solid". decad's public seam has no way to
// author two independent operand sheets at the identical location this
// directly, so this is a hand-built fixture fed straight to the evaluator.
func TestStitchAuditRefusesOverlappingAssembly(t *testing.T) {
	t.Parallel()
	origin := r3.NewVec(0, 0, 0)
	a := stitchTestSquareFace(origin)
	b := stitchTestSquareFace(origin)

	plan := buildStitchWeldPlan([]*Face{a, b})
	d := New()
	_, err := evalStitchContext(context.Background(), d, d.nextProducerID(), []*Face{a, b}, plan, r3.Identity())
	require.ErrorIs(t, err, ErrDegenerate)
	require.Empty(t, d.Bodies())
}

// TestStitchRuleSRefusesMultipleSourceBodies is docs/surface-design.md's
// T35: Rule S's single-source-body restriction, driven directly at
// stitchRuleSAdmits on two faces whose own bodies each individually carry a
// construction proof (prismPayload{surfaceResult: true}, the same shape
// payloadProvesSimple's own prismPayload arm admits) — proving the refusal
// is specifically about the COUNT of source bodies, not about either
// payload's own standing. decad's public seam has no way to reach this
// directly: docs/surface-design.md §6.2's Table J admits a free Line3 edge
// alone (J5 is undecidable for every other curve variant, stitch_weld.go's
// own doc comment), so two curved rims from different features never weld
// into a closed set at all.
func TestStitchRuleSRefusesMultipleSourceBodies(t *testing.T) {
	t.Parallel()
	bodyA := &Body{payload: prismPayload{surfaceResult: true}}
	bodyB := &Body{payload: prismPayload{surfaceResult: true}}
	faceA := &Face{body: bodyA}
	faceB := &Face{body: bodyB}

	require.False(t, stitchRuleSAdmits(context.Background(), []*Face{faceA, faceB}))
	// The single-body case is what Rule S DOES admit, confirming the
	// refusal above is about the count rather than the payload shape.
	require.True(t, stitchRuleSAdmits(context.Background(), []*Face{faceA, faceA}))
}

// stitchTestFanAroundVertex builds a closed 3-face fan around a shared
// vertex v: triangles (v, outer[0], outer[1]), (v, outer[1], outer[2]),
// (v, outer[2], outer[0]), each pair sharing the radial edge (v, outer[i])
// between them, and each triangle's own opposite (outer[i], outer[i+1])
// edge free. At v this is the shape auditVertexLinks admits by construction
// — degree 2 at every face, zero ends, one connected component — the
// interior-vertex CYCLE its own doc comment names. Every non-v vertex reads
// as a two-face PATH, also admitted.
func stitchTestFanAroundVertex(v *Vertex, outer [3]*Vertex) [3]*Face {
	radial := [3]*Edge{
		{curve: Line3{}, start: v, end: outer[0]},
		{curve: Line3{}, start: v, end: outer[1]},
		{curve: Line3{}, start: v, end: outer[2]},
	}
	rim := [3]*Edge{
		{curve: Line3{}, start: outer[0], end: outer[1]},
		{curve: Line3{}, start: outer[1], end: outer[2]},
		{curve: Line3{}, start: outer[2], end: outer[0]},
	}
	var faces [3]*Face
	for i := range 3 {
		j := (i + 1) % 3
		// Triangle (v, outer[i], outer[j]): v -> outer[i] via radial[i]
		// forward, outer[i] -> outer[j] via rim[i] forward, outer[j] -> v
		// via radial[j] backward.
		faces[i] = &Face{loops: []*Loop{{outer: true, coedges: []coedge{
			{edge: radial[i], forward: true},
			{edge: rim[i], forward: true},
			{edge: radial[j], forward: false},
		}}}}
	}
	for i := range 3 {
		j := (i + 1) % 3
		radial[i].faces = append(radial[i].faces, faces[i], faces[j])
		rim[i].faces = []*Face{faces[i]}
	}
	return faces
}

// TestStitchVertexLinkAuditRefusesPinchedVertex is the vertex-link gate's
// own shown-to-fail leg: two independent closed 3-face fans
// (stitchTestFanAroundVertex) built around the SAME shared vertex, with no
// edge at all connecting one fan's faces to the other's. Each fan alone is
// a manifold interior vertex (a connected cycle, admitted); sharing the one
// vertex between two otherwise-unconnected fans is exactly the pinch
// docs/surface-design.md §6.4 records — two lobes of a revolved boundary
// touching at an isolated point — collapsed to its bare combinatorial
// shape, the same way TestStitchOrientationRefusesMobiusAssembly pins
// deriveStitchOrientation from a hand-built face set decad's own public
// seam has no way to reach. No REACHABLE model produces this shape (no
// admitted Plane/Cylinder generatrix can touch the axis at an isolated
// point at all — stitch_flux.go's own top comment), which is exactly why a
// hand-built fixture is the only way to prove the gate does anything.
func TestStitchVertexLinkAuditRefusesPinchedVertex(t *testing.T) {
	t.Parallel()
	v := &Vertex{}
	fanA := stitchTestFanAroundVertex(v, [3]*Vertex{{}, {}, {}})
	fanB := stitchTestFanAroundVertex(v, [3]*Vertex{{}, {}, {}})

	faces := append(append([]*Face{}, fanA[:]...), fanB[:]...)
	err := auditVertexLinksForStitchFaces(context.Background(), faces)
	require.ErrorIs(t, err, ErrUnsupported)
}

// TestStitchFluxRefusesNURBSSurface is docs/surface-design.md's T37: a
// NURBSSurface face driven directly at stitchFaceFluxAndMoment, on the
// sealed switch's own default arm. decad's public seam has no way to close
// a free-form-walled sheet at all — Body.Patch refuses any chain carrying a
// NURBSCurve edge outright — so no fixture reaches this through Stitch.
func TestStitchFluxRefusesNURBSSurface(t *testing.T) {
	t.Parallel()
	f := &Face{surface: NURBSSurface{}}
	_, _, _, _, err := stitchFaceFluxAndMoment(f, r3.NewVec(0, 0, 0)) //nolint:dogsled // only the error matters here.
	require.ErrorIs(t, err, ErrUnsupported)
}

// TestStitchFluxRefusesNonzeroNormalBound is docs/surface-design.md's T38:
// a face carrying a nonzero normalBound — a cap-blend band patch's own
// tell, since its published Plane/Cone tag only approximates the ruled
// surface actually built — driven directly at stitchFaceFluxAndMoment.
// decad's public seam has no way to reach this either: Unstitch-then-Stitch
// of a filleted or chamfered solid never re-closes (§6.5's own "no further"
// limit for any non-all-planar body — confirmed directly, a box filleted on
// its four vertical edges and unstitched leaves 16 free edges after
// restitching, not zero), so no fixture reaches this gate through the
// public seam.
func TestStitchFluxRefusesNonzeroNormalBound(t *testing.T) {
	t.Parallel()
	f := &Face{surface: Plane{Frame: mustPlaneFrame()}, normalBound: 1e-9}
	_, _, _, _, err := stitchFaceFluxAndMoment(f, r3.NewVec(0, 0, 0)) //nolint:dogsled // only the error matters here.
	require.ErrorIs(t, err, ErrUnsupported)
}

// TestStitchCurvedMassCorrectsInwardOrientation is the orientation sign's
// own shown-to-fail leg, driven directly at stitchCurvedMass: neither of
// this increment's own public fixtures ever reaches the actual global
// sign-flip branch, because a revolve build's own outward-normal convention
// already agrees with the flux formula's own — disabling that branch
// entirely leaves TestStitchAnnularRevolveSheetClosesToATube and its
// siblings green, which is exactly the "an untested leg" trap CLAUDE.md
// warns against. This test reverses every face of a real annular tube sheet
// BEFORE handing it to stitchCurvedMass, so the flux sum is genuinely
// negative on the first pass and the correction must fire for the published
// volume to come out positive and right at all.
func TestStitchCurvedMassCorrectsInwardOrientation(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 5, 10, 15)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	uAxis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}

	d := New()
	sheet, err := d.Revolve(s, s.Profiles()[0], uAxis, FullRevolution{}, WithSurfaceResult())
	require.NoError(t, err)

	faces := sheet.Faces()
	for _, f := range faces {
		reverseFaceOrientation(f)
	}

	vol, cen, err := stitchCurvedMass(context.Background(), faces, faces[0].loops[0].coedges[0].Start().Position().Value, 0)
	require.NoError(t, err)
	require.Greater(t, vol.Value.Base(), 0.0, "the global sign correction must recover a positive volume from an inward-reversed face set")
	require.InDelta(t, 2000*3.141592653589793, vol.Value.Base(), 1e-6)
	require.InDelta(t, 5.0, cen.Value.X, 1e-9)
}

// TestStitchCurvedMassPlacementAllowanceWidensBounds is the placement
// allowance's own shown-to-fail leg, isolated from every other source of
// widening by varying ONLY stitchCurvedMass's own delta parameter over the
// SAME faces and anchor: TestStitchCurvedVolumeBoundWidensWhenPlaced (the
// public test) places the body through a real rigid motion, whose OWN
// re-lifted coordinates already carry a little extra rounding of their own
// even with delta's charge deleted, so it stayed green when this leg alone
// was cut — recorded here rather than left implicit, and this direct call
// is what actually isolates the leg.
func TestStitchCurvedMassPlacementAllowanceWidensBounds(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 5, 10, 15)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	uAxis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}

	d := New()
	sheet, err := d.Revolve(s, s.Profiles()[0], uAxis, FullRevolution{}, WithSurfaceResult())
	require.NoError(t, err)
	faces := sheet.Faces()
	anchor := faces[0].loops[0].coedges[0].Start().Position().Value

	unplacedVol, unplacedCen, err := stitchCurvedMass(context.Background(), faces, anchor, 0)
	require.NoError(t, err)
	// delta = 1 makes sweptVolumeAllow/sweptMomentAllow's own contribution
	// (proportional to the body's ~2513 mm^2 of face area) many orders of
	// magnitude larger than the couple of ulps every absSumUpper call nudges
	// a bound by regardless of what it is summing — the margin below is set
	// well above that ulp noise floor and well below the ~2513 mm^3 this
	// leg is expected to add, so the assertion is about the LEG, not about
	// absSumUpper's own unconditional upward nudge.
	placedVol, placedCen, err := stitchCurvedMass(context.Background(), faces, anchor, 1)
	require.NoError(t, err)

	const margin = 1.0
	require.Greater(t, placedVol.Bound.Base(), unplacedVol.Bound.Base()+margin)
	require.Greater(t, placedCen.Bound.Base(), unplacedCen.Bound.Base()+margin)
}

// stitchTestCylinderFace builds a hand-made full-circumference Cylinder
// face — radius r, axis Z through the world origin, rims at z = 0 and
// z = height — with the given area/areaBound, for the Cylinder arm's own
// shown-to-fail legs below.
func stitchTestCylinderFace(r, height, area, areaBound float64) *Face {
	return stitchTestCylinderFaceWithRimBound(r, height, area, areaBound, 0)
}

// stitchTestCylinderFaceWithRimBound is stitchTestCylinderFace with the two
// rims' own lengthBound set explicitly, for boundedCircleRadius's own
// shown-to-fail leg: the rim's length is set to its own exact 2*pi*r (so
// rB.value comes out r, matching every other field here) and rimBound is
// otherwise free to set to whatever the test wants to prove propagates.
func stitchTestCylinderFaceWithRimBound(r, height, area, areaBound, rimBound float64) *Face {
	v0 := &Vertex{position: r3.NewVec(r, 0, 0)}
	v1 := &Vertex{position: r3.NewVec(r, 0, height)}
	length := 2 * math.Pi * r
	e0 := &Edge{curve: Circle3{Center: r3.NewVec(0, 0, 0), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(r)}, start: v0, end: v0, length: length, lengthBound: rimBound}
	e1 := &Edge{curve: Circle3{Center: r3.NewVec(0, 0, height), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(r)}, start: v1, end: v1, length: length, lengthBound: rimBound}
	f := &Face{
		surface: Cylinder{Origin: r3.NewVec(0, 0, 0), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(r)},
		loops: []*Loop{
			{outer: true, coedges: []coedge{{edge: e0, forward: true}}},
			{outer: true, coedges: []coedge{{edge: e1, forward: true}}},
		},
		area:      area,
		areaBound: areaBound,
	}
	e0.faces, e1.faces = []*Face{f}, []*Face{f}
	return f
}

// TestStitchCylinderFluxReusesAreaBound is the Cylinder area-bound
// composition's own shown-to-fail leg: K_F = σ·Radius·f.area reuses the
// face's own already-proven area/areaBound rather than integrating
// anything fresh (this file's own doc comment), so the flux's own bound
// must scale with f.areaBound exactly through boundedMul's productUpper
// term — driven directly at stitchFaceFluxAndMoment so neither piScalar's
// own tiny representation error nor any other term can mask the leg,
// unlike the whole-pipeline reading where every term is the same tiny
// ulp-scale magnitude and a deleted leg is easy to lose in the noise.
func TestStitchCylinderFluxReusesAreaBound(t *testing.T) {
	t.Parallel()
	const r, height = 10.0, 10.0
	area := 2 * math.Pi * r * height
	f := stitchTestCylinderFace(r, height, area, 1.0)                    // areaBound=1, far above ulp noise
	flux, _, _, _, err := stitchFaceFluxAndMoment(f, r3.NewVec(0, 0, 0)) //nolint:dogsled // only flux and the error matter here.
	require.NoError(t, err)
	require.GreaterOrEqual(t, flux.bound, r*1.0, "the flux bound must scale with f.areaBound through the Radius multiply")

	zero := stitchTestCylinderFace(r, height, area, 0)
	fluxZero, _, _, _, err := stitchFaceFluxAndMoment(zero, r3.NewVec(0, 0, 0)) //nolint:dogsled // only flux and the error matter here.
	require.NoError(t, err)
	require.Less(t, fluxZero.bound, 1e-6, "with areaBound zero, the flux bound has no other source of that magnitude")
}

// TestStitchCylinderMomentChargesRadiusBound is boundedCircleRadius's own
// shown-to-fail leg for the Cylinder arm: Circle3/Cylinder carry Radius as a
// bare units.Value with no bound field, and a revolve about an axis that is
// not coordinate-aligned through the origin hands back a Radius that is
// itself a rounded re-expression of a recorded point (stitch_flux.go's own
// top comment, revolve_axis.go's axisFrame.walk). Driven directly with a
// synthetic rim lengthBound far above ulp noise, so the leg is provably
// exercised rather than left to a fixture where the true bound happens to
// be zero (every public fixture in stitch_flux_test.go revolves about a
// coordinate-aligned axis through the origin, where it genuinely is).
func TestStitchCylinderMomentChargesRadiusBound(t *testing.T) {
	t.Parallel()
	const r, height = 10.0, 10.0
	area := 2 * math.Pi * r * height
	// anchor is offset along X, away from the cylinder's own axis (which
	// runs along Z through the world origin): A_x = Origin.X - anchor.X is
	// then nonzero, so mx's own (A_x + Axis_x*zMid) factor is nonzero and
	// the piR2Dz term's radius bound actually multiplies through rather
	// than being annihilated by a zero factor (as it would be at anchor
	// (0,0,0), where every one of the cylinder's own symmetry axes zeroes
	// the moment regardless of the radius bound).
	anchor := r3.NewVec(3, 0, 0)
	f := stitchTestCylinderFaceWithRimBound(r, height, area, 0, 1.0) // rim lengthBound=1
	_, mx, _, mz, err := stitchFaceFluxAndMoment(f, anchor)
	require.NoError(t, err)
	require.Greater(t, mx.bound, 0.1, "the Cylinder moment must scale with the rim's own lengthBound through boundedCircleRadius")
	require.Zero(t, mz.bound, "the axis component's own (1 - Axis_z^2) factor is exactly zero, so it carries no radius term to widen")

	zero := stitchTestCylinderFaceWithRimBound(r, height, area, 0, 0)
	_, mxZero, _, _, err := stitchFaceFluxAndMoment(zero, anchor) //nolint:dogsled // only mx and the error matter here.
	require.NoError(t, err)
	require.Less(t, mxZero.bound, 1e-6, "with the rim's own lengthBound zero, the moment bound has no other source of that magnitude")
}

// stitchTestPlaneDiskFace builds a hand-made Plane face bounded by a single
// full circle of radius r centered at the frame origin, with the circle's
// own rim lengthBound set explicitly — boundedCircleRadius's own
// shown-to-fail leg for the Plane arm, the disk sibling of
// TestStitchCylinderMomentChargesRadiusBound.
func stitchTestPlaneDiskFace(r, rimBound float64) *Face {
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	if err != nil {
		panic(err)
	}
	v := &Vertex{position: r3.NewVec(r, 0, 0)}
	e := &Edge{curve: Circle3{Center: r3.NewVec(0, 0, 0), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(r)}, start: v, end: v, length: 2 * math.Pi * r, lengthBound: rimBound}
	f := &Face{
		surface: Plane{Frame: frame},
		loops:   []*Loop{{outer: true, coedges: []coedge{{edge: e, forward: true}}}},
	}
	e.faces = []*Face{f}
	return f
}

// TestStitchPlaneMomentChargesRadiusBound is
// TestStitchCylinderMomentChargesRadiusBound's Plane-arm sibling: the same
// boundedCircleRadius leg, exercised through planeFaceFluxAndMoment's own
// disk decomposition instead.
func TestStitchPlaneMomentChargesRadiusBound(t *testing.T) {
	t.Parallel()
	const r = 10.0
	// anchor is offset along Z, off the disk's own z=0 plane: c_z =
	// Origin.Z - anchor.Z is then nonzero, so the z-moment's own c_z^2*Area
	// term is nonzero and Area's own radius bound actually multiplies
	// through. At anchor (0,0,0) (in-plane), every moment component comes
	// out exactly zero regardless of the radius bound — n_x = n_y = 0 for a
	// disk lying flat in the XY plane, and c_z = 0 too, so this is not a
	// corner case of this particular anchor choice, it is the ONLY choice
	// that exercises the leg at all for a flat disk.
	anchor := r3.NewVec(0, 0, 5)
	f := stitchTestPlaneDiskFace(r, 1.0)                   // rim lengthBound=1
	_, _, _, mz, err := stitchFaceFluxAndMoment(f, anchor) //nolint:dogsled // only mz and the error matter here.
	require.NoError(t, err)
	require.Greater(t, mz.bound, 0.01, "the Plane arm's own Area/Iuu/Ivv terms must scale with the rim's own lengthBound")

	zero := stitchTestPlaneDiskFace(r, 0)
	_, _, _, mzZero, err := stitchFaceFluxAndMoment(zero, anchor) //nolint:dogsled // only mz and the error matter here.
	require.NoError(t, err)
	require.Less(t, mzZero.bound, 1e-6, "with the rim's own lengthBound zero, the moment bound has no other source of that magnitude")
}

// mustPlaneFrame returns an arbitrary valid orthonormal frame for
// TestStitchFluxRefusesNonzeroNormalBound: the normalBound gate fires
// before the surface is ever read, so the frame's own values never matter,
// only that NewFrame succeeds.
func mustPlaneFrame() r3.Frame {
	f, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	if err != nil {
		panic(err)
	}
	return f
}

// stitchTestConeFace builds a hand-made full-circumference Cone face: apex
// at the world origin, axis +Z, half-angle halfAngle, and two full-circle
// rims at (r0, z0) and (r1, z1) — z the distance from the apex along the
// axis — with the rims' own lengthBound set explicitly, for the Cone arm's
// own shown-to-fail legs below (docs/surface-design.md's T48/T49, and the
// radius-bound leg TestStitchCylinderMomentChargesRadiusBound already gives
// the Cylinder arm).
func stitchTestConeFace(halfAngle, r0, z0, r1, z1, rimBound float64) *Face {
	axis := r3.NewVec(0, 0, 1)
	center0 := r3.NewVec(0, 0, z0)
	center1 := r3.NewVec(0, 0, z1)
	v0 := &Vertex{position: r3.NewVec(r0, 0, z0)}
	v1 := &Vertex{position: r3.NewVec(r1, 0, z1)}
	length0 := 2 * math.Pi * r0
	length1 := 2 * math.Pi * r1
	e0 := &Edge{curve: Circle3{Center: center0, Axis: axis, Radius: units.Millimeters(r0)}, start: v0, end: v0, length: length0, lengthBound: rimBound}
	e1 := &Edge{curve: Circle3{Center: center1, Axis: axis, Radius: units.Millimeters(r1)}, start: v1, end: v1, length: length1, lengthBound: rimBound}
	f := &Face{
		surface: Cone{Origin: r3.NewVec(0, 0, 0), Axis: axis, Radius: units.Millimeters(0), HalfAngle: units.Radians(halfAngle)},
		loops: []*Loop{
			{outer: true, coedges: []coedge{{edge: e0, forward: true}}},
			{outer: true, coedges: []coedge{{edge: e1, forward: true}}},
		},
	}
	e0.faces, e1.faces = []*Face{f}, []*Face{f}
	return f
}

// TestStitchConeApexOffsetsFromNonzeroRadius is docs/surface-design.md's
// T48: coneApex's general division, exercised at a nonzero Radius no
// reachable construction site ever sets (this function's own doc comment).
// Shown to fail: deleting the offset term (using apex = Origin unconditionally,
// ignoring Radius/tan(HalfAngle) entirely) makes this test's InDelta
// assertions fail, since Origin and the true apex differ by the offset —
// watched red before landing, then restored.
func TestStitchConeApexOffsetsFromNonzeroRadius(t *testing.T) {
	t.Parallel()
	origin := r3.NewVec(3, -2, 7)
	axis := r3.NewVec(0, 0, 1)
	const halfAngle, radius = 0.6, 4.0
	cone := Cone{Origin: origin, Axis: axis, Radius: units.Millimeters(radius), HalfAngle: units.Radians(halfAngle)}

	apexX, apexY, apexZ, err := coneApex(cone)
	require.NoError(t, err)

	wantOffset := radius / math.Tan(halfAngle)
	wantApex := origin.Sub(axis.Scale(wantOffset))
	require.InDelta(t, wantApex.X, apexX.value, 1e-9)
	require.InDelta(t, wantApex.Y, apexY.value, 1e-9)
	require.InDelta(t, wantApex.Z, apexZ.value, 1e-9)
	require.NotEqual(t, origin.Z, apexZ.value, "a nonzero Radius must move the apex away from Origin")

	zeroRadius := cone
	zeroRadius.Radius = units.Millimeters(0)
	zx, zy, zz, err := coneApex(zeroRadius)
	require.NoError(t, err)
	require.Equal(t, origin.X, zx.value)
	require.Equal(t, origin.Y, zy.value)
	require.Equal(t, origin.Z, zz.value)
	require.Zero(t, zx.bound, "a literal zero Radius must publish an Exact apex")
	require.Zero(t, zy.bound)
	require.Zero(t, zz.bound)
}

// TestStitchConeHalfAngleRefusesDegenerateTangent is docs/surface-design.md's
// T49: a HalfAngle of 0 gives tan exactly 0, and coneApex refuses rather
// than propagate it into a division — no reachable wallCone ever carries
// this angle, since a real one is math.Atan2(nonzero, nonzero), always
// strictly between 0 and pi/2 (revolve_axis.go's own analytic-walk
// requirement). A HalfAngle of NaN is refused one layer up, by
// units.Value.In itself (ErrNotFinite, wrapped rather than reaching
// coneApex's own tangent check at all) — units.Value.In never hands back a
// non-finite float, so coneApex's own `isNonFinite(tanValue)` guard stands
// as a defensive backstop for a pathological finite HalfAngle whose
// math.Tan happens to round to +/-Inf, which no float64 input this test can
// construct actually triggers; that guard is therefore not independently
// shown-to-fail here, unlike the exactly-zero case below. Shown to fail:
// weakening the exactly-zero guard to `!(tanValue >= 0)` makes the
// HalfAngle-0 case return no error instead of ErrUnsupported — watched red
// before landing, then restored.
func TestStitchConeHalfAngleRefusesDegenerateTangent(t *testing.T) {
	t.Parallel()
	base := Cone{Origin: r3.NewVec(0, 0, 0), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(0)}

	needle := base
	needle.HalfAngle = units.Radians(0)
	_, _, _, err := coneApex(needle) //nolint:dogsled // only the error matters here.
	require.ErrorIs(t, err, ErrUnsupported)

	malformed := base
	malformed.HalfAngle = units.Radians(math.NaN())
	_, _, _, err = coneApex(malformed) //nolint:dogsled // only the error matters here.
	require.Error(t, err, "a NaN half-angle must refuse, even though units.Value.In catches it before coneApex's own tangent check runs")

	// A genuine, non-degenerate half-angle must NOT refuse — confirming the
	// two refusals above are about the degenerate angles, not a blanket
	// failure of coneApex itself.
	ordinary := base
	ordinary.HalfAngle = units.Radians(0.6)
	_, _, _, err = coneApex(ordinary) //nolint:dogsled // only the error matters here.
	require.NoError(t, err)
}

// TestStitchConeFluxAndMomentChargeRimRadiusBound is the Cone arm's own
// radius-bound leg, the sibling of TestStitchCylinderMomentChargesRadiusBound
// and TestStitchPlaneMomentChargesRadiusBound: boundedCircleRadius's proven
// bound on each rim must propagate into both flux (through S_F's own
// R_lo^2-R_hi^2 difference) and the first moment (through the rim-slope
// tan^2(beta)). anchor is off both the apex's own axis (nonzero X) and its
// z=0 plane (nonzero Z), so neither term is annihilated by symmetry the way
// it would be at the apex itself or at any point on the axis. Shown to
// fail: deleting boundedCircleRadius's own lengthBound propagation (reading
// Circle3.Radius bare, bound 0, instead) collapses both bounds below this
// test's own floor — watched red before landing, then restored.
func TestStitchConeFluxAndMomentChargeRimRadiusBound(t *testing.T) {
	t.Parallel()
	anchor := r3.NewVec(3, 0, 5)
	f := stitchTestConeFace(0.6, 3, 2, 8, 5, 1.0) // rims' own lengthBound=1, far above ulp noise
	flux, mx, _, _, err := stitchFaceFluxAndMoment(f, anchor)
	require.NoError(t, err)
	require.Greater(t, flux.bound, 0.01, "the flux bound must scale with the rims' own lengthBound through S_F's radius terms")
	require.Greater(t, mx.bound, 0.01, "the moment bound must scale with the rims' own lengthBound through the rim-slope tan^2(beta)")

	zero := stitchTestConeFace(0.6, 3, 2, 8, 5, 0)
	fluxZero, mxZero, _, _, err := stitchFaceFluxAndMoment(zero, anchor)
	require.NoError(t, err)
	require.Less(t, fluxZero.bound, 1e-6, "with the rims' own lengthBound zero, the flux bound has no other source of that magnitude")
	require.Less(t, mxZero.bound, 1e-6, "with the rims' own lengthBound zero, the moment bound has no other source of that magnitude")
}

// TestStitchCurvedMassCorrectsInwardOrientationForCone is the orientation
// sign's own shown-to-fail leg for the Cone arm, the sibling of
// TestStitchCurvedMassCorrectsInwardOrientation: neither public Cone
// fixture (stitch_flux_test.go) ever reaches the actual global sign-flip
// branch, because a revolve build's own outward-normal convention already
// agrees with the flux formula's own. This test reverses every face of a
// real frustum-shell sheet BEFORE handing it to stitchCurvedMass, so the
// flux sum is genuinely negative on the first pass and the correction must
// fire for the published volume to come out positive and right. Shown to
// fail: deleting the `fluxSum.value < 0` branch in stitchCurvedMass makes
// this test's volume come out negative (or its centroid wrong) — watched
// red before landing, then restored.
func TestStitchCurvedMassCorrectsInwardOrientationForCone(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	p1 := s.CreatePoint(0, 5)
	p2 := s.CreatePoint(10, 8)
	p3 := s.CreatePoint(10, 12)
	p4 := s.CreatePoint(0, 15)
	s.Fix(p1)
	s.CreateLine(p1, p2)
	s.CreateLine(p2, p3)
	s.CreateLine(p3, p4)
	s.CreateLine(p4, p1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	uAxis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}

	d := New()
	sheet, err := d.Revolve(s, s.Profiles()[0], uAxis, FullRevolution{}, WithSurfaceResult())
	require.NoError(t, err)

	faces := sheet.Faces()
	for _, f := range faces {
		reverseFaceOrientation(f)
	}

	anchor := faces[0].loops[0].coedges[0].Start().Position().Value
	vol, cen, err := stitchCurvedMass(context.Background(), faces, anchor, 0)
	require.NoError(t, err)
	require.Greater(t, vol.Value.Base(), 0.0, "the global sign correction must recover a positive volume from an inward-reversed Cone face set")

	wantVol, _, wantCX := frustumShellAnalyticsForInternalTest(10, 5, 15, 8, 12)
	require.InDelta(t, wantVol, vol.Value.Base(), 1e-6)
	require.InDelta(t, wantCX, cen.Value.X, 1e-9)
}

// frustumShellAnalyticsForInternalTest is stitch_flux_test.go's
// frustumShellAnalytics, duplicated here because this file's package
// (decad) cannot import the exported test package (decad_test) that
// function lives in.
func frustumShellAnalyticsForInternalTest(uLen, vLo0, vHi0, vLo1, vHi1 float64) (volume, area, centroidX float64) {
	aO, bO := vHi0, vHi1-vHi0
	aI, bI := vLo0, vLo1-vLo0
	iOuter := uLen * (aO*aO + aO*bO + bO*bO/3)
	iInner := uLen * (aI*aI + aI*bI + bI*bI/3)
	volume = math.Pi * (iOuter - iInner)

	jOuter := uLen * uLen * (aO*aO/2 + 2*aO*bO/3 + bO*bO/4)
	jInner := uLen * uLen * (aI*aI/2 + 2*aI*bI/3 + bI*bI/4)
	centroidX = (jOuter - jInner) / (iOuter - iInner)

	slantOuter := math.Hypot(bO, uLen)
	slantInner := math.Hypot(bI, uLen)
	lateralOuter := math.Pi * (vHi0 + vHi1) * slantOuter
	lateralInner := math.Pi * (vLo0 + vLo1) * slantInner
	annulus0 := math.Pi * (vHi0*vHi0 - vLo0*vLo0)
	annulus1 := math.Pi * (vHi1*vHi1 - vLo1*vLo1)
	area = lateralOuter + lateralInner + annulus0 + annulus1
	return volume, area, centroidX
}
