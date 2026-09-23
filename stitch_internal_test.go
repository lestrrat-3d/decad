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
	v0 := &Vertex{position: r3.NewVec(r, 0, 0)}
	v1 := &Vertex{position: r3.NewVec(r, 0, height)}
	e0 := &Edge{curve: Circle3{Center: r3.NewVec(0, 0, 0), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(r)}, start: v0, end: v0}
	e1 := &Edge{curve: Circle3{Center: r3.NewVec(0, 0, height), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(r)}, start: v1, end: v1}
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
