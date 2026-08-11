package decad_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/prism-boolean-design.md PR2's black-box test suite
// (§14, §15): the clean-nesting structural match for Cut and Intersect.
// discBody (prism_boolean_bounds_test.go), boxBody (clearance_test.go) and
// anyFaceIsFaceted (prism_boolean_test.go) are the shared helpers.
//
// The two tests of the resolved analytic path draw circular sections, because
// what they assert is that the tool's own recorded CircleSeg carries through
// byte-identical. Every fallback test instead draws a rectangular section: it
// asserts which gate the pair misses and that the mesh path's own result is
// unchanged, and neither depends on the section's curve kinds. The choice is a
// cost one. The mesh path tessellates both operands at a chord tolerance
// derived from the pair's own diameter (boolean.go's boolChordFactor), so a
// circular loop's facet count is scale-invariant at a few hundred segments,
// and the facet-pair classification that follows is quadratic in exactly that
// count; a rectangular loop costs four segments at any size. Curved operands
// through the mesh path are covered by boolean_test.go's own suite.

// boxBodySymmetric extrudes the axis-aligned rectangle (x0, y0)-(x1, y1)
// symmetrically about its own sketch plane, spanning [-half, +half] — used to
// keep a fallback fixture's tool clear of the target's own cap planes (a cap
// coincidence is a coplanar contact the mesh path refuses on its own,
// independent of this design, and is not what these fallback tests probe).
func boxBodySymmetric(t *testing.T, doc *decad.Document, x0, y0, x1, y1, half float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(half)})
	require.NoError(t, err)
	return body
}

// holedBoxSymmetric extrudes a square annulus (outer half-width outer, inner
// square hole of half-width inner, both centered on the origin) symmetrically
// about its own sketch plane — G6's holed Cut-tool fixture (§15).
func holedBoxSymmetric(t *testing.T, doc *decad.Document, outer, inner, half float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-outer, -outer, outer, outer)
	s.Fix(rect.A)
	s.CreateRectangle(-inner, -inner, inner, inner)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof, `the annulus's holed region should exist`)
	body, err := doc.Extrude(s, prof, decad.Symmetric{D: units.Millimeters(half)})
	require.NoError(t, err)
	return body
}

// cylinderWall returns the one Cylinder-surfaced face of b — every circular
// fixture in this file is a plain disc, so exactly one exists.
func cylinderWall(t *testing.T, b *decad.Body) decad.Cylinder {
	t.Helper()
	faces, err := decad.Faces(decad.Cylindrical()).Exactly(1).SelectFaces(b)
	require.NoError(t, err)
	cyl, ok := faces[0].Surface().(decad.Cylinder)
	require.True(t, ok)
	return cyl
}

// TestPrismCutCleanNestingBoreThroughHub is §15's F1 workload: a bore cut
// through a hole-free hub, both drawn on one sketch plane with no placement
// between them (identity re-expression, §7's decidable zero case for
// section displacement — the only charge left is the tool's own axial
// reach, which is zero here too since Cut keeps the target's own z0Delta/
// z1Delta unchanged).
func TestPrismCutCleanNestingBoreThroughHub(t *testing.T) {
	const R, r, h = 20.0, 5.0, 10.0
	doc := decad.New()
	target := discBody(t, doc, 0, R, h)
	tool := discBody(t, doc, 0, r, 3*h) // spans the target's full height

	// Snapshot both operands' own boundary before the cut retires them.
	targetCyl := cylinderWall(t, target)
	toolCyl := cylinderWall(t, tool)

	got, err := decad.Cut(target, tool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got), `the clean-nesting cut must build analytically`)

	walls, err := decad.Faces(decad.Cylindrical()).Exactly(2).SelectFaces(got)
	require.NoError(t, err)
	var outerWall, holeWall *decad.Face
	for _, f := range walls {
		cyl := f.Surface().(decad.Cylinder)
		if cyl.Radius == targetCyl.Radius {
			outerWall = f
		} else {
			holeWall = f
		}
	}
	require.NotNil(t, outerWall, `the target's own outer wall survives`)
	require.NotNil(t, holeWall, `the tool's own outer boundary survives as the new hole`)
	// Byte-identical reproduction (§4.2's structural-match claim), not merely
	// matching area: every recorded field of the target's own outer loop and
	// the tool's own outer loop carries through unchanged.
	require.Equal(t, targetCyl, outerWall.Surface().(decad.Cylinder),
		`the result's outer wall is byte-identical to the target's own pre-cut outer loop`)
	require.Equal(t, toolCyl, holeWall.Surface().(decad.Cylinder),
		`the new hole is byte-identical to the tool's own pre-cut outer loop`)

	vol, err := got.Volume()
	require.NoError(t, err)
	want := (R*R - r*r) * math.Pi * h
	bound := boundMM3(t, vol)
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), bound,
		`the reported volume must lie within the published bound of the closed form`)
	require.Positive(t, bound)
	require.Less(t, bound, want*1e-9, `the bound sits many decades below the value`)
}

// TestPrismCutCleanNestingKeepsAToolVertexTheFormulaMisses cuts with a tool
// whose section has a vertex the line parameterization's own float formula does
// not reproduce: for the pair 4/7 → 10/3, start + 1·(end − start) lands one ulp
// short of 10/3. buildPrismScene creates one sketch point per walked endpoint,
// so a walk that answered the formula rather than the record would hand sketch
// two points where the tool's record states one shared vertex; sketch would
// admit that region on its own proximity threshold and RecordProfile would then
// refuse it as ErrUnrecordableProfile, past §3.4's point of no return, for a
// pair §4.2 resolves correctly. The vertex must instead reach the result
// unchanged, as §4.2's verbatim structural match states.
func TestPrismCutCleanNestingKeepsAToolVertexTheFormulaMisses(t *testing.T) {
	u0, u1 := 4.0/7.0, 10.0/3.0
	require.NotEqual(t, u1, u0+1.0*(u1-u0),
		`premise: evaluating the formula at t = 1 misses this tool vertex`)

	const plateSide, plateHeight = 20.0, 10.0
	doc := decad.New()
	w := sketch.NewWorld()
	plate := boxBody(t, doc, 0, 0, plateSide, plateSide, plateHeight)
	// A quadrilateral strictly inside the plate, swept clear through it: the
	// clean-nesting sub-case, with the awkward vertex on its own boundary.
	pts := [][2]float64{{u0, 2}, {u1, 2}, {3, 8}, {1, 8}}
	tool := polyPrism(t, doc, w, w.XY(), pts, 2*plateHeight)

	got, err := decad.Cut(plate, tool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got), `the clean-nesting cut must build analytically`)

	// The result states the tool's own recorded coordinate at both cap levels,
	// bit for bit — not a coordinate re-derived from its parameterization.
	corners := bodyVertexPositions(got)
	require.Contains(t, corners, r3.NewVec(u1, 2, 0),
		`the tool's own vertex carries through byte-identical at the lower cap`)
	require.Contains(t, corners, r3.NewVec(u1, 2, plateHeight),
		`the tool's own vertex carries through byte-identical at the upper cap`)

	// Shoelace: the quadrilateral's own area, which the plate loses over its
	// full height.
	quad := 0.0
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		quad += p[0]*q[1] - q[0]*p[1]
	}
	quad = math.Abs(quad) / 2

	vol, err := got.Volume()
	require.NoError(t, err)
	want := (plateSide*plateSide - quad) * plateHeight
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), boundMM3(t, vol),
		`the reported volume must lie within the published bound of the closed form`)
}

// TestPrismCutCleanNestingKeepsAToolArcEndTheAnglesMiss is the arc's own half of
// the case above, and it needs no platform luck: an arc walk reaches its far end
// through atan2 plus the sweep, so the end angle it evaluates is already rounded
// and the point it lands on can sit an ulp off the arc's recorded End. Where the
// next segment states that same vertex from its own record, the scene would hold
// two points for it and the resolved cut would refuse at RecordProfile.
func TestPrismCutCleanNestingKeepsAToolArcEndTheAnglesMiss(t *testing.T) {
	const plateSide, plateHeight = 30.0, 10.0
	const cu, cv, r, rOuter = 10.3, 9.7, 4.7, 8.0
	th0, th1 := 0.05, 0.75

	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, plateSide, plateSide, plateHeight)

	// A wedge strictly inside the plate: an inner arc of radius r about
	// (cu, cv), two radial sides, and a straight outer chord. Its arc meets a
	// line at each end, which is the junction the scene must not split.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(cu, cv)
	s.Fix(center)
	arcStart := s.CreatePoint(cu+r*math.Cos(th0), cv+r*math.Sin(th0))
	arcEnd := s.CreatePoint(cu+r*math.Cos(th1), cv+r*math.Sin(th1))
	outerEnd := s.CreatePoint(cu+rOuter*math.Cos(th1), cv+rOuter*math.Sin(th1))
	outerStart := s.CreatePoint(cu+rOuter*math.Cos(th0), cv+rOuter*math.Sin(th0))
	s.Fix(arcStart)
	s.CreateArc(center, arcStart, arcEnd)
	s.CreateLine(arcEnd, outerEnd)
	s.CreateLine(outerEnd, outerStart)
	s.CreateLine(outerStart, arcStart)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	tool, err := doc.Extrude(s, profiles[0], decad.Symmetric{D: units.Millimeters(2 * plateHeight)})
	require.NoError(t, err)

	// Both operands' own answers, read before the cut retires them.
	plateVol, err := plate.Volume()
	require.NoError(t, err)
	toolVol, err := tool.Volume()
	require.NoError(t, err)

	got, err := decad.Cut(plate, tool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got), `the clean-nesting cut must build analytically`)

	corners := bodyVertexPositions(got)
	require.Contains(t, corners, r3.NewVec(cu+r*math.Cos(th1), cv+r*math.Sin(th1), plateHeight),
		`the tool's own arc end carries through byte-identical at the upper cap`)
	require.Contains(t, corners, r3.NewVec(cu+r*math.Cos(th1), cv+r*math.Sin(th1), 0),
		`the tool's own arc end carries through byte-identical at the lower cap`)

	// The cut removes the tool's own footprint over the plate's height, and the
	// tool spans four times that height: no closed form for the wedge's area is
	// needed, only both operands' own published volumes and bounds.
	vol, err := got.Volume()
	require.NoError(t, err)
	want := volumeMM(t, plateVol) - volumeMM(t, toolVol)/4
	slack := boundMM3(t, vol) + boundMM3(t, plateVol) + boundMM3(t, toolVol)/4
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), slack,
		`the plate loses exactly the tool's own footprint over its own height`)
}

// bodyVertexPositions collects every vertex position b's faces reach, for a
// byte-identical comparison against a coordinate an operand recorded.
func bodyVertexPositions(b *decad.Body) []r3.Vec {
	var out []r3.Vec
	for _, f := range b.Faces() {
		for _, l := range f.Loops() {
			for _, e := range l.Edges() {
				out = append(out, e.Start().Position().Value, e.End().Position().Value)
			}
		}
	}
	return out
}

// TestPrismIntersectFullyNestedPairReturnsInnerOperand is §15's own required
// test: Intersect of a fully-nested pair returns the inner operand's own
// geometry, verbatim, whichever order the caller passes the operands in
// (§3.2's relation is symmetric).
func TestPrismIntersectFullyNestedPairReturnsInnerOperand(t *testing.T) {
	const R, r, h = 20.0, 5.0, 10.0
	doc := decad.New()

	big1 := discBody(t, doc, 0, R, h)
	small1 := discBody(t, doc, 0, r, h)
	smallCyl := cylinderWall(t, small1)
	gotBigFirst, err := decad.Intersect(big1, small1)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(gotBigFirst))

	big2 := discBody(t, doc, 0, R, h)
	small2 := discBody(t, doc, 0, r, h)
	gotSmallFirst, err := decad.Intersect(small2, big2)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(gotSmallFirst))

	want := r * r * math.Pi * h
	for _, got := range []*decad.Body{gotBigFirst, gotSmallFirst} {
		require.Equal(t, smallCyl, cylinderWall(t, got),
			`the result's outer loop reproduces the inner operand's own loop, byte-identical`)
		vol, err := got.Volume()
		require.NoError(t, err)
		bound := boundMM3(t, vol)
		require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), bound)
	}

	bigFirstVol, err := gotBigFirst.Volume()
	require.NoError(t, err)
	smallFirstVol, err := gotSmallFirst.Volume()
	require.NoError(t, err)
	require.Equal(t, bigFirstVol, smallFirstVol, `both call orders return the same body`)
}

// TestPrismIntersectDisjointFootprintsFallsBack is §15's disjoint trap
// (Task 3.3): two hole-free, coplanar, equal-height prisms whose footprints
// do not touch clear G1-G6 (their z-intervals coincide exactly), so the
// pair reaches §4.2's clean-nesting search — which must NOT mistake a
// disjoint pair's two untouched cells for a nested one. This is the test a
// structural match that skipped the nesting proof (matching only the
// smaller operand's own untouched cell, never checking it comes back as a
// HOLE of the larger one) would fail: it would return the smaller operand
// itself instead of falling back to the mesh path's own unchanged
// BooleanEmpty outcome.
func TestPrismIntersectDisjointFootprintsFallsBack(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 20, 0, 30, 10, 10)
	beforeRecipe := doc.Recipe()

	_, err := decad.Intersect(a, b)
	require.ErrorIs(t, err, decad.ErrBooleanFailed)
	var be *decad.BooleanError
	require.ErrorAs(t, err, &be)
	require.Equal(t, decad.OpIntersect, be.Op)
	require.Equal(t, decad.BooleanEmpty, be.Code,
		`the unchanged mesh-path outcome for a genuinely empty intersection`)
	require.Equal(t, beforeRecipe, doc.Recipe(), `a refused boolean records no step`)
}

// TestPrismCutDisjointFootprintFallsBack is the disjoint trap's Cut arm: the
// tool never touches the target at all, so no hole can reproduce it, and the
// pair falls back to the mesh path's own unchanged "tool doesn't touch"
// outcome — the target survives unchanged.
func TestPrismCutDisjointFootprintFallsBack(t *testing.T) {
	doc := decad.New()
	target := boxBody(t, doc, 0, 0, 10, 10, 10)
	tool := boxBody(t, doc, 20, 0, 30, 10, 10)

	got, err := decad.Cut(target, tool)
	require.NoError(t, err)
	require.True(t, anyFaceIsFaceted(got), `an unresolved topology falls back to the mesh path`)
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, 1000.0, volumeMM(t, vol), `the disjoint tool removes nothing`)
}

// TestPrismCutG5FallsBackWhenToolDoesNotSpanTarget is §15's G5 fallback for
// Cut: a pocketing tool shorter than the target's own height falls back to the
// mesh path's own blind-hole result, unchanged. The tool is built symmetrically
// about the shared sketch plane so its own caps land clear of the target's —
// a coincident cap plane is the mesh path's own separate, pre-existing
// coplanar-contact limitation (§1), not what G5 is under test for here.
func TestPrismCutG5FallsBackWhenToolDoesNotSpanTarget(t *testing.T) {
	// half and toolHalf are the two footprints' own half-widths.
	const half, toolHalf, h, reach = 10.0, 3.0, 10.0, 2.0
	doc := decad.New()
	target := boxBody(t, doc, -half, -half, half, half, h)                            // z: 0..10
	tool := boxBodySymmetric(t, doc, -toolHalf, -toolHalf, toolHalf, toolHalf, reach) // z: -2..2, short of target's z1 = 10

	got, err := decad.Cut(target, tool)
	require.NoError(t, err)
	require.True(t, anyFaceIsFaceted(got), `a non-spanning tool is not the clean-nesting shape`)

	vol, err := got.Volume()
	require.NoError(t, err)
	// Only the tool's positive half (z 0..2) actually removes material; its
	// negative half sticks out below the target's own bottom cap.
	want := 4*half*half*h - 4*toolHalf*toolHalf*reach
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), boundMM3(t, vol))
}

// TestPrismIntersectG5FallsBackOnDisjointZIntervals is §15's G5 fallback for
// Intersect: a coplanar pair drawn on the same sketch plane always shares
// the plane's own z = 0, so the only reachable "disjoint interval" shape for
// an admitted-by-G3 pair is exactly touching there (Along meets Against at
// z = 0) rather than truly separated — this is the mesh path's own
// pre-existing coplanar-cap contact limitation, unaffected by this design.
func TestPrismIntersectG5FallsBackOnDisjointZIntervals(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, -5, -5, 5, 5, 10) // z: 0..10
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-5, -5, 5, 5)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(3), Dir: decad.Against}) // z: -3..0
	require.NoError(t, err)

	_, err = decad.Intersect(a, b)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	var be *decad.BooleanError
	require.ErrorAs(t, err, &be)
	require.Equal(t, decad.BooleanUnsupportedContact, be.Code)
	require.Len(t, doc.Bodies(), 2, `a refused boolean leaves both operands live`)
}

// TestPrismCutG6HoledToolFallsBackKeepingTheStandingPost is §15's own named
// test: a holed Cut tool falls back to the mesh path, which keeps the
// material standing inside the tool's own hole — the post the analytic path
// would have dropped, since its clean-nesting match describes the removed
// tool as ONE new hole, sound only while the tool is hole-free (G6).
func TestPrismCutG6HoledToolFallsBackKeepingTheStandingPost(t *testing.T) {
	// half, outer and inner are the three footprints' own half-widths.
	const half, outer, inner, h = 15.0, 8.0, 3.0, 10.0
	doc := decad.New()
	target := boxBody(t, doc, -half, -half, half, half, h)
	tool := holedBoxSymmetric(t, doc, outer, inner, 11) // spans target fully, caps clear of it

	got, err := decad.Cut(target, tool)
	require.NoError(t, err)
	require.True(t, anyFaceIsFaceted(got), `a holed tool is not the clean-nesting shape (G6)`)

	// The annulus's own hole leaves a standing post: a second, disconnected
	// lump inside the cavity the annulus cut.
	require.Len(t, got.Lumps(), 2, `the post the tool's hole spares stands as its own lump`)

	vol, err := got.Volume()
	require.NoError(t, err)
	want := 4 * (half*half - outer*outer + inner*inner) * h
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), boundMM3(t, vol),
		`the standing post's volume must not be dropped from the result`)
}

// TestPrismCutCrossingToolFallsBackWithNoAnalyticError is §15's crossing-pair
// case: a tool that pokes outside the target cuts the target's own boundary, so
// its edges are Partial and the structural match fails — the pair reaches the
// mesh path with no error surfaced from the analytic attempt itself.
func TestPrismCutCrossingToolFallsBackWithNoAnalyticError(t *testing.T) {
	// half is the target's own footprint half-width.
	const half, h = 10.0, 10.0
	// The tool's own footprint straddles the target's wall at x = half: it
	// enters at x = 7 and its far wall stands outside at x = 13, so two of its
	// four edges are cut by the arrangement.
	const x0, x1, y = 7.0, 13.0, 3.0
	doc := decad.New()
	target := boxBody(t, doc, -half, -half, half, half, h)
	tool := boxBodySymmetric(t, doc, x0, -y, x1, y, 20) // spans the target's full height

	got, err := decad.Cut(target, tool)
	require.NoError(t, err, `the crossing pair must reach the mesh path cleanly, no analytic-resolution error`)
	require.True(t, anyFaceIsFaceted(got))

	overlap := (half - x0) * 2 * y // only the part of the tool inside the target removes material
	want := 4*half*half*h - overlap*h

	vol, err := got.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), boundMM3(t, vol))
}

// TestPrismCutCancellationLeavesDocumentUnchanged is §15's cancellation
// requirement, on the F1 clean-nesting shape: a canceled ctx mid-resolution
// returns ctx.Err() unchanged, with the document and recipe untouched,
// matching the existing modify-op contract and PR1's own cancellation test.
func TestPrismCutCancellationLeavesDocumentUnchanged(t *testing.T) {
	const R, r, h = 20.0, 5.0, 10.0
	doc := decad.New()
	target := discBody(t, doc, 0, R, h)
	tool := discBody(t, doc, 0, r, 3*h)
	beforeRecipe := doc.Recipe()
	beforeBodies := doc.Bodies()

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := decad.CutContext(ctx, target, tool)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, beforeRecipe, doc.Recipe())
	require.Equal(t, beforeBodies, doc.Bodies())
}
