package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin docs/general-boolean-design.md §3 A1's brep result: a
// stacked union whose interface the clean-nesting match leaves unresolved.
// The plate is 40×40×10 on XY; a square boss is 10×10, a round one Ø10.

// internalUnionBrep runs A1's analytic union and requires its brep payload.
func internalUnionBrep(t *testing.T, a, b *Body) brepPayload {
	t.Helper()
	payload, ok, err := tryStackedUnion(t.Context(), a, b)
	require.NoError(t, err)
	require.True(t, ok)
	bp, isBrep := payload.(brepPayload)
	require.True(t, isBrep, "the union is a brep, got %T", payload)
	return bp
}

// internalWallFace finds the one planar face of a brep record whose frame
// normal is n.
func internalWallFace(t *testing.T, bp brepPayload, n r3.Vec) brepFace {
	t.Helper()
	var found []brepFace
	for _, f := range bp.faces {
		if f.planar() && f.frame.N() == n {
			found = append(found, f)
		}
	}
	require.Len(t, found, 1, "one planar face faces %v", n)
	return found[0]
}

// TestStackedUnionBrepFlushCornerBoss is the flush corner boss: the boss
// shares the plate's walls x = 20 and y = −20. Each shared wall is one
// planar face in its own frame, an L of six segments, and every quantity is
// exact: 16000 + 1500 mm³, 5400 mm² (the plate's 1600 + 1500 + 400 + 400 +
// two 550 walls, the boss's 100 top and two 150 walls). Shown to fail with
// the rectangle cancellation in wallFaces deleted (the x = 20 plane then
// chained into two faces that shared an edge, and the record did not pair).
func TestStackedUnionBrepFlushCornerBoss(t *testing.T) {
	t.Parallel()
	doc := New()
	plate := internalBoxBody(t, doc, -20, -20, 20, 20, 10)
	boss := internalBoxBodyAtZ(t, doc, 10, -20, 20, -10, 10, 15)
	bp := internalUnionBrep(t, plate, boss)
	require.Len(t, bp.faces, 9)
	for _, f := range bp.faces {
		require.True(t, f.planar())
		require.Zero(t, f.delta, "every carrier is a recorded plane")
	}
	wall := internalWallFace(t, bp, r3.NewVec(1, 0, 0))
	require.Len(t, wall.region.Outer.Segments, 6, "the flush wall x = 20 is one L-shaped face")
	require.Equal(t, 20.0, wall.z0)
	wall = internalWallFace(t, bp, r3.NewVec(0, -1, 0))
	require.Len(t, wall.region.Outer.Segments, 6, "the flush wall y = −20 is one L-shaped face")
	require.Equal(t, 20.0, wall.z0, "the level along −V is 20")

	got, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	requireClosedTopology(t, got)
	require.Len(t, got.Faces(), 9)
	require.Equal(t, Exact, got.volume.Exactness)
	require.Equal(t, 17500.0, got.volume.Value.Base())
	require.Equal(t, Exact, got.area.Exactness)
	require.Equal(t, 5400.0, got.area.Value.Base())
	require.Equal(t, r3.NewVec(-20, -20, 0), got.bounds.Min)
	require.Equal(t, r3.NewVec(20, 20, 25), got.bounds.Max)
	// 14 vertices and 9 faces: 21 edges, every one convex under evaluator
	// §3's walked boundary (a straight wall reads its outer loop's role).
	require.Len(t, got.Edges(), 21)
	for _, e := range got.Edges() {
		require.True(t, e.IsConvex())
	}
	mesh, err := tessellateContext(t.Context(), got, units.Millimeters(0.05), VerifyAll)
	require.NoError(t, err)
	require.Equal(t, big.NewRat(17500, 1), internalMeshVolumeRat(mesh))
}

// internalPolyPrismBodyAtZ extrudes a closed polygon drawn on the XY plane
// moved to z0, every vertex pinned, by h along +z.
func internalPolyPrismBodyAtZ(t *testing.T, doc *Document, pts [][2]float64, z0, h float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
		s.Fix(sp[i])
	}
	for i := range sp {
		s.CreateLine(sp[i], sp[(i+1)%len(sp)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	body, err := doc.Extrude(s, profiles[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	return body
}

// TestStackedUnionBrepLBossReflexCornerReadsConcave stands an L boss, z 10..15,
// flush in the plate's corner: footprint (0,−20) (20,−20) (20,0) (10,0)
// (10,−10) (0,−10), sharing the walls x = 20 and y = −20. Every wall is a
// planar face recording the stack axis, so the vertical line at the reflex
// corner (10, −10) is a junction of two planar walls, and the walk turns right
// there: it reads concave, and Concave() selects it alone. Every other edge
// reads convex: the remaining vertical lines are left turns, and each rim
// reads its outer loop's role, the boss's base on the plate top included.
// Shown to fail with the planar-planar junction arm of brepgeom.Convex reading
// the owner's loop role, and with wallFaces recording no sweep (the reflex
// line then read convex either way).
func TestStackedUnionBrepLBossReflexCornerReadsConcave(t *testing.T) {
	t.Parallel()
	doc := New()
	plate := internalBoxBody(t, doc, -20, -20, 20, 20, 10)
	boss := internalPolyPrismBodyAtZ(t, doc, [][2]float64{{0, -20}, {20, -20}, {20, 0}, {10, 0}, {10, -10}, {0, -10}}, 10, 5)
	bp := internalUnionBrep(t, plate, boss)
	for _, f := range bp.faces {
		require.True(t, f.planar())
	}
	got, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	requireClosedTopology(t, got)
	require.Equal(t, Exact, got.volume.Exactness)
	require.Equal(t, 16000.0+300*5, got.volume.Value.Base())

	reflex := [2]r3.Vec{r3.NewVec(10, -10, 10), r3.NewVec(10, -10, 15)}
	convexity := internalConvexityByEnds(t, got)
	require.Len(t, convexity, 27)
	for ends, convex := range convexity {
		require.Equal(t, ends != reflex, convex, "edge %v–%v", ends[0], ends[1])
	}
	concave, err := Edges(Concave()).Exactly(1).SelectEdges(got)
	require.NoError(t, err)
	require.Equal(t, reflex[0], concave[0].Start().Position().Value)
	require.Equal(t, reflex[1], concave[0].End().Position().Value)
	convex, err := Edges(Convex()).SelectEdges(got)
	require.NoError(t, err)
	require.Len(t, convex, 26)
	require.NotContains(t, convex, concave[0])
}

// TestStackedUnionBrepFlushEdgeBoss is the flush edge boss: the boss shares
// the plate's wall x = 20 along its middle, so that wall is one T-shaped
// face of eight segments. The body has 10 faces and the same exact 17500 mm³
// and 5400 mm². Shown to fail with cutsOnLine returning nothing (the floor's
// x = 20 edges then ran whole and did not pair with the wall's pieces).
func TestStackedUnionBrepFlushEdgeBoss(t *testing.T) {
	t.Parallel()
	doc := New()
	plate := internalBoxBody(t, doc, -20, -20, 20, 20, 10)
	boss := internalBoxBodyAtZ(t, doc, 10, -5, 20, 5, 10, 15)
	bp := internalUnionBrep(t, plate, boss)
	require.Len(t, bp.faces, 10)
	wall := internalWallFace(t, bp, r3.NewVec(1, 0, 0))
	require.Len(t, wall.region.Outer.Segments, 8, "the flush wall x = 20 is one T-shaped face")

	got, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	requireClosedTopology(t, got)
	require.Len(t, got.Faces(), 10)
	require.Equal(t, Exact, got.volume.Exactness)
	require.Equal(t, 17500.0, got.volume.Value.Base())
	require.Equal(t, 5400.0, got.area.Value.Base())
	mesh, err := tessellateContext(t.Context(), got, units.Millimeters(0.05), VerifyAll)
	require.NoError(t, err)
	require.Equal(t, big.NewRat(17500, 1), internalMeshVolumeRat(mesh))
}

// TestStackedUnionBrepCrossingBoss is the round boss standing on the plate's
// top with its footprint crossing the outline x = 20: centre (18, 0), radius
// 5. The interface scene cuts the plate's line at two points, which key the
// table once each and split the boss's wall into two pieces, the plate's
// x = 20 wall's top edge into three collinear pieces, and the boss's top cap
// into two arcs. The floor is the plate less the disc's inside part, the
// ceiling the disc's outside part, and the union's volume is the plate plus
// the whole boss. The area is 4800 + 150π + 2·S, with S the disc's segment
// past x = 20: 25·acos(0.4) − 2·√21. Shown to fail with the junction table
// keyed without its side (the two crossings then took one point and the
// pieces did not pair), and with circlePoints skipping the whole-circle
// split (the top cap stayed one circle, which paired with no piece).
func TestStackedUnionBrepCrossingBoss(t *testing.T) {
	t.Parallel()
	plate, boss := internalBossOnPlate(t, 18, 10, 15)
	bp := internalUnionBrep(t, plate, boss)
	require.Len(t, bp.faces, 10)
	swept := 0
	for _, f := range bp.faces {
		if !f.planar() {
			swept++
			require.Equal(t, [2]float64{10, 25}, [2]float64{f.z0, f.z1})
			_, isArc := f.wall.(arcSeg)
			require.True(t, isArc, "each cylinder piece is an arc between the two crossings")
		}
		require.Positive(t, f.delta, "every face carries the crossings' cut displacement")
	}
	require.Equal(t, 2, swept)
	wall := internalWallFace(t, bp, r3.NewVec(1, 0, 0))
	require.Len(t, wall.region.Outer.Segments, 6, "the plate's x = 20 wall's top edge is split at both crossings")

	got, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	requireClosedTopology(t, got)
	require.Len(t, got.Faces(), 10)
	require.Len(t, got.Edges(), 20)
	lo, hi := piEnclosed(big.NewRat(16000, 1), big.NewRat(375, 1))
	requireCoversInterval(t, got.volume, lo, hi)
	require.Less(t, got.volume.Bound.Base(), 1e-9)
	segment := 25*math.Acos(0.4) - 2*math.Sqrt(21)
	wantArea := 4800 + 150*math.Pi + 2*segment
	require.LessOrEqual(t, math.Abs(got.area.Value.Base()-wantArea), got.area.Bound.Base())
	require.Less(t, got.area.Bound.Base(), 1e-9)
	mesh, err := tessellateContext(t.Context(), got, units.Millimeters(0.05), VerifyAll)
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	held := internalMeshVolumeRat(mesh)
	bound := new(big.Rat).SetFloat64(mesh.volSymDiff)
	require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, lo)).Cmp(bound), 0)
	require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, hi)).Cmp(bound), 0)
}

// TestStackedUnionBrepRootedFlushBoss roots the flush corner boss inside
// the plate (z = 5..25). The slab both reach is the select-all merge, the
// plate's outline recorded in fragments split at the boss's corners, and the
// vertical edge at the shared corner (20, −20) runs the whole 25 mm as one
// edge: no face has a vertex at z = 5 or z = 10 there. Shown to fail with
// joinCollinear's merge deleted (the edge then split at both slab levels and
// the body carried 25 edges).
func TestStackedUnionBrepRootedFlushBoss(t *testing.T) {
	t.Parallel()
	doc := New()
	plate := internalBoxBody(t, doc, -20, -20, 20, 20, 10)
	boss := internalBoxBodyAtZ(t, doc, 10, -20, 20, -10, 5, 20)
	bp := internalUnionBrep(t, plate, boss)
	require.Len(t, bp.faces, 9)
	got, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	requireClosedTopology(t, got)
	require.Len(t, got.Edges(), 21)
	require.Equal(t, Exact, got.volume.Exactness)
	require.Equal(t, 17500.0, got.volume.Value.Base())
	long := 0
	for _, e := range got.Edges() {
		if e.length == 25 {
			long++
		}
	}
	require.Equal(t, 1, long, "the shared corner's vertical edge is one edge over both operands")
}

// TestStackedUnionBrepMisses pins the pairs the brep build hands to the mesh
// path with no error. An operand carrying a section displacement has
// walls a planar face cannot state (B5). A boss drawn on a plane whose
// origin is offset in the plate's plane re-expresses into the plate's frame,
// which the build admits only as the identity. Shown to fail, one leg at a
// time: with the B5 gate and the walk-and-crossing check deleted together
// (the displaced plate built a brep; either alone still misses it); and with
// the identity gate and the same check deleted together (the offset-frame
// boss built a brep; its merge scene's shared-span width charges a
// crossing, so either alone still misses it).
func TestStackedUnionBrepMisses(t *testing.T) {
	t.Parallel()
	t.Run("displaced operand", func(t *testing.T) {
		t.Parallel()
		doc := New()
		plate := internalBoxBody(t, doc, -20, -20, 20, 20, 10)
		boss := internalBoxBodyAtZ(t, doc, 10, -20, 20, -10, 10, 15)
		pp := plate.payload.(prismPayload)
		pp.sectionDelta = 1e-12
		_, ok, err := tryStackedUnion(t.Context(), &Body{payload: pp}, boss)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("offset frame", func(t *testing.T) {
		t.Parallel()
		doc := New()
		plate := internalBoxBody(t, doc, -20, -20, 20, 20, 10)
		// The boss's plane is the plate's, with its origin moved 0.5 along
		// u: an exact in-plane re-expression that is not the identity.
		w := sketch.NewWorld()
		frame, err := r3.NewFrame(r3.NewVec(0.5, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		plane, err := w.CreatePlaneFromFrame(frame)
		require.NoError(t, err)
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		rect := s.CreateRectangle(9.5, -20, 19.5, -10)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		boss, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(25), Dir: Along})
		require.NoError(t, err)
		require.Zero(t, boss.payload.(prismPayload).sectionDelta)
		_, ok, err := tryStackedUnion(t.Context(), plate, boss)
		require.NoError(t, err)
		require.False(t, ok)
	})
}

// TestStackedUnionBrepPlacedResult re-evaluates the brep under a placement:
// the moved body keeps its exact volume and its box moves with it.
func TestStackedUnionBrepPlacedResult(t *testing.T) {
	t.Parallel()
	doc := New()
	plate := internalBoxBody(t, doc, -20, -20, 20, 20, 10)
	boss := internalBoxBodyAtZ(t, doc, 10, -20, 20, -10, 10, 15)
	got, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(100, 0, 0))
	require.NoError(t, err)
	moved, err := got.Placed(t.Context(), move)
	require.NoError(t, err)
	require.Equal(t, 17500.0, moved.volume.Value.Base())
	require.Equal(t, Exact, moved.volume.Exactness)
	require.Equal(t, r3.NewVec(80, -20, 0), moved.bounds.Min)
	require.Equal(t, r3.NewVec(120, 20, 25), moved.bounds.Max)
}

// TestStackedUnionBrepRootedCrossingBoss roots the Ø10 boss inside the
// plate (z = 5..25) with its centre 2 mm inside x = 20. The slab both reach
// merges into the plate's outline with the disc's overhang, the floor's
// corners at z = 10 lie on the overhanging cylinder piece's side lines, and
// that piece is one face from z = 5 to 25 whose side lines split at z = 10
// (§4.1). Volume: the plate, the whole boss above it, and the overhang's
// segment S = 25·acos(0.4) − 2·√21 over z = 5..10. Area: 4800 + 150π +
// 100·acos(0.4) − 14·√21 (the plate's faces less the notch's 10·√21, the
// segment under the overhang, the plate's top less the disc's inside part,
// the 25π top, and the two cylinder pieces). Shown to fail with sweptFaces
// missing on an event at a piece's end (a miss), with brepgeom.Build
// emitting one use per side line (the split wall's vertical pieces then
// paired with nothing, a miss), and with brepChordWall's split rows deleted
// (the mesh then failed to close).
func TestStackedUnionBrepRootedCrossingBoss(t *testing.T) {
	t.Parallel()
	plate, boss := internalBossOnPlate(t, 18, 5, 20)
	bp := internalUnionBrep(t, plate, boss)
	require.Len(t, bp.faces, 10)
	var outside, inside *brepFace
	for i := range bp.faces {
		f := &bp.faces[i]
		if f.planar() {
			continue
		}
		switch f.z0 {
		case 5:
			outside = f
		case 10:
			inside = f
		}
	}
	require.NotNil(t, outside)
	require.NotNil(t, inside)
	require.Equal(t, 25.0, outside.z1)
	require.Equal(t, []brepSplit{{Z: 10}}, outside.side0, "the overhang's start line splits at the floor")
	require.Equal(t, []brepSplit{{Z: 10}}, outside.side1, "the overhang's end line splits at the floor")
	require.Empty(t, inside.side0)
	require.Empty(t, inside.side1)

	got, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	requireClosedTopology(t, got)
	require.Len(t, got.Faces(), 10)
	require.Len(t, got.Edges(), 22)
	segment := 25*math.Acos(0.4) - 2*math.Sqrt(21)
	wantVolume := 16000 + 375*math.Pi + 5*segment
	require.LessOrEqual(t, math.Abs(got.volume.Value.Base()-wantVolume), got.volume.Bound.Base())
	require.Less(t, got.volume.Bound.Base(), 1e-9)
	wantArea := 4800 + 150*math.Pi + 100*math.Acos(0.4) - 14*math.Sqrt(21)
	require.LessOrEqual(t, math.Abs(got.area.Value.Base()-wantArea), got.area.Bound.Base())
	require.Less(t, got.area.Bound.Base(), 1e-8)
	// The overhang's side lines are two edges each: 5..10, shared with the
	// notched wall x = 20, and 10..25, shared with the inside piece.
	short, long := 0, 0
	for _, e := range got.Edges() {
		if _, line := e.curve.(Line3); !line {
			continue
		}
		switch e.length {
		case 5:
			short++
		case 15:
			long++
		}
	}
	require.Equal(t, 2, short, "the two side pieces below the floor")
	require.Equal(t, 2, long, "the two side pieces above it")
	mesh, err := tessellateContext(t.Context(), got, units.Millimeters(0.05), VerifyAll)
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
}

// TestBrepSideSplitRecordAudit pins falsifyBrepPayload's split rules: a
// split outside the interval, or out of order, is ErrDegenerate.
func TestBrepSideSplitRecordAudit(t *testing.T) {
	t.Parallel()
	base := internalCrossDrilledBrep(t)
	for name, splits := range map[string][]brepSplit{
		"at the interval's end": {{Z: 0}},
		"outside":               {{Z: 3}},
		"out of order":          {{Z: -5}, {Z: -15}},
		"negative displacement": {{Z: -10, ZDelta: -1}},
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			bp := base
			bp.faces = append([]brepFace{}, base.faces...)
			bp.faces[6].side1 = splits
			require.ErrorIs(t, falsifyBrepPayload(t.Context(), bp), ErrDegenerate)
		})
	}
}
