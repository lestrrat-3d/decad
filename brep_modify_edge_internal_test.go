package decad

import (
	"math"
	"math/big"
	"slices"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin route E of docs/brep-modify-design.md (§5, §9): Fillet
// and Chamfer of straight brep edges along one reference axis, on receivers
// the public booleans build. Volumes are checked against closed forms; the
// records are read for the coordinates the construction must carry.

var (
	routeEX = r3.NewVec(1, 0, 0)
	routeEZ = r3.NewVec(0, 0, 1)
)

// internalSquareCrossHole is §9's B1: the 40×20×20 box cut by the 10×10
// square hole along y at x ∈ [15, 25], z ∈ [5, 15].
func internalSquareCrossHole(t *testing.T) (*Document, *Body) {
	t.Helper()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XZ(), -10)
	require.NoError(t, err)
	tool := internalClassBTool(t, doc, w, plane, 11, func(s *sketch.Sketch) {
		r := s.CreateRectangle(15, 5, 25, 15)
		s.Fix(r.A)
	})
	b1, err := Cut(t.Context(), box, tool)
	require.NoError(t, err)
	return doc, b1
}

// internalRouteEPocket is §9's Pocket: the 40×40×10 plate on XY with the
// 20×10 blind pocket x ∈ [10, 30], y ∈ [15, 25] from z = 10 down to z = 5.
func internalRouteEPocket(t *testing.T) (*Document, *Body) {
	t.Helper()
	doc := New()
	plate := internalBoxBody(t, doc, 0, 0, 40, 40, 10)
	tool := internalOffsetBox(t, doc, 10, 15, 30, 25, 5, Distance{D: units.Millimeters(5), Dir: Along})
	pocket, err := Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	_, ok := pocket.payload.(stackedPrismPayload)
	require.True(t, ok, "the pocket is a stacked prism, got %T", pocket.payload)
	return doc, pocket
}

// internalFlushBossUnion unions the plate x, y ∈ [−20, 20], z ∈ [0, 10] with
// a boss extruded from z = 10 to z = 25 over the polygon pts, and requires a
// brep result.
func internalFlushBossUnion(t *testing.T, pts [][2]float64) (*Document, *Body) {
	t.Helper()
	doc := New()
	plate := internalBoxBody(t, doc, -20, -20, 20, 20, 10)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 10)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	points := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		points[i] = s.CreatePoint(p[0], p[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	boss, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(15), Dir: Along})
	require.NoError(t, err)
	union, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	_, ok := union.payload.(brepPayload)
	require.True(t, ok, "the flush boss union is a brep, got %T", union.payload)
	return doc, union
}

// internalCornerBoss is §9's Boss: general-boolean §9's flush corner boss,
// x ∈ [10, 20], y ∈ [−20, −10], standing on the plate's corner.
func internalCornerBoss(t *testing.T) (*Document, *Body) {
	t.Helper()
	return internalFlushBossUnion(t, [][2]float64{{10, -20}, {20, -20}, {20, -10}, {10, -10}})
}

// edgeAt selects the one edge along dir with an endpoint at p.
func edgeAt(dir r3.Vec, p r3.Vec) *EdgeQuery {
	return Edges(ParallelTo(dir), EndpointAt(p)).Exactly(1)
}

// requireBrepResult asserts body is a closed brep of n faces and returns
// its record.
func requireBrepResult(t *testing.T, body *Body, n int) brepPayload {
	t.Helper()
	bp, ok := body.payload.(brepPayload)
	require.True(t, ok, "the result is a brep, got %T", body.payload)
	require.Len(t, bp.faces, n)
	require.Nil(t, bp.stack)
	requireClosedTopology(t, body)
	return bp
}

// planarFaceAt finds the one planar face of a record whose frame normal is n
// and whose level is z.
func planarFaceAt(t *testing.T, bp brepPayload, n r3.Vec, z float64) brepFace {
	t.Helper()
	var found []brepFace
	for _, f := range bp.faces {
		if f.planar() && f.frame.N() == n && f.z0 == z {
			found = append(found, f)
		}
	}
	require.Len(t, found, 1, "one planar face faces %v at level %g", n, z)
	return found[0]
}

// arcEnds is an arc's centre and its two endpoints as an unordered pair.
type arcEnds struct {
	center Point2
	ends   [2]Point2
}

func arcEndsOf(s ArcSeg) arcEnds {
	ends := [2]Point2{s.Start, s.End}
	if ends[1].U < ends[0].U || (ends[1].U == ends[0].U && ends[1].V < ends[0].V) {
		ends[0], ends[1] = ends[1], ends[0]
	}
	return arcEnds{center: s.Center, ends: ends}
}

// loopArcs lists a loop's arcs.
func loopArcs(loop LoopRecord) []arcEnds {
	var out []arcEnds
	for _, seg := range loop.Segments {
		if s, ok := seg.(ArcSeg); ok {
			out = append(out, arcEndsOf(s))
		}
	}
	return out
}

// requireChord asserts a loop holds one LineSeg between a and b, in either
// direction.
func requireChord(t *testing.T, loop LoopRecord, a, b Point2) {
	t.Helper()
	n := 0
	for _, seg := range loop.Segments {
		s, ok := seg.(LineSeg)
		if ok && ((s.Start == a && s.End == b) || (s.Start == b && s.End == a)) {
			n++
		}
	}
	require.Equal(t, 1, n, "the loop holds the chord %v–%v once", a, b)
}

// blendFaces lists the body's faces carrying a kind(k) role.
func blendFaces(body *Body, kind string) []*Face {
	var out []*Face
	for _, f := range body.Faces() {
		for _, o := range f.Origins() {
			if o.producer == body.origin.producer && strings.HasPrefix(o.Role, kind+"(") {
				out = append(out, f)
			}
		}
	}
	return out
}

// TestBrepModifyEdgeFilletS1VerticalEdge rounds S1's edge along z at
// (x, y) = (0, 0) with r = 2: the receiver's x = 0 wall and y = 0 face are
// trimmed to the feet (0, 2) and (2, 0), both caps carry the arc about
// (2, 2), and one swept face is appended carrying fillet(7). The volume is
// 16000 − 180π less the corner's (4 − π)·20. Shown to fail with the (pl)
// trim deleted from rewrite (the record then did not pair) and with
// rewriteLoop inserting a nil connector for the trim (the y = 0 face's audit
// then read a nil segment), and with brepFaceBody's blend role deleted (no
// face carried fillet(7)).
func TestBrepModifyEdgeFilletS1VerticalEdge(t *testing.T) {
	t.Parallel()
	doc, s1 := internalCrossDrilled(t)
	out, err := s1.Fillet(t.Context(), edgeAt(routeEZ, r3.Vec{}), units.Millimeters(2))
	require.NoError(t, err)
	require.Error(t, doc.requireLive(s1), "the receiver is retired")
	bp := requireBrepResult(t, out, 8)
	lo, hi := piEnclosed(big.NewRat(15920, 1), big.NewRat(-160, 1))
	requireCoversInterval(t, out.volume, lo, hi)

	yFace := planarFaceAt(t, bp, r3.NewVec(0, -1, 0), 0)
	require.Contains(t, yFace.region.Outer.Segments,
		CurveSegment(LineSeg{Start: Point2{U: 2, V: 20}, End: Point2{U: 2, V: 0}, TStart: 0, TEnd: 1}),
		"the y = 0 face's x = 0 segment moves to x = 2")
	want := arcEnds{center: Point2{U: 2, V: 2}, ends: [2]Point2{{U: 0, V: 2}, {U: 2, V: 0}}}
	for _, z := range []float64{0, 20} {
		require.Equal(t, []arcEnds{want}, loopArcs(planarFaceAt(t, bp, routeEZ, z).region.Outer))
	}
	wall := bp.faces[7]
	require.Equal(t, "fillet", wall.blend)
	require.Equal(t, CurveSegment(ArcSeg{Center: Point2{U: 2, V: 2}, Start: Point2{U: 0, V: 2}, End: Point2{U: 2, V: 0}, TStart: 0, TEnd: 1}), wall.wall)
	faces := blendFaces(out, "fillet")
	require.Len(t, faces, 1)
	requireRoles(t, out, faces, "wall(7)", "fillet(7)")
	cyl, ok := faces[0].Surface().(Cylinder)
	require.True(t, ok)
	require.Equal(t, 2.0, cyl.Radius.Base())
}

// TestBrepModifyEdgeChamferB1VerticalEdge bevels B1's edge along z at
// (0, 0) with d = 2: every coordinate is a recorded float or a float sum, so
// the volume 14000 − 2·20 is Exact.
func TestBrepModifyEdgeChamferB1VerticalEdge(t *testing.T) {
	t.Parallel()
	_, b1 := internalSquareCrossHole(t)
	out, err := b1.Chamfer(t.Context(), edgeAt(routeEZ, r3.Vec{}), units.Millimeters(2))
	require.NoError(t, err)
	requireBrepResult(t, out, 11)
	require.Equal(t, Exact, out.volume.Exactness)
	require.Equal(t, 13960.0, out.volume.Value.Base())
	require.Len(t, blendFaces(out, "chamfer"), 1)
}

// TestBrepModifyEdgeFilletPocketCorners rounds the Pocket's four vertical
// edges, concave, with r = 2 in one call. The stacked receiver goes through
// its face view, the result is a brep of 11 + 4 faces, the top cap's hole
// loop and the floor's outer loop each carry four arcs about the same four
// centres, the volume gains 4·(4 − π)·5, and the concave-radius survey reads
// 2 mm. The floor lies on the outer side of the pocket's walls' material, so
// each blend face walks against the floor's connector. Shown to fail with
// blendFace's reversal deleted (the build then refused each blend face's
// junction with a pocket wall as two walls walking the same way).
func TestBrepModifyEdgeFilletPocketCorners(t *testing.T) {
	t.Parallel()
	doc, pocket := internalRouteEPocket(t)
	sel := Edges(ParallelTo(routeEZ), Concave()).Exactly(4)
	out, err := pocket.Fillet(t.Context(), sel, units.Millimeters(2))
	require.NoError(t, err)
	bp := requireBrepResult(t, out, 15)
	lo, hi := piEnclosed(big.NewRat(15080, 1), big.NewRat(-20, 1))
	requireCoversInterval(t, out.volume, lo, hi)

	centres := []Point2{{U: 12, V: 17}, {U: 28, V: 17}, {U: 28, V: 23}, {U: 12, V: 23}}
	centresOf := func(arcs []arcEnds) []Point2 {
		var out []Point2
		for _, a := range arcs {
			out = append(out, a.center)
		}
		return out
	}
	top := planarFaceAt(t, bp, routeEZ, 10)
	require.Len(t, top.region.Holes, 1)
	require.ElementsMatch(t, centres, centresOf(loopArcs(top.region.Holes[0])))
	floor := planarFaceAt(t, bp, routeEZ, 5)
	require.ElementsMatch(t, centres, centresOf(loopArcs(floor.region.Outer)))
	require.Len(t, blendFaces(out, "fillet"), 4)

	rep, err := doc.Verify(t.Context(), WithConcaveRadius())
	require.NoError(t, err)
	br, err := rep.ForBody(out)
	require.NoError(t, err)
	require.Equal(t, ScalarMeasured, br.ConcaveRadius.Outcome)
	got := br.ConcaveRadius.Minimum.Measurement
	require.LessOrEqual(t, math.Abs(got.Value.Base()-2), got.Bound.Base())
}

// TestBrepModifyEdgeChamferCornerBoss bevels the Boss's inner vertical edge
// at (10, −10), convex, with d = 2. The plate top's reflex corner and the
// boss top's convex corner compute one chord between (10, −12) and
// (12, −10), and the volume 17500 − 2·15 is Exact over 9 + 1 faces. The
// plate top lies below the boss, so the chamfer face walks against its
// connector. Shown to fail with blendFace's reversal deleted (the volume then
// missed 17470).
func TestBrepModifyEdgeChamferCornerBoss(t *testing.T) {
	t.Parallel()
	_, boss := internalCornerBoss(t)
	out, err := boss.Chamfer(t.Context(), edgeAt(routeEZ, r3.NewVec(10, -10, 25)), units.Millimeters(2))
	require.NoError(t, err)
	bp := requireBrepResult(t, out, 10)
	require.Equal(t, Exact, out.volume.Exactness)
	require.Equal(t, 17470.0, out.volume.Value.Base())
	for _, z := range []float64{10, 25} {
		requireChord(t, planarFaceAt(t, bp, routeEZ, z).region.Outer, Point2{U: 10, V: -12}, Point2{U: 12, V: -10})
	}
}

// TestBrepModifyEdgeFilletConcavePlanarEdge rounds the inner vertical edge
// of an L-shaped boss standing flush on the plate's corner, r = 1. Both
// faces beside the edge are planar walls of the A1 brep recording the stack
// axis, so the receiver's Edge.IsConvex reads the junction's turn and reports
// concave; the blend fills it, adding (1 − π/4)·15 to 17125. Shown to fail
// with blendFace's reversal deleted (the volume then read 17423.9).
func TestBrepModifyEdgeFilletConcavePlanarEdge(t *testing.T) {
	t.Parallel()
	_, boss := internalFlushBossUnion(t, [][2]float64{{10, -20}, {20, -20}, {20, -10}, {15, -10}, {15, -15}, {10, -15}})
	sel := edgeAt(routeEZ, r3.NewVec(15, -15, 25))
	edges, err := sel.SelectEdges(boss)
	require.NoError(t, err)
	require.False(t, edges[0].IsConvex(), "the receiver reads a junction of two planar walls from the walk's turn")
	out, err := boss.Fillet(t.Context(), sel, units.Millimeters(1))
	require.NoError(t, err)
	requireBrepResult(t, out, 12)
	lo, hi := piEnclosed(big.NewRat(17140, 1), big.NewRat(-15, 4))
	requireCoversInterval(t, out.volume, lo, hi)
}

// pocketFloorEdges selects the Pocket's floor edges along dir, or all four
// when dir is zero: the edges of the face the stacked receiver names
// floor(0,0).
func pocketFloorEdges(t *testing.T, pocket *Body, dir r3.Vec, n int) *EdgeQuery {
	t.Helper()
	floor := FeatureRef{producer: pocket.origin.producer, Role: "floor(0,0)"}
	if dir == (r3.Vec{}) {
		return Edges(CreatedBy(floor)).Exactly(n)
	}
	return Edges(ParallelTo(dir), CreatedBy(floor)).Exactly(n)
}

// TestBrepModifyEdgeFilletPocketFloorEdges rounds the Pocket's two floor
// edges along x (y = 15 and y = 25 at z = 5), concave, with r = 1 in one
// call. Each edge ends on a pocket x-wall and runs along a pocket y-wall's
// rim, so all four pocket walls are restated as planes (§5.2), each with its
// frame normal outward: x = 10 (u along y, v along z) and x = 30 (u along z,
// v along y) each carry the two arcs about the corners' centres, and each
// y-wall's rim at the floor moves to z = 6. The result is a brep of
// 11 + 2 faces whose volume gains 2·(1 − π/4)·20. Shown to fail with
// restateRims skipped (the y = 15 wall then read SB8 beside the edge), and
// with findEndFaces refusing every swept straight wall (SB7 at the x = 10
// wall). Its vertical lines read the receiver's turn; shown to fail with
// restate recording no sweep (the pocket's corners then read convex).
func TestBrepModifyEdgeFilletPocketFloorEdges(t *testing.T) {
	t.Parallel()
	_, pocket := internalRouteEPocket(t)
	out, err := pocket.Fillet(t.Context(), pocketFloorEdges(t, pocket, routeEX, 2), units.Millimeters(1))
	require.NoError(t, err)
	bp := requireBrepResult(t, out, 13)
	lo, hi := piEnclosed(big.NewRat(15040, 1), big.NewRat(-10, 1))
	requireCoversInterval(t, out.volume, lo, hi)
	require.Len(t, blendFaces(out, "fillet"), 2)

	swept := 0
	for _, f := range bp.faces {
		if !f.planar() && f.blend == "" {
			swept++
		}
	}
	require.Equal(t, 4, swept, "only the plate's four outer walls stay swept")
	centres := func(f brepFace) []Point2 {
		require.True(t, f.outward)
		var out []Point2
		for _, a := range loopArcs(f.region.Outer) {
			out = append(out, a.center)
		}
		return out
	}
	require.ElementsMatch(t, []Point2{{U: 16, V: 6}, {U: 24, V: 6}}, centres(planarFaceAt(t, bp, routeEX, 10)))
	require.ElementsMatch(t, []Point2{{U: 6, V: 16}, {U: 6, V: 24}}, centres(planarFaceAt(t, bp, routeEX.Scale(-1), -30)))
	// The y = 15 wall's frame has u along z and v along x; the y = 25
	// wall's, u along x and v along z.
	requireChord(t, planarFaceAt(t, bp, r3.NewVec(0, 1, 0), 15).region.Outer, Point2{U: 6, V: 10}, Point2{U: 6, V: 30})
	requireChord(t, planarFaceAt(t, bp, r3.NewVec(0, -1, 0), -25).region.Outer, Point2{U: 10, V: 6}, Point2{U: 30, V: 6})
	floor := planarFaceAt(t, bp, routeEZ, 5).region.Outer
	requireChord(t, floor, Point2{U: 10, V: 16}, Point2{U: 30, V: 16})
	requireChord(t, floor, Point2{U: 10, V: 24}, Point2{U: 30, V: 24})

	// Every vertical line is a junction of two walls sweeping along z, the
	// pocket's four restated walls included, and reads the receiver's turn:
	// the plate's corners convex, the pocket's corners concave.
	turn := map[[2]float64]bool{}
	for ends, convex := range internalConvexityByEnds(t, pocket) {
		if ends[0].X == ends[1].X && ends[0].Y == ends[1].Y {
			turn[[2]float64{ends[0].X, ends[0].Y}] = convex
		}
	}
	require.Len(t, turn, 8)
	vertical := 0
	for ends, convex := range internalConvexityByEnds(t, out) {
		if ends[0].X != ends[1].X || ends[0].Y != ends[1].Y {
			continue
		}
		vertical++
		want, ok := turn[[2]float64{ends[0].X, ends[0].Y}]
		require.True(t, ok, "a vertical line at a receiver corner, got %v–%v", ends[0], ends[1])
		require.Equal(t, want, convex, "edge %v–%v", ends[0], ends[1])
	}
	require.Equal(t, 8, vertical)
	for xy, convex := range turn {
		inPocket := xy[0] > 0 && xy[0] < 40
		require.Equal(t, !inPocket, convex, "corner %v", xy)
	}
}

// TestBrepModifyEdgeChamferS1AlongX bevels S1's edge along x at
// (y, z) = (0, 0) with d = 2. Both end faces are the box's x-walls, swept
// along z, so both are restated as planes with their outward normals (§5.2)
// and each carries the chord between the feet y = 2 and z = 2; the volume is
// 16000 − 180π less the 2·40 the bevel removes. Shown to fail with
// findEndFaces refusing every swept straight wall (SB7 at the x = 0 wall),
// and with the edges not matched again on the restated record (the stale
// use pair then read the z = 0 cap's neighbour of the edge as SB8).
func TestBrepModifyEdgeChamferS1AlongX(t *testing.T) {
	t.Parallel()
	_, s1 := internalCrossDrilled(t)
	out, err := s1.Chamfer(t.Context(), edgeAt(routeEX, r3.Vec{}), units.Millimeters(2))
	require.NoError(t, err)
	bp := requireBrepResult(t, out, 8)
	lo, hi := piEnclosed(big.NewRat(15920, 1), big.NewRat(-180, 1))
	requireCoversInterval(t, out.volume, lo, hi)
	require.Len(t, blendFaces(out, "chamfer"), 1)
	// x = 0: u along z, v along y; x = 40: u along y, v along z.
	for _, f := range []brepFace{planarFaceAt(t, bp, routeEX.Scale(-1), 0), planarFaceAt(t, bp, routeEX, 40)} {
		require.True(t, f.outward)
		requireChord(t, f.region.Outer, Point2{U: 0, V: 2}, Point2{U: 2, V: 0})
	}
}

// internalSplitWallS1 is S1 with one record edit that keeps it closed: the
// x = 0 wall's side line at y = 0 is split at z = 10, and the y = 0 face's
// segment along it is split there to match.
func internalSplitWallS1(t *testing.T) *Body {
	t.Helper()
	doc, s1 := internalCrossDrilled(t)
	bp := s1.payload.(brepPayload)
	bp.faces = slices.Clone(bp.faces)
	wall, side := -1, -1
	for fi, f := range bp.faces {
		if l, ok := f.wall.(LineSeg); ok && l.Start == (Point2{U: 0, V: 20}) && l.End == (Point2{}) {
			wall = fi
		}
		if f.planar() && f.frame.N() == r3.NewVec(0, -1, 0) && f.z0 == 0 {
			side = fi
		}
	}
	require.GreaterOrEqual(t, wall, 0)
	require.GreaterOrEqual(t, side, 0)
	// The wall walks (0, 20) → (0, 0), so its side line at y = 0 is side1.
	bp.faces[wall].side1 = []brepSplit{{Z: 10}}
	region := *bp.faces[side].region
	var segs []CurveSegment
	for _, seg := range region.Outer.Segments {
		if seg == CurveSegment(LineSeg{Start: Point2{U: 0, V: 20}, End: Point2{}, TStart: 0, TEnd: 1}) {
			segs = append(segs,
				LineSeg{Start: Point2{U: 0, V: 20}, End: Point2{U: 0, V: 10}, TStart: 0, TEnd: 1},
				LineSeg{Start: Point2{U: 0, V: 10}, End: Point2{}, TStart: 0, TEnd: 1})
			continue
		}
		segs = append(segs, seg)
	}
	require.Len(t, segs, len(region.Outer.Segments)+1)
	region.Outer = LoopRecord{Segments: segs}
	bp.faces[side].region = &region
	body := internalCommitBrep(t, doc, bp)
	require.Equal(t, s1.volume, body.volume)
	return body
}

// TestBrepModifyEdgeRefusals pins Table SB's route E rows on receivers the
// public booleans build, and on internalSplitWallS1. A hole rim or a
// pocket floor loop is a complete loop route L's fillet arm builds
// (brep_loop_fillet_internal_test.go). Each refusal is
// ErrUnsupported naming its row, and leaves the receiver live and the
// document's body set unchanged. Shown to fail with each named gate deleted:
// the SB7 arm
// naming a blend face (the chained edge then read
// the chamfer face as an oblique wall), the SB9 foot comparison (the boss's
// front edge then failed to pair at closure), the planar faces' audit (the
// large chamfer then read the trimmed wall's S6), and brepgeom.Restate's
// split arm (both split-wall cases then failed to pair the restated record).
func TestBrepModifyEdgeRefusals(t *testing.T) {
	t.Parallel()
	chamferedS1 := func(t *testing.T) *Body {
		_, s1 := internalCrossDrilled(t)
		out, err := s1.Chamfer(t.Context(), edgeAt(routeEZ, r3.Vec{}), units.Millimeters(2))
		require.NoError(t, err)
		return out
	}
	s1 := func(t *testing.T) *Body {
		_, body := internalCrossDrilled(t)
		return body
	}
	boss := func(t *testing.T) *Body {
		_, body := internalCornerBoss(t)
		return body
	}
	split := internalSplitWallS1
	for _, tc := range []struct {
		name    string
		body    func(*testing.T) *Body
		chamfer bool
		sel     *EdgeQuery
		size    float64
		want    []string
	}{
		{"edge ending on a split wall", split, true, edgeAt(routeEX, r3.Vec{}), 1,
			[]string{"brep-modify SB7", "does not read as a plane", "side line is split"}},
		{"rim on a split wall", split, false, edgeAt(r3.NewVec(0, 1, 0), r3.Vec{}), 1,
			[]string{"brep-modify SB8", "holds it as a rim", "side line is split"}},
		{"edge ending on an earlier chamfer", chamferedS1, true, edgeAt(routeEX, r3.NewVec(40, 0, 0)), 1,
			[]string{"brep-modify SB7", "the chamfer face of an earlier call"}},
		{"boss front edge", boss, true, edgeAt(routeEZ, r3.NewVec(10, -20, 25)), 2,
			[]string{"brep-modify SB9", "(8, -20) against (12, -20)"}},
		{"setback past the wall", s1, true, edgeAt(routeEZ, r3.Vec{}), 25, []string{"consumed", "in brep face face("}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			body := tc.body(t)
			before := body.doc.Bodies()
			sel := tc.sel
			var err error
			if tc.chamfer {
				_, err = body.Chamfer(t.Context(), sel, units.Millimeters(tc.size))
			} else {
				_, err = body.Fillet(t.Context(), sel, units.Millimeters(tc.size))
			}
			require.ErrorIs(t, err, ErrUnsupported)
			for _, w := range tc.want {
				require.ErrorContains(t, err, w)
			}
			require.NoError(t, body.doc.requireLive(body))
			require.Equal(t, before, body.doc.Bodies())
		})
	}
}

// TestBrepModifyEdgeConsumersOnS1Fillet runs §8's consumers over S1's
// filleted result: Verify is Sound at the default tolerance; the mesh's
// occupied-volume proof covers 15920 − 160π; the fillet wall is a cylinder
// bounded by one loop alternating two arcs about its axis and two lines
// along it, the shape export's analytic STEP arm takes; the undercut survey
// hooks the fillet wall under a pull along (1, 1, 0) and does not list it
// under a pull along z, to which it runs parallel; and a translated copy
// carries the same volume, area and fillet role.
func TestBrepModifyEdgeConsumersOnS1Fillet(t *testing.T) {
	t.Parallel()
	doc, s1 := internalCrossDrilled(t)
	out, err := s1.Fillet(t.Context(), edgeAt(routeEZ, r3.Vec{}), units.Millimeters(2))
	require.NoError(t, err)
	lo, hi := piEnclosed(big.NewRat(15920, 1), big.NewRat(-160, 1))

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br, err := rep.ForBody(out)
	require.NoError(t, err)
	require.Equal(t, Sound, br.Status)

	tol := 0.05
	mesh, err := tessellateContext(t.Context(), out, units.Millimeters(tol), VerifyAll)
	require.NoError(t, err)
	require.True(t, mesh.symDiffOK)
	held := internalMeshVolumeRat(mesh)
	bound := new(big.Rat).SetFloat64(mesh.volSymDiff)
	require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, lo)).Cmp(bound), 0)
	require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, hi)).Cmp(bound), 0)

	fillet := blendFaces(out, "fillet")
	require.Len(t, fillet, 1)
	cyl, ok := fillet[0].Surface().(Cylinder)
	require.True(t, ok)
	require.Len(t, fillet[0].Loops(), 1)
	coedges := fillet[0].Loops()[0].CoEdges()
	require.Len(t, coedges, 4)
	arcs := 0
	for i, ce := range coedges {
		switch c := ce.Edge().Curve().(type) {
		case Arc3:
			arcs++
			require.True(t, c.Axis == cyl.Axis || c.Axis == cyl.Axis.Scale(-1))
			_, prevArc := coedges[(i+3)%4].Edge().Curve().(Arc3)
			require.False(t, prevArc, "arcs and lines alternate")
		case Line3:
			run := ce.Edge().End().Position().Value.Sub(ce.Edge().Start().Position().Value)
			require.Equal(t, r3.Vec{}, run.Cross(cyl.Axis))
		default:
			t.Fatalf("a fillet wall edge is %T", c)
		}
	}
	require.Equal(t, 2, arcs)

	hooked := func(pull r3.Vec) bool {
		rep, err := doc.Verify(t.Context(), WithPullDirection(pull))
		require.NoError(t, err)
		br, err := rep.ForBody(out)
		require.NoError(t, err)
		require.Equal(t, CoverageComplete, br.Undercut.Coverage)
		return slices.Contains(br.Undercut.Faces, fillet[0])
	}
	require.True(t, hooked(r3.NewVec(1, 1, 0)))
	require.False(t, hooked(routeEZ))

	move, err := r3.Translation(r3.NewVec(5, -7, 3))
	require.NoError(t, err)
	placed, err := out.PlacedCopy(t.Context(), move)
	require.NoError(t, err)
	require.Equal(t, out.volume, placed.volume)
	require.Equal(t, out.area, placed.area)
	requireRoles(t, placed, blendFaces(placed, "fillet"), "wall(7)", "fillet(7)")
}

// TestAuditTrimmedWall pins route E's S6 on a trimmed swept wall: claims
// from its two ends that reach its length refuse, and claims short of it
// pass. Shown to fail with the comparison's direction inverted.
func TestAuditTrimmedWall(t *testing.T) {
	t.Parallel()
	w := survey2d.SegmentWalk{StartU: 0, StartV: 0, EndU: 0, EndV: 20, Length: 20}
	require.NoError(t, auditTrimmedWall(w, 9, 10))
	require.ErrorIs(t, auditTrimmedWall(w, 10, 10), ErrUnsupported)
	require.ErrorIs(t, auditTrimmedWall(w, 25, 0), ErrUnsupported)
}

// TestReverseSegmentWalksACircleTheOtherWay pins docs/modify-general-design.md
// §3.3 step 5's rule for a whole circle: the same centre and radius, the CCW
// sense flipped and the range swapped, so reversing twice is the identity. A
// line and an arc keep their ends and swap their range as before.
func TestReverseSegmentWalksACircleTheOtherWay(t *testing.T) {
	t.Parallel()
	circle := CircleSeg{Center: Point2{U: 1, V: 2}, Radius: units.Millimeters(3), CCW: true, TStart: 0, TEnd: 1}
	got, ok := reverseSegment(circle).(CircleSeg)
	require.True(t, ok)
	require.Equal(t, circle.Center, got.Center)
	require.Equal(t, circle.Radius, got.Radius)
	require.False(t, got.CCW)
	require.Equal(t, [2]float64{1, 0}, [2]float64{got.TStart, got.TEnd})
	require.Equal(t, circle, reverseSegment(got))

	line := LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 5, V: 0}, TStart: 0, TEnd: 1}
	gotLine, ok := reverseSegment(line).(LineSeg)
	require.True(t, ok)
	require.Equal(t, [2]float64{1, 0}, [2]float64{gotLine.TStart, gotLine.TEnd})
	require.Equal(t, line.Start, gotLine.Start)
	require.Equal(t, line.End, gotLine.End)
}
