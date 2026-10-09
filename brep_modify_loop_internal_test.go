package decad

import (
	"fmt"
	"math"
	"math/big"
	"slices"
	"strings"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// internalRoundBoss is modify-general §1's P3: the 40×40×10 plate unioned
// with a Ø10 boss about (20, 20) standing from z = 10 to z = 25, a stacked
// receiver.
func internalRoundBoss(t *testing.T) *Body {
	t.Helper()
	doc := New()
	plate := internalBoxBody(t, doc, 0, 0, 40, 40, 10)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 10)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(20, 20)
	s.Fix(c)
	s.CreateCircle(c, 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	boss, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(15), Dir: Along})
	require.NoError(t, err)
	out, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	return out
}

// internalBlindPort is modify-general §1's P6c: the 60×40×30 enclosure with
// a 20×10 port 13 deep into its x = 60 wall, y ∈ [10, 30], z ∈ [10, 20].
func internalBlindPort(t *testing.T) *Body {
	t.Helper()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 60, 40, 30)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.YZ(), 47)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	r := s.CreateRectangle(10, 10, 30, 20)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(14), Dir: Along})
	require.NoError(t, err)
	out, err := Cut(t.Context(), box, tool)
	require.NoError(t, err)
	return out
}

// planarBodyFace finds the one planar face of body whose outward normal is n
// and whose plane passes through p.
func planarBodyFace(t *testing.T, body *Body, n, p r3.Vec) *Face {
	t.Helper()
	var found []*Face
	for _, f := range body.Faces() {
		pl, ok := f.Surface().(Plane)
		if !ok {
			continue
		}
		normal := pl.Frame.N()
		if f.reversed {
			normal = normal.Scale(-1)
		}
		if normal.Sub(n).Len() > 1e-12 || math.Abs(pl.Frame.Origin().Sub(p).Dot(n)) > 1e-9 {
			continue
		}
		found = append(found, f)
	}
	require.Len(t, found, 1, "one planar face faces %v through %v", n, p)
	return found[0]
}

// loopQuery selects exactly the edges of loop l: each line by its two ends
// and direction, each arc or circle by its ends among circular edges.
func loopQuery(l *Loop) *EdgeQuery {
	return edgesQuery(l.Edges())
}

// edgesQuery selects exactly edges, as loopQuery reads each.
func edgesQuery(edges []*Edge) *EdgeQuery {
	var q *EdgeQuery
	for _, e := range edges {
		a, b := e.Start().Position().Value, e.End().Position().Value
		preds := []EdgePredicate{EndpointAt(a), EndpointAt(b)}
		if _, line := e.Curve().(Line3); line {
			preds = append(preds, ParallelTo(b.Sub(a)))
		} else {
			preds = append(preds, Circular())
		}
		if q == nil {
			q = Edges(preds...)
			continue
		}
		q.Or(preds...)
	}
	return q.Exactly(len(edges))
}

// internalTrapezoidPocket is modify-general §9's bound fixture: the
// trapezoid (0, 0), (100, 0), (72, 45), (28, 45) extruded 10 along z, with a
// blind 20×20 pocket x ∈ [40, 60], y ∈ [10, 30] from z = 10 down to z = 5,
// a stacked receiver that reads as no prism. Its slanted sides run along
// (28, 45), of length 53, so their offset lines have rational coefficients,
// and its base corners turn 58°: the top loop's cap contour corners are
// rationals no float holds, (9/5, 1) at the origin among them. The design
// names a 60° corner; no float pair holds a 60° slope with a rational unit
// normal, and 58° is the nearest Pythagorean angle with small sides.
func internalTrapezoidPocket(t *testing.T) (*Document, *Body) {
	t.Helper()
	doc := New()
	prism := internalPolygonPrism(t, doc, [][2]float64{{0, 0}, {100, 0}, {72, 45}, {28, 45}}, 10)
	tool := internalOffsetBox(t, doc, 40, 10, 60, 30, 5, Distance{D: units.Millimeters(5), Dir: Along})
	out, err := Cut(t.Context(), prism, tool)
	require.NoError(t, err)
	_, ok := out.payload.(stackedPrismPayload)
	require.True(t, ok, "the trapezoid pocket is a stacked prism, got %T", out.payload)
	return doc, out
}

// loopPatches lists the body's faces carrying a chamferLoop(f,l,p) role.
func loopPatches(body *Body) []*Face {
	return blendFaces(body, "chamferLoop")
}

// requirePatchKinds asserts the body's band patches are the given count of
// planes and of cones, and nothing else.
func requirePatchKinds(t *testing.T, body *Body, planes, cones int) []*Face {
	t.Helper()
	patches := loopPatches(body)
	gotPlanes, gotCones := 0, 0
	for _, f := range patches {
		switch f.Surface().(type) {
		case Plane:
			gotPlanes++
		case Cone:
			gotCones++
		default:
			t.Fatalf("a band patch is a %T", f.Surface())
		}
	}
	require.Equal(t, [2]int{planes, cones}, [2]int{gotPlanes, gotCones}, "plane and cone patches")
	return patches
}

// chamferLoopOf chamfers loop li of body's planar face facing n through p by
// d millimetres, and requires a closed brep result carrying one band more
// than the receiver.
func chamferLoopOf(t *testing.T, body *Body, n, p r3.Vec, li int, d float64) (*Body, brepPayload) {
	t.Helper()
	before := 0
	if bp, ok := body.payload.(brepPayload); ok {
		before = len(bp.loopBands)
	}
	f := planarBodyFace(t, body, n, p)
	out, err := body.Chamfer(t.Context(), loopQuery(f.Loops()[li]), units.Millimeters(d))
	require.NoError(t, err)
	require.Error(t, body.doc.requireLive(body), "the receiver is retired")
	bp, ok := out.payload.(brepPayload)
	require.True(t, ok, "route L builds a brep, got %T", out.payload)
	require.Nil(t, bp.stack)
	require.Len(t, bp.loopBands, before+1)
	requireClosedTopology(t, out)
	return out, bp
}

// sweptFaceLevels lists the [z0, z1] of every swept face of the record
// whose wall is wall.
func sweptFaceLevels(bp brepPayload, wall CurveSegment) [][2]float64 {
	var out [][2]float64
	for _, f := range bp.faces {
		if !f.planar() && f.wall == wall {
			out = append(out, [2]float64{f.z0, f.z1})
		}
	}
	return out
}

// TestBrepLoopChamferPocketMouth chamfers the Pocket's mouth, the top face's
// hole loop, by 1.5 (modify-general §9, P2). The walls descend into the
// pocket, so the band removes a wedge: the four pocket walls end at
// z = 8.5, and the top face's hole loop is the mouth offset 1.5 into the top
// face's material. The mouth's corners are reflex corners of the top face,
// so the offset closes each with an arc of radius 1.5 about the corner and
// the band carries an apex cone there, as a prism's hole loop does
// (TestCapBlendPolygonalHoleChamferVolume): four Plane and four Cone
// patches, and a removed volume of 30d² + πd³/3. Shown to fail with
// brepBandMassOf's sign read from sigma alone (the volume then gained the
// band's void) and with loopBandKeys leaving out the side contour (the
// trimmed pocket walls' rims then paired with nothing and the build refused).
func TestBrepLoopChamferPocketMouth(t *testing.T) {
	t.Parallel()
	_, pocket := internalRouteEPocket(t)
	out, bp := chamferLoopOf(t, pocket, routeEZ, r3.NewVec(0, 0, 10), 1, 1.5)
	lo, hi := piEnclosed(big.NewRat(29865, 2), big.NewRat(-9, 8))
	requireCoversInterval(t, out.volume, lo, hi)
	require.Len(t, out.Faces(), len(bp.faces)+8)
	requirePatchKinds(t, out, 4, 4)

	top := planarFaceAt(t, bp, routeEZ, 10)
	require.Len(t, top.region.Holes, 1)
	require.Zero(t, top.delta, "an axis-aligned loop at a millimetre setback has an exact contour")
	centres := []Point2{{U: 10, V: 15}, {U: 30, V: 15}, {U: 30, V: 25}, {U: 10, V: 25}}
	var got []Point2
	for _, a := range loopArcs(top.region.Holes[0]) {
		got = append(got, a.center)
	}
	require.ElementsMatch(t, centres, got)
	requireChord(t, top.region.Holes[0], Point2{U: 8.5, V: 15}, Point2{U: 8.5, V: 25})
	requireChord(t, top.region.Holes[0], Point2{U: 10, V: 13.5}, Point2{U: 30, V: 13.5})
	pocketWalls := 0
	for _, f := range bp.faces {
		if !f.planar() && f.z0 == 5 {
			pocketWalls++
			require.Equal(t, 8.5, f.z1, "pocket wall %v", f.wall)
		}
	}
	require.Equal(t, 4, pocketWalls)
	require.Equal(t, loopBandRoles(bp), rolesOf(loopPatches(out)))
}

// loopBandRoles lists every chamferLoop(f,l,p) role a record's bands mint,
// patch counts read from the body.
func loopBandRoles(bp brepPayload) map[string]struct{} {
	out := map[string]struct{}{}
	for _, b := range bp.loopBands {
		out[fmt.Sprintf("chamferLoop(%d,%d,", b.face, b.loop)] = struct{}{}
	}
	return out
}

// rolesOf lists the chamferLoop role prefixes the faces carry, up to the
// patch index.
func rolesOf(faces []*Face) map[string]struct{} {
	out := map[string]struct{}{}
	for _, f := range faces {
		for _, o := range f.Origins() {
			if i := strings.LastIndex(o.Role, ","); strings.HasPrefix(o.Role, "chamferLoop(") && i > 0 {
				out[o.Role[:i+1]] = struct{}{}
			}
		}
	}
	return out
}

// TestBrepLoopChamferPlateTopLoop chamfers the Pocket's plate top loop, the
// top face's outer loop, by 1.5 (P2): four Plane patches mitred at the four
// convex corners, the plate's four swept walls trimmed to z = 8.5, and the
// volume 15000 − (80d² − 4d³/3), Exact since every coordinate is a float.
func TestBrepLoopChamferPlateTopLoop(t *testing.T) {
	t.Parallel()
	_, pocket := internalRouteEPocket(t)
	out, bp := chamferLoopOf(t, pocket, routeEZ, r3.NewVec(0, 0, 10), 0, 1.5)
	require.Equal(t, Exact, out.volume.Exactness)
	require.Equal(t, 14824.5, out.volume.Value.Base())
	requirePatchKinds(t, out, 4, 0)
	top := planarFaceAt(t, bp, routeEZ, 10)
	requireChord(t, top.region.Outer, Point2{U: 1.5, V: 1.5}, Point2{U: 38.5, V: 1.5})
	walls := 0
	for _, f := range bp.faces {
		if !f.planar() && f.z0 == 0 {
			walls++
			require.Equal(t, 8.5, f.z1, "plate wall %v", f.wall)
		}
	}
	require.Equal(t, 4, walls)
}

// TestBrepLoopChamferCrossDrilledBothLoops chamfers S1's top and bottom
// loops in one call by 1.5 (P1). The y walls are planes across y holding
// each loop's edge between two lines along z, so each moves its top and
// bottom segments to z = 18.5 and z = 1.5 (the (pl) trim); the x walls are
// swept along z and end at the two side levels (the (sw) trim). The volume is
// 16000 − 180π less two outer-loop bands of 60d² − 4d³/3. Chamfering the
// bottom loop in a second call on the top loop's result builds the same
// volume. Shown to fail with trimPlanar's move along the normal deleted (the
// y walls then kept their segments at the loops' levels and the record did
// not pair).
func TestBrepLoopChamferCrossDrilledBothLoops(t *testing.T) {
	t.Parallel()
	_, s1 := internalCrossDrilled(t)
	top := planarBodyFace(t, s1, routeEZ, r3.NewVec(0, 0, 20))
	bottom := planarBodyFace(t, s1, routeEZ.Scale(-1), r3.Vec{})
	q := loopQuery(top.Loops()[0])
	for _, e := range bottom.Loops()[0].Edges() {
		a, b := e.Start().Position().Value, e.End().Position().Value
		q.Or(EndpointAt(a), EndpointAt(b), ParallelTo(b.Sub(a)))
	}
	out, err := s1.Chamfer(t.Context(), q.Exactly(8), units.Millimeters(1.5))
	require.NoError(t, err)
	requireClosedTopology(t, out)
	lo, hi := piEnclosed(big.NewRat(15739, 1), big.NewRat(-180, 1))
	requireCoversInterval(t, out.volume, lo, hi)
	requirePatchKinds(t, out, 8, 0)
	bp := out.payload.(brepPayload)
	require.Len(t, bp.loopBands, 2)
	for _, wall := range []CurveSegment{
		LineSeg{Start: Point2{U: 40, V: 0}, End: Point2{U: 40, V: 20}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 0, V: 20}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	} {
		require.Equal(t, [][2]float64{{1.5, 18.5}}, sweptFaceLevels(bp, wall), "x wall %v", wall)
	}
	// The y = 0 wall's frame has u along x and v along z.
	yWall := planarFaceAt(t, bp, r3.NewVec(0, -1, 0), 0)
	requireChord(t, yWall.region.Outer, Point2{U: 40, V: 18.5}, Point2{U: 0, V: 18.5})
	requireChord(t, yWall.region.Outer, Point2{U: 0, V: 1.5}, Point2{U: 40, V: 1.5})
	requireChord(t, yWall.region.Outer, Point2{U: 0, V: 18.5}, Point2{U: 0, V: 1.5})

	_, again := internalCrossDrilled(t)
	first, _ := chamferLoopOf(t, again, routeEZ, r3.NewVec(0, 0, 20), 0, 1.5)
	second, _ := chamferLoopOf(t, first, routeEZ.Scale(-1), r3.Vec{}, 0, 1.5)
	require.Equal(t, out.volume, second.volume)
	requirePatchKinds(t, second, 8, 0)
}

// TestBrepLoopChamferBossRimAndRoot chamfers P3's boss by 1: its top rim, a
// whole circle on the boss top whose wall descends, and its root, the plate
// top's hole loop around the boss, whose wall rises off the plate. The rim's
// band is one Cone removing the ring of section ½ at radius 5 − 1/3,
// 16000 + 375π − 14π/3. The root's band is one Cone that fills the concave
// corner, adding the ring of section ½ at radius 5 + 1/3, 16000 + 375π +
// 16π/3: the plate top's hole widens to radius 6, the boss wall starts at
// z = 11, and the boss circle there reads concave as the plate's hole loop
// is walked. Shown to fail with attachBrepLoopBands' turn of a rising band's
// patches deleted (the root cone's outward normal then pointed into the
// material) and with brepBandMassOf's sign ignoring sigma (the root's volume
// then lost the fill).
func TestBrepLoopChamferBossRimAndRoot(t *testing.T) {
	t.Parallel()
	boss := internalRoundBoss(t)
	rim, _ := chamferLoopOf(t, boss, routeEZ, r3.NewVec(0, 0, 25), 0, 1)
	lo, hi := piEnclosed(big.NewRat(16000, 1), big.NewRat(1111, 3))
	requireCoversInterval(t, rim.volume, lo, hi)
	cones := requirePatchKinds(t, rim, 0, 1)
	n, err := cones[0].NormalAt(r3.NewVec(24.5, 20, 24.5))
	require.NoError(t, err)
	require.Positive(t, n.Value.Dot(r3.NewVec(1, 0, 1)), "the rim cone faces up and out")

	boss = internalRoundBoss(t)
	root, bp := chamferLoopOf(t, boss, routeEZ, r3.NewVec(0, 0, 10), 1, 1)
	lo, hi = piEnclosed(big.NewRat(16000, 1), big.NewRat(1141, 3))
	requireCoversInterval(t, root.volume, lo, hi)
	cones = requirePatchKinds(t, root, 0, 1)
	n, err = cones[0].NormalAt(r3.NewVec(25.5, 20, 10.5))
	require.NoError(t, err)
	require.Positive(t, n.Value.Dot(r3.NewVec(1, 0, 1)), "the root cone faces up and away from the boss")
	require.Equal(t, sigmaRise, bp.loopBands[0].sigma)

	plateTop := planarFaceAt(t, bp, routeEZ, 10)
	require.Len(t, plateTop.region.Holes, 1)
	hole, ok := plateTop.region.Holes[0].Segments[0].(CircleSeg)
	require.True(t, ok)
	require.Equal(t, 6.0, hole.Radius.Base())
	var bossWall CurveSegment
	for _, f := range bp.faces {
		if _, circle := f.wall.(CircleSeg); circle {
			bossWall = f.wall
		}
	}
	require.Equal(t, [][2]float64{{11, 25}}, sweptFaceLevels(bp, bossWall), "the boss wall")
	found := 0
	for _, e := range root.Edges() {
		c, ok := e.Curve().(Circle3)
		if ok && c.Center == r3.NewVec(20, 20, 11) {
			found++
			require.Equal(t, 5.0, c.Radius.Base())
			require.False(t, e.IsConvex(), "the root's trimmed circle")
		}
	}
	require.Equal(t, 1, found)
}

// sigmaRise is a band's sigma beside walls that rise off its face.
const sigmaRise = 1.0

// TestBrepLoopChamferBlindPortMouth chamfers P6c's port mouth, the x = 60
// wall's hole loop, by 1.5. The port is blind, so the mouth's walls are four
// faces swept along x ending at the port's floor; as for the Pocket's mouth
// the reflex corners carry apex cones, and the volume is 69400 − (30d² +
// πd³/3).
func TestBrepLoopChamferBlindPortMouth(t *testing.T) {
	t.Parallel()
	port := internalBlindPort(t)
	out, _ := chamferLoopOf(t, port, r3.NewVec(1, 0, 0), r3.NewVec(60, 0, 0), 1, 1.5)
	lo, hi := piEnclosed(big.NewRat(138665, 2), big.NewRat(-9, 8))
	requireCoversInterval(t, out.volume, lo, hi)
	requirePatchKinds(t, out, 4, 4)
}

// TestBrepLoopChamferLBracketTopLoop chamfers P7's top loop, the L section's
// outer loop at z = 30, by 1: six Plane patches and one apex Cone at the
// reflex corner (8, 8). The volume is 17280 − 72π less the band, 80d² less
// d³/3 at each of the five convex corners plus πd³/12 at the reflex one.
func TestBrepLoopChamferLBracketTopLoop(t *testing.T) {
	t.Parallel()
	bracket := internalLBracket(t)
	out, _ := chamferLoopOf(t, bracket, routeEZ, r3.NewVec(0, 0, 30), 0, 1)
	lo, hi := piEnclosed(big.NewRat(51605, 3), big.NewRat(-865, 12))
	requireCoversInterval(t, out.volume, lo, hi)
	cones := requirePatchKinds(t, out, 6, 1)
	cone, ok := cones[len(cones)-1].Surface().(Cone)
	if !ok {
		for _, f := range cones {
			if c, isCone := f.Surface().(Cone); isCone {
				cone, ok = c, true
			}
		}
	}
	require.True(t, ok)
	require.Equal(t, r3.NewVec(8, 8, 29), cone.Origin, "the apex sits at the reflex corner on the side level")
}

// TestBrepLoopChamferRoundedPlate chamfers P8 by 1: a hole rim on its y = 0
// wall, a whole circle whose band is one Cone, 15280 − 10π/3; and its top
// loop, four lines joining four fillet arcs at G1 joins, whose band is four
// Plane and four Cone patches, 15232 − 8π/3.
func TestBrepLoopChamferRoundedPlate(t *testing.T) {
	t.Parallel()
	plate := internalRoundedPlate(t)
	rim, _ := chamferLoopOf(t, plate, r3.NewVec(0, -1, 0), r3.Vec{}, 1, 1)
	lo, hi := piEnclosed(big.NewRat(15280, 1), big.NewRat(-10, 3))
	requireCoversInterval(t, rim.volume, lo, hi)
	requirePatchKinds(t, rim, 0, 1)

	plate = internalRoundedPlate(t)
	top, _ := chamferLoopOf(t, plate, routeEZ, r3.NewVec(0, 0, 20), 0, 1)
	lo, hi = piEnclosed(big.NewRat(15232, 1), big.NewRat(-8, 3))
	requireCoversInterval(t, top.volume, lo, hi)
	requirePatchKinds(t, top, 4, 4)
}

// TestBrepLoopChamferContourBound is modify-general §9's bound fixture. The
// trapezoid pocket's top loop, chamfered by 1, has a cap contour whose four
// corners are rationals no float holds, so the contour is a float solve whose
// displacement the top face's section displacement carries (§7). Every
// cap-level vertex's published bound encloses its exact rational distance to
// the corner the offset denotes, the meet of two offset lines n·p = c + 1
// with n each side's unit inward normal. Shown to fail with rewriteLoopFaces'
// charge of the contour displacement into F.delta deleted: the corner
// vertices then published a zero bound. The result carries that displacement,
// so a later chamfer of its bottom loop refuses with SB1.
func TestBrepLoopChamferContourBound(t *testing.T) {
	t.Parallel()
	_, body := internalTrapezoidPocket(t)
	out, bp := chamferLoopOf(t, body, routeEZ, r3.NewVec(0, 0, 10), 0, 1)
	require.Positive(t, planarFaceAt(t, bp, routeEZ, 10).delta)

	// Each side (p, q) walked counter-clockwise, its integer length, and its
	// offset line −dy·x + dx·y = −dy·px + dx·py + length.
	pts := [][2]int64{{0, 0}, {100, 0}, {72, 45}, {28, 45}}
	lengths := []int64{100, 53, 44, 53}
	type line struct{ a, b, c *big.Rat }
	lines := make([]line, len(pts))
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		dx, dy := q[0]-p[0], q[1]-p[1]
		lines[i] = line{big.NewRat(-dy, 1), big.NewRat(dx, 1), big.NewRat(-dy*p[0]+dx*p[1]+lengths[i], 1)}
	}
	vertices := map[*Vertex]struct{}{}
	for _, e := range out.Edges() {
		vertices[e.start], vertices[e.end] = struct{}{}, struct{}{}
	}
	for i := range lines {
		l0, l1 := lines[(i+len(lines)-1)%len(lines)], lines[i]
		det := new(big.Rat).Sub(new(big.Rat).Mul(l0.a, l1.b), new(big.Rat).Mul(l0.b, l1.a))
		x := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(l0.c, l1.b), new(big.Rat).Mul(l0.b, l1.c)), det)
		y := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(l0.a, l1.c), new(big.Rat).Mul(l0.c, l1.a)), det)
		fx, _ := x.Float64()
		fy, _ := y.Float64()
		var held *Vertex
		for v := range vertices {
			if v.position.Sub(r3.NewVec(fx, fy, 10)).Len() < 1e-9 {
				require.Nil(t, held, "one vertex sits at the contour corner (%v, %v)", fx, fy)
				held = v
			}
		}
		require.NotNil(t, held, "a vertex sits at the contour corner (%v, %v)", fx, fy)
		dist := new(big.Rat)
		for k, want := range [3]*big.Rat{x, y, big.NewRat(10, 1)} {
			got := new(big.Rat).SetFloat64([3]float64{held.position.X, held.position.Y, held.position.Z}[k])
			d := new(big.Rat).Sub(got, want)
			dist.Add(dist, d.Mul(d, d))
		}
		bound := new(big.Rat).SetFloat64(held.bound.Base())
		require.LessOrEqual(t, dist.Cmp(new(big.Rat).Mul(bound, bound)), 0,
			"the vertex at (%v, %v) is within its bound %v of the denoted corner", fx, fy, held.bound)
	}

	before := out.doc.Bodies()
	floor := planarBodyFace(t, out, routeEZ, r3.NewVec(0, 0, 5))
	_, err := out.Chamfer(t.Context(), loopQuery(floor.Loops()[0]), units.Millimeters(1))
	requireRefusesUnchanged(t, out, before, err, "brep-modify SB1")
}

// TestBrepLoopChamferRefusals pins Table SL and the shipped rows route L
// keeps (modify-general §4.4, §9). Each refusal names its row and leaves the
// receiver live and the document's body set unchanged: part of the Pocket's
// mouth is SL1 with reach SX4's wording; the Pocket's top loop with one of
// the plate's vertical edges is SL1, the vertical edge lying on no planar
// face's loop; S1's top loop with its y = 0 wall's outer loop is SL1, the two
// loops sharing an edge; P8's y = 0 wall's outer loop is SL2, the top face
// beside it continuing past the loop's vertex on a fillet arc; the Pocket's
// mouth at d = 5 is SX7, the band reaching the pocket's floor; a fillet of the
// mouth is SL3; WithAsymmetricChamfer on it is SX16; and P8's top loop at
// d = 3 is SX6 (ErrDegenerate), the fillet arcs' offsets reaching their
// centres. Shown to fail with each gate deleted in turn: admitLoops' sharing
// arm (S1's wall loop then read SL2 at its swept x wall), classifyLoop's (pl)
// neighbour test (P8's wall loop then read SL2 at its next face), and
// requireBandReach (the deep mouth then failed the record audit's empty sweep
// interval, ErrDegenerate); and with admitLoops' partial-loop wording
// replaced.
func TestBrepLoopChamferRefusals(t *testing.T) {
	t.Parallel()
	pocket := func(t *testing.T) *Body {
		_, body := internalRouteEPocket(t)
		return body
	}
	s1 := func(t *testing.T) *Body {
		_, body := internalCrossDrilled(t)
		return body
	}
	plate := func(t *testing.T) *Body {
		body := internalRoundedPlate(t)
		return body
	}
	mouth := func(t *testing.T, body *Body) []*Edge {
		return planarBodyFace(t, body, routeEZ, r3.NewVec(0, 0, 10)).Loops()[1].Edges()
	}
	for _, tc := range []struct {
		name   string
		body   func(*testing.T) *Body
		sel    func(*testing.T, *Body) []*Edge
		size   float64
		fillet bool
		asym   bool
		want   []string
	}{
		{"part of a loop", pocket, func(t *testing.T, b *Body) []*Edge { return mouth(t, b)[:3] }, 1.5, false, false,
			[]string{"modify-general SL1", "covers only part of a loop"}},
		{"a loop with a lateral edge", pocket, func(t *testing.T, b *Body) []*Edge {
			edges := planarBodyFace(t, b, routeEZ, r3.NewVec(0, 0, 10)).Loops()[0].Edges()
			lateral, err := edgeAt(routeEZ, r3.Vec{}).SelectEdges(b)
			require.NoError(t, err)
			return append(edges, lateral...)
		}, 1.5, false, false, []string{"modify-general SL1", "lies on no loop of a planar face"}},
		{"two loops sharing an edge", s1, func(t *testing.T, b *Body) []*Edge {
			edges := planarBodyFace(t, b, routeEZ, r3.NewVec(0, 0, 20)).Loops()[0].Edges()
			wall := planarBodyFace(t, b, r3.NewVec(0, -1, 0), r3.Vec{}).Loops()[0].Edges()
			for _, e := range wall {
				if !slices.Contains(edges, e) {
					edges = append(edges, e)
				}
			}
			return edges
		}, 1, false, false, []string{"modify-general SL1", "which share it"}},
		{"a neighbour continuing on an arc", plate, func(t *testing.T, b *Body) []*Edge {
			return planarBodyFace(t, b, r3.NewVec(0, -1, 0), r3.Vec{}).Loops()[0].Edges()
		}, 1, false, false, []string{"modify-general SL2", "not a straight line along the face's normal"}},
		{"a band reaching the floor", pocket, mouth, 5, false, false, []string{"modify-reach SX7", "reaches or passes the far end"}},
		{"a fillet of a loop", pocket, mouth, 1, true, false, []string{"modify-general SL3", "vertex-blend problem"}},
		{"an asymmetric chamfer", pocket, mouth, 1, false, true, []string{"modify-reach SX16"}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			body := tc.body(t)
			before := body.doc.Bodies()
			sel := edgesQuery(tc.sel(t, body))
			var err error
			switch {
			case tc.fillet:
				_, err = body.Fillet(t.Context(), sel, units.Millimeters(tc.size))
			case tc.asym:
				top := planarBodyFace(t, body, routeEZ, r3.NewVec(0, 0, 10))
				ref := Faces(FaceCreatedBy(top.Origins()[0])).Exactly(1)
				_, err = body.Chamfer(t.Context(), sel, units.Millimeters(tc.size), WithAsymmetricChamfer(ref, units.Millimeters(2)))
			default:
				_, err = body.Chamfer(t.Context(), sel, units.Millimeters(tc.size))
			}
			requireRefusesUnchanged(t, body, before, err, tc.want...)
		})
	}

	t.Run("a contour that vanishes", func(t *testing.T) {
		t.Parallel()
		body := plate(t)
		before := body.doc.Bodies()
		top := planarBodyFace(t, body, routeEZ, r3.NewVec(0, 0, 20))
		_, err := body.Chamfer(t.Context(), loopQuery(top.Loops()[0]), units.Millimeters(3))
		require.ErrorIs(t, err, ErrDegenerate)
		require.ErrorContains(t, err, "no regular cap contour")
		require.NoError(t, body.doc.requireLive(body))
		require.Equal(t, before, body.doc.Bodies())
	})
}

// TestBrepLoopChamferConsumers runs modify-general Table DG's readers over the
// Pocket's top-loop chamfer. Verify is Sound; the volume is Exact, the area
// encloses its closed form, and the box is the plate's own, Exact, the band
// lying inside its record's faces; a translated copy reproduces volume and
// area; Tessellate meshes it (tessellate_brep_band_internal_test.go owns the
// mesh's proofs); the undercut and concave-radius surveys decide; the
// clearance pairs with two boxes read disjoint and unmeasured, with no
// clearance row; and a further chamfer of the pocket's mouth on the result
// builds the volume of both bands. Shown to fail with addBrepFaces' band guard
// deleted (the clearance pairs then read measured gaps to the record's faces
// alone, missing the band patches).
func TestBrepLoopChamferConsumers(t *testing.T) {
	t.Parallel()
	doc, pocket := internalRouteEPocket(t)
	near := internalBoxBody(t, doc, 45, 0, 50, 40, 10)
	far := internalBoxBody(t, doc, 45, 50, 50, 60, 10)
	out, _ := chamferLoopOf(t, pocket, routeEZ, r3.NewVec(0, 0, 10), 0, 1.5)

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br, err := rep.ForBody(out)
	require.NoError(t, err)
	require.Equal(t, Sound, br.Status)

	// The area is the record's faces and the four 45° patches: the top
	// 37×37 less the 20×10 mouth, the plate walls 8.5 high, the bottom, the
	// pocket's walls and floor, and four trapezoids running 37 to 40 across
	// a slant of 1.5√2, so 4629 + 231√2 in all.
	require.Equal(t, Exact, out.volume.Exactness)
	require.Equal(t, 14824.5, out.volume.Value.Base())
	sqrt2Lo, _ := new(big.Rat).SetString("1.4142135623730950488")
	sqrt2Hi, _ := new(big.Rat).SetString("1.4142135623730950489")
	lo := new(big.Rat).Add(big.NewRat(4629, 1), new(big.Rat).Mul(big.NewRat(231, 1), sqrt2Lo))
	hi := new(big.Rat).Add(big.NewRat(4629, 1), new(big.Rat).Mul(big.NewRat(231, 1), sqrt2Hi))
	requireCoversInterval(t, out.area, lo, hi)
	require.Equal(t, Exact, out.bounds.Exactness)
	require.Equal(t, r3.Vec{}, out.bounds.Min)
	require.Equal(t, r3.NewVec(40, 40, 10), out.bounds.Max)

	move, err := r3.Translation(r3.NewVec(5, -7, 3))
	require.NoError(t, err)
	placed, err := out.PlacedCopy(t.Context(), move)
	require.NoError(t, err)
	require.Equal(t, out.volume, placed.volume)
	require.Equal(t, out.area, placed.area)
	require.Len(t, loopPatches(placed), 4)

	_, err = tessellateContext(t.Context(), out, units.Millimeters(0.05), VerifyAll)
	require.NoError(t, err)

	rep, err = doc.Verify(t.Context(), WithPullDirection(routeEZ), WithConcaveRadius(), WithClearances())
	require.NoError(t, err)
	br, err = rep.ForBody(out)
	require.NoError(t, err)
	require.Equal(t, CoverageComplete, br.Undercut.Coverage)
	require.Empty(t, br.Undercut.Faces, "no face of the top-loop chamfer opposes a pull along +z")
	require.Equal(t, ScalarAbsent, br.ConcaveRadius.Outcome)
	for _, row := range rep.Clearances {
		require.NotSame(t, out, row.A, "no clearance row reads the chamfered body")
		require.NotSame(t, out, row.B, "no clearance row reads the chamfered body")
	}
	undecided := map[*Body]struct{}{}
	for _, d := range rep.Diagnostics {
		if d.Code != DiagUndecidedClearance || d.Pair == nil {
			continue
		}
		switch out {
		case d.Pair.A:
			undecided[d.Pair.B] = struct{}{}
		case d.Pair.B:
			undecided[d.Pair.A] = struct{}{}
		}
	}
	require.Equal(t, map[*Body]struct{}{near: {}, far: {}}, undecided, "the chamfered body's pairs are disjoint and unmeasured")

	both, _ := chamferLoopOf(t, out, routeEZ, r3.NewVec(0, 0, 10), 1, 1.5)
	lo, hi = piEnclosed(big.NewRat(14757, 1), big.NewRat(-9, 8))
	requireCoversInterval(t, both.volume, lo, hi)
	requirePatchKinds(t, both, 8, 4)
}
