package decad

import (
	"math"
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin route S of docs/modify-general-design.md (§3, §9 S-1):
// the shell of a brep that reads as a prism cut by through tools, with one
// or both caps removed. Receivers are built through the public booleans, so
// the records are the real ones, and every volume is asserted as a bound
// relation around its closed form.

var (
	shellUp = r3.NewVec(0, 0, 1)
	shellY  = r3.NewVec(0, 1, 0)
	shellX  = r3.NewVec(1, 0, 0)
)

// internalAlongXTool extrudes a profile drawn on the YZ plane (u = y,
// v = z) moved to x = at, by depth to each side of it.
func internalAlongXTool(t *testing.T, doc *Document, at, depth float64, draw func(*sketch.Sketch)) *Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.YZ(), at)
	require.NoError(t, err)
	return internalClassBTool(t, doc, w, plane, depth, draw)
}

// internalEnclosure is §1's P6: the 60×40×30 box with a 20×10 port along x
// over y ∈ [10, 30], z ∈ [10, 20], through both x walls.
func internalEnclosure(t *testing.T) *Body {
	t.Helper()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 60, 40, 30)
	port := internalAlongXTool(t, doc, 30, 31, func(s *sketch.Sketch) {
		r := s.CreateRectangle(10, 10, 30, 20)
		s.Fix(r.A)
	})
	out, err := Cut(t.Context(), box, port)
	require.NoError(t, err)
	return out
}

// internalPolygonPrism extrudes the polygon pts on XY by h.
func internalPolygonPrism(t *testing.T, doc *Document, pts [][2]float64, h float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
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
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	return body
}

// internalAlongXHole is a Ø6 cylinder along x through (·, y, z), drawn at
// x = at and extruded by depth to each side.
func internalAlongXHole(t *testing.T, doc *Document, at, depth, y, z, r float64) *Body {
	t.Helper()
	return internalAlongXTool(t, doc, at, depth, func(s *sketch.Sketch) {
		c := s.CreatePoint(y, z)
		s.Fix(c)
		s.CreateCircle(c, r)
	})
}

// internalLBracket is §1's P7: the L section (0, 0), (40, 0), (40, 8),
// (8, 8), (8, 40), (0, 40) over z ∈ [0, 30], with a Ø6 hole along x through
// the leg x ∈ [0, 8] at (y, z) = (24, 15).
func internalLBracket(t *testing.T) *Body {
	t.Helper()
	doc := New()
	l := internalPolygonPrism(t, doc, [][2]float64{{0, 0}, {40, 0}, {40, 8}, {8, 8}, {8, 40}, {0, 40}}, 30)
	out, err := Cut(t.Context(), l, internalAlongXHole(t, doc, 4, 5, 24, 15, 3))
	require.NoError(t, err)
	return out
}

// internalRoundedPlate is §1's P8: the 40×20×20 box with its four vertical
// edges filleted r = 3, then drilled Ø6 along y through (20, ·, 10).
func internalRoundedPlate(t *testing.T) *Body {
	t.Helper()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	rounded, err := box.Fillet(t.Context(), Edges(ParallelTo(shellUp)).Exactly(4), units.Millimeters(3))
	require.NoError(t, err)
	out, err := Cut(t.Context(), rounded, internalDrillAlongY(t, doc, 20, 10, 3))
	require.NoError(t, err)
	return out
}

// internalDrilledBox is the 40×20×20 box cut by each Ø6 drill along y
// through (x, ·, z), one Cut per drill.
func internalDrilledBox(t *testing.T, drills ...[2]float64) *Body {
	t.Helper()
	doc := New()
	out := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	for _, d := range drills {
		var err error
		out, err = Cut(t.Context(), out, internalDrillAlongY(t, doc, d[0], d[1], 3))
		require.NoError(t, err)
	}
	return out
}

// internalBandedBox is P1 with a Ø4 hole along z through (8, 10), cut
// after the cross hole, so the prism along z holds a hole of its own.
func internalBandedBox(t *testing.T) (*Document, *Body) {
	t.Helper()
	doc, s1 := internalCrossDrilled(t)
	pin := internalCircleBodyAt(t, doc, 8, 10, 2, -1, 22)
	out, err := Cut(t.Context(), s1, pin)
	require.NoError(t, err)
	_, ok := out.payload.(brepPayload)
	require.True(t, ok, "the vertical hole cuts the brep through class B, got %T", out.payload)
	return doc, out
}

// internalCircleBodyAt extrudes a circle of radius r about (x, y) on the XY
// plane moved to z, by h along +z.
func internalCircleBodyAt(t *testing.T, doc *Document, x, y, r, z, h float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(x, y)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	return body
}

// shellFaceAt selects the one planar face facing dir at level along dir.
func shellFaceAt(t *testing.T, body *Body, dir r3.Vec, level float64) *FaceQuery {
	t.Helper()
	return Faces(shellFacePred(t, body, dir, level)).Exactly(1)
}

// shellFacePred is the predicate naming the one planar face facing dir at
// level along dir.
func shellFacePred(t *testing.T, body *Body, dir r3.Vec, level float64) FacePredicate {
	t.Helper()
	faces, err := Faces(Facing(dir)).SelectFaces(body)
	require.NoError(t, err)
	var found *Face
	for _, f := range faces {
		if f.Loops()[0].CoEdges()[0].Start().Position().Value.Dot(dir) != level {
			continue
		}
		require.Nil(t, found, "one face faces %v at %g", dir, level)
		found = f
	}
	require.NotNil(t, found, "a face faces %v at %g", dir, level)
	return FaceCreatedBy(found.Origins()[0])
}

// shellRecordRole is the role of the record's one planar face across
// reference axis k at reference level.
func shellRecordRole(t *testing.T, bp brepPayload, k int, level float64) string {
	t.Helper()
	embeds, err := brepEmbeds(bp.faces)
	require.NoError(t, err)
	role := ""
	for fi, f := range bp.faces {
		if f.planar() && embeds[fi].Axis[2] == k && brepLevel(f, embeds[fi]) == level {
			require.Empty(t, role, "one planar face lies across axis %d at %g", k, level)
			role = f.role
		}
	}
	require.NotEmpty(t, role, "a planar face lies across axis %d at %g", k, level)
	return role
}

// requireThroughShell shells body removing sel at thickness th, requires a
// closed brep result of one lump whose volume covers a + b·π, and requires
// the receiver retired.
func requireThroughShell(t *testing.T, body *Body, sel FaceSelector, th units.Value, a, b *big.Rat) (*Body, brepPayload) {
	t.Helper()
	return requireThroughShellLumps(t, body, sel, th, a, b, 1)
}

// requireThroughShellLumps is requireThroughShell for a result of lumps
// lumps.
func requireThroughShellLumps(t *testing.T, body *Body, sel FaceSelector, th units.Value, a, b *big.Rat, lumps int) (*Body, brepPayload) {
	t.Helper()
	doc := body.doc
	result, err := body.Shell(t.Context(), sel, th)
	require.NoError(t, err)
	bp, ok := result.payload.(brepPayload)
	require.True(t, ok, "route S builds a brep, got %T", result.payload)
	lo, hi := piEnclosed(a, b)
	requireCoversInterval(t, result.volume, lo, hi)
	require.Len(t, result.Lumps(), lumps)
	for _, e := range result.Edges() {
		require.Len(t, e.Faces(), 2)
	}
	require.True(t, auditBoundary(result))
	require.ErrorIs(t, doc.requireLive(body), ErrRetiredBody, "the receiver is retired")
	return result, bp
}

// shellCylinders lists the body's cylindrical faces of radius r whose axis
// lies along dir.
func shellCylinders(body *Body, r float64, dir r3.Vec) []*Face {
	var out []*Face
	for _, f := range body.Faces() {
		c, ok := f.Surface().(Cylinder)
		if ok && c.Radius.Base() == r && (c.Axis == dir || c.Axis == dir.Scale(-1)) {
			out = append(out, f)
		}
	}
	return out
}

// shellWallLevels returns the sorted reference levels of every swept face of
// the record whose wall is a circle or arc of radius r.
func shellWallLevels(bp brepPayload, r float64) [][2]float64 {
	var out [][2]float64
	for _, f := range bp.faces {
		var radius float64
		switch w := f.wall.(type) {
		case CircleSeg:
			radius = w.Radius.Base()
		case ArcSeg:
			radius = math.Hypot(w.Start.U-w.Center.U, w.Start.V-w.Center.V)
		default:
			continue
		}
		if radius != r {
			continue
		}
		lo, hi := math.Abs(f.z0), math.Abs(f.z1)
		out = append(out, [2]float64{min(lo, hi), max(lo, hi)})
	}
	return out
}

// TestBrepShellThroughCutReadsP1 pins Table TC on §1's P1: it reads as the
// box along z, with the two y walls pierced by one tool along y whose one
// wall is the cylinder, and the x walls as walls of the prism; along x and y
// it reads as no through-cut record (along y the x walls are swept along z,
// which TC2 does not take). Shown to fail with readThroughTools' lower-rim
// test inverted (the tool then read from y = 20 down to y = 0, and the
// reading refused its pierced walls' sense).
func TestBrepShellThroughCutReadsP1(t *testing.T) {
	t.Parallel()
	_, s1 := internalCrossDrilled(t)
	bp := s1.payload.(brepPayload)
	embeds, err := brepEmbeds(bp.faces)
	require.NoError(t, err)
	for k := range 2 {
		_, _, ok, err := readThroughCut(t.Context(), bp, embeds, k)
		require.NoError(t, err)
		require.False(t, ok, "axis %d", k)
	}
	tc, reason, ok, err := readThroughCut(t.Context(), bp, embeds, 2)
	require.NoError(t, err)
	require.True(t, ok, reason)
	require.Len(t, tc.tools, 1)
	tool := tc.tools[0]
	require.Equal(t, 1, tool.j)
	require.Equal(t, [2]float64{0, 20}, [2]float64{tool.lo, tool.hi})
	require.Len(t, tool.walls, 1)
	require.Equal(t, throughTool, tc.kinds[tool.walls[0]])
	require.Len(t, tool.prism.profile.Outer.Segments, 1)
	circle, ok := tool.prism.profile.Outer.Segments[0].(CircleSeg)
	require.True(t, ok)
	require.True(t, circle.CCW, "the tool's section is an outer loop")
	require.Equal(t, 3.0, circle.Radius.Base())
	counts := map[throughFace]int{}
	for _, kind := range tc.kinds {
		counts[kind]++
	}
	require.Equal(t, map[throughFace]int{throughCap: 2, throughWall: 2, throughPierced: 2, throughTool: 1}, counts)
}

// TestBrepShellThroughCutP1 pins §9's first S-1 fixture: P1 shelled at 2 mm
// with its top removed is 13 faces (the receiver's 6 kept, the cavity's 6
// reversed, one rim), volume 5632 + 220π (the receiver 16000 − 180π less the
// cavity 36·16·18 − 25π·16). The rim is one planar face at z = 20 whose one
// hole is the eroded rectangle, the cavity's hole wall is a cylinder of
// radius 5 along y over y ∈ [2, 18], Verify reads the body Sound, and the
// mesh's occupied-volume proof covers the same figure. Shown to fail with
// throughCutRims keeping the cavity face in the removed plane (an edge then
// bounded three face uses and the build refused) and with the cavity faces
// left unreversed (the volume read the receiver's plus the cavity's).
func TestBrepShellThroughCutP1(t *testing.T) {
	t.Parallel()
	doc, s1 := internalCrossDrilled(t)
	before := len(doc.Bodies())
	result, bp := requireThroughShell(t, s1, Faces(Facing(shellUp)).Exactly(1), units.Millimeters(2),
		big.NewRat(5632, 1), big.NewRat(220, 1))
	require.Len(t, bp.faces, 13)
	require.Len(t, doc.Bodies(), before)

	rim := bp.faces[len(bp.faces)-1]
	require.True(t, rim.planar())
	require.Equal(t, 20.0, rim.z0)
	require.Len(t, rim.region.Holes, 1)
	var corners []Point2
	for _, seg := range rim.region.Holes[0].Segments {
		from, _, ok := brepgeom.NaturalLine(seg)
		require.True(t, ok)
		corners = append(corners, from)
	}
	require.ElementsMatch(t, []Point2{{U: 2, V: 2}, {U: 38, V: 2}, {U: 38, V: 18}, {U: 2, V: 18}}, corners)

	require.Len(t, shellCylinders(result, 5, shellY), 1)
	require.Equal(t, [][2]float64{{2, 18}}, shellWallLevels(bp, 5))

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br, err := rep.ForBody(result)
	require.NoError(t, err)
	require.Equal(t, Sound, br.Status)

	mesh, err := tessellateContext(t.Context(), result, units.Millimeters(0.05), VerifyAll)
	require.NoError(t, err)
	require.True(t, mesh.symDiffOK)
	lo, hi := piEnclosed(big.NewRat(5632, 1), big.NewRat(220, 1))
	held := internalMeshVolumeRat(mesh)
	bound := new(big.Rat).SetFloat64(mesh.volSymDiff)
	require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, lo)).Cmp(bound), 0)
	require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, hi)).Cmp(bound), 0)
}

// TestBrepShellThroughCutBothCaps pins P1 with both caps removed: the cavity
// keeps both levels, so it grows by the bottom slab 36·16·2 the dilated hole
// does not reach, and the volume is 5632 + 220π − 1152 = 4480 + 220π, with a
// rim at each level. A translated P1 shells to the same volume. Shown to
// fail with removedCaps reading the bottom as kept (the floor stayed, and the
// volume read 5632 + 220π) and with the rim assembled at the top alone (an
// edge then bounded one face use and the build refused).
func TestBrepShellThroughCutBothCaps(t *testing.T) {
	t.Parallel()
	both := Faces(NormalTo(shellUp)).Exactly(2)
	_, s1 := internalCrossDrilled(t)
	result, bp := requireThroughShell(t, s1, both, units.Millimeters(2), big.NewRat(4480, 1), big.NewRat(220, 1))
	require.Len(t, bp.faces, 12)
	var rims []float64
	for _, f := range bp.faces[len(bp.faces)-2:] {
		require.True(t, f.planar())
		require.Len(t, f.region.Holes, 1)
		rims = append(rims, f.z0)
	}
	slices.Sort(rims)
	require.Equal(t, []float64{0, 20}, rims)

	_, s1 = internalCrossDrilled(t)
	move, err := r3.Translation(r3.NewVec(5, -7, 3))
	require.NoError(t, err)
	placed, err := s1.Placed(t.Context(), move)
	require.NoError(t, err)
	moved, _ := requireThroughShell(t, placed, both, units.Millimeters(2), big.NewRat(4480, 1), big.NewRat(220, 1))
	require.Equal(t, result.volume, moved.volume)
	require.Equal(t, r3.NewVec(5, -7, 3), moved.bounds.Min)
}

// TestBrepShellThroughCutP6 pins P6 with its top removed at 2 mm: 23 faces,
// volume 21472 + 224π (the receiver 72000 − 12000 less the cavity
// 56·36·28 − 56·(320 + 4π)), and the cavity's port four planes and four
// quarter cylinders of radius 2 along x, the tool's convex corners dilated.
// Shown to fail with the tool dilated inward (offsetProfile's sense +1: the
// cavity's port shrank and the volume missed).
func TestBrepShellThroughCutP6(t *testing.T) {
	t.Parallel()
	p6 := internalEnclosure(t)
	result, bp := requireThroughShell(t, p6, shellFaceAt(t, p6, shellUp, 30), units.Millimeters(2),
		big.NewRat(21472, 1), big.NewRat(224, 1))
	require.Len(t, bp.faces, 23)
	require.Len(t, shellCylinders(result, 2, shellX), 4)
	require.Equal(t, [][2]float64{{2, 58}, {2, 58}, {2, 58}, {2, 58}}, shellWallLevels(bp, 2))
}

// TestBrepShellThroughCutP7 pins P7 with its top removed at 2 mm: volume
// 9552 + 56π (the receiver 17280 − 72π less the cavity
// 28·(276 − π) − 4·25π). The L's reflex corner erodes to a quarter cylinder
// of radius 2 along z over z ∈ [2, 30], and the dilated hole runs through
// the eroded leg's 4 mm, x ∈ [2, 6]. Shown to fail with the tool's section
// left as the hole loop it was read from, unreversed (its offset then eroded
// the hole instead of dilating it, and the volume missed).
func TestBrepShellThroughCutP7(t *testing.T) {
	t.Parallel()
	p7 := internalLBracket(t)
	result, bp := requireThroughShell(t, p7, Faces(Facing(shellUp)).Exactly(1), units.Millimeters(2),
		big.NewRat(9552, 1), big.NewRat(56, 1))
	require.Len(t, shellCylinders(result, 2, shellUp), 1)
	require.Equal(t, [][2]float64{{2, 30}}, shellWallLevels(bp, 2))
	require.Equal(t, [][2]float64{{2, 6}}, shellWallLevels(bp, 5))
}

// TestBrepShellThroughCutP8 pins P8 with its top removed at 2 mm: volume
// 4984 + 382π (the receiver 15280 less the cavity 18·(572 + π) − 16·25π),
// the filleted corners eroded to radius 1. Shown to fail with the eroded
// section built at the receiver's levels (no floor: the volume read
// 4984 + 382π − 2·(572 + π) and missed).
func TestBrepShellThroughCutP8(t *testing.T) {
	t.Parallel()
	p8 := internalRoundedPlate(t)
	result, _ := requireThroughShell(t, p8, Faces(Facing(shellUp)).Exactly(1), units.Millimeters(2),
		big.NewRat(4984, 1), big.NewRat(382, 1))
	require.Len(t, shellCylinders(result, 1, shellUp), 4)
}

// TestBrepShellThroughCutBands pins the rim's bands (§3.3 step 5): P1 with a
// Ø4 hole along z through (8, 10) reads as the box with a hole along z, cut
// by the tool along y. Removing the top at 2 mm, the rim at z = 20 is the
// outer region holding the eroded outer loop as its hole, and one band
// between the receiver's hole (radius 2) and the cavity's (radius 4);
// the volume is 5632 + 428π (the receiver 16000 − 260π less the cavity
// 18·(576 − 16π) − 16·25π). With both caps removed, the band appears at both
// levels, the band's wall around the receiver's hole is a lump of its own,
// and the volume is 4480 + 460π. Shown to fail with the band's two loops
// exchanged (S8 refused the band).
func TestBrepShellThroughCutBands(t *testing.T) {
	t.Parallel()
	t.Run("top", func(t *testing.T) {
		t.Parallel()
		_, body := internalBandedBox(t)
		_, bp := requireThroughShell(t, body, Faces(Facing(shellUp)).Exactly(1), units.Millimeters(2),
			big.NewRat(5632, 1), big.NewRat(428, 1))
		rim, band := bp.faces[len(bp.faces)-2], bp.faces[len(bp.faces)-1]
		require.Len(t, rim.region.Holes, 1)
		require.Len(t, band.region.Holes, 1)
		outer, ok := band.region.Outer.Segments[0].(CircleSeg)
		require.True(t, ok)
		hole, ok := band.region.Holes[0].Segments[0].(CircleSeg)
		require.True(t, ok)
		require.Equal(t, [2]float64{4, 2}, [2]float64{outer.Radius.Base(), hole.Radius.Base()})
		require.True(t, outer.CCW)
		require.False(t, hole.CCW)
	})
	t.Run("both", func(t *testing.T) {
		t.Parallel()
		_, body := internalBandedBox(t)
		_, bp := requireThroughShellLumps(t, body, Faces(NormalTo(shellUp)).Exactly(2), units.Millimeters(2),
			big.NewRat(4480, 1), big.NewRat(460, 1), 2)
		bands := 0
		for _, f := range bp.faces {
			if f.planar() && len(f.region.Outer.Segments) == 1 {
				bands++
			}
		}
		require.Equal(t, 2, bands)
	})
}

// atanEnclosed encloses atan(y) for 0 ≤ y < 1 between two consecutive
// partial sums of its alternating series, whose terms decrease.
func atanEnclosed(y *big.Rat, terms int) (*big.Rat, *big.Rat) {
	sum := new(big.Rat)
	pow := new(big.Rat).Set(y)
	y2 := new(big.Rat).Mul(y, y)
	var prev *big.Rat
	for n := range terms + 1 {
		prev = new(big.Rat).Set(sum)
		term := new(big.Rat).Quo(pow, big.NewRat(int64(2*n+1), 1))
		if n%2 == 0 {
			sum.Add(sum, term)
		} else {
			sum.Sub(sum, term)
		}
		pow.Mul(pow, y2)
	}
	if prev.Cmp(sum) > 0 {
		return sum, prev
	}
	return prev, sum
}

// sqrtEnclosed encloses √q between two neighbouring floats, each proven by
// squaring it exactly.
func sqrtEnclosed(t *testing.T, q *big.Rat) (*big.Rat, *big.Rat) {
	t.Helper()
	f, _ := q.Float64()
	s := math.Sqrt(f)
	lo, hi := new(big.Rat).SetFloat64(math.Nextafter(s, 0)), new(big.Rat).SetFloat64(math.Nextafter(s, 4*s))
	require.LessOrEqual(t, new(big.Rat).Mul(lo, lo).Cmp(q), 0)
	require.GreaterOrEqual(t, new(big.Rat).Mul(hi, hi).Cmp(q), 0)
	return lo, hi
}

// TestBrepShellThroughCutNotchedCap pins §9's hole 1.5 mm under the top on a
// 2 mm shell: the dilated hole (radius 5 about z = 15.5) reaches z = 20.5,
// past the cavity's top at z = 20, and the crossing reach builds the cut,
// splitting the cavity's top in two. The rim holds both pieces as holes, and
// the volume is 5632 + 220π less 16 times the circular segment of height
// 0.5 the cap cuts off, 25·acos(0.9) − 4.5·√4.75. Shown to fail with the rim
// taking the first cavity face in its plane only (the rim then held one
// hole).
func TestBrepShellThroughCutNotchedCap(t *testing.T) {
	t.Parallel()
	body := internalDrilledBox(t, [2]float64{20, 15.5})
	result, err := body.Shell(t.Context(), Faces(Facing(shellUp)).Exactly(1), units.Millimeters(2))
	require.NoError(t, err)
	bp, ok := result.payload.(brepPayload)
	require.True(t, ok)
	requireClosedTopology(t, result)
	rim := bp.faces[len(bp.faces)-1]
	require.Equal(t, 20.0, rim.z0)
	require.Len(t, rim.region.Holes, 2)

	// acos(0.9) = atan(√0.19 / 0.9).
	sLo, sHi := sqrtEnclosed(t, big.NewRat(19, 100))
	nine := big.NewRat(9, 10)
	aLo, _ := atanEnclosed(new(big.Rat).Quo(sLo, nine), 40)
	_, aHi := atanEnclosed(new(big.Rat).Quo(sHi, nine), 40)
	qLo, qHi := sqrtEnclosed(t, big.NewRat(19, 4))
	mul := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
	sub := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
	add := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
	segLo := sub(mul(big.NewRat(25, 1), aLo), mul(big.NewRat(9, 2), qHi))
	segHi := sub(mul(big.NewRat(25, 1), aHi), mul(big.NewRat(9, 2), qLo))
	base := big.NewRat(5632, 1)
	lo := sub(add(base, mul(big.NewRat(220, 1), proofbound.PiLower)), mul(big.NewRat(16, 1), segHi))
	hi := sub(add(base, mul(big.NewRat(220, 1), proofbound.PiUpper)), mul(big.NewRat(16, 1), segLo))
	requireCoversInterval(t, result.volume, lo, hi)
}

// shellInchClosedForm is P8 shelled at its top by t = 127/50 mm (0.1 in):
// the receiver 15280 less the cavity, the section (40 − 2t)(20 − 2t) with
// corners of radius 3 − t over 20 − t, less the hole of radius 3 + t over
// 20 − 2t. It returns a and b of a + b·π.
func shellInchClosedForm() (*big.Rat, *big.Rat) {
	th := big.NewRat(127, 50)
	r := func(x int64) *big.Rat { return big.NewRat(x, 1) }
	sub := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
	add := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
	mul := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
	two := mul(r(2), th)
	h := sub(r(20), th)
	corner := mul(sub(r(3), th), sub(r(3), th))
	hole := mul(add(r(3), th), add(r(3), th))
	a := sub(r(15280), mul(h, sub(mul(sub(r(40), two), sub(r(20), two)), mul(r(4), corner))))
	b := add(mul(new(big.Rat).Neg(h), corner), mul(hole, sub(r(20), two)))
	return a, b
}

// TestBrepShellThroughCutDisplacedOffset is §9's bound fixture. P1's
// eroded rectangle at a thickness of 0.1 in has float miters that are not
// exactly axis-aligned, so class B refuses its cut (SG6, pinned in
// TestBrepShellThroughCutRefusals); P8's rounded corners erode through G1
// joins that keep every line on its axis, so P8 builds at that thickness and
// its offsets carry a positive proven displacement. Every floor and top
// vertex of the cavity's wall at x = 40 − t, on y = 3 and y = 17, encloses
// its exact rational position, and the volume encloses the exact closed
// form; P8 placed 10⁶ mm along x encloses it too. Shown to fail with the
// cavity faces' offset displacement deleted (throughCut.cavity's delta set to
// zero): the floor vertices' bounds then missed their exact positions. The
// volume does not see that leg: its own rounding bound exceeds the cavity's
// displacement charge by four orders of magnitude.
func TestBrepShellThroughCutDisplacedOffset(t *testing.T) {
	t.Parallel()
	a, b := shellInchClosedForm()
	th := big.NewRat(127, 50)
	t.Run("vertices", func(t *testing.T) {
		t.Parallel()
		p8 := internalRoundedPlate(t)
		result, bp := requireThroughShell(t, p8, Faces(Facing(shellUp)).Exactly(1), units.Inches(0.1), a, b)
		x := new(big.Rat).Sub(big.NewRat(40, 1), th)
		checked := 0
		for _, v := range result.Vertices() {
			p := v.Position()
			if p.Value.X < 37 || p.Value.X > 38 || (p.Value.Y != 3 && p.Value.Y != 17) {
				continue
			}
			z := big.NewRat(20, 1)
			if p.Value.Z < 10 {
				z = th
			}
			requireCentroidCovers(t, p, [3]*big.Rat{x, new(big.Rat).SetFloat64(p.Value.Y), z})
			checked++
		}
		require.Equal(t, 4, checked)
		require.Positive(t, bp.sectionDelta())
	})
	t.Run("placed", func(t *testing.T) {
		t.Parallel()
		p8 := internalRoundedPlate(t)
		move, err := r3.Translation(r3.NewVec(1e6, 0, 0))
		require.NoError(t, err)
		placed, err := p8.Placed(t.Context(), move)
		require.NoError(t, err)
		requireThroughShell(t, placed, Faces(Facing(shellUp)).Exactly(1), units.Inches(0.1), a, b)
	})
}

// TestBrepShellThroughCutRefusals pins Table SG's refusals (§3.4, §9). Each
// leaves the receiver live and the document unchanged:
//
//   - a stacked pocket (P2) and a blind port (P6c) are SG3 naming the
//     floor, and a stacked boss (P3) SG3 naming the third planar face across
//     z, with SB10's text, since none reads as a prism either;
//   - P1's cylinder removed is SG4; P8's fillet cylinder, a curved wall whose
//     rim is no planar face, is SG5, and so is P7's y = 8 wall, whose end at
//     the L's reflex corner cuts back along its carrier into the material;
//   - a removed wall run keeps the side opening's own codes: P8's y = 0 wall
//     meets its fillets smoothly (shell-opening SO1), and P1's two x walls are
//     no connected run (SO6);
//   - P1 with a second hole 7 mm from the first is SG6: the dilated holes
//     meet, and the second cut meets the first's wall;
//   - P1 at 0.1 in is SG6: its eroded rectangle's miters are not exactly
//     axis-aligned, which class B requires;
//   - a U section whose slot is 1 mm wide, drilled through one arm, is SG6
//     through TC7: the other arm lies within t of the strip beyond the
//     pierced wall;
//   - Outward is SG1 and WithNoOpenings SG2.
//
// Shown to fail with requireStripsClear's call deleted (the U fixture then
// refused with class B's SG6 text instead of TC7's), with
// readThroughCutAnyAxis naming axis 0's reason always (P2, P3 and P6c then
// named no face), with removedFaces' straight-wall test deleted (the fillet
// cylinder then refused with SO1's text, its ends meeting the walls
// smoothly), and with openingThroughSection's reflex-end test deleted (P7's
// wall then refused with S11b's crossing text).
func TestBrepShellThroughCutRefusals(t *testing.T) {
	t.Parallel()
	refuses := func(t *testing.T, body *Body, sel FaceSelector, th units.Value, opts []ShellOption, want ...string) {
		t.Helper()
		before := body.doc.Bodies()
		_, err := body.Shell(t.Context(), sel, th, opts...)
		requireRefusesUnchanged(t, body, before, err, want...)
	}
	top := Faces(Facing(shellUp)).Exactly(1)
	mm2 := units.Millimeters(2)
	t.Run("P2", func(t *testing.T) {
		t.Parallel()
		_, pocket := internalRouteEPocket(t)
		bp, err := brepOfStacked(t.Context(), pocket.payload.(stackedPrismPayload))
		require.NoError(t, err)
		floor := shellRecordRole(t, bp, 2, 5)
		refuses(t, pocket, shellFaceAt(t, pocket, shellUp, 10), mm2, nil, "brep-modify SB10", "modify-general SG3", floor+" is a third planar face")
	})
	t.Run("P6c", func(t *testing.T) {
		t.Parallel()
		doc := New()
		box := internalBoxBody(t, doc, 0, 0, 60, 40, 30)
		port := internalAlongXTool(t, doc, -1, 11, func(s *sketch.Sketch) {
			r := s.CreateRectangle(10, 10, 30, 20)
			s.Fix(r.A)
		})
		p6c, err := Cut(t.Context(), box, port)
		require.NoError(t, err)
		bp, ok := p6c.payload.(brepPayload)
		require.True(t, ok, "the blind port is a brep, got %T", p6c.payload)
		floor := shellRecordRole(t, bp, 0, 10)
		refuses(t, p6c, shellFaceAt(t, p6c, shellUp, 30), mm2, nil, "modify-general SG3", "along reference axis 2, "+floor)
	})
	t.Run("P3", func(t *testing.T) {
		t.Parallel()
		plate, boss := internalBossOnPlate(t, 0, 10, 15)
		p3, err := Union(t.Context(), plate, boss)
		require.NoError(t, err)
		bp, err := brepOfStacked(t.Context(), p3.payload.(stackedPrismPayload))
		require.NoError(t, err)
		third := shellRecordRole(t, bp, 2, 10)
		refuses(t, p3, Faces(Facing(shellUp.Scale(-1))).Exactly(1), mm2, nil, "modify-general SG3", third+" is a third planar face")
	})
	t.Run("SG4 and SG5", func(t *testing.T) {
		t.Parallel()
		_, s1 := internalCrossDrilled(t)
		refuses(t, s1, Faces(Cylindrical()).Exactly(1), mm2, nil, "modify-general SG4")
		refuses(t, s1, Faces(NormalTo(shellX)).Exactly(2), mm2, nil, "shell-opening SO6")
		p8 := internalRoundedPlate(t)
		refuses(t, p8, Faces(Facing(shellY.Scale(-1))).Exactly(1), mm2, nil, "shell-opening SO1")
		var fillet *Face
		for _, f := range p8.Faces() {
			if c, ok := f.Surface().(Cylinder); ok && c.Axis.Cross(shellUp) == (r3.Vec{}) {
				fillet = f
				break
			}
		}
		require.NotNil(t, fillet)
		refuses(t, p8, Faces(FaceCreatedBy(fillet.Origins()[0])).Exactly(1), mm2, nil, "modify-general SG5", "straight wall along a section axis")
		p7 := internalLBracket(t)
		refuses(t, p7, shellFaceAt(t, p7, shellY, 8), mm2, nil, "modify-general SG5", "reflex corner")
	})
	t.Run("SG6 two holes", func(t *testing.T) {
		t.Parallel()
		body := internalDrilledBox(t, [2]float64{20, 10}, [2]float64{27, 10})
		refuses(t, body, top, mm2, nil, "modify-general SG6", "two dilated tools meet")
	})
	t.Run("SG6 inch miters", func(t *testing.T) {
		t.Parallel()
		_, s1 := internalCrossDrilled(t)
		refuses(t, s1, top, units.Inches(0.1), nil, "modify-general SG6")
	})
	t.Run("SG6 strip", func(t *testing.T) {
		t.Parallel()
		doc := New()
		u := internalPolygonPrism(t, doc, [][2]float64{{0, 0}, {40, 0}, {40, 20}, {11, 20}, {11, 10}, {10, 10}, {10, 20}, {0, 20}}, 20)
		body, err := Cut(t.Context(), u, internalAlongXHole(t, doc, 5, 5.5, 15, 10, 2))
		require.NoError(t, err)
		_, ok := body.payload.(brepPayload)
		require.True(t, ok, "the drilled arm is a brep, got %T", body.payload)
		refuses(t, body, top, mm2, nil, "modify-general SG6", "reaches past the wall it pierces")
	})
	t.Run("SG1 and SG2", func(t *testing.T) {
		t.Parallel()
		_, s1 := internalCrossDrilled(t)
		refuses(t, s1, top, mm2, []ShellOption{WithShellSense(Outward)}, "modify-general SG1")
		refuses(t, s1, nil, mm2, []ShellOption{WithNoOpenings()}, "modify-general SG2")
	})
}

// TestBrepShellThroughCutRimFalsifier pins SG7 (§3.3 step 5) on the banded
// box's record: a cavity whose top face lost its hole leaves the removed
// face's hole with no partner, and one holding a loop that is no hole of the
// eroded section partners nothing. Both refuse rather than build a rim. Shown
// to fail with throughCutRims' partner count check deleted (the first case
// then assembled a rim with no band) and with the hole match taking the
// first hole for any loop (the second case then refused with another SG7
// reason).
func TestBrepShellThroughCutRimFalsifier(t *testing.T) {
	t.Parallel()
	_, body := internalBandedBox(t)
	bp := body.payload.(brepPayload)
	embeds, err := brepEmbeds(bp.faces)
	require.NoError(t, err)
	tc, reason, ok, err := readThroughCut(t.Context(), bp, embeds, 2)
	require.NoError(t, err)
	require.True(t, ok, reason)
	budget := proofbound.NewWorkBudget(t.Context())
	eroded, err := offsetProfile(budget, tc.caps.section, 1, 2)
	require.NoError(t, err)
	dilated := make([]ProfileRecord, len(tc.tools))
	for i, tool := range tc.tools {
		dilated[i], err = offsetProfile(budget, tool.prism.profile, -1, 2)
		require.NoError(t, err)
	}
	topOnly := throughRemoval{top: true}
	cavity, err := tc.cavity(t.Context(), bp, eroded, dilated, topOnly, brepShellCall{tmm: 2}, 0)
	require.NoError(t, err)
	_, err = throughCutRims(t.Context(), budget, bp, tc, cavity, eroded, dilated, topOnly)
	require.NoError(t, err, "the undoctored cavity assembles")

	doctor := func(edit func(*ProfileRecord)) brepPayload {
		out := cavity
		out.faces = slices.Clone(cavity.faces)
		for i, f := range out.faces {
			if f.planar() && f.frame.N() == shellUp && f.z0 == 20 {
				region := *f.region
				region.Holes = slices.Clone(region.Holes)
				edit(&region)
				out.faces[i].region = &region
			}
		}
		return out
	}
	_, err = throughCutRims(t.Context(), budget, bp, tc, doctor(func(r *ProfileRecord) { r.Holes = nil }), eroded, dilated, topOnly)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "has no cavity hole to partner")
	require.ErrorContains(t, err, "modify-general SG7")
	_, err = throughCutRims(t.Context(), budget, bp, tc, doctor(func(r *ProfileRecord) {
		r.Holes = append(r.Holes, LoopRecord{Segments: []CurveSegment{CircleSeg{Center: Point2{U: 30, V: 10}, Radius: units.Millimeters(1), TEnd: 1}}})
	}), eroded, dilated, topOnly)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "neither its outer loop nor a hole of the eroded section")
}

// shellLoopCorners lists, in reference coordinates, the walked start of
// every line of one loop of record face fi.
func shellLoopCorners(t *testing.T, bp brepPayload, fi int, loop LoopRecord) [][3]float64 {
	t.Helper()
	embeds, err := brepEmbeds(bp.faces)
	require.NoError(t, err)
	out := make([][3]float64, 0, len(loop.Segments))
	for _, seg := range loop.Segments {
		from, _, ok := brepgeom.NaturalLine(seg)
		require.True(t, ok, "a rim loop holds lines only, got %T", seg)
		out = append(out, embeds[fi].Canon(from.U, from.V, bp.faces[fi].z0))
	}
	return out
}

// requireShellSound requires Verify to read the result Sound and its mesh's
// occupied-volume proof to cover a + b·π.
func requireShellSound(t *testing.T, result *Body, a, b *big.Rat) {
	t.Helper()
	rep, err := result.doc.Verify(t.Context())
	require.NoError(t, err)
	br, err := rep.ForBody(result)
	require.NoError(t, err)
	require.Equal(t, Sound, br.Status)
	mesh, err := tessellateContext(t.Context(), result, units.Millimeters(0.05), VerifyAll)
	require.NoError(t, err)
	require.True(t, mesh.symDiffOK)
	lo, hi := piEnclosed(a, b)
	held := internalMeshVolumeRat(mesh)
	bound := new(big.Rat).SetFloat64(mesh.volSymDiff)
	require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, lo)).Cmp(bound), 0)
	require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, hi)).Cmp(bound), 0)
}

// TestBrepShellThroughCutWallRun pins §9's S-2 fixtures (§3.2's second row):
// a removed wall run takes the side opening's cavity section C, each end cut
// by the rim rule of docs/shell-opening-design.md Table RO, and the rim at
// each removed face is that face less the cavity's trace.
//
//   - P6 with its y = 0 wall removed and both caps kept, a U-channel with its
//     port through both x walls: C is [2, 58] × [0, 38] (the right-angle rim
//     cuts at x = 2 and x = 58), the cavity C × [2, 28] less the dilated port
//     over x ∈ [2, 58], so the volume is 60000 − (2128·26 − 56·(320 + 4π)) =
//     22592 + 224π. The rim is the wall's rectangle holding the opening's
//     four corners (2, 0, 2), (58, 0, 2), (58, 0, 28), (2, 0, 28) as its hole.
//     §9 names P6 with an x wall removed; P6 reads as a prism along x, so route
//     P builds that as its own cup, and the y wall is the U-channel route S
//     takes.
//   - P1 with its y = 0 wall and its top removed: C is [2, 38] × [0, 18], the
//     cavity C × [2, 20] less the hole dilated to radius 5 over y ∈ [0, 18],
//     so the volume is 16000 − 180π − (648·18 − 450π) = 4336 + 270π. The two
//     rims reach each other's edge: the top's is the U [0, 40] × [0, 20] less
//     C, and the wall's the U [0, 40] × [0, 20] less [2, 38] × [2, 20] in
//     (x, z), with one band between the cavity's hole (radius 5) and the
//     receiver's (radius 3).
//   - P1 with its y = 0 wall and both caps removed: the cavity grows by the
//     floor slab, 4336 + 270π − 648·2 = 3040 + 270π, and the wall's rim falls
//     into two rectangles beside the band.
//   - P6 with its y = 0 and x = 0 walls removed, a run of two walls, caps kept:
//     C is [0, 58] × [0, 38], so the volume is
//     60000 − (2204·26 − 58·(320 + 4π)) = 21256 + 232π.
//
// Each result is one lump, every edge bounds two faces, Verify reads it Sound
// and its mesh's occupied-volume proof covers the figure. Shown to fail with
// throughSweptTrace reporting no trace (the U-channel kept the cavity's face
// on the removed carrier beside a rim holding no hole, and the cavity closed
// into a second lump), with throughRimRegions cancelling no piece (the
// touching rims read as R's loop holding a hole that meets it, and S7 refused
// them), and with openingThroughSection handing sideOpeningRegions one
// removed wall alone (the two-wall run then kept the x = 0 wall's cavity face
// whole, and SG7 found no cavity hole to partner the port there).
func TestBrepShellThroughCutWallRun(t *testing.T) {
	t.Parallel()
	mm2 := units.Millimeters(2)
	t.Run("P6 channel", func(t *testing.T) {
		t.Parallel()
		p6 := internalEnclosure(t)
		a, b := big.NewRat(22592, 1), big.NewRat(224, 1)
		result, bp := requireThroughShell(t, p6, shellFaceAt(t, p6, shellY.Scale(-1), 0), mm2, a, b)
		require.Len(t, bp.faces, 23)
		fi := len(bp.faces) - 1
		rim := bp.faces[fi]
		require.True(t, rim.planar())
		require.ElementsMatch(t, [][3]float64{{0, 0, 0}, {60, 0, 0}, {60, 0, 30}, {0, 0, 30}}, shellLoopCorners(t, bp, fi, rim.region.Outer))
		require.Len(t, rim.region.Holes, 1)
		require.ElementsMatch(t, [][3]float64{{2, 0, 2}, {58, 0, 2}, {58, 0, 28}, {2, 0, 28}}, shellLoopCorners(t, bp, fi, rim.region.Holes[0]))
		require.Len(t, shellCylinders(result, 2, shellX), 4)
		requireShellSound(t, result, a, b)
	})
	t.Run("P1 wall and top", func(t *testing.T) {
		t.Parallel()
		_, s1 := internalCrossDrilled(t)
		a, b := big.NewRat(4336, 1), big.NewRat(270, 1)
		sel := Faces(shellFacePred(t, s1, shellY.Scale(-1), 0)).Or(Facing(shellUp)).Exactly(2)
		result, bp := requireThroughShell(t, s1, sel, mm2, a, b)
		require.Len(t, bp.faces, 13)
		n := len(bp.faces)
		top, wall, band := bp.faces[n-3], bp.faces[n-2], bp.faces[n-1]
		require.Empty(t, top.region.Holes)
		require.ElementsMatch(t, [][3]float64{{0, 0, 20}, {2, 0, 20}, {2, 18, 20}, {38, 18, 20}, {38, 0, 20}, {40, 0, 20}, {40, 20, 20}, {0, 20, 20}},
			shellLoopCorners(t, bp, n-3, top.region.Outer))
		require.Empty(t, wall.region.Holes)
		require.ElementsMatch(t, [][3]float64{{0, 0, 0}, {40, 0, 0}, {40, 0, 20}, {38, 0, 20}, {38, 0, 2}, {2, 0, 2}, {2, 0, 20}, {0, 0, 20}},
			shellLoopCorners(t, bp, n-2, wall.region.Outer))
		outer, ok := band.region.Outer.Segments[0].(CircleSeg)
		require.True(t, ok)
		require.Len(t, band.region.Holes, 1)
		hole, ok := band.region.Holes[0].Segments[0].(CircleSeg)
		require.True(t, ok)
		require.Equal(t, [2]float64{5, 3}, [2]float64{outer.Radius.Base(), hole.Radius.Base()})
		requireShellSound(t, result, a, b)
	})
	t.Run("P1 wall and both caps", func(t *testing.T) {
		t.Parallel()
		_, s1 := internalCrossDrilled(t)
		a, b := big.NewRat(3040, 1), big.NewRat(270, 1)
		sel := Faces(shellFacePred(t, s1, shellY.Scale(-1), 0)).Or(NormalTo(shellUp)).Exactly(3)
		result, bp := requireThroughShell(t, s1, sel, mm2, a, b)
		n := len(bp.faces)
		for fi := n - 3; fi < n-1; fi++ {
			require.Empty(t, bp.faces[fi].region.Holes)
			require.Len(t, shellLoopCorners(t, bp, fi, bp.faces[fi].region.Outer), 4)
		}
		requireShellSound(t, result, a, b)
	})
	t.Run("P6 two walls", func(t *testing.T) {
		t.Parallel()
		p6 := internalEnclosure(t)
		a, b := big.NewRat(21256, 1), big.NewRat(232, 1)
		sel := Faces(shellFacePred(t, p6, shellY.Scale(-1), 0)).Or(shellFacePred(t, p6, shellX.Scale(-1), 0)).Exactly(2)
		result, _ := requireThroughShell(t, p6, sel, mm2, a, b)
		requireShellSound(t, result, a, b)
	})
}

// internalHalfRoundedPlate is the 40×20×20 box with its two vertical edges
// at y = 20 filleted r = 3, drilled Ø6 along y through (20, ·, 10): P8 with
// right-angle corners at y = 0, so removing the y = 0 wall cuts each end of
// the kept chain at a right angle (Table RO's exact pair) and every other
// corner of C is a G1 join that keeps its lines on their axes.
func internalHalfRoundedPlate(t *testing.T) *Body {
	t.Helper()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	at := func(x float64) EdgePredicate { return EndpointAt(r3.NewVec(x, 20, 0)) }
	rounded, err := box.Fillet(t.Context(), Edges(ParallelTo(shellUp), at(0)).Or(ParallelTo(shellUp), at(40)).Exactly(2), units.Millimeters(3))
	require.NoError(t, err)
	out, err := Cut(t.Context(), rounded, internalDrillAlongY(t, doc, 20, 10, 3))
	require.NoError(t, err)
	return out
}

// shellWallOpeningInchClosedForm is the half-rounded plate shelled at its
// y = 0 wall by t = 127/50 mm (0.1 in), both caps kept: the receiver
// 20·(782 + 9π/2) − 180π less the cavity, C — [t, 40 − t] × [0, 20 − t] with
// its two far corners rounded to 3 − t — over 20 − 2t, less the hole dilated
// to 3 + t over C's y-extent 20 − t. It returns a and b of a + b·π.
func shellWallOpeningInchClosedForm() (*big.Rat, *big.Rat) {
	th := big.NewRat(127, 50)
	r := func(x int64) *big.Rat { return big.NewRat(x, 1) }
	sub := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
	add := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
	mul := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
	two := mul(r(2), th)
	h := sub(r(20), two)
	corner := mul(sub(r(3), th), sub(r(3), th))
	hole := mul(add(r(3), th), add(r(3), th))
	section := sub(mul(sub(r(40), two), sub(r(20), th)), mul(r(2), corner))
	a := sub(r(15640), mul(h, section))
	b := add(sub(r(-90), mul(h, mul(big.NewRat(1, 2), corner))), mul(hole, sub(r(20), th)))
	return a, b
}

// TestBrepShellThroughCutWallOpeningDisplaced is S-2's bound fixture. The
// half-rounded plate shelled at its y = 0 wall at 0.1 in builds: the cuts at
// both ends are right-angle exact pairs and the far corners erode through G1
// joins, so every line of C stays on its axis, and the side opening's chain
// reach charges the cavity a positive displacement. Every cavity vertex at
// x = 40 − t on y = 0 and y = 17 encloses its exact rational position, and
// the volume encloses the exact closed form. Shown to fail with
// openingThroughSection's displacement deleted (the vertices' bounds then
// missed their exact positions). The volume does not see that leg: its own
// rounding bound exceeds the charge.
func TestBrepShellThroughCutWallOpeningDisplaced(t *testing.T) {
	t.Parallel()
	a, b := shellWallOpeningInchClosedForm()
	th := big.NewRat(127, 50)
	body := internalHalfRoundedPlate(t)
	result, bp := requireThroughShell(t, body, shellFaceAt(t, body, shellY.Scale(-1), 0), units.Inches(0.1), a, b)
	require.Positive(t, bp.sectionDelta())
	x := new(big.Rat).Sub(big.NewRat(40, 1), th)
	checked := 0
	for _, v := range result.Vertices() {
		p := v.Position()
		if p.Value.X < 37 || p.Value.X > 38 || (p.Value.Y != 0 && p.Value.Y != 17) {
			continue
		}
		z := new(big.Rat).Sub(big.NewRat(20, 1), th)
		if p.Value.Z < 10 {
			z = th
		}
		requireCentroidCovers(t, p, [3]*big.Rat{x, new(big.Rat).SetFloat64(p.Value.Y), z})
		checked++
	}
	require.Equal(t, 4, checked)
}

// TestBrepShellThroughCutWallRimCharge pins the wall rim's displacement
// (§3.3 step 5 over a removed wall): its region states the cavity's levels
// and the receiver's cap levels as in-plane coordinates, so it carries the
// largest of the cavity's section displacement and every cavity level's. On
// P1 with its y = 0 wall and top removed, a cavity built with a thickness
// conversion bound of 10⁻⁹ and no section displacement charges its floor's
// level by that bound; the wall's rim regions and band carry at least it,
// and the cap's rim the section displacement alone. In every real fixture
// the section displacement covers the levels', so no reading sees this leg.
// Shown to fail with the wall rim charged the cavity's section displacement
// alone (its delta then read zero).
func TestBrepShellThroughCutWallRimCharge(t *testing.T) {
	t.Parallel()
	_, s1 := internalCrossDrilled(t)
	bp := s1.payload.(brepPayload)
	embeds, err := brepEmbeds(bp.faces)
	require.NoError(t, err)
	tc, reason, ok, err := readThroughCut(t.Context(), bp, embeds, 2)
	require.NoError(t, err)
	require.True(t, ok, reason)
	removed, err := Faces(shellFacePred(t, s1, shellY.Scale(-1), 0)).Or(Facing(shellUp)).Exactly(2).SelectFaces(s1)
	require.NoError(t, err)
	rm, err := tc.removedFaces(s1, bp, removed)
	require.NoError(t, err)
	require.True(t, rm.top)
	require.Len(t, rm.walls, 1)

	budget := proofbound.NewWorkBudget(t.Context())
	call := brepShellCall{t: units.Millimeters(2), tmm: 2, tDelta: 1e-9}
	sec, err := tc.openingThroughSection(budget, bp, rm, call)
	require.NoError(t, err)
	dilated := make([]ProfileRecord, len(tc.tools))
	for i, tool := range tc.tools {
		dilated[i], err = offsetProfile(budget, tool.prism.profile, -1, 2)
		require.NoError(t, err)
	}
	cavity, err := tc.cavity(t.Context(), bp, sec.eroded, dilated, rm, call, 0)
	require.NoError(t, err)
	require.Zero(t, cavity.sectionDelta())
	out, err := throughCutRims(t.Context(), budget, bp, tc, cavity, sec.eroded, dilated, rm)
	require.NoError(t, err)
	n := len(out.faces)
	require.Zero(t, out.faces[n-3].delta, "the cap's rim")
	for _, f := range out.faces[n-2:] {
		require.GreaterOrEqual(t, f.delta, 1e-9, "the wall's rim")
	}
}
