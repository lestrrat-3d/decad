package pair_test

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// Every fixture coordinate is a small dyadic, so every predicate below is
// decided exactly and identically on every host.
//
// Legs shown to fail (each deleted or inverted in turn, fixture red, then
// restored):
//   - the transversal crossing certificate: TestClassifyPlanarCrossingBars;
//   - the full crossing scan, stopped at the first crossing instead:
//     TestClassifyPlanarRecordsCrossings names one edge, and the face-local
//     patch publishes the cavity and two-face corners of
//     planar_face_penetration_test.go;
//   - the matching coplanar overlap: TestClassifyPlanarMatchingCoplanarOverlap;
//   - the parity cast, skipped or inverted: TestClassifyPlanarNesting;
//   - either side's fan vertex sign under a separating plane, and the
//     guest's fan sign in a notch: the octahedron in
//     TestClassifyPlanarUnprovableOverlapsStayUndecided reads Touching;
//   - the support conditions under a separating plane, and the guest support
//     in a notch: the pitted cube in the same test reads Touching;
//   - the notch certificate: TestClassifyPlanarTouching's tray corner reads
//     Undecided;
//   - the gap enclosure's half-width: TestPlanarGapEnclosure's sqrt(3) gap
//     publishes a point that misses the true root.
//
// Legs that no fixture can turn red, and why:
//   - the open-cell direction check: every candidate normal is the normal
//     of a triangle holding the whole cell, or a cross product with the
//     cell's own edge direction, so it is already perpendicular to the cell;
//   - the folded-fan check: no embedded solid has two coplanar triangles
//     with opposed normals through one point;
//   - the empty-site guard: every zero-distance candidate records a site.

func dy(f float64) proof.Dyadic { return proof.MustDyOf(f) }

func vec(x, y, z float64) proof.DyV3 { return proof.DyV3{dy(x), dy(y), dy(z)} }

func noPoll() error { return nil }

// boxSolid is an axis box with outward-wound triangles.
func boxSolid(lo, hi [3]float64) pair.PlanarSolid {
	var s pair.PlanarSolid
	for i := range 8 {
		p := lo
		for axis := range 3 {
			if i>>axis&1 == 1 {
				p[axis] = hi[axis]
			}
		}
		s.Verts = append(s.Verts, vec(p[0], p[1], p[2]))
	}
	s.Tris = boxTris(0)
	return s
}

func boxTris(base int) [][3]int {
	tris := [][3]int{
		{0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5}, {0, 1, 5}, {0, 5, 4},
		{2, 6, 7}, {2, 7, 3}, {0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6},
	}
	for i := range tris {
		for j := range 3 {
			tris[i][j] += base
		}
	}
	return tris
}

// hollowSolid is a box with an inward-wound cavity shell.
func hollowSolid(lo, hi, cavityLo, cavityHi [3]float64) pair.PlanarSolid {
	outer := boxSolid(lo, hi)
	cavity := boxSolid(cavityLo, cavityHi)
	outer.Verts = append(outer.Verts, cavity.Verts...)
	for _, tri := range boxTris(8) {
		outer.Tris = append(outer.Tris, [3]int{tri[0], tri[2], tri[1]})
	}
	return outer
}

// hullSolid winds the given faces of a convex polytope outward, using the
// vertex centroid as an interior reference.
func hullSolid(points [][3]float64, faces [][3]int) pair.PlanarSolid {
	var s pair.PlanarSolid
	var c [3]float64
	for _, p := range points {
		s.Verts = append(s.Verts, vec(p[0], p[1], p[2]))
		for i := range 3 {
			c[i] += p[i] / float64(len(points))
		}
	}
	center := vec(c[0], c[1], c[2])
	for _, f := range faces {
		a, b, cc := s.Verts[f[0]], s.Verts[f[1]], s.Verts[f[2]]
		n := proof.DvCross(proof.DvSub(b, a), proof.DvSub(cc, a))
		if proof.DvDot(n, proof.DvSub(center, a)).Sign() > 0 {
			f[1], f[2] = f[2], f[1]
		}
		s.Tris = append(s.Tris, f)
	}
	return s
}

var tetraFaces = [][3]int{{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}}

// lSolid is the L-shaped prism [0,2]×[0,1] ∪ [0,1]×[0,2], z in [z0, z1].
func lSolid(z0, z1 float64) pair.PlanarSolid {
	section := [][2]float64{{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}}
	var s pair.PlanarSolid
	for _, z := range []float64{z0, z1} {
		for _, p := range section {
			s.Verts = append(s.Verts, vec(p[0], p[1], z))
		}
	}
	caps := [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 4, 5}}
	for _, c := range caps {
		s.Tris = append(s.Tris, [3]int{c[0], c[2], c[1]}, [3]int{6 + c[0], 6 + c[1], 6 + c[2]})
	}
	for i := range 6 {
		j := (i + 1) % 6
		s.Tris = append(s.Tris, [3]int{i, j, 6 + j}, [3]int{i, 6 + j, 6 + i})
	}
	return s
}

func classify(t *testing.T, a, b pair.PlanarSolid) pair.PlanarResult {
	t.Helper()
	for _, s := range []*pair.PlanarSolid{&a, &b} {
		ok, err := pair.CheckPlanarSolid(s, noPoll)
		require.NoError(t, err)
		require.True(t, ok)
	}
	forward, err := pair.ClassifyPlanar(&a, &b, noPoll)
	require.NoError(t, err)
	reversed, err := pair.ClassifyPlanar(&b, &a, noPoll)
	require.NoError(t, err)
	require.Equal(t, forward.Relation, reversed.Relation, "reversal keeps the relation")
	require.Equal(t, forward.Gap, reversed.Gap, "reversal keeps the gap")
	require.Len(t, reversed.Contacts, len(forward.Contacts))
	return forward
}

func TestCheckPlanarSolidAudits(t *testing.T) {
	box := boxSolid([3]float64{0, 0, 0}, [3]float64{1, 1, 1})
	ok, err := pair.CheckPlanarSolid(&box, noPoll)
	require.NoError(t, err)
	require.True(t, ok)

	open := box
	open.Tris = box.Tris[:11]
	ok, err = pair.CheckPlanarSolid(&open, noPoll)
	require.NoError(t, err)
	require.False(t, ok, "a missing triangle leaves unmatched directed edges")

	inverted := box
	inverted.Tris = nil
	for _, tri := range box.Tris {
		inverted.Tris = append(inverted.Tris, [3]int{tri[0], tri[2], tri[1]})
	}
	ok, err = pair.CheckPlanarSolid(&inverted, noPoll)
	require.NoError(t, err)
	require.False(t, ok, "an inward-wound box has negative signed volume")

	stop := errors.New("stop")
	_, err = pair.CheckPlanarSolid(&box, func() error { return stop })
	require.ErrorIs(t, err, stop)
}

func TestPlanarConvexCertificate(t *testing.T) {
	box := boxSolid([3]float64{0, 0, 0}, [3]float64{1, 2, 3})
	convex, err := pair.PlanarConvex(&box, noPoll)
	require.NoError(t, err)
	require.True(t, convex)

	l := lSolid(0, 1)
	convex, err = pair.PlanarConvex(&l, noPoll)
	require.NoError(t, err)
	require.False(t, convex, "the L section's reflex corner puts a vertex in front of a wall")

	hollow := hollowSolid([3]float64{0, 0, 0}, [3]float64{4, 4, 4}, [3]float64{1, 1, 1}, [3]float64{3, 3, 3})
	convex, err = pair.PlanarConvex(&hollow, noPoll)
	require.NoError(t, err)
	require.False(t, convex, "a cavity wall faces the outer vertices")
}

func TestClassifyPlanarSeparatedGap(t *testing.T) {
	a := boxSolid([3]float64{0, 0, 0}, [3]float64{10, 10, 10})
	b := boxSolid([3]float64{0, 0, 13}, [3]float64{10, 10, 23})
	result := classify(t, a, b)
	require.Equal(t, pair.Separated, result.Relation)
	require.Equal(t, 3.0, result.Gap.ValueMM)
	require.Zero(t, result.Gap.BoundMM, "an exactly representable root needs no bound")

	// A diagonal gap (3, 4, 0) reads its exact length 5 off an edge-edge pair.
	c := boxSolid([3]float64{13, 14, 0}, [3]float64{20, 20, 10})
	result = classify(t, a, c)
	require.Equal(t, pair.Separated, result.Relation)
	require.Equal(t, 5.0, result.Gap.ValueMM)
}

func TestPlanarGapEnclosure(t *testing.T) {
	// A box corner one unit off each axis from another box's corner: the gap
	// is sqrt(3), irrational, so the reading is an interval around it.
	a := boxSolid([3]float64{0, 0, 0}, [3]float64{1, 1, 1})
	b := boxSolid([3]float64{2, 2, 2}, [3]float64{3, 3, 3})
	result := classify(t, a, b)
	require.Equal(t, pair.Separated, result.Relation)
	lo, hi := result.Gap.ValueMM-result.Gap.BoundMM, result.Gap.ValueMM+result.Gap.BoundMM
	require.True(t, proof.DyCmp(proof.DyMul(dy(lo), dy(lo)), dy(3)) <= 0, "lower end squares at most 3")
	require.True(t, proof.DyCmp(proof.DyMul(dy(hi), dy(hi)), dy(3)) >= 0, "upper end squares at least 3")
	// The enclosure is a few ulps wide; 1e-12 is slack against FMA and
	// rounding differences, not a pinned figure.
	require.Less(t, result.Gap.BoundMM, 1e-12)
}

func TestClassifyPlanarTouching(t *testing.T) {
	box := boxSolid([3]float64{0, 0, 0}, [3]float64{4, 4, 4})

	// Vertex on a face: a tetrahedron standing on its apex.
	apex := hullSolid([][3]float64{{3, 1, 4}, {2, 0, 6}, {4, 0, 6}, {3, 2, 6}}, tetraFaces)
	result := classify(t, box, apex)
	require.Equal(t, pair.Touching, result.Relation)
	require.Equal(t, &pair.ScalarReading{}, result.Gap)
	require.Contains(t, result.Contacts, pair.PlanarContact{
		A: pair.PlanarFeature{Kind: pair.FeatureFacet, Facet: 10},
		B: pair.PlanarFeature{Kind: pair.FeatureVertex, Vertex: 0},
	}, "the apex lands inside the top face's first triangle")

	// Edge across edge: two wedges whose ridges cross at right angles.
	lower := hullSolid([][3]float64{{-1, 0, 0}, {1, 0, 0}, {0, -1, -1}, {0, 1, -1}}, tetraFaces)
	upper := hullSolid([][3]float64{{0, -1, 0}, {0, 1, 0}, {-1, 0, 1}, {1, 0, 1}}, tetraFaces)
	result = classify(t, lower, upper)
	require.Equal(t, pair.Touching, result.Relation)
	require.Equal(t, []pair.PlanarContact{{
		A: pair.PlanarFeature{Kind: pair.FeatureEdge, Edge: [2]int{0, 1}},
		B: pair.PlanarFeature{Kind: pair.FeatureEdge, Edge: [2]int{0, 1}},
	}}, result.Contacts)

	// Face on face: opposed coplanar patches.
	stacked := boxSolid([3]float64{1, 1, 4}, [3]float64{3, 3, 6})
	result = classify(t, box, stacked)
	require.Equal(t, pair.Touching, result.Relation)
	require.NotEmpty(t, result.Contacts)

	// A box in a tray corner: the floor and two walls meet the box's corner
	// and two of its edges at reflex edges of the tray.
	tray := trayCornerSolid()
	corner := boxSolid([3]float64{1, 1, 0}, [3]float64{2, 2, 1})
	result = classify(t, tray, corner)
	require.Equal(t, pair.Touching, result.Relation)
}

// trayCornerSolid is one closed tray corner: a [0,3]² floor slab for z in
// [-1, 0] under an L-shaped wall pair rising to z = 2, leaving the notch
// [1,3]×[1,3] open above the floor.
func trayCornerSolid() pair.PlanarSolid {
	pts := [][3]float64{
		{0, 0, -1}, {3, 0, -1}, {3, 3, -1}, {0, 3, -1}, // 0-3 bottom square
		{0, 0, 2}, {3, 0, 2}, {3, 1, 2}, {1, 1, 2}, {1, 3, 2}, {0, 3, 2}, // 4-9 L top
		{3, 1, 0}, {1, 1, 0}, {1, 3, 0}, {3, 3, 0}, // 10-13 notch floor
	}
	var s pair.PlanarSolid
	for _, p := range pts {
		s.Verts = append(s.Verts, vec(p[0], p[1], p[2]))
	}
	s.Tris = [][3]int{
		// bottom (z=-1), outward -z
		{0, 2, 1}, {0, 3, 2},
		// L top (z=2), outward +z
		{4, 5, 6}, {4, 6, 7}, {4, 7, 8}, {4, 8, 9},
		// notch floor (z=0), outward +z
		{11, 10, 13}, {11, 13, 12},
		// x=0 side
		{0, 4, 9}, {0, 9, 3},
		// y=0 side
		{0, 1, 5}, {0, 5, 4},
		// x=3 side: to z=0 over y in [0,3], above that over y in [0,1]
		{1, 2, 13}, {1, 13, 10}, {1, 10, 6}, {1, 6, 5},
		// y=3 side: to z=0 over x in [0,3], above that over x in [0,1]
		{3, 12, 13}, {3, 13, 2}, {3, 9, 8}, {3, 8, 12},
		// notch wall x=1 (outward +x), y in [1,3], z in [0,2]
		{11, 12, 8}, {11, 8, 7},
		// notch wall y=1 (outward +y), x in [1,3], z in [0,2]
		{10, 11, 7}, {10, 7, 6},
	}
	return s
}

func TestClassifyPlanarShallowCrossing(t *testing.T) {
	box := boxSolid([3]float64{0, 0, 0}, [3]float64{4, 4, 4})
	apex := hullSolid([][3]float64{{2, 2, 3.5}, {1, 1, 6}, {3, 1, 6}, {2, 3, 6}}, tetraFaces)
	result := classify(t, box, apex)
	require.Equal(t, pair.Overlapping, result.Relation)
	require.Nil(t, result.Gap)
}

func TestClassifyPlanarMatchingCoplanarOverlap(t *testing.T) {
	// Every crossing between these boxes passes through an edge or a corner,
	// so only the shared bottom patch with matching normals proves overlap.
	a := boxSolid([3]float64{0, 0, 0}, [3]float64{2, 2, 2})
	b := boxSolid([3]float64{1, 1, 0}, [3]float64{3, 3, 2})
	result := classify(t, a, b)
	require.Equal(t, pair.Overlapping, result.Relation)
}

func TestClassifyPlanarNesting(t *testing.T) {
	big := boxSolid([3]float64{0, 0, 0}, [3]float64{8, 8, 8})
	small := boxSolid([3]float64{2, 2, 2}, [3]float64{4, 4, 4})
	result := classify(t, big, small)
	require.Equal(t, pair.Overlapping, result.Relation)

	hollow := hollowSolid([3]float64{0, 0, 0}, [3]float64{8, 8, 8}, [3]float64{1, 1, 1}, [3]float64{7, 7, 7})
	result = classify(t, hollow, small)
	require.Equal(t, pair.Separated, result.Relation, "the cavity's two shells cross the ray twice")
	require.Equal(t, 1.0, result.Gap.ValueMM)

	inWall := boxSolid([3]float64{2, 2, 0.25}, [3]float64{4, 4, 0.75})
	result = classify(t, hollow, inWall)
	require.Equal(t, pair.Overlapping, result.Relation)
}

func TestClassifyPlanarRejectsSaddleTouch(t *testing.T) {
	// Two L prisms side by side on one base plane touch along a saddle: no
	// plane separates their cones there and neither is a notch, so the
	// relation stays undecided rather than guessed.
	a := lSolid(0, 1)
	b := boxSolid([3]float64{1, 1, 0}, [3]float64{2, 2, 1})
	result := classify(t, a, b)
	require.Equal(t, pair.Undecided, result.Relation)
	require.Equal(t, pair.AmbiguousFeature, result.Reason)
}

// prismAlongY extrudes a section given in (x, z), its first vertex seeing
// every other one, along y over [y0, y1], winding the result outward.
func prismAlongY(t *testing.T, section [][2]float64, y0, y1 float64) pair.PlanarSolid {
	t.Helper()
	n := len(section)
	var s pair.PlanarSolid
	for _, y := range []float64{y0, y1} {
		for _, p := range section {
			s.Verts = append(s.Verts, vec(p[0], y, p[1]))
		}
	}
	for i := 1; i+1 < n; i++ {
		s.Tris = append(s.Tris, [3]int{0, i, i + 1}, [3]int{n, n + i + 1, n + i})
	}
	for i := range n {
		j := (i + 1) % n
		s.Tris = append(s.Tris, [3]int{i, n + j, j}, [3]int{i, n + i, n + j})
	}
	if ok, err := pair.CheckPlanarSolid(&s, noPoll); err == nil && ok {
		return s
	}
	for i, tri := range s.Tris {
		s.Tris[i] = [3]int{tri[0], tri[2], tri[1]}
	}
	return s
}

func TestClassifyPlanarCrossingBars(t *testing.T) {
	// Two bars cross like a plus sign: no vertex of either lies in the
	// other, and no facet pair is coplanar, so only the transversal edge
	// crossing proves the overlap.
	a := boxSolid([3]float64{-4, -1, -1}, [3]float64{4, 1, 1})
	b := boxSolid([3]float64{-0.5, -4, -2}, [3]float64{0.5, 4, 2})
	result := classify(t, a, b)
	require.Equal(t, pair.Overlapping, result.Relation)
}

func TestClassifyPlanarUnprovableOverlapsStayUndecided(t *testing.T) {
	// Each fixture truly overlaps, yet every vertex of the inner body lies on
	// the outer body's boundary, no edge crosses a facet transversally and
	// no facet pair is coplanar: only the local cone test stands between
	// them and a false Touching.

	// An octahedron whose corners are the cube's face centers.
	cube := boxSolid([3]float64{0, 0, 0}, [3]float64{4, 4, 4})
	octahedron := hullSolid([][3]float64{
		{2, 2, 0}, {2, 2, 4}, {0, 2, 2}, {4, 2, 2}, {2, 0, 2}, {2, 4, 2},
	}, [][3]int{
		{0, 2, 4}, {0, 4, 3}, {0, 3, 5}, {0, 5, 2},
		{1, 2, 4}, {1, 4, 3}, {1, 3, 5}, {1, 5, 2},
	})
	result := classify(t, cube, octahedron)
	require.Equal(t, pair.Undecided, result.Relation)
	require.Equal(t, pair.AmbiguousFeature, result.Reason)

	// A wedge hung from the bottom edge of a V groove, its lower corners on
	// the block's side faces. At the groove edge the block's boundary lies
	// on one side of the wedge's slanted plane, but the block's material
	// fills both sides: no groove wall bounds its own fan.
	block := prismAlongY(t, [][2]float64{{0, 0}, {4, 0}, {4, 4}, {3, 4}, {2, 2}, {1, 4}, {0, 4}}, 0, 4)
	wedge := prismAlongY(t, [][2]float64{{0, 1}, {4, 1}, {2, 2}}, 1, 3)
	result = classify(t, block, wedge)
	require.Equal(t, pair.Undecided, result.Relation)
	require.Equal(t, pair.AmbiguousFeature, result.Reason)

	// An octahedron whose corners are the apexes of six pits dug into a
	// cube's faces. Each contact is a pit apex: the cube's material fills
	// everything around the pit, so its boundary lies on one side of a plane
	// while its material lies on both, and no pit wall bounds its own fan.
	pitted := pittedCubeSolid(t)
	inner := hullSolid([][3]float64{
		{4, 4, 2}, {4, 4, 6}, {2, 4, 4}, {6, 4, 4}, {4, 2, 4}, {4, 6, 4},
	}, [][3]int{
		{0, 2, 4}, {0, 4, 3}, {0, 3, 5}, {0, 5, 2},
		{1, 2, 4}, {1, 4, 3}, {1, 3, 5}, {1, 5, 2},
	})
	result = classify(t, pitted, inner)
	require.Equal(t, pair.Undecided, result.Relation)
	require.Equal(t, pair.AmbiguousFeature, result.Reason)
}

// pittedCubeSolid is the cube [0,8]³ with a square pyramidal pit in each
// face: rim [3,5]² on the face, apex 2 deep at the face's center.
func pittedCubeSolid(t *testing.T) pair.PlanarSolid {
	t.Helper()
	var s pair.PlanarSolid
	index := make(map[[3]float64]int)
	at := func(p [3]float64) int {
		if i, ok := index[p]; ok {
			return i
		}
		index[p] = len(s.Verts)
		s.Verts = append(s.Verts, vec(p[0], p[1], p[2]))
		return index[p]
	}
	outer := [4][2]float64{{0, 0}, {8, 0}, {8, 8}, {0, 8}}
	rim := [4][2]float64{{3, 3}, {5, 3}, {5, 5}, {3, 5}}
	for axis := range 3 {
		for _, side := range []int{0, 1} {
			// (u, v, outward) is right-handed for either side.
			u, v := (axis+1)%3, (axis+2)%3
			if side == 0 {
				u, v = v, u
			}
			point := func(uv [2]float64, depth float64) int {
				var p [3]float64
				p[u], p[v] = uv[0], uv[1]
				p[axis] = 8*float64(side) - depth
				if side == 0 {
					p[axis] = depth
				}
				return at(p)
			}
			apex := point([2]float64{4, 4}, 2)
			for i := range 4 {
				j := (i + 1) % 4
				s.Tris = append(s.Tris,
					[3]int{point(outer[i], 0), point(outer[j], 0), point(rim[j], 0)},
					[3]int{point(outer[i], 0), point(rim[j], 0), point(rim[i], 0)},
					[3]int{point(rim[i], 0), point(rim[j], 0), apex})
			}
		}
	}
	ok, err := pair.CheckPlanarSolid(&s, noPoll)
	require.NoError(t, err)
	require.True(t, ok)
	return s
}

func TestClassifyPlanarPollsAndStops(t *testing.T) {
	a := boxSolid([3]float64{0, 0, 0}, [3]float64{1, 1, 1})
	b := boxSolid([3]float64{0, 0, 3}, [3]float64{1, 1, 4})
	stop := errors.New("stop")
	calls := 0
	_, err := pair.ClassifyPlanar(&a, &b, func() error {
		calls++
		if calls > 50 {
			return stop
		}
		return nil
	})
	require.ErrorIs(t, err, stop)
}

// crossingSets gathers what the crossings name: the faces of tray's
// triangles, the box's edges, and the tray's edges.
func crossingSets(t *testing.T, result pair.PlanarResult, tray pair.PlanarSolid) (map[int]struct{},
	map[[2]int]struct{}, map[[2]int]struct{}) {
	t.Helper()
	faces := make(map[int]struct{})
	boxEdges := make(map[[2]int]struct{})
	trayEdges := make(map[[2]int]struct{})
	for _, crossing := range result.Crossings {
		box, onTray := crossing.A, crossing.B
		require.True(t, box.Edge != onTray.Edge, "an edge crosses a facet: %+v", crossing)
		for _, f := range onTray.Facets {
			faces[tray.Faces[f]] = struct{}{}
		}
		if box.Edge {
			boxEdges[box.Ends] = struct{}{}
		}
		if onTray.Edge {
			trayEdges[onTray.Ends] = struct{}{}
		}
	}
	return faces, boxEdges, trayEdges
}

func TestClassifyPlanarRecordsCrossings(t *testing.T) {
	tray := faceTray()
	// The corner sits 2⁻²⁰ below the floor, 2⁻²² off its diagonal y = x:
	// its edges along u and v cross the floor on either side of the
	// diagonal, which itself passes through the box's sunk corner.
	box := cornerDown([3]float64{3, 3 + 1.0/(1<<22), -tiny})
	result := classify(t, box, tray)
	require.Equal(t, pair.Overlapping, result.Relation)
	require.NotEmpty(t, result.Crossings)
	faces, boxEdges, trayEdges := crossingSets(t, result, tray)
	require.Equal(t, map[int]struct{}{trayFloor: {}}, faces)
	var floorTris []int
	for tri, face := range tray.Faces {
		if face == trayFloor {
			floorTris = append(floorTris, tri)
		}
	}
	named := make(map[int]struct{})
	for _, crossing := range result.Crossings {
		if !crossing.B.Edge {
			named[crossing.B.Facets[0]] = struct{}{}
		}
	}
	require.Equal(t, map[int]struct{}{floorTris[0]: {}, floorTris[1]: {}}, named, "both floor triangles")
	// Every crossing box edge leaves the sunk corner: its three edges, and
	// the diagonals of its three faces, which run inside one face each.
	creases := make(map[[2]int]struct{})
	for _, crossing := range result.Crossings {
		if part := crossing.A; part.Edge && box.Faces[part.Facets[0]] != box.Faces[part.Facets[1]] {
			creases[part.Ends] = struct{}{}
		}
	}
	require.Equal(t, map[[2]int]struct{}{{0, 1}: {}, {0, 2}: {}, {0, 4}: {}}, creases, "the corner's three edges")
	require.Len(t, boxEdges, 6)
	for edge := range boxEdges {
		require.Zero(t, edge[0], "edge %v leaves the corner", edge)
	}
	require.Equal(t, map[[2]int]struct{}{{12, 14}: {}}, trayEdges, "the floor's diagonal")

	reversed, err := pair.ClassifyPlanar(&tray, &box, noPoll)
	require.NoError(t, err)
	require.Len(t, reversed.Crossings, len(result.Crossings))
	for _, crossing := range reversed.Crossings {
		require.Contains(t, result.Crossings, pair.PlanarCrossing{A: crossing.B, B: crossing.A})
	}

	// Below the floor and past the wall x = 12, the corner crosses both.
	wall := parallelepiped([3]float64{12 + 1.0/1024, 0, -1.0 / 1024},
		[3]float64{-4, 0, 1}, [3]float64{0, 4, 1}, [3]float64{-1, -1, 4})
	faces, _, _ = crossingSets(t, classify(t, wall, tray), tray)
	require.Contains(t, faces, trayFloor)
	require.Contains(t, faces, 7, "the inner wall x = 12")
}
