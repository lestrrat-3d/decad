package planar

import (
	"fmt"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// planarConvexExact is PlanarConvex without its float pre-test: every sign
// exact, one poll per triangle and vertex pair.
func planarConvexExact(s *PlanarSolid, poll func() error) (bool, error) {
	for _, tri := range s.Tris {
		a := s.Verts[tri[0]]
		normal := proof.DvCross(proof.DvSub(s.Verts[tri[1]], a), proof.DvSub(s.Verts[tri[2]], a))
		for _, v := range s.Verts {
			if err := poll(); err != nil {
				return false, err
			}
			if proof.DvDot(normal, proof.DvSub(v, a)).Sign() > 0 {
				return false, nil
			}
		}
	}
	return true, nil
}

// ringPoint lifts the plane point (x, y, 0) exactly.
func ringPoint(x, y float64) proof.DyV3 {
	return proof.DyV3{proof.MustDyOf(x), proof.MustDyOf(y), proof.DyZero()}
}

// regularRing returns n points on the circle of radius 10 about the z axis,
// counterclockwise seen from +z, each rounded to floats. jitter, when
// nonzero, scales each radius by 1 + jitter·u with u uniform in [−1, 1).
func regularRing(n int, jitter float64, rng *rand.Rand) []proof.DyV3 {
	ring := make([]proof.DyV3, n)
	for i := range ring {
		angle := 2 * math.Pi * float64(i) / float64(n)
		radius := 10.0
		if jitter != 0 {
			radius *= 1 + jitter*(2*rng.Float64()-1)
		}
		ring[i] = ringPoint(radius*math.Cos(angle), radius*math.Sin(angle))
	}
	return ring
}

// squareRing is the square [−1, 1]² with its four edge midpoints,
// counterclockwise, the midpoint of the edge at y = −1 moved to y = −1 + dy.
func squareRing(dy float64) []proof.DyV3 {
	return []proof.DyV3{ringPoint(-1, -1), ringPoint(0, -1+dy), ringPoint(1, -1), ringPoint(1, 0),
		ringPoint(1, 1), ringPoint(0, 1), ringPoint(-1, 1), ringPoint(-1, 0)}
}

// bipyramid returns the closed solid over ring, a polygon in z = 0 that is
// counterclockwise seen from +z and star-shaped about the origin, with apexes
// at (0, 0, ±h): one triangle per ring edge and apex, wound outward.
func bipyramid(ring []proof.DyV3, h proof.Dyadic) *PlanarSolid {
	n := len(ring)
	s := &PlanarSolid{Verts: append([]proof.DyV3{}, ring...)}
	top, bottom := n, n+1
	s.Verts = append(s.Verts, proof.DyV3{proof.DyZero(), proof.DyZero(), h},
		proof.DyV3{proof.DyZero(), proof.DyZero(), proof.DyNeg(h)})
	for j := range n {
		next := (j + 1) % n
		s.Tris = append(s.Tris, [3]int{j, next, top}, [3]int{next, j, bottom})
	}
	return s
}

// prism returns the closed solid over ring between z = 0 and z = h: each cap
// a fan from the ring's first point, which must see the whole ring, and each
// side a quad split in two.
func prism(ring []proof.DyV3, h proof.Dyadic) *PlanarSolid {
	n := len(ring)
	s := &PlanarSolid{Verts: append([]proof.DyV3{}, ring...)}
	for _, p := range ring {
		s.Verts = append(s.Verts, proof.DyV3{p[0], p[1], h})
	}
	for j := 1; j+1 < n; j++ {
		s.Tris = append(s.Tris, [3]int{0, j + 1, j}, [3]int{n, n + j, n + j + 1})
	}
	for j := range n {
		next := (j + 1) % n
		s.Tris = append(s.Tris, [3]int{j, next, n + next}, [3]int{j, n + next, n + j})
	}
	return s
}

// mapSolid returns s with every vertex replaced by f's image.
func mapSolid(s *PlanarSolid, f func(proof.DyV3) proof.DyV3) *PlanarSolid {
	out := &PlanarSolid{Verts: make([]proof.DyV3, len(s.Verts)), Tris: s.Tris}
	for i, v := range s.Verts {
		out.Verts[i] = f(v)
	}
	return out
}

type convexCase struct {
	name  string
	solid *PlanarSolid
}

// convexCases returns the differential test's solids: convex and non-convex
// bipyramids and prisms, vertices exactly on, an ulp in front of and an ulp
// behind a facet plane, each under exact power-of-two scalings across the
// float range and the range guard, an inexact translation and an inexact
// shear (both with determinant one, so convexity is unchanged). Under the race
// detector the prism and jitter rings have half as many points, and it keeps
// three of the jitters and the scalings at both ends of the float range and on
// both sides of the range guard. The regular bipyramid keeps its 64 points:
// at that size its skip count fails when the bound counts the coordinate
// rounding once instead of twice.
func convexCases() []convexCase {
	rng := rand.New(rand.NewPCG(101, 103))
	ten, one := proof.DyInt(10), proof.DyInt(1)
	prismSize, jitterSize := 40, 48
	if raceDetector {
		prismSize, jitterSize = 20, 24
	}
	bases := []convexCase{
		{"bipyramid64", bipyramid(regularRing(64, 0, nil), ten)},
		{fmt.Sprintf("prism%d", prismSize), prism(regularRing(prismSize, 0, nil), ten)},
		{"cube", prism([]proof.DyV3{ringPoint(-1, -1), ringPoint(1, -1), ringPoint(1, 1), ringPoint(-1, 1)},
			proof.DyInt(2))},
		{"lprism", prism([]proof.DyV3{ringPoint(0, 0), ringPoint(2, 0), ringPoint(2, 1), ringPoint(1, 1),
			ringPoint(1, 2), ringPoint(0, 2)}, one)},
		{"star", bipyramid(starRing(32), ten)},
		// The moved midpoint on its edge's line, one ulp inside the square
		// (in front of the neighbouring facets) and one ulp outside it.
		{"squareOn", bipyramid(squareRing(0), one)},
		{"squareIn", bipyramid(squareRing(0x1p-53), one)},
		{"squareOut", bipyramid(squareRing(-0x1p-52), one)},
		{"squareIn8", bipyramid(squareRing(0x1p-50), one)},
	}
	jitters := []float64{0x1p-10, 0x1p-30, 0x1p-45, 0x1p-50, 0x1p-52}
	if raceDetector {
		jitters = []float64{0x1p-10, 0x1p-50, 0x1p-52}
	}
	for _, jitter := range jitters {
		bases = append(bases, convexCase{fmt.Sprintf("jitter%g", jitter),
			bipyramid(regularRing(jitterSize, jitter, rng), ten)})
	}
	offset := proof.DyV3{proof.DyAdd(proof.MustDyOf(1e6), proof.DyShift(one, -60)),
		proof.DyAdd(proof.MustDyOf(-3e5), proof.DyShift(one, -70)), proof.MustDyOf(0.1)}
	shear := proof.MustDyOf(0.1)
	maps := []struct {
		name string
		f    func(proof.DyV3) proof.DyV3
	}{
		{"", func(v proof.DyV3) proof.DyV3 { return v }},
		{"+translate", func(v proof.DyV3) proof.DyV3 { return proof.DvAdd(v, offset) }},
		{"+shear", func(v proof.DyV3) proof.DyV3 {
			return proof.DyV3{proof.DyAdd(v[0], proof.DyMul(shear, v[1])),
				proof.DyAdd(v[1], proof.DyMul(shear, v[2])), v[2]}
		}},
	}
	scales := []int{-1100, -1000, -400, -340, -300, 200, 246, 247, 250, 251, 300, 340, 400, 1100}
	if raceDetector {
		scales = []int{-1100, 250, 251, 1100}
	}
	for _, k := range scales {
		maps = append(maps, struct {
			name string
			f    func(proof.DyV3) proof.DyV3
		}{fmt.Sprintf("*2^%d", k), func(v proof.DyV3) proof.DyV3 {
			return proof.DyV3{proof.DyShift(v[0], k), proof.DyShift(v[1], k), proof.DyShift(v[2], k)}
		}})
	}
	var out []convexCase
	for _, base := range bases {
		for _, m := range maps {
			out = append(out, convexCase{base.name + m.name, mapSolid(base.solid, m.f)})
		}
	}
	return out
}

// starRing alternates radii 10 and 6 around n points, a ring concave at every
// inner point.
func starRing(n int) []proof.DyV3 {
	ring := make([]proof.DyV3, n)
	for i := range ring {
		angle := 2 * math.Pi * float64(i) / float64(n)
		radius := 10.0
		if i%2 == 1 {
			radius = 6
		}
		ring[i] = ringPoint(radius*math.Cos(angle), radius*math.Sin(angle))
	}
	return ring
}

// TestPlanarConvexMatchesExact holds PlanarConvex to the all-exact loop over
// every case of convexCases: the same certificate and the same poll count.
// Every pair the pre-test skips has an exact sign at or below zero, checked
// over all pairs, not only those the certificate reaches before an early
// exit. On the regular bipyramid, translated or sheared, the pre-test decides
// every pair but each triangle's own three corners.
func TestPlanarConvexMatchesExact(t *testing.T) {
	t.Parallel()
	convex, nonConvex, skipped, deferred, guarded := 0, 0, 0, 0, 0
	regular := map[string]struct{}{"bipyramid64": {}, "bipyramid64+translate": {}, "bipyramid64+shear": {}}
	for _, c := range convexCases() {
		var audit int
		ok, err := CheckPlanarSolid(c.solid, countingPoll(&audit))
		require.NoError(t, err)
		require.True(t, ok, "premise: %s is a closed solid", c.name)

		var wantPolls, gotPolls int
		want, err := planarConvexExact(c.solid, countingPoll(&wantPolls))
		require.NoError(t, err)
		got, err := PlanarConvex(c.solid, countingPoll(&gotPolls))
		require.NoError(t, err)
		require.Equal(t, want, got, "%s", c.name)
		require.Equal(t, wantPolls, gotPolls, "%s", c.name)
		if want {
			convex++
		} else {
			nonConvex++
		}

		filter, filtered := newConvexFilter(c.solid.Verts)
		if !filtered {
			guarded++
			continue
		}
		solidSkips := 0
		for _, tri := range c.solid.Tris {
			a := c.solid.Verts[tri[0]]
			normal := proof.DvCross(proof.DvSub(c.solid.Verts[tri[1]], a), proof.DvSub(c.solid.Verts[tri[2]], a))
			plane := filter.plane(normal, tri[0])
			for v, point := range c.solid.Verts {
				if !plane.behind(&filter.coords[v]) {
					deferred++
					continue
				}
				solidSkips++
				sign := proof.DvDot(normal, proof.DvSub(point, a)).Sign()
				require.LessOrEqual(t, sign, 0, "%s: triangle %v, vertex %d", c.name, tri, v)
			}
		}
		skipped += solidSkips
		if _, ok := regular[c.name]; ok {
			require.Equal(t, len(c.solid.Tris)*(len(c.solid.Verts)-3), solidSkips, "%s", c.name)
		}
	}
	require.Positive(t, convex, "premise: some solids are convex")
	require.Positive(t, nonConvex, "premise: some solids are not convex")
	require.Positive(t, guarded, "premise: some solids fall outside the range guard")
	require.Positive(t, skipped, "premise: the pre-test skips some pairs")
	require.Positive(t, deferred, "premise: some pairs defer to the exact sign")
}

// convexSkips reports whether the pre-test over verts skips the exact sign of
// normal·(verts[v] − verts[anchor]).
func convexSkips(verts []proof.DyV3, normal proof.DyV3, anchor, v int) bool {
	filter, ok := newConvexFilter(verts)
	if !ok {
		return false
	}
	plane := filter.plane(normal, anchor)
	return plane.behind(&filter.coords[v])
}

// nearPlaneDyadic draws a coordinate in [−4, 4)·2^scale.
func nearPlaneDyadic(rng *rand.Rand, scale int) proof.Dyadic {
	return proof.DyShift(proof.MustDyOf(rng.Float64()*8-4), scale)
}

// TestConvexPlaneBehindIsSound holds every skip of the pre-test to an exact
// sign at or below zero over vertices drawn within rounding of a triangle's
// plane, so the exact sign takes both values: random triangles at scales from
// the subnormal products to the range guard, with float vertices and with
// vertices translated by an offset no float holds.
//
// Legs shown to fail (each removed in turn): convexDotEps set to zero, and the
// coordinate rounding term (round) left at zero; each makes the pre-test skip
// a vertex in front of the plane on some draws. Under the race detector it
// draws 1200 triangles instead of 6000.
func TestConvexPlaneBehindIsSound(t *testing.T) {
	t.Parallel()
	trials := 6000
	if raceDetector {
		trials = 1200
	}
	rng := rand.New(rand.NewPCG(107, 109))
	skipped, behind, front := 0, 0, 0
	for trial := range trials {
		scale := []int{0, 0, 20, -20, 120, 240, -300, -340}[trial%8]
		var verts []proof.DyV3
		for range 3 {
			verts = append(verts, proof.DyV3{nearPlaneDyadic(rng, scale), nearPlaneDyadic(rng, scale),
				nearPlaneDyadic(rng, scale)})
		}
		a, b, c := verts[0], verts[1], verts[2]
		for range 8 {
			// a + α(b − a) + β(c − a), each coordinate rounded to a float.
			alpha, beta := rng.Float64()*2-.5, rng.Float64()*2-.5
			var p proof.DyV3
			for axis := range 3 {
				af, _ := a[axis].Float64()
				bf, _ := b[axis].Float64()
				cf, _ := c[axis].Float64()
				p[axis] = proof.MustDyOf(af + alpha*(bf-af) + beta*(cf-af))
			}
			verts = append(verts, p)
		}
		if trial%3 == 0 {
			// A translation no float holds: every coordinate rounds.
			offset := proof.DyAdd(proof.DyShift(proof.MustDyOf(1+rng.Float64()), scale+30),
				proof.DyShift(proof.DyInt(1), scale-40))
			for i := range verts {
				verts[i] = proof.DvAdd(verts[i], proof.DyV3{offset, offset, offset})
			}
		}
		a = verts[0]
		normal := proof.DvCross(proof.DvSub(verts[1], a), proof.DvSub(verts[2], a))
		for v := range verts {
			sign := proof.DvDot(normal, proof.DvSub(verts[v], a)).Sign()
			if sign > 0 {
				front++
			} else if sign < 0 {
				behind++
			}
			if !convexSkips(verts, normal, 0, v) {
				continue
			}
			skipped++
			require.LessOrEqual(t, sign, 0, "trial %d, vertex %d", trial, v)
		}
	}
	require.Positive(t, front, "premise: some vertices lie in front of their plane")
	require.Positive(t, behind, "premise: some vertices lie behind their plane")
	require.Positive(t, skipped, "premise: the pre-test skips some vertices")
}

// TestConvexFilterRangeGuard holds the range guard: a coordinate at 2²⁵⁰ is
// inside it, one at 2²⁵¹ or past the float range outside it, and a vertex in
// front of the plane whose float dot product overflows to −∞ is never skipped.
//
// Leg shown to fail: the guard removed, the pre-test skips the overflowing
// vertex (its products round to −∞, 1.5·2¹⁰²³ and 1.5·2¹⁰²³, which sum to −∞).
func TestConvexFilterRangeGuard(t *testing.T) {
	t.Parallel()
	// n = (2⁶⁰⁰, 2⁶⁰⁰, 2⁶⁰⁰) and v = (−1.25·2⁴²⁴, 1.5·2⁴²³, 1.5·2⁴²³): the
	// exact dot is 3·2¹⁰²³ − 1.25·2¹⁰²⁴ = 2¹⁰²², positive.
	n := proof.DyShift(proof.DyInt(1), 600)
	normal := proof.DyV3{n, n, n}
	v := proof.DyV3{proof.DyShift(proof.DyInt(-5), 422), proof.DyShift(proof.DyInt(3), 422),
		proof.DyShift(proof.DyInt(3), 422)}
	verts := []proof.DyV3{{}, v}
	require.Positive(t, proof.DvDot(normal, v).Sign(), "premise: the vertex lies in front")
	require.False(t, convexSkips(verts, normal, 0, 1))

	at := func(e int) []proof.DyV3 {
		return []proof.DyV3{{}, {proof.DyShift(proof.DyInt(1), e), proof.DyZero(), proof.DyZero()}}
	}
	_, ok := newConvexFilter(at(250))
	require.True(t, ok)
	_, ok = newConvexFilter(at(251))
	require.False(t, ok)
	_, ok = newConvexFilter(at(1100))
	require.False(t, ok)
}

// TestConvexPlaneUnderflowedNormal holds the normal's rounding term: a normal
// component that rounds to zero still counts its exact size against the
// vertex's offset on that axis.
//
// Leg shown to fail: the normal rounding term left at zero, the pre-test skips
// the vertex, since the float dot −2⁻¹⁰⁶⁶ lies below the remaining bound.
func TestConvexPlaneUnderflowedNormal(t *testing.T) {
	t.Parallel()
	normal := proof.DyV3{proof.DyShift(proof.DyInt(1), -1080), proof.DyShift(proof.DyInt(-1), -1066), proof.DyZero()}
	verts := []proof.DyV3{{}, {proof.DyShift(proof.DyInt(1), 20), proof.DyInt(1), proof.DyZero()}}
	require.Positive(t, proof.DvDot(normal, verts[1]).Sign(), "premise: the vertex lies in front")
	require.False(t, convexSkips(verts, normal, 0, 1))
}

// TestPlanarConvexStopsOnPoll holds the certificate to its poll: an error
// returned by the poll is returned unchanged, after as many polls as the
// all-exact loop charges before it.
func TestPlanarConvexStopsOnPoll(t *testing.T) {
	t.Parallel()
	solid := bipyramid(regularRing(16, 0, nil), proof.DyInt(10))
	stop := fmt.Errorf("stop")
	for _, limit := range []int{1, 17, 100, 300} {
		calls := 0
		poll := func() error {
			if calls++; calls == limit {
				return stop
			}
			return nil
		}
		_, err := PlanarConvex(solid, poll)
		require.ErrorIs(t, err, stop)
		require.Equal(t, limit, calls)
	}
}

// BenchmarkPlanarConvex times the convexity certificate over two convex
// solids: a bipyramid over an 800-gon, whose every vertex off a facet lies
// strictly behind it, and a prism over a 400-gon, whose fan-triangulated caps
// hold every cap vertex on the cap's plane.
func BenchmarkPlanarConvex(b *testing.B) {
	solids := []convexCase{
		{"bipyramid800", bipyramid(regularRing(800, 0, nil), proof.DyInt(10))},
		{"prism400", prism(regularRing(400, 0, nil), proof.DyInt(10))},
	}
	for _, c := range solids {
		var calls int
		poll := countingPoll(&calls)
		ok, err := CheckPlanarSolid(c.solid, poll)
		require.NoError(b, err)
		require.True(b, ok, "premise: %s is a closed solid", c.name)
		b.Run(c.name, func(b *testing.B) {
			for b.Loop() {
				convex, err := PlanarConvex(c.solid, poll)
				if err != nil || !convex {
					b.Fatalf("convex %v, err %v", convex, err)
				}
			}
		})
	}
}
