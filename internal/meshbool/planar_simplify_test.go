package meshbool

import (
	"errors"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// prismMesh is the prism of the xy polygon poly (counter-clockwise) from z0 to
// z1, every corner a float, with one source id per side: the bottom is 0, the
// top 1 and wall k is 2+k. Its walls are exactly planar quads.
func prismMesh(t *testing.T, poly [][2]float64, z0, z1 float64) *BoolMesh {
	t.Helper()
	n := len(poly)
	var verts []r3.Vec
	for _, p := range poly {
		verts = append(verts, r3.Vec{X: p[0], Y: p[1], Z: z0})
	}
	for _, p := range poly {
		verts = append(verts, r3.Vec{X: p[0], Y: p[1], Z: z1})
	}
	var tris [][3]int
	var src []int
	add := func(s int, tri ...[3]int) {
		for _, tr := range tri {
			tris = append(tris, tr)
			src = append(src, s)
		}
	}
	// Fan the convex polygon for the caps.
	for i := 1; i+1 < n; i++ {
		add(100, [3]int{0, i + 1, i})
		add(101, [3]int{n, n + i, n + i + 1})
	}
	for k := range n {
		a, b := k, (k+1)%n
		add(102+k, [3]int{a, b, n + b}, [3]int{a, n + b, n + a})
	}
	bm, err := PrepContext(t.Context(), verts, tris, src)
	require.NoError(t, err)
	bm.SetVertexBounds(make([]float64, len(verts)))
	bm.Exact = make([]bool, len(tris))
	for i := range bm.Exact {
		bm.Exact[i] = true
	}
	return bm
}

// dovetailCut is a 10 mm cube minus a through slot whose walls slant by slope
// in y, so the cutter places vertices where B's walls cross the cube's top
// diagonals. A slope that is a power of two keeps every B-rep corner a float.
func dovetailCut(t *testing.T, slope float64) []KeptFacet {
	t.Helper()
	a := exactBoxMesh(t, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
	left := func(y float64) float64 { return 4 - slope*y }
	right := func(y float64) float64 { return 6 + slope*y }
	b := prismMesh(t, [][2]float64{{left(-1), -1}, {right(-1), -1}, {right(11), 11}, {left(11), 11}}, -1, 11)
	// The slot's far end passes through the cube's y = 10 face, so the first
	// operand's facets meet it in crossings only.
	kept, _, err := MeshBooleanStats(t.Context(), OpCut, a, b, NewContactMemo(a, b))
	require.NoError(t, err)
	return kept
}

func heldVolume(s *StitchedMesh) *big.Rat {
	var kept []KeptFacet
	for _, tri := range s.Tris {
		var f KeptFacet
		for k := range 3 {
			f.V[k] = proof.XptOf(s.Verts[tri[k]])
		}
		kept = append(kept, f)
	}
	return keptVolume(kept)
}

func maxOf(v []float64) float64 {
	m := 0.0
	for _, x := range v {
		m = max(m, x)
	}
	return m
}

// TestSimplifyPlanarTrapezoidCut pins §7 on a cube cut by a slanted slot: the
// plain stitch holds vertices the cutter placed on the cube's diagonals, with
// positive rounding; the simplified one holds none, rounds with no charge, and
// encloses the same exact volume.
//
// Leg shown to fail: leaving the planar faces unsimplified keeps Round > 0.
func TestSimplifyPlanarTrapezoidCut(t *testing.T) {
	kept := dovetailCut(t, 0.125)
	plain, err := StitchFacetsContext(t.Context(), kept)
	require.NoError(t, err)
	require.Positive(t, plain.Round, "the cutter's diagonal vertices round")
	simple, err := StitchFacetsWith(t.Context(), kept, StitchOptions{SimplifyPlanar: true})
	require.NoError(t, err)
	require.Zero(t, simple.Round)
	require.Zero(t, maxOf(simple.VertexBound))
	require.Less(t, len(simple.Verts), len(plain.Verts))
	// 1000 minus the slot: the width is 2 + y/4 over y in [0, 10], area 32.5.
	require.Zero(t, heldVolume(simple).Cmp(big.NewRat(675, 1)), "volume %v", heldVolume(simple))
}

// TestSimplifyPlanarNonFloatCorner pins that a slot whose wall meets the cube
// at a rational point leaves those faces alone: the stitch is the plain one.
func TestSimplifyPlanarNonFloatCorner(t *testing.T) {
	kept := dovetailCut(t, 0.1)
	plain, err := StitchFacetsContext(t.Context(), kept)
	require.NoError(t, err)
	simple, err := StitchFacetsWith(t.Context(), kept, StitchOptions{SimplifyPlanar: true})
	require.NoError(t, err)
	require.Equal(t, plain.Tris, simple.Tris)
	require.Equal(t, plain.Verts, simple.Verts)
	require.Equal(t, plain.VertexBound, simple.VertexBound)
}

// planarBox is an exact mesh of the box [0,2]^3 whose top (source 1) is a fan
// of four triangles round the centre vertex 8, whose claim is centreBeta.
func planarBox(centreBeta float64) PlanarMesh {
	var verts []r3.Vec
	for _, z := range []float64{0, 2} {
		verts = append(verts, r3.Vec{Z: z}, r3.Vec{X: 2, Z: z}, r3.Vec{X: 2, Y: 2, Z: z}, r3.Vec{Y: 2, Z: z})
	}
	verts = append(verts, r3.Vec{X: 1, Y: 1, Z: 2})
	in := PlanarMesh{Approx: verts, Xbeta: make([]float64, len(verts))}
	for _, v := range verts {
		in.Xverts = append(in.Xverts, proof.XptOf(v))
	}
	in.Xbeta[8] = centreBeta
	add := func(s int, tri ...[3]int) {
		for _, tr := range tri {
			in.Tris = append(in.Tris, tr)
			in.Src = append(in.Src, s)
			in.FacetBound = append(in.FacetBound, 0)
		}
	}
	add(0, [3]int{0, 2, 1}, [3]int{0, 3, 2})
	for i := range 4 {
		add(1, [3]int{4 + i, 4 + (i+1)%4, 8})
	}
	for k := range 4 {
		a, b := k, (k+1)%4
		add(2+k, [3]int{a, b, 4 + b}, [3]int{a, 4 + b, 4 + a})
	}
	return in
}

func facetsOf(in PlanarMesh, s int) int {
	n := 0
	for _, x := range in.Src {
		if x == s {
			n++
		}
	}
	return n
}

// TestSimplifyPlanarFan pins the interior-vertex rule on a box top held as a
// fan, and S5: a vertex with a positive claim keeps its face untouched.
//
// Leg shown to fail: checking the corners alone simplifies the claimed fan.
func TestSimplifyPlanarFan(t *testing.T) {
	in := planarBox(0)
	out, changed, err := SimplifyPlanarFaces(t.Context(), in)
	require.NoError(t, err)
	require.True(t, changed)
	require.Equal(t, 2, facetsOf(out, 1), "the fan becomes two triangles")
	require.Len(t, out.Xverts, 8, "the centre vertex is dropped")
	require.Len(t, in.Xverts, 9, "the input is not modified")
	require.Equal(t, 14, len(in.Tris))
	for _, v := range out.Xbeta {
		require.Zero(t, v)
	}

	claimed := planarBox(1e-9)
	out, changed, err = SimplifyPlanarFaces(t.Context(), claimed)
	require.NoError(t, err)
	require.False(t, changed)
	require.Equal(t, claimed.Tris, out.Tris)
}

// TestSimplifyPlanarSeamVertex pins the two-face rule: a vertex on the straight
// crease of two candidate faces leaves both, so the shared chain stays one
// chain and the mesh stays closed.
func TestSimplifyPlanarSeamVertex(t *testing.T) {
	in := planarBox(0)
	// Replace the fan by two triangles, then split the top-front edge at its
	// midpoint (vertex 9) in the top and the front wall.
	in.Xverts = append(in.Xverts, proof.XptOf(r3.Vec{X: 1, Z: 2}))
	in.Approx = append(in.Approx, r3.Vec{X: 1, Z: 2})
	in.Xbeta = append(in.Xbeta, 0)
	var tris [][3]int
	for i, tr := range in.Tris {
		switch {
		case in.Src[i] == 1 && tr[0] == 4 && tr[1] == 5: // top, edge 4-5 (the y = 0 edge)
			tris = append(tris, [3]int{4, 9, 8}, [3]int{9, 5, 8})
		case in.Src[i] == 2 && tr == [3]int{0, 1, 5}: // front wall, upper triangle
			tris = append(tris, [3]int{0, 1, 9}, [3]int{1, 5, 9})
		case in.Src[i] == 2 && tr == [3]int{0, 5, 4}:
			tris = append(tris, [3]int{0, 9, 4})
		default:
			tris = append(tris, tr)
		}
	}
	in.Tris = tris
	in.Src = in.Src[:0]
	in.FacetBound = in.FacetBound[:0]
	// Recompute sources: top facets are those with z = 2 corners only.
	for _, tr := range in.Tris {
		var s int
		allTop, allFront, allBottom := true, true, true
		for _, v := range tr {
			z, y := in.Approx[v].Z, in.Approx[v].Y
			allTop = allTop && z == 2
			allBottom = allBottom && z == 0
			allFront = allFront && y == 0
		}
		switch {
		case allTop:
			s = 1
		case allBottom:
			s = 0
		case allFront:
			s = 2
		default:
			s = 3
		}
		in.Src = append(in.Src, s)
		in.FacetBound = append(in.FacetBound, 0)
	}
	edges := make(map[[2]int]uint8)
	require.True(t, stitchEdgesPaired(in.Tris, edges), "the fixture is closed")
	out, changed, err := SimplifyPlanarFaces(t.Context(), in)
	require.NoError(t, err)
	require.True(t, changed)
	require.Less(t, len(out.Xverts), len(in.Xverts))
	for _, p := range out.Approx {
		require.False(t, p == (r3.Vec{X: 1, Z: 2}), "the crease vertex is gone from both faces")
	}
	require.True(t, stitchEdgesPaired(out.Tris, make(map[[2]int]uint8)))
}

// TestSimplifyPlanarFallback pins §7.1 step 6: a refusal on the simplified
// mesh runs the plain stitch, and the result equals it; a context error does
// not fall back.
func TestSimplifyPlanarFallback(t *testing.T) {
	kept := dovetailCut(t, 0.125)
	plain, err := StitchFacetsContext(t.Context(), kept)
	require.NoError(t, err)
	got, err := StitchFacetsWith(t.Context(), kept, StitchOptions{SimplifyPlanar: true,
		inject: &HeldFoldError{}})
	require.NoError(t, err)
	require.Equal(t, plain.Tris, got.Tris)
	require.Equal(t, plain.Verts, got.Verts)
	require.Equal(t, plain.VertexBound, got.VertexBound)

	_, err = StitchFacetsWith(t.Context(), kept, StitchOptions{SimplifyPlanar: true, inject: errors.New("other")})
	require.EqualError(t, err, "other")
}

// TestSimplifyPlanarHoledFace pins that a face with a hole is left alone: a
// cube cut by a blind-through square hole keeps every facet of its top, while
// the hole's walls may simplify.
func TestSimplifyPlanarHoledFace(t *testing.T) {
	a := exactBoxMesh(t, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
	b := prismMesh(t, [][2]float64{{3, 2}, {7, 2}, {7, 7}, {3, 7}}, -1, 11)
	kept, _, err := MeshBooleanStats(t.Context(), OpCut, a, b, NewContactMemo(a, b))
	require.NoError(t, err)
	const top = 5 // boxMesh's z-maximum side
	count := func(s *StitchedMesh) int {
		n := 0
		for _, x := range s.Src {
			if x == top {
				n++
			}
		}
		return n
	}
	plain, err := StitchFacetsContext(t.Context(), kept)
	require.NoError(t, err)
	simple, err := StitchFacetsWith(t.Context(), kept, StitchOptions{SimplifyPlanar: true})
	require.NoError(t, err)
	require.Equal(t, count(plain), count(simple))
	require.Zero(t, heldVolume(simple).Cmp(big.NewRat(1000-4*5*10, 1)))
}
