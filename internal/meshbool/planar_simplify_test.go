package meshbool

import (
	"errors"
	"fmt"
	"math/big"
	"slices"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// polygonPrismMesh is the prism of the xy polygon poly (counter-clockwise) from z0 to
// z1, every corner a float, with one source id per side: the bottom is 0, the
// top 1 and wall k is 2+k. Its walls are exactly planar quads.
func polygonPrismMesh(t testing.TB, poly [][2]float64, z0, z1 float64) *BoolMesh {
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
	b := polygonPrismMesh(t, [][2]float64{{left(-1), -1}, {right(-1), -1}, {right(11), 11}, {left(11), 11}}, -1, 11)
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
	edges := NewEdgeTable(0)
	require.True(t, stitchEdgesPaired(in.Tris, edges), "the fixture is closed")
	out, changed, err := SimplifyPlanarFaces(t.Context(), in)
	require.NoError(t, err)
	require.True(t, changed)
	require.Less(t, len(out.Xverts), len(in.Xverts))
	for _, p := range out.Approx {
		require.False(t, p == (r3.Vec{X: 1, Z: 2}), "the crease vertex is gone from both faces")
	}
	require.True(t, stitchEdgesPaired(out.Tris, NewEdgeTable(0)))
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
	b := polygonPrismMesh(t, [][2]float64{{3, 2}, {7, 2}, {7, 7}, {3, 7}}, -1, 11)
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

// TestSimplifyPlanarOpenInput pins that an open input is returned unchanged:
// removing the vertices round a crack must not close it.
func TestSimplifyPlanarOpenInput(t *testing.T) {
	in := planarBox(0)
	var tris [][3]int
	var src []int
	var bound []float64
	for i, tr := range in.Tris {
		if i == 2 { // one triangle of the fan is missing
			continue
		}
		tris, src, bound = append(tris, tr), append(src, in.Src[i]), append(bound, 0)
	}
	in.Tris, in.Src, in.FacetBound = tris, src, bound
	out, changed, err := SimplifyPlanarFaces(t.Context(), in)
	require.NoError(t, err)
	require.False(t, changed)
	require.Equal(t, in.Tris, out.Tris)
}

// TestSimplifyPlanarTouchingBody pins that a vertex another body shares keeps
// its face: the centre vertex of the fan is also the apex of a tetrahedron, so
// dropping it from the top would leave the apex inside a top facet.
func TestSimplifyPlanarTouchingBody(t *testing.T) {
	in := planarBox(0)
	// A tetrahedron under the apex 8 = (1, 1, 2), with its own source ids.
	base := len(in.Xverts)
	for _, p := range []r3.Vec{{X: 5, Y: 5, Z: 0}, {X: 6, Y: 5, Z: 0}, {X: 5, Y: 6, Z: 0}} {
		in.Xverts = append(in.Xverts, proof.XptOf(p))
		in.Approx = append(in.Approx, p)
		in.Xbeta = append(in.Xbeta, 0)
	}
	for k, tr := range [][3]int{{base, base + 2, base + 1}, {base, base + 1, 8}, {base + 1, base + 2, 8}, {base + 2, base, 8}} {
		in.Tris = append(in.Tris, tr)
		in.Src = append(in.Src, 50+k)
		in.FacetBound = append(in.FacetBound, 0)
	}
	out, changed, err := SimplifyPlanarFaces(t.Context(), in)
	require.NoError(t, err)
	require.False(t, changed, "the fan's centre is shared with the tetrahedron")
	require.Equal(t, in.Tris, out.Tris)
}

// TestSimplifyPlanarTentTop pins that a top lifted off its plane is not
// simplified.
func TestSimplifyPlanarTentTop(t *testing.T) {
	in := planarBox(0)
	// Lift the fan's centre off the top plane.
	in.Xverts[8] = proof.XptOf(r3.Vec{X: 1, Y: 1, Z: 3})
	in.Approx[8] = r3.Vec{X: 1, Y: 1, Z: 3}
	out, changed, err := SimplifyPlanarFaces(t.Context(), in)
	require.NoError(t, err)
	require.False(t, changed, "a tent top is not planar")
	require.Equal(t, in.Tris, out.Tris)
}

// tentWithCollinearFirst: box [0,2]^3, top (src 1) is a tent with apex
// 8=(1,1,3); vertex 9=(1,0,2) splits the top-front edge; the first top facet
// is the degenerate (4,9,5).
func tentWithCollinearFirst() PlanarMesh {
	var verts []r3.Vec
	for _, z := range []float64{0, 2} {
		verts = append(verts, r3.Vec{Z: z}, r3.Vec{X: 2, Z: z}, r3.Vec{X: 2, Y: 2, Z: z}, r3.Vec{Y: 2, Z: z})
	}
	verts = append(verts, r3.Vec{X: 1, Y: 1, Z: 3}, r3.Vec{X: 1, Z: 2})
	in := PlanarMesh{Approx: verts, Xbeta: make([]float64, len(verts))}
	for _, v := range verts {
		in.Xverts = append(in.Xverts, proof.XptOf(v))
	}
	add := func(s int, tri ...[3]int) {
		for _, tr := range tri {
			in.Tris = append(in.Tris, tr)
			in.Src = append(in.Src, s)
			in.FacetBound = append(in.FacetBound, 0)
		}
	}
	add(0, [3]int{0, 2, 1}, [3]int{0, 3, 2})
	add(1, [3]int{4, 9, 5}, [3]int{4, 5, 8}, [3]int{5, 6, 8}, [3]int{6, 7, 8}, [3]int{7, 4, 8})
	// front wall y=0 with 9 on its top edge
	add(2, [3]int{0, 1, 9}, [3]int{1, 5, 9}, [3]int{0, 9, 4})
	for k := 1; k < 4; k++ {
		a, b := k, (k+1)%4
		add(2+k, [3]int{a, b, 4 + b}, [3]int{a, 4 + b, 4 + a})
	}
	return in
}

func TestSimplifyPlanarCollinearFirstFacet(t *testing.T) {
	in := tentWithCollinearFirst()
	require.True(t, stitchEdgesPaired(in.Tris, NewEdgeTable(0)), "fixture closed")
	_, changed, err := SimplifyPlanarFaces(t.Context(), in)
	require.NoError(t, err)
	require.False(t, changed)
}

// Two coplanar top faces (src 1 left x<=1, src 10 right x>=1) sharing the
// seam 9=(1,0,2) .. 10=(1,2,2) with midpoint 11=(1,1,2); 11 is a collinear
// seam vertex and must leave both faces.
func TestSimplifyPlanarCoplanarFacesSeam(t *testing.T) {
	var verts []r3.Vec
	for _, z := range []float64{0, 2} {
		verts = append(verts, r3.Vec{Z: z}, r3.Vec{X: 2, Z: z}, r3.Vec{X: 2, Y: 2, Z: z}, r3.Vec{Y: 2, Z: z})
	}
	verts = append(verts, r3.Vec{}, r3.Vec{X: 1, Z: 2}, r3.Vec{X: 1, Y: 2, Z: 2}, r3.Vec{X: 1, Y: 1, Z: 2})
	// index 8 unused placeholder (orphan)
	in := PlanarMesh{Approx: verts, Xbeta: make([]float64, len(verts))}
	for _, v := range verts {
		in.Xverts = append(in.Xverts, proof.XptOf(v))
	}
	add := func(s int, tri ...[3]int) {
		for _, tr := range tri {
			in.Tris = append(in.Tris, tr)
			in.Src = append(in.Src, s)
			in.FacetBound = append(in.FacetBound, 0)
		}
	}
	add(0, [3]int{0, 2, 1}, [3]int{0, 3, 2})
	// left top: 4(0,0) 9(1,0) 11(1,1) 10(1,2) 7(0,2)
	add(1, [3]int{4, 9, 11}, [3]int{4, 11, 7}, [3]int{7, 11, 10})
	// right top: 9(1,0) 5(2,0) 6(2,2) 10(1,2) 11(1,1)
	add(10, [3]int{9, 5, 11}, [3]int{5, 6, 11}, [3]int{6, 10, 11})
	// front wall y=0: 0 1 5 9 4
	add(2, [3]int{0, 1, 9}, [3]int{1, 5, 9}, [3]int{0, 9, 4})
	// right wall x=2
	add(3, [3]int{1, 2, 6}, [3]int{1, 6, 5})
	// back wall y=2: 2 3 7 10 6
	add(4, [3]int{2, 3, 10}, [3]int{2, 10, 6}, [3]int{3, 7, 10})
	// left wall x=0
	add(5, [3]int{3, 0, 4}, [3]int{3, 4, 7})
	require.True(t, stitchEdgesPaired(in.Tris, NewEdgeTable(0)), "fixture closed")
	out, changed, err := SimplifyPlanarFaces(t.Context(), in)
	require.NoError(t, err)
	require.True(t, changed)
	for _, p := range out.Approx {
		require.False(t, p == (r3.Vec{X: 1, Y: 1, Z: 2}))
	}
	require.True(t, stitchEdgesPaired(out.Tris, NewEdgeTable(0)))
	require.Equal(t, signedVol(in), signedVol(out))
}

func signedVol(m PlanarMesh) float64 {
	v := 0.0
	for _, t := range m.Tris {
		a, b, c := m.Approx[t[0]], m.Approx[t[1]], m.Approx[t[2]]
		v += a.Dot(b.Cross(c))
	}
	return v / 6
}

// The top holds an interior vertex 9 that is removable and the centre 8 that a
// tetrahedron also uses; dropping 9 must not re-triangulate the top over 8.
func TestSimplifyPlanarTouchingBodyWithRemovable(t *testing.T) {
	in := planarBox(0)
	in.Xverts = append(in.Xverts, proof.XptOf(r3.Vec{X: 0.5, Y: 0.5, Z: 2}))
	in.Approx = append(in.Approx, r3.Vec{X: 0.5, Y: 0.5, Z: 2})
	in.Xbeta = append(in.Xbeta, 0)
	var tris [][3]int
	var src []int
	for i, tr := range in.Tris {
		if in.Src[i] == 1 {
			continue
		}
		tris, src = append(tris, tr), append(src, in.Src[i])
	}
	for _, tr := range [][3]int{{4, 5, 9}, {5, 8, 9}, {8, 7, 9}, {7, 4, 9}, {5, 6, 8}, {6, 7, 8}} {
		tris, src = append(tris, tr), append(src, 1)
	}
	in.Tris, in.Src = tris, src
	in.FacetBound = make([]float64, len(tris))
	base := len(in.Xverts)
	for _, p := range []r3.Vec{{X: 5, Y: 5, Z: 0}, {X: 6, Y: 5, Z: 0}, {X: 5, Y: 6, Z: 0}} {
		in.Xverts = append(in.Xverts, proof.XptOf(p))
		in.Approx = append(in.Approx, p)
		in.Xbeta = append(in.Xbeta, 0)
	}
	for k, tr := range [][3]int{{base, base + 2, base + 1}, {base, base + 1, 8}, {base + 1, base + 2, 8}, {base + 2, base, 8}} {
		in.Tris = append(in.Tris, tr)
		in.Src = append(in.Src, 50+k)
		in.FacetBound = append(in.FacetBound, 0)
	}
	require.True(t, stitchEdgesPaired(in.Tris, NewEdgeTable(0)), "fixture closed")
	_, changed, err := SimplifyPlanarFaces(t.Context(), in)
	require.NoError(t, err)
	require.False(t, changed)
}

// TestSimplifyPlanarStartsAtABoundaryVertex pins that a face whose first facet
// has an interior vertex with the lowest id is still simplified: the box top is
// a fan round vertex 0, an interior vertex with a lower id than every corner.
func TestSimplifyPlanarStartsAtABoundaryVertex(t *testing.T) {
	in := planarBox(0)
	// Renumber so the fan's centre is vertex 0.
	perm := []int{1, 2, 3, 4, 5, 6, 7, 8, 0}
	var xv []proof.Xpt
	var ap []r3.Vec
	var xb []float64
	for range perm {
		xv, ap, xb = append(xv, proof.Xpt{}), append(ap, r3.Vec{}), append(xb, 0)
	}
	for old, nw := range perm {
		xv[nw], ap[nw], xb[nw] = in.Xverts[old], in.Approx[old], in.Xbeta[old]
	}
	in.Xverts, in.Approx, in.Xbeta = xv, ap, xb
	for i, tri := range in.Tris {
		in.Tris[i] = [3]int{perm[tri[0]], perm[tri[1]], perm[tri[2]]}
	}
	// The first top facet now starts at the centre.
	for i := range in.Tris {
		if in.Src[i] == 1 {
			in.Tris[0], in.Tris[i] = in.Tris[i], in.Tris[0]
			in.Src[0], in.Src[i] = in.Src[i], in.Src[0]
			// Rotate it to start at the centre, which keeps its winding.
			f := in.Tris[0]
			in.Tris[0] = [3]int{f[2], f[0], f[1]}
			break
		}
	}
	out, changed, err := SimplifyPlanarFaces(t.Context(), in)
	require.NoError(t, err)
	require.True(t, changed)
	require.Len(t, out.Xverts, 8, "the centre vertex is dropped")
}

// perCornerPlanar is the reference plane test of planarCandidate: every corner
// of every facet, repeats included, reads OrientSignExact zero against the
// first facet's points, and that facet is not collinear.
func perCornerPlanar(in PlanarMesh, facets []int) bool {
	first := in.Tris[facets[0]]
	a, b, c := in.Xverts[first[0]], in.Xverts[first[1]], in.Xverts[first[2]]
	for _, f := range facets {
		for _, v := range in.Tris[f] {
			if v == first[0] || v == first[1] || v == first[2] {
				continue
			}
			if proof.OrientSignExact(a, b, c, in.Xverts[v]) != 0 {
				return false
			}
		}
	}
	n := proof.Xcross(proof.Xsub(b, a), proof.Xsub(c, a))
	return n.X.Sign() != 0 || n.Y.Sign() != 0 || n.Z.Sign() != 0
}

// TestPlanarCandidateMatchesPerCorner pins planarCandidate's plane test, one
// sign per distinct vertex against a normal formed once, to the per-corner
// OrientSignExact reference: on every face of a planar box, a box whose top
// holds two interior vertices, and a tent whose first top facet is collinear,
// each as given and with each one of the face's vertices moved off its plane.
// Every face is one hole-free region, so the plane test alone decides.
//
// Legs shown to fail: testing the first facet alone, marking the wrong vertex
// as tested, or reading a vertex against itself instead of the base point
// accepts a moved vertex the reference refuses.
func TestPlanarCandidateMatchesPerCorner(t *testing.T) {
	// The box top as six facets round interior vertices 8 and 9, starting on
	// the rim, so 8 is shared by four facets and tested after facet 0.
	twoInterior := planarBox(0)
	twoInterior.Xverts = append(twoInterior.Xverts, proof.XptOf(r3.Vec{X: 0.5, Y: 0.5, Z: 2}))
	twoInterior.Approx = append(twoInterior.Approx, r3.Vec{X: 0.5, Y: 0.5, Z: 2})
	twoInterior.Xbeta = append(twoInterior.Xbeta, 0)
	var tris [][3]int
	var src []int
	for _, tr := range [][3]int{{4, 5, 9}, {5, 8, 9}, {8, 7, 9}, {7, 4, 9}, {5, 6, 8}, {6, 7, 8}} {
		tris, src = append(tris, tr), append(src, 1)
	}
	for i, tr := range twoInterior.Tris {
		if twoInterior.Src[i] != 1 {
			tris, src = append(tris, tr), append(src, twoInterior.Src[i])
		}
	}
	twoInterior.Tris, twoInterior.Src = tris, src
	twoInterior.FacetBound = make([]float64, len(tris))

	moved := func(in PlanarMesh, v int) PlanarMesh {
		const d = 1.0 / 1024
		xs := slices.Clone(in.Xverts)
		xs[v] = proof.XptOf(in.Approx[v].Add(r3.Vec{X: d, Y: d, Z: d}))
		in.Xverts = xs
		return in
	}
	seen := map[bool]int{}
	for _, fx := range []struct {
		name string
		in   PlanarMesh
	}{
		{"box", planarBox(0)},
		{"two interior", twoInterior},
		{"collinear tent", tentWithCollinearFirst()},
	} {
		owner := NewEdgeTable(3 * len(fx.in.Tris))
		for f, tri := range fx.in.Tris {
			for k := range 3 {
				owner.Set(tri[k], tri[(k+1)%3], f)
			}
		}
		faces := map[int][]int{}
		for f, s := range fx.in.Src {
			faces[s] = append(faces[s], f)
		}
		for s, facets := range faces {
			verts := map[int]struct{}{}
			for _, f := range facets {
				for _, v := range fx.in.Tris[f] {
					verts[v] = struct{}{}
				}
			}
			cases := map[string]PlanarMesh{"as given": fx.in}
			for v := range verts {
				cases[fmt.Sprintf("vertex %d moved", v)] = moved(fx.in, v)
			}
			for name, in := range cases {
				want := perCornerPlanar(in, facets)
				seen[want]++
				n, regions, ok, err := planarCandidate(t.Context(), proofbound.NewWorkBudget(t.Context()), in, owner, facets, s)
				require.NoError(t, err)
				require.Equal(t, want, ok, "%s face %d, %s", fx.name, s, name)
				if !ok {
					continue
				}
				first := in.Tris[facets[0]]
				a, b, c := in.Xverts[first[0]], in.Xverts[first[1]], in.Xverts[first[2]]
				require.Equal(t, proof.Xcross(proof.Xsub(b, a), proof.Xsub(c, a)).Key(), n.Key())
				require.Len(t, regions, 1)
			}
		}
	}
	require.Positive(t, seen[true], "some face is planar")
	require.Positive(t, seen[false], "some face is refused")

	tent := tentWithCollinearFirst()
	var top []int
	for f, s := range tent.Src {
		if s == 1 {
			top = append(top, f)
		}
	}
	_, _, ok, err := planarCandidate(t.Context(), proofbound.NewWorkBudget(t.Context()), tent, NewEdgeTable(0), top, 1)
	require.NoError(t, err)
	require.False(t, ok, "a collinear first facet gives no plane")
}
