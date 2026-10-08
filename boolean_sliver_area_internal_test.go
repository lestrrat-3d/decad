package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// exactFacetAreaSum is the exact area of the given facets, measured at 2048
// bits: each facet's squared doubled area is an exact rational of its float
// vertices, and only the square root and the halving are taken in big.Float.
func exactFacetAreaSum(verts []r3.Vec, tris [][3]int) *big.Float {
	const prec = 2048
	rat := func(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }
	sum := new(big.Float).SetPrec(prec)
	for _, t := range tris {
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		u := [3]*big.Rat{
			new(big.Rat).Sub(rat(b.X), rat(a.X)), new(big.Rat).Sub(rat(b.Y), rat(a.Y)), new(big.Rat).Sub(rat(b.Z), rat(a.Z)),
		}
		v := [3]*big.Rat{
			new(big.Rat).Sub(rat(c.X), rat(a.X)), new(big.Rat).Sub(rat(c.Y), rat(a.Y)), new(big.Rat).Sub(rat(c.Z), rat(a.Z)),
		}
		sq := new(big.Rat)
		for i := range 3 {
			j, k := (i+1)%3, (i+2)%3
			comp := new(big.Rat).Sub(new(big.Rat).Mul(u[j], v[k]), new(big.Rat).Mul(u[k], v[j]))
			sq.Add(sq, new(big.Rat).Mul(comp, comp))
		}
		area := new(big.Float).SetPrec(prec).SetRat(sq)
		area.Sqrt(area)
		area.Quo(area, new(big.Float).SetPrec(prec).SetInt64(2))
		sum.Add(sum, area)
	}
	return sum
}

// facetAreaGap is |held − exact| for a float area held against the exact area
// of the facets that produced it.
func facetAreaGap(held float64, verts []r3.Vec, tris [][3]int) *big.Float {
	gap := new(big.Float).SetPrec(2048).SetFloat64(held)
	gap.Sub(gap, exactFacetAreaSum(verts, tris))
	return gap.Abs(gap)
}

// TestFacetedAreaBoundsCoverSliverFacet pins the per-facet evaluation charge
// (proofbound.FacetAreaTermSlop) at every site that sums float facet areas,
// through a real mesh boolean.
//
// The sliver prism's profile is a cap-shaped triangle: its third vertex sits
// 4·10⁻⁵ mm off the 200 mm edge, so at every corner the two edges are nearly
// parallel and the float cross product rounds at the scale of its edge
// products (about 40 mm²) while the area is about 0.0017 mm². The framed box
// crosses its walls, so the union takes the mesh path; the bottom cap at z = 0
// is nowhere near the box and survives as one face holding one facet, the
// sliver, whose exact float vertices give it a zero geometric term. Without
// the charge its published bound is SumSlop's relative 4·u·area, below the
// facet's own rounding error by three orders of magnitude.
//
// Every face's bound, and the body's, is checked against the exact area of
// the facets it speaks for. The sliver cap is also handed alone to
// meshAreaUpper and tessellation.FaceAreaUpper, the other two sums of float
// facet areas.
//
// Shown to fail first: dropping faceTermSlop from the per-face bound in
// boolean_body.go turns the sliver face red; dropping termSlop from
// facetproof.MeshAreaUpper turns the meshAreaUpper assertion red; dropping
// FacetAreaTermSlop from tessellation.FaceAreaUpper turns that assertion red.
// The body-level assertion stays green without the body's own termSlop here:
// the box's large faces carry a summation charge far above one sliver's error.
func TestFacetedAreaBoundsCoverSliverFacet(t *testing.T) {
	t.Parallel()
	doc := New()
	sliver := internalPolyPrismBody(t, doc, [][2]float64{{-100.1, 7.1}, {100.3, 7.3}, {0.123, 7.20004}}, 10)
	union, err := Union(t.Context(), sliver, internalCrossingFramedBox(t, doc))
	require.NoError(t, err)
	fp, ok := union.payload.(facetedPayload)
	require.True(t, ok, `the union takes the mesh path`)

	faceTris := map[int][][3]int{}
	for i, fi := range fp.faceOf {
		faceTris[fi] = append(faceTris[fi], fp.tris[i])
	}
	faces := union.Faces()
	var sliverTri [3]int
	sliverFaces := 0
	for fi, tris := range faceTris {
		m, err := faces[fi].Area()
		require.NoError(t, err)
		gap := facetAreaGap(m.Value.Base(), fp.verts, tris)
		require.LessOrEqual(t, gap.Cmp(new(big.Float).SetFloat64(m.Bound.Base())), 0,
			`face %d (%d facets): area %v, bound %v, gap %s`, fi, len(tris), m.Value.Base(), m.Bound.Base(), gap.Text('g', 6))
		if len(tris) == 1 && fp.verts[tris[0][0]].Z == 0 && fp.verts[tris[0][1]].Z == 0 && fp.verts[tris[0][2]].Z == 0 {
			sliverTri = tris[0]
			sliverFaces++
			// The fixture is a sliver only while its rounding beats the
			// relative charge SumSlop alone gives one term.
			relative := new(big.Float).SetFloat64(proofbound.SumSlop(1, m.Value.Base()))
			require.Positive(t, gap.Cmp(relative), `the sliver cap must round past SumSlop's relative charge`)
		}
	}
	require.Equal(t, 1, sliverFaces, `the bottom cap survives as one single-facet face`)

	body, err := union.Area()
	require.NoError(t, err)
	gap := facetAreaGap(body.Value.Base(), fp.verts, fp.tris)
	require.LessOrEqual(t, gap.Cmp(new(big.Float).SetFloat64(body.Bound.Base())), 0)

	only := [][3]int{sliverTri}
	exact := exactFacetAreaSum(fp.verts, only)
	require.GreaterOrEqual(t, new(big.Float).SetFloat64(meshAreaUpper(fp.verts, only)).Cmp(exact), 0,
		`meshAreaUpper bounds the sliver's exact area from above`)
	upper := tessellation.FaceAreaUpper(fp.verts, only, []int{0}, make([]float64, len(fp.verts)))
	require.GreaterOrEqual(t, new(big.Float).SetFloat64(upper[0]).Cmp(exact), 0,
		`tessellation.FaceAreaUpper bounds the sliver's exact area from above`)
}
