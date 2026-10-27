package meshbool

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// cutStripGrid is an nx x ny grid of sx x sy quads from (ox, oy), two
// triangles each, with corner heights z(x, y).
func cutStripGrid(tb testing.TB, nx, ny int, ox, oy, sx, sy float64, z func(x, y float64) float64) *BoolMesh {
	tb.Helper()
	verts := make([]r3.Vec, 0, (nx+1)*(ny+1))
	for iy := 0; iy <= ny; iy++ {
		for ix := 0; ix <= nx; ix++ {
			x, y := ox+float64(ix)*sx, oy+float64(iy)*sy
			verts = append(verts, r3.NewVec(x, y, z(x, y)))
		}
	}
	tris := make([][3]int, 0, 2*nx*ny)
	for iy := range ny {
		for ix := range nx {
			v00 := iy*(nx+1) + ix
			v10, v01, v11 := v00+1, v00+nx+1, v00+nx+2
			tris = append(tris, [3]int{v00, v10, v11}, [3]int{v00, v11, v01})
		}
	}
	bm, err := PrepContext(tb.Context(), verts, tris, nil)
	require.NoError(tb, err)
	return bm
}

// cutCrossingFacets collects, for every cut facet of a, the transversal
// contact segments meshBoolean would hand cutTriangle, as cutFixtureFacets
// does. b must cover every facet of a, so every chain ends on a facet edge.
func cutCrossingFacets(tb testing.TB, a, b *BoolMesh) []cutFixtureFacet {
	tb.Helper()
	cutsA := map[int][]Xseg{}
	for i := range a.Tris {
		for j := range b.Tris {
			if !boxesOverlap(a.Boxes[i], b.Boxes[j]) {
				continue
			}
			c, err := TriTriClassify(TriCorners(a, i), TriCorners(b, j), XtriCorners(a, i), XtriCorners(b, j), a.Norms[i], b.Norms[j])
			require.NoError(tb, err)
			if c.Kind != ContactSegment || c.EdgeA >= 0 || c.EdgeB >= 0 {
				continue
			}
			cutsA[i] = append(cutsA[i], Xseg{A: c.P0, B: c.P1, AOnEdge: c.P0OnA, BOnEdge: c.P1OnA, Partner: TriCorners(b, j)})
		}
	}
	var out []cutFixtureFacet
	for i := range a.Tris {
		if segs, ok := cutsA[i]; ok {
			out = append(out, cutFixtureFacet{xtri: XtriCorners(a, i), normal: a.Norms[i], segs: segs})
		}
	}
	return out
}

// cutManyChainFacets is the facets of a coarse tilted n x n grid of the given
// step, crossed by a corrugated grid of 0.3 x fineY quads that covers it.
// The corrugation, of angular frequency freq along x, meets each coarse facet
// along several separate lines, so a facet carries several chains. A
// corrugation facet that crosses a coarse facet from edge to edge leaves a
// chain of one segment, which a long fineY makes common.
func cutManyChainFacets(tb testing.TB, n int, step, fineY, freq float64) []cutFixtureFacet {
	tb.Helper()
	const fineX = 0.3
	a := cutFixtureGrid(tb, n, 0, 0, step, func(x, y float64) float64 {
		return 2 + 0.03*x - 0.02*y
	})
	span := float64(n) * step
	nx := int(math.Ceil((span+0.6)/fineX)) + 1
	ny := int(math.Ceil((span+0.6)/fineY)) + 1
	b := cutStripGrid(tb, nx, ny, -0.17, -0.11, fineX, fineY, func(x, y float64) float64 {
		return 2.05 + 0.45*math.Sin(freq*x+0.02*y)
	})
	return cutCrossingFacets(tb, a, b)
}

// BenchmarkCutTriangleManyChains subdivides every cut facet of
// cutManyChainFacets through one worker's projection scratch.
func BenchmarkCutTriangleManyChains(b *testing.B) {
	facets := cutManyChainFacets(b, 4, 6, 9, 6)
	require.Greater(b, len(facets), 25)
	var scratch cutProjectionScratch
	for _, f := range facets {
		_, err := cutTriangle(b.Context(), f.xtri, f.normal, f.segs, &scratch)
		require.NoError(b, err)
	}
	b.ReportAllocs()
	for b.Loop() {
		for _, f := range facets {
			if _, err := cutTriangle(b.Context(), f.xtri, f.normal, f.segs, &scratch); err != nil {
				b.Fatal(err)
			}
		}
	}
}
