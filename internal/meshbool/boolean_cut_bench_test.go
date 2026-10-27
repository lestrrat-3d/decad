package meshbool

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// cutFixtureFacet is one facet's subdivision input.
type cutFixtureFacet struct {
	xtri   [3]proof.Xpt
	normal proof.Xpt
	segs   []Xseg
}

// cutFixtureGrid is an n x n grid of quads, two triangles each, with corner
// heights z(x, y) over [ox, ox+n*step] x [oy, oy+n*step].
func cutFixtureGrid(tb testing.TB, n int, ox, oy, step float64, z func(x, y float64) float64) *BoolMesh {
	tb.Helper()
	verts := make([]r3.Vec, 0, (n+1)*(n+1))
	for iy := 0; iy <= n; iy++ {
		for ix := 0; ix <= n; ix++ {
			x, y := ox+float64(ix)*step, oy+float64(iy)*step
			verts = append(verts, r3.NewVec(x, y, z(x, y)))
		}
	}
	tris := make([][3]int, 0, 2*n*n)
	for iy := range n {
		for ix := range n {
			v00 := iy*(n+1) + ix
			v10, v01, v11 := v00+1, v00+n+1, v00+n+2
			tris = append(tris, [3]int{v00, v10, v11}, [3]int{v00, v11, v01})
		}
	}
	bm, err := PrepContext(tb.Context(), verts, tris, nil)
	require.NoError(tb, err)
	return bm
}

// cutFixtureFacets collects, for every facet of two crossing wavy grids,
// the transversal contact segments meshBoolean would hand cutTriangle: each
// facet pair's exact segment, skipping the in-plane ones that run along a
// facet edge. A crossing curve shares each endpoint between neighbouring
// segments, the way a boolean's contact chains do.
func cutFixtureFacets(tb testing.TB, n int) []cutFixtureFacet {
	tb.Helper()
	a := cutFixtureGrid(tb, n, 0, 0, 1, func(x, y float64) float64 {
		return 2 + 0.6*math.Sin(0.7*x) + 0.5*math.Cos(0.9*y)
	})
	b := cutFixtureGrid(tb, n, 0.37, 0.21, 1.01, func(x, y float64) float64 {
		return 2.1 + 0.07*x - 0.05*y + 0.4*math.Sin(0.5*x+0.3*y)
	})
	cutsA := map[int][]Xseg{}
	cutsB := map[int][]Xseg{}
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
			cutsB[j] = append(cutsB[j], Xseg{A: c.P0, B: c.P1, AOnEdge: c.P0OnB, BOnEdge: c.P1OnB, Partner: TriCorners(a, i)})
		}
	}
	// Both grids are open, so a facet near the other grid's rim may hold a
	// chain that stops where that grid ends. Keep only facets well inside the
	// region both grids cover, where every chain ends on the facet boundary.
	lo, hi := 1.5, float64(n)-1.5
	var out []cutFixtureFacet
	for _, side := range []struct {
		m    *BoolMesh
		cuts map[int][]Xseg
	}{{a, cutsA}, {b, cutsB}} {
		for i := range side.m.Tris {
			box := side.m.Boxes[i]
			if box[0].X < lo || box[0].Y < lo || box[1].X > hi || box[1].Y > hi {
				continue
			}
			if segs, ok := side.cuts[i]; ok {
				out = append(out, cutFixtureFacet{xtri: XtriCorners(side.m, i), normal: side.m.Norms[i], segs: segs})
			}
		}
	}
	return out
}

func boxesOverlap(a, b [2]r3.Vec) bool {
	return a[0].X <= b[1].X && b[0].X <= a[1].X &&
		a[0].Y <= b[1].Y && b[0].Y <= a[1].Y &&
		a[0].Z <= b[1].Z && b[0].Z <= a[1].Z
}

// BenchmarkCutTriangle subdivides every cut facet of the crossing grids
// through one worker's projection scratch, as subdivideCutFacets does.
func BenchmarkCutTriangle(b *testing.B) {
	facets := cutFixtureFacets(b, 24)
	require.Greater(b, len(facets), 150)
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
