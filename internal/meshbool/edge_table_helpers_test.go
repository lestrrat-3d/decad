package meshbool_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// edgeKeyPool mixes ordinary indices with values at and beyond the flat

// table's range limits, so every edge class is exercised.

var edgeKeyPool = []int{
	0, 1, 2, 3, 7, 100, 1 << 20, 1<<31 - 2, 1<<31 - 1, 1 << 31, 1 << 32, math.MaxInt, -1, -5, math.MinInt,
}

// referenceAdjacency is the map-based construction FacetAdjacencyContext used

// before the flat table.

func referenceAdjacency(tris [][3]int) [][]int {
	owner := make(map[[2]int]int, 3*len(tris))
	for i, tri := range tris {
		for k := range 3 {
			owner[[2]int{tri[k], tri[(k+1)%3]}] = i
		}
	}
	adj := make([][]int, len(tris))
	for i, tri := range tris {
		for k := range 3 {
			if twin, ok := owner[[2]int{tri[(k+1)%3], tri[k]}]; ok && twin != i {
				adj[i] = append(adj[i], twin)
			}
		}
	}
	return adj
}

func edgeTableMeshes() map[string][][3]int {
	out := map[string][][3]int{}
	out["empty"] = nil
	out["tetrahedron"] = [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}}
	out["open strip"] = [][3]int{{0, 1, 2}, {2, 1, 3}, {2, 3, 4}, {4, 3, 5}}
	out["duplicate directed edge"] = [][3]int{{0, 1, 2}, {0, 1, 3}, {1, 0, 4}, {1, 0, 5}}
	out["degenerate"] = [][3]int{{0, 0, 1}, {1, 0, 0}, {2, 2, 2}}
	out["huge indices"] = [][3]int{{1 << 40, 1, 2}, {1, 1 << 40, 3}, {-4, 1, 2}, {2, 1, -4}}
	for _, n := range []int{1, 3, 20} {
		var tris [][3]int
		for iy := range n {
			for ix := range n {
				v00 := iy*(n+1) + ix
				v10, v01, v11 := v00+1, v00+n+1, v00+n+2
				tris = append(tris, [3]int{v00, v10, v11}, [3]int{v00, v11, v01})
			}
		}
		out["grid"+string(rune('0'+n%10))+string(rune('0'+n/10))] = tris
	}
	for seed := range uint64(25) {
		r := rand.New(rand.NewPCG(seed, 99))
		nv := 4 + r.IntN(30)
		tris := make([][3]int, 1+r.IntN(120))
		for i := range tris {
			tris[i] = [3]int{r.IntN(nv), r.IntN(nv), r.IntN(nv)}
		}
		out["soup"+string(rune('a'+seed))] = tris
	}
	return out
}

// tiltedCube is a closed unit cube scaled by size, rotated about z then x and
// moved by at, prepared as an operand.
func tiltedCube(tb testing.TB, size, rz, rx float64, at r3.Vec) *meshbool.BoolMesh {
	tb.Helper()
	verts := make([]r3.Vec, 8)
	for i := range verts {
		x, y, z := float64(i&1)-.5, float64(i>>1&1)-.5, float64(i>>2&1)-.5
		x, y, z = size*x, size*y, size*z
		x, y = x*math.Cos(rz)-y*math.Sin(rz), x*math.Sin(rz)+y*math.Cos(rz)
		y, z = y*math.Cos(rx)-z*math.Sin(rx), y*math.Sin(rx)+z*math.Cos(rx)
		verts[i] = r3.NewVec(x+at.X, y+at.Y, z+at.Z)
	}
	tris := [][3]int{
		{0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6}, {0, 1, 5}, {0, 5, 4},
		{2, 6, 7}, {2, 7, 3}, {0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5},
	}
	bm, err := meshbool.PrepContext(tb.Context(), verts, tris, make([]int, len(tris)))
	require.NoError(tb, err)
	beta := make([]float64, len(verts))
	for i := range beta {
		beta[i] = float64(i+1) * 1e-9
	}
	bm.SetVertexBounds(beta)
	return bm
}

func edgeTableBenchTris(n int) [][3]int {
	var tris [][3]int
	for iy := range n {
		for ix := range n {
			v00 := iy*(n+1) + ix
			v10, v01, v11 := v00+1, v00+n+1, v00+n+2
			tris = append(tris, [3]int{v00, v10, v11}, [3]int{v00, v11, v01})
		}
	}
	return tris
}

func BenchmarkFacetAdjacencyEdgeTable(b *testing.B) {
	tris := edgeTableBenchTris(224) // about 100k triangles
	b.ReportAllocs()
	for b.Loop() {
		_, err := meshbool.FacetAdjacencyContext(b.Context(), tris)
		require.NoError(b, err)
	}
}

func BenchmarkFacetAdjacencyMapReference(b *testing.B) {
	tris := edgeTableBenchTris(224)
	b.ReportAllocs()
	for b.Loop() {
		_ = referenceAdjacency(tris)
	}
}
