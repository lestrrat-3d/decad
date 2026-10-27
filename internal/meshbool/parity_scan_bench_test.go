package meshbool

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// parityGridCube is the closed cube [0, 1]³ with every face cut into k×k
// quads, two facets each, wound outward. Every facet is axis-aligned, so on
// each swept axis two thirds of them project onto axis-parallel segments.
func parityGridCube(k int) ([]r3.Vec, [][3]int) {
	var verts []r3.Vec
	ids := map[[3]int]int{}
	at := func(i, j, l int) int {
		key := [3]int{i, j, l}
		if id, ok := ids[key]; ok {
			return id
		}
		ids[key] = len(verts)
		verts = append(verts, r3.Vec{X: float64(i) / float64(k), Y: float64(j) / float64(k), Z: float64(l) / float64(k)})
		return ids[key]
	}
	var tris [][3]int
	quad := func(a, b, c, d int) {
		tris = append(tris, [3]int{a, b, c}, [3]int{a, c, d})
	}
	for i := range k {
		for j := range k {
			quad(at(i, j, 0), at(i, j+1, 0), at(i+1, j+1, 0), at(i+1, j, 0))
			quad(at(i, j, k), at(i+1, j, k), at(i+1, j+1, k), at(i, j+1, k))
			quad(at(0, i, j), at(0, i, j+1), at(0, i+1, j+1), at(0, i+1, j))
			quad(at(k, i, j), at(k, i+1, j), at(k, i+1, j+1), at(k, i, j+1))
			quad(at(i, 0, j), at(i+1, 0, j), at(i+1, 0, j+1), at(i, 0, j+1))
			quad(at(i, k, j), at(i, k, j+1), at(i+1, k, j+1), at(i+1, k, j))
		}
	}
	return verts, tris
}

// BenchmarkParityPlainScan runs single-use parity queries the way
// pointquery.Locate does: every query prepares a fresh ParityMesh and scans
// every facet, so every query builds every facet's projected box.
func BenchmarkParityPlainScan(b *testing.B) {
	rng := rand.New(rand.NewPCG(0x5ca1, 0xab1e))
	cubeVerts, cubeTris := parityGridCube(20)
	cylVerts, cylTris := parityCylinder(1200)
	slVerts, slTris := paritySlanted(rng, 4800, 4)
	queries := make([]proof.Xpt, 32)
	for i := range queries {
		queries[i] = proof.XptOf(r3.Vec{X: 2*rng.Float64() - 0.5, Y: 2*rng.Float64() - 0.5, Z: 2*rng.Float64() - 0.5})
	}
	for _, mesh := range []struct {
		name  string
		verts []r3.Vec
		tris  [][3]int
	}{{`grid cube`, cubeVerts, cubeTris}, {`cylinder`, cylVerts, cylTris}, {`slanted`, slVerts, slTris}} {
		subset := parityAllFacets(len(mesh.tris))
		b.Run(mesh.name, func(b *testing.B) {
			ctx := b.Context()
			b.ReportAllocs()
			for range b.N {
				for _, p := range queries {
					if _, _, err := MeshParityContext(ctx, p, mesh.verts, mesh.tris, subset); err != nil {
						b.Fatal(err)
					}
				}
			}
		})
	}
}
