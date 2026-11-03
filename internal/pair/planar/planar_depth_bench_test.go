package planar

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
)

func benchmarkSubdividedBox(n int, xOffset int64) PlanarSolid {
	type face struct {
		fixedAxis int
		fixedSign int64
		uAxis     int
		vAxis     int
		uSign     int64
		vSign     int64
	}
	faces := [...]face{
		{0, 1, 1, 2, 1, 1}, {0, -1, 2, 1, 1, 1},
		{1, 1, 2, 0, 1, 1}, {1, -1, 0, 2, 1, 1},
		{2, 1, 0, 1, 1, 1}, {2, -1, 1, 0, 1, 1},
	}
	var solid PlanarSolid
	for _, f := range faces {
		base := len(solid.Verts)
		for i := range n + 1 {
			for j := range n + 1 {
				var point proof.DyV3
				point[f.fixedAxis] = proof.DyInt(f.fixedSign)
				point[f.uAxis] = proof.DyShift(proof.DyInt(f.uSign*int64(2*i-n)), -3)
				point[f.vAxis] = proof.DyShift(proof.DyInt(f.vSign*int64(2*j-n)), -3)
				point[0] = proof.DyAdd(point[0], proof.DyInt(xOffset))
				solid.Verts = append(solid.Verts, point)
			}
		}
		for i := range n {
			for j := range n {
				a := base + i*(n+1) + j
				b, c, d := a+n+1, a+n+2, a+1
				solid.Tris = append(solid.Tris, [3]int{a, b, c}, [3]int{a, c, d})
			}
		}
	}
	return solid
}

func benchmarkNoPoll() error { return nil }

func BenchmarkPlanarDeepVertex(b *testing.B) {
	a := benchmarkSubdividedBox(8, 0)
	other := benchmarkSubdividedBox(8, 3)
	margin := proof.DyZero()
	b.ReportAllocs()
	b.ResetTimer()
	for range b.N {
		deep, err := PlanarDeepVertex(&a, &other, margin, benchmarkNoPoll)
		if err != nil || deep {
			b.Fatalf("PlanarDeepVertex returned deep=%v, err=%v", deep, err)
		}
	}
}
