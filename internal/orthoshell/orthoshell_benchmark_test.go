package orthoshell_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/orthoshell"
)

func BenchmarkPerfCombineInexactGridUnion(b *testing.B) {
	const side = 1.0 / 3
	box := func(lo, hi [3]int) *voxels {
		var v voxels
		for x := lo[0]; x < hi[0]; x++ {
			for y := lo[1]; y < hi[1]; y++ {
				for z := lo[2]; z < hi[2]; z++ {
					v[x][y][z] = true
				}
			}
		}
		return &v
	}
	a := box([3]int{3, 3, 1}, [3]int{6, 6, 4})
	other := box([3]int{3, 0, 2}, [3]int{6, 4, 6})
	a[5][4][1] = false
	va, ta, _, okA := a.mesh()
	vb, tb, _, okB := other.mesh()
	if !okA || !okB {
		b.Fatal("benchmark inputs must be closed meshes")
	}
	va, vb = scaled(va, side), scaled(vb, side)
	outV, outT, err := orthoshell.Combine(va, ta, vb, tb, orthoshell.Union)
	if err != nil {
		b.Fatal(err)
	}
	want := float64(a.combine(other, orthoshell.Union).count()) * side * side * side
	if got := volume(outV, outT); got < want-1e-9 || got > want+1e-9 {
		b.Fatalf("union volume = %g, want %g", got, want)
	}
	b.Logf("input vertices=%d triangles=%d; output vertices=%d triangles=%d",
		len(va)+len(vb), len(ta)+len(tb), len(outV), len(outT))
	b.ReportAllocs()
	b.ResetTimer()
	b.ReportMetric(float64(len(va)+len(vb)), "input-verts/op")
	b.ReportMetric(float64(len(ta)+len(tb)), "input-tris/op")
	for range b.N {
		if _, _, err := orthoshell.Combine(va, ta, vb, tb, orthoshell.Union); err != nil {
			b.Fatal(err)
		}
	}
}
