package orthoshell_test

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/r3"
)

func BenchmarkPerfCombineInexactGridUnion(b *testing.B) {
	rng := rand.New(rand.NewPCG(37, 38))
	const side = 1.0 / 3
	var a, other *voxels
	var va, vb []r3.Vec
	var ta, tb [][3]int
	var outV []r3.Vec
	var outT [][3]int
	for {
		a, other = randomVoxels(rng), randomVoxels(rng)
		var okA, okB bool
		va, ta, _, okA = a.mesh()
		vb, tb, _, okB = other.mesh()
		if !okA || !okB {
			continue
		}
		va, vb = scaled(va, side), scaled(vb, side)
		var err error
		outV, outT, err = orthoshell.Combine(va, ta, vb, tb, orthoshell.Union)
		if err == nil {
			break
		}
	}
	want := float64(a.combine(other, orthoshell.Union).count()) * side * side * side
	if got := volume(outV, outT); got < want-1e-9 || got > want+1e-9 {
		b.Fatalf("union volume = %g, want %g", got, want)
	}
	b.ReportMetric(float64(len(va)+len(vb)), "input-verts/op")
	b.ReportMetric(float64(len(ta)+len(tb)), "input-tris/op")
	b.Logf("input vertices=%d triangles=%d; output vertices=%d triangles=%d",
		len(va)+len(vb), len(ta)+len(tb), len(outV), len(outT))
	b.ReportAllocs()
	b.ResetTimer()
	for range b.N {
		if _, _, err := orthoshell.Combine(va, ta, vb, tb, orthoshell.Union); err != nil {
			b.Fatal(err)
		}
	}
}
