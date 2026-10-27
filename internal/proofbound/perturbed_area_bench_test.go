package proofbound_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ringMesh is a closed rows×cols grid on a torus of major radius 9 turned off
// every axis, so its coordinates carry full float mantissas the way a placed
// revolve or loft mesh does.
func ringMesh(rows, cols int) ([]r3.Vec, [][3]int) {
	verts := make([]r3.Vec, 0, rows*cols)
	for i := range rows {
		phi := 2 * math.Pi * float64(i) / float64(rows)
		for j := range cols {
			theta := 2 * math.Pi * float64(j) / float64(cols)
			rr := 9 + 1.7*math.Cos(theta)
			p := r3.Vec{X: rr * math.Cos(phi), Y: rr * math.Sin(phi), Z: 1.7 * math.Sin(theta)}
			// A small turn about a skew axis, then an offset.
			c, s := math.Cos(0.37), math.Sin(0.37)
			p = r3.Vec{X: c*p.X - s*p.Z + 0.125, Y: p.Y - 3.3, Z: s*p.X + c*p.Z + 41.7}
			verts = append(verts, p)
		}
	}
	tris := make([][3]int, 0, 2*rows*cols)
	for i := range rows {
		for j := range cols {
			a := i*cols + j
			b := i*cols + (j+1)%cols
			c := ((i+1)%rows)*cols + j
			d := ((i+1)%rows)*cols + (j+1)%cols
			tris = append(tris, [3]int{a, b, d}, [3]int{a, d, c})
		}
	}
	return verts, tris
}

// BenchmarkPerturbedArea measures the per-triangle exact area and edge-length
// kernel on a 9216-triangle closed mesh: serially, and through the public
// entry point at its own worker count.
func BenchmarkPerturbedArea(b *testing.B) {
	verts, tris := ringMesh(96, 48)
	b.Run("serial", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			_, oks, err := proofbound.PerturbedTriangleTermsForTest(nil, verts, tris, 1)
			if err != nil || !oks[0] {
				b.Fatalf("terms failed: %v", err)
			}
		}
	})
	b.Run("public", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			area, err := proofbound.PerturbedAreaUpperContext(b.Context(), verts, tris, 1e-6)
			if err != nil || math.IsInf(area, 0) || area <= 0 {
				b.Fatalf("area %v err %v", area, err)
			}
		}
	})
}
