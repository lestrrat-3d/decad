package shellseparation

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// cylinderShell tessellates a closed-side cylinder along X: rings+1 rings of
// segs vertices, two triangles per quad, offset by (0, dy, 0).
func cylinderShell(rings, segs int, radius, length, dy float64) Shell {
	s := Shell{MeshBound: 1e-9}
	for r := 0; r <= rings; r++ {
		x := length * float64(r) / float64(rings)
		for k := range segs {
			ang := 2 * math.Pi * float64(k) / float64(segs)
			s.Verts = append(s.Verts, r3.Vec{X: x, Y: dy + radius*math.Cos(ang), Z: radius * math.Sin(ang)})
			s.VertexBound = append(s.VertexBound, 1e-9)
		}
	}
	for r := range rings {
		for k := range segs {
			a := r*segs + k
			b := r*segs + (k+1)%segs
			c := (r+1)*segs + k
			d := (r+1)*segs + (k+1)%segs
			s.Tris = append(s.Tris, [3]int{a, b, c}, [3]int{b, d, c})
		}
	}
	return s
}

func BenchmarkSlabsClearance(b *testing.B) {
	pa := cylinderShell(50, 40, 1, 10, 0)
	pb := cylinderShell(50, 40, 1, 10, 3)
	clearance, ok := slabsClearance(b.Context(), pa, pb, 0)
	require.True(b, ok)
	require.Greater(b, clearance, 0.5)
	b.ReportAllocs()
	b.ResetTimer()
	for range b.N {
		slabsClearance(b.Context(), pa, pb, 0)
	}
}
