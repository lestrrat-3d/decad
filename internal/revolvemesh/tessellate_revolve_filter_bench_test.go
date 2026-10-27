package revolvemesh

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// filterBenchPairs builds the box-overlapping, vertex-disjoint triangle pairs
// the facet-contact audit hands to RevolveSeparated: small triangles packed in
// a cube, so the pre-test settles most pairs on an early axis and walks all
// seventeen for the rest.
func filterBenchPairs(tb testing.TB) [][2]RevolveAuditTri {
	tb.Helper()
	rng := rand.New(rand.NewPCG(7, 11))
	coord := func(scale float64) float64 { return (rng.Float64() - 0.5) * scale }
	var tris []RevolveAuditTri
	for len(tris) < 200 {
		base := r3.NewVec(coord(6), coord(6), coord(6))
		verts := []r3.Vec{base,
			base.Add(r3.NewVec(coord(1.5), coord(1.5), coord(1.5))),
			base.Add(r3.NewVec(coord(1.5), coord(1.5), coord(1.5)))}
		t, ok := NewRevolveAuditTri(verts, [3]int{0, 1, 2})
		if ok {
			tris = append(tris, t)
		}
	}
	var pairs [][2]RevolveAuditTri
	for i := range tris {
		for j := i + 1; j < len(tris); j++ {
			if !BoxGapExceeds(tris[i].Box, tris[j].Box, 2e-9) {
				pairs = append(pairs, [2]RevolveAuditTri{tris[i], tris[j]})
			}
		}
	}
	require.NotEmpty(tb, pairs)
	return pairs
}

func BenchmarkRevolveSeparatedFloat(b *testing.B) {
	pairs := filterBenchPairs(b)
	b.ReportAllocs()
	b.ResetTimer()
	n := 0
	for range b.N {
		for _, p := range pairs {
			if RevolveSeparatedFloat(p[0], p[1], 1e-9) {
				n++
			}
		}
	}
	require.Positive(b, n)
}

func BenchmarkRevIvDot(b *testing.B) {
	pairs := filterBenchPairs(b)
	b.ReportAllocs()
	b.ResetTimer()
	var sink float64
	for range b.N {
		for _, p := range pairs {
			sink += RevIvDot(p[0].Fn, p[1].Fu).Hi
		}
	}
	require.False(b, sink != sink)
}
