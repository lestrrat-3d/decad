package meshbool

import (
	"context"
	"math"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/r3"
)

// polygonRing is the n-gon of radius r about the origin, its first corner the
// one at index shift.
func polygonRing(n, shift int, r float64) [][2]float64 {
	out := make([][2]float64, n)
	for i := range n {
		a := 2 * math.Pi * float64((i+shift)%n) / float64(n)
		out[i] = [2]float64{r * math.Cos(a), r * math.Sin(a)}
	}
	return out
}

// BenchmarkCoplanarCaps times a Union of two 64-gon prisms stacked cap to cap,
// each cap a fan from a different corner (agent-docs/coplanar-contact-design.md
// §10): every coplanar facet pair is classified.
func BenchmarkCoplanarCaps(b *testing.B) {
	ctx := context.Background()
	lower := polygonPrismMesh(b, polygonRing(64, 0, 5), 0, 1)
	upper := polygonPrismMesh(b, polygonRing(64, 1, 5), 1, 2)
	b.ResetTimer()
	for range b.N {
		if _, _, err := MeshBooleanStats(ctx, OpUnion, lower, upper, NewContactMemo(lower, upper)); err != nil {
			b.Fatal(err)
		}
	}
}

// BenchmarkMeshBooleanCrossing times two boxes crossing, the full
// MeshBooleanStats plus the stitch with planar simplification.
func BenchmarkMeshBooleanCrossing(b *testing.B) {
	ctx := context.Background()
	a := exactBoxMesh(b, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
	c := exactBoxMesh(b, r3.Vec{X: 5, Y: 3, Z: 4}, r3.Vec{X: 15, Y: 13, Z: 14})
	b.ResetTimer()
	for range b.N {
		kept, stats, err := MeshBooleanStats(ctx, OpUnion, a, c, NewContactMemo(a, c))
		require.NoError(b, err)
		_, err = StitchFacetsWith(ctx, kept, StitchOptions{Coplanar: stats.Coplanar, SimplifyPlanar: true})
		require.NoError(b, err)
	}
}
