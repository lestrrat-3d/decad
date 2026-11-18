package meshbool

import (
	"context"
	"math"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/coplanarcontact"
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
// §10, §13 D2). With the switch off the pair is refused at its first caps,
// which is what the retry ladder would then re-mesh; with it on every coplanar
// facet pair is classified.
func BenchmarkCoplanarCaps(b *testing.B) {
	for _, mode := range []struct {
		name string
		on   bool
	}{{"off", false}, {"on", true}} {
		b.Run(mode.name, func(b *testing.B) {
			ctx := coplanarcontact.With(context.Background(), mode.on)
			lower := prismMesh(b, polygonRing(64, 0, 5), 0, 1)
			upper := prismMesh(b, polygonRing(64, 1, 5), 1, 2)
			b.ResetTimer()
			for range b.N {
				_, _, err := MeshBooleanStats(ctx, OpUnion, lower, upper, NewContactMemo(lower, upper))
				if mode.on && err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}

// BenchmarkMeshBooleanCrossing times two boxes crossing, the full
// MeshBooleanStats plus the stitch, with the coplanar pass and planar
// simplification off and on. It reports numbers only.
func BenchmarkMeshBooleanCrossing(b *testing.B) {
	for _, mode := range []struct {
		name string
		on   bool
	}{{"off", false}, {"on", true}} {
		b.Run(mode.name, func(b *testing.B) {
			ctx := coplanarcontact.With(context.Background(), mode.on)
			a := exactBoxMesh(b, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
			c := exactBoxMesh(b, r3.Vec{X: 5, Y: 3, Z: 4}, r3.Vec{X: 15, Y: 13, Z: 14})
			b.ResetTimer()
			for range b.N {
				kept, stats, err := MeshBooleanStats(ctx, OpUnion, a, c, NewContactMemo(a, c))
				require.NoError(b, err)
				_, err = StitchFacetsWith(ctx, kept, StitchOptions{Coplanar: stats.Coplanar, SimplifyPlanar: mode.on})
				require.NoError(b, err)
			}
		})
	}
}
