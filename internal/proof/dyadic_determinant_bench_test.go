package proof_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

type facetTriple [3]r3.Vec

// tessellatedFacets returns facets of a UV-tessellated sphere and cylinder
// with typical model coordinates, plus a few facets with wide exponent spans
// that reach the general fallback.
func tessellatedFacets(n int) []facetTriple {
	rng := rand.New(rand.NewPCG(7, 13))
	var facets []facetTriple
	const rings, segs = 24, 48
	sphere := func(i, j int) r3.Vec {
		theta, phi := math.Pi*float64(i)/rings, 2*math.Pi*float64(j%segs)/segs
		return r3.Vec{
			X: 31.7 + 25*math.Sin(theta)*math.Cos(phi),
			Y: -12.3 + 25*math.Sin(theta)*math.Sin(phi),
			Z: 8.9 + 25*math.Cos(theta),
		}
	}
	for i := range rings {
		for j := range segs {
			facets = append(facets,
				facetTriple{sphere(i, j), sphere(i+1, j), sphere(i+1, j+1)},
				facetTriple{sphere(i, j), sphere(i+1, j+1), sphere(i, j+1)})
		}
	}
	cyl := func(z, j int) r3.Vec {
		phi := 2 * math.Pi * float64(j%segs) / segs
		return r3.Vec{X: 12 * math.Cos(phi), Y: 12 * math.Sin(phi), Z: 40 * float64(z) / 8}
	}
	for z := range 8 {
		for j := range segs {
			facets = append(facets,
				facetTriple{cyl(z, j), cyl(z, j+1), cyl(z+1, j)},
				facetTriple{cyl(z, j+1), cyl(z+1, j+1), cyl(z+1, j)})
		}
	}
	rng.Shuffle(len(facets), func(i, j int) { facets[i], facets[j] = facets[j], facets[i] })
	if len(facets) > n {
		facets = facets[:n]
	}
	wide := func() r3.Vec {
		c := func() float64 { return math.Ldexp(rng.Float64()-0.5, rng.IntN(1500)-750) }
		return r3.Vec{X: c(), Y: c(), Z: c()}
	}
	for range n / 50 {
		facets = append(facets, facetTriple{wide(), wide(), wide()})
	}
	return facets
}

var benchSink proof.Dyadic

func BenchmarkFloatDeterminant(b *testing.B) {
	facets := tessellatedFacets(3000)
	for _, impl := range []struct {
		name string
		fn   func(a, b, c r3.Vec) proof.Dyadic
	}{
		{"legacy", proof.LegacyFloatDeterminant},
		{"current", proof.FloatDeterminant},
	} {
		b.Run(impl.name, func(b *testing.B) {
			b.ReportAllocs()
			for i := range b.N {
				f := &facets[i%len(facets)]
				benchSink = impl.fn(f[0], f[1], f[2])
			}
		})
	}
}
