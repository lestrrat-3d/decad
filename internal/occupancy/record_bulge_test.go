package occupancy

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// oldMeshPokes is the per-facet algorithm meshPokes replaces: every facet reads
// each near leaf's poke on its own, folding the leaves in order with math.Min.
func oldMeshPokes(r *Record, verts []r3.Vec, tris [][3]int) []float64 {
	out := make([]float64, len(tris))
	for t, tri := range tris {
		tv := [3]r3.Vec{verts[tri[0]], verts[tri[1]], verts[tri[2]]}
		lo, hi := triBox(tv)
		poke := math.Inf(1)
		for _, l := range r.Leaves {
			if l.near(lo, hi, r.Cross+l.Delta) {
				poke = math.Min(poke, l.shape.poke(tv))
			}
		}
		out[t] = poke
	}
	return out
}

func randRotation(rng *rand.Rand) [3][3]float64 {
	a, b, c := rng.Float64()*6.3, rng.Float64()*6.3, rng.Float64()*6.3
	sa, ca, sb, cb, sc, cc := math.Sin(a), math.Cos(a), math.Sin(b), math.Cos(b), math.Sin(c), math.Cos(c)
	rx := [3][3]float64{{1, 0, 0}, {0, ca, -sa}, {0, sa, ca}}
	ry := [3][3]float64{{cb, 0, sb}, {0, 1, 0}, {-sb, 0, cb}}
	rz := [3][3]float64{{cc, -sc, 0}, {sc, cc, 0}, {0, 0, 1}}
	mul := func(p, q [3][3]float64) (o [3][3]float64) {
		for i := range 3 {
			for j := range 3 {
				for k := range 3 {
					o[i][j] += p[i][k] * q[k][j]
				}
			}
		}
		return o
	}
	return mul(rz, mul(ry, rx))
}

func randChain(rng *rand.Rand, scale float64) Chain {
	n := 1 + rng.IntN(2)
	chain := make(Chain, n)
	for i := range chain {
		m := randRotation(rng)
		s := 1 + rng.Float64()*0.5
		for r := range m {
			for c := range m[r] {
				m[r][c] *= s
			}
		}
		chain[i] = Affine{M: m, T: [3]float64{
			(rng.Float64() - 0.5) * scale, (rng.Float64() - 0.5) * scale, (rng.Float64() - 0.5) * scale,
		}}
	}
	return chain
}

// randBulgeRecord is a record of cylinder and (unless cylindersOnly) polygon
// leaves, several of which sit near the same facets.
func randBulgeRecord(t *testing.T, rng *rand.Rand, scale float64, cylindersOnly bool) *Record {
	t.Helper()
	var leaves []Leaf
	for range 2 + rng.IntN(4) {
		if !cylindersOnly && rng.IntN(3) == 0 {
			l, err := PolygonLeaf(randChain(rng, scale), [][2]float64{{-1, -1}, {1.5, -1}, {1, 2}, {-1, 1}},
				0, 1+rng.Float64()*scale, 0.01, 0, 0, 1e-9, 0.01)
			require.NoError(t, err)
			leaves = append(leaves, l)
			continue
		}
		l, err := CircleLeaf(randChain(rng, scale), 0, 0, 0.5+rng.Float64()*scale, 0, 1+rng.Float64()*scale,
			rng.Float64()*0.01, 0, 0, 1e-9, 64, 0.01)
		require.NoError(t, err)
		leaves = append(leaves, l)
	}
	rec := LeafRecord(leaves...)
	rec.Cross = []float64{0, 0.05}[rng.IntN(2)]
	return rec
}

// randBulgeMesh clusters vertices so that facets share vertices, some facets
// reach across clusters, and magnitudes span many exponents.
func randBulgeMesh(rng *rand.Rand, scale float64) ([]r3.Vec, [][3]int) {
	var verts []r3.Vec
	var clusters [][]int
	for range 6 {
		c := r3.Vec{X: (rng.Float64() - 0.5) * scale, Y: (rng.Float64() - 0.5) * scale, Z: (rng.Float64() - 0.5) * scale}
		jitter := math.Pow(10, float64(rng.IntN(14)-10))
		var idx []int
		for range 8 {
			idx = append(idx, len(verts))
			verts = append(verts, r3.Vec{
				X: c.X + (rng.Float64()-0.5)*jitter, Y: c.Y + (rng.Float64()-0.5)*jitter, Z: c.Z + (rng.Float64()-0.5)*jitter,
			})
		}
		clusters = append(clusters, idx)
	}
	var tris [][3]int
	for range 120 {
		c := clusters[rng.IntN(len(clusters))]
		tri := [3]int{c[rng.IntN(len(c))], c[rng.IntN(len(c))], c[rng.IntN(len(c))]}
		if rng.IntN(5) == 0 {
			o := clusters[rng.IntN(len(clusters))]
			tri[2] = o[rng.IntN(len(o))]
		}
		tris = append(tris, tri)
	}
	return verts, tris
}

func TestMeshPokesMatchesPerFacetReading(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	finite, infinite := 0, 0
	for trial := range 40 {
		scale := []float64{3, 20, 1e-3, 1e5}[trial%4]
		rec := randBulgeRecord(t, rng, scale, false)
		verts, tris := randBulgeMesh(rng, scale)
		want := oldMeshPokes(rec, verts, tris)
		got := rec.meshPokes(verts, tris)
		require.Len(t, got, len(want))
		for i := range want {
			require.True(t, sameFloat(want[i], got[i]), "trial %d facet %d: want %v got %v", trial, i, want[i], got[i])
			if math.IsInf(want[i], 1) {
				infinite++
			} else {
				finite++
			}
		}
	}
	require.Positive(t, finite, "some facets must reach a leaf")
	require.Positive(t, infinite, "some facets must be far from every leaf")
}

func TestMeshPokesReadsOnlyVerticesTheFacetsReach(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(3, 5))
	rec := randBulgeRecord(t, rng, 3, true)
	verts, tris := randBulgeMesh(rng, 3)

	// A NaN vertex never compares near, so a facet holding it reads no leaf.
	nan := math.NaN()
	unreachable := append(append([]r3.Vec(nil), verts...), r3.Vec{X: nan, Y: nan, Z: nan})
	utris := append(append([][3]int(nil), tris...), [3]int{0, 1, len(verts)})
	require.NotPanics(t, func() { oldMeshPokes(rec, unreachable, utris) })
	require.NotPanics(t, func() { rec.meshPokes(unreachable, utris) })

	// An infinite vertex stretches a reaching facet's box, and the exact
	// reading of it cannot be made.
	var reach [3]int
	for i, p := range oldMeshPokes(rec, verts, tris) {
		if !math.IsInf(p, 1) {
			reach = tris[i]
			break
		}
	}
	require.NotEqual(t, [3]int{}, reach, "some facet must reach a leaf")
	reachable := append(append([]r3.Vec(nil), verts...), r3.Vec{X: math.Inf(1), Y: verts[reach[0]].Y, Z: verts[reach[0]].Z})
	rtris := append(append([][3]int(nil), tris...), [3]int{reach[0], reach[1], len(verts)})
	require.Panics(t, func() { oldMeshPokes(rec, reachable, rtris) })
	require.Panics(t, func() { rec.meshPokes(reachable, rtris) })
}
