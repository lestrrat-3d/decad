package facetedtopology

// CI: isolated-tests

import (
	"fmt"
	"math"
	"math/rand"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestDirectedFacetsTableMatchesMap drives the table-backed index and the map
// it replaced with the same random writes and reads, including pairs outside
// [0, vertices) where the packed key lets distinct pairs collide.
func TestDirectedFacetsTableMatchesMap(t *testing.T) {
	pool := []int{0, 1, 2, 3, 9, 1 << 20, 1<<31 - 1, 1 << 31, 1<<32 - 1, 1 << 32, 1<<33 + 5, math.MaxInt, -1, -2, -(1 << 32),
		math.MinInt}
	for _, vertices := range []int{10, 1 << 31, 1 << 32, 1<<32 + 1, math.MaxInt} {
		for seed := range int64(6) {
			rng := rand.New(rand.NewSource(seed))
			want := newMapDirectedFacets(vertices, int(seed)*5)
			got := newDirectedFacets(vertices, int(seed)*5)
			pick := func() int {
				if rng.Intn(3) == 0 {
					return pool[rng.Intn(len(pool))]
				}
				return rng.Intn(12)
			}
			for step := range 3000 {
				u, v := pick(), pick()
				if rng.Intn(3) == 0 {
					facet := rng.Intn(1000)
					if rng.Intn(25) == 0 {
						facet = math.MaxInt32 + 1 + rng.Intn(3)
					}
					want.set(u, v, facet)
					got.set(u, v, facet)
				}
				wf, wok := want.get(u, v)
				gf, gok := got.get(u, v)
				if wf != gf || wok != gok {
					require.Failf(t, "get differs", "vertices %d seed %d step %d (%d,%d): want %d/%v got %d/%v", vertices,
						seed, step, u, v, wf, wok, gf, gok)
				}
				require.Equal(t, want.len(), got.len(), "vertices %d seed %d step %d len", vertices, seed, step)
			}
		}
	}
}

// TestBuildSparseFaceIDsMatchReference runs the face ids that do not fit the
// dense slices through the map-based reference build.
func TestBuildSparseFaceIDsMatchReference(t *testing.T) {
	var cases []topologyCase
	for _, base := range []topologyCase{cylinderCase(12, 3, 2, 3), sphereCase(8, 4, 1), sphereCase(8, 4, 3)} {
		cases = append(cases, sparseFaceVariants(base)...)
	}
	for _, base := range randomTopologyCases(rand.New(rand.NewSource(77)), 150) {
		cases = append(cases, sparseFaceVariants(base)...)
	}
	cases = append(cases, randomMeshCases(5, 1500)...)
	outcomes := map[string]int{}
	for _, c := range cases {
		want := runTopology(t.Context(), referenceBuild, c, -1)
		requireSameRun(t, c.name, want, runTopology(t.Context(), build, c, -1))
		switch {
		case want.panic != "":
			outcomes["panic"]++
		case want.err != "":
			outcomes["refused"]++
		default:
			outcomes["ok"]++
		}
		for _, limit := range []int{1, want.steps / 2, want.steps - 1, want.steps} {
			if limit >= 1 {
				requireSameRun(t, fmt.Sprintf("%s@%d", c.name, limit), runTopology(t.Context(), referenceBuild, c, limit),
					runTopology(t.Context(), build, c, limit))
			}
		}
	}
	t.Logf("outcomes: %v", outcomes)
	require.Greater(t, outcomes["ok"], 30)
	require.Greater(t, outcomes["refused"], 30)
}
