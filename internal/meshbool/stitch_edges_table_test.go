package meshbool

// CI: isolated-tests

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/edgetable"
)

// TestStitchEdgesPairedTableMatchesMap compares the edge table audit with the
// retained map audit on closed, open, duplicated, flipped, self-edge,
// non-manifold and out-of-range-index inputs.
func TestStitchEdgesPairedTableMatchesMap(t *testing.T) {
	t.Parallel()
	bases := []int{0, 1000, 1 << 31, math.MaxInt - 8, -4}
	for seed := range uint64(60) {
		rng := rand.New(rand.NewPCG(seed, 0x57a7))
		tris := closedStitchFixture(rng, bases[int(seed)%len(bases)], rng.IntN(40))
		switch seed % 6 {
		case 1: // open: drop a triangle
			tris = tris[:len(tris)-1]
		case 2: // flip one triangle
			i := rng.IntN(len(tris))
			tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
		case 3: // duplicate a triangle (non-manifold)
			tris = append(tris, tris[rng.IntN(len(tris))])
		case 4: // append a self-edge triangle
			tris = append(tris, [3]int{tris[0][0], tris[0][0], tris[0][1]})
		case 5: // random soup over a tiny vertex range
			tris = make([][3]int, rng.IntN(16))
			for i := range tris {
				for k := range tris[i] {
					tris[i][k] = rng.IntN(5) + bases[rng.IntN(len(bases))]
				}
			}
		}
		want := stitchEdgesPairedMap(tris, make(map[[2]int]uint8))
		// Size hints must not change the result: too small forces growth.
		for _, hint := range []int{0, len(tris) + len(tris)/2} {
			require.Equal(t, want, stitchEdgesPaired(tris, edgetable.New(hint)), "seed %d hint %d", seed, hint)
		}
	}
}

// TestStitchEdgesPairedTableReuseAfterClear checks the retained-table path:
// a table left dirty by a failed audit, then cleared, audits like a fresh one.
func TestStitchEdgesPairedTableReuseAfterClear(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	edges := edgetable.New(0)
	for range 20 {
		tris := closedStitchFixture(rng, 0, rng.IntN(30))
		if rng.IntN(2) == 0 {
			tris = tris[:len(tris)-1]
		}
		edges.Clear()
		want := stitchEdgesPairedMap(tris, make(map[[2]int]uint8))
		require.Equal(t, want, stitchEdgesPaired(tris, edges))
	}
}
