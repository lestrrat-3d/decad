package meshbool

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func heldQueryGrid(crossing bool) HeldRounding {
	const side = 8
	var h HeldRounding
	for y := range side + 1 {
		for x := range side + 1 {
			h.Verts = append(h.Verts, r3.Vec{X: float64(x), Y: float64(y)})
		}
	}
	for y := range side {
		for x := range side {
			a := y*(side+1) + x
			b := a + side + 1
			h.Tris = append(h.Tris, [3]int{a, a + 1, b}, [3]int{a + 1, b + 1, b})
		}
	}
	if crossing {
		n := len(h.Verts)
		h.Verts = append(h.Verts, r3.Vec{X: 0.2, Y: 0.2, Z: -1},
			r3.Vec{X: 0.8, Y: 0.2, Z: 1}, r3.Vec{X: 0.2, Y: 0.8, Z: 1})
		h.Tris = append(h.Tris, [3]int{n, n + 1, n + 2})
	}
	h.Movable = make([]bool, len(h.Verts))
	return h
}

func TestHeldQueryCeilingsKeepEveryRequiredPair(t *testing.T) {
	for _, crossing := range []bool{false, true} {
		h := heldQueryGrid(crossing)
		for _, selection := range []string{"all", "odd", "first", "last", "none"} {
			var query []int
			mask := make([]bool, len(h.Tris))
			for i := range h.Tris {
				if selection == "all" || selection == "odd" && i%2 == 1 ||
					selection == "first" && i == 0 || selection == "last" && i == len(h.Tris)-1 {
					query = append(query, i)
					mask[i] = true
				}
			}
			// The reference does not traverse a tree or skip by query order.
			// Every pair involving a query must be embedded, in either order.
			reference := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
			want := true
			for _, i := range query {
				for j := range h.Tris {
					if j != i && !reference.pairEmbedded(i, j) {
						want = false
					}
				}
			}
			if selection == "last" && want == crossing {
				t.Fatalf("the last-facet reference did not detect crossing=%t", crossing)
			}
			e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
			require.NoError(t, e.buildGrid())
			clean, err := e.parallelClean(t.Context(), query)
			require.NoError(t, err)
			if clean != want {
				t.Fatalf("crossing=%t selection=%s: clean=%t, want %t", crossing, selection, clean, want)
			}
		}
	}
}

func TestHeldQueryCeilingsSkipCompletedChunks(t *testing.T) {
	h := heldQueryGrid(false)
	mask := make([]bool, len(h.Tris))
	for i := range mask {
		mask[i] = true
	}
	// The final query's lower-index partners belong to earlier query chunks.
	// Its whole tree can be discarded after one node operation.
	counts := [2]int{}
	for variant := range counts {
		e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
		tree := newHeldBoxTree(e)
		if variant == 1 {
			require.NoError(t, tree.prepareQueryCeilings(t.Context(), mask))
		}
		e.pollInterval, e.pollRemaining = 1, 1
		e.pollErr = func() error {
			counts[variant]++
			return nil
		}
		clean, err := e.cleanPairs([]int{len(h.Tris) - 1}, mask, &tree)
		require.NoError(t, err)
		require.True(t, clean)
	}
	require.Greater(t, counts[0], 1)
	require.Equal(t, 1, counts[1])

	e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
	tree := newHeldBoxTree(e)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	require.ErrorIs(t, tree.prepareQueryCeilings(ctx, mask), context.Canceled)
}
