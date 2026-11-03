package facetproof_test

import (
	"context"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/stretchr/testify/require"
)

// oldComponentMembers is the per-component queue and list walk that
// componentMembers replaced.
func oldComponentMembers(adj [][]int, comp []int) [][]int {
	var members [][]int
	for i := range comp {
		if comp[i] != -1 {
			continue
		}
		id := len(members)
		queue := []int{i}
		comp[i] = id
		var list []int
		for len(queue) > 0 {
			f := queue[0]
			queue = queue[1:]
			list = append(list, f)
			for _, nb := range adj[f] {
				if comp[nb] == -1 {
					comp[nb] = id
					queue = append(queue, nb)
				}
			}
		}
		members = append(members, list)
	}
	return members
}

func unassigned(n int) []int {
	comp := make([]int, n)
	for i := range comp {
		comp[i] = -1
	}
	return comp
}

func link(adj [][]int, a, b int) {
	adj[a] = append(adj[a], b)
	adj[b] = append(adj[b], a)
}

func componentGraphs() map[string][][]int {
	graphs := map[string][][]int{"empty": {}, "single": make([][]int, 1), "isolated": make([][]int, 5)}
	chain := make([][]int, 700)
	for i := 1; i < len(chain); i++ {
		link(chain, i-1, i)
	}
	graphs["chain"] = chain
	star := make([][]int, 300)
	for i := 1; i < len(star); i++ {
		link(star, 0, i)
	}
	graphs["star"] = star
	rng := rand.New(rand.NewPCG(7, 11))
	for sample := range 40 {
		n := 1 + rng.IntN(600)
		adj := make([][]int, n)
		for range rng.IntN(n + 1) {
			link(adj, rng.IntN(n), rng.IntN(n))
		}
		graphs["random"+string(rune('A'+sample%26))+string(rune('a'+sample/26))] = adj
	}
	return graphs
}

func TestComponentMembersMatchQueueWalk(t *testing.T) {
	t.Parallel()
	for name, adj := range componentGraphs() {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			wantComp := unassigned(len(adj))
			want := oldComponentMembers(adj, wantComp)
			gotComp := unassigned(len(adj))
			got, err := facetproof.ComponentMembers(t.Context(), adj, gotComp)
			require.NoError(t, err)
			require.Equal(t, wantComp, gotComp)
			require.Len(t, got, len(want))
			for i := range want {
				require.Equal(t, want[i], got[i], "component %d", i)
			}
		})
	}
}

func TestComponentMembersCapsEachList(t *testing.T) {
	t.Parallel()
	adj := make([][]int, 9)
	link(adj, 0, 1)
	link(adj, 1, 2)
	link(adj, 3, 4)
	got, err := facetproof.ComponentMembers(t.Context(), adj, unassigned(len(adj)))
	require.NoError(t, err)
	require.Greater(t, len(got), 2)
	for i, list := range got {
		require.Equal(t, len(list), cap(list), "component %d", i)
	}
	before := append([]int(nil), got[1]...)
	_ = append(got[0], 99)
	require.Equal(t, before, got[1])
}

func TestComponentMembersHonoursCancellation(t *testing.T) {
	t.Parallel()
	adj := make([][]int, 600)
	for i := 1; i < len(adj); i++ {
		link(adj, i-1, i)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := facetproof.ComponentMembers(ctx, adj, unassigned(len(adj)))
	require.ErrorIs(t, err, context.Canceled)
}
