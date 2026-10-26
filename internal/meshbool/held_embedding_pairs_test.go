package meshbool

import (
	"context"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestHeldSelfPairTasksCoverEveryOverlappingPair(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x4317a, 0x9153b))
	for _, count := range []int{0, 1, 16, 17, 65, 257} {
		boxes := make([][2]r3.Vec, count)
		for i := range boxes {
			p := r3.NewVec(float64(rng.IntN(12)), float64(rng.IntN(12)), float64(rng.IntN(3)))
			boxes[i] = [2]r3.Vec{p, p.Add(r3.NewVec(2, 3, 1))}
		}
		if count > 16 {
			boxes[0] = [2]r3.Vec{{X: math.Inf(-1)}, {X: math.Inf(1), Y: 20, Z: 20}}
			boxes[1][0].X = math.NaN()
		}
		tree := newBoxTree(boxes)
		var members func(int) []int
		members = func(node int) []int {
			n := tree.nodes[node]
			if n.leafStart >= 0 {
				return append([]int(nil), tree.facets[n.leafStart:n.leafStart+n.leafCount]...)
			}
			return append(members(n.left), members(n.right)...)
		}
		want := map[[2]int]struct{}{}
		for i := range boxes {
			for j := i + 1; j < len(boxes); j++ {
				if BoxesOverlap(boxes[i], boxes[j]) {
					want[[2]int{i, j}] = struct{}{}
				}
			}
		}
		for _, limit := range []int{1, 5, 64, 4096} {
			tasks, err := tree.selfPairTasks(t.Context(), limit)
			require.NoError(t, err)
			got := map[[2]int]int{}
			for _, task := range tasks {
				for _, i := range members(task.a) {
					for _, j := range members(task.b) {
						if i == j || task.a == task.b && j < i || !BoxesOverlap(boxes[i], boxes[j]) {
							continue
						}
						got[[2]int{min(i, j), max(i, j)}]++
					}
				}
			}
			require.Equal(t, len(want), len(got), "boxes %d limit %d", count, limit)
			for pair := range want {
				require.Equal(t, 1, got[pair], "boxes %d limit %d pair %v", count, limit, pair)
			}
		}
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err := tree.selfPairTasks(ctx, 64)
		require.ErrorIs(t, err, context.Canceled)
	}
}

func TestHeldSelfPairsRejectIsolatedCollapsedFacet(t *testing.T) {
	h := heldQueryGrid(false)
	v := len(h.Verts)
	h.Verts = append(h.Verts, r3.Vec{X: 100}, r3.Vec{X: 101}, r3.Vec{X: 102})
	h.Tris = append(h.Tris, [3]int{v, v + 1, v + 2})
	h.Movable = make([]bool, len(h.Verts))
	e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
	require.NoError(t, e.buildGrid())
	query := make([]int, len(h.Tris))
	for i := range query {
		query[i] = i
	}
	clean, err := e.parallelClean(t.Context(), query)
	require.NoError(t, err)
	require.False(t, clean)
}

func TestHeldRepeatedQueryDoesNotCheckUnqueriedPairs(t *testing.T) {
	h := HeldRounding{
		Verts: []r3.Vec{{}, {X: 1}, {Y: 1},
			{X: 100}, {X: 101}, {X: 100, Y: 1},
			{X: 100}, {X: 101}, {X: 100, Y: 1}},
		Tris:    [][3]int{{0, 1, 2}, {3, 4, 5}, {6, 7, 8}},
		Movable: make([]bool, 9),
	}
	e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
	require.NoError(t, e.buildGrid())
	clean, err := e.parallelClean(t.Context(), []int{0, 0, 0})
	require.NoError(t, err)
	require.True(t, clean)
}

func TestHeldSelfPairPollingContinuesAcrossTasks(t *testing.T) {
	h := HeldRounding{
		Verts: []r3.Vec{{}, {X: 1}, {Y: 1}, {X: 10}, {X: 11}, {X: 10, Y: 1}},
		Tris:  [][3]int{{0, 1, 2}, {3, 4, 5}},
	}
	e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
	tree := newHeldBoxTree(e)
	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	e.pollErr, e.pollInterval, e.pollRemaining = ctx.Err, 4, 4
	clean, err := e.cleanSelfPairs([]heldNodePair{{0, 0}}, &tree)
	require.NoError(t, err)
	require.True(t, clean)
	require.Equal(t, 2, e.pollRemaining)
	cancel()
	clean, err = e.cleanSelfPairs([]heldNodePair{{0, 0}}, &tree)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, clean)
	require.Equal(t, 4, e.pollRemaining)
}
