package meshbool

import (
	"context"
	"math"
	"math/big"
	"math/rand/v2"
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
			for _, axes := range [][3]r3.Vec{{}, {{X: 2, Y: 3, Z: 5}, {X: -3, Y: 2}, {X: 1, Z: -2}}} {
				h.ScanAxes = axes
				e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
				require.NoError(t, e.buildGrid())
				clean, err := e.parallelClean(t.Context(), query)
				require.NoError(t, err)
				if clean != want {
					t.Fatalf("crossing=%t selection=%s axes=%v: clean=%t, want %t", crossing, selection, axes, clean, want)
				}
			}
		}
	}
}

func TestHeldScanBoxesEncloseExactDotProducts(t *testing.T) {
	rng := rand.New(rand.NewPCG(0x1284ab, 0x318f72))
	axes := [3]r3.Vec{{X: 2, Y: 3, Z: 5}, {X: -3, Y: 2}, {X: 1, Z: -2}}
	for _, exp := range []int{-1074, -300, 0, 300} {
		scanAxes := axes
		if exp == -1074 {
			for i := range scanAxes {
				scanAxes[i] = scanAxes[i].Scale(0x1p-10)
			}
		}
		h := HeldRounding{ScanAxes: scanAxes}
		for range 48 {
			h.Verts = append(h.Verts, r3.NewVec(math.Ldexp(2*rng.Float64()-1, exp),
				math.Ldexp(2*rng.Float64()-1, exp), math.Ldexp(2*rng.Float64()-1, exp)))
		}
		for i := 0; i < len(h.Verts); i += 3 {
			h.Tris = append(h.Tris, [3]int{i, i + 1, i + 2})
		}
		e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
		tree, err := newHeldScanTree(t.Context(), e)
		require.NoError(t, err)
		require.True(t, tree.oriented)
		for i, tri := range h.Tris {
			for _, v := range tri {
				p := h.Verts[v]
				for k, axis := range scanAxes {
					coords, dir := [3]float64{p.X, p.Y, p.Z}, [3]float64{axis.X, axis.Y, axis.Z}
					exact := new(big.Rat)
					for j := range coords {
						exact.Add(exact, new(big.Rat).Mul(new(big.Rat).SetFloat64(coords[j]),
							new(big.Rat).SetFloat64(dir[j])))
					}
					lo := new(big.Rat).SetFloat64(CoordOf(tree.boxes[i][0], k))
					hi := new(big.Rat).SetFloat64(CoordOf(tree.boxes[i][1], k))
					require.LessOrEqual(t, lo.Cmp(exact), 0, "triangle %d, axis %d", i, k)
					require.GreaterOrEqual(t, hi.Cmp(exact), 0, "triangle %d, axis %d", i, k)
				}
			}
		}
	}
}

func TestHeldScanAxesFallbackAndBoundaryContact(t *testing.T) {
	h := HeldRounding{
		Verts: []r3.Vec{{}, {X: 1}, {Y: 1}, {X: 1}, {X: 2}, {X: 1, Y: 1}},
		Tris:  [][3]int{{0, 1, 2}, {3, 4, 5}}, Movable: make([]bool, 6),
	}
	for _, axes := range [][3]r3.Vec{
		{}, {{Y: -1}, {Z: 1}, {X: 1}},
		{{X: math.Inf(1)}, {Y: 1}, {Z: 1}},
		{{X: math.NaN()}, {Y: 1}, {Z: 1}},
		{{X: math.MaxFloat64}, {Y: 1}, {Z: 1}},
	} {
		h.ScanAxes = axes
		e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
		tree, err := newHeldScanTree(t.Context(), e)
		require.NoError(t, err)
		require.False(t, tree.oriented)
		for i, tri := range h.Tris {
			require.Equal(t, TriBox(h.Verts, tri), tree.boxes[i])
		}
	}
	// The triangles meet at a closed box boundary but share no index. Their
	// unexpected contact must still reach the exact classifier and refuse.
	h.ScanAxes = [3]r3.Vec{{X: 2, Y: 3, Z: 5}, {X: -3, Y: 2}, {X: 1, Z: -2}}
	e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
	require.NoError(t, e.buildGrid())
	clean, err := e.parallelClean(t.Context(), []int{0, 1})
	require.NoError(t, err)
	require.False(t, clean)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = newHeldScanTree(ctx, e)
	require.ErrorIs(t, err, context.Canceled)
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
