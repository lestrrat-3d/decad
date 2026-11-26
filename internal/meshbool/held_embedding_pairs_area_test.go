package meshbool

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// heldAreaMeshes are the meshes the scan tree is compared on: clean, crossing and
// collapsed grids, and tubes whose long thin facets overlap by box alone.
func heldAreaMeshes() []struct {
	name string
	h    func(*testing.T) HeldRounding
} {
	skew := [3]r3.Vec{{X: 2, Y: 3, Z: 5}, {X: -3, Y: 2}, {X: 1, Z: -2}}
	return []struct {
		name string
		h    func(*testing.T) HeldRounding
	}{
		{"grid", func(t *testing.T) HeldRounding { return heldPrepareGrid(t, 12, "none") }},
		{"grid-crossing", func(t *testing.T) HeldRounding { return heldPrepareGrid(t, 12, "crossing") }},
		{"grid-collapsed", func(t *testing.T) HeldRounding { return heldPrepareGrid(t, 12, "collapsed") }},
		{"grid-crossing-skew", func(t *testing.T) HeldRounding {
			h := heldPrepareGrid(t, 12, "crossing")
			h.ScanAxes = skew
			return h
		}},
		{"revolved", func(*testing.T) HeldRounding { return heldRevolvedTube(200, false) }},
		{"revolved-oriented", func(*testing.T) HeldRounding { return heldRevolvedTube(200, true) }},
		{"coarse-oriented", func(*testing.T) HeldRounding { return heldCoarseRevolve(12, 400, true) }},
	}
}

// heldSelfCandidates lists, in a map, every facet pair the self-pair scan of
// tree hands to the pair predicates: both facet boxes overlap, and for an
// oriented tree both world boxes. It also counts the leaf-pair iterations the
// scan makes to find them.
func heldSelfCandidates(t *testing.T, tree *heldBoxTree) (map[[2]int]int, int) {
	t.Helper()
	tasks, err := tree.selfPairTasks(t.Context(), 1)
	require.NoError(t, err)
	pairs := map[[2]int]int{}
	iterations := 0
	stack := append([]heldNodePair(nil), tasks...)
	for len(stack) > 0 {
		pair := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		a, b := tree.nodes[pair.a], tree.nodes[pair.b]
		if !BoxesOverlap(a.box, b.box) {
			continue
		}
		if a.leafStart < 0 || b.leafStart < 0 {
			children, n := tree.splitSelfPair(pair)
			stack = append(stack, children[:n]...)
			continue
		}
		for s := a.leafStart; s < a.leafStart+a.leafCount; s++ {
			if pair.a != pair.b && !BoxesOverlap(tree.slotBoxes[s], b.box) {
				continue
			}
			start := b.leafStart
			if pair.a == pair.b {
				start = s + 1
			}
			for u := start; u < b.leafStart+b.leafCount; u++ {
				iterations++
				if !BoxesOverlap(tree.slotBoxes[s], tree.slotBoxes[u]) {
					continue
				}
				if tree.oriented && !BoxesOverlap(tree.slotWorld[s], tree.slotWorld[u]) {
					continue
				}
				i, j := tree.facets[s], tree.facets[u]
				pairs[[2]int{min(i, j), max(i, j)}]++
			}
		}
	}
	return pairs, iterations
}

// TestHeldScanTreeMatchesLowerCornerTree compares the scan tree, whose nodes
// split by surface area, with the longest-axis lower-corner tree on the same
// boxes. The reference scan on the old tree and cleanSelfPairs on the scan tree
// must give one answer, and the two trees must hand the pair predicates the
// same facet pairs, each once; the scan tree makes fewer leaf iterations on
// the long-facet meshes, at least half as many on the coarse revolve.
func TestHeldScanTreeMatchesLowerCornerTree(t *testing.T) {
	t.Parallel()
	for _, mesh := range heldAreaMeshes() {
		t.Run(mesh.name, func(t *testing.T) {
			t.Parallel()
			h := mesh.h(t)
			e, scan, tasks := heldSelfScan(t, h)
			old := buildBoxTree(scan.boxes, true)
			old.oriented, old.slotWorld = scan.oriented, scan.slotWorld
			if old.oriented {
				old.slotWorld = make([][2]r3.Vec, len(old.facets))
				for s, facet := range old.facets {
					old.slotWorld[s] = e.triBox(facet)
				}
			}
			requireHeldTreeSound(t, &scan, scan.boxes)
			requireHeldTreeSound(t, &old, old.boxes)
			require.Len(t, scan.facets, len(old.facets))
			require.Len(t, scan.nodes, len(old.nodes))

			wantPairs, wantIterations := heldSelfCandidates(t, &old)
			gotPairs, gotIterations := heldSelfCandidates(t, &scan)
			require.Equal(t, len(wantPairs), len(gotPairs), "candidate pairs")
			for pair, n := range wantPairs {
				require.Equal(t, 1, n, "old tree visits pair %v once", pair)
				require.Equal(t, 1, gotPairs[pair], "scan tree visits pair %v once", pair)
			}
			switch mesh.name {
			case "revolved", "revolved-oriented":
				require.Less(t, gotIterations, wantIterations)
			case "coarse-oriented":
				require.Less(t, 2*gotIterations, wantIterations,
					"scan tree iterations %d, lower-corner tree %d", gotIterations, wantIterations)
			}

			run := func(tree *heldBoxTree, tasks []heldNodePair, scan func(*heldEmbedding, []heldNodePair, *heldBoxTree) (bool, error)) bool {
				e := newHeldEmbeddingScan(h, proofbound.NewWorkBudget(t.Context()), true)
				e.pollErr = t.Context().Err
				e.pollInterval, e.pollRemaining = proofbound.WorkPollInterval, proofbound.WorkPollInterval
				clean, err := scan(e, tasks, tree)
				require.NoError(t, err)
				return clean
			}
			oldTasks, err := old.selfPairTasks(t.Context(), 64)
			require.NoError(t, err)
			want := run(&old, oldTasks, (*heldEmbedding).cleanSelfPairsReference)
			require.Equal(t, want, run(&scan, tasks, (*heldEmbedding).cleanSelfPairs),
				fmt.Sprintf("%s: accept/refuse answer", mesh.name))
		})
	}
}

func heldContains(outer, inner [2]r3.Vec) bool {
	return outer[0].X <= inner[0].X && outer[0].Y <= inner[0].Y && outer[0].Z <= inner[0].Z &&
		outer[1].X >= inner[1].X && outer[1].Y >= inner[1].Y && outer[1].Z >= inner[1].Z
}

// requireHeldTreeSound checks that every node box contains its children's boxes
// and the boxes of the facets it owns, that tree.facets holds each non-NaN
// input facet exactly once, and that the stored leaf-order boxes match. It
// reports only counts and indices.
func requireHeldTreeSound(t *testing.T, tree *heldBoxTree, boxes [][2]r3.Vec) {
	t.Helper()
	want := 0
	for _, b := range boxes {
		if !heldBoxHasNaN(b) {
			want++
		}
	}
	require.Len(t, tree.facets, want, "facet count")
	seen := make([]int, len(boxes))
	for _, f := range tree.facets {
		require.True(t, f >= 0 && f < len(boxes), "facet index %d out of range", f)
		seen[f]++
	}
	for i, n := range seen {
		if heldBoxHasNaN(boxes[i]) {
			require.Zero(t, n, "NaN facet %d kept", i)
		} else {
			require.Equal(t, 1, n, "facet %d occurrences", i)
		}
	}
	for i, node := range tree.nodes {
		if node.leafStart < 0 {
			for _, c := range [2]int{node.left, node.right} {
				require.True(t, c >= 0 && c < len(tree.nodes), "node %d child %d out of range", i, c)
				require.True(t, heldContains(node.box, tree.nodes[c].box), "node %d does not contain child %d", i, c)
			}
			continue
		}
		require.LessOrEqual(t, node.leafStart+node.leafCount, len(tree.facets), "node %d leaf range", i)
		for s := node.leafStart; s < node.leafStart+node.leafCount; s++ {
			require.True(t, heldContains(node.box, boxes[tree.facets[s]]), "node %d does not contain facet %d", i, tree.facets[s])
		}
	}
	if tree.slotBoxes != nil {
		require.Len(t, tree.slotBoxes, len(tree.facets))
		for s, f := range tree.facets {
			require.True(t, tree.slotBoxes[s] == boxes[f], "slot %d box differs from facet %d", s, f)
		}
	}
	if tree.slotWorld != nil {
		require.Len(t, tree.slotWorld, len(tree.facets))
		for s := range tree.slotWorld {
			require.False(t, heldBoxHasNaN(tree.slotWorld[s]), "slot %d world box has NaN", s)
		}
	}
}

// TestHeldTreeTiesAndInfiniteBoxes builds both tree kinds on boxes with tied
// centres, and on boxes with an infinite extent and a NaN box, twice each. The
// two builds must agree on facet order and the tree must stay sound.
func TestHeldTreeTiesAndInfiniteBoxes(t *testing.T) {
	t.Parallel()
	inf := math.Inf(1)
	tied := make([][2]r3.Vec, 0, 80)
	for i := range 80 {
		// Every box is centred at the origin, so every split key ties.
		h := float64(1 + i%4)
		tied = append(tied, [2]r3.Vec{r3.NewVec(-h, -h, -h), r3.NewVec(h, h, h)})
	}
	infinite := make([][2]r3.Vec, 0, 70)
	for i := range 70 {
		x := float64(i % 7)
		infinite = append(infinite, [2]r3.Vec{r3.NewVec(x, x, 0), r3.NewVec(x+1, x+1, 1)})
	}
	infinite[3] = [2]r3.Vec{r3.NewVec(-inf, 0, 0), r3.NewVec(inf, 1, 1)}
	infinite[10] = [2]r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(inf, inf, inf)}
	infinite[20] = [2]r3.Vec{r3.NewVec(math.NaN(), 0, 0), r3.NewVec(1, 1, 1)}
	for name, boxes := range map[string][][2]r3.Vec{"tied": tied, "infinite": infinite} {
		for _, byArea := range []bool{false, true} {
			t.Run(fmt.Sprintf("%s/area=%v", name, byArea), func(t *testing.T) {
				first := buildBoxTreeSplit(boxes, true, byArea)
				second := buildBoxTreeSplit(boxes, true, byArea)
				requireHeldTreeSound(t, &first, boxes)
				requireHeldTreeSound(t, &second, boxes)
				require.Equal(t, len(first.facets), len(second.facets))
				for s := range first.facets {
					require.Equal(t, first.facets[s], second.facets[s], "slot %d differs between builds", s)
				}
			})
		}
	}
}
