package meshbool

import (
	"fmt"
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
