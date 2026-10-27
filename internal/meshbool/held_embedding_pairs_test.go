package meshbool

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/rand/v2"
	"strings"
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

// TestAxisMeetMatchesBoxesOverlap reads axisMeet against BoxesOverlap on one
// axis over every arrangement of signed zeros, infinities, NaN and ordinary
// endpoints, so the self-pair scan's box tests answer as BoxesOverlap does.
func TestAxisMeetMatchesBoxesOverlap(t *testing.T) {
	t.Parallel()
	values := []float64{math.Inf(-1), -1, math.Copysign(0, -1), 0, 0.5, 1, math.Inf(1), math.NaN()}
	for _, lo := range values {
		for _, hi := range values {
			for _, otherLo := range values {
				for _, otherHi := range values {
					got := axisMeet(lo, hi, otherLo, otherHi) == 1
					a := [2]r3.Vec{{X: lo}, {X: hi}}
					b := [2]r3.Vec{{X: otherLo}, {X: otherHi}}
					require.Equal(t, BoxesOverlap(a, b), got, "[%v, %v] against [%v, %v]", lo, hi, otherLo, otherHi)
				}
			}
		}
	}
}

// cleanSelfPairsReference is cleanSelfPairs with every box test read through
// BoxesOverlap, the scan TestHeldSelfPairsMatchReferenceScan compares with.
func (e *heldEmbedding) cleanSelfPairsReference(tasks []heldNodePair, tree *heldBoxTree) (bool, error) {
	remaining, interval, poll := e.pollRemaining, e.pollInterval, e.pollErr
	defer func() { e.pollRemaining = remaining }()
	step := func() error {
		remaining--
		if remaining != 0 {
			return nil
		}
		remaining = interval
		return poll()
	}
	stack := make([]heldNodePair, 0, 64)
	for _, task := range tasks {
		stack = append(stack[:0], task)
		for len(stack) > 0 {
			last := len(stack) - 1
			pair := stack[last]
			stack = stack[:last]
			if err := step(); err != nil {
				return false, err
			}
			a, b := &tree.nodes[pair.a], &tree.nodes[pair.b]
			if !BoxesOverlap(a.box, b.box) {
				continue
			}
			if a.leafStart < 0 || b.leafStart < 0 {
				children, n := tree.splitSelfPair(pair)
				stack = append(stack, children[:n]...)
				continue
			}
			self := pair.a == pair.b
			for s := a.leafStart; s < a.leafStart+a.leafCount; s++ {
				box := tree.slotBoxes[s]
				if !self && !BoxesOverlap(box, b.box) {
					continue
				}
				var world [2]r3.Vec
				if tree.oriented {
					world = tree.slotWorld[s]
				}
				i := tree.facets[s]
				start := b.leafStart
				if self {
					start = s + 1
				}
				for t := start; t < b.leafStart+b.leafCount; t++ {
					if err := step(); err != nil {
						return false, err
					}
					if !BoxesOverlap(box, tree.slotBoxes[t]) {
						continue
					}
					if tree.oriented && !BoxesOverlap(world, tree.slotWorld[t]) {
						continue
					}
					j := tree.facets[t]
					lo, hi := i, j
					if lo > hi {
						lo, hi = hi, lo
					}
					if !e.pairEmbeddedOverlapping(lo, hi) {
						return false, nil
					}
				}
			}
		}
	}
	return true, nil
}

// heldSelfScan prepares a scan of every facet of h as parallelClean does: each
// facet's projection, box and corner cones, the scan tree with its world boxes,
// and its node-pair tasks.
func heldSelfScan(t testing.TB, h HeldRounding) (*heldEmbedding, heldBoxTree, []heldNodePair) {
	t.Helper()
	e := newHeldEmbeddingScan(h, proofbound.NewWorkBudget(t.Context()), true)
	for i := range h.Tris {
		e.projection(i)
		e.triBox(i)
		for k := range 3 {
			e.coneBox(i, k)
		}
	}
	tree, err := newHeldScanTree(t.Context(), e)
	require.NoError(t, err)
	if tree.oriented {
		tree.slotWorld = make([][2]r3.Vec, len(tree.facets))
		for s, facet := range tree.facets {
			tree.slotWorld[s] = e.triBox(facet)
		}
	}
	tasks, err := tree.selfPairTasks(t.Context(), 64)
	require.NoError(t, err)
	return e, tree, tasks
}

// heldTinyTube is heldRevolvedTube scaled by 2⁻³⁴⁰, exactly. Its coordinates
// lie below the range the projection test trusts to floats, so every pair
// sharing no corner reaches the exact plane signs, and a scan that keeps
// sideSigns records, per pair, the vertices it read against each plane.
func heldTinyTube(segments int) HeldRounding {
	h := heldRevolvedTube(segments, false)
	for v, p := range h.Verts {
		h.Verts[v] = p.Scale(0x1p-340)
	}
	return h
}

// TestHeldSelfPairsMatchReferenceScan runs cleanSelfPairs and the reference
// scan on clean, crossing and collapsed-facet meshes, each polled at several
// intervals and stopped at several polls, with and without the shared side
// signs of a worker copy, and requires the same answer, error, poll count and
// countdown, and the same predicate caches afterwards: the facets whose
// normals, rounded normals, lifts and planes were read, and each plane sign
// read, so the same pairs were decided in the same order.
func TestHeldSelfPairsMatchReferenceScan(t *testing.T) {
	t.Parallel()
	skew := [3]r3.Vec{{X: 2, Y: 3, Z: 5}, {X: -3, Y: 2}, {X: 1, Z: -2}}
	meshes := []struct {
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
		{"tube", func(*testing.T) HeldRounding { return heldRevolvedTube(72, false) }},
		{"tube-oriented", func(*testing.T) HeldRounding { return heldRevolvedTube(72, true) }},
		{"tube-tiny", func(*testing.T) HeldRounding { return heldTinyTube(72) }},
		{"coarse-oriented", func(*testing.T) HeldRounding { return heldCoarseRevolve(8, 72, true) }},
	}
	stop := errors.New("stop")
	for _, mesh := range meshes {
		for _, signs := range []bool{false, true} {
			// WorkPollInterval/4 is what each of four workers polls at.
			for _, interval := range []int{1, 5, proofbound.WorkPollInterval / 4} {
				// Zero never stops; k stops the scan at its k-th poll.
				for _, stopAt := range []int{0, 1, 7} {
					name := fmt.Sprintf("%s/signs=%v/interval=%d/stop=%d", mesh.name, signs, interval, stopAt)
					t.Run(name, func(t *testing.T) {
						h := mesh.h(t)
						type outcome struct {
							clean     bool
							err       error
							polls     int
							remaining int
						}
						run := func(scan func(*heldEmbedding, []heldNodePair, *heldBoxTree) (bool, error)) (outcome, *heldEmbedding) {
							e, tree, tasks := heldSelfScan(t, h)
							if signs {
								e.sideSigns = map[[2]int]int{}
							}
							var out outcome
							e.pollErr = func() error {
								out.polls++
								if out.polls == stopAt {
									return stop
								}
								return nil
							}
							e.pollInterval, e.pollRemaining = interval, interval
							out.clean, out.err = scan(e, tasks, &tree)
							out.remaining = e.pollRemaining
							return out, e
						}
						want, ref := run((*heldEmbedding).cleanSelfPairsReference)
						switch {
						case stopAt == 0:
							require.NoError(t, want.err, "the reference scan reads the fixture")
							require.Equal(t, !strings.Contains(mesh.name, "crossing"), want.clean,
								"the reference scan reads the fixture")
						case interval == 1 && stopAt == 1:
							require.ErrorIs(t, want.err, stop, "the reference scan polls at its first step")
						}
						if signs && mesh.name == "tube-tiny" && stopAt == 0 {
							require.Greater(t, len(ref.sideSigns), len(h.Tris), "the tiny tube reaches the plane signs")
						}
						got, e := run((*heldEmbedding).cleanSelfPairs)
						require.Equal(t, want, got)
						requireSameFlags(t, "normals read", ref.normOK, e.normOK)
						requireSameFlags(t, "rounded normals read", ref.fnormOK, e.fnormOK)
						requireSameFlags(t, "vertex lifts read", ref.xvOK, e.xvOK)
						require.Len(t, e.plane, len(ref.plane))
						for i := range ref.plane {
							if ref.plane[i] != e.plane[i] {
								require.Failf(t, "planes interned differ", "facet %d: %d, want %d", i, e.plane[i], ref.plane[i])
							}
						}
						require.Equal(t, len(ref.planes), len(e.planes), "distinct planes")
						require.Equal(t, len(ref.sideSigns), len(e.sideSigns), "plane signs read")
						for key, sign := range ref.sideSigns {
							if got, ok := e.sideSigns[key]; !ok || got != sign {
								require.Failf(t, "plane signs differ", "plane %d vertex %d", key[0], key[1])
							}
						}
					})
				}
			}
		}
	}
}

// requireSameFlags compares two cache flag slices and reports only how many
// entries differ and the first of them.
func requireSameFlags(t *testing.T, what string, want, got []bool) {
	t.Helper()
	require.Len(t, got, len(want), what)
	first, count := -1, 0
	for i := range want {
		if want[i] != got[i] {
			if first < 0 {
				first = i
			}
			count++
		}
	}
	if count > 0 {
		require.Failf(t, what+" differ", "%d entries differ, first at %d", count, first)
	}
}
