package meshbool

import (
	"context"

	"github.com/lestrrat-3d/r3"
)

type heldNodePair struct{ a, b int }

// splitSelfPair partitions the unordered facet pairs of two nodes. A self
// node has left-left, left-right and right-right pairs. Distinct nodes have
// disjoint members, so splitting either one partitions their cross product.
// Leaves need no split. The tree is balanced by member count; the distance
// between child indices estimates that count when both nodes can split.
func (tree *heldBoxTree) splitSelfPair(pair heldNodePair) ([3]heldNodePair, int) {
	a, b := &tree.nodes[pair.a], &tree.nodes[pair.b]
	if pair.a == pair.b {
		if a.leafStart >= 0 {
			return [3]heldNodePair{}, 0
		}
		return [3]heldNodePair{{a.left, a.left}, {a.left, a.right}, {a.right, a.right}}, 3
	}
	if a.leafStart < 0 && (b.leafStart >= 0 || a.right-a.left >= b.right-b.left) {
		return [3]heldNodePair{{a.left, pair.b}, {a.right, pair.b}}, 2
	}
	if b.leafStart < 0 {
		return [3]heldNodePair{{pair.a, b.left}, {pair.a, b.right}}, 2
	}
	return [3]heldNodePair{}, 0
}

// selfPairTasks produces a bounded breadth-first partition of the tree's
// unordered facet pairs. Disjoint node boxes omit only disjoint facet pairs.
// A round starts with fewer than limit tasks and emits at most three per
// task, so this never retains the complete list of candidate facet pairs.
func (tree *heldBoxTree) selfPairTasks(ctx context.Context, limit int) ([]heldNodePair, error) {
	if len(tree.nodes) == 0 {
		return nil, ctx.Err()
	}
	tasks := []heldNodePair{{0, 0}}
	for len(tasks) < limit {
		next := make([]heldNodePair, 0, 3*len(tasks))
		expanded := false
		for _, pair := range tasks {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			if !BoxesOverlap(tree.nodes[pair.a].box, tree.nodes[pair.b].box) {
				continue
			}
			children, n := tree.splitSelfPair(pair)
			if n == 0 {
				next = append(next, pair)
				continue
			}
			expanded = true
			next = append(next, children[:n]...)
		}
		tasks = next
		if !expanded {
			break
		}
	}
	return tasks, ctx.Err()
}

// cleanSelfPairs checks every box-overlapping unordered facet pair in tasks.
// Its caller has checked every facet for collapse and established that every
// facet is a query. The node partition visits a pair once; a leaf paired with
// itself visits its upper triangle, and two different leaves visit their
// cross product. The held predicates and the ordered repair path are unchanged.
func (e *heldEmbedding) cleanSelfPairs(tasks []heldNodePair, tree *heldBoxTree) (bool, error) {
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
			end := b.leafStart + b.leafCount
			for s := a.leafStart; s < a.leafStart+a.leafCount; s++ {
				box := tree.slotBoxes[s]
				// Most rows and candidates miss, in no pattern a branch
				// predictor learns, so the box tests below are BoxesOverlap
				// read through axisMeet: the same six comparisons, all
				// evaluated and joined without a branch. Comparisons have no
				// side effects, so the answer and every step stay the same.
				lx, hx, ly, hy, lz, hz := box[0].X, box[1].X, box[0].Y, box[1].Y, box[0].Z, box[1].Z
				// The two leaf hulls may overlap even when this facet is
				// disjoint from the entire other leaf. Reject that row first.
				if !self && axisMeet(lx, hx, b.box[0].X, b.box[1].X)&axisMeet(ly, hy, b.box[0].Y, b.box[1].Y)&
					axisMeet(lz, hz, b.box[0].Z, b.box[1].Z) == 0 {
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
				for t := start; t < end; t++ {
					if err := step(); err != nil {
						return false, err
					}
					c := tree.slotBoxes[t]
					if axisMeet(lx, hx, c[0].X, c[1].X)&axisMeet(ly, hy, c[0].Y, c[1].Y)&
						axisMeet(lz, hz, c[0].Z, c[1].Z) == 0 {
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

// axisMeet is 1 when lo <= otherHi and otherLo <= hi, BoxesOverlap's two
// comparisons on one axis, and 0 otherwise; a comparison with a NaN is
// false, as there. The AND of the three axes' values is 1 exactly when
// BoxesOverlap is true. Each comparison becomes a flag, not a branch, and
// axisMeet inlines where a whole-box helper would not.
func axisMeet(lo, hi, otherLo, otherHi float64) uint8 {
	return meetBit(lo <= otherHi) & meetBit(otherLo <= hi)
}

// meetBit is 1 for true and 0 for false.
func meetBit(b bool) uint8 {
	if b {
		return 1
	}
	return 0
}
