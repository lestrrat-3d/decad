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
			for s := a.leafStart; s < a.leafStart+a.leafCount; s++ {
				box := tree.slotBoxes[s]
				// The two leaf hulls may overlap even when this facet is
				// disjoint from the entire other leaf. Reject that row first.
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
