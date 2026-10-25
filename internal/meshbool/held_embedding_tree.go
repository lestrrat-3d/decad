package meshbool

import (
	"context"
	"math"
	"math/bits"
	"slices"
	"sort"

	"github.com/lestrrat-3d/r3"
)

// heldBoxTree indexes the unchanged facet boxes for the first read-only scan.
// Repair still uses the padded grid because a moved vertex changes its box.
type heldBoxTree struct {
	nodes  []heldBoxNode
	boxes  [][2]r3.Vec
	facets []int
	// queryCeiling is the highest queried facet index in each node, or the
	// facet count if it contains any non-query facet. Only cleanPairs uses it.
	queryCeiling []int
}

type heldBoxNode struct {
	box                  [2]r3.Vec
	left, right          int
	leafStart, leafCount int
}

// heldBoxLeafSize reduces tree traversal work while keeping leaf checks exact.
const heldBoxLeafSize = 16

func newHeldBoxTree(e *heldEmbedding) heldBoxTree {
	boxes := make([][2]r3.Vec, len(e.h.Tris))
	for i := range boxes {
		boxes[i] = e.triBox(i)
	}
	return newBoxTree(boxes)
}

// prepareQueryCeilings lets the read-only scan discard whole nodes containing
// only query facets already paired from their lower index. A non-query facet
// must still be compared with every query, whatever its own index.
func (tree *heldBoxTree) prepareQueryCeilings(ctx context.Context, queryFacet []bool) error {
	tree.queryCeiling = make([]int, len(tree.nodes))
	// Children follow their parent in nodes, so reverse order reads completed
	// child ceilings without another traversal or a recursive stack.
	for i, node := range slices.Backward(tree.nodes) {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		if node.leafStart < 0 {
			tree.queryCeiling[i] = max(tree.queryCeiling[node.left], tree.queryCeiling[node.right])
			continue
		}
		ceiling := -1
		for _, j := range tree.facets[node.leafStart : node.leafStart+node.leafCount] {
			if !queryFacet[j] {
				ceiling = len(queryFacet)
				break
			}
			ceiling = max(ceiling, j)
		}
		tree.queryCeiling[i] = ceiling
	}
	return ctx.Err()
}

func newBoxTree(boxes [][2]r3.Vec) heldBoxTree {
	facets := make([]int, 0, len(boxes))
	for i, box := range boxes {
		// BoxesOverlap is false for a box with a NaN coordinate.
		if heldBoxHasNaN(box) {
			continue
		}
		facets = append(facets, i)
	}
	tree := heldBoxTree{nodes: make([]heldBoxNode, 0, len(facets)), boxes: boxes,
		facets: make([]int, 0, len(facets))}
	if len(facets) > 0 {
		tree.build(boxes, facets)
	}
	return tree
}

func (tree *heldBoxTree) visitOverlaps(box [2]r3.Vec, poll func() error,
	visit func(int) error) error {
	if poll != nil {
		if err := poll(); err != nil {
			return err
		}
	}
	if len(tree.nodes) == 0 {
		return nil
	}
	stack := make([]int, 1, 64)
	work := 0
	for len(stack) > 0 {
		work++
		if work%256 == 0 && poll != nil {
			if err := poll(); err != nil {
				return err
			}
		}
		last := len(stack) - 1
		nodeIndex := stack[last]
		stack = stack[:last]
		node := tree.nodes[nodeIndex]
		if !BoxesOverlap(box, node.box) {
			continue
		}
		if node.leafStart >= 0 {
			for _, facet := range tree.facets[node.leafStart : node.leafStart+node.leafCount] {
				work++
				if work%256 == 0 && poll != nil {
					if err := poll(); err != nil {
						return err
					}
				}
				if !BoxesOverlap(box, tree.boxes[facet]) {
					continue
				}
				if err := visit(facet); err != nil {
					return err
				}
			}
			continue
		}
		stack = append(stack, node.right, node.left)
	}
	if poll != nil {
		return poll()
	}
	return nil
}

func heldBoxHasNaN(box [2]r3.Vec) bool {
	for _, point := range box {
		if math.IsNaN(point.X) || math.IsNaN(point.Y) || math.IsNaN(point.Z) {
			return true
		}
	}
	return false
}

func (tree *heldBoxTree) build(boxes [][2]r3.Vec, facets []int) int {
	index := len(tree.nodes)
	tree.nodes = append(tree.nodes, heldBoxNode{leafStart: -1})
	box := boxes[facets[0]]
	for _, facet := range facets[1:] {
		other := boxes[facet]
		box[0].X = math.Min(box[0].X, other[0].X)
		box[0].Y = math.Min(box[0].Y, other[0].Y)
		box[0].Z = math.Min(box[0].Z, other[0].Z)
		box[1].X = math.Max(box[1].X, other[1].X)
		box[1].Y = math.Max(box[1].Y, other[1].Y)
		box[1].Z = math.Max(box[1].Z, other[1].Z)
	}
	tree.nodes[index].box = box
	if len(facets) <= heldBoxLeafSize {
		tree.nodes[index].leafStart = len(tree.facets)
		tree.nodes[index].leafCount = len(facets)
		tree.facets = append(tree.facets, facets...)
		return index
	}
	axis := 0
	extent := [3]float64{box[1].X - box[0].X, box[1].Y - box[0].Y, box[1].Z - box[0].Z}
	if extent[1] > extent[axis] {
		axis = 1
	}
	if extent[2] > extent[axis] {
		axis = 2
	}
	middle := len(facets) / 2
	selectHeldBoxMedian(facets, boxes, axis, middle)
	tree.nodes[index].left = tree.build(boxes, facets[:middle])
	tree.nodes[index].right = tree.build(boxes, facets[middle:])
	return index
}

func selectHeldBoxMedian(facets []int, boxes [][2]r3.Vec, axis, middle int) {
	less := func(a, b int) bool {
		x := heldBoxAxis(boxes[a][0], axis)
		y := heldBoxAxis(boxes[b][0], axis)
		return x < y || x == y && a < b
	}
	maxSteps := 2 * bits.Len(uint(len(facets)))
	for lo, hi, steps := 0, len(facets)-1, 0; lo < hi; steps++ {
		if steps == maxSteps {
			sort.Slice(facets[lo:hi+1], func(i, j int) bool {
				return less(facets[lo+i], facets[lo+j])
			})
			return
		}
		mid := lo + (hi-lo)/2
		pivot := medianHeldBoxFacet(facets, lo, mid, hi, less)
		facets[pivot], facets[hi] = facets[hi], facets[pivot]
		store := lo
		for scan := lo; scan < hi; scan++ {
			if less(facets[scan], facets[hi]) {
				facets[store], facets[scan] = facets[scan], facets[store]
				store++
			}
		}
		facets[store], facets[hi] = facets[hi], facets[store]
		switch {
		case store == middle:
			return
		case store < middle:
			lo = store + 1
		default:
			hi = store - 1
		}
	}
}

func medianHeldBoxFacet(facets []int, lo, mid, hi int, less func(int, int) bool) int {
	a, b, c := facets[lo], facets[mid], facets[hi]
	switch {
	case less(a, b):
		if less(b, c) {
			return mid
		}
		if less(a, c) {
			return hi
		}
		return lo
	default:
		if less(a, c) {
			return lo
		}
		if less(b, c) {
			return hi
		}
		return mid
	}
}

func heldBoxAxis(point r3.Vec, axis int) float64 {
	switch axis {
	case 0:
		return point.X
	case 1:
		return point.Y
	default:
		return point.Z
	}
}
