package meshbool

import (
	"math"
	"sort"

	"github.com/lestrrat-3d/r3"
)

// heldBoxTree indexes the unchanged facet boxes for the first read-only scan.
// Repair still uses the padded grid because a moved vertex changes its box.
type heldBoxTree struct {
	nodes []heldBoxNode
}

type heldBoxNode struct {
	box         [2]r3.Vec
	left, right int
	facet       int
}

func newHeldBoxTree(e *heldEmbedding) heldBoxTree {
	boxes := make([][2]r3.Vec, len(e.h.Tris))
	for i := range boxes {
		boxes[i] = e.triBox(i)
	}
	return newBoxTree(boxes)
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
	tree := heldBoxTree{nodes: make([]heldBoxNode, 0, 2*len(facets))}
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
		if node.facet >= 0 {
			if err := visit(node.facet); err != nil {
				return err
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
	tree.nodes = append(tree.nodes, heldBoxNode{})
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
	if len(facets) == 1 {
		tree.nodes[index].facet = facets[0]
		return index
	}
	tree.nodes[index].facet = -1
	axis := 0
	extent := [3]float64{box[1].X - box[0].X, box[1].Y - box[0].Y, box[1].Z - box[0].Z}
	if extent[1] > extent[axis] {
		axis = 1
	}
	if extent[2] > extent[axis] {
		axis = 2
	}
	sort.Slice(facets, func(i, j int) bool {
		a := heldBoxAxis(boxes[facets[i]][0], axis)
		b := heldBoxAxis(boxes[facets[j]][0], axis)
		return a < b || a == b && facets[i] < facets[j]
	})
	middle := len(facets) / 2
	tree.nodes[index].left = tree.build(boxes, facets[:middle])
	tree.nodes[index].right = tree.build(boxes, facets[middle:])
	return index
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
