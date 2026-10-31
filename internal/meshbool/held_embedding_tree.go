package meshbool

import (
	"context"
	"math"
	"math/bits"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// heldBoxTree indexes the unchanged facet boxes for the first read-only scan.
// Repair still uses the padded grid because a moved vertex changes its box.
type heldBoxTree struct {
	nodes  []heldBoxNode
	boxes  [][2]r3.Vec
	facets []int
	// slotBoxes is boxes in leaf order: slotBoxes[s] is boxes[facets[s]].
	// The self-pair scan reads a leaf's boxes in order, not through facets.
	slotBoxes [][2]r3.Vec
	// slotWorld is, for an oriented tree, each facet's world box in leaf
	// order. Only parallelClean fills it, before its workers share the tree.
	slotWorld [][2]r3.Vec
	// queryCeiling is the highest queried facet index in each node, or the
	// facet count if it contains any non-query facet. Only cleanPairs uses it.
	queryCeiling []int
	oriented     bool
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

// newHeldScanTree can index three other linear projections of the same held
// triangles. Their exact dot products lie in the interval hull of the three
// corners, so disjoint closed boxes omit only disjoint triangles. Any nonfinite
// endpoint abandons the entire alternate coordinate system, including earlier
// projections, and builds the world tree instead.
func newHeldScanTree(ctx context.Context, e *heldEmbedding) (heldBoxTree, error) {
	if !orientedScanAxes(e.h.ScanAxes) {
		return newHeldBoxTree(e), nil
	}
	tree, ok, err := newOrientedScanTree(ctx, e.h)
	if err != nil || ok {
		return tree, err
	}
	return newHeldBoxTree(e), nil
}

// orientedScanAxes reports whether axes select an alternate projection, as
// opposed to none or a signed permutation of the world axes, which keep the
// world boxes.
func orientedScanAxes(axes [3]r3.Vec) bool {
	return axes != [3]r3.Vec{} && !coordinateScanAxes(axes)
}

// newOrientedScanTree is newHeldScanTree's alternate-axes tree. It reads only
// h's vertices, facets and axes, never the heldEmbedding caches, so it may run
// beside work that fills them. A false result, with a nil error, means some
// projected endpoint is not finite and the caller builds the world tree.
func newOrientedScanTree(ctx context.Context, h HeldRounding) (heldBoxTree, bool, error) {
	axes := h.ScanAxes
	projected := make([][2]r3.Vec, len(h.Verts))
	for i, vertex := range h.Verts {
		if i%proofbound.WorkPollInterval == 0 {
			if err := ctx.Err(); err != nil {
				return heldBoxTree{}, false, err
			}
		}
		var lo, hi [3]float64
		for k, axis := range axes {
			iv := FivDot(FivVecOf(vertex), FivVecOf(axis))
			if proofbound.IsNonFinite(iv.Lo) || proofbound.IsNonFinite(iv.Hi) {
				return heldBoxTree{}, false, nil
			}
			lo[k], hi[k] = iv.Lo, iv.Hi
		}
		projected[i] = [2]r3.Vec{r3.NewVec(lo[0], lo[1], lo[2]), r3.NewVec(hi[0], hi[1], hi[2])}
	}
	boxes := make([][2]r3.Vec, len(h.Tris))
	for i, tri := range h.Tris {
		if i%proofbound.WorkPollInterval == 0 {
			if err := ctx.Err(); err != nil {
				return heldBoxTree{}, false, err
			}
		}
		box := projected[tri[0]]
		// Every endpoint is finite, so the builtin min and max agree with
		// math.Min and math.Max bit for bit, signed zeros included.
		for _, v := range tri[1:] {
			p := projected[v]
			box[0] = r3.NewVec(min(box[0].X, p[0].X), min(box[0].Y, p[0].Y), min(box[0].Z, p[0].Z))
			box[1] = r3.NewVec(max(box[1].X, p[1].X), max(box[1].Y, p[1].Y), max(box[1].Z, p[1].Z))
		}
		boxes[i] = box
	}
	tree := newBoxTree(boxes)
	tree.oriented = true
	return tree, true, ctx.Err()
}

func coordinateScanAxes(axes [3]r3.Vec) bool {
	for _, axis := range axes {
		nonzero := 0
		for _, c := range [...]float64{axis.X, axis.Y, axis.Z} {
			if c == 0 {
				continue
			}
			if c != 1 && c != -1 {
				return false
			}
			nonzero++
		}
		if nonzero != 1 {
			return false
		}
	}
	return true
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
	return buildBoxTree(boxes, true)
}

// newQueryBoxTree builds a tree for read-only facet queries without keeping
// the leaf-order box copy used only by held self-pair scans.
func newQueryBoxTree(boxes [][2]r3.Vec) heldBoxTree {
	return buildBoxTree(boxes, false)
}

func buildBoxTree(boxes [][2]r3.Vec, keepSlotBoxes bool) heldBoxTree {
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
	if keepSlotBoxes {
		tree.slotBoxes = make([][2]r3.Vec, len(tree.facets))
		for s, facet := range tree.facets {
			tree.slotBoxes[s] = boxes[facet]
		}
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
	// newBoxTree admits no NaN box, so the builtin min and max agree with
	// math.Min and math.Max bit for bit, signed zeros and infinities included.
	for _, facet := range facets[1:] {
		other := boxes[facet]
		box[0].X = min(box[0].X, other[0].X)
		box[0].Y = min(box[0].Y, other[0].Y)
		box[0].Z = min(box[0].Z, other[0].Z)
		box[1].X = max(box[1].X, other[1].X)
		box[1].Y = max(box[1].Y, other[1].Y)
		box[1].Z = max(box[1].Z, other[1].Z)
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
