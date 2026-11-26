package meshbool

import (
	"context"
	"fmt"
	"math"
	"math/bits"
	"slices"
	"sort"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// heldBoxTree indexes the unchanged facet boxes for the first read-only scan.
// Repair indexes boxes padded by the largest move (heldEmbedding.rtree), or
// uses the padded grid, because a moved vertex changes its box.
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
	return newScanBoxTree(boxes)
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
	tree := newScanBoxTree(boxes)
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

// newScanBoxTree is newBoxTree with each node split on the axis whose halves
// have the smaller total surface area, ordered by box centre. A held mesh of
// long thin facets leaves the longest-axis, lower-corner split with sibling
// nodes whose boxes overlap although few of their facets do; the area rule
// keeps siblings apart, so the self-pair scan reaches far fewer node and
// facet pairs. The tree holds the same facets, balanced by count, and its
// node boxes still contain everything below them, so every scan answer is
// the same (agent-docs/tessellation-design.md §1, the Embedding row).
func newScanBoxTree(boxes [][2]r3.Vec) heldBoxTree {
	return buildBoxTreeSplit(boxes, true, true)
}

// newQueryBoxTree builds a tree for read-only facet queries without keeping
// the leaf-order box copy used only by held self-pair scans.
func newQueryBoxTree(boxes [][2]r3.Vec) heldBoxTree {
	return buildBoxTree(boxes, false)
}

func buildBoxTree(boxes [][2]r3.Vec, keepSlotBoxes bool) heldBoxTree {
	return buildBoxTreeSplit(boxes, keepSlotBoxes, false)
}

func buildBoxTreeSplit(boxes [][2]r3.Vec, keepSlotBoxes, byArea bool) heldBoxTree {
	facets := make([]int, 0, len(boxes))
	for i, box := range boxes {
		// BoxesOverlap is false for a box with a NaN coordinate.
		if heldBoxHasNaN(box) {
			continue
		}
		facets = append(facets, i)
	}
	tree := heldBoxTree{nodes: make([]heldBoxNode, 0, heldBoxNodeCount(len(facets))), boxes: boxes,
		facets: make([]int, 0, len(facets))}
	if len(facets) > 0 {
		tree.build(boxes, facets, byArea)
	}
	if keepSlotBoxes {
		tree.slotBoxes = make([][2]r3.Vec, len(tree.facets))
		for s, facet := range tree.facets {
			tree.slotBoxes[s] = boxes[facet]
		}
	}
	return tree
}

func heldBoxNodeCount(facets int) int {
	if facets == 0 {
		return 0
	}
	if facets <= heldBoxLeafSize {
		return 1
	}
	partitions := 1 << (bits.Len(uint((facets-1)/heldBoxLeafSize)) - 1)
	leaves := partitions + min(partitions, facets-heldBoxLeafSize*partitions)
	return 2*leaves - 1
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

// appendWithin appends to dst every indexed box b with BoxesWithin(box, b, slack), in tree order, and returns dst
// and stack, which it uses as scratch. A node's box contains every box below it, and BoxesWithin never turns false
// as its second box grows: each side compares a corner of one box with a corner of the other moved by the same
// slack, through comparisons and one rounded subtraction, both monotone. So a node it rejects holds no box it keeps.
// A box with a NaN coordinate, which the tree leaves out, fails BoxesWithin too.
func (tree *heldBoxTree) appendWithin(dst, stack []int, box [2]r3.Vec, slack float64) ([]int, []int) {
	if len(tree.nodes) == 0 {
		return dst, stack
	}
	stack = append(stack[:0], 0)
	for len(stack) > 0 {
		last := len(stack) - 1
		node := tree.nodes[stack[last]]
		stack = stack[:last]
		if !BoxesWithin(box, node.box, slack) {
			continue
		}
		if node.leafStart < 0 {
			stack = append(stack, node.right, node.left)
			continue
		}
		for _, facet := range tree.facets[node.leafStart : node.leafStart+node.leafCount] {
			if BoxesWithin(box, tree.boxes[facet], slack) {
				dst = append(dst, facet)
			}
		}
	}
	return dst, stack
}

func heldBoxHasNaN(box [2]r3.Vec) bool {
	for _, point := range box {
		if math.IsNaN(point.X) || math.IsNaN(point.Y) || math.IsNaN(point.Z) {
			return true
		}
	}
	return false
}

func (tree *heldBoxTree) build(boxes [][2]r3.Vec, facets []int, byArea bool) int {
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
	if byArea {
		axis = bestAreaSplitAxis(facets, boxes, middle)
	}
	selectHeldBoxMedian(facets, boxes, axis, middle, byArea)
	tree.nodes[index].left = tree.build(boxes, facets[:middle], byArea)
	tree.nodes[index].right = tree.build(boxes, facets[middle:], byArea)
	return index
}

// bestAreaSplitAxis returns the axis whose centre-ordered median split of
// facets has the smaller total surface area of its two halves' boxes, the
// lowest axis on a tie or a non-finite area. Any axis yields a correct tree;
// this one only makes the scan cheaper. It leaves facets partitioned on the
// last axis it tried.
func bestAreaSplitAxis(facets []int, boxes [][2]r3.Vec, middle int) int {
	best, bestCost := 0, math.Inf(1)
	for axis := range 3 {
		selectHeldBoxMedian(facets, boxes, axis, middle, true)
		cost := heldBoxArea(boxes, facets[:middle]) + heldBoxArea(boxes, facets[middle:])
		if cost < bestCost {
			best, bestCost = axis, cost
		}
	}
	return best
}

// heldBoxArea is the surface area of the box bounding the boxes of facets.
func heldBoxArea(boxes [][2]r3.Vec, facets []int) float64 {
	box := boxes[facets[0]]
	for _, f := range facets[1:] {
		o := boxes[f]
		box[0].X, box[0].Y, box[0].Z = min(box[0].X, o[0].X), min(box[0].Y, o[0].Y), min(box[0].Z, o[0].Z)
		box[1].X, box[1].Y, box[1].Z = max(box[1].X, o[1].X), max(box[1].Y, o[1].Y), max(box[1].Z, o[1].Z)
	}
	dx, dy, dz := box[1].X-box[0].X, box[1].Y-box[0].Y, box[1].Z-box[0].Z
	return dx*dy + dy*dz + dz*dx
}

// heldBoxOrderKey is the coordinate on axis that selectHeldBoxMedian orders
// a box by: its lower corner, or with byCentre the sum of its corners, which
// orders as the centre does. A sum that is NaN (opposite infinities) falls
// back to the lower corner.
func heldBoxOrderKey(box [2]r3.Vec, axis int, byCentre bool) float64 {
	lo := heldBoxAxis(box[0], axis)
	if !byCentre {
		return lo
	}
	if sum := lo + heldBoxAxis(box[1], axis); sum == sum {
		return sum
	}
	return lo
}

func selectHeldBoxMedian(facets []int, boxes [][2]r3.Vec, axis, middle int, byCentre bool) {
	less := func(a, b int) bool {
		x := heldBoxOrderKey(boxes[a], axis, byCentre)
		y := heldBoxOrderKey(boxes[b], axis, byCentre)
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

// visitOverlapsWork charges each node and leaf candidate before inspecting it.
// The caller supplies a reusable stack with room for every tree node.
func (tree *heldBoxTree) visitOverlapsWork(box [2]r3.Vec, step func() error, stack []int,
	visit func(int) error) error {
	if len(tree.nodes) == 0 {
		return step()
	}
	if cap(stack) < len(tree.nodes) {
		return fmt.Errorf("%w: bounded box tree stack is too small", decaderr.ErrUnsupported)
	}
	stack = append(stack[:0], 0)
	for len(stack) > 0 {
		if err := step(); err != nil {
			return err
		}
		last := len(stack) - 1
		node := tree.nodes[stack[last]]
		stack = stack[:last]
		if !BoxesOverlap(box, node.box) {
			continue
		}
		if node.leafStart >= 0 {
			for _, facet := range tree.facets[node.leafStart : node.leafStart+node.leafCount] {
				if err := step(); err != nil {
					return err
				}
				if BoxesOverlap(box, tree.boxes[facet]) {
					if err := visit(facet); err != nil {
						return err
					}
				}
			}
			continue
		}
		stack = append(stack, node.right, node.left)
	}
	return nil
}

// newBoxTreeWork builds an order-preserving balanced tree while charging each
// box inspection. The caller has reserved five records per source box.
func newBoxTreeWork(boxes [][2]r3.Vec, budget *proofbound.WorkBudget) (heldBoxTree, error) {
	return buildBoxTreeWork(boxes, budget, false)
}

// newSpatialBoxTreeWork partitions each node on its longest axis. Conform
// queries restore vertex-index order before running their exact predicates.
func newSpatialBoxTreeWork(boxes [][2]r3.Vec, budget *proofbound.WorkBudget) (heldBoxTree, error) {
	return buildBoxTreeWork(boxes, budget, true)
}

func buildBoxTreeWork(boxes [][2]r3.Vec, budget *proofbound.WorkBudget, spatial bool) (heldBoxTree, error) {
	n := uint64(len(boxes))
	if n > ^uint64(0)/2 {
		return heldBoxTree{}, fmt.Errorf("%w: box tree size overflow", decaderr.ErrUnsupported)
	}
	for _, reservation := range [][2]uint64{{n, uint64(unsafe.Sizeof(int(0)))}, {n, uint64(unsafe.Sizeof(int(0)))}, {2 * n, uint64(unsafe.Sizeof(heldBoxNode{}))}, {n, uint64(unsafe.Sizeof([2]r3.Vec{}))}} {
		if err := budget.ReserveBytesMul(reservation[0], reservation[1]); err != nil {
			return heldBoxTree{}, err
		}
	}
	ids := make([]int, 0, len(boxes))
	for i, b := range boxes {
		if err := budget.Step(); err != nil {
			return heldBoxTree{}, err
		}
		if !heldBoxHasNaN(b) {
			ids = append(ids, i)
		}
	}
	tree := heldBoxTree{boxes: boxes, nodes: make([]heldBoxNode, 0, 2*len(ids)), facets: make([]int, 0, len(ids))}
	var build func([]int) (int, error)
	build = func(part []int) (int, error) {
		if err := budget.Step(); err != nil {
			return 0, err
		}
		idx := len(tree.nodes)
		tree.nodes = append(tree.nodes, heldBoxNode{leafStart: -1})
		bound := boxes[part[0]]
		for _, id := range part[1:] {
			if err := budget.Step(); err != nil {
				return 0, err
			}
			other := boxes[id]
			bound[0].X = min(bound[0].X, other[0].X)
			bound[0].Y = min(bound[0].Y, other[0].Y)
			bound[0].Z = min(bound[0].Z, other[0].Z)
			bound[1].X = max(bound[1].X, other[1].X)
			bound[1].Y = max(bound[1].Y, other[1].Y)
			bound[1].Z = max(bound[1].Z, other[1].Z)
		}
		tree.nodes[idx].box = bound
		if len(part) <= heldBoxLeafSize {
			if spatial {
				if _, err := budget.StepN(uint64(len(part))); err != nil {
					return 0, err
				}
			}
			tree.nodes[idx].leafStart = len(tree.facets)
			tree.nodes[idx].leafCount = len(part)
			tree.facets = append(tree.facets, part...)
			return idx, nil
		}
		if spatial {
			if _, err := budget.StepN(8); err != nil {
				return 0, err
			}
			axis := 0
			extent := [3]float64{bound[1].X - bound[0].X, bound[1].Y - bound[0].Y, bound[1].Z - bound[0].Z}
			if extent[1] > extent[axis] {
				axis = 1
			}
			if extent[2] > extent[axis] {
				axis = 2
			}
			if err := sortHeldBoxFacetsWork(part, boxes, axis, budget); err != nil {
				return 0, err
			}
		}
		mid := len(part) / 2
		left, err := build(part[:mid])
		if err != nil {
			return 0, err
		}
		right, err := build(part[mid:])
		if err != nil {
			return 0, err
		}
		tree.nodes[idx].left, tree.nodes[idx].right = left, right
		return idx, nil
	}
	if len(ids) > 0 {
		if _, err := build(ids); err != nil {
			return heldBoxTree{}, err
		}
	}
	tree.slotBoxes = make([][2]r3.Vec, len(tree.facets))
	for i, id := range tree.facets {
		if err := budget.Step(); err != nil {
			return heldBoxTree{}, err
		}
		tree.slotBoxes[i] = boxes[id]
	}
	return tree, nil
}

// treeCandidates lists the facets whose padded box, at the time buildGrid ran,
// overlaps facet i's current box padded the same way. A move never takes a
// facet's box farther than pad from where it was, so two facets whose current
// boxes overlap each list the other, as the grid's cells do. The facets it
// leaves out have disjoint boxes, which pairOK answers as embedded. The list
// is in tree order and is reused by the next call.
func (e *heldEmbedding) treeCandidates(i int) ([]int, error) {
	tree := e.rtree
	b := e.triBox(i)
	pad := r3.NewVec(e.pad, e.pad, e.pad)
	box := [2]r3.Vec{b[0].Sub(pad), b[1].Add(pad)}
	out := e.cands[:0]
	if len(tree.nodes) == 0 {
		e.cands = out
		return out, nil
	}
	nodes, facets, boxes := tree.nodes, tree.facets, tree.boxes
	stack := append(e.rstack[:0], 0)
	for len(stack) > 0 {
		last := len(stack) - 1
		node := nodes[stack[last]]
		stack = stack[:last]
		if err := e.scanStep(); err != nil {
			e.rstack = stack
			return nil, err
		}
		if !BoxesOverlap(box, node.box) {
			continue
		}
		if node.leafStart < 0 {
			stack = append(stack, node.right, node.left)
			continue
		}
		for _, f := range facets[node.leafStart : node.leafStart+node.leafCount] {
			if BoxesOverlap(box, boxes[f]) {
				out = append(out, f)
			}
		}
	}
	e.rstack = stack
	e.cands = out
	return out, nil
}
