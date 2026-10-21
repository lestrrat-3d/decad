package planar

import (
	"math"
	"slices"
)

// boxTree is a static bounding-volume tree over a prep's float boxes. A query
// returns, in ascending index order, every box that is not floatApart from the
// query box (query), or that the distance scan's float gap does not put past
// a bound (queryWithin). That is the set a full scan with the same pre-test
// keeps, so a scan visits the same pairs in the same order without touching
// the rest.
type boxTree struct {
	boxes []floatBox
	order []int
	nodes []boxTreeNode
}

// boxTreeNode holds the union box of order[lo:hi]. An inner node has two
// children; a leaf has none.
type boxTreeNode struct {
	box         floatBox
	lo, hi      int
	left, right int
}

// boxTreeLeaf is the largest run of boxes one leaf holds.
const boxTreeLeaf = 8

func newBoxTree(boxes []floatBox) *boxTree {
	t := &boxTree{boxes: boxes, order: make([]int, len(boxes))}
	for i := range t.order {
		t.order[i] = i
	}
	if len(boxes) > 0 {
		t.build(0, len(boxes))
	}
	return t
}

// build adds the node for order[lo:hi] and returns its index.
func (t *boxTree) build(lo, hi int) int {
	node := boxTreeNode{box: t.boxes[t.order[lo]], lo: lo, hi: hi, left: -1, right: -1}
	cmin, cmax := [3]float64{}, [3]float64{}
	for i, at := range t.order[lo:hi] {
		b := t.boxes[at]
		for axis := range 3 {
			node.box.lo[axis] = math.Min(node.box.lo[axis], b.lo[axis])
			node.box.hi[axis] = math.Max(node.box.hi[axis], b.hi[axis])
			c := boxCentre(b, axis)
			if i == 0 || c < cmin[axis] {
				cmin[axis] = c
			}
			if i == 0 || c > cmax[axis] {
				cmax[axis] = c
			}
		}
	}
	at := len(t.nodes)
	t.nodes = append(t.nodes, node)
	if hi-lo <= boxTreeLeaf {
		return at
	}
	axis := 0
	for a := 1; a < 3; a++ {
		if cmax[a]-cmin[a] > cmax[axis]-cmin[axis] {
			axis = a
		}
	}
	slices.SortFunc(t.order[lo:hi], func(x, y int) int {
		cx, cy := boxCentre(t.boxes[x], axis), boxCentre(t.boxes[y], axis)
		switch {
		case cx < cy:
			return -1
		case cx > cy:
			return 1
		}
		return x - y
	})
	mid := (lo + hi) / 2
	left := t.build(lo, mid)
	right := t.build(mid, hi)
	t.nodes[at].left, t.nodes[at].right = left, right
	return at
}

// boxCentre is the box's midpoint on axis; a box with a non-finite side sorts
// as zero. It only steers the split, never the answer.
func boxCentre(b floatBox, axis int) float64 {
	c := (b.lo[axis] + b.hi[axis]) / 2
	if math.IsNaN(c) || math.IsInf(c, 0) {
		return 0
	}
	return c
}

// query appends to out, in ascending order, every index whose box is not
// floatApart from q. A node box holds its boxes, so a node apart from q holds
// none that is not.
func (t *boxTree) query(q floatBox, out []int) []int {
	out = out[:0]
	if len(t.nodes) == 0 {
		return out
	}
	var stack [64]int
	top := 1
	for top > 0 {
		top--
		n := &t.nodes[stack[top]]
		if floatApart(n.box, q) {
			continue
		}
		if n.left < 0 {
			for _, at := range t.order[n.lo:n.hi] {
				if !floatApart(t.boxes[at], q) {
					out = append(out, at)
				}
			}
			continue
		}
		stack[top], stack[top+1] = n.left, n.right
		top += 2
	}
	slices.Sort(out)
	return out
}

// queryWithin appends to out, in ascending order, every index whose box the
// float pre-test of the distance scan (gapSquaredBelowLimit) does not put past
// limit from q. A node box holds its boxes and the gap only grows as a box
// shrinks, so a node past limit holds none that is not.
func (t *boxTree) queryWithin(q floatBox, limit float64, out []int) []int {
	out = out[:0]
	if len(t.nodes) == 0 {
		return out
	}
	var stack [64]int
	top := 1
	for top > 0 {
		top--
		n := &t.nodes[stack[top]]
		if gapSquaredBelowLimit(q, n.box, limit) > limit {
			continue
		}
		if n.left < 0 {
			for _, at := range t.order[n.lo:n.hi] {
				if !(gapSquaredBelowLimit(q, t.boxes[at], limit) > limit) {
					out = append(out, at)
				}
			}
			continue
		}
		stack[top], stack[top+1] = n.left, n.right
		top += 2
	}
	slices.Sort(out)
	return out
}
