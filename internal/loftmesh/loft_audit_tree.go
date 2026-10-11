package loftmesh

import (
	"slices"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// treeScan groups the member boxes in a balanced hierarchy. A node box is
// the coordinatewise min/max of its descendants. Disjoint node boxes prove
// every descendant pair apart; leaf pairs still use BoxesOverlap, including
// contact at a shared box boundary.
type treeScan struct {
	boxes [][2]r3.Vec
	order []int32
	nodes []treeNode
}

type treeNode struct {
	box         [2]r3.Vec
	start, end  int
	left, right int
}

const treeLeafSize = 8

func newTreeScan(boxes [][2]r3.Vec, members []int) treeScan {
	t := treeScan{boxes: boxes, order: make([]int32, len(members))}
	for k, i := range members {
		t.order[k] = int32(i)
	}
	if len(members) != 0 {
		t.build(0, len(members))
	}
	return t
}

func (t *treeScan) build(start, end int) int {
	box := t.boxes[t.order[start]]
	for _, i := range t.order[start+1 : end] {
		b := t.boxes[i]
		box[0] = r3.Vec{X: min(box[0].X, b[0].X), Y: min(box[0].Y, b[0].Y), Z: min(box[0].Z, b[0].Z)}
		box[1] = r3.Vec{X: max(box[1].X, b[1].X), Y: max(box[1].Y, b[1].Y), Z: max(box[1].Z, b[1].Z)}
	}
	index := len(t.nodes)
	t.nodes = append(t.nodes, treeNode{box: box, start: start, end: end, left: -1, right: -1})
	if end-start <= treeLeafSize {
		return index
	}
	axis := 0
	for a := 1; a < 3; a++ {
		if vecAxis(box[1], a)-vecAxis(box[0], a) > vecAxis(box[1], axis)-vecAxis(box[0], axis) {
			axis = a
		}
	}
	slices.SortFunc(t.order[start:end], func(a, b int32) int {
		alo, ahi := boxAxis(t.boxes[a], axis)
		blo, bhi := boxAxis(t.boxes[b], axis)
		// Comparing halves avoids overflow in a midpoint sum.
		amid, bmid := alo/2+ahi/2, blo/2+bhi/2
		switch {
		case amid < bmid:
			return -1
		case amid > bmid:
			return 1
		default:
			return int(a) - int(b)
		}
	})
	mid := start + (end-start)/2
	left := t.build(start, mid)
	right := t.build(mid, end)
	t.nodes[index].left, t.nodes[index].right = left, right
	return index
}

// visit reports each overlapping member pair once. The same recursion is
// used by workBound, so selection can compare its exact scan count before
// the audit allocates a candidate list. A node visit and a leaf pair each
// consume one unit of the work ceiling and poll the shared budget.
func (t treeScan) visit(budget *proofbound.WorkBudget, limit uint64, fn func(i, j int)) (uint64, bool, error) {
	if len(t.nodes) == 0 {
		return 0, false, nil
	}
	var scanned uint64
	var walk func(a, b int) (bool, error)
	step := func() (bool, error) {
		if err := budget.Step(); err != nil {
			return false, err
		}
		scanned++
		return scanned > limit, nil
	}
	walk = func(a, b int) (bool, error) {
		if exceeded, err := step(); exceeded || err != nil {
			return exceeded, err
		}
		na, nb := t.nodes[a], t.nodes[b]
		if !meshbool.BoxesOverlap(na.box, nb.box) {
			return false, nil
		}
		if na.left < 0 && nb.left < 0 {
			for ia := na.start; ia < na.end; ia++ {
				begin := nb.start
				if a == b {
					begin = ia + 1
				}
				for ib := begin; ib < nb.end; ib++ {
					if exceeded, err := step(); exceeded || err != nil {
						return exceeded, err
					}
					i, j := int(t.order[ia]), int(t.order[ib])
					if !meshbool.BoxesOverlap(t.boxes[i], t.boxes[j]) {
						continue
					}
					if i > j {
						i, j = j, i
					}
					fn(i, j)
				}
			}
			return false, nil
		}
		if a == b {
			for _, pair := range [][2]int{{na.left, na.left}, {na.left, na.right}, {na.right, na.right}} {
				if exceeded, err := walk(pair[0], pair[1]); exceeded || err != nil {
					return exceeded, err
				}
			}
			return false, nil
		}
		if nb.left < 0 || (na.left >= 0 && na.end-na.start >= nb.end-nb.start) {
			if exceeded, err := walk(na.left, b); exceeded || err != nil {
				return exceeded, err
			}
			return walk(na.right, b)
		}
		if exceeded, err := walk(a, nb.left); exceeded || err != nil {
			return exceeded, err
		}
		return walk(a, nb.right)
	}
	exceeded, err := walk(0, 0)
	return scanned, exceeded, err
}

func (t treeScan) workBound(limit uint64) (uint64, bool) {
	if len(t.nodes) == 0 {
		return 0, true
	}
	var scanned uint64
	var walk func(a, b int) bool
	walk = func(a, b int) bool {
		scanned++
		if scanned > limit {
			return false
		}
		na, nb := t.nodes[a], t.nodes[b]
		if !meshbool.BoxesOverlap(na.box, nb.box) {
			return true
		}
		if na.left < 0 && nb.left < 0 {
			pairs := uint64((na.end - na.start) * (nb.end - nb.start))
			if a == b {
				n := uint64(na.end - na.start)
				pairs = n * (n - 1) / 2
			}
			scanned += pairs
			return scanned <= limit
		}
		if a == b {
			return walk(na.left, na.left) && walk(na.left, na.right) && walk(na.right, na.right)
		}
		if nb.left < 0 || (na.left >= 0 && na.end-na.start >= nb.end-nb.start) {
			return walk(na.left, b) && walk(na.right, b)
		}
		return walk(a, nb.left) && walk(a, nb.right)
	}
	ok := walk(0, 0)
	return scanned, ok
}
