package loftmesh

import (
	"errors"
	"math"
	"reflect"
	"slices"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
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
	flat  *flatTree
}

type treeNode struct {
	box         [2]r3.Vec
	start, end  int
	left, right int
	maxID       int32
}

const treeLeafSize = 8

func newTreeScan(boxes [][2]r3.Vec, members []int) treeScan {
	t := treeScan{boxes: boxes, order: make([]int32, len(members))}
	for k, i := range members {
		t.order[k] = int32(i)
	}
	if len(members) != 0 {
		t.build(0, len(members), t.hasNaNMidpoint())
	}
	t.flat = newFlatTree(&t)
	return t
}

// hasNaNMidpoint reports whether a member's box has a NaN midpoint on some
// axis, which is the only way the build comparator stops being a strict total
// order. A NaN coordinate or a -Inf..+Inf extent makes one.
func (t *treeScan) hasNaNMidpoint() bool {
	for _, i := range t.order {
		b := t.boxes[i]
		for axis := range 3 {
			lo, hi := boxAxis(b, axis)
			if math.IsNaN(lo/2 + hi/2) {
				return true
			}
		}
	}
	return false
}

// build appends the node over order[start:end] and its subtree and returns the
// node's index. A node splits at the median of its widest axis. The comparator
// orders by box midpoint on that axis and breaks ties by member id, so it is a
// strict total order and the left half is exactly the smallest members, in the
// order a full sort of the node's range puts them. build therefore selects the
// median (selectNth) instead of sorting the range, and sorts a child's range
// only when that child is a leaf, because a leaf keeps the order its parent
// gave it. An inner child reorders its own range, so the order selection left
// in it does not matter, and a node's box, range and maxID do not depend on
// the order either. The tree equals the one built by a full sort at every
// node. fullSort is for member sets whose comparator is not a total order
// (NaN midpoints); there the result depends on the input order, so every node
// sorts its whole range, as it always has.
func (t *treeScan) build(start, end int, fullSort bool) int {
	box := t.boxes[t.order[start]]
	maxID := t.order[start]
	for _, i := range t.order[start+1 : end] {
		maxID = max(maxID, i)
		b := t.boxes[i]
		box[0] = r3.Vec{X: min(box[0].X, b[0].X), Y: min(box[0].Y, b[0].Y), Z: min(box[0].Z, b[0].Z)}
		box[1] = r3.Vec{X: max(box[1].X, b[1].X), Y: max(box[1].Y, b[1].Y), Z: max(box[1].Z, b[1].Z)}
	}
	index := len(t.nodes)
	t.nodes = append(t.nodes, treeNode{box: box, start: start, end: end, left: -1, right: -1, maxID: maxID})
	if end-start <= treeLeafSize {
		return index
	}
	axis := 0
	for a := 1; a < 3; a++ {
		if vecAxis(box[1], a)-vecAxis(box[0], a) > vecAxis(box[1], axis)-vecAxis(box[0], axis) {
			axis = a
		}
	}
	cmp := func(a, b int32) int {
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
	}
	mid := start + (end-start)/2
	if fullSort {
		slices.SortFunc(t.order[start:end], cmp)
	} else {
		selectNth(t.order[start:end], mid-start, cmp)
		for _, r := range [2][2]int{{start, mid}, {mid, end}} {
			if r[1]-r[0] <= treeLeafSize {
				slices.SortFunc(t.order[r[0]:r[1]], cmp)
			}
		}
	}
	left := t.build(start, mid, fullSort)
	right := t.build(mid, end, fullSort)
	t.nodes[index].left, t.nodes[index].right = left, right
	return index
}

// visitRow reports the boxes overlapping member i, with each partner j > i
// once. It counts node visits and leaf box comparisons, including rejected
// ones, so a caller can bound the work before running any pair audit.
func (t treeScan) visitRow(budget *proofbound.WorkBudget, i int, limit uint64,
	fn func(j int)) (uint64, bool, error) {
	if len(t.nodes) == 0 {
		return 0, false, nil
	}
	var scanned uint64
	step := func() (bool, error) {
		if err := budget.Step(); err != nil {
			return false, err
		}
		scanned++
		return scanned > limit, nil
	}
	var walk func(node int) (bool, error)
	walk = func(node int) (bool, error) {
		if exceeded, err := step(); exceeded || err != nil {
			return exceeded, err
		}
		n := t.nodes[node]
		if int(n.maxID) <= i || !meshbool.BoxesOverlap(t.boxes[i], n.box) {
			return false, nil
		}
		if n.left >= 0 {
			if exceeded, err := walk(n.left); exceeded || err != nil {
				return exceeded, err
			}
			return walk(n.right)
		}
		for _, j := range t.order[n.start:n.end] {
			if int(j) <= i {
				continue
			}
			if exceeded, err := step(); exceeded || err != nil {
				return exceeded, err
			}
			if meshbool.BoxesOverlap(t.boxes[i], t.boxes[j]) {
				fn(int(j))
			}
		}
		return false, nil
	}
	exceeded, err := walk(0)
	return scanned, exceeded, err
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

// loftTreeParallelMin is the member count from which the tree scans fan out.
const loftTreeParallelMin = 4096

// replayRowSteps calls budget.Step work times, as the serial walks do one
// unit at a time, through budget.StepN. It returns the number of successful
// steps and the first error; with no error it returns (work, exceeded, nil).
func replayRowSteps(budget *proofbound.WorkBudget, work uint64, exceeded bool) (uint64, bool, error) {
	if s, err := budget.StepN(work); err != nil {
		return s, false, err
	}
	return work, exceeded, nil
}

// walkRow is visitRow's walk without a budget: the same node and leaf order,
// the same partners passed to fn, and the work count visitRow would step,
// stopping at limit+1. It reads only the immutable tree, so concurrent calls
// are safe as long as each fn is private to its caller.
func (t treeScan) walkRow(i int, limit uint64, fn func(j int)) (uint64, bool) {
	return t.flat.walkRow(t.boxes[i], i, limit, fn)
}

// walkRowAppend is walkRow with the partners appended to out in the order
// walkRow passes them to fn.
func (t treeScan) walkRowAppend(i int, limit uint64, out []int32) ([]int32, uint64, bool) {
	return t.flat.walkRowAppend(t.boxes[i], i, limit, out)
}

// flatTree is treeScan's nodes and member boxes as parallel arrays, so a walk
// reads contiguous numbers instead of 100-byte node structs. It is built once
// by newTreeScan and never written afterwards.
type flatTree struct {
	nbox        []float64 // six per node: min x, y, z then max x, y, z
	left, right []int32   // child nodes; left < 0 for a leaf
	start, end  []int32   // the node's range of order
	maxID       []int32
	order       []int32
	box         []float64 // six per position in order, as nbox

	// The fields below describe the subtree of each node. When inter is
	// non-empty (interOK), every member box and every descendant node box
	// contains it, so a query box that overlaps it overlaps all of them.
	minID   []int32   // smallest member id
	subN    []int32   // nodes in the subtree, the node included
	inter   []float64 // six per node: intersection of the member boxes
	interOK []bool    // inter is non-empty; false when any bound is NaN

	// sorted holds each node's member ids ascending, at sortOff. It is built
	// by ensureSorted on the calling goroutine before any worker reads it.
	sorted  []int32
	sortOff []int32
}

// flatWalkStack is the walk stack depth: a node is replaced by two children,
// so the stack holds at most the tree depth plus one entry, and a tree over
// fewer than 2^31 members is shallower than 32 levels.
const flatWalkStack = 64

func newFlatTree(t *treeScan) *flatTree {
	f := &flatTree{
		nbox:  make([]float64, 0, 6*len(t.nodes)),
		left:  make([]int32, len(t.nodes)),
		right: make([]int32, len(t.nodes)),
		start: make([]int32, len(t.nodes)),
		end:   make([]int32, len(t.nodes)),
		maxID: make([]int32, len(t.nodes)),
		order: t.order,
		box:   make([]float64, 0, 6*len(t.order)),
	}
	for k, n := range t.nodes {
		f.nbox = append(f.nbox, n.box[0].X, n.box[0].Y, n.box[0].Z, n.box[1].X, n.box[1].Y, n.box[1].Z)
		f.left[k], f.right[k] = int32(n.left), int32(n.right)
		f.start[k], f.end[k] = int32(n.start), int32(n.end)
		f.maxID[k] = n.maxID
	}
	for _, j := range t.order {
		b := t.boxes[j]
		f.box = append(f.box, b[0].X, b[0].Y, b[0].Z, b[1].X, b[1].Y, b[1].Z)
	}
	f.fillSubtrees(t)
	return f
}

// fillSubtrees fills minID, subN, inter and interOK. build appends a parent
// before its children, so a descending loop sees children first.
func (f *flatTree) fillSubtrees(t *treeScan) {
	n := len(t.nodes)
	f.minID = make([]int32, n)
	f.subN = make([]int32, n)
	f.inter = make([]float64, 6*n)
	f.interOK = make([]bool, n)
	for k := n - 1; k >= 0; k-- {
		nd := t.nodes[k]
		in := f.inter[6*k : 6*k+6]
		if nd.left < 0 {
			mn := t.order[nd.start]
			b := t.boxes[mn]
			in[0], in[1], in[2], in[3], in[4], in[5] = b[0].X, b[0].Y, b[0].Z, b[1].X, b[1].Y, b[1].Z
			for _, j := range t.order[nd.start:nd.end] {
				mn = min(mn, j)
				b := t.boxes[j]
				in[0], in[1], in[2] = max(in[0], b[0].X), max(in[1], b[0].Y), max(in[2], b[0].Z)
				in[3], in[4], in[5] = min(in[3], b[1].X), min(in[4], b[1].Y), min(in[5], b[1].Z)
			}
			f.minID[k] = mn
			f.subN[k] = 1
		} else {
			l, r := nd.left, nd.right
			f.minID[k] = min(f.minID[l], f.minID[r])
			f.subN[k] = 1 + f.subN[l] + f.subN[r]
			a, b := f.inter[6*l:6*l+6], f.inter[6*r:6*r+6]
			for c := range 3 {
				in[c] = max(a[c], b[c])
				in[c+3] = min(a[c+3], b[c+3])
			}
		}
		// min and max propagate NaN, and a comparison with NaN is false.
		f.interOK[k] = in[0] <= in[3] && in[1] <= in[4] && in[2] <= in[5]
	}
}

// ensureSorted builds the per-node ascending member ids. It writes the
// flatTree, so it runs on one goroutine before workers read the tree.
func (f *flatTree) ensureSorted() {
	if f.sortOff != nil || len(f.left) == 0 {
		return
	}
	n := len(f.left)
	f.sortOff = make([]int32, n)
	total := 0
	for k := range n {
		f.sortOff[k] = int32(total)
		total += int(f.end[k] - f.start[k])
	}
	f.sorted = make([]int32, total)
	for k := n - 1; k >= 0; k-- {
		dst := f.sortedOf(int32(k))
		if f.left[k] < 0 {
			copy(dst, f.order[f.start[k]:f.end[k]])
			slices.Sort(dst)
			continue
		}
		a, b := f.sortedOf(f.left[k]), f.sortedOf(f.right[k])
		x, y := 0, 0
		for o := range dst {
			if y >= len(b) || (x < len(a) && a[x] < b[y]) {
				dst[o] = a[x]
				x++
			} else {
				dst[o] = b[y]
				y++
			}
		}
	}
}

// sortedOf returns node k's member ids ascending; ensureSorted must have run.
func (f *flatTree) sortedOf(k int32) []int32 {
	return f.sorted[f.sortOff[k] : int(f.sortOff[k])+int(f.end[k]-f.start[k])]
}

// pairWork is the unit count of pairWalker.walk over two subtrees of p and q
// members (one subtree of p members when self) in which every node pair and
// member pair overlaps. The tree splits at the middle and makes a leaf from
// treeLeafSize members or fewer, and walk's split rule reads only sizes and
// leafness, so the count depends on the sizes alone. memo caches by size.
func pairWork(memo map[[3]int32]uint64, p, q int32, self bool) uint64 {
	key := [3]int32{p, q, 0}
	if self {
		key[2] = 1
	}
	if w, ok := memo[key]; ok {
		return w
	}
	var w uint64
	switch {
	case self && p <= treeLeafSize:
		w = 1 + uint64(p)*uint64(p-1)/2
	case self:
		l, r := p/2, p-p/2
		w = 1 + pairWork(memo, l, l, true) + pairWork(memo, l, r, false) + pairWork(memo, r, r, true)
	case p <= treeLeafSize && q <= treeLeafSize:
		w = 1 + uint64(p)*uint64(q)
	case q <= treeLeafSize || (p > treeLeafSize && p >= q):
		w = 1 + pairWork(memo, p/2, q, false) + pairWork(memo, p-p/2, q, false)
	default:
		w = 1 + pairWork(memo, p, q/2, false) + pairWork(memo, p, q-q/2, false)
	}
	memo[key] = w
	return w
}

func box6(b [2]r3.Vec) [6]float64 {
	return [6]float64{b[0].X, b[0].Y, b[0].Z, b[1].X, b[1].Y, b[1].Z}
}

// boxes6Overlap is meshbool.BoxesOverlap over two six-number boxes.
func boxes6Overlap(a, b []float64) bool {
	return a[0] <= b[3] && b[0] <= a[3] && a[1] <= b[4] && b[1] <= a[4] && a[2] <= b[5] && b[2] <= a[5]
}

// walkRowAppend is visitRow's walk as an iterative loop with no budget. The
// stack pushes right before left, so nodes come in the order a recursion
// visits them.
func (f *flatTree) walkRowAppend(row [2]r3.Vec, i int, limit uint64, out []int32) ([]int32, uint64, bool) {
	bi := box6(row)
	return f.walkRowAppendBox(&bi, i, limit, out)
}

// walkRowAppendBox is walkRowAppend with the query box already in box6 form,
// so a caller walking several trees for one row converts it once.
func (f *flatTree) walkRowAppendBox(bi *[6]float64, i int, limit uint64, out []int32) ([]int32, uint64, bool) {
	if len(f.left) == 0 {
		return out, 0, false
	}
	var stack [flatWalkStack]int32
	sp := 1
	var scanned uint64
	i32 := int32(i)
	for sp > 0 {
		sp--
		node := stack[sp]
		scanned++
		if scanned > limit {
			return out, scanned, true
		}
		if f.maxID[node] <= i32 || !boxes6Overlap(bi[:], f.nbox[6*int(node):6*int(node)+6:6*int(node)+6]) {
			continue
		}
		if f.minID[node] > i32 && f.interOK[node] &&
			boxes6Overlap(bi[:], f.inter[6*int(node):6*int(node)+6:6*int(node)+6]) {
			// Every member and descendant box contains a point of the query
			// box, so the walk below would visit all of them and keep all.
			extra := uint64(f.subN[node]-1) + uint64(f.end[node]-f.start[node])
			if scanned+extra <= limit {
				scanned += extra
				out = append(out, f.order[f.start[node]:f.end[node]]...)
				continue
			}
		}
		if l := f.left[node]; l >= 0 {
			stack[sp] = f.right[node]
			stack[sp+1] = l
			sp += 2
			continue
		}
		for k := int(f.start[node]); k < int(f.end[node]); k++ {
			j := f.order[k]
			if j <= i32 {
				continue
			}
			scanned++
			if scanned > limit {
				return out, scanned, true
			}
			if boxes6Overlap(bi[:], f.box[6*k:6*k+6:6*k+6]) {
				out = append(out, j)
			}
		}
	}
	return out, scanned, false
}

// walkRow is walkRowAppend with each partner passed to fn.
func (f *flatTree) walkRow(row [2]r3.Vec, i int, limit uint64, fn func(j int)) (uint64, bool) {
	bi := box6(row)
	return f.walkRowBox(&bi, i, limit, fn)
}

// walkRowBox is walkRow with the query box already in box6 form.
func (f *flatTree) walkRowBox(bi *[6]float64, i int, limit uint64, fn func(j int)) (uint64, bool) {
	if len(f.left) == 0 {
		return 0, false
	}
	var stack [flatWalkStack]int32
	sp := 1
	var scanned uint64
	i32 := int32(i)
	for sp > 0 {
		sp--
		node := stack[sp]
		scanned++
		if scanned > limit {
			return scanned, true
		}
		if f.maxID[node] <= i32 || !boxes6Overlap(bi[:], f.nbox[6*int(node):6*int(node)+6:6*int(node)+6]) {
			continue
		}
		if f.minID[node] > i32 && f.interOK[node] &&
			boxes6Overlap(bi[:], f.inter[6*int(node):6*int(node)+6:6*int(node)+6]) {
			// Every member and descendant box contains a point of the query
			// box, so the walk below would visit all of them and keep all.
			extra := uint64(f.subN[node]-1) + uint64(f.end[node]-f.start[node])
			if scanned+extra <= limit {
				scanned += extra
				for _, j := range f.order[f.start[node]:f.end[node]] {
					fn(int(j))
				}
				continue
			}
		}
		if l := f.left[node]; l >= 0 {
			stack[sp] = f.right[node]
			stack[sp+1] = l
			sp += 2
			continue
		}
		for k := int(f.start[node]); k < int(f.end[node]); k++ {
			j := f.order[k]
			if j <= i32 {
				continue
			}
			scanned++
			if scanned > limit {
				return scanned, true
			}
			if boxes6Overlap(bi[:], f.box[6*k:6*k+6:6*k+6]) {
				fn(int(j))
			}
		}
	}
	return scanned, false
}

// walkRowCount is walkRow counting the partners instead of passing them to
// fn. At an exceed it returns the count reached so far, as walkRow's callbacks
// would have made.
func (f *flatTree) walkRowCount(row [2]r3.Vec, i int, limit uint64) (int, uint64, bool) {
	if len(f.left) == 0 {
		return 0, 0, false
	}
	bi := box6(row)
	var stack [flatWalkStack]int32
	sp := 1
	var scanned uint64
	count := 0
	i32 := int32(i)
	for sp > 0 {
		sp--
		node := stack[sp]
		scanned++
		if scanned > limit {
			return count, scanned, true
		}
		if f.maxID[node] <= i32 || !boxes6Overlap(bi[:], f.nbox[6*int(node):6*int(node)+6:6*int(node)+6]) {
			continue
		}
		if f.minID[node] > i32 && f.interOK[node] &&
			boxes6Overlap(bi[:], f.inter[6*int(node):6*int(node)+6:6*int(node)+6]) {
			// Every member and descendant box contains a point of the query
			// box, so the walk below would visit all of them and keep all.
			extra := uint64(f.subN[node]-1) + uint64(f.end[node]-f.start[node])
			if scanned+extra <= limit {
				scanned += extra
				count += int(f.end[node] - f.start[node])
				continue
			}
		}
		if l := f.left[node]; l >= 0 {
			stack[sp] = f.right[node]
			stack[sp+1] = l
			sp += 2
			continue
		}
		for k := int(f.start[node]); k < int(f.end[node]); k++ {
			j := f.order[k]
			if j <= i32 {
				continue
			}
			scanned++
			if scanned > limit {
				return count, scanned, true
			}
			if boxes6Overlap(bi[:], f.box[6*k:6*k+6:6*k+6]) {
				count++
			}
		}
	}
	return count, scanned, false
}

// pairWalker is visit's walk(a, b) without a budget, as an iterative loop.
type pairWalker struct {
	t       *treeScan
	limit   uint64
	scanned uint64
	fn      func(i, j int)
	// counts, when non-nil, replaces fn for subtrees whose member boxes all
	// overlap: keep accepts every pair, and counts[i+1] takes the pairs whose
	// lower index is i. memo is private to the walker.
	counts []int32
	memo   map[[3]int32]uint64
}

// walk visits the node pair (a, b) and everything below it in visit's order.
// It reports whether the work limit was exceeded.
func (w *pairWalker) walk(a, b int) bool {
	t := w.t
	f := t.flat
	stack := make([][2]int32, 1, 3*flatWalkStack)
	stack[0] = [2]int32{int32(a), int32(b)}
	for len(stack) > 0 {
		top := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		a, b := int(top[0]), int(top[1])
		w.scanned++
		if w.scanned > w.limit {
			return true
		}
		if !boxes6Overlap(f.nbox[6*a:6*a+6:6*a+6], f.nbox[6*b:6*b+6:6*b+6]) {
			continue
		}
		if w.counts != nil && f.interOK[a] && f.interOK[b] &&
			boxes6Overlap(f.inter[6*a:6*a+6:6*a+6], f.inter[6*b:6*b+6:6*b+6]) {
			// Every node pair and member pair below (a, b) overlaps.
			extra := pairWork(w.memo, f.end[a]-f.start[a], f.end[b]-f.start[b], a == b) - 1
			if w.scanned+extra <= w.limit {
				w.scanned += extra
				w.addAll(int32(a), int32(b))
				continue
			}
		}
		la, lb := f.left[a], f.left[b]
		if la < 0 && lb < 0 {
			if w.leafPairs(a, b) {
				return true
			}
			continue
		}
		ra, rb := f.right[a], f.right[b]
		// Pushed in reverse so the first pair pops first.
		switch {
		case a == b:
			stack = append(stack, [2]int32{ra, ra}, [2]int32{la, ra}, [2]int32{la, la})
		case lb < 0 || (la >= 0 && f.end[a]-f.start[a] >= f.end[b]-f.start[b]):
			stack = append(stack, [2]int32{ra, int32(b)}, [2]int32{la, int32(b)})
		default:
			stack = append(stack, [2]int32{int32(a), rb}, [2]int32{int32(a), lb})
		}
	}
	return false
}

// addAll counts every member pair of (a, b) at its lower index, as fn would.
func (w *pairWalker) addAll(a, b int32) {
	f := w.t.flat
	ids := f.sortedOf(a)
	if a == b {
		n := int32(len(ids))
		for k, x := range ids {
			w.counts[x+1] += n - 1 - int32(k)
		}
		return
	}
	other := f.sortedOf(b)
	for _, pair := range [2][2][]int32{{ids, other}, {other, ids}} {
		p := 0
		for _, x := range pair[0] {
			for p < len(pair[1]) && pair[1][p] < x {
				p++
			}
			w.counts[x+1] += int32(len(pair[1]) - p)
		}
	}
}

// leafPairs is walk's leaf-leaf case: every member pair of the two leaves.
func (w *pairWalker) leafPairs(a, b int) bool {
	t := w.t
	na, nb := &t.nodes[a], &t.nodes[b]
	for ia := na.start; ia < na.end; ia++ {
		begin := nb.start
		if a == b {
			begin = ia + 1
		}
		for ib := begin; ib < nb.end; ib++ {
			w.scanned++
			if w.scanned > w.limit {
				return true
			}
			i, j := int(t.order[ia]), int(t.order[ib])
			if !meshbool.BoxesOverlap(t.boxes[i], t.boxes[j]) {
				continue
			}
			if i > j {
				i, j = j, i
			}
			w.fn(i, j)
		}
	}
	return false
}

// pairTasks expands walk(0, 0) breadth-first, in visit's order, into at least
// want node pairs whose walks partition the rest of visit's work, and returns
// them with the work units the expansion consumed (one per expanded non-leaf
// pair).
func (t treeScan) pairTasks(want int) ([][2]int, uint64) {
	tasks := [][2]int{{0, 0}}
	var used uint64
	for len(tasks) < want {
		next := make([][2]int, 0, 3*len(tasks))
		grew := false
		for _, task := range tasks {
			a, b := task[0], task[1]
			na, nb := &t.nodes[a], &t.nodes[b]
			if na.left < 0 && nb.left < 0 {
				next = append(next, task)
				continue
			}
			used++
			grew = true
			if !meshbool.BoxesOverlap(na.box, nb.box) {
				continue
			}
			switch {
			case a == b:
				next = append(next, [2]int{na.left, na.left}, [2]int{na.left, na.right}, [2]int{na.right, na.right})
			case nb.left < 0 || (na.left >= 0 && na.end-na.start >= nb.end-nb.start):
				next = append(next, [2]int{na.left, b}, [2]int{na.right, b})
			default:
				next = append(next, [2]int{a, nb.left}, [2]int{a, nb.right})
			}
		}
		tasks = next
		if !grew {
			break
		}
	}
	return tasks, used
}

// isKeepEveryPair reports whether keep is keepEveryPair. A false answer only
// forgoes the count shortcut.
func isKeepEveryPair(keep func(i, j int) bool) bool {
	return reflect.ValueOf(keep).Pointer() == reflect.ValueOf(keepEveryPair).Pointer()
}

// countParallel is sweepCandidateCounts' first pass over the tree with the
// walk split across workers. Workers never touch budget; the steps visit
// would take are replayed here, in count, and the result equals the serial
// pass's. A pair count is a sum, so how the walk is split does not matter.
func (t treeScan) countParallel(budget *proofbound.WorkBudget, f int, keep func(i, j int) bool,
	limit uint64, workers int) (sweepCount, error) {
	if len(t.nodes) == 0 {
		return sweepCount{starts: make([]int, f+1)}, nil
	}
	every := isKeepEveryPair(keep)
	if every {
		t.flat.ensureSorted()
	}
	tasks, used := t.pairTasks(256)
	type part struct {
		work     uint64
		exceeded bool
	}
	parts := make([]part, len(tasks))
	blocks := 4 * workers
	partial := make([][]int32, blocks)
	if _, err := orderedwork.Each(blocks, workers, 1, budget.Err, func(blk int) error {
		local := make([]int32, f+1)
		fn := func(i, j int) {
			if keep(i, j) {
				local[i+1]++
			}
		}
		for k := blk; k < len(tasks); k += blocks {
			w := pairWalker{t: &t, limit: limit, fn: fn}
			if every {
				w.counts, w.memo = local, map[[3]int32]uint64{}
			}
			parts[k].exceeded = w.walk(tasks[k][0], tasks[k][1])
			parts[k].work = w.scanned
		}
		partial[blk] = local
		return nil
	}); err != nil {
		// No worker has charged budget. Replay an exhausted ceiling through
		// the serial walk so its first failing step and metadata are identical.
		if errors.Is(err, proofbound.ErrWallWorkBudget) {
			return sweepCandidateCountsWorkers(budget, t, f, keep, limit, 1)
		}
		return sweepCount{}, err
	}
	total := used
	exceeded := used > limit
	for _, p := range parts {
		total += p.work
		if p.exceeded || total > limit {
			exceeded = true
		}
	}
	if exceeded {
		scanned, _, err := replayRowSteps(budget, limit+1, true)
		return sweepCount{scanned: scanned, exceeded: err == nil}, err
	}
	if scanned, _, err := replayRowSteps(budget, total, false); err != nil {
		return sweepCount{scanned: scanned}, err
	}
	starts := make([]int, f+1)
	for _, local := range partial {
		for i, c := range local {
			starts[i] += int(c)
		}
	}
	for i := 1; i <= f; i++ {
		starts[i] += starts[i-1]
	}
	return sweepCount{starts: starts, count: uint64(starts[f]), scanned: total}, nil
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
