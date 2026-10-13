package loftmesh

import (
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
		t.build(0, len(members))
	}
	t.flat = newFlatTree(&t)
	return t
}

func (t *treeScan) build(start, end int) int {
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
	return f
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
	if len(f.left) == 0 {
		return out, 0, false
	}
	bi := box6(row)
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
	if len(f.left) == 0 {
		return 0, false
	}
	bi := box6(row)
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

// pairWalker is visit's walk(a, b) without a budget, as an iterative loop.
type pairWalker struct {
	t       *treeScan
	limit   uint64
	scanned uint64
	fn      func(i, j int)
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

// countParallel is sweepCandidateCounts' first pass over the tree with the
// walk split across workers. Workers never touch budget; the steps visit
// would take are replayed here, in count, and the result equals the serial
// pass's. A pair count is a sum, so how the walk is split does not matter.
func (t treeScan) countParallel(budget *proofbound.WorkBudget, f int, keep func(i, j int) bool,
	limit uint64, workers int) (sweepCount, error) {
	if len(t.nodes) == 0 {
		return sweepCount{starts: make([]int, f+1)}, nil
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
			parts[k].exceeded = w.walk(tasks[k][0], tasks[k][1])
			parts[k].work = w.scanned
		}
		partial[blk] = local
		return nil
	}); err != nil {
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
