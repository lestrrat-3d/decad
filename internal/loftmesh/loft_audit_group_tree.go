package loftmesh

import (
	"errors"
	"slices"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// groupedTreeScan omits only pairs inside the same proven projected patch.
// Family -1 contains every triangle without such a proof. Separate trees
// prune cap-to-cap and cap-to-wall pairs before S8 charges a scan unit.
type groupedTreeScan struct {
	groups []treeScan
	keys   []int
	family []int
}

func newGroupedTreeScan(boxes [][2]r3.Vec, members, family []int) groupedTreeScan {
	buckets := make(map[int][]int)
	for _, i := range members {
		buckets[family[i]] = append(buckets[family[i]], i)
	}
	keys := make([]int, 0, len(buckets))
	for key := range buckets {
		keys = append(keys, key)
	}
	slices.Sort(keys)
	out := groupedTreeScan{keys: keys, groups: make([]treeScan, len(keys)), family: family}
	for i, key := range keys {
		out.groups[i] = newTreeScan(boxes, buckets[key])
	}
	return out
}

// visitRow emits the same pairs as visit that have i as their lower index.
// Only a proven patch's own tree is omitted; cross-patch and unproved pairs
// retain the ordinary box query and exact contact verdict.
func (s groupedTreeScan) visitRow(budget *proofbound.WorkBudget, i int,
	limit uint64, fn func(j int)) (uint64, bool, error) {
	var scanned uint64
	for k, group := range s.groups {
		if s.keys[k] >= 0 && s.keys[k] == s.family[i] {
			continue
		}
		used, exceeded, err := group.visitRow(budget, i, limit-scanned, fn)
		scanned += used
		if exceeded || err != nil {
			return scanned, exceeded, err
		}
	}
	return scanned, false, nil
}

// walkRow is visitRow's walk without a budget; see treeScan.walkRow.
func (s groupedTreeScan) walkRow(i int, limit uint64, fn func(j int)) (uint64, bool) {
	var scanned uint64
	for k, group := range s.groups {
		if s.keys[k] >= 0 && s.keys[k] == s.family[i] {
			continue
		}
		used, exceeded := group.walkRow(i, limit-scanned, fn)
		scanned += used
		if exceeded {
			return scanned, true
		}
	}
	return scanned, false
}

func (s groupedTreeScan) visit(budget *proofbound.WorkBudget, limit uint64,
	fn func(i, j int)) (uint64, bool, error) {
	var scanned uint64
	step := func() (bool, error) {
		if err := budget.Step(); err != nil {
			return false, err
		}
		scanned++
		return scanned > limit, nil
	}
	var cross func(a treeScan, ai int, b treeScan, bi int) (bool, error)
	cross = func(a treeScan, ai int, b treeScan, bi int) (bool, error) {
		if exceeded, err := step(); exceeded || err != nil {
			return exceeded, err
		}
		na, nb := a.nodes[ai], b.nodes[bi]
		if !meshbool.BoxesOverlap(na.box, nb.box) {
			return false, nil
		}
		if na.left < 0 && nb.left < 0 {
			for _, ia := range a.order[na.start:na.end] {
				for _, ib := range b.order[nb.start:nb.end] {
					if exceeded, err := step(); exceeded || err != nil {
						return exceeded, err
					}
					i, j := int(ia), int(ib)
					if !meshbool.BoxesOverlap(a.boxes[i], b.boxes[j]) {
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
		if nb.left < 0 || (na.left >= 0 && na.end-na.start >= nb.end-nb.start) {
			if exceeded, err := cross(a, na.left, b, bi); exceeded || err != nil {
				return exceeded, err
			}
			return cross(a, na.right, b, bi)
		}
		if exceeded, err := cross(a, ai, b, nb.left); exceeded || err != nil {
			return exceeded, err
		}
		return cross(a, ai, b, nb.right)
	}
	for ia, a := range s.groups {
		if s.keys[ia] == -1 {
			used, exceeded, err := a.visit(budget, limit-scanned, fn)
			scanned += used
			if exceeded || err != nil {
				return scanned, exceeded, err
			}
		}
		for ib := ia + 1; ib < len(s.groups); ib++ {
			if exceeded, err := cross(a, 0, s.groups[ib], 0); exceeded || err != nil {
				return scanned, exceeded, err
			}
		}
	}
	return scanned, false, nil
}

// members is the number of triangles over all groups.
func (s groupedTreeScan) members() int {
	var n int
	for _, g := range s.groups {
		n += len(g.order)
	}
	return n
}

// crossWalker is groupedTreeScan.visit's cross(a, x, b, y) without a budget,
// as an iterative loop over two trees. It visits node pairs and leaf member
// pairs in cross's order and counts one unit for each. It reads only the
// immutable trees, so concurrent walkers are safe as long as each fn is
// private to its caller.
type crossWalker struct {
	a, b    *treeScan
	limit   uint64
	scanned uint64
	fn      func(i, j int)
}

// walk visits the node pair (x, y) of a and b and everything below it in
// cross's order. It reports whether the work limit was exceeded.
func (w *crossWalker) walk(x, y int) bool {
	fa, fb := w.a.flat, w.b.flat
	stack := make([][2]int32, 1, 2*flatWalkStack)
	stack[0] = [2]int32{int32(x), int32(y)}
	for len(stack) > 0 {
		top := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		x, y := int(top[0]), int(top[1])
		w.scanned++
		if w.scanned > w.limit {
			return true
		}
		if !boxes6Overlap(fa.nbox[6*x:6*x+6:6*x+6], fb.nbox[6*y:6*y+6:6*y+6]) {
			continue
		}
		nx, ny := &w.a.nodes[x], &w.b.nodes[y]
		if nx.left < 0 && ny.left < 0 {
			if w.leafPairs(nx, ny) {
				return true
			}
			continue
		}
		// Pushed in reverse so the first pair pops first.
		if ny.left < 0 || (nx.left >= 0 && nx.end-nx.start >= ny.end-ny.start) {
			stack = append(stack, [2]int32{int32(nx.right), int32(y)}, [2]int32{int32(nx.left), int32(y)})
			continue
		}
		stack = append(stack, [2]int32{int32(x), int32(ny.right)}, [2]int32{int32(x), int32(ny.left)})
	}
	return false
}

// leafPairs is walk's leaf-leaf case: every member pair of the two leaves.
func (w *crossWalker) leafPairs(nx, ny *treeNode) bool {
	for _, ia := range w.a.order[nx.start:nx.end] {
		for _, ib := range w.b.order[ny.start:ny.end] {
			w.scanned++
			if w.scanned > w.limit {
				return true
			}
			i, j := int(ia), int(ib)
			if !meshbool.BoxesOverlap(w.a.boxes[i], w.b.boxes[j]) {
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

// crossTasks expands cross(a, 0, b, 0) breadth-first, in cross's order, into
// at least want node pairs whose walks partition the rest of its work, and
// returns them with the units the expansion consumed (one per expanded pair
// that is not two leaves).
func crossTasks(a, b *treeScan, want int) ([][2]int, uint64) {
	tasks := [][2]int{{0, 0}}
	var used uint64
	for len(tasks) < want {
		next := make([][2]int, 0, 2*len(tasks))
		grew := false
		for _, task := range tasks {
			x, y := task[0], task[1]
			nx, ny := &a.nodes[x], &b.nodes[y]
			if nx.left < 0 && ny.left < 0 {
				next = append(next, task)
				continue
			}
			used++
			grew = true
			if !meshbool.BoxesOverlap(nx.box, ny.box) {
				continue
			}
			if ny.left < 0 || (nx.left >= 0 && nx.end-nx.start >= ny.end-ny.start) {
				next = append(next, [2]int{nx.left, y}, [2]int{nx.right, y})
				continue
			}
			next = append(next, [2]int{x, ny.left}, [2]int{x, ny.right})
		}
		tasks = next
		if !grew {
			break
		}
	}
	return tasks, used
}

// groupJob is one walk of groupedTreeScan.countParallel: the self walk of
// group g (cross false; h is unused) or the cross walk of groups g and h.
type groupJob struct {
	g, h  int
	x, y  int
	cross bool
}

// countParallel is sweepCandidateCounts' first pass over every family's tree
// with the walks split across workers. Workers never touch budget; the units
// visit would step are replayed on the caller, and the counts, the total and
// the exceeded flag equal visit's. A pair count is a sum, so how the walk is
// split does not matter.
func (s groupedTreeScan) countParallel(budget *proofbound.WorkBudget, f int,
	keep func(i, j int) bool, limit uint64, workers int) (sweepCount, error) {
	every := isKeepEveryPair(keep)
	var jobs []groupJob
	var used uint64
	for ia := range s.groups {
		a := &s.groups[ia]
		if len(a.nodes) == 0 {
			continue
		}
		if s.keys[ia] == -1 {
			if every {
				a.flat.ensureSorted()
			}
			tasks, n := a.pairTasks(256)
			used += n
			for _, task := range tasks {
				jobs = append(jobs, groupJob{g: ia, x: task[0], y: task[1]})
			}
		}
		for ib := ia + 1; ib < len(s.groups); ib++ {
			b := &s.groups[ib]
			if len(b.nodes) == 0 {
				continue
			}
			tasks, n := crossTasks(a, b, 64)
			used += n
			for _, task := range tasks {
				jobs = append(jobs, groupJob{g: ia, h: ib, x: task[0], y: task[1], cross: true})
			}
		}
	}
	type part struct {
		work     uint64
		exceeded bool
	}
	parts := make([]part, len(jobs))
	blocks := 4 * workers
	partial := make([][]int32, blocks)
	if _, err := orderedwork.Each(blocks, workers, 1, budget.Err, func(blk int) error {
		local := make([]int32, f+1)
		fn := func(i, j int) {
			if keep(i, j) {
				local[i+1]++
			}
		}
		for k := blk; k < len(jobs); k += blocks {
			job := jobs[k]
			if job.cross {
				w := crossWalker{a: &s.groups[job.g], b: &s.groups[job.h], limit: limit, fn: fn}
				parts[k].exceeded = w.walk(job.x, job.y)
				parts[k].work = w.scanned
				continue
			}
			w := pairWalker{t: &s.groups[job.g], limit: limit, fn: fn}
			if every {
				w.counts, w.memo = local, map[[3]int32]uint64{}
			}
			parts[k].exceeded = w.walk(job.x, job.y)
			parts[k].work = w.scanned
		}
		partial[blk] = local
		return nil
	}); err != nil {
		// As in treeScan, workers have not charged budget; the serial walk
		// determines whether a step fails, including a scan with no work.
		if errors.Is(err, proofbound.ErrWallWorkBudget) {
			return sweepCandidateCountsWorkers(budget, s, f, keep, limit, 1)
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
