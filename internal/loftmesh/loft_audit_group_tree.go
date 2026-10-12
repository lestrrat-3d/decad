package loftmesh

import (
	"slices"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// groupedTreeScan omits only pairs inside the same proven projected patch.
// Family -1 contains every triangle without such a proof. Separate trees
// prune cap-to-cap and cap-to-wall pairs before S8 charges a scan unit.
type groupedTreeScan struct {
	groups []treeScan
	keys   []int
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
	out := groupedTreeScan{keys: keys, groups: make([]treeScan, len(keys))}
	for i, key := range keys {
		out.groups[i] = newTreeScan(boxes, buckets[key])
	}
	return out
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
