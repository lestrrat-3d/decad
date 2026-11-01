package revolvemesh

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type replayContactNode struct {
	box         [2]r3.Vec
	lo, hi      int
	left, right int
}

// replayBoxCandidates uses the 3D box tree to enumerate pairs that can reach
// contact classification. It charges each emitted pair against the fixed
// pair ceiling; tree visits remain charged to the replay budget.
// Nodes cover contiguous sweep-order ranges, and emitted rows are sorted by
// the caller after shared-vertex pairs have been added.
func replayBoxCandidates(ctx context.Context, data []RevolveAuditTri, order []int32, rows [][]int32,
	margin float64, remaining uint64, b *proofbound.ReplayBudget) (uint64, error) {
	n := len(order)
	if n == 0 {
		return 0, nil
	}
	if err := b.ReserveRecords(ctx, 2*uint64(n)+1); err != nil {
		return 0, err
	}
	if err := b.ReserveArray(ctx, 1, 256); err != nil {
		return 0, err
	}
	if err := b.ReserveArray(ctx, 2*uint64(n), uint64(unsafe.Sizeof(replayContactNode{}))); err != nil {
		return 0, err
	}
	nodes := make([]replayContactNode, 0, 2*n)
	var build func(int, int) (int, error)
	build = func(lo, hi int) (int, error) {
		if err := b.SpendWork(ctx, 64); err != nil {
			return 0, err
		}
		idx := len(nodes)
		nodes = append(nodes, replayContactNode{lo: lo, hi: hi, left: -1, right: -1})
		box := data[order[lo]].Box
		for _, id := range order[lo+1 : hi] {
			if err := b.SpendWork(ctx, 64); err != nil {
				return 0, err
			}
			p := data[id].Box
			box[0].X, box[0].Y, box[0].Z = min(box[0].X, p[0].X), min(box[0].Y, p[0].Y), min(box[0].Z, p[0].Z)
			box[1].X, box[1].Y, box[1].Z = max(box[1].X, p[1].X), max(box[1].Y, p[1].Y), max(box[1].Z, p[1].Z)
		}
		nodes[idx].box = box
		if hi-lo > 8 {
			mid := lo + (hi-lo)/2
			left, err := build(lo, mid)
			if err != nil {
				return 0, err
			}
			right, err := build(mid, hi)
			if err != nil {
				return 0, err
			}
			nodes[idx].left, nodes[idx].right = left, right
		}
		return idx, nil
	}
	if _, err := build(0, n); err != nil {
		return 0, err
	}
	var scanned uint64
	for p, id := range order {
		var stack [64]int
		stack[0] = 0
		sp := 1
		for sp > 0 {
			if err := b.SpendWork(ctx, 64); err != nil {
				return scanned, err
			}
			sp--
			node := nodes[stack[sp]]
			if node.hi <= p+1 || BoxGapExceeds(data[id].Box, node.box, margin) {
				continue
			}
			if node.left >= 0 {
				if sp+2 > len(stack) {
					return scanned, revolveFacetPairWorkLimit()
				}
				stack[sp], stack[sp+1] = node.right, node.left
				sp += 2
				continue
			}
			for q := max(p+1, node.lo); q < node.hi; q++ {
				if err := b.SpendWork(ctx, 64); err != nil {
					return scanned, err
				}
				other := order[q]
				if BoxGapExceeds(data[id].Box, data[other].Box, margin) {
					continue
				}
				if scanned >= remaining {
					return remaining, revolveFacetPairWorkLimit()
				}
				scanned++
				a, z := min(id, other), max(id, other)
				var err error
				rows[a], err = replayAppend(ctx, rows[a], z, b)
				if err != nil {
					return scanned, err
				}
			}
		}
	}
	return scanned, nil
}
