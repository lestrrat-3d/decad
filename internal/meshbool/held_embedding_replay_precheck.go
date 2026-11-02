package meshbool

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// replayEmbeddingPrecheck proves the held mesh embedded with an exact serial
// scan. A failing pair refuses before the ordinary repair's maps are created.
func replayEmbeddingPrecheck(ctx context.Context, h HeldRounding, meter *proofbound.ReplayBudget) (bool, error) {
	if meter == nil || len(h.Moved) != len(h.Verts) {
		return false, errReplayInvalidReplayEmbeddingInput
	}
	moved := false
	for _, v := range h.Moved {
		if err := meter.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		moved = moved || v
	}
	if !moved {
		return false, nil
	}
	if err := meter.SpendWork(ctx, 3*uint64(len(h.Tris))); err != nil {
		return false, err
	}
	for _, t := range h.Tris {
		for _, vi := range t {
			if vi < 0 || vi >= len(h.Verts) {
				return false, errReplayInvalidReplayFacet
			}
		}
	}
	arrays := []struct{ count, size uint64 }{{uint64(len(h.Verts)), uint64(unsafe.Sizeof(proof.Xpt{}))}, {uint64(len(h.Tris)), uint64(unsafe.Sizeof(proof.Xpt{}))}, {uint64(len(h.Tris)), uint64(unsafe.Sizeof(r3.Vec{}))}, {uint64(len(h.Tris)), uint64(unsafe.Sizeof(false))}, {uint64(len(h.Tris)), uint64(unsafe.Sizeof([2]r3.Vec{}))}}
	for _, a := range arrays {
		if err := meter.ReserveRecords(ctx, a.count); err != nil {
			return false, err
		}
		if err := meter.ReserveArray(ctx, a.count, a.size); err != nil {
			return false, err
		}
	}
	if err := meter.ReserveRecords(ctx, uint64(len(h.Tris))); err != nil {
		return false, err
	}
	if err := meter.ReserveArray(ctx, uint64(len(h.Tris)), uint64(unsafe.Sizeof(proof.FloatOrientationPlane{}))); err != nil {
		return false, err
	}
	if uint64(len(h.Tris)) > ^uint64(0)/128 {
		return false, errReplayOrientationPlaneWorkOverflow
	}
	if err := meter.SpendWork(ctx, uint64(len(h.Tris))*128); err != nil {
		return false, err
	}
	if err := meter.ReserveRecords(ctx, uint64(len(h.Tris))); err != nil {
		return false, err
	}
	if err := meter.ReserveArray(ctx, uint64(len(h.Tris)), uint64(unsafe.Sizeof(heldSATFacet{}))); err != nil {
		return false, err
	}
	if uint64(len(h.Tris)) > ^uint64(0)/2048 {
		return false, errReplayHeldSATWorkOverflow
	}
	if err := meter.SpendWork(ctx, uint64(len(h.Tris))*2048); err != nil {
		return false, err
	}
	heldSAT := make([]heldSATFacet, len(h.Tris))
	orientationPlanes := make([]proof.FloatOrientationPlane, len(h.Tris))
	bm := BoolMesh{Verts: h.Verts, Tris: h.Tris, Xverts: make([]proof.Xpt, len(h.Verts)), Norms: make([]proof.Xpt, len(h.Tris)), Fnorms: make([]r3.Vec, len(h.Tris)), FnormsReady: make([]bool, len(h.Tris)), Boxes: make([][2]r3.Vec, len(h.Tris))}
	reserve := func(n uint64) error {
		if err := meter.ReserveArray(ctx, n, 64); err != nil {
			return err
		}
		return meter.ReserveRecords(ctx, n)
	}
	for i, v := range h.Verts {
		p, err := proof.XptOfBounded(v, reserve)
		if err != nil {
			return false, err
		}
		bm.Xverts[i] = p
	}
	for i, t := range h.Tris {
		n, err := proof.TriangleNormalReplay(h.Verts[t[0]], h.Verts[t[1]], h.Verts[t[2]], reserve, func(n uint64) error { return meter.SpendWork(ctx, n) })
		if err != nil {
			return false, err
		}
		bm.Norms[i] = n
		orientationPlanes[i] = proof.NewFloatOrientationPlane(h.Verts[t[0]], h.Verts[t[1]], h.Verts[t[2]])
		heldSAT[i] = heldSATBuild([3]r3.Vec{h.Verts[t[0]], h.Verts[t[1]], h.Verts[t[2]]})
		bm.Boxes[i] = TriBox(h.Verts, t)
	}
	n := len(h.Tris)
	if n == 0 {
		return false, nil
	}
	if n > (int(^uint(0)>>1)-1)/4 {
		return false, errReplayReplayEmbeddingTreeSize
	}
	type node struct {
		box    [2]r3.Vec
		lo, hi int
	}
	capacity := 4*n + 1
	if err := meter.ReserveRecords(ctx, 2*uint64(capacity)); err != nil {
		return false, err
	}
	if err := meter.ReserveArray(ctx, uint64(capacity), uint64(unsafe.Sizeof(node{}))); err != nil {
		return false, err
	}
	if err := meter.ReserveArray(ctx, uint64(capacity), uint64(unsafe.Sizeof(int(0)))); err != nil {
		return false, err
	}
	nodes := make([]node, capacity)
	stack := make([]int, 0, capacity)
	nodes[1].hi = n
	stack = append(stack, 1)
	for len(stack) > 0 {
		if err := meter.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		id := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		if id < 0 {
			id = -id
			a, b := nodes[2*id].box, nodes[2*id+1].box
			nodes[id].box = [2]r3.Vec{{X: math.Min(a[0].X, b[0].X), Y: math.Min(a[0].Y, b[0].Y), Z: math.Min(a[0].Z, b[0].Z)}, {X: math.Max(a[1].X, b[1].X), Y: math.Max(a[1].Y, b[1].Y), Z: math.Max(a[1].Z, b[1].Z)}}
			continue
		}
		nd := &nodes[id]
		if nd.hi-nd.lo == 1 {
			nd.box = bm.Boxes[nd.lo]
			continue
		}
		mid := nd.lo + (nd.hi-nd.lo)/2
		nodes[2*id].lo, nodes[2*id].hi = nd.lo, mid
		nodes[2*id+1].lo, nodes[2*id+1].hi = mid, nd.hi
		stack = append(stack, -id, 2*id+1, 2*id)
	}
	if err := meter.ReserveRecords(ctx, 2); err != nil {
		return false, err
	}
	if err := meter.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(proof.ReplayProductArena{}))); err != nil {
		return false, err
	}
	if err := meter.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(replayProductBudget{}))); err != nil {
		return false, err
	}
	arena := &proof.ReplayProductArena{}
	productBudget := &replayProductBudget{ReplayBudget: meter, arena: arena, scratch: replayProductScratch{meter}}
	runner := replayContactRunnerWithPlanes(meter, productBudget, orientationPlanes, heldSAT)
	fanRuns, err := replayOpenFanRuns(ctx, h, &bm, meter, productBudget)
	if err != nil {
		return false, err
	}
	for i, t := range h.Tris {
		if !h.Moved[t[0]] && !h.Moved[t[1]] && !h.Moved[t[2]] {
			continue
		}
		if zeroXpt(bm.Norms[i]) {
			return true, nil
		}
		stack = append(stack[:0], 1)
		for len(stack) > 0 {
			if err := meter.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			id := stack[len(stack)-1]
			stack = stack[:len(stack)-1]
			nd := nodes[id]
			if !BoxesOverlap(bm.Boxes[i], nd.box) {
				continue
			}
			if nd.hi-nd.lo > 1 {
				stack = append(stack, 2*id+1, 2*id)
				continue
			}
			j := nd.lo
			if i == j {
				continue
			}
			u := h.Tris[j]
			if j < i && (h.Moved[u[0]] || h.Moved[u[1]] || h.Moved[u[2]]) {
				continue
			}
			if zeroXpt(bm.Norms[j]) {
				return true, nil
			}
			var si, sj [3]int
			shared := 0
			for a, x := range t {
				for b, y := range u {
					if x == y {
						si[shared], sj[shared] = a, b
						shared++
					}
				}
			}
			if shared >= 3 {
				return true, nil
			}
			if shared == 1 && fanRuns != nil {
				if err := meter.SpendWork(ctx, 1); err != nil {
					return false, err
				}
				run := fanRuns[3*i+si[0]]
				if run != 0 && run == fanRuns[3*j+sj[0]] {
					continue
				}
			}
			if shared == 1 {
				pointOnly, err := replaySharedPointSeparatedAxes(ctx, TriCorners(&bm, i), TriCorners(&bm, j),
					bm.Norms[i], bm.Norms[j], meter)
				if err != nil {
					return false, err
				}
				if pointOnly {
					continue
				}
			}
			if shared == 2 {
				xa, xb := XtriCorners(&bm, i), XtriCorners(&bm, j)
				apexI := 3 - si[0] - si[1]
				apexJ := 3 - sj[0] - sj[1]
				sign, err := proof.ReplayOrientSignPrepared(ctx, h.Verts[t[si[0]]], h.Verts[t[si[1]]], h.Verts[t[apexI]], h.Verts[u[apexJ]], xa[si[0]], xb[apexJ], bm.Norms[i], meter)
				if err != nil {
					return false, err
				}
				if sign != 0 {
					continue
				}
				axisU, axisV, err := replayProjectionAxes(ctx, bm.Norms[i], meter)
				if err != nil {
					return false, err
				}
				p, err := replayCross2Numerator(ctx, xa[si[0]], xa[si[1]], xa[apexI], axisU, axisV, productBudget)
				if err != nil {
					return false, err
				}
				q, err := replayCross2Numerator(ctx, xa[si[0]], xa[si[1]], xb[apexJ], axisU, axisV, productBudget)
				if err != nil {
					return false, err
				}
				if p.Sign()*q.Sign() < 0 {
					continue
				}
				return true, nil
			}
			pair := [1]ContactPair{{I: i, J: j}}
			var result [1]ContactBatchResult
			if err := runner(ctx, &bm, &bm, pair[:], result[:], 0); err != nil {
				return false, err
			}
			if result[0].Err != nil {
				return false, result[0].Err
			}
			c := result[0].Contact
			switch shared {
			case 0:
				if c.Kind != ContactNone {
					return true, nil
				}
			case 1:
				if c.Kind != ContactPoint {
					return true, nil
				}
			case 2:
				switch c.Kind {
				case ContactSegment:
				case ContactRegion:
					xa, xb := XtriCorners(&bm, i), XtriCorners(&bm, j)
					axisU, axisV, err := replayProjectionAxes(ctx, bm.Norms[i], meter)
					if err != nil {
						return false, err
					}
					p, err := replayCross2Numerator(ctx, xa[si[0]], xa[si[1]], xa[3-si[0]-si[1]], axisU, axisV, productBudget)
					if err != nil {
						return false, err
					}
					q, err := replayCross2Numerator(ctx, xa[si[0]], xa[si[1]], xb[3-sj[0]-sj[1]], axisU, axisV, productBudget)
					if err != nil {
						return false, err
					}
					if p.Sign()*q.Sign() >= 0 {
						return true, nil
					}
				default:
					return true, nil
				}
			}
		}
	}
	return false, nil
}
