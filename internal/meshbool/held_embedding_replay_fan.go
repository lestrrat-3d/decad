package meshbool

import (
	"context"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// replayOpenFanRuns assigns at most one proven open run to each center.
// A zero entry never authorizes skipping a contact pair.
func replayOpenFanRuns(ctx context.Context, h HeldRounding, bm *BoolMesh, b *proofbound.ReplayBudget, product proof.ReplayKeyBudget) ([]int, error) {
	nv, nf := len(h.Verts), len(h.Tris)
	if nv == int(^uint(0)>>1) || nf > int(^uint(0)>>1)/3 {
		return nil, errReplayFanIncidenceOverflow
	}
	corners := 3 * nf
	// Offsets, cursor, directed link stamps/entries, ring, and corner IDs.
	sizes := []int{nv + 1, nv, nv, nv, nv, nv, nv, corners, corners}
	var total uint64
	for _, n := range sizes {
		if uint64(n) > ^uint64(0)-total {
			return nil, errReplayFanStorageOverflow
		}
		total += uint64(n)
	}
	if err := b.ReserveRecords(ctx, total); err != nil {
		return nil, err
	}
	for _, n := range sizes {
		if err := b.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return nil, err
		}
	}
	if err := b.SpendWork(ctx, total); err != nil {
		return nil, err
	}
	offsets := make([]int, nv+1)
	cursor := make([]int, nv)
	outStamp := make([]int, nv)
	inStamp := make([]int, nv)
	next := make([]int, nv)
	atCorner := make([]int, nv)
	ring := make([]int, nv)
	inc := make([]int, corners)
	runs := make([]int, corners)
	if uint64(corners) > ^uint64(0)/2 {
		return nil, errReplayFanWorkOverflow
	}
	if err := b.SpendWork(ctx, 2*uint64(corners)); err != nil {
		return nil, err
	}
	for _, t := range h.Tris {
		for _, v := range t {
			if v < 0 || v >= nv {
				return nil, errReplayInvalidFanVertex
			}
			offsets[v+1]++
		}
	}
	for i := range nv {
		if err := b.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		offsets[i+1] += offsets[i]
		cursor[i] = offsets[i]
	}
	for f, t := range h.Tris {
		for slot, v := range t {
			inc[cursor[v]] = 3*f + slot
			cursor[v]++
		}
	}
	for center := range nv {
		if err := b.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		degree := offsets[center+1] - offsets[center]
		if degree < 3 || degree > nv {
			continue
		}
		stamp := center + 1
		valid := true
		start := -1
		for _, co := range inc[offsets[center]:offsets[center+1]] {
			if err := b.SpendWork(ctx, 8); err != nil {
				return nil, err
			}
			f, slot := co/3, co%3
			t := h.Tris[f]
			u, v := t[(slot+1)%3], t[(slot+2)%3]
			if u == v || u == center || v == center || outStamp[u] == stamp || inStamp[v] == stamp {
				valid = false
				break
			}
			outStamp[u], inStamp[v], next[u], atCorner[u] = stamp, stamp, v, co
			if start < 0 {
				start = u
			}
		}
		if !valid {
			continue
		}
		at := start
		for k := range degree {
			if err := b.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			if at < 0 || at >= nv || outStamp[at] != stamp || (k > 0 && at == start) {
				valid = false
				break
			}
			ring[k] = atCorner[at]
			at = next[at]
		}
		if !valid || at != start {
			continue
		}
		bestStart, bestLen := 0, 0
		for begin := 0; begin < degree; {
			if err := b.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			co := ring[begin]
			f, slot := co/3, co%3
			t := h.Tris[f]
			normal := bm.Norms[f]
			axis := -1
			for k, x := range []*big.Int{normal.X, normal.Y, normal.Z} {
				if x != nil && x.Sign() != 0 {
					axis = k
					break
				}
			}
			if axis < 0 {
				begin++
				continue
			}
			u, v := (axis+1)%3, (axis+2)%3
			if []*big.Int{normal.X, normal.Y, normal.Z}[axis].Sign() < 0 {
				u, v = v, u
			}
			p := bm.Xverts[center]
			first := bm.Xverts[t[(slot+1)%3]]
			prev := bm.Xverts[t[(slot+2)%3]]
			sector, err := replayCross2Numerator(ctx, p, first, prev, u, v, product)
			if err != nil {
				return nil, err
			}
			if sector.Sign() <= 0 {
				begin++
				continue
			}
			prevHalf := 0
			end := begin + 1
			for end < degree {
				if err := b.SpendWork(ctx, 1); err != nil {
					return nil, err
				}
				nextCo := ring[end]
				nf2, ns := nextCo/3, nextCo%3
				nt := h.Tris[nf2]
				qv := nt[(ns+2)%3]
				q := bm.Xverts[qv]
				sign, err := proof.ReplayOrientSignPrepared(ctx, h.Verts[t[0]], h.Verts[t[1]], h.Verts[t[2]], h.Verts[qv], bm.Xverts[t[0]], q, normal, b)
				if err != nil {
					return nil, err
				}
				if sign != 0 {
					break
				}
				turn, err := replayCross2Numerator(ctx, p, prev, q, u, v, product)
				if err != nil {
					return nil, err
				}
				if turn.Sign() <= 0 {
					break
				}
				startCross, err := replayCross2Numerator(ctx, p, first, q, u, v, product)
				if err != nil {
					return nil, err
				}
				half := 0
				if startCross.Sign() < 0 {
					half = 1
				} else if startCross.Sign() == 0 {
					d, err := replayFanDot(ctx, p, first, q, u, v, product)
					if err != nil {
						return nil, err
					}
					if d.Sign() >= 0 {
						break
					}
					half = 1
				}
				if half < prevHalf {
					break
				}
				prevHalf = half
				prev = q
				end++
			}
			if end-begin > bestLen {
				bestStart, bestLen = begin, end-begin
			}
			begin = end
		}
		if bestLen < 3 {
			continue
		}
		if err := b.SpendWork(ctx, uint64(bestLen)); err != nil {
			return nil, err
		}
		for _, co := range ring[bestStart : bestStart+bestLen] {
			runs[co] = stamp
		}
	}
	return runs, nil
}

// replayFanDot returns the exact projected dot of (a-p) and (b-p).
func replayFanDot(ctx context.Context, p, a, b proof.Xpt, u, v int, m proof.ReplayKeyBudget) (*big.Int, error) {
	var d [4]*big.Int
	for i, term := range [4]struct {
		x    proof.Xpt
		axis int
	}{{a, u}, {a, v}, {b, u}, {b, v}} {
		x, err := proof.ReplayProductDifference(ctx, Coord(term.x, term.axis), p.W, Coord(p, term.axis), term.x.W, m)
		if err != nil {
			return nil, err
		}
		d[i] = x
	}
	x, err := proof.ReplayProductDifference(ctx, d[0], d[2], replayExactZero, replayExactZero, m)
	if err != nil {
		return nil, err
	}
	return proof.ReplayProductDifference(ctx, d[1], d[3], replayExactNegativeOne, x, m)
}
