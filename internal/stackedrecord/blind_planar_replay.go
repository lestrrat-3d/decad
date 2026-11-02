package stackedrecord

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// reserveReplayAppend charges the elements and a conservative backing-array
// envelope before an append of count elements to a private Build slice.
func reserveReplayAppend(ctx context.Context, length, capacity, count int, elemBytes uint64,
	budget *proofbound.ReplayBudget) error {
	if budget == nil || length < 0 || capacity < length || count < 0 ||
		count > int(^uint(0)>>1)-length {
		return decaderr.ErrUnsupported
	}
	if err := budget.ReserveBundle(ctx, uint64(count), uint64(count), 0); err != nil {
		return err
	}
	needed := length + count
	if needed <= capacity {
		return nil
	}
	if capacity > int(^uint(0)>>1)/2 {
		return decaderr.ErrUnsupported
	}
	newCap := max(needed, max(1, 2*capacity))
	return budget.ReserveArray(ctx, uint64(newCap), elemBytes)
}

func appendPlanarLoopReplay(ctx context.Context, alloc topo.Alloc, face *topo.Face,
	loop topo.Loop, budget *proofbound.ReplayBudget) error {
	if err := reserveReplayAppend(ctx, len(face.Loops), cap(face.Loops), 1,
		uint64(unsafe.Sizeof((*topo.Loop)(nil))), budget); err != nil {
		return err
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(topo.Loop{}))); err != nil {
		return err
	}
	face.Loops = append(face.Loops, topobuild.NewLoop(alloc, loop))
	return nil
}

// replayAnchorInsert charges a map insertion before the uncommitted body's
// anchor set grows. A pointer-key bucket and its growth are below 256 bytes
// amortized per distinct entry, including overflow bucket space.
func replayAnchorInsert(ctx context.Context, anchors map[*topo.Face]struct{}, face *topo.Face,
	budget *proofbound.ReplayBudget) error {
	if err := budget.ReserveBundle(ctx, 1, 1, 256); err != nil {
		return err
	}
	anchors[face] = struct{}{}
	return nil
}
