package prismtess

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
)

// circleCapsReplay adds the two caps only after clearance and triangulation succeed.
func circleCapsReplay[F comparable](ctx context.Context, t *tessellation.PrismTopology[F], start, end F, budget *proofbound.ReplayBudget) error {
	if budget == nil {
		return decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return err
	}
	if t == nil || start == end {
		return decaderr.ErrUnsupported
	}
	n := len(t.Points)
	if n < 3 || n > freeform.MaxChordsPerWalk || len(t.LoopIdx) != 1 || len(t.LoopIdx[0]) != n || len(t.LoopSag) != 1 || len(t.Triangles) != 2*n || len(t.Sources) != 2*n || len(t.ReplayFaceBounds) != 1 || start == t.ReplayFaceBounds[0].Face || end == t.ReplayFaceBounds[0].Face || cap(t.Triangles) < 4*n-4 || cap(t.Sources) < 4*n-4 || cap(t.ReplayFaceBounds) < 3 {
		return decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, uint64(32*n+128)); err != nil {
		return err
	}
	minU, maxU, minV, maxV, maxAbs := math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1), 0.0
	for i, p := range t.Points {
		if i%128 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		if proofbound.IsNonFinite(p.U) || proofbound.IsNonFinite(p.V) || t.LoopIdx[0][i] != i {
			return decaderr.ErrUnsupported
		}
		minU, maxU = math.Min(minU, p.U), math.Max(maxU, p.U)
		minV, maxV = math.Min(minV, p.V), math.Max(maxV, p.V)
		maxAbs = math.Max(maxAbs, math.Max(math.Abs(p.U), math.Abs(p.V)))
	}
	floor := 1e-9*math.Hypot(maxU-minU, maxV-minV) + 4*(math.Nextafter(maxAbs, math.Inf(1))-maxAbs)
	if proofbound.IsNonFinite(floor) {
		return decaderr.ErrUnsupported
	}
	// The ordinary one-loop clearance call still computes its scale floor.
	// Reserve its work budget and closures before invoking it.
	if err := budget.ReserveRecords(ctx, 4); err != nil {
		return err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(proofbound.WorkBudget{}))); err != nil {
		return err
	}
	if err := budget.ReserveArray(ctx, 3, 128); err != nil {
		return err
	}
	if err := tessellation.RequireLoopClearance(ctx, t.Points, t.LoopIdx, t.LoopSag); err != nil {
		return err
	}
	tris, err := triangulation.ConvexFanReplay(ctx, t.Points, t.LoopIdx[0], budget)
	if err != nil {
		return err
	}
	if len(tris) != n-2 || math.IsNaN(t.CapTrim) || math.IsInf(t.CapTrim, 0) || t.CapTrim < 0 {
		return decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, uint64(16*n+16)); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	// No allocations or failing operations remain after this point.
	t.ReplayFaceBounds = append(t.ReplayFaceBounds, tessellation.PrismReplayFaceBound[F]{Face: start, Trim: t.CapTrim}, tessellation.PrismReplayFaceBound[F]{Face: end, Trim: t.CapTrim})
	for _, tri := range tris {
		t.Triangles = append(t.Triangles, [3]int{2*tri[0] + 1, 2*tri[1] + 1, 2*tri[2] + 1}, [3]int{2 * tri[0], 2 * tri[2], 2 * tri[1]})
		t.Sources = append(t.Sources, end, start)
	}
	return nil
}
