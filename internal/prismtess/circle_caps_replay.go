package prismtess

import (
	"context"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
)

func circleCapsReplay[F comparable](ctx context.Context, t *tessellation.PrismTopology[F],
	start, end F, z0, z1 float64, budget *proofbound.ReplayBudget) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if t == nil || budget == nil || !budget.IsBrakeReplay() {
		return fmt.Errorf("%w: circular caps require the brake replay budget", decaderr.ErrUnsupported)
	}
	n := len(t.Points)
	if n < 3 || n > freeform.MaxChordsPerWalk || len(t.LoopIdx) != 1 || len(t.LoopIdx[0]) != n ||
		len(t.LoopSag) != 1 || len(t.Triangles) != 2*n || len(t.Sources) != 2*n ||
		cap(t.Triangles) < 4*n-4 || cap(t.Sources) < 4*n-4 || cap(t.ReplayFaceBounds)-len(t.ReplayFaceBounds) < 2 {
		return fmt.Errorf("%w: invalid circular wall topology", decaderr.ErrUnsupported)
	}
	// EarClip removes at least one index per successful pass. At size k it
	// tries at most k corners and scans at most k blockers for each corner.
	// n^3 bounds their sum; the linear charge covers clearance, copies and
	// cap emission. n is capped above, so the arithmetic cannot overflow.
	work := uint64(n)*uint64(n)*uint64(n) + uint64(10*n+8)
	if err := budget.SpendWork(ctx, work); err != nil {
		return err
	}
	for _, i := range t.LoopIdx[0] {
		if i < 0 || i >= n {
			return fmt.Errorf("%w: invalid circular cap index", decaderr.ErrUnsupported)
		}
	}
	// One copied loop, one ear index slice, one triangle result, and the
	// empty-hole sort state. Wall facet arrays already hold the cap output.
	if err := budget.ReserveRecords(ctx, uint64(3*n+8)); err != nil {
		return err
	}
	for _, a := range [][2]uint64{
		{uint64(n), uint64(unsafe.Sizeof(int(0)))},
		{uint64(n), uint64(unsafe.Sizeof(int(0)))},
		{uint64(n), uint64(unsafe.Sizeof([3]int{}))},
		{8, uint64(unsafe.Sizeof(uintptr(0)))},
	} {
		if err := budget.ReserveArray(ctx, a[0], a[1]); err != nil {
			return err
		}
	}
	return tessellation.PrismCaps(ctx, t, false, start, end, z0, z1,
		tessellation.RequireLoopClearance, triangulation.Triangulate)
}
