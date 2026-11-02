package prismtess

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
)

// rectangleCapsReplay bounds the ordinary one-loop cap proof and triangulation.
func rectangleCapsReplay[F comparable](ctx context.Context, t *tessellation.PrismTopology[F],
	start, end F, z0, z1 float64, budget *proofbound.ReplayBudget) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if budget == nil || t == nil || len(t.Points) != 4 || len(t.LoopIdx) != 1 ||
		len(t.LoopIdx[0]) != 4 || len(t.LoopSag) != 1 || len(t.Triangles) != 8 ||
		len(t.Sources) != 8 || t.ReplayFaceBounds == nil {
		return fmt.Errorf("%w: bounded prism caps require four wall stations", decaderr.ErrUnsupported)
	}
	for _, i := range t.LoopIdx[0] {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		if i < 0 || i >= len(t.Points) {
			return fmt.Errorf("%w: bounded prism cap index is invalid", decaderr.ErrUnsupported)
		}
	}
	// Clearance scans the four points and then visits the single loop, with
	// no loop pair. EarClip removes at most two corners: it checks at most
	// 4+3 candidates and 4*4+3*3 blocking vertices. The remaining terms pay
	// for the initial index copy and the two emitted cap pairs.
	if err := budget.SpendWork(ctx, 4+1+4+2+4+3+4*4+3*3+4); err != nil {
		return err
	}
	// The face maps gain at most two entries each. Triangulation allocates
	// three four-record slices. The wall triangle and source slices can each
	// grow once from eight to at most sixteen records when four caps append.
	if err := budget.ReserveRecords(ctx, 4+3*4+2*16); err != nil {
		return err
	}
	return tessellation.PrismCaps(ctx, t, false, start, end, z0, z1,
		tessellation.RequireLoopClearance, triangulation.Triangulate)
}
