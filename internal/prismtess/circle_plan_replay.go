package prismtess

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

func circlePlanReplay(ctx context.Context, pp prismrecord.Payload, b *proofbound.ReplayBudget) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if b == nil || !b.IsBrakeReplay() || pp.SurfaceResult || len(pp.Profile.Holes) != 0 ||
		len(pp.Profile.Outer.Segments) != 1 || pp.Walks == nil || !pp.Walks.Metered ||
		pp.Walks.Spent != 0 || pp.Walks.ReconstructionSpent != 0 || len(pp.Walks.Profile.Holes) != 0 ||
		len(pp.Walks.Profile.Outer.Segments) != 1 || len(pp.Walks.Outer) != 1 || len(pp.Walks.Holes) != 0 {
		return fmt.Errorf("%w: bounded circle requires one recorded solid loop", decaderr.ErrUnsupported)
	}
	if _, ok := pp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg); !ok {
		return fmt.Errorf("%w: bounded circle requires a circle record", decaderr.ErrUnsupported)
	}
	if _, ok := pp.Walks.Profile.Outer.Segments[0].(sectionrecord.CircleSeg); !ok {
		return fmt.Errorf("%w: bounded circle requires recorded circle walks", decaderr.ErrUnsupported)
	}
	if pp.Frame.U() != (r3.Vec{X: 1}) || pp.Frame.V() != (r3.Vec{Y: 1}) || pp.Frame.N() != (r3.Vec{Z: 1}) {
		return fmt.Errorf("%w: bounded circle requires an axis-aligned frame", decaderr.ErrUnsupported)
	}
	basis := pp.Xform.Basis()
	if basis.EX != (r3.Vec{X: 1}) || basis.EY != (r3.Vec{Y: 1}) || basis.EZ != (r3.Vec{Z: 1}) {
		return fmt.Errorf("%w: bounded circle requires translation placement", decaderr.ErrUnsupported)
	}
	if err := b.SpendWork(ctx, 256); err != nil {
		return err
	}
	if err := b.ReserveRecords(ctx, 32); err != nil {
		return err
	}
	// The fixed setup owns the loop and role slices and their adapters.
	return b.ReserveArray(ctx, 32, 256)
}
