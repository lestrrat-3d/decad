package proofbound

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// Multiway ceilings are independent of the existing Revolve facet ceilings.
const (
	MaxMultiwayRecordsPerCall uint64 = 2097152
	MaxMultiwayWorkPerCall    uint64 = 8000000
)

// MultiwayBudget counts cumulative logical records and work for one call.
type MultiwayBudget struct {
	Records, Work       uint64
	maxRecords, maxWork uint64
	reserveBytes        func(context.Context, uint64) error
}

func multiwayCharge(ctx context.Context, used *uint64, add, limit uint64, resource string) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if *used > limit || add > ^uint64(0)-*used {
		return fmt.Errorf("%w: multiway %s count overflow", decaderr.ErrUnsupported, resource)
	}
	next := *used + add
	if next > limit {
		return WithWorkLimit(fmt.Errorf("%w: multiway %s budget exceeded", decaderr.ErrUnsupported, resource), resource, next, limit)
	}
	*used = next
	return nil
}

// ReserveRecords charges before a variable-capacity allocation or insertion.
func (b *MultiwayBudget) ReserveRecords(ctx context.Context, n uint64) error {
	limit := b.maxRecords
	if limit == 0 {
		limit = MaxMultiwayRecordsPerCall
	}
	return multiwayCharge(ctx, &b.Records, n, limit, "multiway_records")
}

// SpendWork charges before candidate, predicate or proof work.
func (b *MultiwayBudget) SpendWork(ctx context.Context, n uint64) error {
	limit := b.maxWork
	if limit == 0 {
		limit = MaxMultiwayWorkPerCall
	}
	return multiwayCharge(ctx, &b.Work, n, limit, "multiway_work")
}

// ReserveBytes passes storage charges to the replay meter when present.
func (b *MultiwayBudget) ReserveBytes(ctx context.Context, n uint64) error {
	if b.reserveBytes == nil {
		return nil
	}
	return b.reserveBytes(ctx, n)
}

// WorkBudget adapts one call's counters to existing proof steps and reservations.
func (b *MultiwayBudget) WorkBudget(ctx context.Context) *WorkBudget {
	return &WorkBudget{StepFn: func() error { return b.SpendWork(ctx, 1) }, ErrFn: ctx.Err,
		ReserveFn: func(n uint64) error { return b.ReserveRecords(ctx, n) },
		ReserveBytesFn: func(n uint64) error {
			if b.reserveBytes == nil {
				return nil
			}
			return b.reserveBytes(ctx, n)
		}}
}
