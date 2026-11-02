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
	prebuiltLimits      *[3]WorkLimitError
	strictReplay        *ReplayBudget
}

var brakeOverflowCauses = [3]error{
	fmt.Errorf("%w: multiway multiway_work count overflow", decaderr.ErrUnsupported),
	fmt.Errorf("%w: multiway multiway_records count overflow", decaderr.ErrUnsupported),
	fmt.Errorf("%w: multiway replay_bytes count overflow", decaderr.ErrUnsupported),
}

var brakeLimitCauses = [3]error{
	fmt.Errorf("%w: multiway multiway_work budget exceeded", decaderr.ErrUnsupported),
	fmt.Errorf("%w: multiway multiway_records budget exceeded", decaderr.ErrUnsupported),
	fmt.Errorf("%w: multiway replay_bytes budget exceeded", decaderr.ErrUnsupported),
}

func brakeResourceIndex(resource string) int {
	switch resource {
	case "multiway_records":
		return 1
	case "replay_bytes":
		return 2
	default:
		return 0
	}
}

func multiwayCharge(ctx context.Context, used *uint64, add, limit uint64, resource string, prebuilt *[3]WorkLimitError) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if *used > limit || add > ^uint64(0)-*used {
		if prebuilt != nil {
			return brakeOverflowCauses[brakeResourceIndex(resource)]
		}
		return fmt.Errorf("%w: multiway %s count overflow", decaderr.ErrUnsupported, resource)
	}
	next := *used + add
	if next > limit {
		if prebuilt != nil {
			index := brakeResourceIndex(resource)
			detail := &prebuilt[index]
			if detail.err == nil {
				*detail = WorkLimitError{Resource: resource, Observed: next, Maximum: limit, err: brakeLimitCauses[index]}
			}
			return detail
		}
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
	if b.strictReplay != nil {
		return b.strictReplay.charge(ctx, &b.Records, n, limit, "multiway_records")
	}
	return multiwayCharge(ctx, &b.Records, n, limit, "multiway_records", b.prebuiltLimits)
}

// SpendWork charges before candidate, predicate or proof work.
func (b *MultiwayBudget) SpendWork(ctx context.Context, n uint64) error {
	limit := b.maxWork
	if limit == 0 {
		limit = MaxMultiwayWorkPerCall
	}
	if b.strictReplay != nil {
		return b.strictReplay.charge(ctx, &b.Work, n, limit, "multiway_work")
	}
	return multiwayCharge(ctx, &b.Work, n, limit, "multiway_work", b.prebuiltLimits)
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
	work := &WorkBudget{StepFn: func() error { return b.SpendWork(ctx, 1) }, ErrFn: ctx.Err,
		ReserveFn: func(n uint64) error { return b.ReserveRecords(ctx, n) },
		ReserveBytesFn: func(n uint64) error {
			if b.reserveBytes == nil {
				return nil
			}
			return b.reserveBytes(ctx, n)
		}}
	if b.prebuiltLimits != nil {
		work.SpendWorkFn = func(n uint64) error { return b.SpendWork(ctx, n) }
	}
	return work
}
