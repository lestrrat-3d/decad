package proofbound

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// ReplayBudget shares one work and record allowance across a private replay.
// A replay owns one instance for all of its steps.
type ReplayBudget struct {
	shared   MultiwayBudget
	bytes    uint64
	maxBytes uint64
}

// Replay trial limits bound the shared trial meter.
const (
	ReplayTrialWork    uint64 = 32000000
	ReplayTrialRecords uint64 = 4194304
	ReplayTrialBytes   uint64 = 2 << 30
)

// NewReplayTrialBudget creates a shared meter for the bounded trial.
func NewReplayTrialBudget() *ReplayBudget {
	return &ReplayBudget{shared: MultiwayBudget{maxWork: ReplayTrialWork, maxRecords: ReplayTrialRecords}}
}

// NewReplayBudgetLimits creates a replay budget no larger than the trial caps.
// All limits must be positive; smaller limits support bounded callers and tests.
func NewReplayBudgetLimits(work, records, bytes uint64) (*ReplayBudget, error) {
	if work == 0 || records == 0 || bytes == 0 || work > ReplayTrialWork || records > ReplayTrialRecords || bytes > ReplayTrialBytes {
		return nil, fmt.Errorf("%w: invalid replay limits", decaderr.ErrUnsupported)
	}
	return &ReplayBudget{shared: MultiwayBudget{maxWork: work, maxRecords: records}, maxBytes: bytes}, nil
}

// ReserveBytes charges cumulative allocated storage before allocation.
func (b *ReplayBudget) ReserveBytes(ctx context.Context, n uint64) error {
	limit := b.maxBytes
	if limit == 0 {
		limit = ReplayTrialBytes
	}
	return multiwayCharge(ctx, &b.bytes, n, limit, "replay_bytes")
}

// ReserveArray charges a checked backing array and a conservative allocation envelope.
func (b *ReplayBudget) ReserveArray(ctx context.Context, count, elementBytes uint64) error {
	if elementBytes != 0 && count > ^uint64(0)/elementBytes {
		return fmt.Errorf("%w: replay allocation size overflow", decaderr.ErrUnsupported)
	}
	size := count * elementBytes
	if size > (^uint64(0)-128)/2 {
		return fmt.Errorf("%w: replay allocation envelope overflow", decaderr.ErrUnsupported)
	}
	return b.ReserveBytes(ctx, 2*size+128)
}

// Bytes returns cumulative precharged storage.
func (b *ReplayBudget) Bytes() uint64 { return b.bytes }

// SpendWork charges work before it is performed.
func (b *ReplayBudget) SpendWork(ctx context.Context, n uint64) error {
	return b.shared.SpendWork(ctx, n)
}

// ReserveRecords charges variable-capacity records before allocation or insertion.
func (b *ReplayBudget) ReserveRecords(ctx context.Context, n uint64) error {
	return b.shared.ReserveRecords(ctx, n)
}

// WorkBudget adapts the same counters to proof code that takes a WorkBudget.
func (b *ReplayBudget) WorkBudget(ctx context.Context) *WorkBudget {
	b.shared.reserveBytes = b.ReserveBytes
	work := b.shared.WorkBudget(ctx)
	return work
}

// MultiwayBudget returns the same counter for existing bounded proof helpers.
func (b *ReplayBudget) MultiwayBudget() *MultiwayBudget {
	b.shared.reserveBytes = b.ReserveBytes
	return &b.shared
}

// Usage returns the cumulative work and record charges.
func (b *ReplayBudget) Usage() (work, records uint64) {
	return b.shared.Work, b.shared.Records
}
