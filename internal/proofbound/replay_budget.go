package proofbound

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// ReplayBudget shares one work and record allowance across a private replay.
// A replay owns one instance for all of its steps. Brake limit errors are
// stored in that instance, retaining the first refusal for each resource.
type ReplayBudget struct {
	shared      MultiwayBudget
	bytes       uint64
	maxBytes    uint64
	brake       bool
	limitErrors [3]WorkLimitError
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

// BrakeReplay limits apply to the authenticated private brake route.
const (
	BrakeReplayWork    uint64 = 11735281028
	BrakeReplayRecords uint64 = 284417498
	BrakeReplayBytes   uint64 = 59714786988
)

// NewBrakeReplayBudget creates a meter for one private brake replay.
func NewBrakeReplayBudget() *ReplayBudget {
	b := &ReplayBudget{shared: MultiwayBudget{maxWork: BrakeReplayWork, maxRecords: BrakeReplayRecords}, maxBytes: BrakeReplayBytes, brake: true}
	b.shared.prebuiltLimits = &b.limitErrors
	return b
}

// IsBrakeReplay reports whether the meter belongs to the private brake route.
func (b *ReplayBudget) IsBrakeReplay() bool { return b != nil && b.brake }

// ReserveBytes charges cumulative allocated storage before allocation.
func (b *ReplayBudget) ReserveBytes(ctx context.Context, n uint64) error {
	limit := b.maxBytes
	if limit == 0 {
		limit = ReplayTrialBytes
	}
	return multiwayCharge(ctx, &b.bytes, n, limit, "replay_bytes", b.shared.prebuiltLimits)
}

var errBrakeArrayOverflow = fmt.Errorf("%w: replay allocation size overflow", decaderr.ErrUnsupported)
var errBrakeEnvelopeOverflow = fmt.Errorf("%w: replay allocation envelope overflow", decaderr.ErrUnsupported)

// ReserveArray charges a checked backing array and a conservative allocation envelope.
func (b *ReplayBudget) ReserveArray(ctx context.Context, count, elementBytes uint64) error {
	if elementBytes != 0 && count > ^uint64(0)/elementBytes {
		if b.brake {
			return errBrakeArrayOverflow
		}
		return fmt.Errorf("%w: replay allocation size overflow", decaderr.ErrUnsupported)
	}
	size := count * elementBytes
	if size > (^uint64(0)-128)/2 {
		if b.brake {
			return errBrakeEnvelopeOverflow
		}
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
	work.Replay = true
	work.ReplayID = b
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
