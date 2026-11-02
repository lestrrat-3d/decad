package proofbound

import (
	"context"
	"fmt"
	"strconv"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// ReplayBudget shares one work and record allowance across a private replay.
// A replay owns one instance for all of its steps.
type ReplayBudget struct {
	shared     MultiwayBudget
	bytes      uint64
	maxBytes   uint64
	strict     bool
	failure    error
	limitError WorkLimitError
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

// NewCircleReplayBudget creates a fail-closed trial meter whose own fixed
// object and limit refusal are accounted before any source proof can run.
func NewCircleReplayBudget() *ReplayBudget {
	b := &ReplayBudget{shared: MultiwayBudget{maxWork: ReplayTrialWork, maxRecords: ReplayTrialRecords}, strict: true}
	b.shared.strictReplay = b
	b.shared.Records = 1
	b.bytes = 2*uint64(unsafe.Sizeof(ReplayBudget{})) + 128
	return b
}

// IsCircleReplay reports whether b uses the fail-closed circle meter.
func (b *ReplayBudget) IsCircleReplay() bool { return b != nil && b.strict }

var (
	errCircleReplayOverflow = fmt.Errorf("%w: replay counter overflow", decaderr.ErrUnsupported)
	errCircleReplayWork     = fmt.Errorf("%w: multiway multiway_work budget exceeded", decaderr.ErrUnsupported)
	errCircleReplayRecords  = fmt.Errorf("%w: multiway multiway_records budget exceeded", decaderr.ErrUnsupported)
	errCircleReplayBytes    = fmt.Errorf("%w: multiway replay_bytes budget exceeded", decaderr.ErrUnsupported)
	errCircleReplayArray    = fmt.Errorf("%w: replay allocation size overflow", decaderr.ErrUnsupported)
	errCircleReplayEnvelope = fmt.Errorf("%w: replay allocation envelope overflow", decaderr.ErrUnsupported)
)

func (b *ReplayBudget) charge(ctx context.Context, used *uint64, add, limit uint64, resource string) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if b.failure != nil {
		return b.failure
	}
	if *used > limit || add > ^uint64(0)-*used {
		b.failure = errCircleReplayOverflow
		return b.failure
	}
	next := *used + add
	if next > limit {
		var cause error
		switch resource {
		case "multiway_work":
			cause = errCircleReplayWork
		case "multiway_records":
			cause = errCircleReplayRecords
		case "replay_bytes":
			cause = errCircleReplayBytes
		default:
			b.failure = errCircleReplayOverflow
			return b.failure
		}
		b.limitError = WorkLimitError{Resource: resource, Observed: next, Maximum: limit, err: cause}
		b.failure = &b.limitError
		return b.failure
	}
	*used = next
	return nil
}

// ReserveBytes charges cumulative allocated storage before allocation.
func (b *ReplayBudget) ReserveBytes(ctx context.Context, n uint64) error {
	limit := b.maxBytes
	if limit == 0 {
		limit = ReplayTrialBytes
	}
	if b.strict {
		return b.charge(ctx, &b.bytes, n, limit, "replay_bytes")
	}
	return multiwayCharge(ctx, &b.bytes, n, limit, "replay_bytes")
}

// ReserveArray charges a checked backing array and a conservative allocation envelope.
func (b *ReplayBudget) ReserveArray(ctx context.Context, count, elementBytes uint64) error {
	if b.strict {
		if err := ctx.Err(); err != nil {
			return err
		}
		if b.failure != nil {
			return b.failure
		}
	}
	if elementBytes != 0 && count > ^uint64(0)/elementBytes {
		if b.strict {
			b.failure = errCircleReplayArray
			return b.failure
		}
		return fmt.Errorf("%w: replay allocation size overflow", decaderr.ErrUnsupported)
	}
	size := count * elementBytes
	if size > (^uint64(0)-128)/2 {
		if b.strict {
			b.failure = errCircleReplayEnvelope
			return b.failure
		}
		return fmt.Errorf("%w: replay allocation envelope overflow", decaderr.ErrUnsupported)
	}
	return b.ReserveBytes(ctx, 2*size+128)
}

type replayIndexedError struct{ text string }

func (e *replayIndexedError) Error() string { return e.text }
func (e *replayIndexedError) Unwrap() error { return decaderr.ErrUnsupported }

// ReplayIndexedRefusal constructs an unsupported error with one or two indices.
// Its fixed buffer holds the prefix, at most 400 bytes of literal text, and two
// signed machine integers. All possible backing storage is reserved first.
func ReplayIndexedRefusal(ctx context.Context, b *ReplayBudget, first, middle, last string, i, j int, two bool) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if b == nil || len(first) > 400 || len(middle) > 400 || len(last) > 400 ||
		len(first)+len(middle)+len(last) > 400 || len(decaderr.ErrUnsupported.Error())+442 > 512 {
		return decaderr.ErrUnsupported
	}
	if err := b.SpendWork(ctx, 2048); err != nil {
		return err
	}
	if err := b.ReserveRecords(ctx, 3); err != nil {
		return err
	}
	for range 2 {
		if err := b.ReserveArray(ctx, 512, 1); err != nil {
			return err
		}
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(replayIndexedError{}))); err != nil {
		return err
	}
	var storage [512]byte
	out := append(storage[:0], decaderr.ErrUnsupported.Error()...)
	out = append(out, ':', ' ')
	out = append(out, first...)
	out = strconv.AppendInt(out, int64(i), 10)
	out = append(out, middle...)
	if two {
		out = strconv.AppendInt(out, int64(j), 10)
	}
	out = append(out, last...)
	return &replayIndexedError{text: string(out)}
}

// ReserveBundle checks all three replay limits before committing any charge.
// It lets one fixed exact-arithmetic step refuse without consuming part of
// its reservation when a later resource runs out.
func (b *ReplayBudget) ReserveBundle(ctx context.Context, work, records, bytes uint64) error {
	if b == nil {
		return fmt.Errorf("%w: nil replay budget", decaderr.ErrUnsupported)
	}
	if b.strict {
		return errCircleReplayEnvelope
	}
	w, r, n := b.shared.Work, b.shared.Records, b.bytes
	wLimit, rLimit, bLimit := b.shared.maxWork, b.shared.maxRecords, b.maxBytes
	if wLimit == 0 {
		wLimit = MaxMultiwayWorkPerCall
	}
	if rLimit == 0 {
		rLimit = MaxMultiwayRecordsPerCall
	}
	if bLimit == 0 {
		bLimit = ReplayTrialBytes
	}
	if err := multiwayCharge(ctx, &w, work, wLimit, "multiway_work"); err != nil {
		return err
	}
	if err := multiwayCharge(ctx, &r, records, rLimit, "multiway_records"); err != nil {
		return err
	}
	if err := multiwayCharge(ctx, &n, bytes, bLimit, "replay_bytes"); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	b.shared.Work, b.shared.Records, b.bytes = w, r, n
	return nil
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
