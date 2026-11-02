package meshbool

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type replayEntry[K comparable, V any] struct {
	key   K
	value V
	used  bool
}

type replayTable[K comparable, V any] struct {
	slots          []replayEntry[K, V]
	count, limit   int
	budget         *proofbound.ReplayBudget
	hash           func(K) uint64
	comparisonWork func(K) uint64
	grow           bool
}

func newReplayTable[K comparable, V any](ctx context.Context, limit int, budget *proofbound.ReplayBudget,
	hash func(K) uint64) (*replayTable[K, V], error) {
	maxInt := int(^uint(0) >> 1)
	if limit < 0 || limit > maxInt/2 || budget == nil || hash == nil {
		return nil, errReplayInvalidBoundedTable
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return nil, err
	}
	n := 2
	for n < 2*limit {
		if n > maxInt/2 {
			return nil, errReplayBoundedTableSizeOverflow
		}
		n *= 2
	}
	if err := budget.ReserveRecords(ctx, uint64(n)+1); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(replayEntry[K, V]{}))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(replayTable[K, V]{}))); err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, uint64(n)); err != nil {
		return nil, err
	}
	return &replayTable[K, V]{slots: make([]replayEntry[K, V], n), limit: limit, budget: budget, hash: hash}, nil
}

func newReplayGrowingTable[K comparable, V any](ctx context.Context, limit int, budget *proofbound.ReplayBudget, hash func(K) uint64) (*replayTable[K, V], error) {
	if limit < 0 {
		return nil, errReplayInvalidBoundedTableCapacity
	}
	out, err := newReplayTable[K, V](ctx, min(limit, 16), budget, hash)
	if err != nil {
		return nil, err
	}
	out.limit = limit
	out.grow = true
	return out, nil
}

func (t *replayTable[K, V]) expand(ctx context.Context) error {
	next := min(t.limit, max(1, len(t.slots)))
	out, err := newReplayTable[K, V](ctx, next, t.budget, t.hash)
	if err != nil {
		return err
	}
	out.comparisonWork = t.comparisonWork
	if err := t.budget.SpendWork(ctx, uint64(len(t.slots))); err != nil {
		return err
	}
	for _, entry := range t.slots {
		if entry.used {
			if err := out.Set(ctx, entry.key, entry.value); err != nil {
				return err
			}
		}
	}
	t.slots = out.slots
	t.count = out.count
	return nil
}

func (t *replayTable[K, V]) find(ctx context.Context, key K) (int, bool, error) {
	if t == nil || t.budget == nil || len(t.slots) == 0 {
		return 0, false, errReplayInvalidBoundedTableLookup
	}
	if err := t.budget.SpendWork(ctx, 65); err != nil {
		return 0, false, err
	}
	i := int(t.hash(key) & uint64(len(t.slots)-1))
	for range len(t.slots) {
		cost := uint64(1)
		if t.comparisonWork != nil {
			extra := t.comparisonWork(key)
			if extra == ^uint64(0) {
				return 0, false, errReplayBoundedComparisonOverflow
			}
			cost += extra
		}
		if err := t.budget.SpendWork(ctx, cost); err != nil {
			return 0, false, err
		}
		e := t.slots[i]
		if !e.used {
			return i, false, nil
		}
		if e.key == key {
			return i, true, nil
		}
		i = (i + 1) & (len(t.slots) - 1)
	}
	return 0, false, errReplayBoundedTableIsFull
}
func (t *replayTable[K, V]) Get(ctx context.Context, key K) (V, bool, error) {
	var zero V
	i, ok, err := t.find(ctx, key)
	if err != nil || !ok {
		return zero, ok, err
	}
	return t.slots[i].value, true, nil
}
func (t *replayTable[K, V]) Set(ctx context.Context, key K, value V) error {
	i, ok, err := t.find(ctx, key)
	if err != nil {
		return err
	}
	if !ok {
		if t.count >= t.limit {
			return errReplayBoundedTableCapacityExceeded
		}
		if t.grow && t.count >= len(t.slots)/2 {
			if err := t.expand(ctx); err != nil {
				return err
			}
			i, ok, err = t.find(ctx, key)
			if err != nil {
				return err
			}
			if ok {
				return errReplayDuplicatedBoundedTableEntry
			}
		}
		t.count++
	}
	t.slots[i] = replayEntry[K, V]{key: key, value: value, used: true}
	return nil
}

func replayStringHash(s string) uint64 {
	h := uint64(1469598103934665603) ^ uint64(len(s))
	// A fixed-size prefix and suffix keeps hashing bounded. Equality still
	// checks the full string after its work is charged.
	for i := range min(len(s), 32) {
		h ^= uint64(s[i])
		h *= 1099511628211
	}
	for i := max(32, len(s)-32); i < len(s); i++ {
		h ^= uint64(s[i])
		h *= 1099511628211
	}
	return h
}

func replayEdgeHash(edge [2]int) uint64 {
	x := uint64(edge[0])*0x9e3779b97f4a7c15 ^ uint64(edge[1])*0xbf58476d1ce4e5b9
	x ^= x >> 30
	x *= 0xbf58476d1ce4e5b9
	x ^= x >> 27
	return x
}

func appendReplay[T any](ctx context.Context, values []T, value T, budget *PairCutsBudget) ([]T, error) {
	if budget == nil || budget.replay == nil {
		return append(values, value), nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, err
	}
	if len(values) == cap(values) {
		maxInt := int(^uint(0) >> 1)
		if cap(values) > maxInt/2 {
			return nil, errReplayReplaySliceCapacityOverflow
		}
		n := max(1, 2*cap(values))
		if err := budget.ReserveRecords(ctx, uint64(n)); err != nil {
			return nil, err
		}
		if err := budget.ReserveBytes(ctx, uint64(n), uint64(unsafe.Sizeof(*new(T)))); err != nil {
			return nil, err
		}
		if err := budget.SpendWork(ctx, uint64(len(values))); err != nil {
			return nil, err
		}
		next := make([]T, len(values), n)
		copy(next, values)
		values = next
	}
	return append(values, value), nil
}

func replayVecHash(v r3.Vec) uint64 {
	h := uint64(1469598103934665603)
	for _, x := range [3]float64{v.X, v.Y, v.Z} {
		if x == 0 {
			x = 0
		}
		h ^= math.Float64bits(x)
		h *= 1099511628211
	}
	return h
}

func appendWork[T any](values []T, additions []T, budget *proofbound.WorkBudget) ([]T, error) {
	if budget == nil || budget.ReserveBytesFn == nil {
		return append(values, additions...), nil
	}
	if len(additions) > int(^uint(0)>>1)-len(values) {
		return nil, errReplayBoundedSliceSizeOverflow
	}
	if _, err := budget.StepN(uint64(len(additions))); err != nil {
		return nil, err
	}
	need := len(values) + len(additions)
	if need > cap(values) {
		if _, err := budget.StepN(64); err != nil {
			return nil, err
		}
		capacity := max(1, cap(values))
		for capacity < need {
			if capacity > int(^uint(0)>>1)/2 {
				return nil, errReplayBoundedSliceCapacityOverflow
			}
			capacity *= 2
		}
		if err := budget.Reserve(uint64(capacity)); err != nil {
			return nil, err
		}
		if err := budget.ReserveBytesMul(uint64(capacity), uint64(unsafe.Sizeof(*new(T)))); err != nil {
			return nil, err
		}
		if _, err := budget.StepN(uint64(len(values))); err != nil {
			return nil, err
		}
		next := make([]T, len(values), capacity)
		copy(next, values)
		values = next
	}
	return append(values, additions...), nil
}

type replayExactWork struct{ work *proofbound.WorkBudget }

func (b replayExactWork) ReserveRecords(_ context.Context, n uint64) error { return b.work.Reserve(n) }
func (b replayExactWork) ReserveBytes(_ context.Context, n uint64) error {
	return b.work.ReserveBytes(n)
}
func (b replayExactWork) SpendWork(_ context.Context, n uint64) error {
	return b.work.SpendWork(n)
}
