package facetedtopology

import (
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type replayEdgeSlot[V any] struct {
	key      [2]int
	value    V
	occupied bool
}
type replayEdgeTable[V any] struct {
	slots        []replayEdgeSlot[V]
	limit, count int
	budget       *proofbound.WorkBudget
}

func newReplayEdgeTable[V any](b *proofbound.WorkBudget, n int) (*replayEdgeTable[V], error) {
	if n < 0 || n > int(^uint(0)>>1)/2 {
		return nil, fmt.Errorf("%w: replay edge capacity", decaderr.ErrUnsupported)
	}
	capacity := 1
	for capacity < 2*n {
		if capacity > int(^uint(0)>>1)/2 {
			return nil, fmt.Errorf("%w: replay edge capacity", decaderr.ErrUnsupported)
		}
		capacity *= 2
	}
	if err := b.Reserve(uint64(capacity)); err != nil {
		return nil, err
	}
	if err := b.ReserveBytesMul(uint64(capacity), uint64(unsafe.Sizeof(replayEdgeSlot[V]{}))); err != nil {
		return nil, err
	}
	if err := b.ReserveBytes(128); err != nil {
		return nil, err
	}
	return &replayEdgeTable[V]{slots: make([]replayEdgeSlot[V], capacity), limit: n, budget: b}, nil
}
func (t *replayEdgeTable[V]) slot(k [2]int) (*replayEdgeSlot[V], bool, error) {
	h := uint64(k[0])*0x9e3779b97f4a7c15 ^ uint64(k[1])*0xc2b2ae3d27d4eb4f
	h ^= h >> 33
	h *= 0xff51afd7ed558ccd
	h ^= h >> 33
	for i := range t.slots {
		if err := t.budget.Step(); err != nil {
			return nil, false, err
		}
		s := &t.slots[(int(h)+i)&(len(t.slots)-1)]
		if !s.occupied {
			return s, false, nil
		}
		if s.key == k {
			return s, true, nil
		}
	}
	return nil, false, fmt.Errorf("%w: replay edge table full", decaderr.ErrUnsupported)
}
func (t *replayEdgeTable[V]) get(k [2]int) (V, bool, error) {
	s, ok, err := t.slot(k)
	if err != nil {
		var zero V
		return zero, false, err
	}
	return s.value, ok, nil
}
func (t *replayEdgeTable[V]) set(k [2]int, v V) error {
	s, ok, err := t.slot(k)
	if err != nil {
		return err
	}
	if !ok {
		if t.count >= t.limit {
			return fmt.Errorf("%w: replay edge capacity", decaderr.ErrUnsupported)
		}
		t.count++
		s.key = k
		s.occupied = true
	}
	s.value = v
	return nil
}
