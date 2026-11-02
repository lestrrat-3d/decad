package facetproof

import (
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type replayEdgeEntry struct {
	key   [2]int
	owner int
}
type replayEdgeTable struct {
	slots  []replayEdgeEntry
	budget *proofbound.WorkBudget
}

func newReplayEdgeTable(n int, b *proofbound.WorkBudget) (*replayEdgeTable, error) {
	capn := 1
	for capn < 2*n {
		if capn > int(^uint(0)>>1)/2 {
			return nil, fmt.Errorf("%w: replay edge table size", decaderr.ErrUnsupported)
		}
		capn *= 2
	}
	if err := b.Reserve(uint64(capn)); err != nil {
		return nil, err
	}
	if err := b.ReserveBytesMul(uint64(capn), uint64(unsafe.Sizeof(replayEdgeEntry{}))); err != nil {
		return nil, err
	}
	if err := b.ReserveBytes(128); err != nil {
		return nil, err
	}
	return &replayEdgeTable{slots: make([]replayEdgeEntry, capn), budget: b}, nil
}
func (t *replayEdgeTable) entry(k [2]int) (*replayEdgeEntry, error) {
	h := uint64(k[0])*0x9e3779b97f4a7c15 ^ uint64(k[1])*0xc2b2ae3d27d4eb4f
	h ^= h >> 33
	h *= 0xff51afd7ed558ccd
	h ^= h >> 33
	for i := range t.slots {
		if err := t.budget.Step(); err != nil {
			return nil, err
		}
		e := &t.slots[(int(h)+i)&(len(t.slots)-1)]
		if e.owner == 0 || e.key == k {
			return e, nil
		}
	}
	return nil, fmt.Errorf("%w: replay edge table full", decaderr.ErrUnsupported)
}
