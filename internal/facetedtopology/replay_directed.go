package facetedtopology

import (
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type replayDirectedEntry struct{ u, v, owner int }

func replayHalfedgeOwners(b *proofbound.WorkBudget, tris [][3]int) (halfedgeOwners, error) {
	n := 3 * len(tris)
	capacity := 1
	for capacity < 2*n {
		if capacity > int(^uint(0)>>1)/2 {
			return halfedgeOwners{}, fmt.Errorf("%w: replay halfedge capacity", decaderr.ErrUnsupported)
		}
		capacity *= 2
	}
	if err := b.Reserve(uint64(capacity + 2*n)); err != nil {
		return halfedgeOwners{}, err
	}
	if err := b.ReserveBytesMul(uint64(capacity), uint64(unsafe.Sizeof(replayDirectedEntry{}))); err != nil {
		return halfedgeOwners{}, err
	}
	for range 2 {
		if err := b.ReserveBytesMul(uint64(n), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return halfedgeOwners{}, err
		}
	}
	table := make([]replayDirectedEntry, capacity)
	entry := func(u, v int) (*replayDirectedEntry, error) {
		h := uint64(u)*0x9e3779b97f4a7c15 ^ uint64(v)*0xc2b2ae3d27d4eb4f
		h ^= h >> 33
		h *= 0xff51afd7ed558ccd
		h ^= h >> 33
		for i := range capacity {
			if err := b.Step(); err != nil {
				return nil, err
			}
			e := &table[(int(h)+i)&(capacity-1)]
			if e.owner == 0 || (e.u == u && e.v == v) {
				return e, nil
			}
		}
		return nil, fmt.Errorf("%w: replay halfedge table full", decaderr.ErrUnsupported)
	}
	distinct := 0
	for i, t := range tris {
		for k := range 3 {
			e, err := entry(t[k], t[(k+1)%3])
			if err != nil {
				return halfedgeOwners{}, err
			}
			if e.owner == 0 {
				distinct++
			}
			e.u, e.v, e.owner = t[k], t[(k+1)%3], i+1
		}
	}
	out := halfedgeOwners{twin: make([]int, n), distinct: distinct}
	if distinct != n {
		out.own = make([]int, n)
	}
	for i, t := range tris {
		for k := range 3 {
			o := 3*i + k
			e, err := entry(t[(k+1)%3], t[k])
			if err != nil {
				return halfedgeOwners{}, err
			}
			out.twin[o] = e.owner - 1
			if out.own != nil {
				e, err = entry(t[k], t[(k+1)%3])
				if err != nil {
					return halfedgeOwners{}, err
				}
				out.own[o] = e.owner - 1
			}
		}
	}
	return out, nil
}
