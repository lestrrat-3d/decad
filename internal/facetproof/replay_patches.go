package facetproof

import (
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func sourcePatchesReplay(b *proofbound.WorkBudget, n int, source []int, adj [][]int) ([]int, error) {
	if err := b.Reserve(uint64(2 * n)); err != nil {
		return nil, err
	}
	for range 2 {
		if err := b.ReserveBytesMul(uint64(n), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return nil, err
		}
	}
	patch := make([]int, n)
	queue := make([]int, n)
	for i := range patch {
		if err := b.Step(); err != nil {
			return nil, err
		}
		patch[i] = -1
	}
	next := 0
	for i := range n {
		if err := b.Step(); err != nil {
			return nil, err
		}
		if patch[i] != -1 {
			continue
		}
		id := next
		next++
		head, tail := 0, 1
		queue[0] = i
		patch[i] = id
		for head < tail {
			if err := b.Step(); err != nil {
				return nil, err
			}
			f := queue[head]
			head++
			for _, nb := range adj[f] {
				if patch[nb] != -1 || source[nb] != source[f] {
					continue
				}
				patch[nb] = id
				queue[tail] = nb
				tail++
			}
		}
	}
	return patch, nil
}
