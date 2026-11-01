package topo

import "fmt"

// ReplayAllocSizer exposes the concrete allocation size of each topology node.
// Replay callers reserve that size before calling the allocator.
type ReplayAllocSizer interface {
	ReplayNodeSize(kind string) (uint64, bool)
}

func ReplayNodeBytes(alloc Alloc, kind string) (uint64, error) {
	sized, ok := alloc.(ReplayAllocSizer)
	if !ok {
		return 0, fmt.Errorf("replay allocator lacks node size capability")
	}
	n, ok := sized.ReplayNodeSize(kind)
	if !ok || n == 0 {
		return 0, fmt.Errorf("replay allocator lacks node size for %s", kind)
	}
	return n, nil
}
