package meshbool

import "sync"

const facetOwnerPoolMaxTriangles = 8192

// facetOwnerScratch belongs to one adjacency call until release. Only the
// integer owner map is cached; returned adjacency storage never enters the pool.
type facetOwnerScratch struct {
	owners              map[[2]int]int
	maxRequestedEntries int
}

var facetOwnerPool sync.Pool

func acquireFacetOwners(pool *sync.Pool, facets int) *facetOwnerScratch {
	// Check the triangle bound before multiplying, including for oversized input.
	if facets <= 0 || facets > facetOwnerPoolMaxTriangles {
		return nil
	}
	entries := 3 * facets
	var scratch *facetOwnerScratch
	if item := pool.Get(); item != nil {
		scratch = item.(*facetOwnerScratch)
	}
	if scratch == nil || scratch.maxRequestedEntries > 3*facetOwnerPoolMaxTriangles {
		return &facetOwnerScratch{
			owners:              make(map[[2]int]int, entries),
			maxRequestedEntries: entries,
		}
	}
	if entries > scratch.maxRequestedEntries {
		scratch.maxRequestedEntries = entries
	}
	return scratch
}

func releaseFacetOwners(pool *sync.Pool, scratch *facetOwnerScratch) {
	// This also runs on cancellation and panic unwinding. Reset time is part
	// of the call, and the entry budget survives clear for retention decisions.
	clear(scratch.owners)
	if scratch.maxRequestedEntries <= 3*facetOwnerPoolMaxTriangles {
		pool.Put(scratch)
	}
}
