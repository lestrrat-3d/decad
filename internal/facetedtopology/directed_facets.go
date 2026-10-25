package facetedtopology

import "math"

// directedFacets indexes an audited mesh's directed halfedges. Vertex indices
// must be in [0, vertices). Packing two 32-bit indices halves the key storage
// on 64-bit platforms; larger meshes keep the full-width pair representation.
type directedFacets struct {
	compact map[uint64]int
	wide    map[[2]int]int
}

func newDirectedFacets(vertices int) directedFacets {
	if uint64(vertices) <= uint64(math.MaxUint32)+1 {
		return directedFacets{compact: make(map[uint64]int)}
	}
	return directedFacets{wide: make(map[[2]int]int)}
}

func (d directedFacets) set(u, v, facet int) {
	if d.compact != nil {
		d.compact[uint64(u)<<32|uint64(v)] = facet
		return
	}
	d.wide[[2]int{u, v}] = facet
}

func (d directedFacets) get(u, v int) (int, bool) {
	if d.compact != nil {
		facet, ok := d.compact[uint64(u)<<32|uint64(v)]
		return facet, ok
	}
	facet, ok := d.wide[[2]int{u, v}]
	return facet, ok
}

func (d directedFacets) len() int {
	if d.compact != nil {
		return len(d.compact)
	}
	return len(d.wide)
}
