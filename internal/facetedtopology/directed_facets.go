package facetedtopology

import "math"

// directedFacets indexes an audited mesh's directed halfedges. Vertex indices
// must be in [0, vertices). Packing two 32-bit indices halves the key storage
// on 64-bit platforms; larger meshes keep the full-width pair representation.
type directedFacets struct {
	compact map[uint64]int
	wide    map[[2]int]int
}

// newDirectedFacets sizes the index for halfedges entries, so filling it
// with every halfedge of a mesh never grows the map. The size hint changes
// no answer.
func newDirectedFacets(vertices, halfedges int) directedFacets {
	if uint64(vertices) <= uint64(math.MaxUint32)+1 {
		return directedFacets{compact: make(map[uint64]int, halfedges)}
	}
	return directedFacets{wide: make(map[[2]int]int, halfedges)}
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

// halfedgeOwners answers, for each halfedge ordinal o = 3i+k (facet i's
// directed edge tris[i][k] → tris[i][(k+1)%3]), what the directedFacets
// filled with every halfedge in ordinal order answers: owner(o) is the facet
// get returns for that directed edge, twin[o] the facet it returns for the
// reverse edge or -1 when get finds none, and distinct is len. Both answers
// depend only on the directed edge, so every ordinal of one edge carries the
// same pair.
//
// own is nil when no directed edge repeats (distinct is every halfedge):
// each edge was then set once, by its only halfedge's own facet o/3.
type halfedgeOwners struct {
	own      []int
	twin     []int
	distinct int
}

// owner is the facet directedFacets.get returns for ordinal o's edge.
func (h halfedgeOwners) owner(o int) int {
	if h.own == nil {
		return o / 3
	}
	return h.own[o]
}

// newHalfedgeOwners reads every halfedge's answers from d, which holds every
// halfedge of tris set in ordinal order. Each later query is then a slice
// read rather than a map lookup.
func newHalfedgeOwners(d directedFacets, tris [][3]int) halfedgeOwners {
	n := 3 * len(tris)
	h := halfedgeOwners{twin: make([]int, n), distinct: d.len()}
	if h.distinct != n {
		h.own = make([]int, n)
	}
	for i, t := range tris {
		for k := range 3 {
			o := 3*i + k
			twin, ok := d.get(t[(k+1)%3], t[k])
			if !ok {
				twin = -1
			}
			h.twin[o] = twin
			if h.own != nil {
				h.own[o], _ = d.get(t[k], t[(k+1)%3])
			}
		}
	}
	return h
}
