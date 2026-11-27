package facetedtopology

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/edgetable"
)

// directedFacets indexes an audited mesh's directed halfedges. Vertex indices
// must be in [0, vertices). It answers exactly as a map from the packed pair
// uint64(u)<<32|uint64(v) does when vertices fit 32 bits, and as a map from
// the pair [2]int otherwise, including for indices outside that range, where
// the packed form lets distinct pairs share a key. A flat edge table holds
// the entries; it has no iteration, so nothing depends on its layout.
type directedFacets struct {
	table *edgetable.Table
	wide  bool
}

// newDirectedFacets sizes the index for halfedges entries, so filling it
// with every halfedge of a mesh never grows the table. The size hint changes
// no answer.
func newDirectedFacets(vertices, halfedges int) directedFacets {
	return directedFacets{
		table: edgetable.New(halfedges),
		wide:  uint64(vertices) > uint64(math.MaxUint32)+1,
	}
}

// key returns the table's edge for the directed pair (u, v). In the packed
// form it splits the 64-bit key into its two 32-bit halves, a bijection, so
// two pairs share an edge exactly when they share a packed key.
func (d directedFacets) key(u, v int) (int, int) {
	if d.wide {
		return u, v
	}
	k := uint64(u)<<32 | uint64(v)
	return int(k >> 32), int(k & math.MaxUint32)
}

func (d directedFacets) set(u, v, facet int) {
	a, b := d.key(u, v)
	d.table.Set(a, b, facet)
}

func (d directedFacets) get(u, v int) (int, bool) {
	a, b := d.key(u, v)
	return d.table.Get(a, b)
}

func (d directedFacets) len() int {
	return d.table.Len()
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
