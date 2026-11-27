package facetedtopology

import (
	"math"
	"slices"
)

// mapDirectedFacets is directedFacets as it stood with Go maps, before the
// flat edge table. The differential tests compare the table against it, and
// referenceBuild uses it so that build is checked against the old code.
type mapDirectedFacets struct {
	compact map[uint64]int
	wide    map[[2]int]int
}

func newMapDirectedFacets(vertices, halfedges int) mapDirectedFacets {
	if uint64(vertices) <= uint64(math.MaxUint32)+1 {
		return mapDirectedFacets{compact: make(map[uint64]int, halfedges)}
	}
	return mapDirectedFacets{wide: make(map[[2]int]int, halfedges)}
}

func (d mapDirectedFacets) set(u, v, facet int) {
	if d.compact != nil {
		d.compact[uint64(u)<<32|uint64(v)] = facet
		return
	}
	d.wide[[2]int{u, v}] = facet
}

func (d mapDirectedFacets) get(u, v int) (int, bool) {
	if d.compact != nil {
		facet, ok := d.compact[uint64(u)<<32|uint64(v)]
		return facet, ok
	}
	facet, ok := d.wide[[2]int{u, v}]
	return facet, ok
}

func (d mapDirectedFacets) len() int {
	if d.compact != nil {
		return len(d.compact)
	}
	return len(d.wide)
}

// mapCollectFaces is collectFaces as it stood with a map and a sort.
func mapCollectFaces(facetFace []int) []int {
	seen := make(map[int]struct{}, len(facetFace))
	out := make([]int, 0, len(facetFace))
	for _, f := range facetFace {
		if _, ok := seen[f]; ok {
			continue
		}
		seen[f] = struct{}{}
		out = append(out, f)
	}
	slices.Sort(out)
	return out
}
