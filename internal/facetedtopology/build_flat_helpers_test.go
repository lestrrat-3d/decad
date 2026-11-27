package facetedtopology

import (
	"math"
	"math/rand"

	"github.com/lestrrat-3d/r3"
)

// relabelFaces returns a copy of c whose face ids are mapped through id, so
// the planar flags follow their faces.
func (c topologyCase) relabelFaces(name string, id func(int) int) topologyCase {
	out := c.clone(name)
	out.planar = map[int]bool{}
	for f, planar := range c.planar {
		out.planar[id(f)] = planar
	}
	for i, f := range out.facetFace {
		out.facetFace[i] = id(f)
	}
	return out
}

// sparseFaceVariants relabel the faces of c to ids outside [0, len(facetFace)),
// which the dense face tables cannot hold: negative, one past the facets, and
// huge. Both the order and the gaps between ids change.
func sparseFaceVariants(c topologyCase) []topologyCase {
	n := len(c.facetFace)
	return []topologyCase{
		c.relabelFaces("faces-negative", func(f int) int { return -1 - f }),
		c.relabelFaces("faces-past-end", func(f int) int { return n + f }),
		c.relabelFaces("faces-huge", func(f int) int { return (f+1)<<40 - 3 }),
		c.relabelFaces("faces-reversed", func(f int) int { return 9 - f }),
		c.relabelFaces("faces-mixed", func(f int) int {
			if f%2 == 0 {
				return f
			}
			return math.MinInt + f
		}),
	}
}

// randomMeshCases are random soups over few vertices with a mix of face ids,
// including ids that are not dense, so self-edges, repeated directed edges,
// missing twins, ties between loops and lost facets all occur.
func randomMeshCases(seed int64, count int) []topologyCase {
	rng := rand.New(rand.NewSource(seed))
	var out []topologyCase
	for k := range count {
		nv := 4 + rng.Intn(8)
		c := topologyCase{name: "soup", planar: map[int]bool{}}
		for range nv {
			c.verts = append(c.verts, r3.NewVec(rng.Float64(), rng.Float64(), rng.Float64()))
		}
		ids := []int{0, 1, 2, 3, 5, 40, -2}
		for range 4 + rng.Intn(40) {
			c.tris = append(c.tris, [3]int{rng.Intn(nv), rng.Intn(nv), rng.Intn(nv)})
			c.facetFace = append(c.facetFace, ids[rng.Intn(len(ids))])
		}
		for _, id := range ids {
			c.planar[id] = rng.Intn(2) == 0
		}
		c.name = "soup-" + string(rune('a'+k%26)) + string(rune('0'+k/26%10))
		out = append(out, c)
	}
	return out
}
