package facetedtopology

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// refusalMesh joins tetras-1 tetrahedra, each facet its own face, with one
// more separate tetrahedron that is a single face and so has no boundary loop.
func refusalMesh(tetras int) topologyCase {
	c := topologyCase{name: "no-loop-refusal", planar: map[int]bool{}}
	base := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
	face := 0
	for k := range tetras {
		off := len(c.verts)
		for _, v := range base {
			c.verts = append(c.verts, v.Add(r3.NewVec(float64(3*k), 0, 0)))
		}
		for _, t := range tris {
			c.tris = append(c.tris, [3]int{t[0] + off, t[1] + off, t[2] + off})
			if k == tetras-1 {
				c.facetFace = append(c.facetFace, face)
				continue
			}
			c.facetFace = append(c.facetFace, face)
			face++
		}
		if k == tetras-1 {
			c.planar[face] = true
		}
	}
	for f := range face {
		c.planar[f] = true
	}
	return c
}

// TestBuildNoBoundaryLoopRefusalIsDeterministic checks that the work spent
// before the "no boundary loop" refusal does not depend on map order.
func TestBuildNoBoundaryLoopRefusalIsDeterministic(t *testing.T) {
	t.Parallel()
	c := refusalMesh(4) // 12 loop-bearing faces plus one without
	want := runTopology(t.Context(), build, c, -1)
	require.Contains(t, want.err, "has no boundary loop")
	require.Empty(t, want.panic)
	spread := map[int]struct{}{}
	for range 200 {
		got := runTopology(t.Context(), build, c, -1)
		require.Equal(t, want.err, got.err)
		spread[got.steps] = struct{}{}
	}
	require.Len(t, spread, 1, "budget steps varied across runs: %v", spread)
}
