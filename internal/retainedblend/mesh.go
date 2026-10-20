package retainedblend

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// facetsOfFace lists the facets that faceOf maps to face index fi.
func facetsOfFace(faceOf []int, fi int) []int {
	var out []int
	for i, f := range faceOf {
		if f == fi {
			out = append(out, i)
		}
	}
	return out
}

// boundaryLoops chains the directed boundary edges of the facet set into
// closed loops of vertex indices. A facet set with outward winding walks its
// outer boundary counter-clockwise and its holes clockwise, seen from outside.
// A vertex that starts two boundary edges (a pinch) refuses.
func boundaryLoops(tris [][3]int, facets []int) ([][]int, error) {
	type edge struct{ a, b int }
	have := map[edge]struct{}{}
	for _, f := range facets {
		t := tris[f]
		for k := range 3 {
			have[edge{t[k], t[(k+1)%3]}] = struct{}{}
		}
	}
	next := map[int]int{}
	var starts []int
	for _, f := range facets {
		t := tris[f]
		for k := range 3 {
			a, b := t[k], t[(k+1)%3]
			if _, twin := have[edge{b, a}]; twin {
				continue
			}
			if _, dup := next[a]; dup {
				return nil, fmt.Errorf("%w: a face boundary touches itself at one vertex", decaderr.ErrUnsupported)
			}
			next[a] = b
			starts = append(starts, a)
		}
	}
	var loops [][]int
	seen := map[int]bool{}
	for _, s := range starts {
		if seen[s] {
			continue
		}
		var loop []int
		for v := s; !seen[v]; v = next[v] {
			seen[v] = true
			loop = append(loop, v)
			if _, ok := next[v]; !ok {
				return nil, fmt.Errorf("%w: a face boundary does not close", decaderr.ErrUnsupported)
			}
		}
		if next[loop[len(loop)-1]] != s {
			return nil, fmt.Errorf("%w: a face boundary does not close on itself", decaderr.ErrUnsupported)
		}
		loops = append(loops, loop)
	}
	return loops, nil
}
