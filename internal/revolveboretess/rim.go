// Package revolveboretess assembles verification meshes for axial Revolve bores.
package revolveboretess

import (
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// capRim returns the one directed boundary of a cap's triangle patch.
func capRim(mesh *topo.Mesh, face *topo.Face) ([]int, error) {
	type edge struct{ a, b int }
	counts := make(map[edge]int)
	uses := make(map[edge]edge)
	for i, tri := range mesh.Triangles {
		if mesh.Source[i] != face {
			continue
		}
		for j, a := range tri {
			b := tri[(j+1)%3]
			key := edge{min(a, b), max(a, b)}
			counts[key]++
			uses[key] = edge{a, b}
		}
	}
	next := map[int]int{}
	for key, count := range counts {
		if count == 2 {
			continue
		}
		if count != 1 {
			return nil, fmt.Errorf("%w: cap mesh has a nonmanifold edge", decaderr.ErrUnsupported)
		}
		e := uses[key]
		if _, exists := next[e.a]; exists {
			return nil, fmt.Errorf("%w: cap mesh rim branches", decaderr.ErrUnsupported)
		}
		next[e.a] = e.b
	}
	if len(next) < 3 {
		return nil, fmt.Errorf("%w: cap mesh has no rim", decaderr.ErrUnsupported)
	}
	keys := make([]int, 0, len(next))
	for v := range next {
		keys = append(keys, v)
	}
	slices.Sort(keys)
	start := keys[0]
	rim := make([]int, 0, len(next))
	for v := start; ; {
		rim = append(rim, v)
		end, ok := next[v]
		if !ok || len(rim) > len(next) {
			return nil, fmt.Errorf("%w: cap mesh rim does not close", decaderr.ErrUnsupported)
		}
		if end == start {
			break
		}
		v = end
	}
	if len(rim) != len(next) {
		return nil, fmt.Errorf("%w: cap mesh has more than one rim", decaderr.ErrUnsupported)
	}
	return rim, nil
}
