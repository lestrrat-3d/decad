// Package patchchain holds the topology and exact plane checks for Body.Patch.
// Root adapters supply topology identities and curve records without importing
// the public package into this internal package.
package patchchain

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// EdgeRef retains the identity of one selected edge and its two endpoint uses.
type EdgeRef[E comparable, V comparable] struct {
	Edge       E
	Start, End V
}

// Partition checks degree over every selected use, then walks each cycle.
// It preserves input edge order within the recovered groups.
func Partition[E comparable, V comparable](edges []EdgeRef[E, V], render func(V) string) ([][]E, error) {
	degree := map[V]int{}
	for _, e := range edges {
		degree[e.Start]++
		degree[e.End]++
	}
	for v, n := range degree {
		if n != 2 {
			return nil, fmt.Errorf(`%w: Body.Patch selection is not exactly closed chains — a vertex at %s has degree %d over the selection, not 2 (docs/surface-design.md Table R row R5)`,
				decaderr.ErrDegenerate, render(v), n)
		}
	}

	adj := map[V][]E{}
	ends := map[E][2]V{}
	for _, e := range edges {
		ends[e.Edge] = [2]V{e.Start, e.End}
		if e.Start == e.End {
			continue
		}
		adj[e.Start] = append(adj[e.Start], e.Edge)
		adj[e.End] = append(adj[e.End], e.Edge)
	}

	visited := map[E]bool{}
	var chains [][]E
	for _, e0 := range edges {
		if visited[e0.Edge] {
			continue
		}
		if e0.Start == e0.End {
			visited[e0.Edge] = true
			chains = append(chains, []E{e0.Edge})
			continue
		}
		visited[e0.Edge] = true
		chain := []E{e0.Edge}
		start, current := e0.Start, e0.End
		for current != start {
			var next E
			found := false
			for _, cand := range adj[current] {
				if !visited[cand] {
					next = cand
					found = true
					break
				}
			}
			if !found {
				return nil, fmt.Errorf(`%w: Body.Patch selection does not partition into closed chains (docs/surface-design.md Table R row R5)`, decaderr.ErrDegenerate)
			}
			visited[next] = true
			chain = append(chain, next)
			pair := ends[next]
			if pair[0] == current {
				current = pair[1]
			} else {
				current = pair[0]
			}
		}
		chains = append(chains, chain)
	}
	return chains, nil
}
