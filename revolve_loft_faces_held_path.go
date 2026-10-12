package decad

import (
	"context"
	"math"
	"sort"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// revolveLoftFacesHeldPath is one complete connected component of a
// cross-operand source-face switch. It carries actual output vertices and
// published bounds; the true-source correspondence remains a separate gate.
type revolveLoftFacesHeldPath struct {
	pair     revolveLoftFacesHeldPair
	vertices []int
	closed   bool
	box      [3]loftFaceInterval
	minBeta  float64
}

// Every returned component is a simple path or loop. A branch, repeated
// edge, invalid vertex or incomplete traversal refuses before a geometric
// certificate can interpret the switched curve.
func revolveLoftFacesHeldPaths(ctx context.Context, result facetedPayload,
	pairs []revolveLoftFacesHeldPair) ([]revolveLoftFacesHeldPath, bool, error) {
	if ctx == nil || len(result.verts) == 0 ||
		len(result.vertexBound) != len(result.verts) || len(pairs) == 0 {
		return nil, false, nil
	}
	var paths []revolveLoftFacesHeldPath
	for _, pair := range pairs {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		if len(pair.edges) == 0 || len(pair.edges) > 1<<18 {
			return nil, false, nil
		}
		adjacent := make(map[int][]int, len(pair.edges)+1)
		usedEdges := make(map[[2]int]bool, len(pair.edges))
		for _, edge := range pair.edges {
			a, b := edge[0], edge[1]
			if a < 0 || b < 0 || a >= len(result.verts) || b >= len(result.verts) || a == b {
				return nil, false, nil
			}
			if a > b {
				a, b = b, a
			}
			key := [2]int{a, b}
			if usedEdges[key] {
				return nil, false, nil
			}
			usedEdges[key] = true
			adjacent[a] = append(adjacent[a], b)
			adjacent[b] = append(adjacent[b], a)
		}
		starts := make([]int, 0, len(adjacent))
		for vertex, neighbors := range adjacent {
			if len(neighbors) < 1 || len(neighbors) > 2 {
				return nil, false, nil
			}
			starts = append(starts, vertex)
		}
		sort.Ints(starts)
		visited := make(map[int]bool, len(adjacent))
		for _, seed := range starts {
			if visited[seed] {
				continue
			}
			// Select a degree-one endpoint if this component is open. For a
			// loop, the least index gives a stable start without geometry hints.
			component := make([]int, 0)
			queue := []int{seed}
			seen := map[int]bool{seed: true}
			for len(queue) != 0 {
				vertex := queue[len(queue)-1]
				queue = queue[:len(queue)-1]
				component = append(component, vertex)
				for _, next := range adjacent[vertex] {
					if !seen[next] {
						seen[next] = true
						queue = append(queue, next)
					}
				}
			}
			var endpoints []int
			for _, vertex := range component {
				if len(adjacent[vertex]) == 1 {
					endpoints = append(endpoints, vertex)
				}
			}
			if len(endpoints) != 0 && len(endpoints) != 2 {
				return nil, false, nil
			}
			start := seed
			if len(endpoints) == 2 {
				start = min(endpoints[0], endpoints[1])
			}
			path := revolveLoftFacesHeldPath{pair: pair,
				closed: len(endpoints) == 0, minBeta: math.Inf(1)}
			previous, current := -1, start
			for {
				if len(path.vertices)%128 == 0 {
					if err := ctx.Err(); err != nil {
						return nil, false, err
					}
				}
				if visited[current] {
					return nil, false, nil
				}
				visited[current] = true
				path.vertices = append(path.vertices, current)
				point, beta := result.verts[current], result.vertexBound[current]
				if !proofbound.FiniteVec(point) || proofbound.IsNonFinite(beta) || beta < 0 {
					return nil, false, nil
				}
				path.minBeta = min(path.minBeta, beta)
				for axis, coordinate := range [...]float64{point.X, point.Y, point.Z} {
					value := lfF(coordinate)
					if path.box[axis].lo == nil || value.Cmp(path.box[axis].lo) < 0 {
						path.box[axis].lo = value
					}
					if path.box[axis].hi == nil || value.Cmp(path.box[axis].hi) > 0 {
						path.box[axis].hi = value
					}
				}
				next := -1
				for _, neighbor := range adjacent[current] {
					if neighbor != previous {
						next = neighbor
						break
					}
				}
				if next < 0 || path.closed && next == start {
					break
				}
				previous, current = current, next
			}
			if len(path.vertices) != len(component) ||
				!path.closed && len(adjacent[path.vertices[len(path.vertices)-1]]) != 1 {
				return nil, false, nil
			}
			paths = append(paths, path)
		}
	}
	return paths, len(paths) != 0, nil
}
