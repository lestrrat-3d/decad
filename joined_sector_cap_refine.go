package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func sectorConeCapBounds(nodes []sectorPathNode, caps [][3]int,
	record *pointConeTrimRecord) (float64, float64, error) {
	far := make([]r3.Vec, len(nodes))
	for i, node := range nodes {
		far[i] = node.p
	}
	capGeom := pointcone.CapGeometry(far, caps)
	upper, _, err := pointcone.CapBound(capGeom, record.upper.constants())
	if err != nil {
		return 0, 0, err
	}
	lower, _, err := pointcone.CapBound(capGeom, record.lower.constants())
	return upper, lower, err
}

// sectorRefineCap splits cells whose cone-map departure exceeds limit. All
// triangles sharing a marked edge split it, preserving the cap and root seam.
func sectorRefineCap(ctx context.Context, nodes []sectorPathNode,
	triangles [][3]int, edges []sectorCapEdge, record *pointConeTrimRecord,
	limit float64, maxCells int) ([]sectorPathNode, [][3]int, []sectorCapEdge, float64, error) {
	if !(limit > 0) || proofbound.IsNonFinite(limit) ||
		len(triangles) == 0 || len(triangles) > maxCells {
		return nil, nil, nil, 0, fmt.Errorf("%w: sector cap exceeds its refinement budget", ErrUnsupported)
	}
	marked := make(map[pointcone.Edge]struct{})
	for _, tri := range triangles {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, 0, err
		}
		far := [3]r3.Vec{nodes[tri[0]].p, nodes[tri[1]].p, nodes[tri[2]].p}
		geom := pointcone.CellGeometry(far)
		upper, _, _, err := pointcone.TriangleBound(geom, record.upper.constants())
		if err != nil {
			return nil, nil, nil, 0, err
		}
		lower, _, _, err := pointcone.TriangleBound(geom, record.lower.constants())
		if err != nil {
			return nil, nil, nil, 0, err
		}
		if max(upper, lower) <= limit {
			continue
		}
		marked[pointcone.OrderedEdge(tri[0], tri[1])] = struct{}{}
		marked[pointcone.OrderedEdge(tri[1], tri[2])] = struct{}{}
		marked[pointcone.OrderedEdge(tri[2], tri[0])] = struct{}{}
	}
	if len(marked) == 0 {
		return nodes, triangles, edges, 0, nil
	}
	boundary := make(map[pointcone.Edge]int, len(edges))
	for _, edge := range edges {
		boundary[pointcone.OrderedEdge(edge.u, edge.v)] = edge.group
	}
	midpoints := make(map[pointcone.Edge]int, len(marked))
	levelRound := 0.0
	midpoint := func(u, v int) int {
		key := pointcone.OrderedEdge(u, v)
		if _, needed := marked[key]; !needed {
			return -1
		}
		if id, ok := midpoints[key]; ok {
			return id
		}
		p, q := nodes[u].p, nodes[v].p
		held := p.Scale(0.5).Add(q.Scale(0.5))
		levelRound = max(levelRound, proofbound.ProductUpper(
			proofbound.AbsSumUpper(p.Len(), q.Len()),
			8*math.Nextafter(1, 2)*math.Ldexp(1, -53)))
		group, onBoundary := boundary[key]
		root := onBoundary && group == -1
		angle := 0.0
		if root {
			angle = nodes[u].angle*0.5 + nodes[v].angle*0.5
		}
		id := len(nodes)
		nodes = append(nodes, sectorPathNode{p: held, crossing: -1,
			rootBoundary: root, angle: angle})
		midpoints[key] = id
		return id
	}
	next := make([][3]int, 0, min(maxCells, 4*len(triangles)))
	for _, tri := range triangles {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, 0, err
		}
		a, b, c := tri[0], tri[1], tri[2]
		ab, bc, ca := midpoint(a, b), midpoint(b, c), midpoint(c, a)
		mask := 0
		if ab >= 0 {
			mask |= 1
		}
		if bc >= 0 {
			mask |= 2
		}
		if ca >= 0 {
			mask |= 4
		}
		switch mask {
		case 0:
			next = append(next, tri)
		case 1:
			next = append(next, [3]int{a, ab, c}, [3]int{ab, b, c})
		case 2:
			next = append(next, [3]int{a, b, bc}, [3]int{a, bc, c})
		case 4:
			next = append(next, [3]int{a, b, ca}, [3]int{ca, b, c})
		case 3:
			next = append(next, [3]int{a, ab, c}, [3]int{ab, b, bc}, [3]int{ab, bc, c})
		case 5:
			next = append(next, [3]int{a, ab, ca}, [3]int{ab, b, c}, [3]int{ab, c, ca})
		case 6:
			next = append(next, [3]int{a, b, ca}, [3]int{b, bc, ca}, [3]int{bc, c, ca})
		case 7:
			next = append(next, [3]int{a, ab, ca}, [3]int{ab, b, bc},
				[3]int{ca, bc, c}, [3]int{ab, bc, ca})
		}
		if len(next) > maxCells {
			return nil, nil, nil, 0, fmt.Errorf("%w: sector cap exceeds the audit mesh limit", ErrUnsupported)
		}
	}
	newEdges := make([]sectorCapEdge, 0, 2*len(edges))
	for _, edge := range edges {
		mid, split := midpoints[pointcone.OrderedEdge(edge.u, edge.v)]
		if !split {
			newEdges = append(newEdges, edge)
			continue
		}
		newEdges = append(newEdges,
			sectorCapEdge{u: edge.u, v: mid, group: edge.group},
			sectorCapEdge{u: mid, v: edge.v, group: edge.group})
	}
	return nodes, next, newEdges, levelRound, nil
}
