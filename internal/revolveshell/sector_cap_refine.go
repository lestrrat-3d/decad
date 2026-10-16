package revolveshell

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func SectorConeCapBounds(nodes []pointcone.SectorNode, caps [][3]int,
	record *facetedrecord.PointConeTrimRecord) (float64, float64, error) {
	far := make([]r3.Vec, len(nodes))
	for i, node := range nodes {
		far[i] = node.P
	}
	capGeom := pointcone.CapGeometry(far, caps)
	upper, _, err := pointcone.CapBound(capGeom, record.Upper.Constants())
	if err != nil {
		return 0, 0, err
	}
	lower, _, err := pointcone.CapBound(capGeom, record.Lower.Constants())
	return upper, lower, err
}

// SectorRefineCap splits cells whose cone-map departure exceeds limit. All
// triangles sharing a marked edge split it, preserving the cap and root seam.
func SectorRefineCap(ctx context.Context, nodes []pointcone.SectorNode,
	triangles [][3]int, edges []pointcone.SectorEdge, record *facetedrecord.PointConeTrimRecord,
	limit float64, maxCells int) ([]pointcone.SectorNode, [][3]int, []pointcone.SectorEdge, float64, error) {
	if !(limit > 0) || proofbound.IsNonFinite(limit) ||
		len(triangles) == 0 || len(triangles) > maxCells {
		return nil, nil, nil, 0, fmt.Errorf("%w: sector cap exceeds its refinement budget", decaderr.ErrUnsupported)
	}
	marked := make(map[pointcone.Edge]struct{})
	for _, tri := range triangles {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, 0, err
		}
		far := [3]r3.Vec{nodes[tri[0]].P, nodes[tri[1]].P, nodes[tri[2]].P}
		geom := pointcone.CellGeometry(far)
		upper, _, _, err := pointcone.TriangleBound(geom, record.Upper.Constants())
		if err != nil {
			return nil, nil, nil, 0, err
		}
		lower, _, _, err := pointcone.TriangleBound(geom, record.Lower.Constants())
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
		boundary[pointcone.OrderedEdge(edge.U, edge.V)] = edge.Group
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
		p, q := nodes[u].P, nodes[v].P
		held := p.Scale(0.5).Add(q.Scale(0.5))
		levelRound = max(levelRound, proofbound.ProductUpper(
			proofbound.AbsSumUpper(p.Len(), q.Len()),
			8*math.Nextafter(1, 2)*math.Ldexp(1, -53)))
		group, onBoundary := boundary[key]
		root := onBoundary && group == -1
		angle := 0.0
		if root {
			angle = nodes[u].Angle*0.5 + nodes[v].Angle*0.5
		}
		id := len(nodes)
		nodes = append(nodes, pointcone.SectorNode{P: held, Crossing: -1,
			RootBoundary: root, Angle: angle})
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
			return nil, nil, nil, 0, fmt.Errorf("%w: sector cap exceeds the audit mesh limit", decaderr.ErrUnsupported)
		}
	}
	newEdges := make([]pointcone.SectorEdge, 0, 2*len(edges))
	for _, edge := range edges {
		mid, split := midpoints[pointcone.OrderedEdge(edge.U, edge.V)]
		if !split {
			newEdges = append(newEdges, edge)
			continue
		}
		newEdges = append(newEdges,
			pointcone.SectorEdge{U: edge.U, V: mid, Group: edge.Group},
			pointcone.SectorEdge{U: mid, V: edge.V, Group: edge.Group})
	}
	return nodes, next, newEdges, levelRound, nil
}
