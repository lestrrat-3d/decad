package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

const (
	bevelCombineCapOnePassLimit = 10_000
	bevelCombineCapSourceLimit  = 12_000
	bevelCombineCapFinalLimit   = 120_000
	bevelCombineCapBound        = 0.08
)

// bevelRefineCap splits every cell whose upper or lower cone-map departure
// exceeds limit. Neighboring cells split on the same edge, so the original
// planar cap remains a conforming triangulation. The caller recomputes the
// final cap-map bound from every resulting triangle.
func bevelRefineCap(ctx context.Context, nodes []sectorPathNode,
	triangles [][3]int, edges []sectorCapEdge, record *pointConeTrimRecord,
	limit float64, maxCells int) ([]sectorPathNode, [][3]int, []sectorCapEdge, float64, error) {
	if len(triangles) == 0 || len(triangles) > maxCells {
		return nil, nil, nil, 0, fmt.Errorf("%w: straight pattern cap has too many source cells", ErrUnsupported)
	}
	marked := make(map[pointConeEdge]struct{})
	for _, tri := range triangles {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, 0, err
		}
		far := [3]r3.Vec{nodes[tri[0]].p, nodes[tri[1]].p, nodes[tri[2]].p}
		upper, _, _, err := pointConeTriangleBound(far, record.upper)
		if err != nil {
			return nil, nil, nil, 0, err
		}
		lower, _, _, err := pointConeTriangleBound(far, record.lower)
		if err != nil {
			return nil, nil, nil, 0, err
		}
		if max(upper, lower) <= limit {
			continue
		}
		marked[orderedPointConeEdge(tri[0], tri[1])] = struct{}{}
		marked[orderedPointConeEdge(tri[1], tri[2])] = struct{}{}
		marked[orderedPointConeEdge(tri[2], tri[0])] = struct{}{}
	}

	boundary := make(map[pointConeEdge]int, len(edges))
	for _, edge := range edges {
		boundary[orderedPointConeEdge(edge.u, edge.v)] = edge.group
	}
	midpoints := make(map[pointConeEdge]int, len(marked))
	levelRound := 0.0
	midpoint := func(u, v int) int {
		key := orderedPointConeEdge(u, v)
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

	next := make([][3]int, 0, 4*len(triangles))
	for _, tri := range triangles {
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
	}
	newEdges := make([]sectorCapEdge, 0, 2*len(edges))
	for _, edge := range edges {
		mid, split := midpoints[orderedPointConeEdge(edge.u, edge.v)]
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

// bevelUpperFarSliverBound encloses the difference between the smooth upper
// cone map and the source's clipped map, which stays on the far plane where
// the cone crossing lies beyond it. A radial supporting plane bounds g=x+kρ
// from below on every convex cap cell, including its interior.
func bevelUpperFarSliverBound(ctx context.Context, nodes []sectorPathNode,
	caps [][3]int, cone *pointConeLimit) (float64, error) {
	if cone.apex.Sign() <= 0 || cone.slope.Sign() < 0 {
		return 0, fmt.Errorf("%w: upper cap has no positive cone branch", ErrUnsupported)
	}
	bound := 0.0
	for _, tri := range caps {
		if err := ctx.Err(); err != nil {
			return 0, err
		}
		var y, z *big.Rat
		y, z = new(big.Rat), new(big.Rat)
		for _, index := range tri {
			y.Add(y, proofarith.FloatRat(nodes[index].p.Y))
			z.Add(z, proofarith.FloatRat(nodes[index].p.Z))
		}
		norm2 := new(big.Rat).Add(new(big.Rat).Mul(y, y), new(big.Rat).Mul(z, z))
		norm, ok := proofbound.IntervalSqrt(proofbound.PointInterval(norm2))
		if !ok || norm.Hi.Sign() <= 0 {
			return 0, fmt.Errorf("%w: upper cap has no radial supporting plane", ErrUnsupported)
		}
		var minG *big.Rat
		reach := 0.0
		for _, index := range tri {
			p := nodes[index].p
			py, pz := proofarith.FloatRat(p.Y), proofarith.FloatRat(p.Z)
			dot := new(big.Rat).Add(new(big.Rat).Mul(y, py), new(big.Rat).Mul(z, pz))
			if dot.Sign() <= 0 {
				return 0, fmt.Errorf("%w: upper cap crosses its radial support", ErrUnsupported)
			}
			g := new(big.Rat).Add(proofarith.FloatRat(p.X), new(big.Rat).Mul(cone.slope,
				new(big.Rat).Quo(dot, norm.Hi)))
			if minG == nil || g.Cmp(minG) < 0 {
				minG = g
			}
			reach = max(reach, proofbound.DvLenUpper(proofbound.HeldDelta(p, r3.Vec{})))
		}
		if minG.Sign() <= 0 {
			return 0, fmt.Errorf("%w: upper cap has no positive ray denominator", ErrUnsupported)
		}
		excess := new(big.Rat).Sub(new(big.Rat).Quo(cone.apex, minG), big.NewRat(1, 1))
		if excess.Sign() > 0 {
			bound = max(bound, proofbound.ProductUpper(reach, proofbound.RatFloatUp(excess)))
		}
	}
	return bound, nil
}
