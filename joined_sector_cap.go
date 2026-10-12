package decad

import (
	"context"
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

type sectorPathNode struct {
	p            r3.Vec
	groupToNext  int
	crossing     int // -1 except for the two certified root contacts
	rootBoundary bool
	angle        float64 // exact mesh station shared with the blank ring
}

type sectorCapEdge struct {
	u, v, group int // group -1 is the shared root arc
}

func sectorPolygonArea(points []triangulation.Point2) float64 {
	a := 0.0
	for i := range points {
		j := (i + 1) % len(points)
		a += points[i].U*points[j].V - points[j].U*points[i].V
	}
	return a * 0.5
}

func sectorCapTriangulate(ctx context.Context, nodes []sectorPathNode,
	frame r3.Frame) ([][3]int, error) {
	local := make([]triangulation.Point2, len(nodes))
	loop := make([]int, len(nodes))
	for i, node := range nodes {
		p := frame.ToLocal(node.p)
		if !proofbound.FiniteVec(p) {
			return nil, fmt.Errorf("%w: the sector cap has a non-finite station", ErrUnsupported)
		}
		local[i], loop[i] = triangulation.Point2{U: p.X, V: p.Y}, i
	}
	if sectorPolygonArea(local) < 0 {
		for i, j := 0, len(loop)-1; i < j; i, j = i+1, j-1 {
			loop[i], loop[j] = loop[j], loop[i]
		}
	}
	return triangulation.Triangulate(ctx, local, [][]int{loop})
}

// sectorSubdivideCap reduces the cone-map interpolation bound. Boundary edge
// labels are carried through every split so the exposed side and root seam
// are reconstructed from the same triangulation, without a guessed contact.
func sectorSubdivideCap(ctx context.Context, nodes []sectorPathNode,
	triangles [][3]int, rounds int) ([]sectorPathNode, [][3]int, []sectorCapEdge, float64, error) {
	edges := make([]sectorCapEdge, len(nodes))
	for i := range nodes {
		edges[i] = sectorCapEdge{u: i, v: (i + 1) % len(nodes), group: nodes[i].groupToNext}
	}
	totalRound := 0.0
	for range rounds {
		midpoints := map[pointConeEdge]int{}
		levelRound := 0.0
		boundary := map[pointConeEdge]int{}
		for _, edge := range edges {
			boundary[orderedPointConeEdge(edge.u, edge.v)] = edge.group
		}
		midpoint := func(u, v int) int {
			key := orderedPointConeEdge(u, v)
			if id, ok := midpoints[key]; ok {
				return id
			}
			p, q := nodes[u].p, nodes[v].p
			held := p.Scale(0.5).Add(q.Scale(0.5))
			// The same held interpolation is used for the two cap maps; its
			// arithmetic departure is charged in the result bound.
			levelRound = max(levelRound, proofbound.ProductUpper(
				proofbound.AbsSumUpper(p.Len(), q.Len()), 8*math.Nextafter(1, 2)*math.Ldexp(1, -53)))
			group, onBoundary := boundary[key]
			id := len(nodes)
			root := onBoundary && group == -1
			angle := 0.0
			if root {
				angle = nodes[u].angle*0.5 + nodes[v].angle*0.5
			}
			nodes = append(nodes, sectorPathNode{p: held, crossing: -1,
				rootBoundary: root, angle: angle})
			midpoints[key] = id
			return id
		}
		next := make([][3]int, 0, 4*len(triangles))
		for _, tri := range triangles {
			if err := ctx.Err(); err != nil {
				return nil, nil, nil, 0, err
			}
			a, b, c := tri[0], tri[1], tri[2]
			ab, bc, ca := midpoint(a, b), midpoint(b, c), midpoint(c, a)
			next = append(next, [3]int{a, ab, ca}, [3]int{ab, b, bc},
				[3]int{ca, bc, c}, [3]int{ab, bc, ca})
		}
		newEdges := make([]sectorCapEdge, 0, len(edges)*2)
		for _, edge := range edges {
			m, ok := midpoints[orderedPointConeEdge(edge.u, edge.v)]
			if !ok {
				return nil, nil, nil, 0, fmt.Errorf("%w: cap boundary was not triangulated", ErrUnsupported)
			}
			newEdges = append(newEdges,
				sectorCapEdge{u: edge.u, v: m, group: edge.group},
				sectorCapEdge{u: m, v: edge.v, group: edge.group})
		}
		triangles, edges = next, newEdges
		totalRound = proofbound.AbsSumUpper(totalRound, levelRound)
	}
	return nodes, triangles, edges, totalRound, nil
}

func sectorMergedAngles(base []float64, nodes []sectorPathNode) ([]float64, []int, error) {
	angles := append([]float64(nil), base...)
	rootNode := make([]int, len(nodes))
	for i, node := range nodes {
		rootNode[i] = -1
		if !node.rootBoundary {
			continue
		}
		angle := node.angle
		if !proofbound.FiniteVec(r3.Vec{X: angle}) {
			return nil, nil, fmt.Errorf("%w: root angle is non-finite", ErrUnsupported)
		}
		angles = append(angles, angle)
	}
	sort.Float64s(angles)
	unique := angles[:0]
	for _, angle := range angles {
		if len(unique) == 0 || angle != unique[len(unique)-1] {
			unique = append(unique, angle)
		}
	}
	for i, node := range nodes {
		if node.rootBoundary {
			angle := node.angle
			j := sort.SearchFloat64s(unique, angle)
			if j == len(unique) || unique[j] != angle {
				return nil, nil, fmt.Errorf("%w: root node has no angular station", ErrUnsupported)
			}
			rootNode[i] = j
		}
	}
	return unique, rootNode, nil
}

// sectorUpperCapNode charges any replacement of the exact smooth cone map by
// the source far plane at one cap station. Cell interiors are charged by the
// caller's upper far-plane sliver bound when that source class is admitted.
func sectorUpperCapNode(far r3.Vec, sample pointConeSample,
	upperFarSliverCharged bool) (r3.Vec, float64, error) {
	if !upperFarSliverCharged && sample.q+sample.qError < 1 {
		return r3.Vec{}, 0, fmt.Errorf("%w: sector cap reaches its far source plane", ErrUnsupported)
	}
	smooth := far.Scale(sample.alpha)
	if !proofbound.FiniteVec(smooth) {
		return r3.Vec{}, 0, fmt.Errorf("%w: upper cap has no finite crossing", ErrUnsupported)
	}
	if sample.q-sample.qError > 1 {
		return smooth, sample.pointError, nil
	}
	return far, proofbound.AbsSumUpper(
		proofbound.DvLenUpper(proofbound.HeldDelta(far, smooth)), sample.pointError), nil
}
