package pointcone

import (
	"context"
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// SectorNode is one station of a sector cap boundary path.
type SectorNode struct {
	P            r3.Vec
	GroupToNext  int
	Crossing     int // -1 except for the two certified root contacts
	RootBoundary bool
	Angle        float64 // exact mesh station shared with the blank ring
}

// SectorEdge is one boundary edge of a sector cap between node indices U and V.
type SectorEdge struct {
	U, V, Group int // group -1 is the shared root arc
}

func polygonArea(points []triangulation.Point2) float64 {
	a := 0.0
	for i := range points {
		j := (i + 1) % len(points)
		a += points[i].U*points[j].V - points[j].U*points[i].V
	}
	return a * 0.5
}

// SectorTriangulate triangulates the sector cap polygon through frame.
func SectorTriangulate(ctx context.Context, nodes []SectorNode,
	frame r3.Frame) ([][3]int, error) {
	local := make([]triangulation.Point2, len(nodes))
	loop := make([]int, len(nodes))
	for i, node := range nodes {
		p := frame.ToLocal(node.P)
		if !proofbound.FiniteVec(p) {
			return nil, fmt.Errorf("%w: the sector cap has a non-finite station", decaderr.ErrUnsupported)
		}
		local[i], loop[i] = triangulation.Point2{U: p.X, V: p.Y}, i
	}
	if polygonArea(local) < 0 {
		for i, j := 0, len(loop)-1; i < j; i, j = i+1, j-1 {
			loop[i], loop[j] = loop[j], loop[i]
		}
	}
	return triangulation.Triangulate(ctx, local, [][]int{loop})
}

// SectorSubdivide reduces the cone-map interpolation bound. Boundary edge
// labels are carried through every split so the exposed side and root seam
// are reconstructed from the same triangulation, without a guessed contact.
func SectorSubdivide(ctx context.Context, nodes []SectorNode,
	triangles [][3]int, rounds int) ([]SectorNode, [][3]int, []SectorEdge, float64, error) {
	edges := make([]SectorEdge, len(nodes))
	for i := range nodes {
		edges[i] = SectorEdge{U: i, V: (i + 1) % len(nodes), Group: nodes[i].GroupToNext}
	}
	totalRound := 0.0
	for range rounds {
		midpoints := map[Edge]int{}
		levelRound := 0.0
		boundary := map[Edge]int{}
		for _, edge := range edges {
			boundary[OrderedEdge(edge.U, edge.V)] = edge.Group
		}
		midpoint := func(u, v int) int {
			key := OrderedEdge(u, v)
			if id, ok := midpoints[key]; ok {
				return id
			}
			p, q := nodes[u].P, nodes[v].P
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
				angle = nodes[u].Angle*0.5 + nodes[v].Angle*0.5
			}
			nodes = append(nodes, SectorNode{P: held, Crossing: -1,
				RootBoundary: root, Angle: angle})
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
		newEdges := make([]SectorEdge, 0, len(edges)*2)
		for _, edge := range edges {
			m, ok := midpoints[OrderedEdge(edge.U, edge.V)]
			if !ok {
				return nil, nil, nil, 0, fmt.Errorf("%w: cap boundary was not triangulated", decaderr.ErrUnsupported)
			}
			newEdges = append(newEdges,
				SectorEdge{U: edge.U, V: m, Group: edge.Group},
				SectorEdge{U: m, V: edge.V, Group: edge.Group})
		}
		triangles, edges = next, newEdges
		totalRound = proofbound.AbsSumUpper(totalRound, levelRound)
	}
	return nodes, triangles, edges, totalRound, nil
}

// SectorMergedAngles returns the sorted unique union of base and the root-node
// angles, and for every node the index of its root angle or -1.
func SectorMergedAngles(base []float64, nodes []SectorNode) ([]float64, []int, error) {
	angles := append([]float64(nil), base...)
	rootNode := make([]int, len(nodes))
	for i, node := range nodes {
		rootNode[i] = -1
		if !node.RootBoundary {
			continue
		}
		angle := node.Angle
		if !proofbound.FiniteVec(r3.Vec{X: angle}) {
			return nil, nil, fmt.Errorf("%w: root angle is non-finite", decaderr.ErrUnsupported)
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
		if node.RootBoundary {
			angle := node.Angle
			j := sort.SearchFloat64s(unique, angle)
			if j == len(unique) || unique[j] != angle {
				return nil, nil, fmt.Errorf("%w: root node has no angular station", decaderr.ErrUnsupported)
			}
			rootNode[i] = j
		}
	}
	return unique, rootNode, nil
}

// MatchingFaceSlack is the largest shortfall of a cap station's Q below 1 that
// a caller's matching-faces declaration absorbs: 64 binary64 steps at 1. A cap
// drawn in a float frame cannot touch a cone exactly; the frame, the involute
// samples and the cone's own stored numbers each round, and their sum stays
// within a few steps of 1.
const MatchingFaceSlack = 64 * 0x1p-52

// UpperCapNode charges any replacement of the exact smooth cone map by
// the source far plane at one cap station. Cell interiors are charged by the
// caller's upper far-plane sliver bound when that source class is admitted.
//
// matchingFaces is the caller's declaration that the cap is meant to meet the
// cone exactly. It does not admit a station: it only stops a station whose
// held Q falls short of 1 by no more than MatchingFaceSlack, beyond the
// sample's own error, from being refused. That station takes the far point
// with the same charge as one inside the error band. A larger shortfall
// disproves the declaration and still refuses.
func UpperCapNode(far r3.Vec, sample Sample,
	upperFarSliverCharged, matchingFaces bool) (r3.Vec, float64, error) {
	if !upperFarSliverCharged && sample.Q+sample.QError < 1 &&
		!(matchingFaces && 1-(sample.Q+sample.QError) <= MatchingFaceSlack) {
		shortfall := 1 - (sample.Q + sample.QError)
		if shortfall <= MatchingFaceSlack {
			return r3.Vec{}, 0, fmt.Errorf("%w: sector cap reaches its far source plane: "+
				"a cap station misses the cone by %.3g, within rounding; pass decad.WithMatchingFaces "+
				"to the Cut or Intersect that made the cone cut if the cap is meant to touch the cone",
				decaderr.ErrUnsupported, shortfall)
		}
		return r3.Vec{}, 0, fmt.Errorf("%w: sector cap reaches its far source plane: "+
			"a cap station misses the cone by %.3g", decaderr.ErrUnsupported, shortfall)
	}
	smooth := far.Scale(sample.Alpha)
	if !proofbound.FiniteVec(smooth) {
		return r3.Vec{}, 0, fmt.Errorf("%w: upper cap has no finite crossing", decaderr.ErrUnsupported)
	}
	if sample.Q-sample.QError > 1 {
		return smooth, sample.PointError, nil
	}
	return far, proofbound.AbsSumUpper(
		proofbound.DvLenUpper(proofbound.HeldDelta(far, smooth)), sample.PointError), nil
}

// SectorRootEdgeOrdered reports whether the edges labelled as the shared root arc
// form one monotone chain of root nodes whose angles run across sector.
func SectorRootEdgeOrdered(nodes []SectorNode, edges []SectorEdge,
	sector [2]float64) bool {
	if len(edges) < 1 || !(sector[0] < sector[1]) {
		return false
	}
	started, ascending, lastVertex, lastAngle := false, false, -1, 0.0
	for _, edge := range edges {
		if edge.Group != -1 {
			if started {
				return false
			}
			continue
		}
		if edge.U < 0 || edge.U >= len(nodes) || edge.V < 0 || edge.V >= len(nodes) ||
			!nodes[edge.U].RootBoundary || !nodes[edge.V].RootBoundary {
			return false
		}
		if !started {
			lastAngle = nodes[edge.U].Angle
			ascending = lastAngle == sector[0]
			if lastAngle != sector[0] && lastAngle != sector[1] {
				return false
			}
			started = true
		} else if edge.U != lastVertex || nodes[edge.U].Angle != lastAngle {
			return false
		}
		angle := nodes[edge.V].Angle
		if proofbound.IsNonFinite(angle) || angle < sector[0] || angle > sector[1] ||
			(ascending && angle <= lastAngle) || (!ascending && angle >= lastAngle) {
			return false
		}
		lastAngle, lastVertex = angle, edge.V
	}
	return started && ((ascending && lastAngle == sector[1]) ||
		(!ascending && lastAngle == sector[0]))
}
