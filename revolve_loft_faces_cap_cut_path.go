package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// A held cone-cap X section supplies only a topology template. Its ordered
// points are rebuilt on the exact occupied source circle before use. The
// source proof, not the proximity of held facets, admits those circle points.
type revolveLoftFacesCapCutPath struct {
	arc    *revolveLoftFacesToolCapOccupiedArc
	points []revolveLoftFacesCapCutPoint // negative Z through +Y to positive Z
}

type revolveLoftFacesCapCutPoint struct {
	point       r3.Vec
	ray         [2]*big.Rat
	sourceBound float64 // true occupied circle to held point
	facetBound  float64 // original cone-cap facet correspondence
	moveBound   float64 // original exact held cut to new held point
	endpoint    int     // -1 for interior, 0/1 for negative/positive Z
}

type revolveLoftFacesCapCutNode struct {
	y, z       *big.Rat
	edge       [2]int
	facetBound float64
	adjacent   []int
}

// This cuts the actual held heel-cap triangles by the authenticated X plane,
// requires one simple ordered path with live source-face endpoints, then
// places its interior rays on the true occupied circle. It neither selects
// retained facets nor grants a Union contact exception.
func revolveLoftFacesHeldCapCutPath(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	arc *revolveLoftFacesToolCapOccupiedArc) (*revolveLoftFacesCapCutPath, bool, error) {
	if ctx == nil || proof == nil || proof.tooth == nil || proof.trim == nil ||
		arc == nil || arc.circle.capFace != proof.heelFace ||
		arc.circle.tool != 1 || arc.circle.x == nil || arc.circle.radius == nil ||
		arc.circle.radius.Sign() <= 0 || !arc.throughPositiveY {
		return nil, false, nil
	}
	current, err := revolveLoftFacesToolCapOccupiedArcCurrent(ctx, proof, arc)
	if err != nil || !current {
		return nil, false, err
	}
	p, ok := proof.tooth.payload.(facetedPayload)
	if !ok || p.loftFacesTrim != proof.trim ||
		!revolveLoftFacesTrimCurrent(proof.tooth, proof.trim) ||
		len(p.tris) != len(p.src) || len(p.tris) != len(p.faceOf) ||
		len(p.verts) != len(p.vertexBound) ||
		proof.trim.heelCapGroup < 0 || proof.trim.heelCapGroup >= len(p.groups) {
		return nil, false, nil
	}
	type owner struct{ triangle int }
	owners := make(map[[2]int][]owner, len(p.tris)*3/2)
	for ti, triangle := range p.tris {
		if ti%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
		}
		for corner := range 3 {
			a, b := triangle[corner], triangle[(corner+1)%3]
			if a < 0 || b < 0 || a >= len(p.verts) || b >= len(p.verts) || a == b {
				return nil, false, nil
			}
			key := [2]int{min(a, b), max(a, b)}
			owners[key] = append(owners[key], owner{ti})
			if len(owners[key]) > 2 {
				return nil, false, nil
			}
		}
	}
	nodes := make([]revolveLoftFacesCapCutNode, 0)
	index := make(map[string]int)
	seenSegments := make(map[[2]int]struct{})
	capCount, cutCount := 0, 0
	for ti, triangle := range p.tris {
		if p.src[ti] != proof.trim.heelCapGroup {
			continue
		}
		capCount++
		if capCount > 1<<18 {
			return nil, false, nil
		}
		facetBound := max(p.vertexBound[triangle[0]],
			p.vertexBound[triangle[1]], p.vertexBound[triangle[2]])
		if proofbound.IsNonFinite(facetBound) || facetBound < 0 {
			return nil, false, nil
		}
		var hits [2]int
		hitCount := 0
		for corner := range 3 {
			a, b := triangle[corner], triangle[(corner+1)%3]
			first, last := p.verts[a], p.verts[b]
			x0, x1 := lfF(first.X), lfF(last.X)
			s0, s1 := x0.Cmp(arc.circle.x), x1.Cmp(arc.circle.x)
			if s0 == 0 || s1 == 0 {
				return nil, false, nil // a degenerate plane cut needs its own proof
			}
			if s0 == s1 {
				continue
			}
			fraction := lfDiv(lfSub(arc.circle.x, x0), lfSub(x1, x0))
			if fraction.Sign() <= 0 || fraction.Cmp(lfOne()) >= 0 {
				return nil, false, nil
			}
			y := lfAdd(lfF(first.Y), lfMul(fraction, lfSub(lfF(last.Y), lfF(first.Y))))
			z := lfAdd(lfF(first.Z), lfMul(fraction, lfSub(lfF(last.Z), lfF(first.Z))))
			if y.Sign() <= 0 {
				return nil, false, nil // this bounded class uses one +Y arc
			}
			key := y.RatString() + "/" + z.RatString()
			edge := [2]int{min(a, b), max(a, b)}
			id, found := index[key]
			if !found {
				id = len(nodes)
				index[key] = id
				nodes = append(nodes, revolveLoftFacesCapCutNode{
					y: y, z: z, edge: edge, facetBound: facetBound})
			} else if nodes[id].edge != edge {
				return nil, false, nil
			}
			nodes[id].facetBound = max(nodes[id].facetBound, facetBound)
			if hitCount >= len(hits) {
				return nil, false, nil
			}
			hits[hitCount] = id
			hitCount++
		}
		if hitCount == 0 {
			continue
		}
		if hitCount != 2 || hits[0] == hits[1] {
			return nil, false, nil
		}
		key := [2]int{min(hits[0], hits[1]), max(hits[0], hits[1])}
		if _, duplicate := seenSegments[key]; duplicate {
			return nil, false, nil
		}
		seenSegments[key] = struct{}{}
		nodes[hits[0]].adjacent = append(nodes[hits[0]].adjacent, hits[1])
		nodes[hits[1]].adjacent = append(nodes[hits[1]].adjacent, hits[0])
		cutCount++
	}
	if capCount == 0 || cutCount < 2 || len(nodes) != cutCount+1 {
		return nil, false, nil
	}
	ends := [2]int{-1, -1}
	for i, node := range nodes {
		if len(node.adjacent) == 2 {
			continue
		}
		if len(node.adjacent) != 1 || node.z.Sign() == 0 {
			return nil, false, nil
		}
		side := 0
		if node.z.Sign() > 0 {
			side = 1
		}
		if ends[side] >= 0 || len(owners[node.edge]) != 2 {
			return nil, false, nil
		}
		other := -1
		for _, owner := range owners[node.edge] {
			if p.src[owner.triangle] != proof.trim.heelCapGroup {
				other = owner.triangle
			}
		}
		if other < 0 || p.faceOf[other] < 0 ||
			p.faceOf[other] >= len(proof.tooth.Faces()) {
			return nil, false, nil
		}
		expected := arc.negativeZ.face
		if side == 1 {
			expected = arc.positiveZ.face
		}
		if proof.tooth.Faces()[p.faceOf[other]] != expected {
			return nil, false, nil
		}
		ends[side] = i
	}
	if ends[0] < 0 || ends[1] < 0 {
		return nil, false, nil
	}
	endpointVertices, ok, err := revolveLoftFacesCapArcSeamVertices(ctx, proof, arc)
	if err != nil || !ok {
		return nil, false, err
	}
	result := &revolveLoftFacesCapCutPath{arc: arc,
		points: make([]revolveLoftFacesCapCutPoint, 0, len(nodes))}
	previous, currentID := -1, ends[0]
	visited := make(map[int]struct{}, len(nodes))
	for len(result.points) < len(nodes) {
		if _, duplicate := visited[currentID]; duplicate {
			return nil, false, nil
		}
		visited[currentID] = struct{}{}
		node := nodes[currentID]
		if node.y.Sign() <= 0 {
			return nil, false, nil
		}
		entry := revolveLoftFacesCapCutPoint{ray: [2]*big.Rat{node.y, node.z},
			facetBound: node.facetBound, endpoint: -1}
		side := -1
		switch currentID {
		case ends[0]:
			side = 0
		case ends[1]:
			side = 1
		}
		if side >= 0 {
			entry.endpoint = side
			entry.point = endpointVertices[side].point
			entry.sourceBound = endpointVertices[side].loftBound
			entry.ray = [2]*big.Rat{lfF(entry.point.Y), lfF(entry.point.Z)}
		} else {
			if !revolveLoftFacesCapCutRayInsideArc(node, arc) {
				return nil, false, nil
			}
			var placed bool
			entry.point, entry.sourceBound, placed = revolveLoftFacesMeridianRayPoint(
				entry.ray, arc.circle.x, arc.circle.radius)
			if !placed {
				return nil, false, nil
			}
		}
		entry.moveBound = proofbound.Radius3D(max(
			proofbound.RatAbsDiff(arc.circle.x, entry.point.X),
			proofbound.RatAbsDiff(node.y, entry.point.Y),
			proofbound.RatAbsDiff(node.z, entry.point.Z)))
		if proofbound.IsNonFinite(entry.sourceBound) ||
			proofbound.IsNonFinite(entry.moveBound) ||
			proofbound.IsNonFinite(proofbound.AbsSumUpper(
				entry.facetBound, entry.moveBound, entry.sourceBound)) {
			return nil, false, nil
		}
		result.points = append(result.points, entry)
		if currentID == ends[1] {
			break
		}
		next := -1
		for _, candidate := range node.adjacent {
			if candidate != previous {
				next = candidate
			}
		}
		if next < 0 || len(result.points) >= len(nodes) {
			return nil, false, nil
		}
		following := nodes[next]
		if lfSub(lfMul(node.y, following.z),
			lfMul(node.z, following.y)).Sign() <= 0 {
			return nil, false, nil
		}
		previous, currentID = currentID, next
	}
	if len(result.points) != len(nodes) ||
		result.points[0].endpoint != 0 ||
		result.points[len(result.points)-1].endpoint != 1 ||
		!revolveLoftFacesCapCutFinalRaysOrdered(result.points) {
		return nil, false, nil
	}
	return result, true, nil
}

// Endpoint rays use rounded, source-certified cap vertices. Recheck their
// exact stored order after replacing the original held-cut rays.
func revolveLoftFacesCapCutFinalRaysOrdered(points []revolveLoftFacesCapCutPoint) bool {
	if len(points) < 2 {
		return false
	}
	for index, point := range points {
		if point.ray[0] == nil || point.ray[1] == nil || point.ray[0].Sign() <= 0 {
			return false
		}
		if index > 0 {
			previous := points[index-1].ray
			cross := lfSub(lfMul(previous[0], point.ray[1]),
				lfMul(previous[1], point.ray[0]))
			if cross.Sign() <= 0 {
				return false
			}
		}
	}
	return true
}

func revolveLoftFacesCapCutRayInsideArc(node revolveLoftFacesCapCutNode,
	arc *revolveLoftFacesToolCapOccupiedArc) bool {
	y := loftFaceInterval{lo: node.y, hi: node.y}
	z := loftFaceInterval{lo: node.z, hi: node.z}
	negative := lfIntervalSub(
		revolveLoftFacesCapCutIntervalMul(arc.negativeZ.box[1], z),
		revolveLoftFacesCapCutIntervalMul(arc.negativeZ.box[2], y))
	positive := lfIntervalSub(
		revolveLoftFacesCapCutIntervalMul(y, arc.positiveZ.box[2]),
		revolveLoftFacesCapCutIntervalMul(z, arc.positiveZ.box[1]))
	return negative.lo.Sign() > 0 && positive.lo.Sign() > 0
}

func revolveLoftFacesCapCutIntervalMul(a, b loftFaceInterval) loftFaceInterval {
	products := [4]*big.Rat{lfMul(a.lo, b.lo), lfMul(a.lo, b.hi),
		lfMul(a.hi, b.lo), lfMul(a.hi, b.hi)}
	out := loftFaceInterval{products[0], products[0]}
	for _, value := range products[1:] {
		out.lo, out.hi = ratMin(out.lo, value), ratMax(out.hi, value)
	}
	return out
}
