package revolveloftfaces

import (
	"context"
	"math/big"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// VerticalTracePlanarPoint reads a held shell vertex in its radial plane.
func VerticalTracePlanarPoint(vertices []r3.Vec, vertex int) meshbool.Xp2 {
	p := vertices[vertex]
	return meshbool.NewXP2(lfF(p.Y), lfF(p.Z))
}

// VerticalTracePlanarDistance2 returns the exact squared radial distance.
func VerticalTracePlanarDistance2(a, b meshbool.Xp2) *big.Rat {
	y := lfSub(a.U, b.U)
	z := lfSub(a.V, b.V)
	return lfAdd(lfMul(y, y), lfMul(z, z))
}

func verticalTraceEdgesCross(points map[int]meshbool.Xp2, a, b [2]int) bool {
	pa, pb := points[a[0]], points[a[1]]
	qa, qb := points[b[0]], points[b[1]]
	for _, end := range [...]int{b[0], b[1]} {
		if end == a[0] || end == a[1] {
			continue
		}
		if on, _ := meshbool.OnSegment2(pa, pb, points[end]); on {
			return true
		}
	}
	for _, end := range [...]int{a[0], a[1]} {
		if end == b[0] || end == b[1] {
			continue
		}
		if on, _ := meshbool.OnSegment2(qa, qb, points[end]); on {
			return true
		}
	}
	if a[0] == b[0] || a[0] == b[1] || a[1] == b[0] || a[1] == b[1] {
		return false
	}
	p1, p2 := meshbool.Cross2xSign(pa, pb, qa), meshbool.Cross2xSign(pa, pb, qb)
	q1, q2 := meshbool.Cross2xSign(qa, qb, pa), meshbool.Cross2xSign(qa, qb, pb)
	return p1*p2 < 0 && q1*q2 < 0
}

func verticalTraceAngleHalf(y, z *big.Rat) int {
	if z.Sign() > 0 || z.Sign() == 0 && y.Sign() >= 0 {
		return 0
	}
	return 1
}

// VerticalTraceWedgeFaces checks and triangulates one planar shell wedge.
func VerticalTraceWedgeFaces(ctx context.Context, vertices []r3.Vec,
	edges [][2]int) ([][3]int, bool, error) {
	if len(edges) < 3 || len(edges) > 512 {
		return nil, false, nil
	}
	points := make(map[int]meshbool.Xp2)
	adjacency := make(map[int][]int)
	seen := make(map[[2]int]struct{})
	for _, edge := range edges {
		if edge[0] == edge[1] {
			return nil, false, nil
		}
		key := edge
		if key[0] > key[1] {
			key[0], key[1] = key[1], key[0]
		}
		if _, duplicate := seen[key]; duplicate {
			return nil, false, nil
		}
		seen[key] = struct{}{}
		for _, vertex := range key {
			if _, ok := points[vertex]; !ok {
				points[vertex] = VerticalTracePlanarPoint(vertices, vertex)
			}
		}
		adjacency[edge[0]] = append(adjacency[edge[0]], edge[1])
		adjacency[edge[1]] = append(adjacency[edge[1]], edge[0])
	}
	for i, a := range edges {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		for _, b := range edges[i+1:] {
			if verticalTraceEdgesCross(points, a, b) {
				return nil, false, nil
			}
		}
	}
	for vertex, neighbors := range adjacency {
		center := points[vertex]
		sort.Slice(neighbors, func(i, j int) bool {
			a, b := points[neighbors[i]], points[neighbors[j]]
			ay, az := lfSub(a.U, center.U), lfSub(a.V, center.V)
			by, bz := lfSub(b.U, center.U), lfSub(b.V, center.V)
			ha, hb := verticalTraceAngleHalf(ay, az), verticalTraceAngleHalf(by, bz)
			if ha != hb {
				return ha < hb
			}
			return lfSub(lfMul(ay, bz), lfMul(az, by)).Sign() > 0
		})
		for i := 1; i < len(neighbors); i++ {
			a, b := points[neighbors[i-1]], points[neighbors[i]]
			ay, az := lfSub(a.U, center.U), lfSub(a.V, center.V)
			by, bz := lfSub(b.U, center.U), lfSub(b.V, center.V)
			if meshbool.Cross2xSign(center, a, b) == 0 &&
				lfAdd(lfMul(ay, by), lfMul(az, bz)).Sign() > 0 {
				return nil, false, nil
			}
		}
		adjacency[vertex] = neighbors
	}
	visited := make(map[[2]int]bool, 2*len(edges))
	triangles := make([][3]int, 0)
	positiveFaces := 0
	for _, edge := range edges {
		for _, directed := range [...][2]int{{edge[0], edge[1]}, {edge[1], edge[0]}} {
			if visited[directed] {
				continue
			}
			walk := directed
			poly := make([]int, 0)
			for step := 0; step <= 2*len(edges); step++ {
				if visited[walk] {
					if walk != directed {
						return nil, false, nil
					}
					break
				}
				visited[walk] = true
				poly = append(poly, walk[0])
				neighbors := adjacency[walk[1]]
				reverse := slices.Index(neighbors, walk[0])
				if reverse < 0 {
					return nil, false, nil
				}
				walk = [2]int{walk[1], neighbors[(reverse-1+len(neighbors))%len(neighbors)]}
			}
			if walk != directed || len(poly) < 3 {
				return nil, false, nil
			}
			area := new(big.Rat)
			for i, vertex := range poly {
				a, b := points[vertex], points[poly[(i+1)%len(poly)]]
				area = lfAdd(area, lfSub(lfMul(a.U, b.V), lfMul(a.V, b.U)))
			}
			if area.Sign() == 0 {
				return nil, false, nil
			}
			if area.Sign() < 0 {
				continue // the unbounded face
			}
			positiveFaces++
			local := make([]meshbool.Xp2, len(poly))
			ids := make([]int, len(poly))
			for i, vertex := range poly {
				local[i], ids[i] = points[vertex], i
			}
			clipped, err := meshbool.EarClipX(proofbound.NewWorkBudget(ctx), local, ids)
			if err != nil {
				return nil, false, err
			}
			for _, tri := range clipped {
				triangles = append(triangles, [3]int{poly[tri[0]], poly[tri[1]], poly[tri[2]]})
			}
		}
	}
	return triangles, positiveFaces > 0 && len(triangles) > 0, nil
}
