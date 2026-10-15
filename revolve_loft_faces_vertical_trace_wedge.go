package decad

import (
	"context"
	"math/big"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type revolveLoftFacesTracePlanarEdge struct{ a, b int }

func revolveLoftFacesTracePlanarPoint(shell *revolveLoftFacesBlankSeamShell,
	vertex int) meshbool.Xp2 {
	p := shell.mesh.Verts[vertex]
	return meshbool.NewXP2(lfF(p.Y), lfF(p.Z))
}

func revolveLoftFacesTracePlanarDistance2(a, b meshbool.Xp2) *big.Rat {
	y := lfSub(a.U, b.U)
	z := lfSub(a.V, b.V)
	return lfAdd(lfMul(y, y), lfMul(z, z))
}

func revolveLoftFacesTraceRadialNodes(shell *revolveLoftFacesBlankSeamShell,
	nodes []revolveLoftFacesTraceNode, center int, ring revolveLoftFacesShellCapRing,
) ([][]int, bool) {
	byRay := make([][]int, shell.rayCount)
	centerPoint := revolveLoftFacesTracePlanarPoint(shell, center)
	for _, node := range nodes {
		if node.ray < 0 {
			continue
		}
		if node.ray >= shell.rayCount || node.vertex == center {
			return nil, false
		}
		if node.vertex == ring.vertices[node.ray] {
			continue
		}
		byRay[node.ray] = append(byRay[node.ray], node.vertex)
	}
	for ray := range byRay {
		ringDistance := revolveLoftFacesTracePlanarDistance2(centerPoint,
			revolveLoftFacesTracePlanarPoint(shell, ring.vertices[ray]))
		sort.Slice(byRay[ray], func(i, j int) bool {
			return revolveLoftFacesTracePlanarDistance2(centerPoint,
				revolveLoftFacesTracePlanarPoint(shell, byRay[ray][i])).Cmp(
				revolveLoftFacesTracePlanarDistance2(centerPoint,
					revolveLoftFacesTracePlanarPoint(shell, byRay[ray][j]))) < 0
		})
		prior := lfZero()
		for _, vertex := range byRay[ray] {
			distance := revolveLoftFacesTracePlanarDistance2(centerPoint,
				revolveLoftFacesTracePlanarPoint(shell, vertex))
			if distance.Cmp(prior) <= 0 || distance.Cmp(ringDistance) >= 0 {
				return nil, false
			}
			prior = distance
		}
	}
	return byRay, true
}

func revolveLoftFacesTraceEdgesCross(points map[int]meshbool.Xp2,
	a, b revolveLoftFacesTracePlanarEdge) bool {
	pa, pb := points[a.a], points[a.b]
	qa, qb := points[b.a], points[b.b]
	for _, end := range [...]int{b.a, b.b} {
		if end == a.a || end == a.b {
			continue
		}
		if on, _ := meshbool.OnSegment2(pa, pb, points[end]); on {
			return true
		}
	}
	for _, end := range [...]int{a.a, a.b} {
		if end == b.a || end == b.b {
			continue
		}
		if on, _ := meshbool.OnSegment2(qa, qb, points[end]); on {
			return true
		}
	}
	if a.a == b.a || a.a == b.b || a.b == b.a || a.b == b.b {
		return false
	}
	p1, p2 := meshbool.Cross2xSign(pa, pb, qa), meshbool.Cross2xSign(pa, pb, qb)
	q1, q2 := meshbool.Cross2xSign(qa, qb, pa), meshbool.Cross2xSign(qa, qb, pb)
	return p1*p2 < 0 && q1*q2 < 0
}

func revolveLoftFacesTraceAngleHalf(y, z *big.Rat) int {
	if z.Sign() > 0 || z.Sign() == 0 && y.Sign() >= 0 {
		return 0
	}
	return 1
}

func revolveLoftFacesTraceWedgeFaces(ctx context.Context,
	shell *revolveLoftFacesBlankSeamShell, edges []revolveLoftFacesTracePlanarEdge,
) ([][3]int, bool, error) {
	if len(edges) < 3 || len(edges) > 512 {
		return nil, false, nil
	}
	points := make(map[int]meshbool.Xp2)
	adjacency := make(map[int][]int)
	seen := make(map[[2]int]struct{})
	for _, edge := range edges {
		if edge.a == edge.b {
			return nil, false, nil
		}
		key := [2]int{edge.a, edge.b}
		if key[0] > key[1] {
			key[0], key[1] = key[1], key[0]
		}
		if _, duplicate := seen[key]; duplicate {
			return nil, false, nil
		}
		seen[key] = struct{}{}
		for _, vertex := range key {
			if _, ok := points[vertex]; !ok {
				points[vertex] = revolveLoftFacesTracePlanarPoint(shell, vertex)
			}
		}
		adjacency[edge.a] = append(adjacency[edge.a], edge.b)
		adjacency[edge.b] = append(adjacency[edge.b], edge.a)
	}
	for i, a := range edges {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		for _, b := range edges[i+1:] {
			if revolveLoftFacesTraceEdgesCross(points, a, b) {
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
			ha, hb := revolveLoftFacesTraceAngleHalf(ay, az),
				revolveLoftFacesTraceAngleHalf(by, bz)
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
		for _, directed := range [...][2]int{{edge.a, edge.b}, {edge.b, edge.a}} {
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
			area := lfZero()
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

func revolveLoftFacesVerticalTraceSplitWedges(ctx context.Context,
	shell *revolveLoftFacesBlankSeamShell, ring revolveLoftFacesShellCapRing,
	section int, nodes []revolveLoftFacesTraceNode, path []revolveLoftFacesTraceEdge,
) (bool, error) {
	if section < 0 || section >= len(shell.sectionRings) ||
		len(ring.vertices) != shell.rayCount || len(path)+1 != len(nodes) {
		return false, nil
	}
	band := -1
	for i := 0; i+1 < len(shell.sectionRings[section]); i++ {
		if slices.Equal(shell.sectionRings[section][i+1], ring.vertices) {
			if band >= 0 {
				return false, nil
			}
			band = i
		}
	}
	if band < 0 || len(shell.sectionRings[section][band]) != 1 {
		return false, nil
	}
	center := shell.sectionRings[section][band][0]
	byRay, ok := revolveLoftFacesTraceRadialNodes(shell, nodes, center, ring)
	if !ok {
		return false, nil
	}
	byWedge := make([][]revolveLoftFacesTracePlanarEdge, shell.rayCount)
	for _, edge := range path {
		if edge.wedge < 0 || edge.wedge >= shell.rayCount {
			return false, nil
		}
		byWedge[edge.wedge] = append(byWedge[edge.wedge],
			revolveLoftFacesTracePlanarEdge{nodes[edge.start].vertex, nodes[edge.end].vertex})
	}
	nextTris := make([][3]int, 0, len(shell.mesh.Tris)+4*len(nodes))
	nextSrc := make([]int, 0, cap(nextTris))
	nextCells := make([]revolveLoftFacesShellTriangleCell, 0, cap(nextTris))
	seenRay := make([]bool, shell.rayCount)
	for i, tri := range shell.mesh.Tris {
		cell := shell.triangleCells[i]
		if cell.section != section || cell.band != band {
			nextTris = append(nextTris, tri)
			nextSrc = append(nextSrc, shell.mesh.Src[i])
			nextCells = append(nextCells, cell)
			continue
		}
		ray := cell.ray
		if ray < 0 || ray >= shell.rayCount || seenRay[ray] || cell.half != 0 ||
			!slices.Contains(tri[:], center) || !slices.Contains(tri[:], ring.vertices[ray]) ||
			!slices.Contains(tri[:], ring.vertices[(ray+1)%shell.rayCount]) {
			return false, nil
		}
		seenRay[ray] = true
		if len(byRay[ray]) == 0 && len(byRay[(ray+1)%shell.rayCount]) == 0 &&
			len(byWedge[ray]) == 0 {
			nextTris = append(nextTris, tri)
			nextSrc = append(nextSrc, shell.mesh.Src[i])
			nextCells = append(nextCells, cell)
			continue
		}
		edges := make([]revolveLoftFacesTracePlanarEdge, 0,
			3+len(byRay[ray])+len(byRay[(ray+1)%shell.rayCount])+len(byWedge[ray]))
		for _, side := range [...]int{ray, (ray + 1) % shell.rayCount} {
			prior := center
			for _, vertex := range byRay[side] {
				edges = append(edges, revolveLoftFacesTracePlanarEdge{prior, vertex})
				prior = vertex
			}
			edges = append(edges, revolveLoftFacesTracePlanarEdge{prior, ring.vertices[side]})
		}
		edges = append(edges, revolveLoftFacesTracePlanarEdge{
			ring.vertices[ray], ring.vertices[(ray+1)%shell.rayCount]})
		edges = append(edges, byWedge[ray]...)
		clipped, built, err := revolveLoftFacesTraceWedgeFaces(ctx, shell, edges)
		if err != nil || !built {
			return false, err
		}
		for half, triangle := range clipped {
			nextTris = append(nextTris, triangle)
			nextSrc = append(nextSrc, shell.mesh.Src[i])
			nextCells = append(nextCells, revolveLoftFacesShellTriangleCell{
				section: section, band: band, ray: ray, half: half})
		}
	}
	for _, seen := range seenRay {
		if !seen {
			return false, nil
		}
	}
	shell.mesh.Tris, shell.mesh.Src, shell.triangleCells = nextTris, nextSrc, nextCells
	return true, nil
}
