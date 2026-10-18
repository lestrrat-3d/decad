package decad

import (
	"context"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
)

func revolveLoftFacesTraceRadialNodes(shell *revolveLoftFacesBlankSeamShell,
	nodes []revolveLoftFacesTraceNode, center int, ring revolveLoftFacesShellCapRing,
) ([][]int, bool) {
	byRay := make([][]int, shell.rayCount)
	centerPoint := revolveloftfaces.VerticalTracePlanarPoint(shell.mesh.Verts, center)
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
		ringDistance := revolveloftfaces.VerticalTracePlanarDistance2(centerPoint,
			revolveloftfaces.VerticalTracePlanarPoint(shell.mesh.Verts, ring.vertices[ray]))
		sort.Slice(byRay[ray], func(i, j int) bool {
			return revolveloftfaces.VerticalTracePlanarDistance2(centerPoint,
				revolveloftfaces.VerticalTracePlanarPoint(shell.mesh.Verts, byRay[ray][i])).Cmp(
				revolveloftfaces.VerticalTracePlanarDistance2(centerPoint,
					revolveloftfaces.VerticalTracePlanarPoint(shell.mesh.Verts, byRay[ray][j]))) < 0
		})
		prior := lfZero()
		for _, vertex := range byRay[ray] {
			distance := revolveloftfaces.VerticalTracePlanarDistance2(centerPoint,
				revolveloftfaces.VerticalTracePlanarPoint(shell.mesh.Verts, vertex))
			if distance.Cmp(prior) <= 0 || distance.Cmp(ringDistance) >= 0 {
				return nil, false
			}
			prior = distance
		}
	}
	return byRay, true
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
	byWedge := make([][][2]int, shell.rayCount)
	for _, edge := range path {
		if edge.wedge < 0 || edge.wedge >= shell.rayCount {
			return false, nil
		}
		byWedge[edge.wedge] = append(byWedge[edge.wedge],
			[2]int{nodes[edge.start].vertex, nodes[edge.end].vertex})
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
		edges := make([][2]int, 0,
			3+len(byRay[ray])+len(byRay[(ray+1)%shell.rayCount])+len(byWedge[ray]))
		for _, side := range [...]int{ray, (ray + 1) % shell.rayCount} {
			prior := center
			for _, vertex := range byRay[side] {
				edges = append(edges, [2]int{prior, vertex})
				prior = vertex
			}
			edges = append(edges, [2]int{prior, ring.vertices[side]})
		}
		edges = append(edges, [2]int{
			ring.vertices[ray], ring.vertices[(ray+1)%shell.rayCount]})
		edges = append(edges, byWedge[ray]...)
		clipped, built, err := revolveloftfaces.VerticalTraceWedgeFaces(ctx, shell.mesh.Verts, edges)
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
