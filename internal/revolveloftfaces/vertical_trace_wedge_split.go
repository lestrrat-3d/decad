package revolveloftfaces

import (
	"context"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
)

func verticalTraceRadialNodes(mesh *facetedrecord.JoinedMesh,
	rayCount int, nodes []VerticalTraceNode, center int, ring []int,
) ([][]int, bool) {
	byRay := make([][]int, rayCount)
	centerPoint := VerticalTracePlanarPoint(mesh.Verts, center)
	for _, node := range nodes {
		ray := node.Ray
		if ray < 0 {
			continue
		}
		vertex := node.Vertex
		if ray >= rayCount || vertex == center {
			return nil, false
		}
		if vertex == ring[ray] {
			continue
		}
		byRay[ray] = append(byRay[ray], vertex)
	}
	for ray := range byRay {
		ringDistance := VerticalTracePlanarDistance2(centerPoint,
			VerticalTracePlanarPoint(mesh.Verts, ring[ray]))
		sort.Slice(byRay[ray], func(i, j int) bool {
			return VerticalTracePlanarDistance2(centerPoint,
				VerticalTracePlanarPoint(mesh.Verts, byRay[ray][i])).Cmp(
				VerticalTracePlanarDistance2(centerPoint,
					VerticalTracePlanarPoint(mesh.Verts, byRay[ray][j]))) < 0
		})
		prior := lfZero()
		for _, vertex := range byRay[ray] {
			distance := VerticalTracePlanarDistance2(centerPoint,
				VerticalTracePlanarPoint(mesh.Verts, vertex))
			if distance.Cmp(prior) <= 0 || distance.Cmp(ringDistance) >= 0 {
				return nil, false
			}
			prior = distance
		}
	}
	return byRay, true
}

// VerticalTraceSplitWedges rewrites one fan band after the caller has bound
// its live source graph and shell. It returns replacement slices only on success.
func VerticalTraceSplitWedges(ctx context.Context,
	mesh *facetedrecord.JoinedMesh, sectionRings [][][]int, rayCount int,
	ring []int, section int, nodes []VerticalTraceNode, path []VerticalTraceEdge,
	cells []ShellTriangleCell,
) ([][3]int, []int, []ShellTriangleCell, bool, error) {
	if section < 0 || section >= len(sectionRings) ||
		len(ring) != rayCount || len(path)+1 != len(nodes) {
		return nil, nil, nil, false, nil
	}
	band := -1
	for i := 0; i+1 < len(sectionRings[section]); i++ {
		if slices.Equal(sectionRings[section][i+1], ring) {
			if band >= 0 {
				return nil, nil, nil, false, nil
			}
			band = i
		}
	}
	if band < 0 || len(sectionRings[section][band]) != 1 {
		return nil, nil, nil, false, nil
	}
	center := sectionRings[section][band][0]
	byRay, ok := verticalTraceRadialNodes(mesh, rayCount, nodes, center, ring)
	if !ok {
		return nil, nil, nil, false, nil
	}
	byWedge := make([][][2]int, rayCount)
	for _, edge := range path {
		wedge := edge.Wedge
		if wedge < 0 || wedge >= rayCount {
			return nil, nil, nil, false, nil
		}
		byWedge[wedge] = append(byWedge[wedge],
			[2]int{nodes[edge.Start].Vertex, nodes[edge.End].Vertex})
	}
	nextTris := make([][3]int, 0, len(mesh.Tris)+4*len(nodes))
	nextSrc := make([]int, 0, cap(nextTris))
	nextCells := make([]ShellTriangleCell, 0, cap(nextTris))
	seenRay := make([]bool, rayCount)
	for i, tri := range mesh.Tris {
		cell := cells[i]
		ray := cell.Ray
		if cell.Section != section || cell.Band != band {
			nextTris = append(nextTris, tri)
			nextSrc = append(nextSrc, mesh.Src[i])
			nextCells = append(nextCells, cell)
			continue
		}
		if ray < 0 || ray >= rayCount || seenRay[ray] || cell.Half != 0 ||
			!slices.Contains(tri[:], center) || !slices.Contains(tri[:], ring[ray]) ||
			!slices.Contains(tri[:], ring[(ray+1)%rayCount]) {
			return nil, nil, nil, false, nil
		}
		seenRay[ray] = true
		if len(byRay[ray]) == 0 && len(byRay[(ray+1)%rayCount]) == 0 &&
			len(byWedge[ray]) == 0 {
			nextTris = append(nextTris, tri)
			nextSrc = append(nextSrc, mesh.Src[i])
			nextCells = append(nextCells, cell)
			continue
		}
		edges := make([][2]int, 0,
			3+len(byRay[ray])+len(byRay[(ray+1)%rayCount])+len(byWedge[ray]))
		for _, side := range [...]int{ray, (ray + 1) % rayCount} {
			prior := center
			for _, vertex := range byRay[side] {
				edges = append(edges, [2]int{prior, vertex})
				prior = vertex
			}
			edges = append(edges, [2]int{prior, ring[side]})
		}
		edges = append(edges, [2]int{ring[ray], ring[(ray+1)%rayCount]})
		edges = append(edges, byWedge[ray]...)
		clipped, built, err := VerticalTraceWedgeFaces(ctx, mesh.Verts, edges)
		if err != nil || !built {
			return nil, nil, nil, false, err
		}
		for clippedHalf, triangle := range clipped {
			nextTris = append(nextTris, triangle)
			nextSrc = append(nextSrc, mesh.Src[i])
			nextCells = append(nextCells, ShellTriangleCell{
				Section: section, Band: band, Ray: ray, Half: clippedHalf})
		}
	}
	for _, seen := range seenRay {
		if !seen {
			return nil, nil, nil, false, nil
		}
	}
	return nextTris, nextSrc, nextCells, true, nil
}
