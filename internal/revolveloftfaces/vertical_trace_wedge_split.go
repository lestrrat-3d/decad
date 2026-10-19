package revolveloftfaces

import (
	"context"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
)

// VerticalTraceSplitReaders reads transient root trace records without
// retaining a second mesh or cell representation.
type VerticalTraceSplitReaders[N, E, C any] struct {
	NodeVertex func(N) int
	NodeRay    func(N) int
	EdgeEnds   func(E) (int, int)
	EdgeWedge  func(E) int
	CellLabels func(C) (int, int, int, int)
	NewCell    func(int, int, int, int) C
}

func verticalTraceRadialNodes[N, E, C any](mesh *facetedrecord.JoinedMesh,
	rayCount int, nodes []N, center int, ring []int,
	read VerticalTraceSplitReaders[N, E, C],
) ([][]int, bool) {
	byRay := make([][]int, rayCount)
	centerPoint := VerticalTracePlanarPoint(mesh.Verts, center)
	for _, node := range nodes {
		ray := read.NodeRay(node)
		if ray < 0 {
			continue
		}
		vertex := read.NodeVertex(node)
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
func VerticalTraceSplitWedges[N, E, C any](ctx context.Context,
	mesh *facetedrecord.JoinedMesh, sectionRings [][][]int, rayCount int,
	ring []int, section int, nodes []N, path []E, cells []C,
	read VerticalTraceSplitReaders[N, E, C],
) ([][3]int, []int, []C, bool, error) {
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
	byRay, ok := verticalTraceRadialNodes(mesh, rayCount, nodes, center, ring, read)
	if !ok {
		return nil, nil, nil, false, nil
	}
	byWedge := make([][][2]int, rayCount)
	for _, edge := range path {
		wedge := read.EdgeWedge(edge)
		if wedge < 0 || wedge >= rayCount {
			return nil, nil, nil, false, nil
		}
		start, end := read.EdgeEnds(edge)
		byWedge[wedge] = append(byWedge[wedge],
			[2]int{read.NodeVertex(nodes[start]), read.NodeVertex(nodes[end])})
	}
	nextTris := make([][3]int, 0, len(mesh.Tris)+4*len(nodes))
	nextSrc := make([]int, 0, cap(nextTris))
	nextCells := make([]C, 0, cap(nextTris))
	seenRay := make([]bool, rayCount)
	for i, tri := range mesh.Tris {
		cell := cells[i]
		cellSection, cellBand, ray, half := read.CellLabels(cell)
		if cellSection != section || cellBand != band {
			nextTris = append(nextTris, tri)
			nextSrc = append(nextSrc, mesh.Src[i])
			nextCells = append(nextCells, cell)
			continue
		}
		if ray < 0 || ray >= rayCount || seenRay[ray] || half != 0 ||
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
			nextCells = append(nextCells, read.NewCell(section, band, ray, clippedHalf))
		}
	}
	for _, seen := range seenRay {
		if !seen {
			return nil, nil, nil, false, nil
		}
	}
	return nextTris, nextSrc, nextCells, true, nil
}
