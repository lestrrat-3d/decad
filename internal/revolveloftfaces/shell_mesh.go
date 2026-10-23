package revolveloftfaces

import (
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
)

// ShellMesh is a closed Revolve blank shell on one shared angular grid. Its
// face bound charges angular chording and vertex movement. It is a
// construction piece, not a Union result.
type ShellMesh struct {
	Mesh          facetedrecord.JoinedMesh
	VertexBounds  []float64
	Seams         [][]int // shell vertex of each source run station
	TriangleCells []ShellTriangleCell
	SectionRings  [][][]int // every ring of each section side, start to end
	Rays          []ShellRay
	RayCount      int
	ChordBound    float64
	FaceBound     float64
}

// ShellCut is one exact meridian ring on a Revolve section side at axial X
// and radius Radius. A seam cut places run SeamRun's held stations on their
// own rays. Any other cut is a cap cut: it places its two certified cap
// endpoints CapIDs on their rays, and ThroughPositiveY selects which walk
// between them the source arc occupies.
type ShellCut struct {
	Section          int
	X, Radius        *big.Rat
	Seam             bool
	SeamRun          int
	CapIDs           [2]int
	ThroughPositiveY bool
}

// ShellCapRing is the ring of one cap cut. Cut is the cut's index in
// ShellMeshInput.CapCuts. Endpoints are the shell vertices of the two cap
// endpoints, and Occupied is the ring walk between them that the source arc
// occupies.
type ShellCapRing struct {
	Section   int
	Cut       int
	Vertices  []int
	Endpoints [2]int
	Occupied  []int
}

// ShellMeshInput is the authenticated data one blank shell places. Groups
// gives the mesh group of each section side; -1 marks the side on the
// Revolve axis, which gets no wall. SeamCuts holds one cut per source run,
// in run order.
type ShellMeshInput struct {
	Spans         []revolveSectionSpan
	Groups        []int
	MeshGroups    []facetedrecord.Group
	Anchors       ShellAnchors
	Rays          []ShellRay
	PositiveYAxis int
	ChordBound    float64
	SeamCuts      []ShellCut
	CapCuts       []ShellCut
}

// CellMatches reports whether tri has the vertices that cell labels, in any
// order, read from the cell's two section rings. The caller checks that the
// cell's section, band and ray lie inside the shell.
func (s *ShellMesh) CellMatches(tri [3]int, cell ShellTriangleCell) bool {
	want, valid := s.cellTriangle(cell)
	if !valid {
		return false
	}
	slices.Sort(tri[:])
	slices.Sort(want[:])
	return tri == want
}

func (s *ShellMesh) cellTriangle(cell ShellTriangleCell) ([3]int, bool) {
	var zero [3]int
	start := s.SectionRings[cell.Section][cell.Band]
	end := s.SectionRings[cell.Section][cell.Band+1]
	if (len(start) != 1 && len(start) != s.RayCount) ||
		(len(end) != 1 && len(end) != s.RayCount) ||
		len(start) == 1 && len(end) == 1 {
		return zero, false
	}
	next := (cell.Ray + 1) % s.RayCount
	switch {
	case len(start) == 1:
		if cell.Half != 0 {
			return zero, false
		}
		return [3]int{start[0], end[cell.Ray], end[next]}, true
	case len(end) == 1:
		if cell.Half != 0 {
			return zero, false
		}
		return [3]int{start[cell.Ray], end[0], start[next]}, true
	case cell.Half == 0:
		return [3]int{start[cell.Ray], end[cell.Ray], end[next]}, true
	case cell.Half == 1:
		return [3]int{start[cell.Ray], end[next], start[next]}, true
	default:
		return zero, false
	}
}

// OccupiedRingPath returns the walk along ring between its two endpoint
// vertices that a certified cap arc occupies. The positive-Y-axis ray
// distinguishes the two walks: throughPositiveY selects the one through it.
func OccupiedRingPath(ring []int, endpoints [2]int,
	positiveYAxis int, throughPositiveY bool) ([]int, bool) {
	if len(ring) < 4 || positiveYAxis < 0 || positiveYAxis >= len(ring) ||
		endpoints[0] == endpoints[1] {
		return nil, false
	}
	indices := [2]int{-1, -1}
	for index, vertex := range ring {
		for side, endpoint := range endpoints {
			if vertex != endpoint {
				continue
			}
			if indices[side] >= 0 {
				return nil, false
			}
			indices[side] = index
		}
	}
	if indices[0] < 0 || indices[1] < 0 ||
		indices[0] == positiveYAxis || indices[1] == positiveYAxis {
		return nil, false
	}
	walk := func(direction int) ([]int, bool) {
		var result []int
		for index := indices[0]; ; index = (index + direction + len(ring)) % len(ring) {
			result = append(result, ring[index])
			if index == indices[1] {
				break
			}
			if len(result) >= len(ring) {
				return nil, false
			}
		}
		containsAxis := false
		for _, vertex := range result {
			containsAxis = containsAxis || vertex == ring[positiveYAxis]
		}
		return result, containsAxis
	}
	forward, hasAxis := walk(1)
	if len(forward) == 0 {
		return nil, false
	}
	if hasAxis == throughPositiveY {
		return forward, true
	}
	backward, hasAxis := walk(-1)
	return backward, len(backward) >= 2 && hasAxis == throughPositiveY
}
