package revolveloftfaces

import (
	"context"
	"math"
	"math/big"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
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

// ShellGridSize picks the quarter-circle ray count n for a Revolve section
// and returns it with its chord bound. A rational quarter-circle parameter
// advances by at most 2/n radians, so the chord sagitta is at most
// maxRadius/(2*n*n).
func ShellGridSize(spans []revolveSectionSpan, axisSpan int) (int, float64, bool) {
	maxRadius := 0.0
	for _, span := range spans {
		maxRadius = max(maxRadius, span.Start.V, span.End.V)
	}
	if axisSpan < 0 || !proofbound.FiniteVec(r3.Vec{X: maxRadius}) ||
		maxRadius <= 0 || maxRadius > 0.04*1024*1024 {
		return 0, 0, false
	}
	n := max(16, int(math.Ceil(math.Sqrt(maxRadius/0.04))))
	if n > 1024 {
		return 0, 0, false
	}
	return n, maxRadius / (2 * float64(n*n)), true
}

// ShellGrid builds the angular grid that every ring of one blank shell shares
// and returns it with the index of its positive-Y-axis ray. It refuses a grid
// larger than the ring budget of spanCount section sides, an anchor out of
// strict angular order, and a grid without exactly one positive-Y-axis ray.
func ShellGrid(n int, anchors ShellAnchors, spanCount int) ([]ShellRay, int, bool) {
	stationCount := 0
	for _, run := range anchors.Seams {
		stationCount += len(run)
	}
	rays := ShellRays(n, anchors)
	if len(rays) < 4 || len(rays) > 4*n+stationCount+len(anchors.Caps) ||
		len(rays)*spanCount > 1<<16 {
		return nil, -1, false
	}
	if !ShellRayOrderBounded(rays, anchors) {
		return nil, -1, false
	}
	positiveYAxis := -1
	for index, ray := range rays {
		if ray.Y.Sign() > 0 && ray.Z.Sign() == 0 {
			if positiveYAxis >= 0 {
				return nil, -1, false
			}
			positiveYAxis = index
		}
	}
	if positiveYAxis < 0 {
		return nil, -1, false
	}
	return rays, positiveYAxis, true
}

// shellRingCut is one cut being placed: its ring vertices and, for a cap
// cut, its index in ShellMeshInput.CapCuts.
type shellRingCut struct {
	ShellCut
	capIndex int
	vertices []int
}

// BuildShellMesh places every section-side start ring and every cut ring on
// one angular grid and joins adjacent rings with triangle bands. Each cut
// must lie strictly inside its straight section side; cuts on one side are
// sorted by their exact meridian fractions, and equal fractions refuse. A
// seam station and a cap endpoint keep their held points on their own rays.
// The finished shell must pass Audit.
func BuildShellMesh(ctx context.Context, in ShellMeshInput) (ShellMesh, []ShellCapRing, bool, error) {
	spans, rays, seams, caps := in.Spans, in.Rays, in.Anchors.Seams, in.Anchors.Caps
	if len(in.Groups) != len(spans) || len(in.SeamCuts) != len(seams) || len(seams) == 0 {
		return ShellMesh{}, nil, false, nil
	}
	out := ShellMesh{ChordBound: in.ChordBound,
		Seams: make([][]int, len(seams)), RayCount: len(rays), Rays: rays}
	out.Mesh.Groups = in.MeshGroups
	for run := range seams {
		out.Seams[run] = make([]int, len(seams[run]))
		for i := range out.Seams[run] {
			out.Seams[run][i] = -1
		}
	}
	appendPoint := func(point r3.Vec, bound float64) int {
		index := len(out.Mesh.Verts)
		out.Mesh.Verts = append(out.Mesh.Verts, point)
		out.VertexBounds = append(out.VertexBounds, bound)
		return index
	}
	makeRing := func(x, radius *big.Rat, cut *ShellCut) ([]int, bool) {
		if radius.Sign() < 0 {
			return nil, false
		}
		if radius.Sign() == 0 {
			point, bound, valid := HeldPoint(loftface.RatVec{x, lfZero(), lfZero()})
			if !valid {
				return nil, false
			}
			return []int{appendPoint(point, bound)}, true
		}
		ring := make([]int, len(rays))
		for j, ray := range rays {
			if cut != nil && cut.Seam {
				matched := false
				for _, ref := range ray.SeamRefs {
					if ref.Run != cut.SeamRun {
						continue
					}
					vertex := seams[ref.Run][ref.Station]
					ring[j] = appendPoint(vertex.Point, vertex.BlankBound)
					out.Seams[ref.Run][ref.Station] = ring[j]
					matched = true
					break
				}
				if matched {
					continue
				}
			}
			if cut != nil && !cut.Seam {
				found := -1
				for endpoint, id := range cut.CapIDs {
					if slices.Contains(ray.CapIDs, id) {
						found = endpoint
					}
				}
				if found >= 0 {
					vertex := caps[cut.CapIDs[found]]
					ring[j] = appendPoint(vertex.Point, vertex.BlankBound)
					continue
				}
			}
			point, bound, valid := MeridianRayPoint(
				[2]*big.Rat{ray.Y, ray.Z}, x, radius)
			if !valid {
				return nil, false
			}
			ring[j] = appendPoint(point, bound)
		}
		return ring, true
	}
	rings := make([][]int, len(spans))
	for i, span := range spans {
		if err := ctx.Err(); err != nil {
			return ShellMesh{}, nil, false, err
		}
		var valid bool
		rings[i], valid = makeRing(lfF(span.Start.U), lfF(span.Start.V), nil)
		if !valid {
			return ShellMesh{}, nil, false, nil
		}
	}
	cuts := make([][]shellRingCut, len(spans))
	for _, cut := range in.SeamCuts {
		if cut.Section < 0 || cut.Section >= len(spans) || !cut.Seam ||
			cut.X == nil || cut.Radius == nil {
			return ShellMesh{}, nil, false, nil
		}
		cuts[cut.Section] = append(cuts[cut.Section], shellRingCut{ShellCut: cut, capIndex: -1})
	}
	for i, cut := range in.CapCuts {
		if cut.Section < 0 || cut.Section >= len(spans) || cut.Seam ||
			cut.X == nil || cut.Radius == nil {
			return ShellMesh{}, nil, false, nil
		}
		cuts[cut.Section] = append(cuts[cut.Section], shellRingCut{ShellCut: cut, capIndex: i})
	}
	if (len(spans)+len(in.CapCuts)+len(in.SeamCuts))*len(rays) > 1<<16 {
		return ShellMesh{}, nil, false, nil
	}
	var capRings []ShellCapRing
	for i := range cuts {
		span := spans[i]
		parameter := func(cut shellRingCut) (*big.Rat, bool) {
			startX, endX := lfF(span.Start.U), lfF(span.End.U)
			startR, endR := lfF(span.Start.V), lfF(span.End.V)
			if startX.Cmp(endX) != 0 {
				fraction := lfDiv(lfSub(cut.X, startX), lfSub(endX, startX))
				return fraction, lfAdd(startR, lfMul(fraction, lfSub(endR, startR))).Cmp(cut.Radius) == 0
			}
			if cut.X.Cmp(startX) != 0 || startR.Cmp(endR) == 0 {
				return nil, false
			}
			return lfDiv(lfSub(cut.Radius, startR), lfSub(endR, startR)), true
		}
		for _, cut := range cuts[i] {
			fraction, valid := parameter(cut)
			if !valid || fraction.Sign() <= 0 || fraction.Cmp(lfOne()) >= 0 {
				return ShellMesh{}, nil, false, nil
			}
		}
		sort.Slice(cuts[i], func(a, b int) bool {
			first, _ := parameter(cuts[i][a])
			second, _ := parameter(cuts[i][b])
			return first.Cmp(second) < 0
		})
		for j := range cuts[i] {
			cut := &cuts[i][j]
			if j > 0 {
				previous, _ := parameter(cuts[i][j-1])
				current, _ := parameter(*cut)
				if previous.Cmp(current) >= 0 {
					return ShellMesh{}, nil, false, nil
				}
			}
			var valid bool
			cut.vertices, valid = makeRing(cut.X, cut.Radius, &cut.ShellCut)
			if !valid {
				return ShellMesh{}, nil, false, nil
			}
			if cut.Seam {
				continue
			}
			endpoints := [2]int{-1, -1}
			for rayIndex, ray := range rays {
				for side, id := range cut.CapIDs {
					if slices.Contains(ray.CapIDs, id) {
						if endpoints[side] >= 0 {
							return ShellMesh{}, nil, false, nil
						}
						endpoints[side] = cut.vertices[rayIndex]
					}
				}
			}
			if endpoints[0] < 0 || endpoints[1] < 0 || endpoints[0] == endpoints[1] {
				return ShellMesh{}, nil, false, nil
			}
			occupied, pathOK := OccupiedRingPath(cut.vertices, endpoints,
				in.PositiveYAxis, cut.ThroughPositiveY)
			if !pathOK {
				return ShellMesh{}, nil, false, nil
			}
			capRings = append(capRings, ShellCapRing{Section: i, Cut: cut.capIndex,
				Vertices: cut.vertices, Endpoints: endpoints, Occupied: occupied})
		}
	}
	out.SectionRings = make([][][]int, len(cuts))
	for i := range cuts {
		out.SectionRings[i] = append(out.SectionRings[i], rings[i])
		for _, cut := range cuts[i] {
			out.SectionRings[i] = append(out.SectionRings[i], cut.vertices)
		}
		out.SectionRings[i] = append(out.SectionRings[i], rings[(i+1)%len(rings)])
	}
	addBand := func(start, end []int, source, section, band int) bool {
		if len(start) != len(rays) && len(start) != 1 ||
			len(end) != len(rays) && len(end) != 1 || source < 0 {
			return false
		}
		for j := range rays {
			next := (j + 1) % len(rays)
			switch {
			case len(start) == 1 && len(end) == 1:
				return false
			case len(start) == 1:
				out.Mesh.Add([3]int{start[0], end[j], end[next]}, source)
				out.TriangleCells = append(out.TriangleCells,
					ShellTriangleCell{Section: section, Band: band, Ray: j})
			case len(end) == 1:
				out.Mesh.Add([3]int{start[j], end[0], start[next]}, source)
				out.TriangleCells = append(out.TriangleCells,
					ShellTriangleCell{Section: section, Band: band, Ray: j})
			default:
				out.Mesh.Add([3]int{start[j], end[j], end[next]}, source)
				out.Mesh.Add([3]int{start[j], end[next], start[next]}, source)
				out.TriangleCells = append(out.TriangleCells,
					ShellTriangleCell{Section: section, Band: band, Ray: j},
					ShellTriangleCell{Section: section, Band: band, Ray: j, Half: 1})
			}
		}
		return true
	}
	for i := range spans {
		if in.Groups[i] < 0 {
			continue
		}
		start := rings[i]
		for band, cut := range cuts[i] {
			if !addBand(start, cut.vertices, in.Groups[i], i, band) {
				return ShellMesh{}, nil, false, nil
			}
			start = cut.vertices
		}
		if !addBand(start, rings[(i+1)%len(rings)], in.Groups[i], i, len(cuts[i])) {
			return ShellMesh{}, nil, false, nil
		}
	}
	for _, run := range out.Seams {
		for _, index := range run {
			if index < 0 {
				return ShellMesh{}, nil, false, nil
			}
		}
	}
	// The ideal meridian quads have zero twist because every meridian edge
	// is straight and all rings use the same rays. The end caps are planar.
	// Angular chord sagitta and vertex movement bound each complete face.
	audited, err := out.Audit(ctx, math.Inf(1))
	if err != nil || !audited {
		return ShellMesh{}, nil, false, err
	}
	return out, capRings, true, nil
}

// Audit sets FaceBound to ChordBound plus the largest vertex bound and
// refuses a face bound at or above faceLimit. It then orients the mesh and
// requires one closed, connected genus-zero shell with no exact facet
// crossings and positive exact volume.
func (s *ShellMesh) Audit(ctx context.Context, faceLimit float64) (bool, error) {
	maxVertexBound := 0.0
	for _, bound := range s.VertexBounds {
		if proofbound.IsNonFinite(bound) || bound < 0 {
			return false, nil
		}
		maxVertexBound = max(maxVertexBound, bound)
	}
	s.FaceBound = proofbound.AbsSumUpper(s.ChordBound, maxVertexBound)
	if proofbound.IsNonFinite(s.FaceBound) || s.FaceBound >= faceLimit {
		return false, nil
	}
	if err := tessellation.OrientJoinedMesh(s.Mesh.Verts, s.Mesh.Tris); err != nil {
		return false, err
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx),
		s.Mesh.Verts, s.Mesh.Tris, len(s.Mesh.Tris), 1<<20); err != nil {
		return false, err
	}
	if !s.EulerTwo() {
		return false, nil
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, s.Mesh.Verts, s.Mesh.Tris)
	if err != nil {
		return false, err
	}
	return audit != nil && len(audit.Members) == 1 && audit.CompVol[0].Sign() > 0, nil
}

// EulerTwo reports whether the mesh has valid distinct triangle corners,
// references every vertex, and has Euler characteristic two.
func (s *ShellMesh) EulerTwo() bool {
	mesh := &s.Mesh
	if len(mesh.Verts) == 0 || len(mesh.Tris) == 0 {
		return false
	}
	edges := make(map[[2]int]struct{}, 3*len(mesh.Tris)/2)
	used := make([]bool, len(mesh.Verts))
	for _, tri := range mesh.Tris {
		for side := range 3 {
			a, b := tri[side], tri[(side+1)%3]
			if a < 0 || b < 0 || a >= len(mesh.Verts) || b >= len(mesh.Verts) || a == b {
				return false
			}
			used[a], used[b] = true, true
			edges[[2]int{min(a, b), max(a, b)}] = struct{}{}
		}
	}
	for _, referenced := range used {
		if !referenced {
			return false
		}
	}
	return len(mesh.Verts)-len(edges)+len(mesh.Tris) == 2
}

// Clone copies the mesh, vertex bounds, triangle cells, section rings and
// seam vertices. The rays are shared.
func (s *ShellMesh) Clone() ShellMesh {
	cloned := *s
	cloned.Mesh.Verts = slices.Clone(s.Mesh.Verts)
	cloned.Mesh.Tris = slices.Clone(s.Mesh.Tris)
	cloned.Mesh.Src = slices.Clone(s.Mesh.Src)
	cloned.Mesh.Groups = slices.Clone(s.Mesh.Groups)
	for i := range cloned.Mesh.Groups {
		cloned.Mesh.Groups[i].Origins = slices.Clone(s.Mesh.Groups[i].Origins)
	}
	cloned.VertexBounds = slices.Clone(s.VertexBounds)
	cloned.TriangleCells = slices.Clone(s.TriangleCells)
	cloned.SectionRings = make([][][]int, len(s.SectionRings))
	for i, section := range s.SectionRings {
		cloned.SectionRings[i] = make([][]int, len(section))
		for j, ring := range section {
			cloned.SectionRings[i][j] = slices.Clone(ring)
		}
	}
	cloned.Seams = make([][]int, len(s.Seams))
	for i, seam := range s.Seams {
		cloned.Seams[i] = slices.Clone(seam)
	}
	return cloned
}

// SameMesh reports whether two shells have equal vertices, triangles, triangle
// groups and group origins, vertex bounds, triangle cells, ray count, chord
// and face bounds, and section rings. Seam vertices and rays are not compared.
func (s *ShellMesh) SameMesh(other *ShellMesh) bool {
	if !slices.Equal(s.Mesh.Verts, other.Mesh.Verts) ||
		!slices.Equal(s.Mesh.Tris, other.Mesh.Tris) ||
		!slices.Equal(s.Mesh.Src, other.Mesh.Src) ||
		!slices.EqualFunc(s.Mesh.Groups, other.Mesh.Groups, func(x, y facetedrecord.Group) bool {
			return slices.Equal(x.Origins, y.Origins)
		}) ||
		!slices.Equal(s.VertexBounds, other.VertexBounds) ||
		!slices.Equal(s.TriangleCells, other.TriangleCells) ||
		s.RayCount != other.RayCount || s.ChordBound != other.ChordBound ||
		s.FaceBound != other.FaceBound ||
		len(s.SectionRings) != len(other.SectionRings) {
		return false
	}
	for i := range s.SectionRings {
		if len(s.SectionRings[i]) != len(other.SectionRings[i]) {
			return false
		}
		for j := range s.SectionRings[i] {
			if !slices.Equal(s.SectionRings[i][j], other.SectionRings[i][j]) {
				return false
			}
		}
	}
	return true
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

// SectionGroup reports whether the given mesh group belongs to the given
// section side: the side has a wall, the group keeps the side's face origins,
// and groups number the walled sides in section order.
func (s *ShellMesh) SectionGroup(spans []revolveSectionSpan, section, group int) bool {
	span := spans[section]
	if span.Face == nil || group >= len(s.Mesh.Groups) ||
		!slices.Equal(s.Mesh.Groups[group].Origins, span.Origins) {
		return false
	}
	want := 0
	for i := range spans {
		if i == section {
			return group == want
		}
		if spans[i].Face != nil {
			want++
		}
	}
	return false
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
