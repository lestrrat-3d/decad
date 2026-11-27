package loftrecord

import (
	"slices"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// directSections records two bounded analytic loops at original construction.
// Line and arc segment values contain no pointers into the caller's records.
type directSections struct {
	body          *topo.Body
	frames        [2]r3.Frame
	planes        [2]sectionrecord.PlaneRecord
	segments      [2][]momentinput.CurveSegment
	alignment     []int
	cellSegments  []int
	capFaces      [2]*topo.Face
	wallFaces     []*topo.Face
	wallTriangles [][3]r3.Vec
	capTriangles  [2][][3]r3.Vec
}

func originalLineArcSections(p Payload) bool {
	if !p.AuthenticatedReconstruction || p.SurfaceResult || p.ThroughBoreCut || p.Xform != r3.Identity() || p.ConstructionDelta != 0 || p.JoinDelta != 0 || p.JoinPlaneGap != 0 || len(p.Alignment) > 1 {
		return false
	}
	for _, profile := range [2]momentinput.Profile{p.Profile0, p.Profile1} {
		if len(profile.Holes) != 0 || len(profile.Outer.Segments) == 0 {
			return false
		}
		for _, s := range profile.Outer.Segments {
			switch s.(type) {
			case momentinput.LineSeg, momentinput.ArcSeg:
			default:
				return false
			}
		}
	}
	return true
}
func directSectionClass(p Payload) bool {
	return len(p.Profile0.Outer.Segments) <= 64 && len(p.Profile1.Outer.Segments) <= 64 && originalLineArcSections(p)
}

func sealDirectSections(b *topo.Body, p Payload) *directSections {
	if !directSectionClass(p) {
		return nil
	}
	return &directSections{
		body:      b,
		frames:    [2]r3.Frame{p.Frame0, p.Frame1},
		planes:    [2]sectionrecord.PlaneRecord{p.Plane0, p.Plane1},
		alignment: slices.Clone(p.Alignment),
		segments:  [2][]momentinput.CurveSegment{slices.Clone(p.Profile0.Outer.Segments), slices.Clone(p.Profile1.Outer.Segments)},
	}
}

// MatchesDirectSections compares the original section carriers, alignment and
// placement used to bound a stopped bevel against their construction records.
func (p Payload) MatchesDirectSections(b *topo.Body) bool {
	if p.SourceIdentity == nil || p.SourceIdentity.direct == nil || !directSectionClass(p) {
		return false
	}
	s := p.SourceIdentity.direct
	return s.body == b && s.frames == [2]r3.Frame{p.Frame0, p.Frame1} &&
		s.planes == [2]sectionrecord.PlaneRecord{p.Plane0, p.Plane1} && slices.Equal(s.alignment, p.Alignment) &&
		slices.Equal(s.segments[0], p.Profile0.Outer.Segments) && slices.Equal(s.segments[1], p.Profile1.Outer.Segments)
}

// sealDirectCellSegments records the original builder's bounded outer-loop map.
// It runs before the source identity becomes visible outside the builder.
func sealDirectCellSegments(s *directSections, segments [][]int, sourceCount int, budget *proofbound.WorkBudget) error {
	if s == nil || s.cellSegments != nil || len(segments) != 1 || len(segments[0]) == 0 ||
		len(segments[0]) > 8192 || sourceCount <= 0 {
		return nil
	}
	for _, segment := range segments[0] {
		if err := budget.Step(); err != nil {
			return err
		}
		if segment < 0 || segment >= sourceCount {
			return nil
		}
	}
	if err := budget.Err(); err != nil {
		return err
	}
	s.cellSegments = slices.Clone(segments[0])
	return nil
}

// DirectCellSourceSegment reads the original outer-loop cell's authenticated
// first-section segment. It does not prove that any mesh facet is that cell.
func (p Payload) DirectCellSourceSegment(body *topo.Body, loop, cell int) (int, bool) {
	if body == nil || p.SourceIdentity == nil || p.SourceIdentity.Origin != body.Origin.Producer ||
		!p.MatchesDirectSections(body) || loop != 0 {
		return 0, false
	}
	segments := p.SourceIdentity.direct.cellSegments
	if cell < 0 || cell >= len(segments) {
		return 0, false
	}
	return segments[cell], true
}

// sealDirectFaces binds the builder's ordered source faces before publication.
func sealDirectFaces(s *directSections, body *topo.Body, caps [2]*topo.Face, walls []*topo.Face,
	cells [][2]int, sides []uint8, verts []r3.Vec, tris [][3]int, capStartCount int, budget *proofbound.WorkBudget) error {
	if capStartCount <= 0 || len(tris) < len(walls) || capStartCount > len(tris)-len(walls) {
		return nil
	}
	endCount := len(tris) - len(walls) - capStartCount
	if s == nil || s.body != body || len(s.cellSegments) == 0 || s.wallFaces != nil ||
		len(walls) != 2*len(s.cellSegments) || len(cells) != len(walls) || len(sides) != len(walls) ||
		capStartCount > len(s.cellSegments) || endCount <= 0 || endCount > len(s.cellSegments) ||
		caps[0] == nil || caps[1] == nil || caps[0] == caps[1] || caps[0].Body != body || caps[1].Body != body {
		return nil
	}
	seen := map[*topo.Face]struct{}{caps[0]: {}, caps[1]: {}}
	triangles := make([][3]r3.Vec, len(walls))
	for i, face := range walls {
		if err := budget.Step(); err != nil {
			return err
		}
		if face == nil || face.Body != body || cells[i] != [2]int{0, i / 2} || sides[i] != uint8(i%2) {
			return nil
		}
		if _, exists := seen[face]; exists {
			return nil
		}
		for j, index := range tris[i] {
			if index < 0 || index >= len(verts) || !proofbound.FiniteVec(verts[index]) {
				return nil
			}
			triangles[i][j] = verts[index]
		}
		if !directFaceTriangle(face, triangles[i]) {
			return nil
		}
		seen[face] = struct{}{}
	}
	var capTriangles [2][][3]r3.Vec
	for section, bounds := range [2][2]int{{len(walls), len(walls) + capStartCount}, {len(walls) + capStartCount, len(tris)}} {
		capTriangles[section] = make([][3]r3.Vec, bounds[1]-bounds[0])
		for i := bounds[0]; i < bounds[1]; i++ {
			if err := budget.Step(); err != nil {
				return err
			}
			for j, index := range tris[i] {
				if index < 0 || index >= len(verts) || !proofbound.FiniteVec(verts[index]) {
					return nil
				}
				capTriangles[section][i-bounds[0]][j] = verts[index]
			}
			tri := capTriangles[section][i-bounds[0]]
			if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
				return nil
			}
		}
	}
	if err := budget.Err(); err != nil {
		return err
	}
	s.capFaces, s.wallFaces, s.wallTriangles, s.capTriangles = caps, slices.Clone(walls), triangles, capTriangles
	return nil
}

func directFaceTriangle(face *topo.Face, triangle [3]r3.Vec) bool {
	if len(face.Loops) != 1 || face.Loops[0] == nil || len(face.Loops[0].CoEdges) != 3 ||
		triangle[0] == triangle[1] || triangle[1] == triangle[2] || triangle[2] == triangle[0] {
		return false
	}
	var starts, ends [3]*topo.Vertex
	for i, ce := range face.Loops[0].CoEdges {
		if ce.Edge == nil || ce.Edge.Start == nil || ce.Edge.End == nil {
			return false
		}
		starts[i], ends[i] = ce.Edge.Start, ce.Edge.End
		if !ce.Forward {
			starts[i], ends[i] = ends[i], starts[i]
		}
	}
	var used [3]bool
	for i, start := range starts {
		if ends[i] != starts[(i+1)%3] {
			return false
		}
		found := false
		for j, point := range triangle {
			if !used[j] && start.Position == point {
				used[j] = true
				found = true
				break
			}
		}
		if !found {
			return false
		}
	}
	return true
}

// DirectWallFaceSourceSegment binds an original wall face to its cell and
// first-section source segment. It does not identify a mesh facet.
func (p Payload) DirectWallFaceSourceSegment(body *topo.Body, face *topo.Face, cell, side int) (int, bool) {
	segment, ok := p.DirectCellSourceSegment(body, 0, cell)
	if !ok || face == nil || side < 0 || side > 1 {
		return 0, false
	}
	walls := p.SourceIdentity.direct.wallFaces
	i := 2*cell + side
	if i < 0 || i >= len(walls) || walls[i] != face || face.Body != body {
		return 0, false
	}
	return segment, true
}

// DirectCapFaceMatches reports whether face is the original cap of section.
func (p Payload) DirectCapFaceMatches(body *topo.Body, face *topo.Face, section int) bool {
	return body != nil && face != nil && (section == 0 || section == 1) && p.SourceIdentity != nil &&
		p.SourceIdentity.Origin == body.Origin.Producer && p.MatchesDirectSections(body) &&
		p.SourceIdentity.direct.capFaces[section] == face && face.Body == body
}

// DirectCapFacetMatches binds a verified cap facet and its prepared
// coordinates to one original cap triangle.
func (p Payload) DirectCapFacetMatches(body *topo.Body, mesh *topo.Mesh, facet, section int, prepared [3]r3.Vec) bool {
	if mesh == nil || !mesh.BoundaryOK || facet < 0 || facet >= len(mesh.Triangles) || facet >= len(mesh.Source) ||
		!p.DirectCapFaceMatches(body, mesh.Source[facet], section) {
		return false
	}
	s := p.SourceIdentity.direct
	index := facet - len(s.wallFaces)
	if section == 1 {
		index -= len(s.capTriangles[0])
	}
	if index < 0 || index >= len(s.capTriangles[section]) {
		return false
	}
	for i, v := range mesh.Triangles[facet] {
		if v < 0 || v >= len(mesh.Vertices) || mesh.Vertices[v] != s.capTriangles[section][index][i] ||
			prepared[i] != s.capTriangles[section][index][i] {
			return false
		}
	}
	return true
}

// DirectCapFacetRange reads the original builder's ordered cap-facet range.
// Callers must still verify every mesh facet against DirectCapFacetMatches.
func (p Payload) DirectCapFacetRange(body *topo.Body, section int) (int, int, bool) {
	if section < 0 || section > 1 || p.SourceIdentity == nil || p.SourceIdentity.direct == nil {
		return 0, 0, false
	}
	s := p.SourceIdentity.direct
	if !p.DirectCapFaceMatches(body, s.capFaces[section], section) || len(s.capTriangles[section]) == 0 {
		return 0, 0, false
	}
	start := len(s.wallFaces)
	if section == 1 {
		start += len(s.capTriangles[0])
	}
	return start, start + len(s.capTriangles[section]), true
}

// DirectWallFacetSourceSegment binds a verified mesh facet and its prepared
// coordinates to one original wall face, cell, side and section segment.
func (p Payload) DirectWallFacetSourceSegment(body *topo.Body, mesh *topo.Mesh, facet, cell, side int,
	prepared [3]r3.Vec) (int, bool) {
	if mesh == nil || !mesh.BoundaryOK || facet < 0 || facet >= len(mesh.Triangles) || facet >= len(mesh.Source) {
		return 0, false
	}
	segment, ok := p.DirectWallFaceSourceSegment(body, mesh.Source[facet], cell, side)
	if !ok {
		return 0, false
	}
	index := 2*cell + side
	triangles := p.SourceIdentity.direct.wallTriangles
	if index < 0 || index >= len(triangles) {
		return 0, false
	}
	for i, v := range mesh.Triangles[facet] {
		if v < 0 || v >= len(mesh.Vertices) || mesh.Vertices[v] != triangles[index][i] || prepared[i] != triangles[index][i] {
			return 0, false
		}
	}
	return segment, true
}
