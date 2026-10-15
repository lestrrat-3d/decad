package decad

import (
	"context"
	"maps"
	"slices"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/pointquery"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// The source certificate fixes the faceted tool payload that existed when
// its source records were accepted. A changed payload cannot borrow that
// certificate or a cached VerifyAll mesh.
type revolveLoftFacesToolPayloadSnapshot struct {
	verts       []r3.Vec
	tris        [][3]int
	vertexBound []float64
	src         []int
	faceOf      []int
	faces       []*Face
	meshBound   float64
	volSymDiff  float64
	areaSlack   float64
}

func newRevolveLoftFacesToolPayloadSnapshot(tooth *Body,
	p facetedPayload) *revolveLoftFacesToolPayloadSnapshot {
	return &revolveLoftFacesToolPayloadSnapshot{
		verts: slices.Clone(p.Verts), tris: slices.Clone(p.Tris),
		vertexBound: slices.Clone(p.vertexBound), src: slices.Clone(p.Src),
		faceOf: slices.Clone(p.FaceOf), faces: tooth.Faces(),
		meshBound: p.meshBound, volSymDiff: p.volSymDiff, areaSlack: p.areaSlack,
	}
}

func (s *revolveLoftFacesToolPayloadSnapshot) current(tooth *Body,
	p facetedPayload) bool {
	return s != nil && tooth != nil && slices.Equal(s.verts, p.Verts) &&
		slices.Equal(s.tris, p.Tris) && slices.Equal(s.vertexBound, p.vertexBound) &&
		slices.Equal(s.src, p.Src) && slices.Equal(s.faceOf, p.FaceOf) &&
		slices.Equal(s.faces, tooth.Faces()) && s.meshBound == p.meshBound &&
		s.volSymDiff == p.volSymDiff && s.areaSlack == p.areaSlack
}

func cloneRevolveLoftFacesProofMesh(m *Mesh) *Mesh {
	cloned := bornMesh(&Mesh{n: m.n})
	cloned.n.Vertices = slices.Clone(m.n.Vertices)
	cloned.n.Triangles = slices.Clone(m.n.Triangles)
	cloned.n.Source = slices.Clone(m.n.Source)
	cloned.n.VertexBound = slices.Clone(m.n.VertexBound)
	cloned.n.FaceBound = maps.Clone(m.n.FaceBound)
	return cloned
}

func sameRevolveLoftFacesProofMesh(a, b *Mesh) bool {
	return a != nil && b != nil && slices.Equal(a.n.Vertices, b.n.Vertices) &&
		slices.Equal(a.n.Triangles, b.n.Triangles) && slices.Equal(a.n.Source, b.n.Source) &&
		slices.Equal(a.n.VertexBound, b.n.VertexBound) && maps.Equal(a.n.FaceBound, b.n.FaceBound) &&
		a.n.Bound == b.n.Bound && a.n.CoordBound == b.n.CoordBound &&
		a.n.AreaSlack == b.n.AreaSlack && a.n.VolSymDiff == b.n.VolSymDiff &&
		a.n.SymDiffOK == b.n.SymDiffOK && a.n.BoundaryOK == b.n.BoundaryOK
}

func cloneRevolveLoftFacesClearShell(shell *revolveLoftFacesBlankSeamShell) *revolveLoftFacesBlankSeamShell {
	cloned := *shell
	cloned.mesh.verts = slices.Clone(shell.mesh.verts)
	cloned.mesh.tris = slices.Clone(shell.mesh.tris)
	cloned.mesh.src = slices.Clone(shell.mesh.src)
	cloned.mesh.groups = slices.Clone(shell.mesh.groups)
	for i := range cloned.mesh.groups {
		cloned.mesh.groups[i].origins = slices.Clone(shell.mesh.groups[i].origins)
	}
	cloned.vertexBounds = slices.Clone(shell.vertexBounds)
	cloned.triangleCells = slices.Clone(shell.triangleCells)
	cloned.sectionRings = make([][][]int, len(shell.sectionRings))
	for i, section := range shell.sectionRings {
		cloned.sectionRings[i] = make([][]int, len(section))
		for j, ring := range section {
			cloned.sectionRings[i][j] = slices.Clone(ring)
		}
	}
	cloned.seams = make([][]int, len(shell.seams))
	for i, seam := range shell.seams {
		cloned.seams[i] = slices.Clone(seam)
	}
	if len(cloned.seams) != 0 {
		cloned.seam = cloned.seams[0]
	}
	return &cloned
}

func sameRevolveLoftFacesClearShell(a, b *revolveLoftFacesBlankSeamShell) bool {
	if a == nil || b == nil ||
		!slices.Equal(a.mesh.verts, b.mesh.verts) ||
		!slices.Equal(a.mesh.tris, b.mesh.tris) ||
		!slices.Equal(a.mesh.src, b.mesh.src) ||
		!slices.EqualFunc(a.mesh.groups, b.mesh.groups, func(x, y facetGroup) bool {
			return slices.Equal(x.origins, y.origins)
		}) ||
		!slices.Equal(a.vertexBounds, b.vertexBounds) ||
		!slices.Equal(a.triangleCells, b.triangleCells) ||
		a.rayCount != b.rayCount || a.chordBound != b.chordBound || a.faceBound != b.faceBound ||
		len(a.sectionRings) != len(b.sectionRings) {
		return false
	}
	for i := range a.sectionRings {
		if len(a.sectionRings[i]) != len(b.sectionRings[i]) {
			return false
		}
		for j := range a.sectionRings[i] {
			if !slices.Equal(a.sectionRings[i][j], b.sectionRings[i][j]) {
				return false
			}
		}
	}
	return true
}

// This private context owns the real producers consumed by every candidate
// triangle. Construction checks the complete contact graph and shell once.
type revolveLoftFacesClearPatchContext struct {
	source     *revolveLoftFacesSourceCertificate
	shell      *revolveLoftFacesBlankSeamShell
	tool       *Mesh
	shellProof *revolveLoftFacesBlankSeamShell
	toolProof  *Mesh
}

func newRevolveLoftFacesClearPatchContext(ctx context.Context,
	source *revolveLoftFacesSourceCertificate, chord units.Value,
) (*revolveLoftFacesClearPatchContext, bool, error) {
	if ctx == nil || source == nil || source.tooth == nil {
		return nil, false, nil
	}
	payload, ok := source.tooth.n.Payload.(facetedPayload)
	if !ok || !source.toolProof.current(source.tooth, payload) {
		return nil, false, nil
	}
	tool, err := tessellateContext(ctx, source.tooth, chord, VerifyAll)
	if err != nil || tool == nil || !tool.n.BoundaryOK || !tool.n.SymDiffOK ||
		proofbound.IsNonFinite(tool.n.Bound) || tool.n.Bound < 0 ||
		proofbound.IsNonFinite(tool.n.VolSymDiff) || tool.n.VolSymDiff < 0 {
		return nil, false, err
	}
	fresh, err := tessellateFaceted(ctx, source.tooth, payload, payload.meshBound)
	if fresh != nil {
		// tessellateContext sets this flag after the payload restatement.
		fresh.n.BoundaryOK = true
	}
	if err != nil || !sameRevolveLoftFacesProofMesh(tool, fresh) ||
		!source.toolProof.current(source.tooth, payload) {
		return nil, false, err
	}
	shell, _, ok, err := revolveLoftFacesBlankShellWithAllRuns(
		revolveLoftFacesWithCallScope(ctx), source)
	if err != nil || !ok || !source.toolProof.current(source.tooth, payload) {
		return nil, false, err
	}
	ownedShell := cloneRevolveLoftFacesClearShell(shell)
	ownedTool := cloneRevolveLoftFacesProofMesh(tool)
	return &revolveLoftFacesClearPatchContext{
		source: source, shell: ownedShell, tool: ownedTool,
		shellProof: cloneRevolveLoftFacesClearShell(ownedShell),
		toolProof:  cloneRevolveLoftFacesProofMesh(ownedTool),
	}, true, nil
}

func (c *revolveLoftFacesClearPatchContext) classify(ctx context.Context,
	index int) (pointquery.Location, bool, error) {
	if c == nil {
		return pointquery.Undecided, false, nil
	}
	return c.classifyClearPatch(ctx, index)
}

// A clear-patch certificate transfers one point's tool occupancy to an
// entire audited blank-shell triangle only after every tool facet is proved
// farther away than the two complete boundary departure bounds.
func (c *revolveLoftFacesClearPatchContext) classifyClearPatch(ctx context.Context,
	index int) (pointquery.Location, bool, error) {
	source, shell, tool := c.source, c.shell, c.tool
	if ctx == nil || source == nil || shell == nil || tool == nil ||
		source.tooth == nil || source.trim == nil ||
		!revolveLoftFacesClearPatchToolCurrent(source) ||
		!sameRevolveLoftFacesClearShell(shell, c.shellProof) ||
		!sameRevolveLoftFacesProofMesh(tool, c.toolProof) ||
		!revolveLoftFacesSectionCurrent(source) ||
		!revolveLoftFacesTrimCurrent(source.tooth, source.trim) ||
		!revolveLoftFacesSourceRefsCurrent(source.trim, source.tooth, source.sourceRefs) ||
		!tool.n.BoundaryOK || !tool.n.SymDiffOK ||
		proofbound.IsNonFinite(tool.n.Bound) || tool.n.Bound < 0 ||
		proofbound.IsNonFinite(tool.n.VolSymDiff) || tool.n.VolSymDiff < 0 ||
		index < 0 || index >= len(shell.mesh.tris) ||
		len(shell.triangleCells) != len(shell.mesh.tris) ||
		len(shell.mesh.src) != len(shell.mesh.tris) ||
		len(shell.vertexBounds) != len(shell.mesh.verts) ||
		proofbound.IsNonFinite(shell.faceBound) || shell.faceBound < shell.chordBound {
		return pointquery.Undecided, false, nil
	}
	cell := shell.triangleCells[index]
	if cell.section < 0 || cell.section >= len(source.section.spans) ||
		cell.section >= len(shell.sectionRings) ||
		source.section.spans[cell.section].face == nil ||
		cell.band < 0 || cell.band+1 >= len(shell.sectionRings[cell.section]) ||
		cell.ray < 0 || cell.ray >= shell.rayCount {
		return pointquery.Undecided, false, nil
	}
	group := shell.mesh.src[index]
	if group < 0 || group >= len(shell.mesh.groups) ||
		!sameRevolveLoftFacesShellGroup(source, shell, cell.section, group) {
		return pointquery.Undecided, false, nil
	}
	tri := shell.mesh.tris[index]
	for _, vertex := range tri {
		if vertex < 0 || vertex >= len(shell.mesh.verts) ||
			!proofbound.FiniteVec(shell.mesh.verts[vertex]) ||
			proofbound.IsNonFinite(shell.vertexBounds[vertex]) ||
			shell.vertexBounds[vertex] < 0 {
			return pointquery.Undecided, false, nil
		}
	}
	want, valid := revolveLoftFacesShellCellTriangle(shell, cell)
	if !valid || !sameTriangleVertices(tri, want) {
		return pointquery.Undecided, false, nil
	}
	required := proofbound.AbsSumUpper(shell.chordBound,
		max(shell.vertexBounds[tri[0]], shell.vertexBounds[tri[1]], shell.vertexBounds[tri[2]]))
	if proofbound.IsNonFinite(required) || shell.faceBound < required {
		return pointquery.Undecided, false, nil
	}
	separation := proofbound.AbsSumUpper(shell.faceBound, tool.n.Bound)
	if proofbound.IsNonFinite(separation) || separation <= 0 {
		return pointquery.Undecided, false, nil
	}
	held := [3]proof.Xpt{
		proof.XptOf(shell.mesh.verts[tri[0]]),
		proof.XptOf(shell.mesh.verts[tri[1]]),
		proof.XptOf(shell.mesh.verts[tri[2]]),
	}
	for _, facet := range tool.n.Triangles {
		if err := ctx.Err(); err != nil {
			return pointquery.Undecided, false, err
		}
		var other [3]proof.Xpt
		for i, vertex := range facet {
			if vertex < 0 || vertex >= len(tool.n.Vertices) || !proofbound.FiniteVec(tool.n.Vertices[vertex]) {
				return pointquery.Undecided, false, nil
			}
			other[i] = proof.XptOf(tool.n.Vertices[vertex])
		}
		if !meshbool.DisjointTriDistanceAbove(held, other, separation) {
			return pointquery.Undecided, false, nil
		}
	}
	centroid := meshbool.XCentroid(held[0], held[1], held[2])
	point := centroid.Vec()
	if !proofbound.FiniteVec(point) {
		return pointquery.Undecided, false, nil
	}
	// The rounded query lies within the proved separation margin of the
	// exact interior centroid; its displacement is charged explicitly.
	if meshbool.PointRoundBound(centroid, point) >= shell.faceBound {
		return pointquery.Undecided, false, nil
	}
	location, err := pointquery.Locate(ctx, point, tool.n.Vertices, tool.n.Triangles,
		tool.n.Bound, tool.n.VolSymDiff)
	if err != nil || location != pointquery.Inside && location != pointquery.Outside {
		return pointquery.Undecided, false, err
	}
	return location, true, nil
}

func revolveLoftFacesClearPatchToolCurrent(source *revolveLoftFacesSourceCertificate) bool {
	if source == nil || source.tooth == nil {
		return false
	}
	payload, ok := source.tooth.n.Payload.(facetedPayload)
	return ok && source.toolProof.current(source.tooth, payload)
}

func sameRevolveLoftFacesShellGroup(source *revolveLoftFacesSourceCertificate,
	shell *revolveLoftFacesBlankSeamShell, section, group int) bool {
	span := source.section.spans[section]
	if span.face == nil || group >= len(shell.mesh.groups) ||
		!slices.Equal(shell.mesh.groups[group].origins, span.origins) {
		return false
	}
	want := 0
	for i := range source.section.spans {
		if i == section {
			return group == want
		}
		if source.section.spans[i].face != nil {
			want++
		}
	}
	return false
}

func revolveLoftFacesShellCellTriangle(shell *revolveLoftFacesBlankSeamShell,
	cell revolveLoftFacesShellTriangleCell) ([3]int, bool) {
	var zero [3]int
	start := shell.sectionRings[cell.section][cell.band]
	end := shell.sectionRings[cell.section][cell.band+1]
	if (len(start) != 1 && len(start) != shell.rayCount) ||
		(len(end) != 1 && len(end) != shell.rayCount) ||
		len(start) == 1 && len(end) == 1 {
		return zero, false
	}
	next := (cell.ray + 1) % shell.rayCount
	switch {
	case len(start) == 1:
		if cell.half != 0 {
			return zero, false
		}
		return [3]int{start[0], end[cell.ray], end[next]}, true
	case len(end) == 1:
		if cell.half != 0 {
			return zero, false
		}
		return [3]int{start[cell.ray], end[0], start[next]}, true
	case cell.half == 0:
		return [3]int{start[cell.ray], end[cell.ray], end[next]}, true
	case cell.half == 1:
		return [3]int{start[cell.ray], end[next], start[next]}, true
	default:
		return zero, false
	}
}

func sameTriangleVertices(a, b [3]int) bool {
	slices.Sort(a[:])
	slices.Sort(b[:])
	return a == b
}
