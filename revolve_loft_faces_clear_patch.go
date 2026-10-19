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
		vertexBound: slices.Clone(p.VertexBound), src: slices.Clone(p.Src),
		faceOf: slices.Clone(p.FaceOf), faces: tooth.Faces(),
		meshBound: p.MeshBound, volSymDiff: p.VolSymDiff, areaSlack: p.AreaSlack,
	}
}

func (s *revolveLoftFacesToolPayloadSnapshot) current(tooth *Body,
	p facetedPayload) bool {
	return s != nil && tooth != nil && slices.Equal(s.verts, p.Verts) &&
		slices.Equal(s.tris, p.Tris) && slices.Equal(s.vertexBound, p.VertexBound) &&
		slices.Equal(s.src, p.Src) && slices.Equal(s.faceOf, p.FaceOf) &&
		slices.Equal(s.faces, tooth.Faces()) && s.meshBound == p.MeshBound &&
		s.volSymDiff == p.VolSymDiff && s.areaSlack == p.AreaSlack
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
	cloned.ShellMesh = shell.Clone()
	if len(cloned.Seams) != 0 {
		cloned.seam = cloned.Seams[0]
	}
	return &cloned
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
	fresh, err := tessellateFaceted(ctx, source.tooth, payload, payload.MeshBound)
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
		c.shellProof == nil || !shell.SameMesh(&c.shellProof.ShellMesh) ||
		!sameRevolveLoftFacesProofMesh(tool, c.toolProof) ||
		!revolveLoftFacesSectionCurrent(source) ||
		!revolveLoftFacesTrimCurrent(source.tooth, source.trim) ||
		!revolveLoftFacesSourceRefsCurrent(source.trim, source.tooth, source.sourceRefs) ||
		!tool.n.BoundaryOK || !tool.n.SymDiffOK ||
		proofbound.IsNonFinite(tool.n.Bound) || tool.n.Bound < 0 ||
		proofbound.IsNonFinite(tool.n.VolSymDiff) || tool.n.VolSymDiff < 0 ||
		index < 0 || index >= len(shell.Mesh.Tris) ||
		len(shell.TriangleCells) != len(shell.Mesh.Tris) ||
		len(shell.Mesh.Src) != len(shell.Mesh.Tris) ||
		len(shell.VertexBounds) != len(shell.Mesh.Verts) ||
		proofbound.IsNonFinite(shell.FaceBound) || shell.FaceBound < shell.ChordBound {
		return pointquery.Undecided, false, nil
	}
	cell := shell.TriangleCells[index]
	if cell.Section < 0 || cell.Section >= len(source.section.Spans) ||
		cell.Section >= len(shell.SectionRings) ||
		source.section.Spans[cell.Section].Face == nil ||
		cell.Band < 0 || cell.Band+1 >= len(shell.SectionRings[cell.Section]) ||
		cell.Ray < 0 || cell.Ray >= shell.RayCount {
		return pointquery.Undecided, false, nil
	}
	group := shell.Mesh.Src[index]
	if group < 0 || group >= len(shell.Mesh.Groups) ||
		!shell.SectionGroup(source.section.Spans, cell.Section, group) {
		return pointquery.Undecided, false, nil
	}
	tri := shell.Mesh.Tris[index]
	for _, vertex := range tri {
		if vertex < 0 || vertex >= len(shell.Mesh.Verts) ||
			!proofbound.FiniteVec(shell.Mesh.Verts[vertex]) ||
			proofbound.IsNonFinite(shell.VertexBounds[vertex]) ||
			shell.VertexBounds[vertex] < 0 {
			return pointquery.Undecided, false, nil
		}
	}
	if !shell.CellMatches(tri, cell) {
		return pointquery.Undecided, false, nil
	}
	required := proofbound.AbsSumUpper(shell.ChordBound,
		max(shell.VertexBounds[tri[0]], shell.VertexBounds[tri[1]], shell.VertexBounds[tri[2]]))
	if proofbound.IsNonFinite(required) || shell.FaceBound < required {
		return pointquery.Undecided, false, nil
	}
	separation := proofbound.AbsSumUpper(shell.FaceBound, tool.n.Bound)
	if proofbound.IsNonFinite(separation) || separation <= 0 {
		return pointquery.Undecided, false, nil
	}
	held := [3]proof.Xpt{
		proof.XptOf(shell.Mesh.Verts[tri[0]]),
		proof.XptOf(shell.Mesh.Verts[tri[1]]),
		proof.XptOf(shell.Mesh.Verts[tri[2]]),
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
	if meshbool.PointRoundBound(centroid, point) >= shell.FaceBound {
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
