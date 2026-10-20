package revolvecontact

import (
	"context"
	"maps"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/pointquery"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
	"github.com/lestrrat-3d/r3"
)

// The source certificate fixes the faceted tool payload that existed when
// its source records were accepted. A changed payload cannot borrow that
// certificate or a cached VerifyAll mesh.
type ToolPayloadSnapshot struct {
	Verts       []r3.Vec
	Tris        [][3]int
	VertexBound []float64
	Src         []int
	FaceOf      []int
	Faces       []*topo.Face
	MeshBound   float64
	VolSymDiff  float64
	AreaSlack   float64
}

func NewToolPayloadSnapshot(tooth *topo.Body,
	p facetedrecord.Payload) *ToolPayloadSnapshot {
	return &ToolPayloadSnapshot{
		Verts: slices.Clone(p.Verts), Tris: slices.Clone(p.Tris),
		VertexBound: slices.Clone(p.VertexBound), Src: slices.Clone(p.Src),
		FaceOf: slices.Clone(p.FaceOf), Faces: topobuild.Faces(tooth),
		MeshBound: p.MeshBound, VolSymDiff: p.VolSymDiff, AreaSlack: p.AreaSlack,
	}
}

func (s *ToolPayloadSnapshot) current(tooth *topo.Body,
	p facetedrecord.Payload) bool {
	return s != nil && tooth != nil && slices.Equal(s.Verts, p.Verts) &&
		slices.Equal(s.Tris, p.Tris) && slices.Equal(s.VertexBound, p.VertexBound) &&
		slices.Equal(s.Src, p.Src) && slices.Equal(s.FaceOf, p.FaceOf) &&
		slices.Equal(s.Faces, topobuild.Faces(tooth)) && s.MeshBound == p.MeshBound &&
		s.VolSymDiff == p.VolSymDiff && s.AreaSlack == p.AreaSlack
}

func CloneProofMesh(m *topo.Mesh) *topo.Mesh {
	node := *m
	cloned := &node
	cloned.Owner = nil
	cloned.Vertices = slices.Clone(m.Vertices)
	cloned.Triangles = slices.Clone(m.Triangles)
	cloned.Source = slices.Clone(m.Source)
	cloned.VertexBound = slices.Clone(m.VertexBound)
	cloned.FaceBound = maps.Clone(m.FaceBound)
	return cloned
}

func SameProofMesh(a, b *topo.Mesh) bool {
	return a != nil && b != nil && slices.Equal(a.Vertices, b.Vertices) &&
		slices.Equal(a.Triangles, b.Triangles) && slices.Equal(a.Source, b.Source) &&
		slices.Equal(a.VertexBound, b.VertexBound) && maps.Equal(a.FaceBound, b.FaceBound) &&
		a.Bound == b.Bound && a.CoordBound == b.CoordBound &&
		a.AreaSlack == b.AreaSlack && a.VolSymDiff == b.VolSymDiff &&
		a.SymDiffOK == b.SymDiffOK && a.BoundaryOK == b.BoundaryOK
}

func CloneClearShell(shell *BlankSeamShell) *BlankSeamShell {
	cloned := *shell
	cloned.ShellMesh = shell.Clone()
	if len(cloned.Seams) != 0 {
		cloned.Seam = cloned.Seams[0]
	}
	return &cloned
}

// This private context owns the real producers consumed by every candidate
// triangle. Construction checks the complete contact graph and shell once.
type ClearPatchContext struct {
	Source     *SourceCertificate
	Shell      *BlankSeamShell
	Tool       *topo.Mesh
	ShellProof *BlankSeamShell
	ToolProof  *topo.Mesh
}

func NewClearPatchContext(ctx context.Context,
	mesher verifiedmesh.Mesher, source *SourceCertificate, chordMM float64,
) (*ClearPatchContext, bool, error) {
	if ctx == nil || source == nil || source.Tooth == nil {
		return nil, false, nil
	}
	payload, ok := source.Tooth.Payload.(facetedrecord.Payload)
	if !ok || !source.ToolProof.current(source.Tooth, payload) {
		return nil, false, nil
	}
	tool, err := mesher.VerifyAll(ctx, source.Tooth, chordMM)
	if err != nil || tool == nil || !tool.BoundaryOK || !tool.SymDiffOK ||
		proofbound.IsNonFinite(tool.Bound) || tool.Bound < 0 ||
		proofbound.IsNonFinite(tool.VolSymDiff) || tool.VolSymDiff < 0 {
		return nil, false, err
	}
	freshNode, err := facetedrecord.Restate(ctx, source.Tooth, payload, payload.MeshBound)
	var fresh *topo.Mesh
	if freshNode != nil {
		copied := *freshNode
		fresh = &copied
		fresh.Owner = nil
		// The verified tessellation sets this flag after the payload restatement.
		fresh.BoundaryOK = true
	}
	if err != nil || !SameProofMesh(tool, fresh) ||
		!source.ToolProof.current(source.Tooth, payload) {
		return nil, false, err
	}
	shell, _, ok, err := BlankShellWithAllRuns(
		WithCallScope(ctx), source)
	if err != nil || !ok || !source.ToolProof.current(source.Tooth, payload) {
		return nil, false, err
	}
	ownedShell := CloneClearShell(shell)
	ownedTool := CloneProofMesh(tool)
	return &ClearPatchContext{
		Source: source, Shell: ownedShell, Tool: ownedTool,
		ShellProof: CloneClearShell(ownedShell),
		ToolProof:  CloneProofMesh(ownedTool),
	}, true, nil
}

func (c *ClearPatchContext) Classify(ctx context.Context,
	index int) (pointquery.Location, bool, error) {
	if c == nil {
		return pointquery.Undecided, false, nil
	}
	return c.classifyClearPatch(ctx, index)
}

// A clear-patch certificate transfers one point's tool occupancy to an
// entire audited blank-shell triangle only after every tool facet is proved
// farther away than the two complete boundary departure bounds.
func (c *ClearPatchContext) classifyClearPatch(ctx context.Context,
	index int) (pointquery.Location, bool, error) {
	source, shell, tool := c.Source, c.Shell, c.Tool
	if ctx == nil || source == nil || shell == nil || tool == nil ||
		source.Tooth == nil || source.Trim == nil ||
		!ClearPatchToolCurrent(source) ||
		c.ShellProof == nil || !shell.SameMesh(&c.ShellProof.ShellMesh) ||
		!SameProofMesh(tool, c.ToolProof) ||
		!SectionCurrent(source) ||
		!TrimCurrent(source.Tooth, source.Trim) ||
		!SourceRefsCurrent(source.Trim, source.Tooth, source.SourceRefs) ||
		!tool.BoundaryOK || !tool.SymDiffOK ||
		proofbound.IsNonFinite(tool.Bound) || tool.Bound < 0 ||
		proofbound.IsNonFinite(tool.VolSymDiff) || tool.VolSymDiff < 0 ||
		index < 0 || index >= len(shell.Mesh.Tris) ||
		len(shell.TriangleCells) != len(shell.Mesh.Tris) ||
		len(shell.Mesh.Src) != len(shell.Mesh.Tris) ||
		len(shell.VertexBounds) != len(shell.Mesh.Verts) ||
		proofbound.IsNonFinite(shell.FaceBound) || shell.FaceBound < shell.ChordBound {
		return pointquery.Undecided, false, nil
	}
	cell := shell.TriangleCells[index]
	if cell.Section < 0 || cell.Section >= len(source.Section.Spans) ||
		cell.Section >= len(shell.SectionRings) ||
		source.Section.Spans[cell.Section].Face == nil ||
		cell.Band < 0 || cell.Band+1 >= len(shell.SectionRings[cell.Section]) ||
		cell.Ray < 0 || cell.Ray >= shell.RayCount {
		return pointquery.Undecided, false, nil
	}
	group := shell.Mesh.Src[index]
	if group < 0 || group >= len(shell.Mesh.Groups) ||
		!shell.SectionGroup(source.Section.Spans, cell.Section, group) {
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
	separation := proofbound.AbsSumUpper(shell.FaceBound, tool.Bound)
	if proofbound.IsNonFinite(separation) || separation <= 0 {
		return pointquery.Undecided, false, nil
	}
	held := [3]proof.Xpt{
		proof.XptOf(shell.Mesh.Verts[tri[0]]),
		proof.XptOf(shell.Mesh.Verts[tri[1]]),
		proof.XptOf(shell.Mesh.Verts[tri[2]]),
	}
	for _, facet := range tool.Triangles {
		if err := ctx.Err(); err != nil {
			return pointquery.Undecided, false, err
		}
		var other [3]proof.Xpt
		for i, vertex := range facet {
			if vertex < 0 || vertex >= len(tool.Vertices) || !proofbound.FiniteVec(tool.Vertices[vertex]) {
				return pointquery.Undecided, false, nil
			}
			other[i] = proof.XptOf(tool.Vertices[vertex])
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
	location, err := pointquery.Locate(ctx, point, tool.Vertices, tool.Triangles,
		tool.Bound, tool.VolSymDiff)
	if err != nil || location != pointquery.Inside && location != pointquery.Outside {
		return pointquery.Undecided, false, err
	}
	return location, true, nil
}

func ClearPatchToolCurrent(source *SourceCertificate) bool {
	if source == nil || source.Tooth == nil {
		return false
	}
	payload, ok := source.Tooth.Payload.(facetedrecord.Payload)
	return ok && source.ToolProof.current(source.Tooth, payload)
}
