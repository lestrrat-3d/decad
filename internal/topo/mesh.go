package topo

import (
	"github.com/lestrrat-3d/r3"
)

// Mesh is the data of the root Mesh handle, which holds one by
// value.
type Mesh struct {
	// Vertices are the mesh vertex positions in millimetres.
	Vertices []r3.Vec
	// Triangles are the facets, as index triples into Vertices.
	Triangles [][3]int
	// Bound is the proven displacement of the mesh from the true boundary, in millimetres.
	Bound float64
	// CoordBound is the largest stored-vertex displacement from construction
	// and placement, excluding curve chording. Composite sweep contact proofs
	// read it when they reuse prism and revolve span meshes.
	CoordBound float64
	// VertexBound is agent-docs/faceted-vertex-bounds-design.md §2's per-vertex
	// record β(v), one entry per vertex in millimetres: a point of the true
	// boundary lies within VertexBound[v] of Vertices[v], and every facet's
	// true piece lies within the largest of its three corners' entries of the
	// facet. nil means the payload publishes no record; topobuild.VertexBounds
	// then derives one from the face bounds (§2.1). When present, every face-bound entry
	// is the corner maximum over that face's own triangles.
	VertexBound []float64
	// AreaSlack is a proven bound (mm²) on how far the mesh's total facet
	// area falls short of — or, over a hole, overshoots — the body's true
	// boundary area: the chord-versus-arc deficit, closed form per walk, plus
	// the per-facet allowance every coordinate the build itself computed costs.
	// The mesh boolean's area bounds compose from it.
	AreaSlack float64
	// VolSymDiff bounds volume(TrueBody △ MeshSolid) — occupied volume, never
	// signed volume, so no term of it may cancel another
	// (agent-docs/tessellation-design.md §2). It is meaningful ONLY when SymDiffOK:
	// a payload class whose occupied-volume proof has not landed publishes a
	// mesh for export with SymDiffOK false, and the mesh boolean refuses that
	// operand rather than substituting bound × held area, which
	// agent-docs/tessellation-design.md §11 forbids outright. A [BodySheet] mesh is a
	// DIFFERENT reason for the same false: it is a body KIND that encloses no
	// region, not a staged payload class waiting on a proof — there is no
	// occupied volume for any future increment to bound
	// (agent-docs/surface-design.md §10), so SymDiffOK stays false permanently and
	// VolSymDiff stays zero.
	VolSymDiff float64
	// SymDiffOK reports whether VolSymDiff is meaningful.
	SymDiffOK bool
	// BoundaryOK is agent-docs/tessellation-design.md §1's Embedding row: every
	// boundary audit this body's payload supports ran and passed, the
	// facet-contact audit included. It is false exactly when VerifyNone was
	// asked of a payload that carries such an audit — revolve and its curved
	// stitched reuse today
	// (root payloadAuditsFacetContact) — and true everywhere else, since every
	// other boundary audit is linear and runs at every level.
	// [Mesh.BoundaryVerified] publishes it; root tessellateContext is the one
	// writer.
	BoundaryOK bool
	// Source holds, parallel to Triangles, the node of the face each facet
	// approximates. Root Mesh.SourceFaces maps them back to their handles.
	Source []*Face
	// FaceBound is agent-docs/tessellation-design.md §2's sourceBound(face): the
	// two-sided displacement between the true trimmed face patch and the facets
	// held for it, one entry for EVERY face appearing in Source, keyed by the
	// face node. Bound is the maximum over it, so a mesh never publishes a
	// global figure no face accounts for, and the boolean's hidden-tangency
	// pre-pass charges each face what its own construction cost rather than
	// inferring a displacement from the surface kind. A source face missing
	// from this map is an evaluator invariant failure, never a zero: zero is
	// the CLAIM that the held polygon is the true trimmed patch and that its
	// stored coordinates add nothing.
	FaceBound map[*Face]float64
	// ReplayFaceBounds holds private replay bounds without a runtime map.
	ReplayFaceBounds *FaceBoundTable
	// Owner is the root Mesh handle that holds this node by value. Root sets
	// it when the handle is born and maps a node back to its handle through
	// it; nothing else writes it.
	Owner any
}

// TessEntry is one cached tessellation of a body (root Body.Tessellate): the
// mesh node built at the chord tolerance whose float64 bits are ChordBits,
// at verification level Verify (root Verification's value). Root maps Mesh
// back to its handle on a hit, so a second Tessellate returns the same *Mesh.
type TessEntry struct {
	ChordBits uint64
	// Verify is the level the cached mesh was BUILT at, and half of the entry's
	// key (agent-docs/tessellation-design.md §1.1). Keying on the tolerance alone
	// would hand a VerifyNone mesh back to a boolean whose own internal
	// tolerance happened to match, and the boolean's gates are stated over a
	// mesh it believes was proven.
	Verify int
	Mesh   *Mesh
}
