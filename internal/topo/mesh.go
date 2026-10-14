package topo

import (
	"github.com/lestrrat-3d/r3"
)

// Mesh is the data of the root Mesh handle, which holds one by
// value. The handle keeps the source faces and per-face bounds, which name
// root types.
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
	// VertexBound is docs/faceted-vertex-bounds-design.md §2's per-vertex
	// record β(v), one entry per vertex in millimetres: a point of the true
	// boundary lies within VertexBound[v] of Vertices[v], and every facet's
	// true piece lies within the largest of its three corners' entries of the
	// facet. nil means the payload publishes no record; vertexBounds then
	// derives one from the root Mesh's face bounds (§2.1). When present, every face-bound entry
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
	// (docs/tessellation-design.md §2). It is meaningful ONLY when SymDiffOK:
	// a payload class whose occupied-volume proof has not landed publishes a
	// mesh for export with SymDiffOK false, and the mesh boolean refuses that
	// operand rather than substituting bound × held area, which
	// docs/tessellation-design.md §11 forbids outright. A [BodySheet] mesh is a
	// DIFFERENT reason for the same false: it is a body KIND that encloses no
	// region, not a staged payload class waiting on a proof — there is no
	// occupied volume for any future increment to bound
	// (docs/surface-design.md §10), so SymDiffOK stays false permanently and
	// VolSymDiff stays zero.
	VolSymDiff float64
	// SymDiffOK reports whether VolSymDiff is meaningful.
	SymDiffOK bool
	// BoundaryOK is docs/tessellation-design.md §1's Embedding row: every
	// boundary audit this body's payload supports ran and passed, the
	// facet-contact audit included. It is false exactly when VerifyNone was
	// asked of a payload that carries such an audit — revolve and its curved
	// stitched reuse today
	// (root payloadAuditsFacetContact) — and true everywhere else, since every
	// other boundary audit is linear and runs at every level.
	// [Mesh.BoundaryVerified] publishes it; root tessellateContext is the one
	// writer.
	BoundaryOK bool
}
