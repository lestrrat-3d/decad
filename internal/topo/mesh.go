package topo

import (
	"github.com/lestrrat-3d/r3"
)

// Mesh is the data of the root Mesh handle, which holds one by
// value.
type Mesh struct {
	Vertices  []r3.Vec
	Triangles [][3]int
	Bound     float64 // millimetres
	// CoordBound is the largest stored-vertex displacement from construction
	// and placement, excluding curve chording. Composite sweep contact proofs
	// read it when they reuse prism and revolve span meshes.
	CoordBound float64
	// VertexBound is docs/faceted-vertex-bounds-design.md §2's per-vertex
	// record β(v), one entry per vertex in millimetres: a point of the true
	// boundary lies within vertexBound[v] of vertices[v], and every facet's
	// true piece lies within the largest of its three corners' entries of the
	// facet. nil means the payload publishes no record; vertexBounds then
	// derives one from faceBound (§2.1). When present, every faceBound entry
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
	// (docs/tessellation-design.md §2). It is meaningful ONLY when symDiffOK:
	// a payload class whose occupied-volume proof has not landed publishes a
	// mesh for export with symDiffOK false, and the mesh boolean refuses that
	// operand rather than substituting bound × held area, which
	// docs/tessellation-design.md §11 forbids outright. A [BodySheet] mesh is a
	// DIFFERENT reason for the same false: it is a body KIND that encloses no
	// region, not a staged payload class waiting on a proof — there is no
	// occupied volume for any future increment to bound
	// (docs/surface-design.md §10), so symDiffOK stays false permanently and
	// volSymDiff stays zero.
	VolSymDiff float64
	SymDiffOK  bool
	// BoundaryOK is docs/tessellation-design.md §1's Embedding row: every
	// boundary audit this body's payload supports ran and passed, the
	// facet-contact audit included. It is false exactly when VerifyNone was
	// asked of a payload that carries such an audit — revolve and its curved
	// stitched reuse today
	// (payloadAuditsFacetContact) — and true everywhere else, since every
	// other boundary audit is linear and runs at every level.
	// [Mesh.BoundaryVerified] publishes it; tessellateContext is the one
	// writer.
	BoundaryOK bool
}
