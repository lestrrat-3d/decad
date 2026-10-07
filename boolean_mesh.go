package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// This file is the mesh-boolean pipeline of docs/evaluator-design.md §9: the
// two operands' tessellations meet in exact triangle/triangle intersection
// segments, each cut facet is subdivided along them (internal/meshbool/boolean_cut.go), every
// resulting piece is classified against the other solid by an exact local
// side-of-contact test (or exact ray parity for the uncut regions), the kept
// pieces are stitched by shared exact vertices, and a conforming pass plus a
// closure audit prove the result watertight by construction — an audit
// failure is an error, never a wrong mesh.

// prepBoolMesh lifts a tessellation into the exact domain and attaches its
// per-vertex and per-facet bounds. A non-finite vertex has no exact form, so
// it is rejected outright.
//
// A COLLAPSED operand facet — one whose exact normal is zero, which a rigid
// placement's own rounding can produce on an already-faceted body — is refused
// (ErrUnsupported). A collapsed facet has no plane and no interior, so every
// contact predicate in this file is blind to it: a point or tangent contact the
// other operand makes THERE would be classified by nothing at all, and the
// facet would ride silently through the boolean with its component's verdict.
// The contact is exactly what this pipeline must not miss, so the operand is
// refused rather than partly examined. Loud beats silently wrong.
func prepBoolMeshContext(ctx context.Context, m *Mesh, src []int) (*meshbool.BoolMesh, error) {
	bm := &meshbool.BoolMesh{Verts: m.vertices, Tris: m.triangles, Src: src}
	bm.Xverts = make([]proof.Xpt, len(m.vertices))
	for i, v := range m.vertices {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		if proofbound.IsNonFinite(v.X) || proofbound.IsNonFinite(v.Y) || proofbound.IsNonFinite(v.Z) {
			return nil, fmt.Errorf(`%w: a mesh vertex is not finite`, ErrBooleanFailed)
		}
		bm.Xverts[i] = proof.XptOf(v)
	}
	bm.Norms = make([]proof.Xpt, len(m.triangles))
	bm.Fnorms = make([]r3.Vec, len(m.triangles))
	bm.FnormsReady = make([]bool, len(m.triangles))
	bm.Boxes = make([][2]r3.Vec, len(m.triangles))
	bm.Owner = make(map[[2]int]int, 3*len(m.triangles))
	for i, tri := range m.triangles {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		a, b, c := bm.Xverts[tri[0]], bm.Xverts[tri[1]], bm.Xverts[tri[2]]
		n := proof.Xcross(proof.Xsub(b, a), proof.Xsub(c, a))
		if n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0 {
			return nil, fmt.Errorf(`%w: an operand holds a collapsed facet, which carries no plane and no interior — a contact made on it could not be classified at all, so this evaluator refuses the operand rather than examine it in part`, ErrUnsupported)
		}
		bm.Norms[i] = n
		bm.Boxes[i] = meshbool.TriBox(bm.Verts, tri)
		for k := range 3 {
			bm.Owner[[2]int{tri[k], tri[(k+1)%3]}] = i
		}
	}
	// Per-vertex bounds and their facet maxima (docs/faceted-vertex-bounds-
	// design.md §2, §4.5): the mesh's own record, or §2.1's reading derived
	// from its face bounds.
	beta, err := m.vertexBounds()
	if err != nil {
		return nil, err
	}
	bm.VertexBound = beta
	bm.FacetBound = make([]float64, len(m.triangles))
	for i, tri := range m.triangles {
		bm.FacetBound[i] = max(beta[tri[0]], beta[tri[1]], beta[tri[2]])
	}
	// The operand is now proven liftable, so its parity queries may share one
	// projection cache. This allocates the holder alone: no vertex is projected
	// until a query actually sweeps an axis, so an operand nothing probes pays
	// nothing.
	bm.Parity = meshbool.NewParityMesh(bm.Verts, bm.Tris)
	return bm, nil
}
