package meshbool

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PrepContext lifts a tessellation into the exact domain: each vertex to its
// rational form, each facet's exact normal and box, and the directed-edge
// owner map. It leaves VertexBound and FacetBound unset; SetVertexBounds
// attaches them.
//
// A non-finite vertex has no exact form, so it is rejected outright with
// ErrBooleanFailed. A COLLAPSED facet — one whose exact normal is zero, which
// a rigid placement's own rounding can produce on an already-faceted body —
// is refused with ErrUnsupported: a collapsed facet has no plane and no
// interior, so every contact predicate is blind to it, and a contact the other
// operand makes there would be classified by nothing at all. Loud beats
// silently wrong.
func PrepContext(ctx context.Context, verts []r3.Vec, tris [][3]int, src []int) (*BoolMesh, error) {
	bm := &BoolMesh{Verts: verts, Tris: tris, Src: src}
	bm.Xverts = make([]proof.Xpt, len(verts))
	for i, v := range verts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		if proofbound.IsNonFinite(v.X) || proofbound.IsNonFinite(v.Y) || proofbound.IsNonFinite(v.Z) {
			return nil, fmt.Errorf(`%w: a mesh vertex is not finite`, decaderr.ErrBooleanFailed)
		}
		bm.Xverts[i] = proof.XptOf(v)
	}
	bm.Norms = make([]proof.Xpt, len(tris))
	bm.Fnorms = make([]r3.Vec, len(tris))
	bm.FnormsReady = make([]bool, len(tris))
	bm.Boxes = make([][2]r3.Vec, len(tris))
	bm.Owner = make(map[[2]int]int, 3*len(tris))
	for i, tri := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		a, b, c := proof.DyVec(bm.Verts[tri[0]]), proof.DyVec(bm.Verts[tri[1]]), proof.DyVec(bm.Verts[tri[2]])
		d := proof.DvCross(proof.DvSub(b, a), proof.DvSub(c, a))
		n := proof.Xpt(proof.XhpStripTwosOwned(proof.XhpOfDyadics(d[0], d[1], d[2])))
		if n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0 {
			return nil, fmt.Errorf(`%w: an operand holds a collapsed facet, which carries no plane and no interior — a contact made on it could not be classified at all, so this evaluator refuses the operand rather than examine it in part`, decaderr.ErrUnsupported)
		}
		bm.Norms[i] = n
		bm.Boxes[i] = TriBox(bm.Verts, tri)
		for k := range 3 {
			bm.Owner[[2]int{tri[k], tri[(k+1)%3]}] = i
		}
	}
	bm.setDepthBlocks()
	return bm, nil
}

// SetVertexBounds attaches the per-vertex bounds beta and their facet maxima
// (agent-docs/faceted-vertex-bounds-design.md §2, §4.5), then the shared parity
// holder. The operand is proven liftable by then, so its parity queries may
// share one projection cache; the holder allocates nothing per vertex until a
// query sweeps an axis.
func (bm *BoolMesh) SetVertexBounds(beta []float64) {
	bm.VertexBound = beta
	bm.FacetBound = make([]float64, len(bm.Tris))
	for i, tri := range bm.Tris {
		bm.FacetBound[i] = max(beta[tri[0]], beta[tri[1]], beta[tri[2]])
	}
	bm.SlabBound = append([]float64(nil), bm.FacetBound...)
	bm.Parity = NewParityMesh(bm.Verts, bm.Tris)
	bm.Parity.exactVerts = bm.Xverts
	bm.Parity.exactNorms = bm.Norms
}

// SourceIDs maps a tessellation's per-facet source faces to the global group
// ids in faceID. A source absent from faceID is an ErrBooleanFailed.
func SourceIDs[F comparable](ctx context.Context, source []F, faceID map[F]int) ([]int, error) {
	out := make([]int, len(source))
	for i, f := range source {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		id, ok := faceID[f]
		if !ok {
			return nil, fmt.Errorf(`%w: a facet's source is not an operand face`, decaderr.ErrBooleanFailed)
		}
		out[i] = id
	}
	return out, nil
}

// OperandSymDiff returns the volume of the symmetric difference between an
// operand's held tessellation and the body it stands for, from the mesh's own
// proof record, or ErrUnsupported when the mesh carries no such proof. A mesh
// built below VerifyBoundary (boundaryOK false) is refused first, and with its
// own message, because every homotopy composing the volume proof has the
// facet-contact audit as an antecedent.
func OperandSymDiff(boundaryOK, symDiffOK bool, volSymDiff float64) (float64, error) {
	if !boundaryOK {
		return 0, fmt.Errorf(`%w: this operand's tessellation ran no facet-contact audit, so its facets are not proven to be an embedded boundary and no boolean may compose it; tessellate it at VerifyAll`, decaderr.ErrUnsupported)
	}
	if !symDiffOK {
		return 0, fmt.Errorf(`%w: this operand's tessellation carries no proof of the volume it and the body it stands for differ by, so no boolean may compose it`, decaderr.ErrUnsupported)
	}
	return volSymDiff, nil
}
