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

// PrepContextMultiway prepares a private operand under the shared trial budget.
// It leaves depth summaries empty; consumers then use the equivalent direct scan.
func PrepContextMultiway(ctx context.Context, verts []r3.Vec, tris [][3]int, src []int,
	budget *proofbound.MultiwayBudget) (*BoolMesh, error) {
	return prepContextBudgeted(ctx, verts, tris, src, budget, false)
}

// PrepContextReplay permits wider finite normals using bit-length work charges.
func PrepContextReplay(ctx context.Context, verts []r3.Vec, tris [][3]int, src []int,
	budget *proofbound.ReplayBudget) (*BoolMesh, error) {
	if budget == nil {
		return nil, fmt.Errorf("%w: bounded mesh preparation needs a budget", decaderr.ErrUnsupported)
	}
	return prepContextReplay(ctx, verts, tris, src, budget)
}

func prepContextBudgeted(ctx context.Context, verts []r3.Vec, tris [][3]int, src []int,
	budget *proofbound.MultiwayBudget, replay bool) (*BoolMesh, error) {
	if budget == nil {
		return nil, fmt.Errorf("%w: bounded mesh preparation needs a budget", decaderr.ErrUnsupported)
	}
	if len(tris) > 65536 {
		return nil, fmt.Errorf("%w: bounded mesh preparation facet cap exceeded", decaderr.ErrUnsupported)
	}
	if err := budget.ReserveRecords(ctx, uint64(len(verts))); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, 7*uint64(len(tris))); err != nil {
		return nil, err
	}
	bm := &BoolMesh{Verts: verts, Tris: tris, Src: src}
	bm.Xverts = make([]proof.Xpt, len(verts))
	for i, v := range verts {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		if proofbound.IsNonFinite(v.X) || proofbound.IsNonFinite(v.Y) || proofbound.IsNonFinite(v.Z) {
			return nil, fmt.Errorf("%w: a mesh vertex is not finite", decaderr.ErrBooleanFailed)
		}
		p, err := proof.XptOfBounded(v, func(n uint64) error {
			if replay {
				if n > ^uint64(0)/64 {
					return fmt.Errorf("%w: exact point byte reservation overflow", decaderr.ErrUnsupported)
				}
				if err := budget.ReserveBytes(ctx, n*64); err != nil {
					return err
				}
			}
			return budget.ReserveRecords(ctx, n)
		})
		if err != nil {
			return nil, err
		}
		bm.Xverts[i] = p
	}
	bm.Norms = make([]proof.Xpt, len(tris))
	bm.Fnorms = make([]r3.Vec, len(tris))
	bm.FnormsReady = make([]bool, len(tris))
	bm.Boxes = make([][2]r3.Vec, len(tris))
	bm.Owner = make(map[[2]int]int, 3*len(tris))
	for i, tri := range tris {
		if err := budget.SpendWork(ctx, 4); err != nil {
			return nil, err
		}
		var n proof.Xpt
		var err error
		reserve := func(n uint64) error { return budget.ReserveRecords(ctx, n) }
		if replay {
			n, err = proof.TriangleNormalReplay(verts[tri[0]], verts[tri[1]], verts[tri[2]], func(n uint64) error {
				if n > ^uint64(0)/64 {
					return fmt.Errorf("%w: exact normal byte reservation overflow", decaderr.ErrUnsupported)
				}
				if err := budget.ReserveBytes(ctx, n*64); err != nil {
					return err
				}
				return reserve(n)
			}, func(n uint64) error { return budget.SpendWork(ctx, n) })
		} else {
			n, err = proof.TriangleNormalBounded(verts[tri[0]], verts[tri[1]], verts[tri[2]], reserve)
		}
		if err != nil {
			return nil, err
		}
		if n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0 {
			return nil, fmt.Errorf("%w: an operand holds a collapsed facet", decaderr.ErrUnsupported)
		}
		bm.Norms[i] = n
		bm.Boxes[i] = TriBox(verts, tri)
		for k := range 3 {
			bm.Owner[[2]int{tri[k], tri[(k+1)%3]}] = i
		}
	}
	return bm, nil
}
