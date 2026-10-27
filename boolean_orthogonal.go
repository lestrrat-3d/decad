package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/r3"
)

// orthogonalBoolean performs a Union, Cut or Intersect of two faceted solids every face of which squares to a
// world axis, held at no vertex bound and unplaced, such as mesh solids and the results of this route
// (agent-docs/orthogonal-boolean-design.md). The operands are read onto one grid of their coordinates and the
// result is built from its cells, so it is exact and a pair that touches or overlaps in a face is decided like any
// other. The result is not committed. ok is false when the route does not apply or refuses the pair, and the
// caller takes the general route: an operand outside the class, a result with no cell, a grid past its cells, or a
// result the solid proof refuses (kept cells that meet only along an edge).
func orthogonalBoolean(ctx context.Context, d *Document, op meshbool.OperationKind, a, b *Body) (*Body, bool) {
	var mode orthoshell.Op
	switch op {
	case meshbool.OpUnion:
		mode = orthoshell.Union
	case meshbool.OpCut:
		mode = orthoshell.Cut
	case meshbool.OpIntersect:
		mode = orthoshell.Intersect
	default:
		return nil, false
	}
	pa, okA := orthogonalOperand(a)
	pb, okB := orthogonalOperand(b)
	if !okA || !okB {
		return nil, false
	}
	verts, tris, err := orthoshell.Combine(pa.Verts, pa.Tris, pb.Verts, pb.Tris, mode)
	if err != nil {
		return nil, false
	}
	mesh, err := NewMeshBody(ctx, verts, tris)
	if err != nil {
		return nil, false
	}
	body, err := solidFromMeshBody(ctx, d, mesh)
	if err != nil {
		return nil, false
	}
	return body, true
}

// orthogonalOperand returns b's faceted payload when it is unplaced and holds no vertex bound.
func orthogonalOperand(b *Body) (facetedPayload, bool) {
	payload, ok := b.n.Payload.(facetedPayload)
	if !ok || len(payload.Tris) == 0 || payload.Xform != r3.Identity() {
		return facetedPayload{}, false
	}
	for _, beta := range payload.VertexBound {
		if beta != 0 {
			return facetedPayload{}, false
		}
	}
	return payload, true
}
