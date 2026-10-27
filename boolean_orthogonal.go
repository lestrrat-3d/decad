package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
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
	if !okA && !okB {
		return nil, false // a pair of prisms or breps keeps its own certified routes
	}
	if !okA {
		if pa, okA = orthogonalMeshOf(ctx, a); !okA {
			return nil, false
		}
	}
	if !okB {
		if pb, okB = orthogonalMeshOf(ctx, b); !okB {
			return nil, false
		}
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

// orthogonalMeshOf returns the tessellation of a solid that is not itself faceted, such as an extruded
// rectilinear profile, as an operand when the mesh is the body exactly: it holds no displacement bound, so every
// vertex lies on the body's boundary, and every triangle is square to an axis. It is asked only beside a faceted
// operand; two bodies that are not faceted keep their own routes.
func orthogonalMeshOf(ctx context.Context, b *Body) (facetedPayload, bool) {
	if b.n.Payload == nil || b.Kind() != BodySolid {
		return facetedPayload{}, false
	}
	mesh, err := tessellateContext(ctx, b, units.Millimeters(0.1), VerifyAll)
	if err != nil || mesh.n.Bound != 0 || len(mesh.n.Triangles) == 0 {
		return facetedPayload{}, false
	}
	return facetedPayload{Verts: mesh.n.Vertices, Tris: mesh.n.Triangles}, true
}
