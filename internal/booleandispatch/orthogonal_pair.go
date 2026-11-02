package booleandispatch

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
)

// OrthogonalPair returns the grid mesh of a Union, Cut or Intersect whose
// operands satisfy orthogonal-boolean design B1 and B7. At least one operand
// must have an admitted faceted payload; the other may be a solid whose fully
// verified 0.1 mm tessellation has no displacement bound. The caller must prove
// the returned mesh a solid. A declined pair leaves the ordinary route unchanged.
// ctx, a, b and meshes must not be nil.
func OrthogonalPair(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body,
	meshes verifiedmesh.Mesher) (orthoshell.Part, bool) {
	var mode orthoshell.Op
	switch op {
	case meshbool.OpUnion:
		mode = orthoshell.Union
	case meshbool.OpCut:
		mode = orthoshell.Cut
	case meshbool.OpIntersect:
		mode = orthoshell.Intersect
	default:
		return orthoshell.Part{}, false
	}
	pa, okA := orthogonalFacetedPart(a)
	pb, okB := orthogonalFacetedPart(b)
	if !okA && !okB {
		return orthoshell.Part{}, false // Two analytic operands keep their certified routes.
	}
	if !okA {
		if pa, okA = orthogonalMeshedPart(ctx, a, meshes); !okA {
			return orthoshell.Part{}, false
		}
	}
	if !okB {
		if pb, okB = orthogonalMeshedPart(ctx, b, meshes); !okB {
			return orthoshell.Part{}, false
		}
	}
	verts, tris, err := orthoshell.Combine(pa.Verts, pa.Tris, pb.Verts, pb.Tris, mode)
	if err != nil {
		return orthoshell.Part{}, false
	}
	return orthoshell.Part{Verts: verts, Tris: tris}, true
}

func orthogonalFacetedPart(body *topo.Body) (orthoshell.Part, bool) {
	payload, ok := body.Payload.(facetedrecord.Payload)
	if !ok {
		return orthoshell.Part{}, false
	}
	return orthoshell.FacetedPart(payload)
}

func orthogonalMeshedPart(ctx context.Context, body *topo.Body, meshes verifiedmesh.Mesher) (orthoshell.Part, bool) {
	if body.Payload == nil || body.Kind != topo.BodySolid {
		return orthoshell.Part{}, false
	}
	mesh, err := meshes.VerifyAll(ctx, body, 0.1)
	if err != nil || mesh.Bound != 0 || len(mesh.Triangles) == 0 {
		return orthoshell.Part{}, false
	}
	return orthoshell.Part{Verts: mesh.Vertices, Tris: mesh.Triangles}, true
}
