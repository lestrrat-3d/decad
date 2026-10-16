package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// tryClassB attempts op over the pair of bodies. ok=false (err nil) is a
// silent miss: the caller takes the mesh path unchanged. It is
// classbgeom.OfPayloads over the operands' payloads, placed as the first
// operand is.
func tryClassB(ctx context.Context, op meshbool.OperationKind, a, b *Body) (topo.Payload, bool, error) {
	// A body this evaluator did not build has no payload and no class.
	if a.n.Payload == nil || b.n.Payload == nil {
		return nil, false, nil
	}
	return classbgeom.OfPayloads(ctx, op, a.n.Payload, b.n.Payload, a.n.Payload.Transform(), 0)
}
