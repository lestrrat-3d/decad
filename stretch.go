package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Stretch returns the command that scales a faceted solid by a separate factor along each world axis about
// anchor. Do returns the result as a new solid, retiring b (agent-docs/mesh-solid-design.md §3). It is the non-uniform scale
// [ScaledAt] does not offer: a box becomes a longer box, and a part with planar faces keeps every face
// planar while a slanted one tilts.
//
// b MUST be a live faceted solid whose held vertices carry no bound, such as one made by
// [SolidFromMesh] or by [MoveFace]; any other body is ErrUnsupported, because a stretched curved or
// bounded body is not a solid this route can prove. The result is built as [ScaleMesh] builds a mesh and
// read back as [SolidFromMesh] reads one, so its planes are exact over the stretched coordinates. Each
// factor is dimensionless, finite and not zero (ErrUnitKind, ErrNotFinite, ErrDegenerate); a negative
// factor mirrors across the plane through anchor. A nil context or body is ErrDegenerate.
func Stretch(b *Body, anchor r3.Vec, x, y, z units.Value) *StretchCommand {
	c := &StretchCommand{b: b, anchor: anchor, x: x, y: y, z: z}
	if b == nil {
		c.state.Record("body", errNilBody)
	}
	return c
}

// StretchCommand scales a faceted solid by a factor per world axis. Build it with [Stretch].
type StretchCommand struct {
	state   opcommand.State // consumed flag and first recorded fault
	b       *Body           // stored as given
	anchor  r3.Vec          // the fixed point, gated at Do
	x, y, z units.Value     // the factors, gated at Do
}

// Do stretches the body and retires it, or returns the error of the first gate that refuses. A nil ctx is
// ErrDegenerate, and a second call returns ErrConsumed.
func (c *StretchCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationStretch, c.run)
}

func (c *StretchCommand) run(ctx context.Context) (*Body, error) {
	return stretch(ctx, c.b, c.anchor, c.x, c.y, c.z)
}

// stretch is Stretch's implementation. ctx is not nil, and the errors it returns carry no operation tag
// (agent-docs/command-api-design.md §11).
func stretch(ctx context.Context, b *Body, anchor r3.Vec, x, y, z units.Value) (*Body, error) {
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if b.Kind() != BodySolid {
		return nil, fmt.Errorf(`%w: Stretch needs a solid body`, ErrNotSolid)
	}
	payload, ok := b.n.Payload.(facetedPayload)
	if !ok || len(payload.Tris) == 0 {
		return nil, fmt.Errorf(`%w: Stretch takes a faceted solid, such as a mesh solid`, ErrUnsupported)
	}
	for _, beta := range payload.VertexBound {
		if beta != 0 {
			return nil, fmt.Errorf(`%w: Stretch takes a faceted solid whose held vertices carry no bound`, ErrUnsupported)
		}
	}
	for _, c := range []float64{anchor.X, anchor.Y, anchor.Z} {
		if math.IsNaN(c) || math.IsInf(c, 0) {
			return nil, fmt.Errorf(`%w: the stretch anchor is not finite`, ErrNotFinite)
		}
	}
	mesh, err := NewMeshBody(ctx, payload.Verts, payload.Tris)
	if err != nil {
		return nil, err
	}
	scaled, err := ScaleMesh(ctx, mesh, anchor, x, y, z)
	if err != nil {
		return nil, err
	}
	stretched, err := solidFromMeshBody(ctx, d, scaled)
	if err != nil {
		// A cancel inside the conversion is the caller's, not an unsupported solid.
		if cerr := ctx.Err(); cerr != nil {
			return nil, cerr
		}
		return nil, fmt.Errorf("%w: the stretched solid is not one the evaluator can prove: %s", ErrUnsupported, err)
	}
	return commitModifyResult(ctx, OperationStretch, b, stretched)
}
