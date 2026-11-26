package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/r3"
)

// QuarterTurn returns the command that turns a faceted solid by a whole number of quarter turns about an axis
// through about, a world axis with the right-hand sense. Do returns the result as a new solid, retiring b
// (agent-docs/orthogonal-design.md B9). A quarter turn only exchanges and negates coordinates, so the
// turned solid is exact where [Transform] by a 90 degree [r3.Rotation] is not: its vertices stay unbounded and it
// stays on the exact routes of an axis-square solid.
//
// b MUST be a live faceted solid whose held vertices carry no bound, such as a mesh solid, otherwise
// ErrUnsupported. axis MUST be exactly a world axis, +X, +Y, +Z or its negative, otherwise ErrUnsupported; turns
// turn counter-clockwise looking down the axis toward its origin when positive and the other way when
// negative, and a multiple of four turns changes nothing, which is ErrDegenerate. A turn whose coordinates
// cannot be held exactly (about a point the sums round on) is ErrUnsupported and changes nothing. A nil
// context or body is ErrDegenerate.
func QuarterTurn(b *Body, axis, about r3.Vec, turns int) *QuarterTurnCommand {
	c := &QuarterTurnCommand{b: b, axis: axis, about: about, turns: turns}
	if b == nil {
		c.state.Record("body", errNilBody)
	}
	return c
}

// QuarterTurnCommand turns a faceted solid by whole quarter turns. Build it with [QuarterTurn].
type QuarterTurnCommand struct {
	state       opcommand.State // consumed flag and first recorded fault
	b           *Body           // stored as given
	axis, about r3.Vec          // the axis direction and a point on it, gated at Do
	turns       int             // the number of quarter turns
}

// Do turns the body and retires it, or returns the error of the first gate that refuses. A nil ctx is
// ErrDegenerate, and a second call returns ErrConsumed.
func (c *QuarterTurnCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationQuarterTurn, c.run)
}

func (c *QuarterTurnCommand) run(ctx context.Context) (*Body, error) {
	return quarterTurn(ctx, c.b, c.axis, c.about, c.turns)
}

// quarterTurn is QuarterTurn's implementation. ctx is not nil, and the errors it returns carry no operation tag
// (agent-docs/command-api-design.md §11).
func quarterTurn(ctx context.Context, b *Body, axis, about r3.Vec, turns int) (*Body, error) {
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if b.Kind() != BodySolid {
		return nil, fmt.Errorf(`%w: QuarterTurn needs a solid body`, ErrNotSolid)
	}
	for _, c := range []float64{about.X, about.Y, about.Z} {
		if math.IsNaN(c) || math.IsInf(c, 0) {
			return nil, fmt.Errorf(`%w: the quarter turn centre is not finite`, ErrNotFinite)
		}
	}
	index, sign, ok := unitAxis(axis)
	if !ok {
		return nil, fmt.Errorf(`%w: QuarterTurn turns about a world axis, +X, +Y, +Z or a negative one`, ErrUnsupported)
	}
	quarters := ((int(sign)*turns)%4 + 4) % 4
	if quarters == 0 {
		return nil, fmt.Errorf(`%w: a whole number of turns changes nothing`, ErrDegenerate)
	}
	payload, ok := b.n.Payload.(facetedPayload)
	if !ok || len(payload.Tris) == 0 {
		return nil, fmt.Errorf(`%w: QuarterTurn takes a faceted solid, such as a mesh solid`, ErrUnsupported)
	}
	for _, beta := range payload.VertexBound {
		if beta != 0 {
			return nil, fmt.Errorf(`%w: QuarterTurn takes a faceted solid whose held vertices carry no bound`, ErrUnsupported)
		}
	}
	verts, err := quarterTurnVerts(payload.Verts, index, quarters, about)
	if err != nil {
		return nil, err
	}
	mesh, err := NewMeshBody(ctx, verts, payload.Tris)
	if err != nil {
		return nil, err
	}
	turned, err := solidFromMeshBody(ctx, d, mesh)
	if err != nil {
		// A cancel inside the conversion is the caller's, not an unsupported solid.
		if cerr := ctx.Err(); cerr != nil {
			return nil, cerr
		}
		return nil, fmt.Errorf("%w: the turned solid is not one the evaluator can prove: %s", ErrUnsupported, err)
	}
	return commitModifyResult(ctx, OperationQuarterTurn, b, turned)
}

// quarterTurnVerts turns the held vertices by quarters (1 to 3) quarter turns about the world axis index through about,
// in the right-hand sense: a quarter turn only exchanges and negates coordinates, so each result is exact, and a vertex
// whose coordinate relative to the centre, or the turned one added back, would round is ErrUnsupported.
func quarterTurnVerts(held []r3.Vec, index, quarters int, about r3.Vec) ([]r3.Vec, error) {
	// One quarter turn about +X sends (y, z) to (-z, y), about +Y (z, x) to (-x, z) and about +Z (x, y) to (-y, x).
	u, v := (index+1)%3, (index+2)%3
	verts := make([]r3.Vec, len(held))
	o := [3]float64{about.X, about.Y, about.Z}
	for i, p := range held {
		pc := [3]float64{p.X, p.Y, p.Z}
		var c [3]float64
		for k := range 3 {
			// The coordinate relative to the centre must be held exactly: the sum has no rounding error.
			sum, exact := exactSum(pc[k], -o[k])
			if !exact {
				return nil, fmt.Errorf(`%w: vertex %d does not hold exactly relative to the quarter turn centre`, ErrUnsupported, i)
			}
			c[k] = sum
		}
		for range quarters {
			c[u], c[v] = -c[v], c[u]
		}
		var q [3]float64
		for k := range 3 {
			sum, exact := exactSum(o[k], c[k])
			if !exact {
				return nil, fmt.Errorf(`%w: vertex %d does not turn exactly about the centre`, ErrUnsupported, i)
			}
			q[k] = sum
		}
		verts[i] = r3.NewVec(q[0], q[1], q[2])
	}
	return verts, nil
}

// exactSum returns a + b and whether that float is the exact sum, by the error-free transformation of Knuth's
// TwoSum: the rounding error of a sum is itself a float, so the sum is exact when the error is zero.
func exactSum(a, b float64) (float64, bool) {
	s := a + b
	bb := s - a
	err := (a - (s - bb)) + (b - bb)
	return s, err == 0 && !math.IsInf(s, 0) && !math.IsNaN(s)
}
