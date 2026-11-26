package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/decad/internal/surfacetrim"
)

// This file implements Trim, Extend and Split: the public entry checks, the
// result builds and the commits. Their §2.1 gates live in
// internal/surfacetrim (prism family), internal/revolveplan (revolve Trim and
// Extend) and surface_split_revolve.go (revolve Split), and share
// prism-boolean's exact generator comparison, identity re-expression check and
// sweep-span relation. All three build a private scene through prismcells;
// Trim reads prismcells.Classify unchanged, while Split reads only the target
// label. Every miss is a typed refusal at the call: a sheet's mesh proves no
// occupied volume, and a mesh trim would decide topology from float signs on
// chorded triangles (agent-docs/surface-intersection-design.md §2.1).
//
// Trim and Extend take the REVOLVE family too, over the two operands' meridian
// views: §11's PR4, this file's own last section. Split's revolve arm is §11's
// PR5, in surface_split_revolve.go.

// Extend returns the command that lengthens b along the named edges' own carriers until
// they meet tool, and returns the lengthened sheet. It admits a b and
// tool sharing ONE generator, exactly as Trim does — both a straight sweep, or
// both a revolve about one axis — under surface-intersection §2.1's own gate.
// A nil context given to Do or a nil b is ErrDegenerate. The edge query is copied at the call.
func Extend(b *Body, edges *EdgeQuery, tool *Body) *ExtendCommand {
	c := &ExtendCommand{b: b, edges: copyEdgeQuery(edges), tool: tool}
	if b == nil {
		c.state.Record("body", errNilBody)
	}
	return c
}

// ExtendCommand lengthens a sheet to meet a tool. Build it with [Extend].
type ExtendCommand struct {
	state opcommand.State // consumed flag and first recorded fault
	b     *Body           // stored as given
	edges *EdgeQuery      // deep-copied at Extend
	tool  *Body           // stored as given
}

// Do lengthens the sheet and retires both operands, or returns the error of the first gate that refuses. A second
// call returns [ErrConsumed].
func (c *ExtendCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationExtend, c.run)
}

func (c *ExtendCommand) run(ctx context.Context) (*Body, error) {
	return extend(ctx, c.b, c.edges, c.tool)
}

// extend is Extend's implementation. ctx is not nil, and the errors it returns carry no operation tag
// (agent-docs/command-api-design.md §11).
func extend(ctx context.Context, b *Body, edges *EdgeQuery, tool *Body) (*Body, error) {
	if b == nil || b.n.Doc == nil {
		return nil, errNilBody
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := d.requireLive(tool); err != nil {
		return nil, err
	}
	if err := d.requireSameContainer([]*Body{b, tool}); err != nil {
		return nil, err
	}
	if b == tool {
		return nil, fmt.Errorf(`%w: an extension needs two distinct bodies`, ErrDegenerate)
	}
	selected, err := edges.SelectEdges(b)
	if err != nil {
		return nil, err
	}
	node, err := surfacetrim.Extend(ctx, envOf(documentNode(d), d.producerAt(ctx)), &b.n, edgeNodes(selected), &tool.n)
	if err != nil {
		return nil, err
	}
	body := bodyHandle(node)
	if err := d.commitOperationIn(ctx, nil, OperationExtend,
		[]OperationInput{{Body: b, Role: operationRoleTarget, Use: InputConsumed},
			{Body: tool, Role: operationRoleTool, Use: InputConsumed}}, body, b, tool); err != nil {
		return nil, err
	}
	return body, nil
}

// TrimSide names which pieces of the receiver a Trim keeps
// (agent-docs/surface-intersection-design.md §8).
type TrimSide int

const (
	// KeepOutside keeps the pieces of the receiver outside the tool's
	// section. KeepInside keeps the pieces inside it.
	KeepOutside TrimSide = iota
	KeepInside
)

// String renders the side for diagnostics.
func (s TrimSide) String() string {
	switch s {
	case KeepOutside:
		return "KeepOutside"
	case KeepInside:
		return "KeepInside"
	default:
		return fmt.Sprintf("TrimSide(%d)", int(s))
	}
}

// Trim returns the command that cuts b where it meets tool and returns the kept pieces as
// one sheet body (agent-docs/surface-intersection-design.md §8). b and
// tool are consumed on the document's uniform terms (core §6); a refusal
// consumes neither. It admits a pair whose two sweeps share ONE generator —
// both the prism family (a straight Extrude/chain Extrude sweep) or both the
// revolve family (a Revolve/chain Revolve spin about one axis,
// agent-docs/api-design.md's evaluator §5). A mixed pair, or any condition §2.1's
// S1-S7 states, is [ErrUnsupported] at the call, never a fallback: a sheet's
// mesh proves no occupied volume (booloperand.RequireVolumeProof),
// so there is no mesh path for a miss to reroute to. A tool that separates no
// fragment of b, or every fragment, is [ErrDegenerate] (R30): no
// trimmed body exists either way. A nil context given to Do or a nil b is ErrDegenerate.
func Trim(b *Body, tool *Body, side TrimSide) *TrimCommand {
	c := &TrimCommand{b: b, tool: tool, side: side}
	if b == nil {
		c.state.Record("body", errNilBody)
	}
	return c
}

// TrimCommand cuts a sheet where it meets a tool. Build it with [Trim].
type TrimCommand struct {
	state opcommand.State // consumed flag and first recorded fault
	b     *Body           // stored as given
	tool  *Body           // stored as given
	side  TrimSide        // gated at Do
}

// Do trims the sheet and retires both operands, or returns the error of the first gate that refuses. A second call
// returns [ErrConsumed].
func (c *TrimCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationTrim, c.run)
}

func (c *TrimCommand) run(ctx context.Context) (*Body, error) {
	return trim(ctx, c.b, c.tool, c.side)
}

// trim is Trim's implementation. ctx is not nil, and the errors it returns carry no operation tag
// (agent-docs/command-api-design.md §11).
func trim(ctx context.Context, b *Body, tool *Body, side TrimSide) (*Body, error) {
	if b == nil || b.n.Doc == nil {
		return nil, errNilBody
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := d.requireLive(tool); err != nil {
		return nil, err
	}
	if err := d.requireSameContainer([]*Body{b, tool}); err != nil {
		return nil, err
	}
	if b == tool {
		return nil, fmt.Errorf(`%w: a trim needs two distinct bodies`, ErrDegenerate)
	}
	if side != KeepOutside && side != KeepInside {
		return nil, fmt.Errorf(`%w: unknown trim side %d`, ErrDegenerate, int(side))
	}

	node, err := surfacetrim.Trim(ctx, envOf(documentNode(d), d.producerAt(ctx)), &b.n, &tool.n, side == KeepInside)
	if err != nil {
		return nil, err
	}
	body := bodyHandle(node)
	if err := d.commitOperationIn(ctx, nil, OperationTrim,
		[]OperationInput{{Body: b, Role: operationRoleTarget, Use: InputConsumed},
			{Body: tool, Role: operationRoleTool, Use: InputConsumed}}, body, b, tool); err != nil {
		return nil, err
	}
	return body, nil
}

// Split returns the command that cuts target with tool and returns one solid per arranged target cell
// in sketch's cell order. The target is a solid and the tool a sheet sharing
// its generator: both straight sweeps, or both revolves about one axis
// (agent-docs/surface-intersection-design.md §2.1). Any other pair, or a miss on
// that section's gate, is ErrUnsupported. Both operands are consumed only
// after every piece has been built. A tool that separates no target cell is
// ErrDegenerate, as is a nil context given to Do or a nil target. The document is the
// target's own.
func Split(target, tool *Body) *SplitCommand {
	c := &SplitCommand{target: target, tool: tool}
	if target == nil {
		c.state.Record("target", errSplitNilTarget)
	}
	return c
}

// errSplitNilTarget is the refusal of a nil target, which Split records because the operation reports it right
// after the nil ctx check.
var errSplitNilTarget = fmt.Errorf(`%w: the split target belongs to no document`, ErrDegenerate)

// SplitCommand cuts a solid into the cells a sheet separates. Build it with [Split].
type SplitCommand struct {
	state  opcommand.State // consumed flag and first recorded fault
	target *Body           // stored as given
	tool   *Body           // stored as given
}

// Do splits the target and retires both operands, or returns the error of the first gate that refuses. A second
// call returns [ErrConsumed].
func (c *SplitCommand) Do(ctx context.Context) ([]*Body, error) {
	return runCommand(ctx, &c.state, OperationSplit, c.run)
}

func (c *SplitCommand) run(ctx context.Context) ([]*Body, error) {
	return split(ctx, c.target, c.tool)
}

// split is Split's implementation. ctx is not nil, and the errors it returns carry no operation tag
// (agent-docs/command-api-design.md §11).
func split(ctx context.Context, target, tool *Body) ([]*Body, error) {
	if target == nil || target.n.Doc == nil {
		return nil, errSplitNilTarget
	}
	d := documentHandle(target.n.Doc)
	if err := d.requireLive(target); err != nil {
		return nil, err
	}
	if err := d.requireLive(tool); err != nil {
		return nil, err
	}
	if err := d.requireSameContainer([]*Body{target, tool}); err != nil {
		return nil, err
	}
	if target == tool {
		return nil, fmt.Errorf(`%w: a split needs two distinct bodies`, ErrDegenerate)
	}
	nodes, err := surfacetrim.Split(ctx, envOf(documentNode(d), d.producerAt(ctx)), &target.n, &tool.n)
	if err != nil {
		return nil, err
	}
	bodies := bodyHandles(nodes)
	if err := d.commitOperationManyIn(ctx, nil, OperationSplit,
		[]OperationInput{{Body: target, Role: operationRoleTarget, Use: InputConsumed},
			{Body: tool, Role: operationRoleTool, Use: InputConsumed}}, bodies, target, tool); err != nil {
		return nil, err
	}
	return bodies, nil
}
