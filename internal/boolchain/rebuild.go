package boolchain

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// MaxRebuildSteps bounds the Boolean calls of one RebuildUnion: the rebuilt
// Unions and the final one. It bounds the work on the refusal path; no
// measurement chose it.
const MaxRebuildSteps = 32

// RebuildUnion evaluates Union(a, b) after rebuilding every expandable Union
// result beneath a and b from its recorded inputs
// (agent-docs/prism-boolean-design.md Part R, "Rim-split rebuild"). The rebuild
// repeats the recorded grouping exactly: depth first, each Union's inputs in
// recorded order, a Union of k inputs folded left to right, each Union one call
// to full. A body that does not expand, or whose producer is not a Union, is
// used as it is. So the result denotes the same set as Union(a, b) by
// construction, with no set identity. ok is false when neither operand
// expands, when the rebuild needs more than MaxRebuildSteps calls, or when a
// call misses. A non-nil error is a full error or ctx.Err(). a, b, src and
// full must not be nil.
func RebuildUnion(ctx context.Context, a, b *topo.Body, src Source, full Step) (*topo.Body, bool, error) {
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	count := rebuildCount{steps: 1}
	if !count.add(src, a) || !count.add(src, b) || !count.expanded {
		return nil, false, nil
	}
	r := rebuilder{src: src, full: full}
	x, ok, err := r.rebuild(ctx, a)
	if err != nil || !ok {
		return nil, false, err
	}
	y, ok, err := r.rebuild(ctx, b)
	if err != nil || !ok {
		return nil, false, err
	}
	return r.step(ctx, x, y)
}

// rebuildCount counts the calls a rebuild makes before making any.
type rebuildCount struct {
	steps    int
	expanded bool
}

// add counts the Unions beneath n. It returns false once the count passes
// MaxRebuildSteps.
func (c *rebuildCount) add(src Source, n *topo.Body) bool {
	in, ok := unionInputs(src, n)
	if !ok {
		return true
	}
	c.expanded = true
	c.steps += len(in) - 1
	if c.steps > MaxRebuildSteps {
		return false
	}
	for _, child := range in {
		if !c.add(src, child) {
			return false
		}
	}
	return true
}

// unionInputs returns the recorded inputs of n when n is an expandable Union
// result of at least two inputs.
func unionInputs(src Source, n *topo.Body) ([]*topo.Body, bool) {
	kind, in, ok := Expandable(src, n)
	if !ok || kind != meshbool.OpUnion || len(in) < 2 {
		return nil, false
	}
	return in, true
}

type rebuilder struct {
	src  Source
	full Step
}

func (r rebuilder) rebuild(ctx context.Context, n *topo.Body) (*topo.Body, bool, error) {
	in, ok := unionInputs(r.src, n)
	if !ok {
		return n, true, nil
	}
	acc, ok, err := r.rebuild(ctx, in[0])
	if err != nil || !ok {
		return nil, false, err
	}
	for _, child := range in[1:] {
		next, ok, err := r.rebuild(ctx, child)
		if err != nil || !ok {
			return nil, false, err
		}
		if acc, ok, err = r.step(ctx, acc, next); err != nil || !ok {
			return nil, false, err
		}
	}
	return acc, true, nil
}

func (r rebuilder) step(ctx context.Context, x, y *topo.Body) (*topo.Body, bool, error) {
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	return r.full(ctx, meshbool.OpUnion, x, y)
}
