package boolchain

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Step performs one Boolean. A non-nil error is cancellation or a surfaced invariant
// break and Run returns it unchanged; every other failure is ok == false.
type Step func(ctx context.Context, op meshbool.OperationKind, x, y *topo.Body) (*topo.Body, bool, error)

// Steps are the callbacks Run evaluates a chain with.
type Steps struct {
	// Exact runs analytic routes only; ok=false on any miss or failure.
	Exact Step
	// Full runs the complete route order, mesh pipeline included; ok=false on any failure.
	Full Step
}

type missKey struct {
	op   meshbool.OperationKind
	x, y *topo.Body
}

type runner struct {
	steps Steps
	miss  map[missKey]struct{}
}

// Run evaluates chain's plan: every pair Exact admits first, then at most one Full
// step. ok is false when the plan cannot finish. A non-nil error is a Step error or
// ctx.Err(). Run never calls Exact twice with one (op, x, y).
func Run(ctx context.Context, chain Chain, steps Steps) (*topo.Body, bool, error) {
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	r := &runner{steps: steps, miss: map[missKey]struct{}{}}
	if chain.Disjunction {
		return r.disjunction(ctx, chain)
	}
	return r.conjunction(ctx, chain)
}

func opOf(l Literal) meshbool.OperationKind {
	if l.Negated {
		return meshbool.OpCut
	}
	return meshbool.OpIntersect
}

func (r *runner) exact(ctx context.Context, op meshbool.OperationKind, x, y *topo.Body) (*topo.Body, bool, error) {
	key := missKey{op: op, x: x, y: y}
	if _, missed := r.miss[key]; missed {
		return nil, false, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	body, ok, err := r.steps.Exact(ctx, op, x, y)
	if err != nil {
		return nil, false, err
	}
	if !ok {
		r.miss[key] = struct{}{}
		return nil, false, nil
	}
	return body, true, nil
}

func (r *runner) full(ctx context.Context, op meshbool.OperationKind, x, y *topo.Body) (*topo.Body, bool, error) {
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	body, ok, err := r.steps.Full(ctx, op, x, y)
	if err != nil {
		return nil, false, err
	}
	return body, ok, nil
}

func (r *runner) conjunction(ctx context.Context, chain Chain) (*topo.Body, bool, error) {
	acc := chain.Literals[0].Body
	pending := append([]Literal(nil), chain.Literals[1:]...)
	var err error
	if acc, pending, err = r.absorb(ctx, acc, pending); err != nil {
		return nil, false, err
	}
	if len(pending) >= 2 && allNegated(pending) {
		items := make([]*topo.Body, len(pending))
		for i, l := range pending {
			items[i] = l.Body
		}
		if items, err = r.merge(ctx, items); err != nil {
			return nil, false, err
		}
		pending = pending[:0]
		for _, b := range items {
			pending = append(pending, Literal{Body: b, Negated: true})
		}
		if acc, pending, err = r.absorb(ctx, acc, pending); err != nil {
			return nil, false, err
		}
	}
	switch len(pending) {
	case 0:
		return acc, true, nil
	case 1:
		return r.full(ctx, opOf(pending[0]), acc, pending[0].Body)
	}
	return nil, false, nil
}

// absorb applies every pending literal an analytic route admits to acc, scanning in
// order and restarting after each success.
func (r *runner) absorb(ctx context.Context, acc *topo.Body, pending []Literal) (*topo.Body, []Literal, error) {
	for {
		absorbed := false
		for i, l := range pending {
			next, ok, err := r.exact(ctx, opOf(l), acc, l.Body)
			if err != nil {
				return nil, nil, err
			}
			if !ok {
				continue
			}
			acc = next
			pending = append(pending[:i:i], pending[i+1:]...)
			absorbed = true
			break
		}
		if !absorbed {
			return acc, pending, nil
		}
	}
}

// merge unions the first admitted pair of items, restarting after each success.
func (r *runner) merge(ctx context.Context, items []*topo.Body) ([]*topo.Body, error) {
	for {
		merged := false
	pairs:
		for i := range items {
			for j := i + 1; j < len(items); j++ {
				next, ok, err := r.exact(ctx, meshbool.OpUnion, items[i], items[j])
				if err != nil {
					return nil, err
				}
				if !ok {
					continue
				}
				items[i] = next
				items = append(items[:j:j], items[j+1:]...)
				merged = true
				break pairs
			}
		}
		if !merged {
			return items, nil
		}
	}
}

func (r *runner) disjunction(ctx context.Context, chain Chain) (*topo.Body, bool, error) {
	items := make([]*topo.Body, len(chain.Literals))
	for i, l := range chain.Literals {
		items[i] = l.Body
	}
	items, err := r.merge(ctx, items)
	if err != nil {
		return nil, false, err
	}
	switch len(items) {
	case 1:
		return items[0], true, nil
	case 2:
		return r.full(ctx, meshbool.OpUnion, items[0], items[1])
	}
	return nil, false, nil
}

func allNegated(ls []Literal) bool {
	for _, l := range ls {
		if !l.Negated {
			return false
		}
	}
	return true
}
