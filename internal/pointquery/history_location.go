package pointquery

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// LocateBoolean classifies the world point p against the set a mesh Boolean
// result denotes, by combining its operands' answers through the operation
// history src records (agent-docs/point-containment-design.md §8). Only a
// body boolchain.Expandable accepts is combined; an operand is read through
// LocatePrism or, when it is itself such a result, through its own operands.
// Every other operand reads Undecided. decided is false when the combination
// leaves the point open; the caller then classifies against the verified
// mesh (§2). p must be finite and n and src must not be nil.
func LocateBoolean(ctx context.Context, p r3.Vec, n *topo.Body, src boolchain.Source) (Location, bool, error) {
	if err := ctx.Err(); err != nil {
		return Undecided, false, err
	}
	w := historyWalk{budget: proofbound.NewWorkBudget(ctx), p: p, src: src}
	result, err := w.locate(ctx, n)
	if err != nil {
		return Undecided, false, err
	}
	return result, result != Undecided, nil
}

// historyWalk carries one LocateBoolean query through the producer tree.
type historyWalk struct {
	budget *proofbound.WorkBudget
	p      r3.Vec
	src    boolchain.Source
}

// locate classifies the walk's point against n: an undisplaced prism through
// §7, an expandable Boolean result through its recorded operands, and every
// other body as Undecided.
func (w historyWalk) locate(ctx context.Context, n *topo.Body) (Location, error) {
	if err := w.budget.Step(); err != nil {
		return Undecided, err
	}
	if pp, ok := n.Payload.(prismrecord.Payload); ok {
		result, decided, err := LocatePrism(ctx, w.p, pp)
		if err != nil || !decided {
			return Undecided, err
		}
		return result, nil
	}
	kind, inputs, ok := boolchain.Expandable(w.src, n)
	if !ok {
		return Undecided, nil
	}
	switch kind {
	case meshbool.OpUnion:
		return w.union(ctx, inputs)
	case meshbool.OpIntersect:
		return w.intersect(ctx, inputs)
	case meshbool.OpCut:
		if len(inputs) != 2 {
			return Undecided, nil
		}
		return w.cut(ctx, inputs[0], inputs[1])
	}
	return Undecided, nil
}

// union joins every input's answer, stopping once the point is inside.
func (w historyWalk) union(ctx context.Context, inputs []*topo.Body) (Location, error) {
	if len(inputs) == 0 {
		return Undecided, nil
	}
	acc := Outside
	for _, in := range inputs {
		x, err := w.locate(ctx, in)
		if err != nil {
			return Undecided, err
		}
		if acc = join(acc, x); acc == Inside {
			return Inside, nil
		}
	}
	return acc, nil
}

// intersect meets every input's answer, stopping once the point is outside.
func (w historyWalk) intersect(ctx context.Context, inputs []*topo.Body) (Location, error) {
	if len(inputs) == 0 {
		return Undecided, nil
	}
	acc := Inside
	for _, in := range inputs {
		x, err := w.locate(ctx, in)
		if err != nil {
			return Undecided, err
		}
		if acc = meet(acc, x); acc == Outside {
			return Outside, nil
		}
	}
	return acc, nil
}

// cut meets the target's answer with the complement of the tool's; a point
// outside the target needs no tool reading.
func (w historyWalk) cut(ctx context.Context, target, tool *topo.Body) (Location, error) {
	a, err := w.locate(ctx, target)
	if err != nil || a == Outside {
		return a, err
	}
	b, err := w.locate(ctx, tool)
	if err != nil {
		return Undecided, err
	}
	return meet(a, complement(b)), nil
}

// meet classifies p against the regularized intersection of two regular
// closed sets from p's class in each (§8.3). Outside either set is outside;
// inside both is inside; inside one and on the other's boundary is on the
// boundary, because near p the intersection equals the second set. Any other
// pair, including on both boundaries, is Undecided.
func meet(a, b Location) Location {
	switch {
	case a == Outside || b == Outside:
		return Outside
	case a == Inside && b == Inside:
		return Inside
	case a == Inside && b == OnBoundary, a == OnBoundary && b == Inside:
		return OnBoundary
	}
	return Undecided
}

// complement classifies p against the closure of a regular closed set's
// complement: inside and outside swap, and the boundary is shared.
func complement(a Location) Location {
	switch a {
	case Inside:
		return Outside
	case Outside:
		return Inside
	}
	return a
}

// join classifies p against the union of two regular closed sets, the
// complement of the meet of their complements.
func join(a, b Location) Location {
	return complement(meet(complement(a), complement(b)))
}
