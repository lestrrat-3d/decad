package verifyrun

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file is the history containment certificate
// (agent-docs/interference-design.md §3.2, "The history certificate"): a Cut
// result's interior misses its tool's interior, so it misses the interior of
// any body the tool contains. A rail glued into the socket its own stock cut,
// whose faces coincide with the socket's, is the case: no held mesh decides
// the flush faces, but the history does. The walk reads a Boolean result's
// operation and inputs when the result denotes it exactly, mesh or analytic
// (historyWalk.expand). Containment is read from records:
//
//   - a Cut result lies in its target, an Intersect in either operand, a
//     Union in the union of its operands;
//   - a prism lies in another prism with the same frame and placement, levels
//     within the other's, and a section Sketch proves inside the other's;
//   - an analytic body lies in an analytically equal one;
//   - two prisms with one frame and placement miss each other when their
//     levels or their sections share no interior, which is how a wedge glued
//     flat on a rail is decided.

// historyMaxDepth bounds each walk; a deeper chain stays undecided.
const historyMaxDepth = 32

// historySeparated reports whether the recorded histories prove a's and b's
// interiors disjoint.
func historySeparated(ctx context.Context, history boolchain.Source, a, b *topo.Body) (bool, error) {
	w := historyWalk{ctx: ctx, history: history, budget: proofbound.NewWorkBudget(ctx)}
	ok, err := w.misses(a, b, 0)
	if err != nil || ok {
		return ok, err
	}
	return w.misses(b, a, 0)
}

type historyWalk struct {
	ctx     context.Context //nolint:containedctx // per-call walk state, never stored past historySeparated.
	history boolchain.Source
	budget  *proofbound.WorkBudget
}

// misses reports whether a's interior misses b's, read through a's history.
func (w historyWalk) misses(a, b *topo.Body, depth int) (bool, error) {
	if depth > historyMaxDepth {
		return false, nil
	}
	if err := w.budget.Step(); err != nil {
		return false, err
	}
	op, inputs, ok := w.expand(a)
	if !ok {
		if _, _, expands := w.expand(b); expands {
			return w.misses(b, a, depth+1)
		}
		return w.leafApart(a, b)
	}
	switch op {
	case meshbool.OpCut:
		if in, err := w.within(b, inputs[1], 0); err != nil || in {
			return in, err
		}
		return w.misses(inputs[0], b, depth+1)
	case meshbool.OpIntersect:
		for _, part := range inputs {
			if m, err := w.misses(part, b, depth+1); err != nil || m {
				return m, err
			}
		}
		return false, nil
	case meshbool.OpUnion:
		for _, part := range inputs {
			if m, err := w.misses(part, b, depth+1); err != nil || !m {
				return false, err
			}
		}
		return true, nil
	}
	return false, nil
}

// within reports whether inner lies in outer.
func (w historyWalk) within(inner, outer *topo.Body, depth int) (bool, error) {
	if depth > historyMaxDepth {
		return false, nil
	}
	if err := w.budget.Step(); err != nil {
		return false, err
	}
	if in, err := w.leafWithin(inner, outer); err != nil || in {
		return in, err
	}
	if op, inputs, ok := w.expand(outer); ok {
		switch op {
		case meshbool.OpUnion:
			for _, part := range inputs {
				if in, err := w.within(inner, part, depth+1); err != nil || in {
					return in, err
				}
			}
		case meshbool.OpIntersect:
			all := true
			for _, part := range inputs {
				in, err := w.within(inner, part, depth+1)
				if err != nil {
					return false, err
				}
				all = all && in
				if !all {
					break
				}
			}
			if all {
				return true, nil
			}
		}
	}
	op, inputs, ok := w.expand(inner)
	if !ok {
		return false, nil
	}
	switch op {
	case meshbool.OpCut:
		return w.within(inputs[0], outer, depth+1)
	case meshbool.OpIntersect:
		for _, part := range inputs {
			if in, err := w.within(part, outer, depth+1); err != nil || in {
				return in, err
			}
		}
		return false, nil
	case meshbool.OpUnion:
		for _, part := range inputs {
			if in, err := w.within(part, outer, depth+1); err != nil || !in {
				return false, err
			}
		}
		return true, nil
	}
	return false, nil
}

// expand returns body's recorded Boolean producer and inputs when the body
// denotes that operation exactly: a mesh Boolean result the chain admits
// (boolchain.Expandable), or any other Boolean result with no operation
// warning and no private shift. An analytic result's record may hold its
// section within a displacement of the set, but the set it denotes is the
// operation's exact result, which is all the walk reads.
func (w historyWalk) expand(body *topo.Body) (meshbool.OperationKind, []*topo.Body, bool) {
	return expandRecorded(w.history, body)
}

// expandRecorded is historyWalk.expand over a bare history; nil reads none.
func expandRecorded(history boolchain.Source, body *topo.Body) (meshbool.OperationKind, []*topo.Body, bool) {
	if op, inputs, ok := expandable(history, body); ok {
		return op, inputs, true
	}
	if history == nil || len(body.OperationWarnings) != 0 || body.PrivateShift != nil {
		return 0, nil, false
	}
	if _, mesh := body.Payload.(facetedrecord.Payload); mesh {
		return 0, nil, false
	}
	op, inputs, ok := history.BooleanProducer(body)
	if !ok || !arityHolds(op, inputs) {
		return 0, nil, false
	}
	return op, inputs, true
}

// leafApart decides between two records directly whether their interiors are
// disjoint: two prisms with one frame and placement whose levels or sections
// share no interior.
func (w historyWalk) leafApart(a, b *topo.Body) (bool, error) {
	pa, okA := a.Payload.(prismrecord.Payload)
	pb, okB := b.Payload.(prismrecord.Payload)
	if !okA || !okB || !sameExactLift(pa, pb) {
		return false, nil
	}
	alo, ahi := min(pa.Z0, pa.Z1), max(pa.Z0, pa.Z1)
	blo, bhi := min(pb.Z0, pb.Z1), max(pb.Z0, pb.Z1)
	if ahi <= blo || bhi <= alo {
		return true, nil
	}
	return prismcells.SectionsApart(w.ctx, w.budget, []momentinput.Profile{pa.Profile}, []momentinput.Profile{pb.Profile})
}

// sameExactLift reports whether two prisms share one frame and placement and
// hold their sections and levels exactly, so their sections and levels compare
// in one coordinate system.
func sameExactLift(a, b prismrecord.Payload) bool {
	if a.Frame != b.Frame || a.Xform != b.Xform || a.SurfaceResult || b.SurfaceResult {
		return false
	}
	for _, d := range []float64{a.SectionDelta, b.SectionDelta, a.Z0Delta, a.Z1Delta, b.Z0Delta, b.Z1Delta} {
		if d != 0 {
			return false
		}
	}
	return true
}

// leafWithin decides containment between two records directly.
func (w historyWalk) leafWithin(inner, outer *topo.Body) (bool, error) {
	if inner == outer {
		return true, nil
	}
	pi, okI := inner.Payload.(prismrecord.Payload)
	po, okO := outer.Payload.(prismrecord.Payload)
	if okI && okO {
		return prismWithin(w.ctx, w.budget, pi, po)
	}
	return analyticBodiesEqual(w.budget, inner, outer)
}

// prismWithin reports whether prism inner lies in prism outer: one frame and
// placement, levels held exactly with inner's within outer's, and a section
// Sketch proves inside outer's. Every other pair answers false.
func prismWithin(ctx context.Context, budget *proofbound.WorkBudget, inner, outer prismrecord.Payload) (bool, error) {
	if !sameExactLift(inner, outer) {
		return false, nil
	}
	ilo, ihi := min(inner.Z0, inner.Z1), max(inner.Z0, inner.Z1)
	olo, ohi := min(outer.Z0, outer.Z1), max(outer.Z0, outer.Z1)
	if ilo < olo || ihi > ohi {
		return false, nil
	}
	return prismcells.SectionWithin(ctx, budget, []momentinput.Profile{inner.Profile}, []momentinput.Profile{outer.Profile})
}
