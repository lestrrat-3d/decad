package verifyrun

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
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

// sameSetSource is a history that also names the producers whose result is
// the same true solid as their one input: a Refine moves no vertex and only
// tightens the bound it publishes, so its result is read as its source.
type sameSetSource interface {
	SameSetProducer(n *topo.Body) (*topo.Body, bool)
}

// sameSetOf follows history's same-set producers back from b, a bounded
// number of steps.
func sameSetOf(history boolchain.Source, b *topo.Body) *topo.Body {
	s, ok := history.(sameSetSource)
	if !ok {
		return b
	}
	for range historyMaxDepth {
		src, ok := s.SameSetProducer(b)
		if !ok || src == nil {
			break
		}
		b = src
	}
	return b
}

// misses reports whether a's interior misses b's, read through a's history.
func (w historyWalk) misses(a, b *topo.Body, depth int) (bool, error) {
	if depth > historyMaxDepth {
		return false, nil
	}
	a, b = sameSetOf(w.history, a), sameSetOf(w.history, b)
	if err := w.budget.Step(); err != nil {
		return false, err
	}
	// Every body's interior lies in the interior of its box inflated by the
	// box's own proven bound, so two such boxes with disjoint interiors keep
	// the bodies apart; the comparison is exact (measurement.BoxesDisjoint).
	// A Union's operand far from the other body is decided here.
	if measurement.BoxesDisjoint(a.Bounds, b.Bounds) {
		return true, nil
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
		if apart, err := cutToolMeridianApart(w.ctx, w.budget, w.history, inputs[0], inputs[1], b); err != nil || apart {
			return apart, err
		}
		if apart, err := axialApart(w.ctx, w.budget, w.history, a, b); err != nil || apart {
			return apart, err
		}
		if apart, err := revolvePiecesApart(w.ctx, w.budget, w.history, a, b); err != nil || apart {
			return apart, err
		}
		if apart, err := boxApart(w.ctx, w.budget, w.history, a, b); err != nil || apart {
			return apart, err
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
	inner, outer = sameSetOf(w.history, inner), sameSetOf(w.history, outer)
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

// leafApartOtherwise decides two leaves the shared-lift prism rule leaves
// open: by their meridians about one axis, by a prism's end-cap plane, or by
// one of a prism's frame planes (sideApart).
func (w historyWalk) leafApartOtherwise(a, b *topo.Body) (bool, error) {
	if apart, err := meridianApart(w.ctx, w.budget, a, b); err != nil || apart {
		return apart, err
	}
	if capApart(a, b) || capApart(b, a) || sideApart(a, b) || sideApart(b, a) {
		return true, nil
	}
	return axialApart(w.ctx, w.budget, w.history, a, b)
}

// leafApart decides between two records directly whether their interiors are
// disjoint: two prisms with one exact linear lift whose levels or sections,
// read in one prism's coordinates, share no interior.
func (w historyWalk) leafApart(a, b *topo.Body) (bool, error) {
	pa, okA := a.Payload.(prismrecord.Payload)
	pb, okB := b.Payload.(prismrecord.Payload)
	if !okA || !okB {
		return w.leafApartOtherwise(a, b)
	}
	off, ok := liftOffset(pa, pb)
	if !ok {
		return w.leafApartOtherwise(a, b)
	}
	alo, ahi := levels(pa, nil)
	blo, bhi := levels(pb, off.dz)
	if ahi.Cmp(blo) <= 0 || bhi.Cmp(alo) <= 0 {
		return true, nil
	}
	moved, ok := off.profile(pb.Profile)
	if !ok {
		return false, nil
	}
	return prismcells.SectionsApart(w.ctx, w.budget, []momentinput.Profile{pa.Profile}, []momentinput.Profile{moved})
}

// prismOffset is how a second prism's section and levels read in the first
// prism's coordinates, when both share one exact linear lift: its section
// moves by (du, dv) and its levels by dz.
type prismOffset struct {
	du, dv, dz *big.Rat
}

// liftOffset reports how b reads in a's coordinates. Both prisms must hold
// their sections and levels exactly and have the same exact linear lift
// (frame axes carried by the placement basis, compared in rationals); the
// translation between them then solves exactly into a section shift and a
// level shift.
func liftOffset(a, b prismrecord.Payload) (prismOffset, bool) {
	if a.SurfaceResult || b.SurfaceResult {
		return prismOffset{}, false
	}
	for _, d := range []float64{a.SectionDelta, b.SectionDelta, a.Z0Delta, a.Z1Delta, b.Z0Delta, b.Z1Delta} {
		if d != 0 {
			return prismOffset{}, false
		}
	}
	la, ta := prismLift(a)
	lb, tb := prismLift(b)
	for i := range 3 {
		for k := range 3 {
			if la[i][k].Cmp(lb[i][k]) != 0 {
				return prismOffset{}, false
			}
		}
	}
	w, ok := solve(la, subVec(tb, ta))
	if !ok {
		return prismOffset{}, false
	}
	return prismOffset{du: w[0], dv: w[1], dz: w[2]}, true
}

// levels returns a prism's levels in rationals, shifted by dz when given.
func levels(p prismrecord.Payload, dz *big.Rat) (*big.Rat, *big.Rat) {
	lo, hi := ratOf(min(p.Z0, p.Z1)), ratOf(max(p.Z0, p.Z1))
	if dz != nil {
		lo.Add(lo, dz)
		hi.Add(hi, dz)
	}
	return lo, hi
}

// profile returns p moved by the offset's section shift, and false when a
// moved coordinate is not exactly a float or a segment is not a line, circle
// or arc.
func (o prismOffset) profile(p momentinput.Profile) (momentinput.Profile, bool) {
	if o.du.Sign() == 0 && o.dv.Sign() == 0 {
		return p, true
	}
	move := func(q sectionrecord.Point2) (sectionrecord.Point2, bool) {
		ru := new(big.Rat).Add(ratOf(q.U), o.du)
		rv := new(big.Rat).Add(ratOf(q.V), o.dv)
		u, _ := ru.Float64()
		v, _ := rv.Float64()
		if ratOf(u).Cmp(ru) != 0 || ratOf(v).Cmp(rv) != 0 {
			return q, false
		}
		return sectionrecord.Point2{U: u, V: v}, true
	}
	loop := func(l sectionrecord.LoopRecord) (sectionrecord.LoopRecord, bool) {
		out := sectionrecord.LoopRecord{Segments: make([]sectionrecord.CurveSegment, len(l.Segments))}
		for i, seg := range l.Segments {
			var ok bool
			switch g := seg.(type) {
			case sectionrecord.LineSeg:
				var a, b bool
				g.Start, a = move(g.Start)
				g.End, b = move(g.End)
				ok = a && b
				out.Segments[i] = g
			case sectionrecord.CircleSeg:
				g.Center, ok = move(g.Center)
				out.Segments[i] = g
			case sectionrecord.ArcSeg:
				var a, b, c bool
				g.Center, a = move(g.Center)
				g.Start, b = move(g.Start)
				g.End, c = move(g.End)
				ok = a && b && c
				out.Segments[i] = g
			default:
				ok = false
			}
			if !ok {
				return l, false
			}
		}
		return out, true
	}
	outer, ok := loop(p.Outer)
	if !ok {
		return p, false
	}
	moved := momentinput.Profile{Outer: outer}
	for _, h := range p.Holes {
		hole, ok := loop(h)
		if !ok {
			return p, false
		}
		moved.Holes = append(moved.Holes, hole)
	}
	return moved, true
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

// prismWithin reports whether prism inner lies in prism outer: one exact
// linear lift, levels held exactly with inner's within outer's after the
// offset, and a section Sketch proves inside outer's once moved by it.
func prismWithin(ctx context.Context, budget *proofbound.WorkBudget, inner, outer prismrecord.Payload) (bool, error) {
	off, ok := liftOffset(outer, inner)
	if !ok {
		return boxWithin(inner, outer), nil
	}
	ilo, ihi := levels(inner, off.dz)
	olo, ohi := levels(outer, nil)
	if ilo.Cmp(olo) < 0 || ihi.Cmp(ohi) > 0 {
		return false, nil
	}
	moved, ok := off.profile(inner.Profile)
	if !ok {
		return false, nil
	}
	return prismcells.SectionWithin(ctx, budget, []momentinput.Profile{moved}, []momentinput.Profile{outer.Profile})
}
