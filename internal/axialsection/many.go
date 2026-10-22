package axialsection

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// UnionMany is the Union of axial extrusions that lie between the same two planes
// and share section axes (agent-docs/axial-section-boolean-design.md). The result
// is the loft of two identical sections, the 2D union of the operands' sections, so
// the loft's own routes read it. ok is false, with a nil error, for operands this
// route does not take, so the caller tries its next route. An error is a genuine
// refusal: sketch left a region or an edge uncertified.
//
// A pair of analytic prisms has its own exact route, so this one takes a set only
// when some operand's section holds a free-form segment.
func UnionMany(ctx context.Context, env topo.Env, bodies []*topo.Body) (*topo.Body, bool, error) {
	payloads := make([]topo.Payload, len(bodies))
	for i, b := range bodies {
		payloads[i] = b.Payload
	}
	return UnionManyPayloads(ctx, env, payloads)
}

// UnionManyPayloads is UnionMany over the operands' payloads alone. This route reads
// nothing of a body but its payload, so a caller that wants only this route can hand it
// payloads it has not built bodies for (loftrecord.Payload.WithPlacement).
func UnionManyPayloads(ctx context.Context, env topo.Env, payloads []topo.Payload) (*topo.Body, bool, error) {
	return unionMany(ctx, env, payloads, false)
}

// UnionManyPrisms is UnionMany for a set that holds no loft, which the other routes
// have already declined. The first unplaced prism supplies the two planes the result
// records (Operand.AsReference). It runs after the prism-pattern route so that route
// keeps every set it certifies; this one takes the sets whose root contact it
// cannot certify, such as a root arc that lies on the blank's rim.
func UnionManyPrisms(ctx context.Context, env topo.Env, bodies []*topo.Body) (*topo.Body, bool, error) {
	payloads := make([]topo.Payload, len(bodies))
	for i, b := range bodies {
		payloads[i] = b.Payload
	}
	return unionMany(ctx, env, payloads, true)
}

func unionMany(ctx context.Context, env topo.Env, payloads []topo.Payload, prismReference bool) (*topo.Body, bool, error) {
	if len(payloads) < 2 {
		return nil, false, nil
	}
	ops := make([]Operand, len(payloads))
	anyFreeform := false
	for i, payload := range payloads {
		op, ok, err := OperandOf(ctx, payload)
		if err != nil || !ok {
			return nil, false, err
		}
		ops[i] = op
		anyFreeform = anyFreeform || hasFreeform(op.Section)
	}
	if !anyFreeform {
		return nil, false, nil
	}
	if prismReference && !slices.ContainsFunc(ops, func(op Operand) bool { return op.FromLoft }) {
		// A set of prisms has no loft to supply the two planes the result records,
		// so an unplaced prism supplies them.
		i := slices.IndexFunc(ops, func(op Operand) bool { return op.Xform == r3.Identity() })
		if i < 0 {
			return nil, false, nil
		}
		ref, ok := ops[i].AsReference()
		if !ok {
			return nil, false, nil
		}
		ops[i] = ref
	}
	ops, ok, err := Place(ctx, ops)
	if err != nil || !ok {
		return nil, false, err
	}
	loft, ok := Shared(ops)
	if !ok {
		return nil, false, nil
	}
	sections := make([]momentinput.Profile, len(ops))
	for i, op := range ops {
		sections[i] = op.Section
	}
	section, area, ok, err := unionArea(ctx, sections)
	if err != nil || !ok {
		return nil, false, err
	}
	src := ops[loft]
	work0, work1 := newWorks(section)
	recordArea0, err := section.FalsifyLoftRecordedArea(area, work0)
	if err != nil {
		return nil, true, err
	}
	// The two records are the same section, so the second integral is the first: it gives
	// the same area and spends the same work.
	recordArea1 := recordArea0
	work1.Spent, work1.ReconstructionSpent = work0.Spent, work0.ReconstructionSpent
	body, err := loftrecord.Build(ctx, env, loftrecord.Payload{
		Profile0: section, Profile1: section,
		Plane0: src.Plane0, Plane1: src.Plane1,
		Frame0: src.Frame0, Frame1: src.Frame1,
		Xform:                       r3.Identity(),
		AuthenticatedReconstruction: true,
		JoinDelta:                   JoinDelta(ops, loft),
		RecordArea:                  [2]float64{recordArea0, recordArea1},
	}, proofbound.NewWorkBudget(ctx), work0, work1)
	if err != nil {
		return nil, true, err
	}
	return body, true, ctx.Err()
}

func newWorks(section momentinput.Profile) (*freeform.FreeformWork, *freeform.FreeformWork) {
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	p := uint64(len(section.Outer.Segments))
	for _, h := range section.Holes {
		p += uint64(len(h.Segments))
	}
	limit := loftmesh.StationWorkLimit(0, p)
	for _, w := range []*freeform.FreeformWork{work0, work1} {
		w.RaiseLimit(limit)
		w.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
	}
	return work0, work1
}

// hasFreeform reports whether any segment of the section is free-form.
func hasFreeform(p momentinput.Profile) bool {
	for _, loop := range append([]sectionrecord.LoopRecord{p.Outer}, p.Holes...) {
		for _, seg := range loop.Segments {
			switch seg.(type) {
			case sectionrecord.SplineSeg, sectionrecord.NURBSSeg, sectionrecord.ClosedSplineSeg, sectionrecord.FitSplineSeg:
				return true
			}
		}
	}
	return false
}
