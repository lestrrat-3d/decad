package axialsection

import (
	"context"

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
	if len(bodies) < 2 {
		return nil, false, nil
	}
	ops := make([]Operand, len(bodies))
	anyFreeform := false
	for i, b := range bodies {
		op, ok, err := OperandOf(ctx, b.Payload)
		if err != nil || !ok {
			return nil, false, err
		}
		ops[i] = op
		anyFreeform = anyFreeform || hasFreeform(op.Section)
	}
	if !anyFreeform {
		return nil, false, nil
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
	recordArea1, err := section.FalsifyLoftRecordedArea(area, work1)
	if err != nil {
		return nil, true, err
	}
	body, err := loftrecord.Build(ctx, env, loftrecord.Payload{
		Profile0: section, Profile1: section,
		Plane0: src.Plane0, Plane1: src.Plane1,
		Frame0: src.Frame0, Frame1: src.Frame1,
		Xform:                       r3.Identity(),
		AuthenticatedReconstruction: true,
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
