package surfacetrim

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// AdmitTrimPair runs S1-S7 (docs/surface-intersection-design.md §2.1) for a
// Trim over the prism family. Every miss is ErrUnsupported at the call —
// there is no mesh fallback for this family of operations (§2.1). A revolve
// pair never reaches here: Trim routes it to revolveplan.AdmitTrimPair through
// RevolvePair first, so S1 here refuses an unknown or a mixed pair.
func AdmitTrimPair(budget *proofbound.WorkBudget, receiver, tool *topo.Body) (prismrecord.Payload, prismrecord.Payload, error) {
	fail := func(format string, args ...any) (prismrecord.Payload, prismrecord.Payload, error) {
		return prismrecord.Payload{}, prismrecord.Payload{}, fmt.Errorf(format, args...)
	}
	// S1: both operands the same sweep family.
	rf, tf := FamilyOf(receiver), FamilyOf(tool)
	if rf == FamilyUnknown || tf == FamilyUnknown {
		return fail(`%w: Trim needs a body built by a straight sweep or a revolve`, decaderr.ErrUnsupported)
	}
	if rf != tf {
		return fail(`%w: the receiver's %s and the tool's %s share no generator`, decaderr.ErrUnsupported, rf, tf)
	}

	// S7's own section-displacement clause is checked here, ahead of the
	// closed-loop requirement below, and reads generically off EITHER a
	// prism or a chain receiver: a body this design already trimmed carries a
	// chain record with a nonzero SectionDelta, and a second Trim on it must
	// name THAT — its own displacement — rather than the open-walk shape every
	// chain receiver carries regardless (T173: "the refusal is the
	// displacement's doing, not the tool's").
	rcvDelta, tlDelta := SectionDelta(receiver), SectionDelta(tool)
	if rcvDelta != 0 || tlDelta != 0 {
		return fail(`%w: Trim does not admit an operand carrying its own section displacement (receiver %v mm, tool %v mm)`,
			decaderr.ErrUnsupported, rcvDelta, tlDelta)
	}

	// The receiver's section must be closed (§2.2; RS3 for the open-walk
	// case) and the tool's must be one closed, hole-free loop (S5). Both are
	// prism-family here, so an operand that is not a prism record is a chain
	// record — an open walk.
	rcv, rcvOk := receiver.Payload.(prismrecord.Payload)
	if !rcvOk {
		return fail(`%w: Trim's receiver section is an open walk, which closes onto nothing for the tool to trim`,
			decaderr.ErrUnsupported)
	}
	tl, tlOk := tool.Payload.(prismrecord.Payload)
	if !tlOk {
		return fail(`%w: Trim's tool section is an open walk, which bounds no region to keep a side of`,
			decaderr.ErrUnsupported)
	}

	if err := prismcells.AdmitSurfacePair(budget, prismcells.SurfaceTrim,
		prismcells.SurfaceOperandOf(rcv), prismcells.SurfaceOperandOf(tl)); err != nil {
		return prismrecord.Payload{}, prismrecord.Payload{}, err
	}
	return rcv, tl, nil
}

// AdmitExtendPair applies S1-S4, S6 and S7 to a chain receiver and a
// prism-family tool. The selected segment enters the scene at full domain;
// every other receiver segment stays outside it. A revolve pair never reaches
// here: Extend routes it to revolveplan.AdmitExtendPair through RevolvePair
// first, exactly as Trim routes one.
func AdmitExtendPair(budget *proofbound.WorkBudget, receiver, tool *topo.Body) (chainrecord.Payload, prismrecord.Payload, error) {
	fail := func(format string, args ...any) (chainrecord.Payload, prismrecord.Payload, error) {
		return chainrecord.Payload{}, prismrecord.Payload{}, fmt.Errorf(format, args...)
	}
	rf, tf := FamilyOf(receiver), FamilyOf(tool)
	if rf != tf || rf != FamilyPrism {
		return fail(`%w: Extend needs a receiver and tool sharing a straight-sweep generator (receiver %s, tool %s)`,
			decaderr.ErrUnsupported, rf, tf)
	}
	rcv, ok := receiver.Payload.(chainrecord.Payload)
	if !ok || receiver.Kind != topo.BodySheet {
		return fail(`%w: Extend's receiver needs an open section`, decaderr.ErrUnsupported)
	}
	var tl prismrecord.Payload
	switch p := tool.Payload.(type) {
	case prismrecord.Payload:
		tl = p
	case chainrecord.Payload:
		tl = p.Prism()
	default:
		return fail(`%w: Extend's tool has no prism section`, decaderr.ErrUnsupported)
	}
	if rcv.SectionDelta != 0 || SectionDelta(tool) != 0 {
		return fail(`%w: Extend does not admit an operand carrying its own section displacement`, decaderr.ErrUnsupported)
	}
	if err := prismcells.AdmitSurfacePair(budget, prismcells.SurfaceExtend,
		prismcells.SurfaceOperandOf(rcv.Prism()), prismcells.SurfaceOperandOf(tl)); err != nil {
		return chainrecord.Payload{}, prismrecord.Payload{}, err
	}
	return rcv, tl, nil
}

// AdmitSplitPair applies S1-S4, S6 and S7 to a solid prism target and a
// prism-family sheet. S5 belongs to Trim alone: Split reads no tool side. The
// closed-sheet and chain-sheet views share prismcells.BuildScene's input shape.
// A revolve pair never reaches here: Split routes it to its own arm through
// RevolvePair first.
func AdmitSplitPair(budget *proofbound.WorkBudget, target, tool *topo.Body) (prismrecord.Payload, prismrecord.Payload, error) {
	fail := func(format string, args ...any) (prismrecord.Payload, prismrecord.Payload, error) {
		return prismrecord.Payload{}, prismrecord.Payload{}, fmt.Errorf(format, args...)
	}
	rf, tf := FamilyOf(target), FamilyOf(tool)
	if rf != tf || rf != FamilyPrism {
		return fail(`%w: Split needs a solid and sheet sharing a straight-sweep generator (target %s, tool %s)`,
			decaderr.ErrUnsupported, rf, tf)
	}
	rcv, ok := target.Payload.(prismrecord.Payload)
	if !ok || target.Kind != topo.BodySolid || !target.Solid {
		return fail(`%w: Split's target must be a solid prism`, decaderr.ErrUnsupported)
	}
	if tool.Kind != topo.BodySheet {
		return fail(`%w: Split's tool must be a sheet`, decaderr.ErrUnsupported)
	}
	var tl prismrecord.Payload
	switch p := tool.Payload.(type) {
	case prismrecord.Payload:
		tl = p
	case chainrecord.Payload:
		tl = p.Prism()
	default:
		return fail(`%w: Split's tool has no prism section`, decaderr.ErrUnsupported)
	}
	if SectionDelta(target) != 0 || SectionDelta(tool) != 0 {
		return fail(`%w: Split does not admit an operand carrying its own section displacement`, decaderr.ErrUnsupported)
	}
	if err := prismcells.AdmitSurfacePair(budget, prismcells.SurfaceSplit,
		prismcells.SurfaceOperandOf(rcv), prismcells.SurfaceOperandOf(tl)); err != nil {
		return prismrecord.Payload{}, prismrecord.Payload{}, err
	}
	return rcv, tl, nil
}
