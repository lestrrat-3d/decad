package surfacetrim

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file is Split over the revolve family
// (agent-docs/surface-intersection-design.md §11's PR5). The cell selection and the
// per-cell recording run through prismcells over the two operands' MERIDIAN
// views, as they do for Trim's revolve arm. Each
// selected cell becomes one solid revolvePayload over the target's own sweep,
// carrying its cell's δ_cut as sectionDelta with sectionWhole false: only the
// cut ends a cell records moved, and the solid build charges them through
// prismcells.TrimRevolveSegmentCharges (§7.2).

// SplitRevolve is Split over a pair S1 has already routed to the revolve
// family. Each piece's axis gates rerun over the piece's own meridian about
// the target's oriented axis (revolveBlendAxis), so the snap allowances and
// the radial proof each piece's build reads belong to that piece.
func SplitRevolve(ctx context.Context, budget *proofbound.WorkBudget, env topo.Env, target, tool *topo.Body) ([]*topo.Body, error) {
	rcv, rcvView, tlView, err := admitSplitRevolvePair(budget, target, tool)
	if err != nil {
		return nil, err
	}
	cells, err := prismcells.SplitPieces(ctx, budget, rcvView, tlView)
	if err != nil {
		return nil, err
	}
	bodies := make([]*topo.Body, len(cells))
	for i, cell := range cells {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		work := freeform.NewFreeformWork()
		ax, err := rcv.BlendAxis(ctx, cell.Profile, work)
		if err != nil {
			return nil, err
		}
		bodies[i], err = revolveplan.Build(ctx, topo.Env{Doc: env.Doc, Ref: env.Ref + provenance.Producer(i), Alloc: env.Alloc}, revolveplan.Payload{
			Profile:     cell.Profile,
			Frame:       rcv.Frame,
			Ax:          ax,
			Phi0:        rcv.Phi0,
			Phi1:        rcv.Phi1,
			Full:        rcv.Full,
			Den:         rcv.Den,
			Xform:       rcv.Xform,
			RadialProof: ax.RadialProof,
			// prismcells.SplitPieces's own δ_cut for this cell is the whole of it: S4
			// and S7 zero every other term prism §7 derives, and a cell's
			// uncut edges record their entity's own data verbatim.
			SectionDelta: cell.SectionDelta,
		}, work)
		if err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return bodies, nil
}

// admitSplitRevolvePair runs S2-S4, S6 and S7 for a Split over the revolve
// family; S1 has already routed the pair here and S5 belongs to Trim alone.
// It returns the target's own revolve record beside the two MERIDIAN views
// prismcells.SplitPieces consumes.
//
// S4's axis-clearance clause does not reach Split. It guards Trim and Extend,
// whose results are open-meridian sheets with no pole for a cut end on the
// axis; Split's pieces are solids, and the solid build places a pole for a
// meridian meeting the axis exactly as Revolve does, after revolveBlendAxis
// reruns the axis-contact audit over each piece.
func admitSplitRevolvePair(budget *proofbound.WorkBudget, target, tool *topo.Body) (revolveplan.Payload, prismrecord.Payload, prismrecord.Payload, error) {
	fail := func(format string, args ...any) (revolveplan.Payload, prismrecord.Payload, prismrecord.Payload, error) {
		return revolveplan.Payload{}, prismrecord.Payload{}, prismrecord.Payload{}, fmt.Errorf(format, args...)
	}
	pass := func(err error) (revolveplan.Payload, prismrecord.Payload, prismrecord.Payload, error) {
		return revolveplan.Payload{}, prismrecord.Payload{}, prismrecord.Payload{}, err
	}

	rcv, ok := target.Payload.(revolveplan.Payload)
	if !ok || target.Kind != topo.BodySolid || !target.Solid {
		return fail(`%w: Split's target must be a solid revolve`, decaderr.ErrUnsupported)
	}
	if tool.Kind != topo.BodySheet {
		return fail(`%w: Split's tool must be a sheet`, decaderr.ErrUnsupported)
	}
	var tl revolveplan.Payload
	switch p := tool.Payload.(type) {
	case revolveplan.Payload:
		tl = p
	case revolveplan.ChainPayload:
		tl = p.Revolve()
	default:
		return fail(`%w: Split's tool has no revolve meridian`, decaderr.ErrUnsupported)
	}
	if SectionDelta(target) != 0 || SectionDelta(tool) != 0 {
		return fail(`%w: Split does not admit an operand carrying its own section displacement`, decaderr.ErrUnsupported)
	}

	rcvView, tlView := rcv.Meridian(), tl.Meridian()

	// S2: neither operand's accumulated placement is a reflection.
	if rcvView.Reflected() || tlView.Reflected() {
		return fail(`%w: Split does not admit a reflected operand`, decaderr.ErrUnsupported)
	}

	// S3: every segment of both meridians is a LineSeg, CircleSeg or ArcSeg.
	rcvAnalytic, err := prismcells.ProfileAnalytic(budget, rcv.Profile)
	if err != nil {
		return pass(err)
	}
	tlAnalytic, err := prismcells.ProfileAnalytic(budget, tl.Profile)
	if err != nil {
		return pass(err)
	}
	if !rcvAnalytic || !tlAnalytic {
		return fail(`%w: Split admits only line, circle and arc segments`, decaderr.ErrUnsupported)
	}

	// S4's revolve arm: the plane frames and the resolved axis agree on the
	// stored floats, the eight-field comparison Trim's own takes.
	if rcv.Frame != tl.Frame {
		return fail(`%w: the target and tool do not spin about the same frame, exactly`, decaderr.ErrUnsupported)
	}
	if !revolveplan.AxisIdentical(rcv.Ax, tl.Ax) {
		return fail(`%w: the target and tool do not spin about the same axis, exactly`, decaderr.ErrUnsupported)
	}

	// S6's revolve arm, on the stored endpoint floats: S4 has proven the two
	// axes identical, so no origin shift applies.
	if tl.Phi0 > rcv.Phi0 || tl.Phi1 < rcv.Phi1 {
		return fail(`%w: the tool's angular span [%v, %v] does not cover the target's [%v, %v]`,
			decaderr.ErrUnsupported, tl.Phi0, tl.Phi1, rcv.Phi0, rcv.Phi1)
	}

	// S7's remaining two clauses.
	reexpress, err := prismcells.NewReexpression(rcvView.PlacementOperand(), tlView.PlacementOperand())
	if err != nil {
		return pass(err)
	}
	if !reexpress.Identity {
		return fail(`%w: the target and tool do not share one frame and placement, so their re-expression is not the identity`, decaderr.ErrUnsupported)
	}
	rcvWhole, err := prismcells.TrimProfileFullyWhole(budget, rcv.Profile.Outer, rcv.Profile.Holes)
	if err != nil {
		return pass(err)
	}
	tlWhole, err := prismcells.TrimProfileFullyWhole(budget, tl.Profile.Outer, tl.Profile.Holes)
	if err != nil {
		return pass(err)
	}
	if !rcvWhole || !tlWhole {
		return fail(`%w: every segment the target or tool consumes must span its entity's own natural domain`, decaderr.ErrUnsupported)
	}
	return rcv, rcvView, tlView, nil
}
