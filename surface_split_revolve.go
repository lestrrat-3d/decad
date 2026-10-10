package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file is Document.Split over the revolve family
// (docs/surface-intersection-design.md §11's PR5). The cell selection and the
// per-cell recording run through prismcells over the two operands' MERIDIAN
// views, as they do for Trim's revolve arm. Each
// selected cell becomes one solid revolvePayload over the target's own sweep,
// carrying its cell's δ_cut as sectionDelta with sectionWhole false: only the
// cut ends a cell records moved, and the solid build charges them through
// prismcells.TrimRevolveSegmentCharges (§7.2).

// splitRevolve is Split over a pair S1 has already routed to the revolve
// family. Each piece's axis gates rerun over the piece's own meridian about
// the target's oriented axis (revolveBlendAxis), so the snap allowances and
// the radial proof each piece's build reads belong to that piece.
func (d *Document) splitRevolve(ctx context.Context, budget *proofbound.WorkBudget, target, tool *Body) ([]*Body, error) {
	rcv, rcvView, tlView, err := admitSplitRevolvePair(budget, target, tool)
	if err != nil {
		return nil, err
	}
	cells, err := resolveSplit(ctx, budget, rcvView, tlView)
	if err != nil {
		return nil, err
	}
	ref := d.nextProducerID()
	bodies := make([]*Body, len(cells))
	for i, cell := range cells {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		work := freeform.NewFreeformWork()
		ax, err := revolveBlendAxis(ctx, rcv, cell.profile, work)
		if err != nil {
			return nil, err
		}
		bodies[i], err = evalRevolveContextWork(ctx, d, ref+producerID(i), revolvePayload{
			profile:     cell.profile,
			frame:       rcv.frame,
			ax:          ax,
			phi0:        rcv.phi0,
			phi1:        rcv.phi1,
			full:        rcv.full,
			den:         rcv.den,
			xform:       rcv.xform,
			radialProof: ax.RadialProof,
			// resolveSplit's own δ_cut for this cell is the whole of it: S4
			// and S7 zero every other term prism §7 derives, and a cell's
			// uncut edges record their entity's own data verbatim.
			sectionDelta: cell.sectionDelta,
		}, work)
		if err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commitMany(bodies, target, tool)
	return bodies, nil
}

// admitSplitRevolvePair runs S2-S4, S6 and S7 for a Split over the revolve
// family; S1 has already routed the pair here and S5 belongs to Trim alone.
// It returns the target's own revolve record beside the two MERIDIAN views
// resolveSplit consumes.
//
// S4's axis-clearance clause does not reach Split. It guards Trim and Extend,
// whose results are open-meridian sheets with no pole for a cut end on the
// axis; Split's pieces are solids, and the solid build places a pole for a
// meridian meeting the axis exactly as Revolve does, after revolveBlendAxis
// reruns the axis-contact audit over each piece.
func admitSplitRevolvePair(budget *proofbound.WorkBudget, target, tool *Body) (revolvePayload, prismPayload, prismPayload, error) {
	fail := func(format string, args ...any) (revolvePayload, prismPayload, prismPayload, error) {
		return revolvePayload{}, prismPayload{}, prismPayload{}, fmt.Errorf(format, args...)
	}
	pass := func(err error) (revolvePayload, prismPayload, prismPayload, error) {
		return revolvePayload{}, prismPayload{}, prismPayload{}, err
	}

	rcv, ok := target.payload.(revolvePayload)
	if !ok || target.Kind() != BodySolid || !target.IsSolid() {
		return fail(`%w: Split's target must be a solid revolve`, ErrUnsupported)
	}
	if tool.Kind() != BodySheet {
		return fail(`%w: Split's tool must be a sheet`, ErrUnsupported)
	}
	var tl revolvePayload
	switch p := tool.payload.(type) {
	case revolvePayload:
		tl = p
	case chainRevolvePayload:
		tl = p.revolve()
	default:
		return fail(`%w: Split's tool has no revolve meridian`, ErrUnsupported)
	}
	if trimOperandSectionDelta(target) != 0 || trimOperandSectionDelta(tool) != 0 {
		return fail(`%w: Split does not admit an operand carrying its own section displacement`, ErrUnsupported)
	}

	rcvView, tlView := rcv.meridian(), tl.meridian()

	// S2: neither operand's accumulated placement is a reflection.
	if rcvView.reflected() || tlView.reflected() {
		return fail(`%w: Split does not admit a reflected operand`, ErrUnsupported)
	}

	// S3: every segment of both meridians is a LineSeg, CircleSeg or ArcSeg.
	rcvAnalytic, err := prismcells.ProfileAnalytic(budget, rcv.profile)
	if err != nil {
		return pass(err)
	}
	tlAnalytic, err := prismcells.ProfileAnalytic(budget, tl.profile)
	if err != nil {
		return pass(err)
	}
	if !rcvAnalytic || !tlAnalytic {
		return fail(`%w: Split admits only line, circle and arc segments`, ErrUnsupported)
	}

	// S4's revolve arm: the plane frames and the resolved axis agree on the
	// stored floats, the eight-field comparison Trim's own takes.
	if rcv.frame != tl.frame {
		return fail(`%w: the target and tool do not spin about the same frame, exactly`, ErrUnsupported)
	}
	if !revolveAxisIdentical(rcv.ax, tl.ax) {
		return fail(`%w: the target and tool do not spin about the same axis, exactly`, ErrUnsupported)
	}

	// S6's revolve arm, on the stored endpoint floats: S4 has proven the two
	// axes identical, so no origin shift applies.
	if tl.phi0 > rcv.phi0 || tl.phi1 < rcv.phi1 {
		return fail(`%w: the tool's angular span [%v, %v] does not cover the target's [%v, %v]`,
			ErrUnsupported, tl.phi0, tl.phi1, rcv.phi0, rcv.phi1)
	}

	// S7's remaining two clauses.
	reexpress, err := prismcells.NewReexpression(prismPlacementOf(rcvView), prismPlacementOf(tlView))
	if err != nil {
		return pass(err)
	}
	if !reexpress.Identity {
		return fail(`%w: the target and tool do not share one frame and placement, so their re-expression is not the identity`, ErrUnsupported)
	}
	rcvWhole, err := trimProfileFullyWhole(budget, rcv.profile)
	if err != nil {
		return pass(err)
	}
	tlWhole, err := trimProfileFullyWhole(budget, tl.profile)
	if err != nil {
		return pass(err)
	}
	if !rcvWhole || !tlWhole {
		return fail(`%w: every segment the target or tool consumes must span its entity's own natural domain`, ErrUnsupported)
	}
	return rcv, rcvView, tlView, nil
}
