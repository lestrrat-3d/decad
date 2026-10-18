package revolveplan

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// AdmitExtendPair runs S2-S4, S6 and S7 for an Extend over the revolve
// family; S1 has already routed the pair here. S5 does not reach an Extend,
// which reads no side. The root caller reads S7's section deltas before this
// revolve-only gate. It returns the receiver's chain record beside the two
// meridian views resolution consumes.
func AdmitExtendPair(ctx context.Context, budget *proofbound.WorkBudget,
	receiver, tool *topo.Body, rcvDelta, tlDelta float64,
) (ChainPayload, prismrecord.Payload, prismrecord.Payload, error) {
	fail := func(format string, args ...any) (ChainPayload, prismrecord.Payload, prismrecord.Payload, error) {
		return ChainPayload{}, prismrecord.Payload{}, prismrecord.Payload{}, fmt.Errorf(format, args...)
	}
	pass := func(err error) (ChainPayload, prismrecord.Payload, prismrecord.Payload, error) {
		return ChainPayload{}, prismrecord.Payload{}, prismrecord.Payload{}, err
	}

	// S7's section-displacement clause runs first. The root caller reads
	// either revolve payload shape, so a trimmed body names its own
	// displacement rather than the shape of its section.
	if rcvDelta != 0 || tlDelta != 0 {
		return fail(`%w: Extend does not admit an operand carrying its own section displacement (receiver %v mm, tool %v mm)`,
			decaderr.ErrUnsupported, rcvDelta, tlDelta)
	}

	rcv, rcvOk := receiver.Payload.(ChainPayload)
	if !rcvOk || receiver.Kind != topo.BodySheet {
		return fail(`%w: Extend's receiver needs an open meridian to lengthen; a closed one has no free end and its other free direction is its own sweep`, decaderr.ErrUnsupported)
	}
	// The FULL-revolution restriction, and it is a reading rather than a bound:
	// root extendRevolveEndSegment names a free end by the single-coedge
	// latitude loop fullRevLoops (build.go) puts on the first and last
	// wall. A PARTIAL revolution carries no such loop — each wall holds one loop
	// gathering its two cap edges beside the swept arcs, and which member of it
	// is the meridian's own free end is not a recorded fact — so the pair is
	// refused rather than resolved by position.
	if !rcv.Full {
		return fail(`%w: Extend over the revolve family admits a full revolution alone; a partially revolved ribbon's swept free edge shares one loop with its wall's two cap edges, and no recorded fact separates them`, decaderr.ErrUnsupported)
	}
	var tl Payload
	switch p := tool.Payload.(type) {
	case Payload:
		tl = p
	case ChainPayload:
		tl = p.Revolve()
	default:
		return fail(`%w: Extend's tool has no revolve meridian`, decaderr.ErrUnsupported)
	}

	rcvRev := rcv.Revolve()
	rcvView, tlView := rcvRev.Meridian(), tl.Meridian()

	// S2: neither operand's accumulated placement is a reflection.
	if rcvView.Reflected() || tlView.Reflected() {
		return fail(`%w: Extend does not admit a reflected operand`, decaderr.ErrUnsupported)
	}

	// S3: every segment of both meridians is a LineSeg, CircleSeg or ArcSeg.
	rcvAnalytic, err := prismcells.ProfileAnalytic(budget, rcvRev.Profile)
	if err != nil {
		return pass(err)
	}
	tlAnalytic, err := prismcells.ProfileAnalytic(budget, tl.Profile)
	if err != nil {
		return pass(err)
	}
	if !rcvAnalytic || !tlAnalytic {
		return fail(`%w: Extend admits only line, circle and arc segments; a free-form segment blinds sketch's whole-scene TExact gate`, decaderr.ErrUnsupported)
	}

	// S4's revolve arm, the identical eight-field comparison Trim's own takes.
	if rcv.Frame != tl.Frame {
		return fail(`%w: the receiver and tool do not spin about the same frame, exactly`, decaderr.ErrUnsupported)
	}
	if !AxisIdentical(rcv.Ax, tl.Ax) {
		return fail(`%w: the receiver and tool do not spin about the same axis, exactly`, decaderr.ErrUnsupported)
	}
	rcvClear, err := ChainClearOfAxis(ctx, rcv)
	if err != nil {
		return pass(err)
	}
	tlClear, err := MeridianClearOfAxis(ctx, tl)
	if err != nil {
		return pass(err)
	}
	if !rcvClear || !tlClear {
		return fail(`%w: Extend needs both meridians clear of the revolve axis; one of them touches it, and the end a lengthening would place there sweeps a pole this arm does not build`, decaderr.ErrUnsupported)
	}

	// S6's revolve arm: S4 has proven the two axes identical, so the prism
	// arm's origin shift has no counterpart and the comparison is on the
	// stored endpoint floats alone.
	if tl.Phi0 > rcv.Phi0 || tl.Phi1 < rcv.Phi1 {
		return fail(`%w: the tool's angular span [%v, %v] does not cover the receiver's [%v, %v]`,
			decaderr.ErrUnsupported, tl.Phi0, tl.Phi1, rcv.Phi0, rcv.Phi1)
	}

	// S7's remaining two clauses. Only the TOOL is required whole: §3.1 puts
	// the receiver's extended segment into the scene over its own full domain
	// and leaves every other receiver segment outside it, so a receiver
	// carrying narrowed segments elsewhere clears S7 for an Extend and refuses
	// it for a Trim.
	reexpress, err := prismcells.NewReexpression(rcvView.PlacementOperand(), tlView.PlacementOperand())
	if err != nil {
		return pass(err)
	}
	if !reexpress.Identity {
		return fail(`%w: the receiver and tool do not share one frame and placement, so their re-expression is not the identity`, decaderr.ErrUnsupported)
	}
	tlWhole, err := prismcells.TrimProfileFullyWhole(budget, tl.Profile.Outer, tl.Profile.Holes)
	if err != nil {
		return pass(err)
	}
	if !tlWhole {
		return fail(`%w: every segment the tool consumes must span its entity's own natural domain`, decaderr.ErrUnsupported)
	}

	return rcv, rcvView, tlView, nil
}
