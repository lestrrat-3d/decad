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

// AdmitTrimPair runs S2-S7 for a Trim over the revolve family; S1 has
// already routed the pair here. The root caller reads S7's section deltas
// from either payload shape before this revolve-only gate. It returns the
// receiver's record beside the two meridian views resolution consumes.
func AdmitTrimPair(ctx context.Context, budget *proofbound.WorkBudget,
	receiver, tool *topo.Body, rcvDelta, tlDelta float64,
) (Payload, prismrecord.Payload, prismrecord.Payload, error) {
	fail := func(format string, args ...any) (Payload, prismrecord.Payload, prismrecord.Payload, error) {
		return Payload{}, prismrecord.Payload{}, prismrecord.Payload{}, fmt.Errorf(format, args...)
	}

	// S7's section-displacement clause runs before either payload admission.
	if rcvDelta != 0 || tlDelta != 0 {
		return fail(`%w: Trim does not admit an operand carrying its own section displacement (receiver %v mm, tool %v mm)`,
			decaderr.ErrUnsupported, rcvDelta, tlDelta)
	}

	rcv, rcvOk := receiver.Payload.(Payload)
	if !rcvOk {
		return fail(`%w: Trim's receiver meridian is an open walk, which closes onto nothing for the tool to trim`, decaderr.ErrUnsupported)
	}
	tl, tlOk := tool.Payload.(Payload)
	if !tlOk {
		return fail(`%w: Trim's tool meridian is an open walk, which bounds no region to keep a side of`, decaderr.ErrUnsupported)
	}

	rcvView, tlView := rcv.Meridian(), tl.Meridian()

	// S2: neither operand's accumulated placement is a reflection.
	if rcvView.Reflected() || tlView.Reflected() {
		return fail(`%w: Trim does not admit a reflected operand`, decaderr.ErrUnsupported)
	}

	// S3: every segment of both meridians is a LineSeg, CircleSeg or ArcSeg.
	rcvAnalytic, err := prismcells.ProfileAnalytic(budget, rcv.Profile)
	if err != nil {
		return Payload{}, prismrecord.Payload{}, prismrecord.Payload{}, err
	}
	tlAnalytic, err := prismcells.ProfileAnalytic(budget, tl.Profile)
	if err != nil {
		return Payload{}, prismrecord.Payload{}, prismrecord.Payload{}, err
	}
	if !rcvAnalytic || !tlAnalytic {
		return fail(`%w: Trim admits only line, circle and arc segments; a free-form segment blinds sketch's whole-scene TExact gate`, decaderr.ErrUnsupported)
	}

	// S4's revolve arm. The plane frames and the resolved axis must agree
	// component-wise, and both meridians must stand clear of that axis.
	if rcv.Frame != tl.Frame {
		return fail(`%w: the receiver and tool do not spin about the same frame, exactly`, decaderr.ErrUnsupported)
	}
	if !AxisIdentical(rcv.Ax, tl.Ax) {
		return fail(`%w: the receiver and tool do not spin about the same axis, exactly`, decaderr.ErrUnsupported)
	}
	rcvClear, err := MeridianClearOfAxis(ctx, rcv)
	if err != nil {
		return Payload{}, prismrecord.Payload{}, prismrecord.Payload{}, err
	}
	tlClear, err := MeridianClearOfAxis(ctx, tl)
	if err != nil {
		return Payload{}, prismrecord.Payload{}, prismrecord.Payload{}, err
	}
	if !rcvClear || !tlClear {
		return fail(`%w: Trim needs both meridians clear of the revolve axis; one of them touches it, and the fragment a cut would leave there sweeps a pole this arm does not yet place`, decaderr.ErrUnsupported)
	}

	// S5: for Trim, the tool's meridian is one closed hole-free loop.
	if len(tl.Profile.Holes) != 0 {
		return fail(`%w: Trim's tool meridian carries a hole, whose interior this evaluator cannot yet distinguish as a side`, decaderr.ErrUnsupported)
	}

	// S6 compares stored angular endpoints after S4 proved equal axes.
	if tl.Phi0 > rcv.Phi0 || tl.Phi1 < rcv.Phi1 {
		return fail(`%w: the tool's angular span [%v, %v] does not cover the receiver's [%v, %v]`,
			decaderr.ErrUnsupported, tl.Phi0, tl.Phi1, rcv.Phi0, rcv.Phi1)
	}

	// S7's remaining two clauses.
	reexpress, err := prismcells.NewReexpression(rcvView.PlacementOperand(), tlView.PlacementOperand())
	if err != nil {
		return Payload{}, prismrecord.Payload{}, prismrecord.Payload{}, err
	}
	if !reexpress.Identity {
		return fail(`%w: the receiver and tool do not share one frame and placement, so their re-expression is not the identity`, decaderr.ErrUnsupported)
	}
	rcvWhole, err := prismcells.TrimProfileFullyWhole(budget, rcv.Profile.Outer, rcv.Profile.Holes)
	if err != nil {
		return Payload{}, prismrecord.Payload{}, prismrecord.Payload{}, err
	}
	tlWhole, err := prismcells.TrimProfileFullyWhole(budget, tl.Profile.Outer, tl.Profile.Holes)
	if err != nil {
		return Payload{}, prismrecord.Payload{}, prismrecord.Payload{}, err
	}
	if !rcvWhole || !tlWhole {
		return fail(`%w: every segment the receiver or tool consumes must span its entity's own natural domain`, decaderr.ErrUnsupported)
	}

	return rcv, rcvView, tlView, nil
}
