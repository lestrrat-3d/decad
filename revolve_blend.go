package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the revolve junction rewrite of docs/modify-reach-design.md §7
// (Table RX row RX2): Body.Fillet and Body.Chamfer on a revolvePayload
// receiver. A revolve's meridian has the same corner-to-edge mapping a prism's
// section has — an off-axis corner between two consecutive walks sweeps a
// latitude Circle3 on a full turn and a junction Arc3 on a partial one — so
// blending that edge is the same 2D corner rewrite fillet.go and chamfer.go
// already run, applied to the recorded meridian, followed by the revolve's own
// axis gates on the rewritten meridian and a rebuild through evalRevolve with
// the receiver's frame, axis, angular interval and placement.
//
// A selected edge maps to its corner by matching the stored analytic edge
// against the junction circle the build stamps (revolvePayload.junctionCircle),
// never by a topology index. An edge that matches no junction — a cap edge of a
// partial turn, or the shared on-axis edge of a wedge — is SX5
// (ErrUnsupported).

// revolveBlendOp names one modify op's part in the shared rewrite: the role
// kind its blend wall carries ("fillet" or "chamfer", also the noun its
// refusals use), and the per-corner construction (computeFillet or
// computeChamfer, bound to its magnitudes). corner reads corner ci of loop,
// whose index in the recorded section is li, and the selected edge e that
// mapped to it; an asymmetric chamfer reads li and e to assign its two
// setbacks (docs/modify-reach-design.md §6), and every other construction
// ignores them.
type revolveBlendOp struct {
	kind   string
	corner func(loop cornerLoop, li, ci int, e *Edge) (*cornerBlend, error)
}

// revolveJunction is one off-axis junction of the receiver's axis-local
// coalesced meridian walk: the placed circle it sweeps, and the recorded
// segment whose start it sits at (the leaving walk's first segment), which is
// what maps it onto the plane-local corner walk the rewrite reads.
type revolveJunction struct {
	center, axis r3.Vec
	radius       units.Value
	seg          int
}

// blendRevolveJunctions is the shared body of Fillet and Chamfer on a
// revolvePayload receiver, from the receiver/target stage on
// (docs/modify-reach-design.md §4 stages 4 to 6, §7). Stage 1 — the live
// receiver, the magnitude and the selector — has already run in the caller.
func (b *Body) blendRevolveJunctions(ctx context.Context, sel EdgeSelector, edges []*Edge, rp revolvePayload, op revolveBlendOp) (*Body, error) {
	d := b.doc
	// The section-displacement guard, read as a modify refusal: the rewrite
	// is of the recorded meridian, and a meridian displaced from the one it
	// denotes has no proven rewrite (requireExactSection's prism reading).
	if err := requireExactRevolveSection(rp, "this evaluator's junction "+op.kind); err != nil {
		return nil, err
	}

	budget := proofbound.NewWorkBudget(ctx)
	loops, err := profileCornerLoopsBudget(budget, rp.profile)
	if err != nil {
		return nil, err
	}
	junctions, err := revolveJunctionsOf(ctx, rp)
	if err != nil {
		return nil, err
	}

	// Stage 4 (reach §4): every selected edge is a swept meridian junction
	// (SX5 otherwise), mapped to its (loop, corner) of the meridian.
	blendAt := make([]map[int]*cornerBlend, len(loops))
	for i := range blendAt {
		blendAt[i] = map[int]*cornerBlend{}
	}
	matched := make([]matchedCorner, 0, len(edges))
	for ei, e := range edges {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		li, ci, found, err := matchRevolveJunction(rp, junctions, loops, e)
		if err != nil {
			return nil, fmt.Errorf(`%w; selector %s, %s`, err, sel, selectedEdgeContext(ei, e))
		}
		if !found {
			return nil, fmt.Errorf(`%w: a %s of a revolve edge that is not a swept meridian junction (a cap edge, or an edge on the axis) is not supported; selector %s, %s`,
				ErrUnsupported, op.kind, sel, selectedEdgeContext(ei, e))
		}
		matched = append(matched, newMatchedCorner(ei, e, li, ci, loops[li]))
		blendAt[li][ci] = nil // marked; the blend is computed in stage 5
	}

	// Stage 5: the construction's own gates per corner — S4, then S5 for a
	// fillet (a chord has no S5).
	for _, corner := range matched {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		cb, err := op.corner(loops[corner.loop], corner.loop, corner.corner, corner.edge)
		if err != nil {
			return nil, fmt.Errorf(`%w; selector %s, %s`, err, sel, corner)
		}
		blendAt[corner.loop][corner.corner] = cb
	}

	profile, blendSegs, err := rewriteProfileBudget(budget, rp.profile, loops, blendAt)
	if err != nil {
		return nil, err
	}

	// Stage 6: the base §5 audit of the rewritten meridian (S8, S6, S7, S9).
	if err := auditRewriteBudget(budget, rp.profile, profile, loops, blendAt); err != nil {
		if errors.Is(err, context.Canceled) {
			return nil, context.Canceled
		}
		if errors.Is(err, context.DeadlineExceeded) {
			return nil, context.DeadlineExceeded
		}
		return nil, wrapModifyAuditError(sel, matched, err)
	}

	// The rewritten meridian is a NEW record: no axis resolution has seen it,
	// so its own side gate, axis-contact audit and snap allowances are proven
	// here, and the build below continues the same free-form work counter
	// (docs/spline-design.md §5.2).
	work := freeform.NewFreeformWork()
	ax, err := revolveBlendAxis(ctx, rp, profile, work)
	if err != nil {
		return nil, wrapModifyAuditError(sel, matched, err)
	}

	ref := d.nextProducerID()
	body, err := evalRevolveContextWork(ctx, d, ref, revolvePayload{
		profile:     profile,
		frame:       rp.frame,
		ax:          ax,
		phi0:        rp.phi0,
		phi1:        rp.phi1,
		full:        rp.full,
		den:         rp.den,
		xform:       rp.xform,
		radialProof: ax.radialProof,
		blendSegs:   blendSegs,
		blendKind:   op.kind,
	}, work)
	if err != nil {
		return nil, err
	}
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body, b)
	return body, nil
}

// revolveBlendAxis reruns the revolve axis gates (docs/evaluator-design.md §6)
// on a rewritten meridian about the receiver's own oriented axis: the side
// gate, the axis-contact audit and the snap allowances, each with the sentinel
// the base Revolve call gives it — a new interior axis contact is
// ErrDegenerate, a blend arc centred across the axis (a spindle torus) is
// ErrUnsupported. The receiver's axis already has the region on its
// non-negative side, so a rewrite whose region the gate puts on the far side
// has crossed the axis and is ErrDegenerate.
func revolveBlendAxis(ctx context.Context, rp revolvePayload, profile ProfileRecord, work *freeform.FreeformWork) (axisFrame, error) {
	ax, side, err := resolveAxisSide(ctx, profile, axisLine2{
		aU: rp.ax.aU, aV: rp.ax.aV, aUBound: rp.ax.aUBound, aVBound: rp.ax.aVBound,
		dU: rp.ax.dU, dV: rp.ax.dV, dUBound: rp.ax.dUBound, dVBound: rp.ax.dVBound,
	}, work)
	if err != nil {
		return axisFrame{}, err
	}
	if side < 0 {
		return axisFrame{}, fmt.Errorf(`%w: the rewritten meridian lies across the revolve axis from the receiver's`, ErrDegenerate)
	}
	return ax, nil
}

// revolveJunctionsOf lists every off-axis junction of the receiver, per loop,
// in the axis-local coalesced walk the build itself reads (revolveaxis.ResolveLoop).
// A loop that is one whole closed curve has none, and a junction on the axis
// sweeps a point rather than an edge.
func revolveJunctionsOf(ctx context.Context, rp revolvePayload) ([][]revolveJunction, error) {
	work := freeform.NewFreeformWork()
	basis := rp.basis()
	loops := append([]LoopRecord{rp.profile.Outer}, rp.profile.Holes...)
	out := make([][]revolveJunction, len(loops))
	for li, loop := range loops {
		resolved, err := revolveaxis.ResolveLoop(ctx, loop, work, "the revolve junction blend",
			rp.chargedWalk, rp.ax.snapTol)
		if err != nil {
			return nil, err
		}
		if resolved.SingleClosed {
			continue
		}
		for _, w := range resolved.Walks {
			if w.StartV == 0 || len(w.Segs) == 0 {
				continue
			}
			out[li] = append(out[li], newRevolveJunction(rp, basis, w.StartU, w.StartV, w.Segs[0]))
		}
	}
	return out, nil
}

func newRevolveJunction(rp revolvePayload, basis revolvemesh.RevolveBasis, z, rho float64, seg int) revolveJunction {
	center, axis, radius := rp.junctionCircle(basis, z, rho)
	return revolveJunction{center: center, axis: axis, radius: radius, seg: seg}
}

// matchRevolveJunction maps a selected edge to the (loop, corner) of the
// plane-local corner walk whose junction swept it. The edge must carry the
// curve kind the receiver's sweep stamps — a Circle3 on a full turn, an Arc3
// on a partial one — with the centre, axis and radius of one junction circle
// exactly (both radii are millimetre values, so their magnitudes compare
// directly): the build stamps both from revolvePayload.junctionCircle over the
// same payload, so an equal edge is that junction's own and a cap edge (whose
// circle lies in a cap plane) or any other curve matches none. found is false
// for every such edge. A junction whose leaving walk starts at a recorded
// segment no plane-local corner starts at — the two coalescings disagreeing
// about a near-collinear run — is refused as ErrUnsupported rather than mapped
// to a neighbouring corner.
func matchRevolveJunction(rp revolvePayload, junctions [][]revolveJunction, loops []cornerLoop, e *Edge) (int, int, bool, error) {
	var center, axis r3.Vec
	var radius units.Value
	switch c := e.curve.(type) {
	case Circle3:
		if !rp.full {
			return 0, 0, false, nil
		}
		center, axis, radius = c.Center, c.Axis, c.Radius
	case Arc3:
		if rp.full {
			return 0, 0, false, nil
		}
		center, axis, radius = c.Center, c.Axis, c.Radius
	default:
		return 0, 0, false, nil
	}
	for li, js := range junctions {
		for _, j := range js {
			if j.center != center || j.axis != axis || j.radius.Mag() != radius.Mag() {
				continue
			}
			for ci, w := range loops[li].walks {
				if len(w.Segs) > 0 && w.Segs[0] == j.seg {
					return li, ci, true, nil
				}
			}
			return 0, 0, false, fmt.Errorf(`%w: the revolve junction at recorded segment %d of loop %d is no corner of the recorded meridian's own walk`, ErrUnsupported, j.seg, li)
		}
	}
	return 0, 0, false, nil
}
