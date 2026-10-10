package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/units"
)

// This file is the revolve shell of docs/modify-reach-design.md §9.3 (Table RX
// row RX2, Table BX rows BX6 and BX7): Body.Shell on a revolvePayload receiver. A
// surface of revolution's normal lies in its meridian plane, so a wall of
// thickness t behind every kept face is the meridian's own offset swept over
// the receiver's unchanged angular interval. The offset is the same exact
// line/arc construction a prism shell runs (shell_offset.go), taken on the
// EFFECTIVE meridian: the recorded region where it stays off the axis, and
// otherwise the region united with its own mirror image across the axis and
// cut back to the non-negative radial half-plane, so a walk on the axis grows
// no wall.
//
// Three calls build. A partial turn with both angular caps removed sweeps the
// wall region over the receiver's own interval. A full turn under
// WithNoOpenings sweeps it a whole turn into a closed hollow body: the cavity
// wall is the offset's own swept surface, a void shell beside the outer one
// (revolve_build.go's fullRevolveShellsContext). A side opening — a connected
// run of generated side faces removed, on a full turn or beside both removed
// angular caps — sweeps the open-chain wall region closed at each opening by
// docs/shell-opening-design.md's Table RO rim instead (revolveShellSideWall).
// Every other selection refuses before a face is made: a kept angular cap and
// a holed meridian are SX8, and so is an effective meridian whose mirror union
// would hold a hole or whose offset reaches across the axis, a removed run that
// is not one proper connected run, and one that leaves two wall regions; an
// opening end at a smooth corner is that document's SO1, and a rim running past
// the removed walk's far end its SO2.

// shellRevolve is Body.Shell's revolve receiver, from reach §4's stage 4 on.
// Stage 1 — the live receiver, the options, the magnitude and the selector —
// has already run in Shell. removed is nil for a WithNoOpenings full turn. s
// is +1 inward, −1 outward.
func (b *Body) shellRevolve(ctx context.Context, rp revolvePayload, removed []*Face, s float64, t units.Value, tmm, tDelta float64) (*Body, error) {
	d := b.doc
	// The section-displacement guard, read as a modify refusal: the offset
	// is of the recorded meridian, and a meridian displaced from the one it
	// denotes has no proven offset (requireExactSection's prism reading).
	if err := requireExactRevolveSection(rp, "this evaluator's revolve shell"); err != nil {
		return nil, err
	}

	// Stage 4 (reach §4): the receiver and its targets.
	if len(rp.profile.Holes) > 0 {
		return nil, fmt.Errorf(`%w: a shell of a revolve whose meridian holds %d hole(s) is not supported; its offset would carry a hole-lining wall (modify-reach SX8)`, ErrUnsupported, len(rp.profile.Holes))
	}
	caps := prismCapsOf(b)
	var start, end bool
	sides := 0
	removedSegs := map[int]struct{}{}
	for _, f := range removed {
		switch {
		case caps.start != nil && f == caps.start:
			start = true
		case caps.end != nil && f == caps.end:
			end = true
		default:
			sides++
			if err := revolveSideFaceSegments(b, f, removedSegs); err != nil {
				return nil, err
			}
		}
	}
	if !rp.full && (!start || !end) {
		// A kept angular cap needs a wall at constant distance behind a plane
		// through the axis; that offset plane is no constant-angle cap, so no
		// change of the angular interval builds it (reach §9.3).
		return nil, fmt.Errorf(`%w: a partial-revolve shell must remove both angular caps; a kept cap's wall is a plane at constant distance from it, which no revolve holds (modify-reach SX8)`, ErrUnsupported)
	}

	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	loops, err := profileCornerLoopsBudget(budget, rp.profile)
	if err != nil {
		return nil, err
	}
	walks := loops[0].walks
	axisAt := -1
	for i, w := range walks {
		if !rp.ax.IsAxis(rp.ax.Walk(w.SegmentWalk)) {
			continue
		}
		if axisAt >= 0 {
			// Two separate on-axis walks: the meridian united with its mirror
			// image encloses the axis span between them, a hole.
			return nil, fmt.Errorf(`%w: the revolve meridian meets the axis along more than one walk, so its mirror union holds a hole (modify-reach SX8)`, ErrUnsupported)
		}
		axisAt = i
	}
	var removedWalks map[int]struct{}
	if sides > 0 {
		removedWalks, err = revolveshell.RemovedWalks(walks, axisAt, removedSegs)
		if err != nil {
			return nil, err
		}
	}

	// Stage 5 (reach §4; modify §8): S18, then S10's section limit, inward
	// only, on the effective meridian. A partial turn with both caps open and
	// a full turn both keep no angular floor, so the section limit is the
	// only one. A side opening opens the cavity through the removed faces, so
	// no section limit applies: the open chain's own offset, its S11a drop
	// gate and the §5 audit of its wall section decide it.
	if s > 0 && sides == 0 {
		inradius, enough, err := revolveshell.Inradius(budget, rp.profile, rp.ax, axisAt >= 0, tmm, tDelta, shellTol)
		if err != nil {
			return nil, err
		}
		if err := requireSectionCavity(t, tmm, inradius, enough); err != nil {
			return nil, err
		}
	}

	var wall profileRecord
	var delta float64
	switch {
	case sides > 0:
		wall, delta, err = revolveshell.SideWall(budget, rp.profile, rp.ax, walks, axisAt, removedWalks,
			s, tmm, tDelta, shellTol, auditOffsetSectionBudget)
	case axisAt < 0:
		wall, delta, err = revolveShellOffAxisWall(budget, rp.profile, s, tmm, tDelta)
	default:
		wall, delta, err = revolveshell.AxisWall(budget, rp.profile, rp.ax, walks, axisAt,
			s, tmm, tDelta, shellTol, auditOffsetSectionBudget)
	}
	if err != nil {
		if errors.Is(err, context.Canceled) {
			return nil, context.Canceled
		}
		if errors.Is(err, context.DeadlineExceeded) {
			return nil, context.DeadlineExceeded
		}
		return nil, err
	}

	// The wall region is a NEW record: its own side gate, axis-contact audit and
	// snap allowances are proven here about the receiver's oriented axis. The
	// construction keeps it on the axis's non-negative side except where the
	// offset itself reaches across, which is the meridian's offset meeting its
	// own mirror image — a merge this offset does not build.
	work := freeform.NewFreeformWork()
	ax, err := revolveBlendAxis(ctx, rp, wall, work)
	if err != nil {
		if errors.Is(err, ErrDegenerate) {
			return nil, fmt.Errorf(`%w: the shell's offset meridian reaches the revolve axis, where it would meet its own mirror image; a trimmed-offset kernel is not available (modify-reach SX8): %v`, ErrUnsupported, err)
		}
		return nil, err
	}

	ref := d.nextProducerID()
	body, err := evalRevolveContextWork(ctx, d, ref, revolvePayload{
		profile:      wall,
		frame:        rp.frame,
		ax:           ax,
		phi0:         rp.phi0,
		phi1:         rp.phi1,
		full:         rp.full,
		den:          rp.den,
		xform:        rp.xform,
		radialProof:  ax.RadialProof,
		sectionDelta: delta,
		sectionWhole: delta > 0,
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

// revolveShellOffAxisWall is the wall region of a meridian that is its own
// effective meridian: the tube section of modify Table B, {P, Q} inward and
// {Q, P} outward, the inner loop walked as a hole. The offset runs the prism
// shell's S11a construction and §5 audit (S8, S11b, S9) unchanged, and so does
// its displacement: the prism cup's offsetSectionDelta over the same loop.
func revolveShellOffAxisWall(budget *proofbound.WorkBudget, profile profileRecord, s, tmm, tDelta float64) (profileRecord, float64, error) {
	offset, err := offsetProfile(budget, profile, s, tmm)
	if err != nil {
		return profileRecord{}, 0, err
	}
	if err := auditOffsetSectionBudget(budget, profile, offset); err != nil {
		return profileRecord{}, 0, err
	}
	delta, err := offsetSectionDelta(budget, profile, s, tmm, tDelta)
	if err != nil {
		return profileRecord{}, 0, err
	}
	outer, inner := profile.Outer, offset.Outer
	if s < 0 {
		outer, inner = offset.Outer, profile.Outer
	}
	hole, err := offset2d.ReverseLoopRecordBudget(budget, inner)
	if err != nil {
		return profileRecord{}, 0, err
	}
	return profileRecord{Outer: outer, Holes: []loopRecord{hole}}, delta, nil
}

// revolveSideFaceSegments adds the recorded meridian segments a removed
// generated side face sweeps — the j of its own side(0,j) roles — to segs. A
// face naming no such role is not a side face of this revolve.
func revolveSideFaceSegments(b *Body, f *Face, segs map[int]struct{}) error {
	named := false
	for _, o := range f.origins {
		var li, j int
		if o.producer != b.origin.producer {
			continue
		}
		if n, _ := fmt.Sscanf(o.Role, "side(%d,%d)", &li, &j); n != 2 || li != 0 {
			continue
		}
		segs[j] = struct{}{}
		named = true
	}
	if !named {
		return fmt.Errorf(`%w: a removed face is neither an angular cap nor a generated side face of this revolve`, ErrUnsupported)
	}
	return nil
}
