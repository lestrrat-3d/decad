package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// This file is the revolve shell of docs/modify-reach-design.md §9.3 (Table RX
// row RX2, Table BX row BX7): Body.Shell on a revolvePayload receiver. A
// surface of revolution's normal lies in its meridian plane, so a wall of
// thickness t behind every kept face is the meridian's own offset swept over
// the receiver's unchanged angular interval. The offset is the same exact
// line/arc construction a prism shell runs (shell_offset.go), taken on the
// EFFECTIVE meridian: the recorded region where it stays off the axis, and
// otherwise the region united with its own mirror image across the axis and
// cut back to the non-negative radial half-plane, so a walk on the axis grows
// no wall.
//
// What builds is a partial turn with both angular caps removed and no side
// face removed. Every other selection refuses before a face is made: a side
// opening is S2 until the open-chain wall section of reach §9.2 lands, a kept
// angular cap and a holed meridian are SX8, and so is an effective meridian
// whose mirror union would hold a hole or whose offset reaches across the axis.

// shellRevolve is Body.Shell's revolve receiver, from reach §4's stage 4 on.
// Stage 1 — the live receiver, the options, the magnitude and the selector —
// has already run in Shell. s is +1 inward, −1 outward.
func (b *Body) shellRevolve(ctx context.Context, rp revolvePayload, removed []*Face, s float64, t units.Value, tmm, tDelta float64) (*Body, error) {
	d := b.doc
	// RS13's guard, read as a modify refusal: the offset is of the recorded
	// meridian, and a meridian displaced from the one it denotes has no proven
	// offset (requireExactSection's prism reading).
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
	for _, f := range removed {
		switch {
		case caps.start != nil && f == caps.start:
			start = true
		case caps.end != nil && f == caps.end:
			end = true
		default:
			sides++
		}
	}
	if !rp.full && (!start || !end) {
		// A kept angular cap needs a wall at constant distance behind a plane
		// through the axis; that offset plane is no constant-angle cap, so no
		// change of the angular interval builds it (reach §9.3).
		return nil, fmt.Errorf(`%w: a partial-revolve shell must remove both angular caps; a kept cap's wall is a plane at constant distance from it, which no revolve holds (modify-reach SX8)`, ErrUnsupported)
	}
	if sides > 0 {
		return nil, fmt.Errorf(`%w: a revolve shell that removes %d generated side face(s) needs the open-chain wall section of modify-reach §9.2, which is not built (S2)`, ErrUnsupported, sides)
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
		if !rp.ax.IsAxis(rp.ax.walk(w.SegmentWalk)) {
			continue
		}
		if axisAt >= 0 {
			// Two separate on-axis walks: the meridian united with its mirror
			// image encloses the axis span between them, a hole.
			return nil, fmt.Errorf(`%w: the revolve meridian meets the axis along more than one walk, so its mirror union holds a hole (modify-reach SX8)`, ErrUnsupported)
		}
		axisAt = i
	}

	// Stage 5 (reach §4; modify §8): S18, then S10's section limit, inward
	// only, on the effective meridian. A partial turn with both caps open keeps
	// no angular floor, so the section limit is the only one.
	if s > 0 {
		inradius, enough, err := revolveShellInradius(budget, rp, axisAt >= 0, tmm, tDelta)
		if err != nil {
			return nil, err
		}
		if err := requireSectionCavity(t, tmm, inradius, enough); err != nil {
			return nil, err
		}
	}

	var wall ProfileRecord
	if axisAt < 0 {
		wall, err = revolveShellOffAxisWall(budget, rp.profile, s, tmm)
	} else {
		wall, err = revolveShellAxisWall(budget, rp, walks, axisAt, s, tmm)
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
		profile:     wall,
		frame:       rp.frame,
		ax:          ax,
		phi0:        rp.phi0,
		phi1:        rp.phi1,
		full:        rp.full,
		den:         rp.den,
		xform:       rp.xform,
		radialProof: ax.radialProof,
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

// revolveShellInradius is S10's reading of the effective meridian. A meridian
// off the axis is its own effective meridian, read as a prism section is. One
// with an on-axis walk is read united with its mirror image across the axis,
// in axis coordinates (z, ρ) where the mirror is ρ ↦ −ρ exactly: a solid
// cylinder of radius R keeps a cavity for every thickness below min(R, H/2),
// not merely below the recorded half-section's own inradius.
func revolveShellInradius(budget *proofbound.WorkBudget, rp revolvePayload, onAxis bool, tmm, tDelta float64) (float64, bool, error) {
	if !onAxis {
		return sectionInradius(budget, rp.profile, tmm, tDelta)
	}
	loops, err := revolveLoops(budget, rp)
	if err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return 0, false, err
		}
		return 0, false, fmt.Errorf(`%w: this evaluator cannot read the shell meridian: %v`, ErrUnsupported, err)
	}
	loop := loops[0]
	n := len(loop)
	var elems []survey2d.SurveyElem
	var verts [][2]float64
	for i, w := range loop {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, false, err
		}
		if rp.ax.IsAxis(w.SegmentWalk) {
			continue
		}
		el, ok := walkElem(w.SegmentWalk)
		if !ok {
			return 0, false, fmt.Errorf(`%w: this evaluator cannot survey the shell meridian's curve type`, ErrUnsupported)
		}
		elems = append(elems, el, survey2d.MirrorElem(el))
		ends := [][2]float64{{w.StartU, w.StartV}}
		if rp.ax.IsAxis(loop[(i+1)%n].SegmentWalk) {
			// The walk's end on the axis is a corner of the mirror union too.
			ends = append(ends, [2]float64{w.EndU, w.EndV})
		}
		for _, p := range ends {
			verts = append(verts, p)
			if p[1] > 0 {
				verts = append(verts, [2]float64{p[0], -p[1]})
			}
		}
	}
	if err := requireWallSurveyWork(budget, len(elems), len(verts)); err != nil {
		return 0, false, err
	}
	inradius, err := wallSurveyInradius(budget, elems, verts)
	return inradius, false, err
}

// revolveShellOffAxisWall is the wall region of a meridian that is its own
// effective meridian: the tube section of modify Table B, {P, Q} inward and
// {Q, P} outward, the inner loop walked as a hole. The offset runs the prism
// shell's S11a construction and §5 audit (S8, S11b, S9) unchanged.
func revolveShellOffAxisWall(budget *proofbound.WorkBudget, profile ProfileRecord, s, tmm float64) (ProfileRecord, error) {
	offset, err := offsetProfile(budget, profile, s, tmm)
	if err != nil {
		return ProfileRecord{}, err
	}
	if err := auditOffsetSectionBudget(budget, profile, offset); err != nil {
		return ProfileRecord{}, err
	}
	outer, inner := profile.Outer, offset.Outer
	if s < 0 {
		outer, inner = offset.Outer, profile.Outer
	}
	hole, err := reverseLoopRecordBudget(budget, inner)
	if err != nil {
		return ProfileRecord{}, err
	}
	return ProfileRecord{Outer: outer, Holes: []LoopRecord{hole}}, nil
}

// revolveShellAxisWall is the wall region of a meridian with one on-axis walk
// A, from E (where the kept chain K arrives at the axis) to B (where K
// leaves it). Its effective meridian is P united with its mirror image, with A
// cancelled against its own mirrored reverse, and its offset is cut back to
// the non-negative half-plane: K's own offset, whose two end joins are the
// corners K makes with its mirror image at E and at B (offset2d's
// MirrorCornerJoin), so the mirror half is never built. The cut-back offset
// Q ends at qB and qE on the axis and closes along it.
//
// The wall walks K, then the axis from E to qE, then Q's chain backward, then
// the axis from qB to B — inward. Outward it walks Q's chain, the axis from qE
// to E, K backward, and the axis from B to qB. Either way A's span holds the
// four axis points in one order (E, qE, qB, B inward; qE, E, B, qB outward),
// and an offset whose ends land out of that order has crossed its own mirror
// image on the axis (S11b).
func revolveShellAxisWall(budget *proofbound.WorkBudget, rp revolvePayload, walks []survey2d.SideWalk, axisAt int, s, tmm float64) (ProfileRecord, error) {
	n := len(walks)
	axisWalk := walks[axisAt]
	chain := make([]survey2d.SideWalk, 0, n-1)
	for k := 1; k < n; k++ {
		chain = append(chain, walks[(axisAt+k)%n])
	}
	segs := rp.profile.Outer.Segments
	var kept []CurveSegment
	for _, w := range chain {
		for _, si := range w.Segs {
			kept = append(kept, segs[si])
		}
	}

	offChain, qB, qE, err := offsetMirrorChain(budget, chain, rp.ax, s, tmm)
	if err != nil {
		return ProfileRecord{}, err
	}
	pE := Point2{U: axisWalk.StartU, V: axisWalk.StartV}
	pB := Point2{U: axisWalk.EndU, V: axisWalk.EndV}
	z := func(p Point2) float64 { return (p.U-rp.ax.aU)*rp.ax.dU + (p.V-rp.ax.aV)*rp.ax.dV }
	dir := 1.0
	if z(pB) < z(pE) {
		dir = -1.0
	}
	along := func(from, to Point2) bool { return dir*(z(to)-z(from)) > 0 }
	ordered := along(pE, qE) && along(qE, qB) && along(qB, pB)
	if s < 0 {
		ordered = along(qE, pE) && along(pB, qB)
	}
	if !ordered {
		return ProfileRecord{}, fmt.Errorf(`%w: the offset meridian's ends do not land in order on its axis walk, so the offset crosses its own mirror image on the axis; a trimmed-offset kernel is not available (modify S11b)`, ErrUnsupported)
	}

	// The cut-back offset closes along the axis and faces the §5 audit as a
	// section of its own: S8, then S11b (a crossing or contact, its closing axis
	// segment included), then S9.
	cavity := append(append([]CurveSegment(nil), offChain...), LineSeg{Start: qE, End: qB, TStart: 0, TEnd: 1})
	if err := auditOffsetSectionBudget(budget, rp.profile, ProfileRecord{Outer: LoopRecord{Segments: cavity}}); err != nil {
		return ProfileRecord{}, err
	}

	var loop []CurveSegment
	if s > 0 {
		back, err := reverseLoopRecordBudget(budget, LoopRecord{Segments: offChain})
		if err != nil {
			return ProfileRecord{}, err
		}
		loop = append(loop, kept...)
		loop = append(loop, LineSeg{Start: pE, End: qE, TStart: 0, TEnd: 1})
		loop = append(loop, back.Segments...)
		loop = append(loop, LineSeg{Start: qB, End: pB, TStart: 0, TEnd: 1})
	} else {
		back, err := reverseLoopRecordBudget(budget, LoopRecord{Segments: kept})
		if err != nil {
			return ProfileRecord{}, err
		}
		loop = append(loop, offChain...)
		loop = append(loop, LineSeg{Start: qE, End: pE, TStart: 0, TEnd: 1})
		loop = append(loop, back.Segments...)
		loop = append(loop, LineSeg{Start: pB, End: qB, TStart: 0, TEnd: 1})
	}
	return ProfileRecord{Outer: LoopRecord{Segments: loop}}, nil
}

// offsetMirrorChain offsets the open chain K of a meridian whose two ends lie
// on the revolve axis, by the corner rules of docs/modify-design.md §7: each
// interior corner by offset2d's CornerJoin, and each end by the corner K makes
// with its own mirror image there (MirrorCornerJoin). It returns the offset
// chain in K's own walk order, which starts at qB and ends at qE, both on the
// axis. A dropped walk is S11a and a miter that does not close is S11, as in
// offsetLoopBudget.
func offsetMirrorChain(budget *proofbound.WorkBudget, chain []survey2d.SideWalk, ax axisFrame, s, t float64) ([]CurveSegment, Point2, Point2, error) {
	m := len(chain)
	if m == 0 {
		return nil, Point2{}, Point2{}, fmt.Errorf(`%w: an offset chain holds no walks`, ErrDegenerate)
	}
	for _, w := range chain {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, Point2{}, Point2{}, err
		}
		if w.IsCircular() {
			if _, ok := offsetRadius(w, s, t); !ok {
				return nil, Point2{}, Point2{}, errOffsetDrop
			}
		}
	}
	axis := offset2d.Curve{IsLine: true, PX: ax.aU, PY: ax.aV, DX: ax.dU, DY: ax.dV}
	// joins[i] is the corner at chain[i]'s start; joins[m] is the corner at
	// the last walk's end.
	joins := make([]offset2d.Join, m+1)
	var err error
	joins[0], err = offset2d.MirrorCornerJoin(chain[0], false, axis, s, t, shellTol)
	if err == nil {
		joins[m], err = offset2d.MirrorCornerJoin(chain[m-1], true, axis, s, t, shellTol)
	}
	for i := 1; err == nil && i < m; i++ {
		if err = survey2d.WallBudgetStep(budget); err != nil {
			return nil, Point2{}, Point2{}, err
		}
		joins[i], err = offset2d.CornerJoin(chain[i-1], chain[i], s, t, shellTol)
	}
	switch {
	case errors.Is(err, offset2d.ErrNoDirection):
		return nil, Point2{}, Point2{}, fmt.Errorf(`%w: a corner walk has no direction`, ErrDegenerate)
	case errors.Is(err, offset2d.ErrNoIntersection):
		return nil, Point2{}, Point2{}, errOffsetTopology
	case err != nil:
		return nil, Point2{}, Point2{}, err
	}

	pt := func(p offset2d.Point) Point2 { return Point2{U: p.U, V: p.V} }
	arcAt := func(j offset2d.Join) CurveSegment {
		// The connector winds CCW outward (s < 0) and CW inward (s > 0), as in
		// offsetLoopBudget.
		return arcSegment(Point2{U: j.VertU, V: j.VertV}, pt(j.PA), pt(j.PB), s < 0)
	}
	var segs []CurveSegment
	if joins[0].Arc {
		segs = append(segs, arcAt(joins[0]))
	}
	for i, w := range chain {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, Point2{}, Point2{}, err
		}
		head, tail := joins[i], joins[i+1]
		start, end := pt(head.M), pt(tail.M)
		if head.Arc {
			start = pt(head.PB)
		}
		if tail.Arc {
			end = pt(tail.PA)
		}
		if walkOffsetConsumed(w, start, end) {
			return nil, Point2{}, Point2{}, errOffsetDrop
		}
		seg, err := offsetWalkSegment(w, s, t, start, end)
		if err != nil {
			return nil, Point2{}, Point2{}, err
		}
		segs = append(segs, seg)
		if tail.Arc {
			segs = append(segs, arcAt(tail))
		}
	}
	qB, qE := pt(joins[0].M), pt(joins[m].M)
	if joins[0].Arc {
		qB = pt(joins[0].PA)
	}
	if joins[m].Arc {
		qE = pt(joins[m].PB)
	}
	return segs, qB, qE, nil
}
