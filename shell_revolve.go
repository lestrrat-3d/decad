package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
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
// angular caps — sweeps reach §9.2's open-chain wall region instead
// (revolveShellSideWall). Every other selection refuses before a face is made:
// a kept angular cap and a holed meridian are SX8, and so is an effective
// meridian whose mirror union would hold a hole or whose offset reaches across
// the axis, a removed run that is not one proper connected run, one that
// leaves two wall regions, and an opening whose rim cannot follow the removed
// face.

// shellRevolve is Body.Shell's revolve receiver, from reach §4's stage 4 on.
// Stage 1 — the live receiver, the options, the magnitude and the selector —
// has already run in Shell. removed is nil for a WithNoOpenings full turn. s
// is +1 inward, −1 outward.
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
	var removedWalks map[int]struct{}
	if sides > 0 {
		removedWalks, err = revolveRemovedWalks(walks, axisAt, removedSegs)
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
		inradius, enough, err := revolveShellInradius(budget, rp, axisAt >= 0, tmm, tDelta)
		if err != nil {
			return nil, err
		}
		if err := requireSectionCavity(t, tmm, inradius, enough); err != nil {
			return nil, err
		}
	}

	var wall ProfileRecord
	switch {
	case sides > 0:
		wall, err = revolveShellSideWall(budget, rp, walks, axisAt, removedWalks, s, tmm)
	case axisAt < 0:
		wall, err = revolveShellOffAxisWall(budget, rp.profile, s, tmm)
	default:
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
// on the revolve axis (offsetOpenChain with both ends on the axis). It returns
// the offset chain in K's own walk order, which starts at qB and ends at qE,
// both on the axis.
func offsetMirrorChain(budget *proofbound.WorkBudget, chain []survey2d.SideWalk, ax axisFrame, s, t float64) ([]CurveSegment, Point2, Point2, error) {
	return offsetOpenChain(budget, chain, ax, true, true, s, t)
}

// offsetOpenChain offsets an open chain K by the corner rules of
// docs/modify-design.md §7: each interior corner by offset2d's CornerJoin.
// An end on the revolve axis (axisStart, axisEnd) takes the corner K makes
// with its own mirror image there (MirrorCornerJoin), so the offset ends on
// the axis; an end at a side opening (docs/modify-reach-design.md §9.2) takes
// the walk's own offset foot, so the exact normal segment of length t joins
// it back to K. It returns the offset chain in K's own walk order with its
// first and last points. A dropped walk is S11a and a miter that does not
// close is S11, as in offsetLoopBudget.
func offsetOpenChain(budget *proofbound.WorkBudget, chain []survey2d.SideWalk, ax axisFrame, axisStart, axisEnd bool, s, t float64) ([]CurveSegment, Point2, Point2, error) {
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
	// foot is an opening end's join: the walk's own offset foot, s*t along
	// its unit left normal at the end.
	foot := func(vU, vV, tu, tv float64) (offset2d.Join, error) {
		tx, ty, l := offset2d.Normalize(tu, tv)
		if l == 0 {
			return offset2d.Join{}, offset2d.ErrNoDirection
		}
		return offset2d.Join{VertU: vU, VertV: vV, M: offset2d.Point{U: vU + s*t*(-ty), V: vV + s*t*tx}}, nil
	}
	// joins[i] is the corner at chain[i]'s start; joins[m] is the corner at
	// the last walk's end.
	joins := make([]offset2d.Join, m+1)
	var err error
	if axisStart {
		joins[0], err = offset2d.MirrorCornerJoin(chain[0], false, axis, s, t, shellTol)
	} else {
		joins[0], err = foot(chain[0].StartU, chain[0].StartV, chain[0].TanInU, chain[0].TanInV)
	}
	if err == nil {
		last := chain[m-1]
		if axisEnd {
			joins[m], err = offset2d.MirrorCornerJoin(last, true, axis, s, t, shellTol)
		} else {
			joins[m], err = foot(last.EndU, last.EndV, last.TanOutU, last.TanOutV)
		}
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

// revolveRemovedWalks maps the removed side faces' segments onto the
// meridian's coalesced walks and checks reach §9.2's run rule: the removed
// walks are one proper connected run, and what remains is one kept chain. On
// a meridian with an on-axis walk the kept chain runs from one axis end, so
// the run must touch the axis walk at one end of the chain: a run between two
// kept pieces would leave two wall regions, which one revolve record does not
// hold. A walk the faces cover only in part is a coalescing this evaluator
// does not match. Each refusal is SX8, ErrUnsupported.
func revolveRemovedWalks(walks []survey2d.SideWalk, axisAt int, segs map[int]struct{}) (map[int]struct{}, error) {
	n := len(walks)
	removed := map[int]struct{}{}
	for i, w := range walks {
		hit := 0
		for _, si := range w.Segs {
			if _, ok := segs[si]; ok {
				hit++
			}
		}
		switch {
		case hit == 0:
		case hit == len(w.Segs) && i != axisAt:
			removed[i] = struct{}{}
		default:
			return nil, fmt.Errorf(`%w: a removed side face covers only part of a meridian walk (modify-reach SX8)`, ErrUnsupported)
		}
	}
	sides := n
	if axisAt >= 0 {
		sides--
	}
	if len(removed) == 0 || len(removed) >= sides {
		return nil, fmt.Errorf(`%w: a side opening must remove a proper run of the revolve's generated side faces, not all of them (modify-reach SX8)`, ErrUnsupported)
	}
	// Count the run's boundaries around the cycle of side walks (the axis
	// walk is not a side walk and closes the cycle).
	var order []int
	for k := 1; k <= n; k++ {
		i := (max(axisAt, 0) + k) % n
		if i != axisAt {
			order = append(order, i)
		}
	}
	in := func(k int) bool { _, ok := removed[order[k]]; return ok }
	runs := 0
	for k := range order {
		if in(k) && (k == 0 && axisAt >= 0 || !in((k+len(order)-1)%len(order))) {
			runs++
		}
	}
	if runs != 1 {
		return nil, fmt.Errorf(`%w: a side opening's removed faces must be one connected run (modify-reach SX8)`, ErrUnsupported)
	}
	if axisAt >= 0 && !in(0) && !in(len(order)-1) {
		return nil, fmt.Errorf(`%w: a side opening between two kept walks of a meridian on the axis leaves two wall regions, which one revolve does not hold (modify-reach SX8)`, ErrUnsupported)
	}
	return removed, nil
}

// requireOpeningRim admits one opening end of a kept chain only where §9.2's
// exact normal segment lies on the removed neighbour walk's own line: that
// walk is straight and meets the kept walk's tangent (tu, tv) at a right
// angle, the float dot product of the two directions exactly zero. Where the
// segment runs from the corner into the neighbour's span (along it at the
// kept chain's end, back along it at the start), the neighbour must be longer
// than t, so the segment ends on it. There the rim is the removed face's own
// cut through the wall. At any other corner the normal segment leaves that
// face's line — an inward wall at an acute corner reaches past the removed
// face, outside the receiver — so the opening is ErrUnsupported (§9.2's open
// question).
func requireOpeningRim(neighbour survey2d.SideWalk, tu, tv float64, atEnd bool, s, t float64) error {
	refuse := fmt.Errorf(`%w: a side opening's rim follows the removed face only where the removed walk is straight and meets the kept walk at a right angle (modify-reach §9.2)`, ErrUnsupported)
	if neighbour.IsCircular() {
		return refuse
	}
	du, dv := neighbour.TanInU, neighbour.TanInV
	if du*tu+dv*tv != 0 {
		return refuse
	}
	// The segment runs along s times the kept walk's left normal (−tv, tu).
	along := s*(-tv*du+tu*dv) > 0
	if atEnd != along {
		return nil
	}
	length := math.Hypot(neighbour.EndU-neighbour.StartU, neighbour.EndV-neighbour.StartV)
	if length <= t*(1+shellTol) {
		return fmt.Errorf(`%w: a side opening's removed walk is no longer than the shell thickness, so the rim would leave it (modify-reach §9.2)`, ErrUnsupported)
	}
	return nil
}

// revolveShellSideWall is reach §9.2's open-chain wall section on a revolve
// meridian with a side opening. The kept chain K is the meridian less the
// removed run and its on-axis walk; each end of K is either on the axis,
// where the offset takes the corner K makes with its mirror image and ends on
// the axis, or at the opening, where the exact normal segment of length t
// joins K to its offset K'. The wall walks K, the joining segment at K's end,
// K' backward and the joining segment at K's start, inward; outward it walks
// K', the segment back to K's end, K backward and the segment out from K's
// start. An axis end's offset point must land on the axis walk on the
// material side, as revolveShellAxisWall requires of both (S11b). The wall
// faces the §5 audit (S8, S11b, S9) before it is swept.
func revolveShellSideWall(budget *proofbound.WorkBudget, rp revolvePayload, walks []survey2d.SideWalk, axisAt int, removed map[int]struct{}, s, tmm float64) (ProfileRecord, error) {
	n := len(walks)
	// K runs from the first kept walk after the removed run (and after the
	// axis walk) to the last kept walk before it.
	begin := -1
	for k := range n {
		i := (k + 1) % n
		prev := k % n
		_, iRemoved := removed[i]
		_, prevRemoved := removed[prev]
		if i != axisAt && !iRemoved && (prevRemoved || prev == axisAt) {
			begin = i
			break
		}
	}
	if begin < 0 {
		return ProfileRecord{}, fmt.Errorf(`%w: the side opening leaves no kept chain`, ErrDegenerate)
	}
	var chain []survey2d.SideWalk
	for k := range n {
		i := (begin + k) % n
		if _, ok := removed[i]; ok || i == axisAt {
			break
		}
		chain = append(chain, walks[i])
	}
	axisStart := axisAt >= 0 && (begin+n-1)%n == axisAt
	axisEnd := axisAt >= 0 && (begin+len(chain))%n == axisAt
	// Each opening end's normal segment must lie on the removed neighbour's
	// own line, which is where the removed face leaves the wall's rim.
	if !axisStart {
		first := chain[0]
		if err := requireOpeningRim(walks[(begin+n-1)%n], first.TanInU, first.TanInV, false, s, tmm); err != nil {
			return ProfileRecord{}, err
		}
	}
	if !axisEnd {
		last := chain[len(chain)-1]
		if err := requireOpeningRim(walks[(begin+len(chain))%n], last.TanOutU, last.TanOutV, true, s, tmm); err != nil {
			return ProfileRecord{}, err
		}
	}
	segs := rp.profile.Outer.Segments
	var kept []CurveSegment
	for _, w := range chain {
		for _, si := range w.Segs {
			kept = append(kept, segs[si])
		}
	}
	offChain, qS, qE, err := offsetOpenChain(budget, chain, rp.ax, axisStart, axisEnd, s, tmm)
	if err != nil {
		return ProfileRecord{}, err
	}
	kS := Point2{U: chain[0].StartU, V: chain[0].StartV}
	kE := Point2{U: chain[len(chain)-1].EndU, V: chain[len(chain)-1].EndV}
	if axisStart || axisEnd {
		axisWalk := walks[axisAt]
		pE := Point2{U: axisWalk.StartU, V: axisWalk.StartV}
		pB := Point2{U: axisWalk.EndU, V: axisWalk.EndV}
		z := func(p Point2) float64 { return (p.U-rp.ax.aU)*rp.ax.dU + (p.V-rp.ax.aV)*rp.ax.dV }
		dir := 1.0
		if z(pB) < z(pE) {
			dir = -1.0
		}
		along := func(from, to Point2) bool { return dir*(z(to)-z(from)) > 0 }
		var ordered bool
		switch {
		case axisStart && s > 0: // K leaves the axis at pB: qB lies on A short of it.
			ordered = along(pE, qS) && along(qS, pB)
		case axisStart:
			ordered = along(pB, qS)
		case s > 0: // K arrives on the axis at pE: qE lies on A past it.
			ordered = along(pE, qE) && along(qE, pB)
		default:
			ordered = along(qE, pE)
		}
		if !ordered {
			return ProfileRecord{}, fmt.Errorf(`%w: the offset meridian's end does not land on its axis walk, so the offset crosses its own mirror image on the axis; a trimmed-offset kernel is not available (modify S11b)`, ErrUnsupported)
		}
	}
	line := func(a, b Point2) CurveSegment { return LineSeg{Start: a, End: b, TStart: 0, TEnd: 1} }
	var loop []CurveSegment
	if s > 0 {
		back, err := reverseLoopRecordBudget(budget, LoopRecord{Segments: offChain})
		if err != nil {
			return ProfileRecord{}, err
		}
		loop = append(loop, kept...)
		loop = append(loop, line(kE, qE))
		loop = append(loop, back.Segments...)
		loop = append(loop, line(qS, kS))
	} else {
		back, err := reverseLoopRecordBudget(budget, LoopRecord{Segments: kept})
		if err != nil {
			return ProfileRecord{}, err
		}
		loop = append(loop, offChain...)
		loop = append(loop, line(qE, kE))
		loop = append(loop, back.Segments...)
		loop = append(loop, line(kS, qS))
	}
	wall := ProfileRecord{Outer: LoopRecord{Segments: loop}}
	if err := auditOffsetSectionBudget(budget, rp.profile, wall); err != nil {
		return ProfileRecord{}, err
	}
	return wall, nil
}
