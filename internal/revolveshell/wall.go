package revolveshell

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// AxisWall is the wall region of a meridian with one on-axis walk
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
func AxisWall(budget *proofbound.WorkBudget, profile momentinput.Profile, ax revolveaxis.Frame,
	walks []survey2d.SideWalk, axisAt int, s, tmm, tDelta, tol float64,
	audit func(*proofbound.WorkBudget, momentinput.Profile, momentinput.Profile) error) (momentinput.Profile, float64, error) {
	n := len(walks)
	axisWalk := walks[axisAt]
	chain := make([]survey2d.SideWalk, 0, n-1)
	for k := 1; k < n; k++ {
		chain = append(chain, walks[(axisAt+k)%n])
	}
	segs := profile.Outer.Segments
	var kept []sectionrecord.CurveSegment
	for _, w := range chain {
		for _, si := range w.Segs {
			kept = append(kept, segs[si])
		}
	}

	off, err := offset2d.OffsetOpenChain(budget, chain, AxisCurve(ax),
		offset2d.OpenEnd{Mirror: true}, offset2d.OpenEnd{Mirror: true}, s, tmm, tol)
	if err != nil {
		return momentinput.Profile{}, 0, offset2d.InLoop(err, 0)
	}
	offChain, qB, qE, ends := off.Segs, off.QStart, off.QEnd, off.Ends
	pE := sectionrecord.Point2{U: axisWalk.StartU, V: axisWalk.StartV}
	pB := sectionrecord.Point2{U: axisWalk.EndU, V: axisWalk.EndV}
	z := func(p sectionrecord.Point2) float64 { return (p.U-ax.AU)*ax.DU + (p.V-ax.AV)*ax.DV }
	dir := 1.0
	if z(pB) < z(pE) {
		dir = -1.0
	}
	along := func(from, to sectionrecord.Point2) bool { return dir*(z(to)-z(from)) > 0 }
	ordered := along(pE, qE) && along(qE, qB) && along(qB, pB)
	if s < 0 {
		ordered = along(qE, pE) && along(pB, qB)
	}
	if !ordered {
		return momentinput.Profile{}, 0, fmt.Errorf(`%w: the offset meridian's ends do not land in order on its axis walk, so the offset crosses its own mirror image on the axis; a trimmed-offset kernel is not available (modify S11b)`, decaderr.ErrUnsupported)
	}

	// The cut-back offset closes along the axis and faces the §5 audit as a
	// section of its own: S8, then S11b (a crossing or contact, its closing axis
	// segment included), then S9.
	cavity := append(append([]sectionrecord.CurveSegment(nil), offChain...), sectionrecord.LineSeg{Start: qE, End: qB, TStart: 0, TEnd: 1})
	if err := audit(budget, profile, momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: cavity}}); err != nil {
		return momentinput.Profile{}, 0, err
	}
	delta, err := ChainSectionDelta(budget, chain, ax, ends, s, tmm, tDelta, tol)
	if err != nil {
		return momentinput.Profile{}, 0, err
	}

	var loop []sectionrecord.CurveSegment
	if s > 0 {
		back, err := offset2d.ReverseLoopRecordBudget(budget, sectionrecord.LoopRecord{Segments: offChain})
		if err != nil {
			return momentinput.Profile{}, 0, err
		}
		loop = append(loop, kept...)
		loop = append(loop, sectionrecord.LineSeg{Start: pE, End: qE, TStart: 0, TEnd: 1})
		loop = append(loop, back.Segments...)
		loop = append(loop, sectionrecord.LineSeg{Start: qB, End: pB, TStart: 0, TEnd: 1})
	} else {
		back, err := offset2d.ReverseLoopRecordBudget(budget, sectionrecord.LoopRecord{Segments: kept})
		if err != nil {
			return momentinput.Profile{}, 0, err
		}
		loop = append(loop, offChain...)
		loop = append(loop, sectionrecord.LineSeg{Start: qE, End: pE, TStart: 0, TEnd: 1})
		loop = append(loop, back.Segments...)
		loop = append(loop, sectionrecord.LineSeg{Start: pB, End: qB, TStart: 0, TEnd: 1})
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: loop}}, delta, nil
}

// SideWall is the open-chain wall section of a revolve meridian
// with a side opening (docs/modify-reach-design.md §9.3.2,
// docs/shell-opening-design.md §8). The kept chain K is the meridian less the
// removed run and its on-axis walk. Each end of K is either on the axis, where
// the offset takes the corner K makes with its mirror image and ends on the
// axis, or at the opening, where Table RO's rim — the removed neighbour
// walk's own carrier from K's end to its cut q with the offset K' — joins K to
// K'. The wall walks K, the rim at K's end, K' backward and the rim at K's
// start, inward; outward it walks K', the rim back to K's end, K backward and
// the rim out to K's start. An axis end's offset point must land on the axis
// walk on the material side, as AxisWall requires of both
// (S11b). The wall faces the §5 audit (S8, S11b, S9) before it is swept, and
// it returns the wall's section displacement (ChainSectionDelta), which
// charges every float cut: the interior miters, the axis joins and each
// opening's rim cut.
func SideWall(budget *proofbound.WorkBudget, profile momentinput.Profile, ax revolveaxis.Frame,
	walks []survey2d.SideWalk, axisAt int, removed map[int]struct{}, s, tmm, tDelta, tol float64,
	audit func(*proofbound.WorkBudget, momentinput.Profile, momentinput.Profile) error) (momentinput.Profile, float64, error) {
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
		return momentinput.Profile{}, 0, fmt.Errorf(`%w: the side opening leaves no kept chain`, decaderr.ErrDegenerate)
	}
	var chain []survey2d.SideWalk
	for k := range n {
		i := (begin + k) % n
		if _, ok := removed[i]; ok || i == axisAt {
			break
		}
		chain = append(chain, walks[i])
	}
	first, last := chain[0], chain[len(chain)-1]
	start := offset2d.OpenEnd{Mirror: true}
	if before := (begin + n - 1) % n; before != axisAt {
		start = offset2d.OpenEnd{Removed: walks[before]}
	}
	end := offset2d.OpenEnd{Mirror: true}
	if after := (begin + len(chain)) % n; after != axisAt {
		end = offset2d.OpenEnd{Removed: walks[after]}
	}
	segs := profile.Outer.Segments
	var kept []sectionrecord.CurveSegment
	for _, w := range chain {
		for _, si := range w.Segs {
			kept = append(kept, segs[si])
		}
	}
	off, err := offset2d.OffsetOpenChain(budget, chain, AxisCurve(ax), start, end, s, tmm, tol)
	if err != nil {
		return momentinput.Profile{}, 0, offset2d.InLoop(err, 0)
	}
	qS, qE := off.QStart, off.QEnd
	kS := sectionrecord.Point2{U: first.StartU, V: first.StartV}
	kE := sectionrecord.Point2{U: last.EndU, V: last.EndV}
	if start.Mirror || end.Mirror {
		axisWalk := walks[axisAt]
		pE := sectionrecord.Point2{U: axisWalk.StartU, V: axisWalk.StartV}
		pB := sectionrecord.Point2{U: axisWalk.EndU, V: axisWalk.EndV}
		z := func(p sectionrecord.Point2) float64 { return (p.U-ax.AU)*ax.DU + (p.V-ax.AV)*ax.DV }
		dir := 1.0
		if z(pB) < z(pE) {
			dir = -1.0
		}
		along := func(from, to sectionrecord.Point2) bool { return dir*(z(to)-z(from)) > 0 }
		var ordered bool
		switch {
		case start.Mirror && s > 0: // K leaves the axis at pB: qB lies on A short of it.
			ordered = along(pE, qS) && along(qS, pB)
		case start.Mirror:
			ordered = along(pB, qS)
		case s > 0: // K arrives on the axis at pE: qE lies on A past it.
			ordered = along(pE, qE) && along(qE, pB)
		default:
			ordered = along(qE, pE)
		}
		if !ordered {
			return momentinput.Profile{}, 0, fmt.Errorf(`%w: the offset meridian's end does not land on its axis walk, so the offset crosses its own mirror image on the axis; a trimmed-offset kernel is not available (modify S11b)`, decaderr.ErrUnsupported)
		}
	}
	// closing returns the segment joining K to K' at one end: the axis line
	// at an axis end, the rim at an opening end. fromK says it runs from K's
	// endpoint to the offset's.
	closing := func(e offset2d.OpenEnd, w survey2d.SideWalk, atEnd bool, j offset2d.Join, k, q sectionrecord.Point2, fromK bool) (sectionrecord.CurveSegment, error) {
		if e.Mirror {
			if fromK {
				return sectionrecord.LineSeg{Start: k, End: q, TStart: 0, TEnd: 1}, nil
			}
			return sectionrecord.LineSeg{Start: q, End: k, TStart: 0, TEnd: 1}, nil
		}
		return offset2d.RimSegment(w, e.Removed, atEnd, s, j, fromK)
	}
	inward := s > 0
	atK, err := closing(end, last, true, off.Ends[1].Join, kE, qE, inward)
	if err != nil {
		return momentinput.Profile{}, 0, err
	}
	atStart, err := closing(start, first, false, off.Ends[0].Join, kS, qS, !inward)
	if err != nil {
		return momentinput.Profile{}, 0, err
	}
	loop, err := offset2d.OpenChainWallLoop(budget, kept, off.Segs, []sectionrecord.CurveSegment{atK}, []sectionrecord.CurveSegment{atStart}, inward)
	if err != nil {
		return momentinput.Profile{}, 0, err
	}
	wall := momentinput.Profile{Outer: loop}
	if err := audit(budget, profile, wall); err != nil {
		return momentinput.Profile{}, 0, err
	}
	delta, err := ChainSectionDelta(budget, chain, ax, off.Ends, s, tmm, tDelta, tol)
	if err != nil {
		return momentinput.Profile{}, 0, err
	}
	return wall, delta, nil
}
