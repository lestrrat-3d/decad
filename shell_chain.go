package decad

import (
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file offsets an open chain K: the kept walks of a shell whose wall
// does not close around the section (docs/shell-opening-design.md §3). Each
// interior corner of K takes docs/modify-design.md §7's join, and each end
// takes the join its end kind names.

// chainEndKind names how one end of an open chain's offset closes.
type chainEndKind int

const (
	// axisEnd is an end on a revolve's axis: the offset takes the corner K
	// makes with its own mirror image there (offset2d.MirrorCornerJoin), so
	// it ends on the axis (docs/modify-reach-design.md §9.3.1).
	axisEnd chainEndKind = iota
	// openingEnd is an end at a side opening: the offset ends where the
	// removed neighbour walk's own carrier cuts it (offset2d.OpeningJoin,
	// docs/shell-opening-design.md Table RO).
	openingEnd
)

// chainEnd is one end of an open chain: its kind and, at an opening end, the
// removed walk beside it.
type chainEnd struct {
	kind    chainEndKind
	removed survey2d.SideWalk
}

// openChainOffset is offsetOpenChain's result: the offset chain K' in K's own
// walk order, its first and last points, and its two ends as the build took
// them, for openChainSectionDelta.
type openChainOffset struct {
	segs   []CurveSegment
	qStart Point2
	qEnd   Point2
	ends   [2]offset2d.ChainEnd
}

// offsetOpenChain offsets the open chain K by s*t. ax is the revolve axis an
// axis end reads; a chain with no axis end ignores it. A dropped walk is S11a
// and a miter that does not close is S11, as in offsetLoopBudget; an opening
// end's own refusals are offset2d.ErrOpeningCorner (SO1) and
// offset2d.ErrOpeningSpan (SO2). A walk at an opening end may run past its
// own end to the rim cut, and offset2d.OpenWalkConsumed decides its S11a
// (docs/shell-opening-design.md §2.4).
func offsetOpenChain(budget *proofbound.WorkBudget, chain []survey2d.SideWalk, ax axisFrame, start, end chainEnd, s, t float64) (openChainOffset, error) {
	m := len(chain)
	if m == 0 {
		return openChainOffset{}, fmt.Errorf(`%w: an offset chain holds no walks`, ErrDegenerate)
	}
	for _, w := range chain {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return openChainOffset{}, err
		}
		if w.IsCircular() {
			if _, ok := offsetRadius(w, s, t); !ok {
				return openChainOffset{}, errOffsetDrop
			}
		}
	}
	axis := revolveAxisCurve(ax)
	endJoin := func(e chainEnd, w survey2d.SideWalk, atEnd bool) (offset2d.Join, error) {
		if e.kind == axisEnd {
			return offset2d.MirrorCornerJoin(w, atEnd, axis, s, t, shellTol)
		}
		return offset2d.OpeningJoin(w, e.removed, atEnd, s, t, shellTol)
	}
	// joins[i] is the corner at chain[i]'s start; joins[m] is the corner at
	// the last walk's end.
	joins := make([]offset2d.Join, m+1)
	var err error
	joins[0], err = endJoin(start, chain[0], false)
	if err == nil {
		joins[m], err = endJoin(end, chain[m-1], true)
	}
	for i := 1; err == nil && i < m; i++ {
		if err = survey2d.WallBudgetStep(budget); err != nil {
			return openChainOffset{}, err
		}
		joins[i], err = offset2d.CornerJoin(chain[i-1], chain[i], s, t, shellTol)
	}
	switch {
	case errors.Is(err, offset2d.ErrNoDirection):
		return openChainOffset{}, fmt.Errorf(`%w: a corner walk has no direction`, ErrDegenerate)
	case errors.Is(err, offset2d.ErrNoIntersection):
		return openChainOffset{}, errOffsetTopology
	case err != nil:
		return openChainOffset{}, err
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
			return openChainOffset{}, err
		}
		head, tail := joins[i], joins[i+1]
		from, to := pt(head.M), pt(tail.M)
		if head.Arc {
			from = pt(head.PB)
		}
		if tail.Arc {
			to = pt(tail.PA)
		}
		// A walk at an opening end may run past its own end to the rim cut;
		// at an axis end or an interior corner OpenWalkConsumed reads as
		// WalkConsumed does.
		openStart := i == 0 && start.kind == openingEnd
		openEnd := i == m-1 && end.kind == openingEnd
		if offset2d.OpenWalkConsumed(w, offset2d.Point{U: from.U, V: from.V}, offset2d.Point{U: to.U, V: to.V}, openStart, openEnd, shellTol) {
			return openChainOffset{}, errOffsetDrop
		}
		seg, err := offsetWalkSegment(w, s, t, from, to)
		if err != nil {
			return openChainOffset{}, err
		}
		segs = append(segs, seg)
		if tail.Arc {
			segs = append(segs, arcAt(tail))
		}
	}
	chainEndOf := func(e chainEnd, j offset2d.Join) offset2d.ChainEnd {
		return offset2d.ChainEnd{Mirror: e.kind == axisEnd, Removed: e.removed, Join: j}
	}
	out := openChainOffset{
		segs: segs, qStart: pt(joins[0].M), qEnd: pt(joins[m].M),
		ends: [2]offset2d.ChainEnd{chainEndOf(start, joins[0]), chainEndOf(end, joins[m])},
	}
	if joins[0].Arc {
		out.qStart = pt(joins[0].PA)
	}
	if joins[m].Arc {
		out.qEnd = pt(joins[m].PB)
	}
	return out, nil
}

// offsetMirrorChain offsets the open chain K of a meridian whose two ends lie
// on the revolve axis (offsetOpenChain with two axis ends). It returns the
// offset chain in K's own walk order, which starts at qB and ends at qE, both
// on the axis, and the chain's two ends.
func offsetMirrorChain(budget *proofbound.WorkBudget, chain []survey2d.SideWalk, ax axisFrame, s, t float64) ([]CurveSegment, Point2, Point2, [2]offset2d.ChainEnd, error) {
	off, err := offsetOpenChain(budget, chain, ax, chainEnd{kind: axisEnd}, chainEnd{kind: axisEnd}, s, t)
	if err != nil {
		return nil, Point2{}, Point2{}, [2]offset2d.ChainEnd{}, err
	}
	return off.segs, off.qStart, off.qEnd, off.ends, nil
}

// chainSectionDelta is the section displacement of an open chain's offset
// (docs/modify-reach-design.md §9.3.1, docs/shell-opening-design.md §4.4):
// three times offset2d.ChainReach's largest reach over K's interior joins and
// its two ends, on offsetSectionDelta's own argument. line is the mirror line
// an axis end meets; a chain with two opening ends never reads it. K and its
// end vertices are the receiver's own record and move by nothing, so the
// figure is exactly zero wherever every join and cut encloses to the float the
// build holds, which keeps a right-angle shell Exact.
func chainSectionDelta(budget *proofbound.WorkBudget, chain []survey2d.SideWalk, line offset2d.MirrorLine, ends [2]offset2d.ChainEnd, s, t, tDelta float64) (float64, error) {
	amount, err := offsetAmount(s, t, tDelta)
	if err != nil {
		return 0, err
	}
	reach, err := offset2d.ChainReach(budget, chain, line, ends[0], ends[1], s, t, amount, shellTol)
	if err != nil {
		return 0, err
	}
	if reach == 0 {
		return 0, nil
	}
	delta := proofbound.ProductUpper(3, reach)
	if proofbound.IsNonFinite(delta) {
		return 0, errOffsetUnbounded
	}
	return delta, nil
}

// openChainWallLoop is the wall section's loop over an open chain
// (docs/shell-opening-design.md §3's W): inward it walks K, the closing
// segment atEnd at K's end, K' backward and the closing segment atStart at
// K's start; outward it walks K', atEnd back to K's end, K backward and
// atStart out to K's start. kept is K's record and offset K' in K's own walk
// order; each closing run of segments already runs the way the loop walks
// it.
func openChainWallLoop(budget *proofbound.WorkBudget, kept, offset, atEnd, atStart []CurveSegment, inward bool) (LoopRecord, error) {
	first, back := kept, offset
	if !inward {
		first, back = offset, kept
	}
	rev, err := reverseLoopRecordBudget(budget, LoopRecord{Segments: back})
	if err != nil {
		return LoopRecord{}, err
	}
	loop := make([]CurveSegment, 0, len(kept)+len(offset)+len(atEnd)+len(atStart))
	loop = append(loop, first...)
	loop = append(loop, atEnd...)
	loop = append(loop, rev.Segments...)
	loop = append(loop, atStart...)
	return LoopRecord{Segments: loop}, nil
}
