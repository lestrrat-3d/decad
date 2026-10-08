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
// offset2d.ErrOpeningSpan (SO2).
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
		if walkOffsetConsumed(w, from, to) {
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
