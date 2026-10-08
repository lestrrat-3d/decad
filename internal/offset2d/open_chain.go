package offset2d

import (
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file offsets an open chain K: the kept walks of a shell whose wall
// does not close around the section (docs/shell-opening-design.md §3). Each
// interior corner of K takes docs/modify-design.md §7's join, and each end
// takes the join its end kind names.

// OpenEnd names how one end of an open chain closes. A mirror end meets the
// revolve axis; an opening end meets its removed neighbour walk.
type OpenEnd struct {
	Mirror  bool
	Removed survey2d.SideWalk
}

// OpenChainOffset carries the offset chain in its source walk order, its first
// and last points, and the two end joins used for the displacement proof.
type OpenChainOffset struct {
	Segs         []CurveSegment
	QStart, QEnd Point2
	Ends         [2]ChainEnd
}

// OffsetOpenChain offsets the open chain K by s*t. ax is the revolve axis an
// axis end reads; a chain with no axis end ignores it. A dropped walk is S11a
// and a miter that does not close is S11, a CornerTopologyError naming the
// corner, as in BuildLoop; the caller names the loop with InLoop. An opening
// end's own refusals are ErrOpeningCorner (SO1) and
// ErrOpeningSpan (SO2). A walk at an opening end may run past its own end to
// the rim cut, and OpenWalkConsumed decides its S11a
// (docs/shell-opening-design.md §2.4).
func OffsetOpenChain(budget *proofbound.WorkBudget, chain []survey2d.SideWalk, axis Curve,
	start, end OpenEnd, s, t, tol float64,
) (OpenChainOffset, error) {
	m := len(chain)
	if m == 0 {
		return OpenChainOffset{}, fmt.Errorf(`%w: an offset chain holds no walks`, decaderr.ErrDegenerate)
	}
	for _, w := range chain {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return OpenChainOffset{}, err
		}
		if w.IsCircular() {
			if _, ok := OffsetRadius(w, s, t, tol); !ok {
				return OpenChainOffset{}, ErrDrop
			}
		}
	}
	endJoin := func(e OpenEnd, w survey2d.SideWalk, atEnd bool) (Join, error) {
		if e.Mirror {
			return MirrorCornerJoin(w, atEnd, axis, s, t, tol)
		}
		return OpeningJoin(w, e.Removed, atEnd, s, t, tol)
	}
	// joins[i] is the corner at chain[i]'s start; joins[m] is the corner at
	// the last walk's end.
	// cornerU, cornerV is the corner the last join resolved.
	joins := make([]Join, m+1)
	var err error
	cornerU, cornerV := chain[0].StartU, chain[0].StartV
	joins[0], err = endJoin(start, chain[0], false)
	if err == nil {
		cornerU, cornerV = chain[m-1].EndU, chain[m-1].EndV
		joins[m], err = endJoin(end, chain[m-1], true)
	}
	for i := 1; err == nil && i < m; i++ {
		if err = survey2d.WallBudgetStep(budget); err != nil {
			return OpenChainOffset{}, err
		}
		cornerU, cornerV = chain[i].StartU, chain[i].StartV
		joins[i], err = CornerJoin(chain[i-1], chain[i], s, t, tol)
	}
	switch {
	case errors.Is(err, ErrNoDirection):
		return OpenChainOffset{}, fmt.Errorf(`%w: a corner walk has no direction`, decaderr.ErrDegenerate)
	case errors.Is(err, ErrNoIntersection):
		return OpenChainOffset{}, &CornerTopologyError{U: cornerU, V: cornerV, Loop: -1}
	case err != nil:
		return OpenChainOffset{}, err
	}

	pt := func(p Point) Point2 { return Point2{U: p.U, V: p.V} }
	arcAt := func(j Join) CurveSegment {
		// The connector winds CCW outward (s < 0) and CW inward (s > 0), as in
		// BuildLoop.
		return ArcSegment(Point2{U: j.VertU, V: j.VertV}, pt(j.PA), pt(j.PB), s < 0)
	}
	var segs []CurveSegment
	if joins[0].Arc {
		segs = append(segs, arcAt(joins[0]))
	}
	for i, w := range chain {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return OpenChainOffset{}, err
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
		openStart := i == 0 && !start.Mirror
		openEnd := i == m-1 && !end.Mirror
		if OpenWalkConsumed(w, Point(from), Point(to), openStart, openEnd, tol) {
			return OpenChainOffset{}, ErrDrop
		}
		seg, err := WalkSegment(w, s, t, Point(from), Point(to), tol)
		if err != nil {
			return OpenChainOffset{}, err
		}
		segs = append(segs, seg)
		if tail.Arc {
			segs = append(segs, arcAt(tail))
		}
	}
	chainEndOf := func(e OpenEnd, j Join) ChainEnd {
		return ChainEnd{Mirror: e.Mirror, Removed: e.Removed, Join: j}
	}
	out := OpenChainOffset{
		Segs: segs, QStart: pt(joins[0].M), QEnd: pt(joins[m].M),
		Ends: [2]ChainEnd{chainEndOf(start, joins[0]), chainEndOf(end, joins[m])},
	}
	if joins[0].Arc {
		out.QStart = pt(joins[0].PA)
	}
	if joins[m].Arc {
		out.QEnd = pt(joins[m].PB)
	}
	return out, nil
}

// ChainSectionDelta is the section displacement of an open chain's offset
// (docs/modify-reach-design.md §9.3.1, docs/shell-opening-design.md §4.4):
// three times ChainReach's largest reach over K's interior joins and
// its two ends, on offsetSectionDelta's own argument. line is the mirror line
// an axis end meets; a chain with two opening ends never reads it. K and its
// end vertices are the receiver's own record and move by nothing, so the
// figure is exactly zero wherever every join and cut encloses to the float the
// build holds, which keeps a right-angle shell Exact.
func ChainSectionDelta(budget *proofbound.WorkBudget, chain []survey2d.SideWalk, line MirrorLine,
	ends [2]ChainEnd, s, t, tDelta, tol float64,
) (float64, error) {
	amount, err := OffsetAmount(s, t, tDelta)
	if err != nil {
		return 0, err
	}
	reach, err := ChainReach(budget, chain, line, ends[0], ends[1], s, t, amount, tol)
	if err != nil {
		return 0, err
	}
	if reach == 0 {
		return 0, nil
	}
	delta := proofbound.ProductUpper(3, reach)
	if proofbound.IsNonFinite(delta) {
		return 0, ErrUnbounded
	}
	return delta, nil
}

// OpenChainWallLoop is the wall section's loop over an open chain
// (docs/shell-opening-design.md §3's W): inward it walks K, the closing
// segment atEnd at K's end, K' backward and the closing segment atStart at
// K's start; outward it walks K', atEnd back to K's end, K backward and
// atStart out to K's start. kept is K's record and offset K' in K's own walk
// order; each closing run of segments already runs the way the loop walks
// it.
func OpenChainWallLoop(budget *proofbound.WorkBudget, kept, offset, atEnd, atStart []CurveSegment, inward bool) (LoopRecord, error) {
	first, back := kept, offset
	if !inward {
		first, back = offset, kept
	}
	rev, err := ReverseLoopRecordBudget(budget, LoopRecord{Segments: back})
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
