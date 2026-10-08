package boundarywalk

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// DenotedStartBound is walk's StartBound measured against the point seg
// DENOTES at its recorded TStart (docs/evaluator-design.md §4). seg is the
// recorded segment the walk starts on: a coalesced line run's first segment.
// The walk's own bound already states that distance for every end except an
// arc's natural t == 1 end, which arcWalkEnd pins to the recorded End at zero
// bound while the arc denotes the point at Start's radius and End's angle.
// That end adds circularbounds.ArcRadialResidualUpper to both components.
func DenotedStartBound(seg CurveSegment, walk survey2d.SegmentWalk) proofbound.WalkEndBound {
	return arcNaturalEndBound(seg, walk.StartBound, true)
}

// DenotedEndBound is DenotedStartBound for the walk's end at the recorded
// TEnd of seg, the segment the walk ends on.
func DenotedEndBound(seg CurveSegment, walk survey2d.SegmentWalk) proofbound.WalkEndBound {
	return arcNaturalEndBound(seg, walk.EndBound, false)
}

func arcNaturalEndBound(seg CurveSegment, bound proofbound.WalkEndBound, start bool) proofbound.WalkEndBound {
	arc, ok := valueSegment(seg).(ArcSeg)
	if !ok {
		return bound
	}
	t := arc.TEnd
	if start {
		t = arc.TStart
	}
	if t != 1 {
		return bound
	}
	residual := circularbounds.ArcRadialResidualUpper(arc)
	if residual == 0 {
		return bound
	}
	return proofbound.WalkEndBound{U: proofbound.AbsSumUpper(bound.U, residual), V: proofbound.AbsSumUpper(bound.V, residual)}
}

// SameCircleSeam reports whether prev's end and next's start are one point of
// one recorded circle: two fragments of the same CircleSeg (equal centre and
// radius) meeting at one shared t, or at t = 1 and t = 0, which name the same
// angle. The two ends then denote the same point whatever their walks hold.
func SameCircleSeam(prev, next CurveSegment) bool {
	a, ok := valueSegment(prev).(CircleSeg)
	if !ok {
		return false
	}
	b, ok := valueSegment(next).(CircleSeg)
	if !ok || a.Center != b.Center || a.Radius != b.Radius {
		return false
	}
	ta, tb := a.TEnd, b.TStart
	return ta == tb || (ta == 1 && tb == 0) || (ta == 0 && tb == 1)
}

// JunctionVertex places a junction vertex where walk prev ends and walk next
// starts, and returns its held plane position with the per-component bound on
// its distance from the points BOTH neighbours denote there: next at its
// recorded start, and prev at its recorded end. prevSeg is the segment prev
// ends on and nextSeg the one next starts on. A single whole closed walk
// passes itself as both neighbours.
//
// The vertex sits at next's held start. At a cut junction the two neighbours
// denote different points: each records its own entity at its own recorded
// parameter, and neither parameter is the exact crossing
// (docs/evaluator-design.md §4). The vertex is within DenotedStartBound of
// next's point, and within |held − prev's held end| plus DenotedEndBound of
// prev's point, so the bound is the larger of the two in each component.
//
// Where both neighbours provably denote one point (SameCircleSeam), either
// distance alone reaches it, so the bound is the smaller, and the vertex sits
// at whichever held end is the closer proven one. A clockwise circle starts at
// t = 1, whose held point runs through a float 2π, and ends at t = 0, whose
// held point is the recorded centre plus radius. Its seam vertex therefore
// sits at the t = 0 end, which is exact wherever that sum is.
//
// Where the two held ends coincide the cross term is the other end's own bound
// verbatim, so a junction of two natural line ends at one recorded coordinate
// answers exactly zero. A non-finite component answers +Inf.
func JunctionVertex(prevSeg CurveSegment, prev survey2d.SegmentWalk, nextSeg CurveSegment, next survey2d.SegmentWalk) (float64, float64, proofbound.WalkEndBound) {
	own := DenotedStartBound(nextSeg, next)
	end := DenotedEndBound(prevSeg, prev)
	du, dv := next.StartU-prev.EndU, next.StartV-prev.EndV
	if !SameCircleSeam(prevSeg, nextSeg) {
		return next.StartU, next.StartV, proofbound.WalkEndBound{
			U: math.Max(own.U, reachAcross(du, end.U)),
			V: math.Max(own.V, reachAcross(dv, end.V)),
		}
	}
	atNext := proofbound.WalkEndBound{U: math.Min(own.U, reachAcross(du, end.U)), V: math.Min(own.V, reachAcross(dv, end.V))}
	atPrev := proofbound.WalkEndBound{U: math.Min(end.U, reachAcross(du, own.U)), V: math.Min(end.V, reachAcross(dv, own.V))}
	if math.Max(atPrev.U, atPrev.V) < math.Max(atNext.U, atNext.V) {
		return prev.EndU, prev.EndV, atPrev
	}
	return next.StartU, next.StartV, atNext
}

// reachAcross is |d| + bound rounded up, where d is the float difference of
// two held coordinates. AbsSumUpper's first step rounds |d| up by one ulp,
// which covers the subtraction's own rounding. A zero difference returns bound
// unchanged.
func reachAcross(d, bound float64) float64 {
	if d == 0 {
		return bound
	}
	return proofbound.AbsSumUpper(d, bound)
}

// valueSegment dereferences a pointer-typed segment (normalizeSegment). A
// segment it cannot normalize is returned as given; the kind switches above
// then match nothing.
func valueSegment(seg CurveSegment) CurveSegment {
	if v, err := normalizeSegment(seg); err == nil {
		return v
	}
	return seg
}
