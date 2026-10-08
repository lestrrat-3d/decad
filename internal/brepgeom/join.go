package brepgeom

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// Profile is the loop portion of a BRep face's recorded region.
type Profile struct {
	Outer sectionrecord.LoopRecord
	Holes []sectionrecord.LoopRecord
}

// JoinLoop makes every junction of a loop one point. A boolean's cut fragment
// records its carrier and a narrowed range, so adjacent fragments can walk to
// the same cut at different floats. A line's walked point wins because its
// fixed coordinate stays exact; otherwise the lexicographically smaller point
// wins so a loop and its reversal choose alike. Each segment is rewritten
// between its two junctions: a line whole, a circular fragment as an arc
// pinned about its recorded centre. The returned allowance is the largest
// distance of a chosen point from the junction it stands for: its distance
// from the point its own segment denotes there (boundarywalk.DenotedEndBound
// and DenotedStartBound: the walk's rounding, plus the radial residual at an
// arc's natural t = 1 end), and, where the two segments lie on different
// carriers, its proven distance from their exact crossing
// (CrossingOffsetUpper), +Inf where that crossing cannot be stated. Two
// fragments of one carrier — a line run, a circle cut at its seam — name no
// crossing and charge the first term alone. A whole closed segment is left
// alone.
func JoinLoop(loop sectionrecord.LoopRecord) (sectionrecord.LoopRecord, float64, error) {
	n := len(loop.Segments)
	if n < 2 {
		return loop, 0, nil
	}
	for _, seg := range loop.Segments {
		switch seg.(type) {
		case sectionrecord.LineSeg, sectionrecord.CircleSeg, sectionrecord.ArcSeg:
		default:
			// A free-form loop has no BRep face, so it has nothing to join.
			return loop, 0, nil
		}
	}
	walks := make([]survey2d.SegmentWalk, n)
	for i, seg := range loop.Segments {
		w, err := boundarywalk.WalkOf(seg, nil)
		if err != nil {
			return sectionrecord.LoopRecord{}, 0, err
		}
		walks[i] = w
	}
	joins := make([]sectionrecord.Point2, n)
	allow := 0.0
	met := true
	for i := range n {
		j := (i + 1) % n
		end := sectionrecord.Point2{U: walks[i].EndU, V: walks[i].EndV}
		start := sectionrecord.Point2{U: walks[j].StartU, V: walks[j].StartV}
		if end == start {
			joins[i] = end
			continue
		}
		met = false
		pick, bound := end, boundarywalk.DenotedEndBound(loop.Segments[i], walks[i])
		switch {
		case walks[j].IsLine() && !walks[i].IsLine():
			pick, bound = start, boundarywalk.DenotedStartBound(loop.Segments[j], walks[j])
		case walks[i].IsLine() && !walks[j].IsLine():
		case start.U < end.U || (start.U == end.U && start.V < end.V):
			pick, bound = start, boundarywalk.DenotedStartBound(loop.Segments[j], walks[j])
		}
		joins[i] = pick
		allow = math.Max(allow, proofbound.WalkEndBoundAllow(bound))
		if !SameCarrier(loop.Segments[i], loop.Segments[j]) {
			allow = math.Max(allow, CrossingOffsetUpper(loop.Segments[i], loop.Segments[j], pick))
		}
	}
	if met {
		return loop, 0, nil
	}
	out := sectionrecord.LoopRecord{Segments: make([]sectionrecord.CurveSegment, n)}
	for i, w := range walks {
		from, to := joins[(i+n-1)%n], joins[i]
		if w.IsLine() {
			out.Segments[i] = sectionrecord.LineSeg{Start: from, End: to, TStart: 0, TEnd: 1}
			continue
		}
		out.Segments[i] = joinedArc(sectionrecord.Point2{U: w.CU, V: w.CV}, from, to, w.Th1 > w.Th0)
	}
	return out, allow, nil
}

func joinedArc(center, start, end sectionrecord.Point2, ccw bool) sectionrecord.CurveSegment {
	if ccw {
		return sectionrecord.ArcSeg{Center: center, Start: start, End: end, TStart: 0, TEnd: 1}
	}
	return sectionrecord.ArcSeg{Center: center, Start: end, End: start, TStart: 1, TEnd: 0}
}

// JoinProfile applies JoinLoop to the outer loop and holes in record order.
func JoinProfile(p Profile) (Profile, float64, error) {
	outer, allow, err := JoinLoop(p.Outer)
	if err != nil {
		return Profile{}, 0, err
	}
	out := Profile{Outer: outer}
	for _, hole := range p.Holes {
		joined, a, err := JoinLoop(hole)
		if err != nil {
			return Profile{}, 0, err
		}
		out.Holes = append(out.Holes, joined)
		allow = math.Max(allow, a)
	}
	return out, allow, nil
}
