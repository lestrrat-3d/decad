package offset2d

import (
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// ErrDrop reports a segment or loop consumed by a section offset (S11a).
var ErrDrop = fmt.Errorf(`%w: the offset drops a section feature; a trimmed-offset kernel is not available`, decaderr.ErrUnsupported)

// ErrTopology reports offset carriers that cannot close into a miter (S11).
var ErrTopology = fmt.Errorf(`%w: the offset changes the section's topology; a trimmed-offset kernel is not available`, decaderr.ErrUnsupported)

// CornerTopologyError is ErrTopology at one corner: the walks' offset carriers
// meeting at (U, V) do not intersect. Loop is the index of the section loop
// the corner sits on, or −1 where the caller has not named one (InLoop). It
// unwraps to ErrTopology, so errors.Is reads it as that sentinel.
type CornerTopologyError struct {
	U, V float64
	Loop int
}

func (e *CornerTopologyError) Error() string {
	where := fmt.Sprintf(`(%v, %v)`, e.U, e.V)
	if e.Loop >= 0 {
		where = fmt.Sprintf(`(%v, %v) on loop %d`, e.U, e.V, e.Loop)
	}
	return fmt.Sprintf(`%v: the offsets of the two walls meeting at %s do not intersect`, ErrTopology, where)
}

func (e *CornerTopologyError) Unwrap() error { return ErrTopology }

// InLoop names loop li on a CornerTopologyError that names no loop yet, and
// returns every other error unchanged.
func InLoop(err error, li int) error {
	var ce *CornerTopologyError
	if !errors.As(err, &ce) || ce.Loop >= 0 {
		return err
	}
	named := *ce
	named.Loop = li
	return &named
}

// BuildLoop offsets a coalesced section loop by s*t while retaining its walk
// sense. Circular walks and consumed straight walks are rejected before a
// record can be published. The corner rule is shared with JoinsBudget.
func BuildLoop(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, s, t, tol float64) ([]sectionrecord.CurveSegment, error) {
	n := len(walks)
	if n == 0 {
		return nil, fmt.Errorf(`%w: an offset loop holds no walks`, decaderr.ErrDegenerate)
	}
	for _, w := range walks {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		if w.IsCircular() {
			if _, ok := OffsetRadius(w, s, t, tol); !ok {
				return nil, ErrDrop
			}
		}
	}
	if n == 1 && walks[0].Closed {
		w := walks[0]
		rr, ok := OffsetRadius(w, s, t, tol)
		if !ok {
			return nil, ErrDrop
		}
		return []sectionrecord.CurveSegment{CircleSegment(w.CU, w.CV, rr, w.Th1 > w.Th0)}, nil
	}
	joins, err := SectionJoinsBudget(budget, walks, s, t, tol)
	if err != nil {
		return nil, err
	}
	var segs []sectionrecord.CurveSegment
	for i := range n {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		w := walks[i]
		start := joins[i].M
		if joins[i].Arc {
			start = joins[i].PB
		}
		j1 := joins[(i+1)%n]
		end := j1.M
		if j1.Arc {
			end = j1.PA
		}
		if WalkConsumed(w, start, end, tol) {
			return nil, ErrDrop
		}
		seg, err := WalkSegment(w, s, t, start, end, tol)
		if err != nil {
			return nil, err
		}
		segs = append(segs, seg)
		if j1.Arc {
			segs = append(segs, ArcSegment(
				sectionrecord.Point2{U: j1.VertU, V: j1.VertV},
				sectionrecord.Point2{U: j1.PA.U, V: j1.PA.V},
				sectionrecord.Point2{U: j1.PB.U, V: j1.PB.V}, s < 0))
		}
	}
	return segs, nil
}

// SectionJoinsBudget maps carrier refusals to section offset errors. Carriers
// that do not meet are a CornerTopologyError naming the corner; the caller
// names the loop with InLoop.
func SectionJoinsBudget(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, s, t, tol float64) ([]Join, error) {
	joins, err := JoinsBudget(budget, walks, s, t, tol)
	if errors.Is(err, ErrNoDirection) {
		return nil, fmt.Errorf(`%w: a corner walk has no direction`, decaderr.ErrDegenerate)
	}
	if errors.Is(err, ErrNoIntersection) {
		u, v := math.NaN(), math.NaN()
		var ce *cornerError
		if errors.As(err, &ce) {
			u, v = ce.u, ce.v
		}
		return nil, &CornerTopologyError{U: u, V: v, Loop: -1}
	}
	return joins, err
}

// WalkSegment records a trimmed line or concentric arc in the walk's sense.
func WalkSegment(w survey2d.SideWalk, s, t float64, start, end Point, tol float64) (sectionrecord.CurveSegment, error) {
	a := sectionrecord.Point2{U: start.U, V: start.V}
	b := sectionrecord.Point2{U: end.U, V: end.V}
	if !w.IsCircular() {
		return sectionrecord.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1}, nil
	}
	if _, ok := OffsetRadius(w, s, t, tol); !ok {
		return nil, ErrDrop
	}
	return ArcSegment(sectionrecord.Point2{U: w.CU, V: w.CV}, a, b, w.Th1 > w.Th0), nil
}

// CircleSegment records a full circle in the requested walk sense.
func CircleSegment(cu, cv, rr float64, ccw bool) sectionrecord.CurveSegment {
	seg := sectionrecord.CircleSeg{
		Center: sectionrecord.Point2{U: cu, V: cv}, Radius: units.Millimeters(rr), CCW: ccw,
	}
	if ccw {
		seg.TStart, seg.TEnd = 0, 1
	} else {
		seg.TStart, seg.TEnd = 1, 0
	}
	return seg
}

// ArcSegment records a directed arc from start to end about center.
func ArcSegment(center, start, end sectionrecord.Point2, ccw bool) sectionrecord.CurveSegment {
	if ccw {
		return sectionrecord.ArcSeg{Center: center, Start: start, End: end, TStart: 0, TEnd: 1}
	}
	return sectionrecord.ArcSeg{Center: center, Start: end, End: start, TStart: 1, TEnd: 0}
}
