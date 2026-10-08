package offset2d

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ErrUnbounded reports an offset section whose displacement cannot be enclosed.
var ErrUnbounded = fmt.Errorf(`%w: this evaluator cannot prove how far the shell's offset section sits from the offset it denotes, so the cup's readings would carry no bound`, decaderr.ErrUnsupported)

// LoopReach is the largest reach of one loop's recorded offset points from
// their enclosures in the section displacement proof (modify §9).
func LoopReach(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, s, t float64, amount proofbound.RatInterval, tol float64) (float64, error) {
	n := len(walks)
	if n == 0 {
		return 0, fmt.Errorf(`%w: an offset loop holds no walks`, decaderr.ErrDegenerate)
	}
	if n == 1 && walks[0].Closed {
		// A concentric circle: every recorded point sits at the held radius
		// about the exact centre, so the radial gap is the whole displacement.
		w := walks[0]
		held, ok := OffsetRadius(w, s, t, tol)
		if !ok {
			return 0, ErrDrop
		}
		r, ok := capcontour.OffsetCircleRadius(w, amount)
		if !ok {
			return 0, ErrUnbounded
		}
		gap, ok := capcontour.AxisSpread(r, held)
		if !ok {
			return 0, ErrUnbounded
		}
		return proofbound.RatFloatUp(gap), nil
	}
	joins, err := SectionJoinsBudget(budget, walks, s, t, tol)
	if err != nil {
		return 0, err
	}
	reach := 0.0
	for _, w := range walks {
		// A recorded arc's end is pinned to its record and may sit off the
		// circle its start fixes; that radial gap moves the denoted foot off
		// the denoted carrier, so it joins the reach the arc argument reads.
		if !w.IsCircular() {
			continue
		}
		gap, ok := capcontour.CircularWalkEndGap(w)
		if !ok {
			return 0, ErrUnbounded
		}
		reach = math.Max(reach, gap)
	}
	for i, j := range joins {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, err
		}
		prev, cur := walks[(i+n-1)%n], walks[i]
		end, okE := capcontour.WalkPointEnclosure(prev.EndU, prev.EndV, prev.EndBound)
		start, okS := capcontour.WalkPointEnclosure(cur.StartU, cur.StartV, cur.StartBound)
		if !okE || !okS {
			return 0, ErrUnbounded
		}
		corner := capcontour.Union(end, start)
		a, okA := capcontour.OffsetFootEnclosure(corner, prev, true, amount)
		b, okB := capcontour.OffsetFootEnclosure(corner, cur, false, amount)
		if !okA || !okB {
			return 0, ErrUnbounded
		}
		switch {
		case j.Arc:
			reach = math.Max(reach, math.Max(a.Reach(j.PA.U, j.PA.V), b.Reach(j.PB.U, j.PB.V)))
		case j.G1:
			reach = math.Max(reach, capcontour.Union(a, b).Reach(j.M.U, j.M.V))
		default:
			ca, okA := capcontour.OffsetCarrierEnclosure(prev, amount)
			cb, okB := capcontour.OffsetCarrierEnclosure(cur, amount)
			if !okA || !okB {
				return 0, ErrUnbounded
			}
			cands, ok := capcontour.Intersect(ca, cb)
			if !ok {
				return 0, ErrUnbounded
			}
			m, ok := capcontour.NearestTo(cands, corner)
			if !ok {
				return 0, ErrUnbounded
			}
			reach = math.Max(reach, m.Reach(j.M.U, j.M.V))
		}
	}
	return reach, nil
}
