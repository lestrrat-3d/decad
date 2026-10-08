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
	reach, err := arcEndGap(walks)
	if err != nil {
		return 0, err
	}
	for i, j := range joins {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, err
		}
		r, err := cornerReach(walks[(i+n-1)%n], walks[i], j, amount)
		if err != nil {
			return 0, err
		}
		reach = math.Max(reach, r)
	}
	return reach, nil
}

// arcEndGap is the largest radial gap between a recorded arc's end and the
// circle its start fixes, over every circular walk. That gap moves the
// denoted foot off the denoted carrier, so it joins the reach the arc
// argument reads.
func arcEndGap(walks []survey2d.SideWalk) (float64, error) {
	reach := 0.0
	for _, w := range walks {
		if !w.IsCircular() {
			continue
		}
		gap, ok := capcontour.CircularWalkEndGap(w)
		if !ok {
			return 0, ErrUnbounded
		}
		reach = math.Max(reach, gap)
	}
	return reach, nil
}

// cornerReach is how far corner join j, where prev arrives at cur's start,
// sits from its enclosure: an arc's two feet, a G1 join's hull of both feet,
// or a miter's carrier intersection nearest the corner.
func cornerReach(prev, cur survey2d.SideWalk, j Join, amount proofbound.RatInterval) (float64, error) {
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
		return math.Max(a.Reach(j.PA.U, j.PA.V), b.Reach(j.PB.U, j.PB.V)), nil
	case j.G1:
		return capcontour.Union(a, b).Reach(j.M.U, j.M.V), nil
	}
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
	return m.Reach(j.M.U, j.M.V), nil
}

// ChainEnd is how one end of an open offset chain closes: on a line the
// chain meets at its own mirror image (MirrorCornerJoin), or at a side
// opening, where the end takes OpeningJoin's rim cut against the removed
// neighbour walk.
type ChainEnd struct {
	// Mirror is true for an end on the mirror line.
	Mirror bool
	// Removed is an opening end's removed neighbour walk.
	Removed survey2d.SideWalk
	// Join is the end's join exactly as the build took it.
	Join Join
}

// ChainReach is LoopReach for an open chain whose two ends close by end0
// (at chain[0]'s start) and end1 (at the last walk's end). Interior corners
// read LoopReach's own enclosures, through CornerJoin's rule. An opening end
// encloses its rim cut (OpeningReach). A mirror end encloses the join
// MirrorCornerJoin builds against line, the mirror line widened by its own
// proven bounds: a miter is the walk's offset carrier met with that line, a
// G1 join and an arc's line end are the line point amount from the corner
// along the line's direction, and an arc's other end is the walk's own
// offset foot. A G1 end is charged the hull of the line point and the foot,
// so a join the dead zone classified G1 with a residual turn is charged that
// spread, as LoopReach charges an interior G1 join. The caller reads the
// result as LoopReach's: three reaches bound the displacement of every
// recorded boundary point.
func ChainReach(budget *proofbound.WorkBudget, chain []survey2d.SideWalk, line MirrorLine, end0, end1 ChainEnd, s, t float64, amount proofbound.RatInterval, tol float64) (float64, error) {
	m := len(chain)
	if m == 0 {
		return 0, fmt.Errorf(`%w: an offset chain holds no walks`, decaderr.ErrDegenerate)
	}
	reach, err := arcEndGap(chain)
	if err != nil {
		return 0, err
	}
	for i := 1; i < m; i++ {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, err
		}
		j, err := CornerJoin(chain[i-1], chain[i], s, t, tol)
		if err != nil {
			return 0, ErrUnbounded
		}
		r, err := cornerReach(chain[i-1], chain[i], j, amount)
		if err != nil {
			return 0, err
		}
		reach = math.Max(reach, r)
	}
	for _, e := range []struct {
		w     survey2d.SideWalk
		atEnd bool
		end   ChainEnd
	}{{chain[0], false, end0}, {chain[m-1], true, end1}} {
		r, err := chainEndReach(e.w, e.atEnd, e.end, line, s, amount, tol)
		if err != nil {
			return 0, err
		}
		reach = math.Max(reach, r)
	}
	return reach, nil
}

// MirrorLine is the line a chain's mirror ends meet, held as the build took
// it and enclosed over its own proven bounds.
type MirrorLine struct {
	Held      Curve
	Enclosure capcontour.Carrier
}

// chainEndReach encloses one end join of an open chain (ChainReach).
func chainEndReach(w survey2d.SideWalk, atEnd bool, end ChainEnd, line MirrorLine, s float64, amount proofbound.RatInterval, tol float64) (float64, error) {
	if !end.Mirror {
		return OpeningReach(w, end.Removed, atEnd, s, end.Join, amount, tol)
	}
	cu, cv, cb := w.StartU, w.StartV, w.StartBound
	if atEnd {
		cu, cv, cb = w.EndU, w.EndV, w.EndBound
	}
	corner, ok := capcontour.WalkPointEnclosure(cu, cv, cb)
	if !ok {
		return 0, ErrUnbounded
	}
	j := end.Join
	foot, ok := capcontour.OffsetFootEnclosure(corner, w, atEnd, amount)
	if !ok {
		return 0, ErrUnbounded
	}
	if !line.Held.IsLine || !line.Enclosure.IsLine {
		return 0, ErrUnbounded
	}
	// on steps amount along the line's direction, on the side the build's
	// own held normal chose (MirrorCornerJoin's sign).
	nx, ny := -w.TanInV, w.TanInU
	if atEnd {
		nx, ny = -w.TanOutV, w.TanOutU
	}
	along := amount
	if nx*line.Held.DX+ny*line.Held.DY < 0 {
		along = proofbound.IntervalNeg(amount)
	}
	on := capcontour.Point{
		U: proofbound.IntervalAdd(corner.U, proofbound.IntervalMul(along, line.Enclosure.Dir.U)),
		V: proofbound.IntervalAdd(corner.V, proofbound.IntervalMul(along, line.Enclosure.Dir.V)),
	}
	switch {
	case j.Arc && atEnd:
		return math.Max(foot.Reach(j.PA.U, j.PA.V), on.Reach(j.PB.U, j.PB.V)), nil
	case j.Arc:
		return math.Max(on.Reach(j.PA.U, j.PA.V), foot.Reach(j.PB.U, j.PB.V)), nil
	case j.G1:
		return capcontour.Union(on, foot).Reach(j.M.U, j.M.V), nil
	}
	carrier, ok := capcontour.OffsetCarrierEnclosure(w, amount)
	if !ok {
		return 0, ErrUnbounded
	}
	cands, ok := capcontour.Intersect(carrier, line.Enclosure)
	if !ok {
		return 0, ErrUnbounded
	}
	mp, ok := capcontour.NearestTo(cands, corner)
	if !ok {
		return 0, ErrUnbounded
	}
	return mp.Reach(j.M.U, j.M.V), nil
}
