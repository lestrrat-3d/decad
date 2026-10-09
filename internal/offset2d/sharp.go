package offset2d

import (
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file is the sharp offset family of docs/draft-design.md §2: every wall
// carrier moves its own amount into the material (the draft's offset t for a
// drafted wall, zero for a wall a subset draft keeps, §10.2), and every corner
// is the intersection of its two moved carriers. It differs from BuildLoop's
// §7 offset in one row: a corner BuildLoop would close with a connector arc (a
// reflex corner of the offset's sense) is mitered here too, because a drafted
// wall tilts about its own trace and re-meets its neighbour rather than
// keeping a constant distance from the corner point.

// ErrCircularMiter reports a corner at a circular walk that the held-tangent
// rule does not classify as a G1 join. The sharp corner of such a pair moves
// along a conic as t grows, so no line-and-arc record states the family at
// every t (docs/draft-design.md SD4).
var ErrCircularMiter = errors.New("a corner at a circular walk is not a G1 join")

// ErrLoopConsumed reports a loop every walk of which the sharp offset
// consumes: each trimmed line stops advancing and each trimmed arc overruns
// its source span (WalkConsumed), so no walk of the loop survives at t.
var ErrLoopConsumed = errors.New("the offset consumes every walk of the loop")

// ErrMixedCircularCorner reports a corner at a circular walk whose two walks
// move by different amounts (docs/draft-design.md §10.2): the moved carrier
// and the other one are no longer tangent, so their junction moves along a
// conic, as a non-G1 circular corner's does. It wraps ErrCircularMiter.
var ErrMixedCircularCorner = fmt.Errorf(`%w: its two walks move by different amounts`, ErrCircularMiter)

// UniformAmounts is n walks each offset by t, the amounts a draft of every
// wall moves its walks by.
func UniformAmounts(n int, t float64) []float64 {
	out := make([]float64, n)
	for i := range out {
		out[i] = t
	}
	return out
}

// SharpCornerJoin resolves the corner where prev, its carrier moved tPrev into
// the material, arrives at cur's start, its carrier moved tCur. With equal
// amounts, a G1 join (the same held-tangent dead zone CornerJoin reads) moves
// the corner tCur along the leaving walk's left normal, and any other corner
// is the intersection of the two moved carriers nearest the corner; a corner
// at a circular walk that is not G1 returns ErrCircularMiter before any
// intersection is solved. Two zero amounts leave the corner at the recorded
// corner itself, after the same classification. With unequal amounts
// (docs/draft-design.md §10.2), a corner at a circular walk returns
// ErrMixedCircularCorner whatever its tangents, and two lines miter even
// inside the G1 dead zone. Two moved lines that do not meet (a cusp) return
// ErrNoIntersection.
func SharpCornerJoin(prev, cur survey2d.SideWalk, tPrev, tCur, tol float64) (Join, error) {
	vU, vV := cur.StartU, cur.StartV
	aox, aoy, la := Normalize(prev.TanOutU, prev.TanOutV)
	bix, biy, lb := Normalize(cur.TanInU, cur.TanInV)
	if la == 0 || lb == 0 {
		return Join{}, ErrNoDirection
	}
	if tPrev != tCur {
		if prev.IsCircular() || cur.IsCircular() {
			return Join{}, ErrMixedCircularCorner
		}
		return sharpMiter(prev, cur, tPrev, tCur, tol)
	}
	cross := aox*biy - aoy*bix
	if math.Abs(cross) <= tol && aox*bix+aoy*biy > 0 {
		return Join{G1: true, VertU: vU, VertV: vV, M: Point{U: vU + tCur*(-biy), V: vV + tCur*bix}}, nil
	}
	if prev.IsCircular() || cur.IsCircular() {
		return Join{}, ErrCircularMiter
	}
	if tCur == 0 {
		return Join{VertU: vU, VertV: vV, M: Point{U: vU, V: vV}}, nil
	}
	return sharpMiter(prev, cur, tPrev, tCur, tol)
}

// sharpMiter is the intersection of prev's carrier moved tPrev and cur's moved
// tCur nearest the corner at cur's start.
func sharpMiter(prev, cur survey2d.SideWalk, tPrev, tCur, tol float64) (Join, error) {
	vU, vV := cur.StartU, cur.StartV
	mx, my, ok := Intersect(offsetCarrier(prev, 1, tPrev, tol), offsetCarrier(cur, 1, tCur, tol), vU, vV)
	if !ok {
		return Join{}, ErrNoIntersection
	}
	return Join{VertU: vU, VertV: vV, M: Point{U: mx, V: my}}, nil
}

// SharpJoinsBudget resolves every corner of one coalesced loop of two or more
// walks by SharpCornerJoin, in walk order, walk i moved amounts[i]: corner i
// sits at walk i's start. A walk with no direction is ErrDegenerate and a cusp
// is a CornerTopologyError naming the corner, as SectionJoinsBudget maps
// them; the caller names the loop with InLoop. ErrCircularMiter and
// ErrMixedCircularCorner are returned unwrapped, so the caller states its own
// refusal for each.
func SharpJoinsBudget(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, amounts []float64, tol float64) ([]Join, error) {
	n := len(walks)
	if len(amounts) != n {
		return nil, fmt.Errorf(`%w: %d offset amounts for %d walks`, decaderr.ErrDegenerate, len(amounts), n)
	}
	joins := make([]Join, n)
	for i := range n {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		p := (i + n - 1) % n
		j, err := SharpCornerJoin(walks[p], walks[i], amounts[p], amounts[i], tol)
		switch {
		case errors.Is(err, ErrNoDirection):
			return nil, fmt.Errorf(`%w: a corner walk has no direction`, decaderr.ErrDegenerate)
		case errors.Is(err, ErrNoIntersection):
			return nil, &CornerTopologyError{U: walks[i].StartU, V: walks[i].StartV, Loop: -1}
		case err != nil:
			return nil, err
		}
		joins[i] = j
	}
	return joins, nil
}

// BuildSharpLoop offsets a coalesced section loop under the sharp rule, walk i
// moved amounts[i] into the material, keeping its walk sense, and returns the
// record beside the joins it was trimmed at (nil for a lone closed circle,
// which has no corner). A walk whose amount is zero keeps its own carrier. The
// corners are resolved first (SharpJoinsBudget's refusals), then each walk is
// offset: a circular walk whose offset radius collapses is ErrDrop, and a walk
// its own corners consume is ErrDrop too, unless every walk of the loop is
// consumed, which is ErrLoopConsumed.
func BuildSharpLoop(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, amounts []float64, tol float64) ([]sectionrecord.CurveSegment, []Join, error) {
	n := len(walks)
	if n == 0 {
		return nil, nil, fmt.Errorf(`%w: an offset loop holds no walks`, decaderr.ErrDegenerate)
	}
	if len(amounts) != n {
		return nil, nil, fmt.Errorf(`%w: %d offset amounts for %d walks`, decaderr.ErrDegenerate, len(amounts), n)
	}
	if n == 1 && walks[0].Closed {
		w := walks[0]
		rr, ok := OffsetRadius(w, 1, amounts[0], tol)
		if !ok {
			return nil, nil, ErrDrop
		}
		return []sectionrecord.CurveSegment{CircleSegment(w.CU, w.CV, rr, w.Th1 > w.Th0)}, nil, nil
	}
	joins, err := SharpJoinsBudget(budget, walks, amounts, tol)
	if err != nil {
		return nil, nil, err
	}
	segs := make([]sectionrecord.CurveSegment, 0, n)
	consumed := 0
	for i := range n {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, nil, err
		}
		w := walks[i]
		if w.IsCircular() {
			if _, ok := OffsetRadius(w, 1, amounts[i], tol); !ok {
				return nil, nil, ErrDrop
			}
		}
		start, end := joins[i].M, joins[(i+1)%n].M
		if WalkConsumed(w, start, end, tol) {
			consumed++
			continue
		}
		seg, err := WalkSegment(w, 1, amounts[i], start, end, tol)
		if err != nil {
			return nil, nil, err
		}
		segs = append(segs, seg)
	}
	switch consumed {
	case 0:
		return segs, joins, nil
	case n:
		return nil, nil, ErrLoopConsumed
	default:
		return nil, nil, ErrDrop
	}
}
