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
// carrier moves s*t into the material, and every corner is the intersection
// of its two moved carriers. It differs from BuildLoop's §7 offset in one row:
// a corner BuildLoop would close with a connector arc (a reflex corner of the
// offset's sense) is mitered here too, because a drafted wall tilts about its
// own trace and re-meets its neighbour rather than keeping a constant distance
// from the corner point.

// ErrCircularMiter reports a corner at a circular walk that the held-tangent
// rule does not classify as a G1 join. The sharp corner of such a pair moves
// along a conic as t grows, so no line-and-arc record states the family at
// every t (docs/draft-design.md SD4).
var ErrCircularMiter = errors.New("a corner at a circular walk is not a G1 join")

// ErrLoopConsumed reports a loop every walk of which the sharp offset
// consumes: each trimmed line stops advancing and each trimmed arc overruns
// its source span (WalkConsumed), so no walk of the loop survives at t.
var ErrLoopConsumed = errors.New("the offset consumes every walk of the loop")

// SharpCornerJoin resolves the corner where prev arrives at cur's start by the
// sharp rule. A G1 join (the same held-tangent dead zone CornerJoin reads)
// moves the corner s*t along the leaving walk's left normal. Any other corner
// is the intersection of the two moved carriers nearest the corner. A corner
// at a circular walk that is not G1 returns ErrCircularMiter before any
// intersection is solved, and two moved lines that do not meet (a cusp)
// return ErrNoIntersection.
func SharpCornerJoin(prev, cur survey2d.SideWalk, s, t, tol float64) (Join, error) {
	vU, vV := cur.StartU, cur.StartV
	aox, aoy, la := Normalize(prev.TanOutU, prev.TanOutV)
	bix, biy, lb := Normalize(cur.TanInU, cur.TanInV)
	if la == 0 || lb == 0 {
		return Join{}, ErrNoDirection
	}
	cross := aox*biy - aoy*bix
	if math.Abs(cross) <= tol && aox*bix+aoy*biy > 0 {
		return Join{G1: true, VertU: vU, VertV: vV, M: Point{U: vU + s*t*(-biy), V: vV + s*t*bix}}, nil
	}
	if prev.IsCircular() || cur.IsCircular() {
		return Join{}, ErrCircularMiter
	}
	mx, my, ok := Intersect(offsetCarrier(prev, s, t, tol), offsetCarrier(cur, s, t, tol), vU, vV)
	if !ok {
		return Join{}, ErrNoIntersection
	}
	return Join{VertU: vU, VertV: vV, M: Point{U: mx, V: my}}, nil
}

// SharpJoinsBudget resolves every corner of one coalesced loop of two or more
// walks by SharpCornerJoin, in walk order: corner i sits at walk i's start. A
// walk with no direction is ErrDegenerate and a cusp is ErrTopology, as
// SectionJoinsBudget maps them. ErrCircularMiter is returned unwrapped, so the
// caller states its own refusal for it.
func SharpJoinsBudget(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, s, t, tol float64) ([]Join, error) {
	n := len(walks)
	joins := make([]Join, n)
	for i := range n {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		j, err := SharpCornerJoin(walks[(i+n-1)%n], walks[i], s, t, tol)
		switch {
		case errors.Is(err, ErrNoDirection):
			return nil, fmt.Errorf(`%w: a corner walk has no direction`, decaderr.ErrDegenerate)
		case errors.Is(err, ErrNoIntersection):
			return nil, ErrTopology
		case err != nil:
			return nil, err
		}
		joins[i] = j
	}
	return joins, nil
}

// BuildSharpLoop offsets a coalesced section loop by s*t under the sharp rule,
// keeping its walk sense, and returns the record beside the joins it was
// trimmed at (nil for a lone closed circle, which has no corner). The corners
// are resolved first (SharpJoinsBudget's refusals), then each walk is offset:
// a circular walk whose offset radius collapses is ErrDrop, and a walk its own
// corners consume is ErrDrop too, unless every walk of the loop is consumed,
// which is ErrLoopConsumed.
func BuildSharpLoop(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, s, t, tol float64) ([]sectionrecord.CurveSegment, []Join, error) {
	n := len(walks)
	if n == 0 {
		return nil, nil, fmt.Errorf(`%w: an offset loop holds no walks`, decaderr.ErrDegenerate)
	}
	if n == 1 && walks[0].Closed {
		w := walks[0]
		rr, ok := OffsetRadius(w, s, t, tol)
		if !ok {
			return nil, nil, ErrDrop
		}
		return []sectionrecord.CurveSegment{CircleSegment(w.CU, w.CV, rr, w.Th1 > w.Th0)}, nil, nil
	}
	joins, err := SharpJoinsBudget(budget, walks, s, t, tol)
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
			if _, ok := OffsetRadius(w, s, t, tol); !ok {
				return nil, nil, ErrDrop
			}
		}
		start, end := joins[i].M, joins[(i+1)%n].M
		if WalkConsumed(w, start, end, tol) {
			consumed++
			continue
		}
		seg, err := WalkSegment(w, s, t, start, end, tol)
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
