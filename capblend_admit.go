package decad

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// This file decides whether docs/tessellation-reach-design.md §7's slice-wise
// occupied-volume proof covers a capBlendPayload. The proof holds where every
// band section of the ideal polyhedron B1 is the chord polygon of the true
// offset section at the same azimuths: a whole turn, a line-line miter, and a
// join whose two walks meet exactly tangent (G1). Every test here is exact
// rational arithmetic over the record's own floats. None is a tolerance or a
// residual, so the predicate classifies the denoted geometry and never admits
// a band because its float readings happen to sit close together.
//
// Two consumers read it and must agree: boolean.go's
// requireVolumeProvingPayload before any mesh is built, and
// tessellateCapBlend when it decides whether to publish the proof.

// capBlendOccupiedVolumeAdmission decides whether docs/tessellation-reach-design.md §7's
// slice-wise occupied-volume proof covers every band of this payload. refusal is the
// staging ErrUnsupported naming the loop and corner it fails on (nil when admitted); err is
// an infrastructure error (budget, record) and never a refusal.
//
// Every loop's walks are checked, because every loop's side wall is chorded and
// enters the proof's trimmed term through the same chord-polygon argument; the
// corner joins are checked only on a loop chamfered on at least one cap, since
// only a band has an offset foot whose locus the proof must follow.
func capBlendOccupiedVolumeAdmission(budget *proofbound.WorkBudget, cbp capBlendPayload) (error, error) {
	for li, loop := range cbp.loops() {
		cl, err := oneLoopCornerLoop(budget, loop, freeform.NewFreeformWork())
		if err != nil {
			return nil, err
		}
		walks := cl.walks
		n := len(walks)
		for i, w := range walks {
			if !w.IsLine() && !w.IsCircular() {
				// Unreachable today: Chamfer refuses a free-form wall before a
				// payload exists. The arm keeps the predicate's own contract
				// true for any payload that reaches it.
				return capBlendAdmissionRefusal(li, i, `a wall that is neither straight nor circular`), nil
			}
		}
		if n == 1 && walks[0].Closed {
			// The whole turn: one closed circle, no corner, and a band whose two
			// directrices sweep the same exact window.
			continue
		}
		for i, w := range walks {
			for _, si := range w.Segs {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				seg, err := normalizeSegment(loop.Segments[si])
				if err != nil {
					return nil, err
				}
				if why := capBlendSegmentRefusal(seg); why != "" {
					return capBlendAdmissionRefusal(li, i, why), nil
				}
			}
		}
		if !cbp.startLoops[li] && !cbp.endLoops[li] {
			continue
		}
		joins, err := capOffsetJoins(budget, cl, cbp.d)
		if err != nil {
			return nil, err
		}
		for i := range n {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			prev, cur := walks[(i+n-1)%n], walks[i]
			if joins[i].arc {
				return capBlendAdmissionRefusal(li, i, `a reflex corner, whose apex fan's stations no recorded window states`), nil
			}
			if prev.IsLine() && cur.IsLine() {
				// A line-line miter: the foot is the intersection of two offset
				// lines, affine in the offset amount.
				continue
			}
			prevSeg, err := normalizeSegment(loop.Segments[prev.Segs[len(prev.Segs)-1]])
			if err != nil {
				return nil, err
			}
			curSeg, err := normalizeSegment(loop.Segments[cur.Segs[0]])
			if err != nil {
				return nil, err
			}
			if !capJoinIsG1(prevSeg, curSeg) {
				return capBlendAdmissionRefusal(li, i, `a corner this evaluator cannot prove a line-line miter or an exactly tangent join`), nil
			}
		}
	}
	return nil, nil //nolint:nilnil // both answers absent is the admission: no refusal and no infrastructure error
}

// capBlendAdmissionRefusal is the staging ErrUnsupported a band the proof does
// not cover surfaces, naming the loop and the walk (corner) it fails on. Its
// text carries "no proof of the volume", the phrase Verify's diagnostic and
// every caller matching on the cause read.
func capBlendAdmissionRefusal(li, corner int, why string) error {
	return fmt.Errorf(`%w: loop %d of this cap-loop chamfer has %s at walk %d, so its mesh carries no proof of the volume it and the body it stands for differ by, and no boolean may compose it`, ErrUnsupported, li, why, corner)
}

// capBlendSegmentRefusal states why one recorded segment of a cornered loop
// gives the proof no exact junction to pair, or "" when it gives one.
//
// A walk's junction vertex is the recorded coordinate only for a LineSeg or an
// ArcSeg over its natural parameter range; a trimmed range or a CircleSeg
// inside a multi-walk loop puts the junction at a computed angle instead. An
// ArcSeg denotes the circle through Start about Center
// (circularEndpointInterval), while the walk pins its end to the recorded End
// (pinArcWalkEnds); only exact equality of the two squared radii puts that
// pinned junction ON the denoted curve (arcWalkEnd's doc comment).
func capBlendSegmentRefusal(seg CurveSegment) string {
	switch s := seg.(type) {
	case LineSeg:
		if !capBlendNaturalRange(s.TStart, s.TEnd) {
			return `a trimmed or circular-segment junction that has no recorded coordinate the proof can pair`
		}
		return ""
	case ArcSeg:
		if !capBlendNaturalRange(s.TStart, s.TEnd) {
			return `a trimmed or circular-segment junction that has no recorded coordinate the proof can pair`
		}
		start := capBlendSquaredRadius(s.Start, s.Center)
		end := capBlendSquaredRadius(s.End, s.Center)
		if start == nil || end == nil || start.Cmp(end) != 0 {
			return `an arc whose recorded end is not on the circle its start states`
		}
		return ""
	default:
		return `a trimmed or circular-segment junction that has no recorded coordinate the proof can pair`
	}
}

// capBlendNaturalRange reports whether a segment runs over its whole natural
// parameter range, in either sense.
func capBlendNaturalRange(tStart, tEnd float64) bool {
	return (tStart == 0 && tEnd == 1) || (tStart == 1 && tEnd == 0)
}

// capBlendSquaredRadius is |p − c|² over the rationals, or nil for a
// coordinate that denotes no rational.
func capBlendSquaredRadius(p, c Point2) *big.Rat {
	du, dv := capBlendRatSub(p.U, c.U), capBlendRatSub(p.V, c.V)
	if du == nil || dv == nil {
		return nil
	}
	return new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
}

// capBlendRatSub is a − b over the rationals, or nil for a non-finite operand.
func capBlendRatSub(a, b float64) *big.Rat {
	ra, rb := proofarith.FloatRat(a), proofarith.FloatRat(b)
	if ra == nil || rb == nil {
		return nil
	}
	return new(big.Rat).Sub(ra, rb)
}

// capJoinIsG1 reports whether prev's walk end and cur's walk start are the same recorded
// point and the two exact tangent directions there are parallel with the same sense —
// exact rational arithmetic over the record's floats, never a tolerance.
//
// A LineSeg walks from Start to End, or End to Start when TStart == 1, with
// direction end − start at both ends. An ArcSeg walks counter-clockwise from
// Start to End, or clockwise from End to Start when TStart == 1, and its
// tangent at a point P is that sense times rot90(P − Center), rot90(x, y) =
// (−y, x). Any other kind, or a coordinate that denotes no rational, is not a
// join this test can prove.
func capJoinIsG1(prev, cur CurveSegment) bool {
	p, ok := capJoinEnds(prev)
	if !ok {
		return false
	}
	c, ok := capJoinEnds(cur)
	if !ok {
		return false
	}
	prevEnd, prevTan := p.end, p.tanEnd
	curStart, curTan := c.start, c.tanStart
	if prevEnd[0].Cmp(curStart[0]) != 0 || prevEnd[1].Cmp(curStart[1]) != 0 {
		return false
	}
	cross := new(big.Rat).Sub(new(big.Rat).Mul(prevTan[0], curTan[1]), new(big.Rat).Mul(prevTan[1], curTan[0]))
	if cross.Sign() != 0 {
		return false
	}
	dot := new(big.Rat).Add(new(big.Rat).Mul(prevTan[0], curTan[0]), new(big.Rat).Mul(prevTan[1], curTan[1]))
	return dot.Sign() > 0
}

// capJoinEnd is one segment's walk start and end and its exact tangent
// direction at each, as capJoinIsG1 states them.
type capJoinEnd struct {
	start, end       [2]*big.Rat
	tanStart, tanEnd [2]*big.Rat
}

// capJoinEnds reads one segment's capJoinEnd, ok == false for a kind
// capJoinIsG1 does not pair or a coordinate that denotes no rational.
func capJoinEnds(seg CurveSegment) (capJoinEnd, bool) {
	ratPoint := func(p Point2) ([2]*big.Rat, bool) {
		u, v := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
		return [2]*big.Rat{u, v}, u != nil && v != nil
	}
	switch s := seg.(type) {
	case LineSeg:
		a, b := s.Start, s.End
		if s.TStart == 1 {
			a, b = b, a
		}
		ra, okA := ratPoint(a)
		rb, okB := ratPoint(b)
		if !okA || !okB {
			return capJoinEnd{}, false
		}
		dir := [2]*big.Rat{new(big.Rat).Sub(rb[0], ra[0]), new(big.Rat).Sub(rb[1], ra[1])}
		return capJoinEnd{start: ra, end: rb, tanStart: dir, tanEnd: dir}, true
	case ArcSeg:
		a, b := s.Start, s.End
		sense := big.NewRat(1, 1)
		if s.TStart == 1 {
			a, b = b, a
			sense = big.NewRat(-1, 1)
		}
		ra, okA := ratPoint(a)
		rb, okB := ratPoint(b)
		rc, okC := ratPoint(s.Center)
		if !okA || !okB || !okC {
			return capJoinEnd{}, false
		}
		tangent := func(p [2]*big.Rat) [2]*big.Rat {
			du := new(big.Rat).Sub(p[0], rc[0])
			dv := new(big.Rat).Sub(p[1], rc[1])
			return [2]*big.Rat{new(big.Rat).Mul(sense, new(big.Rat).Neg(dv)), new(big.Rat).Mul(sense, du)}
		}
		return capJoinEnd{start: ra, end: rb, tanStart: tangent(ra), tanEnd: tangent(rb)}, true
	default:
		return capJoinEnd{}, false
	}
}
