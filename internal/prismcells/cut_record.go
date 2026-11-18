package prismcells

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// This file is the clean-nesting Cut's carrier record
// (agent-docs/prism-boolean-design.md Part K §5.4): each result segment and
// junction reads its entity tag for the target (hole, segment) it came from,
// because the matched profile may order loops and start segments differently
// from the target.

// CutRecordInput is everything §5.4 reads.
type CutRecordInput struct {
	Target, Tool prismrecord.Payload
	Reexpress    *Reexpression
	SceneDelta   SceneDelta
	Match        *sketch.Profile
	Tags         map[sketch.Entity]Origin
	// Profile is the matched profile as recorded (RecordProfileContext).
	Profile momentinput.Profile
	// SectionDelta is prism-boolean §7's value for the result.
	SectionDelta float64
	// Apart is §6's stability audit over every pair of a tool segment and a
	// target segment with a nonzero h. It receives the target's record and
	// returns false on a failed audit.
	Apart func(target prismrecord.CarrierRecord, hTarget func(loop, seg int) float64) (bool, error)
}

// CleanCutRecord builds §5.4's record. ok=false (err nil) means a condition
// failed and the Cut builds as today, with no record. The second result is
// the result's SectionDelta raised to every entry and h_loop (CR4).
func CleanCutRecord(budget *proofbound.WorkBudget, in CutRecordInput) (*prismrecord.CarrierRecord, float64, bool, error) {
	if in.SectionDelta == 0 || in.Tool.SectionDelta != 0 || !in.Reexpress.Identity || in.SceneDelta.B != 0 {
		return nil, 0, false, nil
	}
	tr, valid, err := in.Target.CarrierRecord(budget)
	if err != nil || !valid {
		return nil, 0, false, err
	}
	for l := range tr.Segment {
		if math.IsInf(tr.LoopH(l), 1) {
			return nil, 0, false, nil
		}
	}
	if in.Apart != nil && tr.Max() > 0 {
		ok, err := in.Apart(tr, tr.SegmentH)
		if err != nil || !ok {
			return nil, 0, false, err
		}
	}
	loops := append([][]sketch.BoundaryEdge{in.Match.Outer}, in.Match.Holes...)
	rec := append([]momentinput.LoopRecord{in.Profile.Outer}, in.Profile.Holes...)
	if len(loops) != len(rec) {
		return nil, 0, false, nil
	}
	targetLoops := append([]momentinput.LoopRecord{in.Target.Profile.Outer}, in.Target.Profile.Holes...)
	cr := prismrecord.ZeroCarrierRecord(in.Profile)
	for l, edges := range loops {
		if len(edges) != len(rec[l].Segments) {
			return nil, 0, false, nil
		}
		n := len(edges)
		for j, e := range edges {
			o, ok := in.Tags[e.Entity]
			if !ok {
				return nil, 0, false, nil
			}
			if o.IsB {
				continue // the tool's own record: every entry zero
			}
			tl := o.Hole + 1
			if tl < 0 || tl >= len(targetLoops) || o.Segment < 0 || o.Segment >= len(targetLoops[tl].Segments) {
				return nil, 0, false, nil
			}
			eps, ok := segmentEntry(tr, targetLoops[tl], o)
			if !ok {
				return nil, 0, false, nil
			}
			cr.Segment[l][j] = eps
			k := (j + 1) % n
			next, ok := in.Tags[edges[k].Entity]
			if !ok || next.IsB || next.Hole != o.Hole {
				return nil, 0, false, nil
			}
			d, ok := cutJunction(tr, targetLoops[tl], rec[l], j, k, o, next, tl)
			if !ok {
				return nil, 0, false, nil
			}
			cr.Junction[l][j] = d
		}
	}
	delta := max(in.SectionDelta, cr.Max(), cr.HMax())
	if proofbound.IsNonFinite(delta) {
		return nil, 0, false, nil
	}
	cr.SectionDelta = delta
	return &cr, delta, true, nil
}

// segmentEntry is the ε of a result segment from target (hole, segment) o: a
// whole target segment's own ε, or for a narrowed line the larger of its two
// ends' junction δ plus the walk allowance.
func segmentEntry(tr prismrecord.CarrierRecord, loop momentinput.LoopRecord, o Origin) (float64, bool) {
	tl := o.Hole + 1
	seg := loop.Segments[o.Segment]
	if narrowed, line := narrowedLine(seg); !narrowed {
		return tr.Segment[tl][o.Segment], true
	} else if !line {
		return 0, false
	}
	w, err := boundarywalk.WalkOf(seg, nil)
	if err != nil {
		return 0, false
	}
	a, err := WalkChargeOf(seg, w)
	if err != nil {
		return 0, false
	}
	n := len(loop.Segments)
	ds, de := tr.Junction[tl][(o.Segment+n-1)%n], tr.Junction[tl][o.Segment]
	return max(proofbound.AbsSumUpper(ds, a), proofbound.AbsSumUpper(de, a)), true
}

// narrowedLine reports whether a segment is a trimmed segment and whether it is a line.
func narrowedLine(seg momentinput.CurveSegment) (narrowed, line bool) {
	switch s := seg.(type) {
	case momentinput.LineSeg:
		return !WholeSegmentRange(s.TStart, s.TEnd), true
	case momentinput.ArcSeg:
		return !WholeSegmentRange(s.TStart, s.TEnd), false
	case momentinput.CircleSeg:
		return !WholeSegmentRange(s.TStart, s.TEnd), false
	}
	return true, false
}

// cutJunction is the junction entry between result edges j and k (consecutive
// segments a then b of one target loop, in either walk direction): the
// target's own junction δ plus τ, the path rule's distance between the
// result's junction path and the target's there (CR3).
func cutJunction(tr prismrecord.CarrierRecord, tloop, rloop momentinput.LoopRecord, j, k int,
	a, b Origin, tl int) (float64, bool) {
	n := len(tloop.Segments)
	var delta float64
	forward := false
	switch {
	case b.Segment == (a.Segment+1)%n:
		delta, forward = tr.Junction[tl][a.Segment], true
	case a.Segment == (b.Segment+1)%n:
		delta = tr.Junction[tl][b.Segment]
	default:
		return 0, false
	}
	na, _ := narrowedLine(tloop.Segments[a.Segment])
	nb, _ := narrowedLine(tloop.Segments[b.Segment])
	if !na && !nb {
		return delta, true
	}
	tau, ok := junctionTau(tloop.Segments[a.Segment], tloop.Segments[b.Segment], rloop.Segments[j], rloop.Segments[k], forward)
	if !ok {
		return 0, false
	}
	return proofbound.AbsSumUpper(delta, tau), true
}

// pathEnds is a segment's denoted read ends in its own walk order, exact; nil
// where the segment states none (a circular segment's ends are floats read
// verbatim, so a pair of identical circular segments has distance zero).
func pathEnds(seg momentinput.CurveSegment) (start, end *freeform.RatPoint) {
	if l, ok := seg.(momentinput.LineSeg); ok {
		return momentinput.ExactLineEnds(l)
	}
	return nil, nil
}

// junctionTau is the path rule's (CR3) exact distance between the target's
// junction path from segment a to segment b and the result's path between the
// result's two segments ra and rb: matching breakpoints, with the chord point
// at the corner's arc-length fraction where one path is a chord and the other
// a corner path. forward says the result runs a to b in the target's walk
// direction. Non-line neighbours keep their recorded ends in both records, so
// only a line's ends move.
func junctionTau(ta, tb, ra, rb momentinput.CurveSegment, forward bool) (float64, bool) {
	// The target's prev/next in its walk direction, and the result's points at
	// the same junction in the same roles: running against the target, the
	// result walks edge ra then rb, so the target-direction end of rb's segment
	// is its walk start.
	tpEnd, tqStart := endOf(ta), startOf(tb)
	rpEnd, rqStart := endOf(ra), startOf(rb)
	pa, pb := ra, rb
	if !forward {
		ta, tb = tb, ta
		tpEnd, tqStart = endOf(ta), startOf(tb)
		rpEnd, rqStart = startOf(rb), endOf(ra)
		pa, pb = rb, ra
	}
	ra, rb = pa, pb
	if (tpEnd == nil) != (rpEnd == nil) || (tqStart == nil) != (rqStart == nil) {
		// A circular neighbour keeps its recorded end in both records.
		if tpEnd == nil && rpEnd == nil {
			return endDistance(tqStart, rqStart), tqStart != nil && rqStart != nil
		}
		return 0, false
	}
	tCorner, tHas := junctionPathCorner(ta, tb, tpEnd, tqStart)
	rCorner, rHas := junctionPathCorner(ra, rb, rpEnd, rqStart)
	dp, dq := endDistance(tpEnd, rpEnd), endDistance(tqStart, rqStart)
	tau := math.Max(dp, dq)
	switch {
	case !tHas && !rHas:
	case tHas && rHas:
		tau = math.Max(tau, distanceUpper(&tCorner, &rCorner))
	case tHas:
		tau = math.Max(tau, cornerChordGap(tpEnd, &tCorner, tqStart, rpEnd, rqStart))
	default:
		tau = math.Max(tau, cornerChordGap(rpEnd, &rCorner, rqStart, tpEnd, tqStart))
	}
	return tau, !proofbound.IsNonFinite(tau)
}

func startOf(seg momentinput.CurveSegment) *freeform.RatPoint {
	s, _ := pathEnds(seg)
	return s
}

func endOf(seg momentinput.CurveSegment) *freeform.RatPoint {
	_, e := pathEnds(seg)
	return e
}

func junctionPathCorner(prev, next momentinput.CurveSegment, p, q *freeform.RatPoint) (freeform.RatPoint, bool) {
	if p == nil || q == nil {
		return freeform.RatPoint{}, false
	}
	return momentinput.JunctionCorner(prev, next, p, q)
}

// endDistance is the distance between two path ends at one junction. A pair of
// absent ends are a circular neighbour's, read verbatim in both records, so
// they are zero apart; one absent end has no distance (+Inf).
func endDistance(p, q *freeform.RatPoint) float64 {
	if p == nil && q == nil {
		return 0
	}
	return distanceUpper(p, q)
}

// distanceUpper is |p − q| rounded up (+Inf for an absent point).
func distanceUpper(p, q *freeform.RatPoint) float64 {
	if p == nil || q == nil {
		return math.Inf(1)
	}
	du, dv := new(big.Rat).Sub(p.U, q.U), new(big.Rat).Sub(p.V, q.V)
	return proofbound.RatSqrtUp(du.Add(du.Mul(du, du), dv.Mul(dv, dv)))
}

// cornerChordGap is the path rule's breakpoint distance between the corner path
// p → c → q and the chord p′ → q′: the chord point at the corner's arc-length
// fraction f = |pc|/(|pc| + |cq|) must lie near c. f is enclosed from rounded
// leg lengths and the distance is the larger over the two ends of that
// enclosure (the distance is convex in f).
func cornerChordGap(p, c, q, pc, qc *freeform.RatPoint) float64 {
	l1Lo, l1Hi := legBounds(p, c)
	l2Lo, l2Hi := legBounds(c, q)
	den1, den2 := l1Hi+l2Lo, l1Lo+l2Hi
	if !(den1 > 0) || !(den2 > 0) {
		return math.Inf(1)
	}
	worst := 0.0
	for _, f := range []*big.Rat{
		proofarith.FloatRat(math.Nextafter(l1Lo/den2, 0)),
		proofarith.FloatRat(math.Nextafter(l1Hi/den1, math.Inf(1))),
	} {
		if f == nil {
			return math.Inf(1)
		}
		m := freeform.RatPoint{
			U: new(big.Rat).Add(pc.U, new(big.Rat).Mul(f, new(big.Rat).Sub(qc.U, pc.U))),
			V: new(big.Rat).Add(pc.V, new(big.Rat).Mul(f, new(big.Rat).Sub(qc.V, pc.V))),
		}
		worst = math.Max(worst, distanceUpper(c, &m))
	}
	return worst
}

func legBounds(p, q *freeform.RatPoint) (lo, hi float64) {
	du, dv := new(big.Rat).Sub(p.U, q.U), new(big.Rat).Sub(p.V, q.V)
	sq := du.Add(du.Mul(du, du), dv.Mul(dv, dv))
	return proofbound.RatSqrtDown(sq), proofbound.RatSqrtUp(sq)
}
