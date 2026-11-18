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
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/sketch"
)

// arrangedEdge is one arranged edge of a scene (agent-docs/prism-boolean-design.md Part K
// §6.3): its tag, its source segment, its recorded (possibly narrowed)
// segment and float walk, and its two end vertices.
type arrangedEdge struct {
	entity     sketch.Entity
	origin     Origin
	src        sectionrecord.CurveSegment
	srcEps     float64
	srcH       float64
	seg        sectionrecord.CurveSegment
	walk       survey2d.SegmentWalk
	start, end [2]float64
	t0, t1     float64
	eta        float64 // η_e
	c          float64 // c_e, the walk's allowance
	closing    float64 // cs + ce, the closing paths of its own-vertex ends
}

// oneSegH is Lemma H's h for a lone segment with carrier entry eps and both
// junction entries delta: the bound h of a segment is a function of its
// larger junction entry only.
func oneSegH(seg sectionrecord.CurveSegment, eps, delta float64) float64 {
	cr := prismrecord.CarrierRecord{
		Profile:  momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{seg}}},
		Segment:  [][]float64{{eps}},
		Junction: [][]float64{{delta}},
	}
	return cr.SegmentH(0, 0)
}

// walkAllowance is c for a walk: Lemma H's h read with the walk's radius
// bound as ε and each end's WalkEndBoundAllow as δ (§6.2).
func walkAllowance(seg sectionrecord.CurveSegment, w survey2d.SegmentWalk) float64 {
	delta := 0.0
	if !w.Closed {
		delta = math.Max(proofbound.WalkEndBoundAllow(w.StartBound), proofbound.WalkEndBoundAllow(w.EndBound))
	}
	return oneSegH(seg, w.RadiusBound, delta)
}

// trimmedAllowance is c for the edge once ASA2 has trimmed it at its cut
// vertices: the trimmed walk's own allowance plus the closing paths of the
// ends it did not trim, which are the edge's own-vertex ends (a cut vertex
// carries no closing path).
func (e *arrangedEdge) trimmedAllowance(trim sectionrecord.CurveSegment, w survey2d.SegmentWalk) float64 {
	return proofbound.AbsSumUpper(walkAllowance(trim, w), e.closing)
}

// endDisplacement is how far an edge's read end at one of its vertices lies
// from S*'s junction there: Δ_v at a cut vertex, the operand's own junction
// entry at an own vertex. closing is the exact length of the junction path
// between the edge's own read end and the adjacent source segment's, which no
// edge holds (§6.2): zero at a cut vertex and at a junction whose ends meet.
func endDisplacement(end vertexEnd, at [2]float64, o Origin, src []prismrecord.CarrierRecord,
	vr VertexReading, srcSeg sectionrecord.CurveSegment) (displacement, closing float64, ok bool) {
	if cutAt(end.edge, end.seg, end.t) {
		if v, ok := vr.Vertices[at]; ok {
			return v.Delta, 0, true
		}
		// A circle's seam: both of its pieces end at the exact point of the
		// recorded circle at parameter 0 (or 1), which the denoted circle
		// moves by at most its carrier entry.
		if _, circle := srcSeg.(sectionrecord.CircleSeg); circle && (end.t == 0 || end.t == 1) {
			return src[o.Region].Segment[o.Hole+1][o.Segment], 0, true
		}
		return 0, 0, false
	}
	w, err := boundarywalk.WalkOf(srcSeg, nil)
	if err != nil || w.Closed {
		return 0, 0, err == nil
	}
	r := src[o.Region]
	l := o.Hole + 1
	n := len(r.Junction[l])
	loop := append([]sectionrecord.LoopRecord{r.Profile.Outer}, r.Profile.Holes...)[l]
	// An end the arrangement did not cut is one of the source segment's own
	// ends; the arranger's polyline may differ from the walk by an ulp, so the
	// nearer end names it.
	ds := math.Hypot(at[0]-w.StartU, at[1]-w.StartV)
	de := math.Hypot(at[0]-w.EndU, at[1]-w.EndV)
	if ds <= de {
		prev := (o.Segment + n - 1) % n
		return r.Junction[l][prev], junctionClosing(loop.Segments[prev], srcSeg), true
	}
	return r.Junction[l][o.Segment], junctionClosing(srcSeg, loop.Segments[(o.Segment+1)%n]), true
}

// junctionEnd is a walk end as an exact point with the slack its float
// carries: zero for a line's rational end, WalkEndBoundAllow otherwise.
type junctionEnd struct {
	pt    freeform.RatPoint
	slack float64
}

func segmentJunctionEnd(seg sectionrecord.CurveSegment, atEnd bool) (junctionEnd, bool) {
	if line, ok := seg.(sectionrecord.LineSeg); ok {
		s, e := momentinput.ExactLineEnds(line)
		p := s
		if atEnd {
			p = e
		}
		if p == nil || p.U == nil || p.V == nil {
			return junctionEnd{}, false
		}
		return junctionEnd{pt: *p}, true
	}
	w, err := boundarywalk.WalkOf(seg, nil)
	if err != nil {
		return junctionEnd{}, false
	}
	u, v, bound := w.StartU, w.StartV, boundarywalk.DenotedStartBound(seg, w)
	if atEnd {
		u, v, bound = w.EndU, w.EndV, boundarywalk.DenotedEndBound(seg, w)
	}
	ru, rv := proofarith.FloatRat(u), proofarith.FloatRat(v)
	if ru == nil || rv == nil {
		return junctionEnd{}, false
	}
	return junctionEnd{pt: freeform.RatPoint{U: ru, V: rv}, slack: proofbound.WalkEndBoundAllow(bound)}, true
}

// junctionClosing is the exact length of the closing path of the junction from
// prev's walk end to next's walk start, each end rounded up by its own bound:
// both legs of a line corner (momentinput.JunctionCorner, the evaluator's own
// rule) or the straight chord. It is +Inf where an end has no exact point.
func junctionClosing(prev, next sectionrecord.CurveSegment) float64 {
	p, ok1 := segmentJunctionEnd(prev, true)
	q, ok2 := segmentJunctionEnd(next, false)
	if !ok1 || !ok2 {
		return math.Inf(1)
	}
	slack := proofbound.AbsSumUpper(p.slack, q.slack)
	if corner, ok := momentinput.JunctionCorner(prev, next, &p.pt, &q.pt); ok {
		return proofbound.AbsSumUpper(ratDistUpper(p.pt, corner), ratDistUpper(corner, q.pt), slack)
	}
	return proofbound.AbsSumUpper(ratDistUpper(p.pt, q.pt), slack)
}

func ratDistUpper(p, q freeform.RatPoint) float64 {
	du, dv := new(big.Rat).Sub(p.U, q.U), new(big.Rat).Sub(p.V, q.V)
	return proofbound.RatSqrtUp(du.Add(du.Mul(du, du), dv.Mul(dv, dv)))
}

// collectEdges lists the distinct arranged edges of the cells with their
// displacement allowances. ok=false (err nil) is a miss.
func collectEdges(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, cells []*sketch.Profile,
	src []prismrecord.CarrierRecord, vr VertexReading) ([]arrangedEdge, bool, error) {
	type edgeKey struct {
		entity sketch.Entity
		t0, t1 float64
	}
	seen := map[edgeKey]struct{}{}
	var edges []arrangedEdge
	for _, p := range cells {
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.Step(); err != nil {
					return nil, false, err
				}
				lo, hi := e.TStart, e.TEnd
				if lo > hi {
					lo, hi = hi, lo
				}
				k := edgeKey{entity: e.Entity, t0: lo, t1: hi}
				if _, dup := seen[k]; dup {
					continue
				}
				seen[k] = struct{}{}
				o, ok := tags[e.Entity]
				if !ok || len(e.Polyline) == 0 {
					return nil, false, nil
				}
				seg, t0, t1, ok := recordedRange(e)
				if !ok {
					return nil, false, nil
				}
				srcSeg, eps, h, ok := SourceSegment(src, o)
				if !ok {
					return nil, false, nil
				}
				w, err := boundarywalk.WalkOf(seg, nil)
				if err != nil || (!w.IsLine() && !w.IsCircular()) {
					return nil, false, nil //nolint:nilerr // an edge with no walk is not audited: a miss
				}
				a := arrangedEdge{entity: e.Entity, origin: o, src: srcSeg, srcEps: eps, srcH: h, seg: seg, walk: w,
					start: e.Polyline[0], end: e.Polyline[len(e.Polyline)-1], t0: t0, t1: t1}
				ds, cs, ok1 := endDisplacement(vertexEnd{edge: e, seg: seg, t: t0, atStart: true}, a.start, o, src, vr, srcSeg)
				de, ce, ok2 := endDisplacement(vertexEnd{edge: e, seg: seg, t: t1}, a.end, o, src, vr, srcSeg)
				if !ok1 || !ok2 {
					return nil, false, nil
				}
				a.eta = oneSegH(seg, eps, math.Max(ds, de))
				a.closing = proofbound.AbsSumUpper(cs, ce)
				a.c = proofbound.AbsSumUpper(walkAllowance(seg, w), a.closing)
				edges = append(edges, a)
			}
		}
	}
	return edges, true, nil
}

// seamJunction is the junction displacement where two pieces of one recorded
// circle meet at its seam (parameter 0 or 1): the circle's carrier entry.
func seamJunction(a, b sketch.BoundaryEdge, tags map[sketch.Entity]Origin, src []prismrecord.CarrierRecord) (float64, bool) {
	if a.Entity != b.Entity {
		return 0, false
	}
	sa, ta0, ta1, ok1 := recordedRange(a)
	sb, tb0, _, ok2 := recordedRange(b)
	if !ok1 || !ok2 {
		return 0, false
	}
	_, c1 := sa.(sectionrecord.CircleSeg)
	_, c2 := sb.(sectionrecord.CircleSeg)
	if !c1 || !c2 || !(ta1 == 0 || ta1 == 1) || !(tb0 == 0 || tb0 == 1) {
		return 0, false
	}
	_ = ta0
	o, ok := tags[a.Entity]
	if !ok {
		return 0, false
	}
	_, eps, _, ok := SourceSegment(src, o)
	return eps, ok
}

// SeamJunction is seamJunction for a caller outside this package.
func SeamJunction(a, b sketch.BoundaryEdge, tags map[sketch.Entity]Origin, src []prismrecord.CarrierRecord) (float64, bool) {
	return seamJunction(a, b, tags, src)
}
