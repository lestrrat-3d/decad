package pointquery

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// This file classifies an exact plane-local point against the region a
// recorded profile denotes, using only exact sign tests over the record's
// own rational data (agent-docs/point-containment-design.md §7.3). Any
// segment kind, range or point position it cannot settle exactly leaves the
// answer undecided.

// regionClass is the exact class of a plane-local point against a region.
type regionClass uint8

const (
	regionUndecided regionClass = iota
	regionOutside
	regionInside
	regionBoundary
)

type planePoint struct{ u, v *big.Rat }

func planePointOf(p freeform.RatPoint) planePoint { return planePoint{u: p.U, v: p.V} }

func recordPoint(p sectionrecord.Point2) (planePoint, bool) {
	u, v := proof.FloatRat(p.U), proof.FloatRat(p.V)
	return planePoint{u: u, v: v}, u != nil && v != nil
}

func (p planePoint) equal(q planePoint) bool { return p.u.Cmp(q.u) == 0 && p.v.Cmp(q.v) == 0 }

func (p planePoint) sub(q planePoint) planePoint {
	return planePoint{u: new(big.Rat).Sub(p.u, q.u), v: new(big.Rat).Sub(p.v, q.v)}
}

func (p planePoint) dot(q planePoint) *big.Rat {
	out := new(big.Rat).Mul(p.u, q.u)
	return out.Add(out, new(big.Rat).Mul(p.v, q.v))
}

// side is the exact cross product (b − a) × (q − a): positive with q left of
// the directed line a → b, zero on it.
func side(a, b, q planePoint) int {
	d, e := b.sub(a), q.sub(a)
	return new(big.Rat).Sub(new(big.Rat).Mul(d.u, e.v), new(big.Rat).Mul(e.u, d.v)).Sign()
}

// onSegment reports whether q lies on the closed segment a → b.
func onSegment(a, b, q planePoint) bool {
	return side(a, b, q) == 0 && q.sub(a).dot(q.sub(b)).Sign() <= 0
}

// ringEdge is one straight edge of a loop's ring. chord marks the chord that
// stands in for an arc; every other edge is part of the denoted boundary.
type ringEdge struct {
	from, to planePoint
	chord    bool
}

// ringArc is a whole arc of exact radius: the circle points on the closed
// right of the directed chord start → end, start and end the arc's own
// natural ends. sign is +1 when the loop walks it from start to end and −1
// when it walks it back.
type ringArc struct {
	center, start, end planePoint
	radiusSquared      *big.Rat
	sign               int
}

// locateInProfile classifies q against the region the outer loop bounds less
// every hole's. A point inside two holes, inside a hole but not the outer
// loop, or on more than one loop is undecided: a sketch-arranged record has
// none of these, and the route does not decide what such a record denotes.
func locateInProfile(budget *proofbound.WorkBudget, profile momentinput.Profile, q planePoint) (regionClass, error) {
	outer, err := locateInLoop(budget, profile.Outer, q)
	if err != nil || outer == regionUndecided {
		return regionUndecided, err
	}
	inHoles, onHoles := 0, 0
	for _, hole := range profile.Holes {
		class, err := locateInLoop(budget, hole, q)
		if err != nil || class == regionUndecided {
			return regionUndecided, err
		}
		switch class {
		case regionInside:
			inHoles++
		case regionBoundary:
			onHoles++
		}
	}
	switch {
	case inHoles+onHoles > 1:
		return regionUndecided, nil
	case outer == regionOutside && inHoles+onHoles == 0:
		return regionOutside, nil
	case outer == regionOutside:
		return regionUndecided, nil
	case outer == regionBoundary && inHoles+onHoles == 0:
		return regionBoundary, nil
	case outer == regionBoundary:
		return regionUndecided, nil
	case onHoles == 1:
		return regionBoundary, nil
	case inHoles == 1:
		return regionOutside, nil
	default:
		return regionInside, nil
	}
}

// locateInLoop classifies q against the closed region one recorded loop
// bounds. A single whole circle compares squared distances. Otherwise the
// loop is walked into a ring of exact vertices: each line at its recorded
// parameters, each whole exact-radius arc by its chord, and each junction
// closed as the evaluator closes it (momentinput.JunctionCorner, else the
// straight chord). The winding number of the ring about q, corrected by every
// arc's circular segment that holds q, is the loop's winding number.
func locateInLoop(budget *proofbound.WorkBudget, loop sectionrecord.LoopRecord, q planePoint) (regionClass, error) {
	segments := make([]sectionrecord.CurveSegment, len(loop.Segments))
	for i, segment := range loop.Segments {
		if err := budget.Step(); err != nil {
			return regionUndecided, err
		}
		normalized, ok := normalizedSegment(segment)
		if !ok {
			return regionUndecided, nil
		}
		segments[i] = normalized
	}
	if len(segments) == 1 {
		circle, ok := segments[0].(sectionrecord.CircleSeg)
		if !ok {
			return regionUndecided, nil
		}
		return locateInCircle(circle, q), nil
	}
	starts := make([]planePoint, len(segments))
	ends := make([]planePoint, len(segments))
	var arcs []ringArc
	arcOf := make([]int, len(segments))
	for i, segment := range segments {
		arcOf[i] = -1
		switch s := segment.(type) {
		case sectionrecord.LineSeg:
			start, end := momentinput.ExactLineEnds(s)
			if start == nil || end == nil {
				return regionUndecided, nil
			}
			starts[i], ends[i] = planePointOf(*start), planePointOf(*end)
		case sectionrecord.ArcSeg:
			arc, ok := exactArc(s)
			if !ok {
				return regionUndecided, nil
			}
			starts[i], ends[i] = arc.start, arc.end
			if arc.sign < 0 {
				starts[i], ends[i] = arc.end, arc.start
			}
			arcOf[i] = len(arcs)
			arcs = append(arcs, arc)
		default:
			return regionUndecided, nil
		}
	}
	var edges []ringEdge
	for i := range segments {
		if err := budget.Step(); err != nil {
			return regionUndecided, err
		}
		edges = append(edges, ringEdge{from: starts[i], to: ends[i], chord: arcOf[i] >= 0})
		j := (i + 1) % len(segments)
		if ends[i].equal(starts[j]) {
			continue
		}
		p := freeform.RatPoint{U: ends[i].u, V: ends[i].v}
		next := freeform.RatPoint{U: starts[j].u, V: starts[j].v}
		if corner, ok := momentinput.JunctionCorner(segments[i], segments[j], &p, &next); ok {
			c := planePointOf(corner)
			edges = append(edges, ringEdge{from: ends[i], to: c}, ringEdge{from: c, to: starts[j]})
			continue
		}
		edges = append(edges, ringEdge{from: ends[i], to: starts[j]})
	}
	for _, arc := range arcs {
		if arc.holds(q) == regionBoundary {
			return regionBoundary, nil
		}
	}
	for _, edge := range edges {
		if !edge.chord && onSegment(edge.from, edge.to, q) {
			return regionBoundary, nil
		}
	}
	winding := 0
	for _, edge := range edges {
		if err := budget.Step(); err != nil {
			return regionUndecided, err
		}
		if edge.chord && onSegment(edge.from, edge.to, q) {
			return regionUndecided, nil
		}
		winding += crossingOf(edge, q)
	}
	for _, arc := range arcs {
		if arc.holds(q) == regionInside {
			winding += arc.sign
		}
	}
	switch winding {
	case 0:
		return regionOutside, nil
	case 1, -1:
		return regionInside, nil
	default:
		return regionUndecided, nil
	}
}

// normalizedSegment reads a segment in its value form. A segment the record
// codec does not accept is not one this route reads.
func normalizedSegment(segment sectionrecord.CurveSegment) (sectionrecord.CurveSegment, bool) {
	normalized, err := sectionrecord.NormalizeSegment(segment)
	return normalized, err == nil
}

// crossingOf is one edge's signed contribution to the winding number about a
// point that does not lie on it: +1 for an upward crossing of the point's
// horizontal with the point on its left, −1 for a downward crossing with the
// point on its right.
func crossingOf(edge ringEdge, q planePoint) int {
	if edge.from.v.Cmp(q.v) <= 0 {
		if edge.to.v.Cmp(q.v) > 0 && side(edge.from, edge.to, q) > 0 {
			return 1
		}
		return 0
	}
	if edge.to.v.Cmp(q.v) <= 0 && side(edge.from, edge.to, q) < 0 {
		return -1
	}
	return 0
}

// isWholeRange reports a recorded range that spans the entity's natural
// domain [0, 1] in either sense, by exact float equality.
func isWholeRange(t0, t1 float64) bool {
	return (t0 == 0 && t1 == 1) || (t0 == 1 && t1 == 0)
}

// locateInCircle classifies q against a whole circle's closed disk. Its
// radius must be recorded in millimetres and positive; a cut circle's ends
// carry trigonometry, so a fragment is undecided.
func locateInCircle(circle sectionrecord.CircleSeg, q planePoint) regionClass {
	if !isWholeRange(circle.TStart, circle.TEnd) || circle.Radius.Unit() != units.Millimeter {
		return regionUndecided
	}
	center, ok := recordPoint(circle.Center)
	radius := proof.FloatRat(circle.Radius.Mag())
	if !ok || radius == nil || radius.Sign() <= 0 {
		return regionUndecided
	}
	offset := q.sub(center)
	switch offset.dot(offset).Cmp(new(big.Rat).Mul(radius, radius)) {
	case -1:
		return regionInside
	case 0:
		return regionBoundary
	default:
		return regionOutside
	}
}

// exactArc reads a whole arc whose recorded End lies exactly on Start's
// radius, so both of its denoted ends are the recorded Start and End. An arc
// fragment, an End off that radius, or a degenerate arc reports false: their
// denoted ends carry trigonometry or a square root.
func exactArc(arc sectionrecord.ArcSeg) (ringArc, bool) {
	if !isWholeRange(arc.TStart, arc.TEnd) {
		return ringArc{}, false
	}
	center, okC := recordPoint(arc.Center)
	start, okS := recordPoint(arc.Start)
	end, okE := recordPoint(arc.End)
	if !okC || !okS || !okE || start.equal(end) {
		return ringArc{}, false
	}
	fromStart, fromEnd := start.sub(center), end.sub(center)
	radiusSquared := fromStart.dot(fromStart)
	if radiusSquared.Sign() == 0 || radiusSquared.Cmp(fromEnd.dot(fromEnd)) != 0 {
		return ringArc{}, false
	}
	sign := 1
	if arc.TStart > arc.TEnd {
		sign = -1
	}
	return ringArc{center: center, start: start, end: end, radiusSquared: radiusSquared, sign: sign}, true
}

// holds classifies q against the arc's closed circular segment, the disk
// part on the closed right of the directed chord start → end: boundary when
// q lies on the arc itself, inside when it lies strictly within the segment,
// and outside otherwise, including a point on the chord's open interior,
// which the caller tests separately.
func (a ringArc) holds(q planePoint) regionClass {
	offset := q.sub(a.center)
	radial := offset.dot(offset).Cmp(a.radiusSquared)
	chord := side(a.start, a.end, q)
	switch {
	case radial == 0 && chord <= 0:
		return regionBoundary
	case radial < 0 && chord < 0:
		return regionInside
	default:
		return regionOutside
	}
}
