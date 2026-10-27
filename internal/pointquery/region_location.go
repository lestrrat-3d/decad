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

// regionQuery is a query point the region tests read only through exact
// signs: a rational planePoint, or a surdPoint whose coordinates carry one
// square root (agent-docs/point-containment-design.md §10.3).
type regionQuery interface {
	// sideOf is the sign of (b − a) × (q − a): positive with q left of the
	// directed line a → b, zero on it.
	sideOf(a, b planePoint) int
	// spanOf is the sign of (q − a)·(q − b): not positive exactly when q,
	// known to lie on the line through a and b, lies on the closed segment.
	spanOf(a, b planePoint) int
	// heightOf is the sign of q.v − v.
	heightOf(v *big.Rat) int
	// radialOf is the sign of |q − c|² − r2.
	radialOf(c planePoint, r2 *big.Rat) int
}

func (p planePoint) sideOf(a, b planePoint) int { return side(a, b, p) }

func (p planePoint) spanOf(a, b planePoint) int { return p.sub(a).dot(p.sub(b)).Sign() }

func (p planePoint) heightOf(v *big.Rat) int { return p.v.Cmp(v) }

func (p planePoint) radialOf(c planePoint, r2 *big.Rat) int {
	offset := p.sub(c)
	return offset.dot(offset).Cmp(r2)
}

// side is the exact cross product (b − a) × (q − a): positive with q left of
// the directed line a → b, zero on it.
func side(a, b, q planePoint) int {
	d, e := b.sub(a), q.sub(a)
	return new(big.Rat).Sub(new(big.Rat).Mul(d.u, e.v), new(big.Rat).Mul(e.u, d.v)).Sign()
}

// onSegment reports whether q lies on the closed segment a → b.
func onSegment(a, b planePoint, q regionQuery) bool {
	return q.sideOf(a, b) == 0 && q.spanOf(a, b) <= 0
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

// profileRing is a recorded profile read into exact rings: the outer loop's
// and each hole's.
type profileRing struct {
	outer loopRing
	holes []loopRing
}

// readProfile reads every loop of profile into its ring. ok is false when a
// loop holds a segment, range or junction the route does not read exactly.
func readProfile(budget *proofbound.WorkBudget, profile momentinput.Profile) (profileRing, bool, error) {
	outer, ok, err := readRing(budget, profile.Outer)
	if err != nil || !ok {
		return profileRing{}, false, err
	}
	out := profileRing{outer: outer, holes: make([]loopRing, len(profile.Holes))}
	for i, hole := range profile.Holes {
		out.holes[i], ok, err = readRing(budget, hole)
		if err != nil || !ok {
			return profileRing{}, false, err
		}
	}
	return out, true, nil
}

// closed reports that every loop's junctions close at two equal ends.
func (pr profileRing) closed() bool {
	if !pr.outer.closed {
		return false
	}
	for _, hole := range pr.holes {
		if !hole.closed {
			return false
		}
	}
	return true
}

// locateInProfile classifies q against the region the outer loop bounds less
// every hole's. A point inside two holes, inside a hole but not the outer
// loop, or on more than one loop is undecided: a sketch-arranged record has
// none of these, and the route does not decide what such a record denotes.
func locateInProfile(budget *proofbound.WorkBudget, profile momentinput.Profile, q regionQuery) (regionClass, error) {
	rings, ok, err := readProfile(budget, profile)
	if err != nil || !ok {
		return regionUndecided, err
	}
	return rings.locate(budget, q)
}

// locate is locateInProfile over rings already read.
func (pr profileRing) locate(budget *proofbound.WorkBudget, q regionQuery) (regionClass, error) {
	outer, err := pr.outer.locate(budget, q)
	if err != nil || outer == regionUndecided {
		return regionUndecided, err
	}
	inHoles, onHoles := 0, 0
	for _, hole := range pr.holes {
		class, err := hole.locate(budget, q)
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

// loopRing is one recorded loop read exactly: a single whole circle, or a
// ring of straight edges in walk order with the whole exact-radius arcs whose
// chords stand in it. closed reports that every junction closes at two equal
// ends, so the ring's vertices are the record's own segment ends.
type loopRing struct {
	circle *sectionrecord.CircleSeg
	edges  []ringEdge
	arcs   []ringArc
	closed bool
}

// readRing walks one loop into its ring: each line at its recorded
// parameters, each whole exact-radius arc by its chord, and each junction
// closed as the evaluator closes it (momentinput.JunctionCorner, else the
// straight chord). A lone segment must be a circle. ok is false for any
// segment the route does not read.
func readRing(budget *proofbound.WorkBudget, loop sectionrecord.LoopRecord) (loopRing, bool, error) {
	segments := make([]sectionrecord.CurveSegment, len(loop.Segments))
	for i, segment := range loop.Segments {
		if err := budget.Step(); err != nil {
			return loopRing{}, false, err
		}
		normalized, ok := normalizedSegment(segment)
		if !ok {
			return loopRing{}, false, nil
		}
		segments[i] = normalized
	}
	if len(segments) == 1 {
		circle, ok := segments[0].(sectionrecord.CircleSeg)
		if !ok {
			return loopRing{}, false, nil
		}
		return loopRing{circle: &circle, closed: true}, true, nil
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
				return loopRing{}, false, nil
			}
			starts[i], ends[i] = planePointOf(*start), planePointOf(*end)
		case sectionrecord.ArcSeg:
			arc, ok := exactArc(s)
			if !ok {
				return loopRing{}, false, nil
			}
			starts[i], ends[i] = arc.start, arc.end
			if arc.sign < 0 {
				starts[i], ends[i] = arc.end, arc.start
			}
			arcOf[i] = len(arcs)
			arcs = append(arcs, arc)
		default:
			return loopRing{}, false, nil
		}
	}
	ring := loopRing{arcs: arcs, closed: true}
	for i := range segments {
		if err := budget.Step(); err != nil {
			return loopRing{}, false, err
		}
		ring.edges = append(ring.edges, ringEdge{from: starts[i], to: ends[i], chord: arcOf[i] >= 0})
		j := (i + 1) % len(segments)
		if ends[i].equal(starts[j]) {
			continue
		}
		ring.closed = false
		p := freeform.RatPoint{U: ends[i].u, V: ends[i].v}
		next := freeform.RatPoint{U: starts[j].u, V: starts[j].v}
		if corner, ok := momentinput.JunctionCorner(segments[i], segments[j], &p, &next); ok {
			c := planePointOf(corner)
			ring.edges = append(ring.edges, ringEdge{from: ends[i], to: c}, ringEdge{from: c, to: starts[j]})
			continue
		}
		ring.edges = append(ring.edges, ringEdge{from: ends[i], to: starts[j]})
	}
	return ring, true, nil
}

// locate classifies q against the closed region the loop bounds. A single
// whole circle compares squared distances. Otherwise the winding number of
// the ring about q, corrected by every arc's circular segment that holds q,
// is the loop's winding number.
func (r loopRing) locate(budget *proofbound.WorkBudget, q regionQuery) (regionClass, error) {
	if r.circle != nil {
		return locateInCircle(*r.circle, q), nil
	}
	for _, arc := range r.arcs {
		if arc.holds(q) == regionBoundary {
			return regionBoundary, nil
		}
	}
	for _, edge := range r.edges {
		if !edge.chord && onSegment(edge.from, edge.to, q) {
			return regionBoundary, nil
		}
	}
	winding := 0
	for _, edge := range r.edges {
		if err := budget.Step(); err != nil {
			return regionUndecided, err
		}
		if edge.chord && onSegment(edge.from, edge.to, q) {
			return regionUndecided, nil
		}
		winding += crossingOf(edge, q)
	}
	for _, arc := range r.arcs {
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
func crossingOf(edge ringEdge, q regionQuery) int {
	if q.heightOf(edge.from.v) >= 0 {
		if q.heightOf(edge.to.v) < 0 && q.sideOf(edge.from, edge.to) > 0 {
			return 1
		}
		return 0
	}
	if q.heightOf(edge.to.v) >= 0 && q.sideOf(edge.from, edge.to) < 0 {
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
func locateInCircle(circle sectionrecord.CircleSeg, q regionQuery) regionClass {
	if !isWholeRange(circle.TStart, circle.TEnd) || circle.Radius.Unit() != units.Millimeter {
		return regionUndecided
	}
	center, ok := recordPoint(circle.Center)
	radius := proof.FloatRat(circle.Radius.Mag())
	if !ok || radius == nil || radius.Sign() <= 0 {
		return regionUndecided
	}
	switch q.radialOf(center, new(big.Rat).Mul(radius, radius)) {
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
func (a ringArc) holds(q regionQuery) regionClass {
	radial := q.radialOf(a.center, a.radiusSquared)
	chord := q.sideOf(a.start, a.end)
	switch {
	case radial == 0 && chord <= 0:
		return regionBoundary
	case radial < 0 && chord < 0:
		return regionInside
	default:
		return regionOutside
	}
}
