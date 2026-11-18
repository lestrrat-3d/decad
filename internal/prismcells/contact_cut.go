package prismcells

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/geom"
)

// ContactsCut reports whether every point where two entities of different
// operands touch, other than a transverse crossing inside both, is a scene
// vertex where an edge of each ends, within crossingMatchBand of the point.
// Two boundaries crossing transversally inside both entities leave their four
// half-edges alternating round the point whether or not the arrangement cut
// it. Every other contact can pinch a face or a solid there, and a caller's
// cell reading sees it only at a vertex:
//
//   - a point within lineContactBand of a line's end, or within
//     crossingMatchBand of an arc's end, such as a corner resting on the other
//     side's edge or on its corner;
//   - a tangency, which sketch's query reads as one point on the carriers: a
//     line tangent to a circle, or two tangent circles. The arrangement cuts no
//     tangency, so one passes only where edges end there anyway, as where a
//     window's corner round meets a boss circle at a shared span's end.
//
// The points are sketch's own queries' (geom.LineLineIntersection,
// lineCurvePoints, geom.CircleCircleIntersections), each kept within its
// band of both entities. A span two entities share is emitted under one of
// them, so its partner counts as ending wherever an edge of the span ends
// (CoincidentReading.Lookup), and the two entities of one span are not put to
// the queries. A curve kind the queries do not cover refuses. The check only
// refuses: a matched vertex admits nothing.
func ContactsCut(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin,
	profiles []*sketch.Profile, coincident CoincidentReading) (bool, error) {
	at, ok, err := arrangedVertexEntities(budget, profiles)
	if err != nil || !ok {
		return false, err
	}
	for _, cell := range profiles {
		for _, ring := range append([][]sketch.BoundaryEdge{cell.Outer}, cell.Holes...) {
			for _, e := range ring {
				if err := budget.Step(); err != nil {
					return false, err
				}
				c, shared := coincident.Lookup(e.Entity, e.TStart, e.TEnd)
				if !shared {
					continue
				}
				for _, p := range [][2]float64{e.Polyline[0], e.Polyline[len(e.Polyline)-1]} {
					at[p][c.Partner] = struct{}{}
				}
			}
		}
	}
	entities := make([]sketch.Entity, 0, len(tags))
	for e := range tags {
		entities = append(entities, e)
	}
	for i, a := range entities {
		for _, b := range entities[i+1:] {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if tags[a].IsB == tags[b].IsB || coincident.Partners(a, b) {
				continue
			}
			points, tangent, covered := contactPoints(a, b)
			if !covered {
				return false, nil
			}
			for _, q := range points {
				if !tangent && !atEnd(a, q) && !atEnd(b, q) {
					continue
				}
				cut, err := crossingVertex(budget, at, a, b, q)
				if err != nil || !cut {
					return false, err
				}
			}
		}
	}
	return true, nil
}

// contactPoints is sketch's reading of where two entities meet: the points
// on both, and whether the query read the carriers as tangent (one point for
// a line and a curve, or two circles). Parallel lines and concentric circles
// report no point. covered is false for a kind the queries do not cover.
func contactPoints(a, b sketch.Entity) ([]*geom.Point, bool, bool) {
	la, aLine := a.(*sketch.Line)
	lb, bLine := b.(*sketch.Line)
	switch {
	case aLine && bLine:
		ga, gb := la.Geometry(), lb.Geometry()
		p, ok := geom.LineLineIntersection(ga, gb)
		if !ok || !withinSegment(ga, p) || !withinSegment(gb, p) {
			return nil, false, true
		}
		return []*geom.Point{p}, false, true
	case aLine:
		return lineCurvePoints(la, b)
	case bLine:
		return lineCurvePoints(lb, a)
	}
	ca, arcA, okA := curveCarrier(a)
	cb, arcB, okB := curveCarrier(b)
	if !okA || !okB {
		return nil, false, false
	}
	raw := geom.CircleCircleIntersections(ca, cb)
	var on []*geom.Point
	for _, p := range raw {
		if (arcA != nil && !arcA.Contains(p)) || (arcB != nil && !arcB.Contains(p)) {
			continue
		}
		on = append(on, p)
	}
	return on, len(raw) == 1, true
}

// curveCarrier is a circle's or an arc's carrier circle, and the arc itself
// for an arc.
func curveCarrier(e sketch.Entity) (*geom.Circle, *geom.Arc, bool) {
	switch c := e.(type) {
	case *sketch.Circle:
		return c.Geometry(), nil, true
	case *sketch.Arc:
		arc := c.Geometry()
		return &geom.Circle{Center: arc.Center, Radius: arc.Radius()}, arc, true
	}
	return nil, nil, false
}

// atEnd reports whether q lies at one end of e: within lineContactBand of a
// line's parameter 0 or 1, or within crossingMatchBand of an arc's end. A
// circle has no end.
func atEnd(e sketch.Entity, q *geom.Point) bool {
	switch c := e.(type) {
	case *sketch.Line:
		_, t := geom.ClosestPointOnLine(c.Geometry(), q)
		return t <= lineContactBand || t >= 1-lineContactBand
	case *sketch.Arc:
		arc := c.Geometry()
		band := crossingMatchBand * matchScale(q, e)
		return math.Hypot(arc.Start.X-q.X, arc.Start.Y-q.Y) <= band ||
			math.Hypot(arc.End.X-q.X, arc.End.Y-q.Y) <= band
	}
	return false
}

// matchScale is crossingMatchBand's scale at q: one, q's coordinates, and
// the radius of every curved entity named.
func matchScale(q *geom.Point, entities ...sketch.Entity) float64 {
	scale := max(1, math.Abs(q.X), math.Abs(q.Y))
	for _, e := range entities {
		switch c := e.(type) {
		case *sketch.Circle:
			scale = max(scale, c.Geometry().Radius)
		case *sketch.Arc:
			scale = max(scale, c.Geometry().Radius())
		}
	}
	return scale
}
