package sectionaudit

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// adjacent reports whether two segments legitimately share an endpoint: the
// same loop and cyclically consecutive positions.
func adjacent(a, b Entry) bool {
	if a.loop != b.loop {
		return false
	}
	d := a.idx - b.idx
	if d < 0 {
		d = -d
	}
	return d == 1 || d == a.n-1
}

// segCrossEps is the interior margin for the crossing test: an intersection
// within it of a segment's endpoint is a shared vertex, not a crossing.
const segCrossEps = 1e-7

// segCross reports whether two segment primitives meet in both their interiors.
func segCross(a, b survey2d.SegmentWalk) bool {
	switch {
	case a.IsLine() && b.IsLine():
		return lineLineSegCross(a, b)
	case a.IsCircular() && b.IsCircular():
		return arcArcSegCross(a, b)
	case a.IsCircular():
		return lineArcSegCross(b, a)
	default:
		return lineArcSegCross(a, b)
	}
}

// lineLineSegCross reports an interior crossing of two line segments.
func lineLineSegCross(a, b survey2d.SegmentWalk) bool {
	rx, ry := a.EndU-a.StartU, a.EndV-a.StartV
	sx, sy := b.EndU-b.StartU, b.EndV-b.StartV
	den := rx*sy - ry*sx
	if math.Abs(den) <= Tolerance {
		return false // parallel: no transversal crossing
	}
	qpx, qpy := b.StartU-a.StartU, b.StartV-a.StartV
	t := (qpx*sy - qpy*sx) / den
	u := (qpx*ry - qpy*rx) / den
	return interior(t) && interior(u)
}

// lineArcSegCross reports an interior crossing of a line segment and an arc.
func lineArcSegCross(line, arc survey2d.SegmentWalk) bool {
	dx, dy := line.EndU-line.StartU, line.EndV-line.StartV
	dl := math.Hypot(dx, dy)
	if dl <= Tolerance {
		return false
	}
	ux, uy := dx/dl, dy/dl
	fx, fy := line.StartU-arc.CU, line.StartV-arc.CV
	bb := fx*ux + fy*uy
	cc := fx*fx + fy*fy - arc.Radius*arc.Radius
	disc := bb*bb - cc
	if disc < 0 {
		return false
	}
	sq := math.Sqrt(disc)
	for _, s := range []float64{-bb + sq, -bb - sq} {
		x, y := line.StartU+s*ux, line.StartV+s*uy
		t := s / dl
		if !interior(t) {
			continue
		}
		if angleInterior(arc, x, y) {
			return true
		}
	}
	return false
}

// arcArcSegCross reports an interior crossing of two arcs.
func arcArcSegCross(a, b survey2d.SegmentWalk) bool {
	pts := CircleCircle(a.CU, a.CV, a.Radius, b.CU, b.CV, b.Radius)
	for _, p := range pts {
		if angleInterior(a, p[0], p[1]) && angleInterior(b, p[0], p[1]) {
			return true
		}
	}
	return false
}

// interior reports whether a segment parameter is strictly inside (0, 1).
func interior(t float64) bool { return t > segCrossEps && t < 1-segCrossEps }

// angleInterior reports whether the point (x, y) lies strictly inside the arc's
// angular walk range.
func angleInterior(arc survey2d.SegmentWalk, x, y float64) bool {
	lo, hi := math.Min(arc.Th0, arc.Th1), math.Max(arc.Th0, arc.Th1)
	a := math.Atan2(y-arc.CV, x-arc.CU)
	for k := math.Floor((lo-a)/(2*math.Pi)) * 2 * math.Pi; a+k <= hi+segCrossEps; k += 2 * math.Pi {
		th := a + k
		if th > lo+segCrossEps && th < hi-segCrossEps {
			return true
		}
	}
	return false
}

// angleWithin reports whether (x, y) lies within the arc's angular walk range,
// inclusive of its endpoints — the membership the minimum-distance candidates
// need (a nearest point may sit at an arc's own end).
func angleWithin(arc survey2d.SegmentWalk, x, y float64) bool {
	lo, hi := math.Min(arc.Th0, arc.Th1), math.Max(arc.Th0, arc.Th1)
	a := math.Atan2(y-arc.CV, x-arc.CU)
	for k := math.Floor((lo-a)/(2*math.Pi)) * 2 * math.Pi; a+k <= hi+segCrossEps; k += 2 * math.Pi {
		th := a + k
		if th >= lo-segCrossEps && th <= hi+segCrossEps {
			return true
		}
	}
	return false
}

// segMinDist is the minimum Euclidean distance between two closed segment
// primitives (line or arc), in closed form over their own line and arc data. It
// is the boundary-contact classifier behind S7: a value at or below the
// section-scaled ContactFloor is a tangency or a shared boundary point the
// interior-only crossing test misses. The candidate set is complete for the attained infimum over line/arc
// boundaries — the four endpoint-against-the-other distances, the interior
// radial/aligned criticals, and zero at any interior intersection.
func segMinDist(a, b survey2d.SegmentWalk) float64 {
	switch {
	case a.IsLine() && b.IsLine():
		return lineLineMinDist(a, b)
	case a.IsCircular() && b.IsCircular():
		return arcArcMinDist(a, b)
	case a.IsCircular():
		return lineArcMinDist(b, a)
	default:
		return lineArcMinDist(a, b)
	}
}

// pointLineSegDist is the distance from (px, py) to the closed line segment.
func pointLineSegDist(px, py float64, l survey2d.SegmentWalk) float64 {
	dx, dy := l.EndU-l.StartU, l.EndV-l.StartV
	l2 := dx*dx + dy*dy
	if l2 <= Tolerance*Tolerance {
		return math.Hypot(px-l.StartU, py-l.StartV)
	}
	t := ((px-l.StartU)*dx + (py-l.StartV)*dy) / l2
	t = math.Max(0, math.Min(1, t))
	return math.Hypot(px-(l.StartU+t*dx), py-(l.StartV+t*dy))
}

// pointArcDist is the distance from (px, py) to the closed arc: the radial gap
// when the point's bearing falls within the walk range, else the nearer of the
// two arc endpoints.
func pointArcDist(px, py float64, a survey2d.SegmentWalk) float64 {
	best := math.Min(math.Hypot(px-a.StartU, py-a.StartV), math.Hypot(px-a.EndU, py-a.EndV))
	dc := math.Hypot(px-a.CU, py-a.CV)
	if dc > Tolerance && angleWithin(a, px, py) {
		best = math.Min(best, math.Abs(dc-a.Radius))
	}
	return best
}

// lineLineMinDist is the minimum distance between two closed line segments:
// zero where their interiors cross, else the nearest endpoint-to-segment reach.
func lineLineMinDist(a, b survey2d.SegmentWalk) float64 {
	if lineLineSegCross(a, b) {
		return 0
	}
	return math.Min(
		math.Min(pointLineSegDist(a.StartU, a.StartV, b), pointLineSegDist(a.EndU, a.EndV, b)),
		math.Min(pointLineSegDist(b.StartU, b.StartV, a), pointLineSegDist(b.EndU, b.EndV, a)),
	)
}

// lineArcMinDist is the minimum distance between a closed line segment and a
// closed arc.
func lineArcMinDist(line, arc survey2d.SegmentWalk) float64 {
	if lineArcSegCross(line, arc) {
		return 0
	}
	best := math.Min(
		math.Min(pointArcDist(line.StartU, line.StartV, arc), pointArcDist(line.EndU, line.EndV, arc)),
		math.Min(pointLineSegDist(arc.StartU, arc.StartV, line), pointLineSegDist(arc.EndU, arc.EndV, line)),
	)
	// Interior critical: the arc point along the perpendicular from the centre
	// to the line, when its foot is interior to the segment and its bearing is
	// within the walk — the tangency/near-approach the endpoints miss.
	dx, dy := line.EndU-line.StartU, line.EndV-line.StartV
	l2 := dx*dx + dy*dy
	if l2 > Tolerance*Tolerance {
		s := ((arc.CU-line.StartU)*dx + (arc.CV-line.StartV)*dy) / l2
		if s > 0 && s < 1 {
			fx, fy := line.StartU+s*dx, line.StartV+s*dy
			ux, uy := fx-arc.CU, fy-arc.CV
			ul := math.Hypot(ux, uy)
			if ul > Tolerance {
				cpx, cpy := arc.CU+arc.Radius*ux/ul, arc.CV+arc.Radius*uy/ul
				if angleWithin(arc, cpx, cpy) {
					best = math.Min(best, pointLineSegDist(cpx, cpy, line))
				}
			}
		}
	}
	return best
}

// arcArcMinDist is the minimum distance between two closed arcs.
func arcArcMinDist(a, b survey2d.SegmentWalk) float64 {
	if arcArcSegCross(a, b) {
		return 0
	}
	best := math.Min(
		math.Min(pointArcDist(a.StartU, a.StartV, b), pointArcDist(a.EndU, a.EndV, b)),
		math.Min(pointArcDist(b.StartU, b.StartV, a), pointArcDist(b.EndU, b.EndV, a)),
	)
	dcx, dcy := b.CU-a.CU, b.CV-a.CV
	d := math.Hypot(dcx, dcy)
	if d > Tolerance {
		ux, uy := dcx/d, dcy/d
		// The mutually nearest points of two non-concentric circles lie on the
		// centre line; test each circle's two axis points against the other arc,
		// which captures external and internal tangency alike.
		for _, s := range []float64{1, -1} {
			pax, pay := a.CU+s*a.Radius*ux, a.CV+s*a.Radius*uy
			if angleWithin(a, pax, pay) {
				best = math.Min(best, pointArcDist(pax, pay, b))
			}
			pbx, pby := b.CU+s*b.Radius*ux, b.CV+s*b.Radius*uy
			if angleWithin(b, pbx, pby) {
				best = math.Min(best, pointArcDist(pbx, pby, a))
			}
		}
	} else if arcSpansOverlap(a, b) {
		// Concentric arcs whose walks overlap in bearing: the radial gap.
		best = math.Min(best, math.Abs(a.Radius-b.Radius))
	}
	return best
}

// arcSpansOverlap reports whether two concentric arcs' walk ranges share any
// bearing — an endpoint of one falling within the other's range.
func arcSpansOverlap(a, b survey2d.SegmentWalk) bool {
	return angleWithin(b, a.StartU, a.StartV) || angleWithin(b, a.EndU, a.EndV) ||
		angleWithin(a, b.StartU, b.StartV) || angleWithin(a, b.EndU, b.EndV)
}

// CircleCircle intersects two circles.
func CircleCircle(c0x, c0y, r0, c1x, c1y, r1 float64) [][2]float64 {
	dx, dy := c1x-c0x, c1y-c0y
	dsq := dx*dx + dy*dy
	d := math.Sqrt(dsq)
	if d <= Tolerance || d > r0+r1+Tolerance || d < math.Abs(r0-r1)-Tolerance {
		return nil
	}
	a := (dsq + r0*r0 - r1*r1) / (2 * d)
	h2 := r0*r0 - a*a
	if h2 < 0 {
		h2 = 0
	}
	h := math.Sqrt(h2)
	mx, my := c0x+a*dx/d, c0y+a*dy/d
	ox, oy := -dy/d*h, dx/d*h
	return [][2]float64{{mx + ox, my + oy}, {mx - ox, my - oy}}
}
