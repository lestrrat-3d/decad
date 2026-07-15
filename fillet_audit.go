package decad

import (
	"fmt"
	"math"
)

// This file is the §5 audit of a fillet's rewritten section
// (docs/modify-design.md §5), run before any face is built, in the order §4
// fixes: S8 (orientation — the existence question, asked first), S6 (no walk
// consumed by its own corners), then S7 (no crossing). Every test is a
// closed-form fact of decad's own line and arc segments, so its verdict is the
// same under every evaluator — never a residual.
//
// S9 (nesting) is discharged by construction here, but only once S7 rules out
// boundary CONTACT as well as crossing. A fillet is a bounded local rewrite of
// each corner, so a hole can leave the outer region — or two holes overlap —
// only if their boundaries stop being disjoint, and two Jordan loops stop being
// disjoint in exactly two ways: their boundaries CROSS, or they merely TOUCH (a
// tangency, or a shared boundary point with no interior crossing). The large
// fillet that brings the rewritten section's loops into contact is the second
// case, and an interior-only crossing test passes it — a silently inconsistent
// body whose loops pinch. So S7 rejects contact as well as crossing (segCross
// for a transversal crossing, segMinDist for a tangency or shared point), and
// S8 proves no loop inverted; the section sketch admitted was validly nested
// (RecordProfile, core §7). A contact-free, crossing-free, orientation-
// preserving local rewrite of a validly-nested section keeps its nesting, so
// there is no undecidable containment to decline.

// auditRewrite runs the §5 audit on the rewritten profile in §4's order.
func auditRewrite(orig, rewritten ProfileRecord, loops []cornerLoop, filletAt []map[int]*filletData) error {
	origLoops := append([]LoopRecord{orig.Outer}, orig.Holes...)
	newLoops := append([]LoopRecord{rewritten.Outer}, rewritten.Holes...)

	// S8: orientation preserved — a loop whose signed area changed sign (or
	// collapsed) has turned itself inside out; the modification consumed it.
	for i := range origLoops {
		oa, err := loopSignedArea(origLoops[i])
		if err != nil {
			return err
		}
		na, err := loopSignedArea(newLoops[i])
		if err != nil {
			return err
		}
		if math.Signbit(oa) != math.Signbit(na) || math.Abs(na) <= 1e-9*math.Abs(oa) {
			return fmt.Errorf(`%w: the fillet turned a loop inside out — the radius does not fit`, ErrDegenerate)
		}
	}

	// S6: no walk consumed by its own corners — the two ends' cutbacks must fit
	// strictly inside the walk they claim.
	for li, cl := range loops {
		n := len(cl.walks)
		for i, w := range cl.walks {
			cut := 0.0
			if fd := filletAt[li][i]; fd != nil {
				cut += fd.cutbackB
			}
			if fd := filletAt[li][(i+1)%n]; fd != nil {
				cut += fd.cutbackA
			}
			if cut >= w.length-1e-9*math.Max(1, w.length) {
				return fmt.Errorf(`%w: two fillets claim the same wall from both ends; merging them is not supported`, ErrUnsupported)
			}
		}
	}

	// S7: no crossing AND no boundary contact — any pair of non-adjacent
	// segments meeting in both their interiors is a self-intersection, and a
	// pair that merely touches (a tangency, or a shared boundary point) is a
	// pinch; either is a rewrite a resolving kernel would have to trim.
	return crossingAudit(newLoops)
}

// loopSignedArea is one loop's signed area (positive counter-clockwise): the
// Green's-theorem boundary integral of its own segments.
func loopSignedArea(loop LoopRecord) (float64, error) {
	var ig regionIntegrals
	for _, seg := range loop.Segments {
		if err := ig.add(seg); err != nil {
			return 0, err
		}
	}
	return ig.area, nil
}

// segEntry is one rewritten segment as a boundary primitive, tagged by the loop
// and position it came from (for adjacency).
type segEntry struct {
	loop int
	idx  int
	n    int // segments in this loop
	w    segmentWalk
}

// crossingAudit tests every pair of segments for an interior crossing OR a
// boundary contact — a tangency or a shared boundary point — skipping only
// pairs that legitimately share an endpoint (same-loop neighbours). A crossing
// and a contact both stop two Jordan loops being disjoint, so both must be
// rejected before S9's nesting can be discharged by construction: an
// interior-only crossing test would pass a large fillet whose rewritten loops
// merely pinch, and Body.Tessellate would then refuse the very body Fillet
// returned. Both are S7 (ErrUnsupported): the touch is the boundary case of a
// crossing, a body a resolving/trimmed-offset kernel could build but this
// evaluator cannot (§1 existence test — the body exists; §4 Table S, S7).
func crossingAudit(loops []LoopRecord) error {
	var segs []segEntry
	for li, loop := range loops {
		n := len(loop.Segments)
		for i, seg := range loop.Segments {
			w, err := walkOf(seg)
			if err != nil {
				return err
			}
			segs = append(segs, segEntry{loop: li, idx: i, n: n, w: w})
		}
	}
	for i := range segs {
		for j := i + 1; j < len(segs); j++ {
			if adjacent(segs[i], segs[j]) {
				continue
			}
			if segCross(segs[i].w, segs[j].w) {
				return fmt.Errorf(`%w: the fillet rewrite crosses itself; a resolving kernel is not available`, ErrUnsupported)
			}
			if segMinDist(segs[i].w, segs[j].w) <= segTouchEps {
				return fmt.Errorf(`%w: the fillet rewrite brings two boundaries into contact; a resolving kernel is not available`, ErrUnsupported)
			}
		}
	}
	return nil
}

// segTouchEps is the absolute contact tolerance (millimetres) for the
// boundary-contact test. The rewritten section is decad's own exact geometry,
// so a true pinch reads as a distance at float-noise scale (the crossing of
// square-root coordinates lands within ~1e-13 of zero) while any valid gap
// between two distinct features is macroscopic; the band between them holds no
// admissible body, so this only absorbs float noise, never a residual.
const segTouchEps = 1e-7

// adjacent reports whether two segments legitimately share an endpoint: the
// same loop and cyclically consecutive positions.
func adjacent(a, b segEntry) bool {
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
func segCross(a, b segmentWalk) bool {
	switch {
	case !a.circular && !b.circular:
		return lineLineSegCross(a, b)
	case a.circular && b.circular:
		return arcArcSegCross(a, b)
	case a.circular:
		return lineArcSegCross(b, a)
	default:
		return lineArcSegCross(a, b)
	}
}

// lineLineSegCross reports an interior crossing of two line segments.
func lineLineSegCross(a, b segmentWalk) bool {
	rx, ry := a.endU-a.startU, a.endV-a.startV
	sx, sy := b.endU-b.startU, b.endV-b.startV
	den := rx*sy - ry*sx
	if math.Abs(den) <= filletTol {
		return false // parallel: no transversal crossing
	}
	qpx, qpy := b.startU-a.startU, b.startV-a.startV
	t := (qpx*sy - qpy*sx) / den
	u := (qpx*ry - qpy*rx) / den
	return interior(t) && interior(u)
}

// lineArcSegCross reports an interior crossing of a line segment and an arc.
func lineArcSegCross(line, arc segmentWalk) bool {
	dx, dy := line.endU-line.startU, line.endV-line.startV
	dl := math.Hypot(dx, dy)
	if dl <= filletTol {
		return false
	}
	ux, uy := dx/dl, dy/dl
	fx, fy := line.startU-arc.cU, line.startV-arc.cV
	bb := fx*ux + fy*uy
	cc := fx*fx + fy*fy - arc.radius*arc.radius
	disc := bb*bb - cc
	if disc < 0 {
		return false
	}
	sq := math.Sqrt(disc)
	for _, s := range []float64{-bb + sq, -bb - sq} {
		x, y := line.startU+s*ux, line.startV+s*uy
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
func arcArcSegCross(a, b segmentWalk) bool {
	pts := circleCircle(a.cU, a.cV, a.radius, b.cU, b.cV, b.radius)
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
func angleInterior(arc segmentWalk, x, y float64) bool {
	lo, hi := math.Min(arc.th0, arc.th1), math.Max(arc.th0, arc.th1)
	a := math.Atan2(y-arc.cV, x-arc.cU)
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
func angleWithin(arc segmentWalk, x, y float64) bool {
	lo, hi := math.Min(arc.th0, arc.th1), math.Max(arc.th0, arc.th1)
	a := math.Atan2(y-arc.cV, x-arc.cU)
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
// is the boundary-contact classifier behind S7: a value at or below segTouchEps
// is a tangency or a shared boundary point the interior-only crossing test
// misses. The candidate set is complete for the attained infimum over line/arc
// boundaries — the four endpoint-against-the-other distances, the interior
// radial/aligned criticals, and zero at any interior intersection.
func segMinDist(a, b segmentWalk) float64 {
	switch {
	case !a.circular && !b.circular:
		return lineLineMinDist(a, b)
	case a.circular && b.circular:
		return arcArcMinDist(a, b)
	case a.circular:
		return lineArcMinDist(b, a)
	default:
		return lineArcMinDist(a, b)
	}
}

// pointLineSegDist is the distance from (px, py) to the closed line segment.
func pointLineSegDist(px, py float64, l segmentWalk) float64 {
	dx, dy := l.endU-l.startU, l.endV-l.startV
	l2 := dx*dx + dy*dy
	if l2 <= filletTol*filletTol {
		return math.Hypot(px-l.startU, py-l.startV)
	}
	t := ((px-l.startU)*dx + (py-l.startV)*dy) / l2
	t = math.Max(0, math.Min(1, t))
	return math.Hypot(px-(l.startU+t*dx), py-(l.startV+t*dy))
}

// pointArcDist is the distance from (px, py) to the closed arc: the radial gap
// when the point's bearing falls within the walk range, else the nearer of the
// two arc endpoints.
func pointArcDist(px, py float64, a segmentWalk) float64 {
	best := math.Min(math.Hypot(px-a.startU, py-a.startV), math.Hypot(px-a.endU, py-a.endV))
	dc := math.Hypot(px-a.cU, py-a.cV)
	if dc > filletTol && angleWithin(a, px, py) {
		best = math.Min(best, math.Abs(dc-a.radius))
	}
	return best
}

// lineLineMinDist is the minimum distance between two closed line segments:
// zero where their interiors cross, else the nearest endpoint-to-segment reach.
func lineLineMinDist(a, b segmentWalk) float64 {
	if lineLineSegCross(a, b) {
		return 0
	}
	return math.Min(
		math.Min(pointLineSegDist(a.startU, a.startV, b), pointLineSegDist(a.endU, a.endV, b)),
		math.Min(pointLineSegDist(b.startU, b.startV, a), pointLineSegDist(b.endU, b.endV, a)),
	)
}

// lineArcMinDist is the minimum distance between a closed line segment and a
// closed arc.
func lineArcMinDist(line, arc segmentWalk) float64 {
	if lineArcSegCross(line, arc) {
		return 0
	}
	best := math.Min(
		math.Min(pointArcDist(line.startU, line.startV, arc), pointArcDist(line.endU, line.endV, arc)),
		math.Min(pointLineSegDist(arc.startU, arc.startV, line), pointLineSegDist(arc.endU, arc.endV, line)),
	)
	// Interior critical: the arc point along the perpendicular from the centre
	// to the line, when its foot is interior to the segment and its bearing is
	// within the walk — the tangency/near-approach the endpoints miss.
	dx, dy := line.endU-line.startU, line.endV-line.startV
	l2 := dx*dx + dy*dy
	if l2 > filletTol*filletTol {
		s := ((arc.cU-line.startU)*dx + (arc.cV-line.startV)*dy) / l2
		if s > 0 && s < 1 {
			fx, fy := line.startU+s*dx, line.startV+s*dy
			ux, uy := fx-arc.cU, fy-arc.cV
			ul := math.Hypot(ux, uy)
			if ul > filletTol {
				cpx, cpy := arc.cU+arc.radius*ux/ul, arc.cV+arc.radius*uy/ul
				if angleWithin(arc, cpx, cpy) {
					best = math.Min(best, pointLineSegDist(cpx, cpy, line))
				}
			}
		}
	}
	return best
}

// arcArcMinDist is the minimum distance between two closed arcs.
func arcArcMinDist(a, b segmentWalk) float64 {
	if arcArcSegCross(a, b) {
		return 0
	}
	best := math.Min(
		math.Min(pointArcDist(a.startU, a.startV, b), pointArcDist(a.endU, a.endV, b)),
		math.Min(pointArcDist(b.startU, b.startV, a), pointArcDist(b.endU, b.endV, a)),
	)
	dcx, dcy := b.cU-a.cU, b.cV-a.cV
	d := math.Hypot(dcx, dcy)
	if d > filletTol {
		ux, uy := dcx/d, dcy/d
		// The mutually nearest points of two non-concentric circles lie on the
		// centre line; test each circle's two axis points against the other arc,
		// which captures external and internal tangency alike.
		for _, s := range []float64{1, -1} {
			pax, pay := a.cU+s*a.radius*ux, a.cV+s*a.radius*uy
			if angleWithin(a, pax, pay) {
				best = math.Min(best, pointArcDist(pax, pay, b))
			}
			pbx, pby := b.cU+s*b.radius*ux, b.cV+s*b.radius*uy
			if angleWithin(b, pbx, pby) {
				best = math.Min(best, pointArcDist(pbx, pby, a))
			}
		}
	} else if arcSpansOverlap(a, b) {
		// Concentric arcs whose walks overlap in bearing: the radial gap.
		best = math.Min(best, math.Abs(a.radius-b.radius))
	}
	return best
}

// arcSpansOverlap reports whether two concentric arcs' walk ranges share any
// bearing — an endpoint of one falling within the other's range.
func arcSpansOverlap(a, b segmentWalk) bool {
	return angleWithin(b, a.startU, a.startV) || angleWithin(b, a.endU, a.endV) ||
		angleWithin(a, b.startU, b.startV) || angleWithin(a, b.endU, b.endV)
}
