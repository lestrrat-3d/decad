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
// S9 (nesting) is discharged by construction here: a fillet is a bounded local
// rewrite of each corner, so a hole can leave the outer region — or two holes
// overlap — only by their boundaries crossing (Jordan's theorem). S7 proves no
// two loops cross and S8 proves no loop inverted; the section sketch admitted
// was validly nested (RecordProfile, core §7). A crossing-free,
// orientation-preserving local rewrite of a validly-nested section keeps its
// nesting, so there is no undecidable containment to decline.

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

	// S7: no crossing — any pair of non-adjacent segments meeting in both their
	// interiors is a self-intersection a resolving kernel would have to trim.
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

// crossingAudit tests every pair of segments for an interior intersection,
// skipping pairs that legitimately share an endpoint (same-loop neighbours).
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
		}
	}
	return nil
}

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
