package sectionhistory

import (
	"slices"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// permutation names one signed axis permutation of a plane frame: the
// candidate's U is su times (V when swap, else U) and its V is sv times the
// other axis. Every candidate spans the requested plane.
type permutation struct {
	swap   bool
	su, sv float64
}

// permutations lists the eight candidates, identity first, then the
// orientation-preserving ones, then the reflections.
var permutations = []permutation{
	{swap: false, su: 1, sv: 1},
	{swap: false, su: -1, sv: -1},
	{swap: true, su: 1, sv: -1},
	{swap: true, su: -1, sv: 1},
	{swap: false, su: 1, sv: -1},
	{swap: false, su: -1, sv: 1},
	{swap: true, su: 1, sv: 1},
	{swap: true, su: -1, sv: -1},
}

// frame builds the candidate frame at plane's origin. The identity candidate
// is plane itself. Any other is rebuilt by r3.NewFrame, which normalizes and
// may move the held axes of a tilted frame, so ok is false unless its held
// origin, U and V are bit for bit the requested origin and the signed
// permutation of the requested held axes.
func (p permutation) frame(plane r3.Frame) (r3.Frame, bool) {
	if !p.swap && p.su > 0 && p.sv > 0 {
		return plane, true
	}
	a, b := plane.U(), plane.V()
	if p.swap {
		a, b = b, a
	}
	u, v := a.Scale(p.su), b.Scale(p.sv)
	f, err := r3.NewFrame(plane.Origin(), u, v)
	if err != nil || f.Origin() != plane.Origin() || f.U() != u || f.V() != v {
		return r3.Frame{}, false
	}
	return f, true
}

// reflects reports whether the permutation reverses the plane's orientation,
// so a counter-clockwise loop in the candidate frame is clockwise in plane.
func (p permutation) reflects() bool {
	det := p.su * p.sv
	if p.swap {
		det = -det
	}
	return det < 0
}

// point maps a candidate-frame local point to plane's local coordinates. A
// point at u'U' + v'V' is the same world point in plane's held basis, because
// U' and V' are plane's held axes negated or swapped bit for bit. Negation and
// a swap of binary64 values are exact; 0 - x keeps a zero coordinate +0. This
// is a sign and order change of two plane coordinates, which r3's 3D types do
// not express.
func (p permutation) point(q sectionrecord.Point2) sectionrecord.Point2 {
	sign := func(s, x float64) float64 {
		if s < 0 {
			return 0 - x
		}
		return x
	}
	u, v := sign(p.su, q.U), sign(p.sv, q.V)
	if p.swap {
		return sectionrecord.Point2{U: v, V: u}
	}
	return sectionrecord.Point2{U: u, V: v}
}

// mapBack maps a profile of whole lines, arcs and circles from the candidate
// frame into plane's coordinates. A reflection re-walks every loop backwards,
// so the outer loop stays counter-clockwise and holes clockwise: a line swaps
// its ends, an arc swaps its pinned ends and keeps its range, and a circle keeps
// its winding and range, because the reflection and the reversed walk cancel.
// A whole circle's parameter origin moves with the frame, which a closed whole
// edge does not observe.
func (p permutation) mapBack(profile momentinput.Profile) momentinput.Profile {
	loop := func(in sectionrecord.LoopRecord) sectionrecord.LoopRecord {
		out := make([]sectionrecord.CurveSegment, len(in.Segments))
		for i, seg := range in.Segments {
			out[i] = p.segment(seg)
		}
		if p.reflects() {
			slices.Reverse(out)
		}
		return sectionrecord.LoopRecord{Segments: out}
	}
	result := momentinput.Profile{Outer: loop(profile.Outer)}
	for _, hole := range profile.Holes {
		result.Holes = append(result.Holes, loop(hole))
	}
	return result
}

// segment maps one whole segment wholeCurves admitted; the loop order is
// mapBack's.
func (p permutation) segment(seg sectionrecord.CurveSegment) sectionrecord.CurveSegment {
	switch seg := seg.(type) {
	case sectionrecord.LineSeg:
		start, end := p.point(seg.Start), p.point(seg.End)
		if p.reflects() {
			start, end = end, start
		}
		return sectionrecord.LineSeg{Start: start, End: end, TStart: 0, TEnd: 1}
	case sectionrecord.ArcSeg:
		start, end := p.point(seg.Start), p.point(seg.End)
		if p.reflects() {
			start, end = end, start
		}
		return sectionrecord.ArcSeg{Center: p.point(seg.Center), Start: start, End: end, TStart: seg.TStart, TEnd: seg.TEnd}
	case sectionrecord.CircleSeg:
		seg.Center = p.point(seg.Center)
		return seg
	}
	return seg
}

// wholeCurves restates every line as a forward whole LineSeg and keeps every
// whole arc and whole circle. A narrowed segment enters the scene at a computed
// walked end, and any other kind has no record this path rebuilds; either
// returns ok=false.
func wholeCurves(profile momentinput.Profile) (momentinput.Profile, bool) {
	loop := func(in sectionrecord.LoopRecord) (sectionrecord.LoopRecord, bool) {
		out := make([]sectionrecord.CurveSegment, len(in.Segments))
		for i, seg := range in.Segments {
			switch seg := seg.(type) {
			case sectionrecord.LineSeg:
				switch {
				case seg.TStart == 0 && seg.TEnd == 1:
					out[i] = seg
				case seg.TStart == 1 && seg.TEnd == 0:
					out[i] = sectionrecord.LineSeg{Start: seg.End, End: seg.Start, TStart: 0, TEnd: 1}
				default:
					return sectionrecord.LoopRecord{}, false
				}
			case sectionrecord.ArcSeg:
				if seg.TStart == seg.TEnd || !prismcells.WholeSegmentRange(seg.TStart, seg.TEnd) {
					return sectionrecord.LoopRecord{}, false
				}
				out[i] = seg
			case sectionrecord.CircleSeg:
				if len(in.Segments) != 1 || seg.TStart == seg.TEnd || !prismcells.WholeSegmentRange(seg.TStart, seg.TEnd) {
					return sectionrecord.LoopRecord{}, false
				}
				out[i] = seg
			default:
				return sectionrecord.LoopRecord{}, false
			}
		}
		return sectionrecord.LoopRecord{Segments: out}, true
	}
	outer, ok := loop(profile.Outer)
	if !ok {
		return momentinput.Profile{}, false
	}
	result := momentinput.Profile{Outer: outer}
	for _, hole := range profile.Holes {
		h, ok := loop(hole)
		if !ok {
			return momentinput.Profile{}, false
		}
		result.Holes = append(result.Holes, h)
	}
	return result, true
}
