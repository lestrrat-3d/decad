package brepgeom

import (
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// NaturalRange reports whether a recorded segment is a line, arc or circle
// over its whole natural range, 0→1 or 1→0.
func NaturalRange(seg sectionrecord.CurveSegment) bool {
	var t0, t1 float64
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		t0, t1 = s.TStart, s.TEnd
	case sectionrecord.ArcSeg:
		t0, t1 = s.TStart, s.TEnd
	case sectionrecord.CircleSeg:
		t0, t1 = s.TStart, s.TEnd
	default:
		return false
	}
	return (t0 == 0 && t1 == 1) || (t0 == 1 && t1 == 0)
}

// BrepCornerAt finds the corner of a planar face's corner walk whose
// junction lies at reference coordinates v: the walk start that maps there
// exactly (brep-modify §5.3 step 1). It reports false when none or several
// do.
func BrepCornerAt(loops []survey2d.CornerLoop, e Embed, level float64, v [3]float64) (int, int, bool) {
	li, ci, n := 0, 0, 0
	for l, loop := range loops {
		for c, w := range loop.Walks {
			if w.Closed || e.Canon(w.StartU, w.StartV, level) != v {
				continue
			}
			li, ci = l, c
			n++
		}
	}
	return li, ci, n == 1
}

// ReverseSegment walks a natural-range line, arc or circle the other way. A
// line or arc is the same record over the reversed range. A circle flips its
// CCW sense and swaps its range, so its centre and radius are unchanged.
func ReverseSegment(seg sectionrecord.CurveSegment) sectionrecord.CurveSegment {
	switch s := seg.(type) {
	case sectionrecord.CircleSeg:
		s.CCW = !s.CCW
		s.TStart, s.TEnd = s.TEnd, s.TStart
		return s
	case sectionrecord.LineSeg:
		s.TStart, s.TEnd = s.TEnd, s.TStart
		return s
	case sectionrecord.ArcSeg:
		s.TStart, s.TEnd = s.TEnd, s.TStart
		return s
	default:
		return seg
	}
}
