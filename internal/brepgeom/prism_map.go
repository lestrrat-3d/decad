package brepgeom

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// PrismMap carries recorded face geometry between frames whose normals lie
// on one reference axis. A reflection reverses loop order and rewinds natural
// line and arc walks. A whole circle maps its centre alone.
type PrismMap struct {
	from, to Embed
	reflect  bool
}

// NewPrismMap records the signed permutation between two face frames.
func NewPrismMap(from, to Embed) PrismMap {
	a, b := to.Local(from.Canon(1, 0, 0)), to.Local(from.Canon(0, 1, 0))
	return PrismMap{from: from, to: to, reflect: a[0]*b[1]-a[1]*b[0] < 0}
}

func (m PrismMap) point(p sectionrecord.Point2) sectionrecord.Point2 {
	c := m.to.Local(m.from.Canon(p.U, p.V, 0))
	return sectionrecord.Point2{U: c[0], V: c[1]}
}

// Region carries a whole region loop by loop, preserving loop order.
func (m PrismMap) Region(p momentinput.Profile) (momentinput.Profile, bool) {
	if m.from == m.to {
		return p, true
	}
	outer, ok := m.Loop(p.Outer)
	if !ok {
		return momentinput.Profile{}, false
	}
	out := momentinput.Profile{Outer: outer, Holes: make([]sectionrecord.LoopRecord, 0, len(p.Holes))}
	for _, h := range p.Holes {
		hole, ok := m.Loop(h)
		if !ok {
			return momentinput.Profile{}, false
		}
		out.Holes = append(out.Holes, hole)
	}
	return out, true
}

// Loop carries one recorded loop and reverses segment order on reflection.
func (m PrismMap) Loop(l sectionrecord.LoopRecord) (sectionrecord.LoopRecord, bool) {
	segs := make([]sectionrecord.CurveSegment, len(l.Segments))
	for i, seg := range l.Segments {
		mapped, ok := m.Segment(seg)
		if !ok {
			return sectionrecord.LoopRecord{}, false
		}
		j := i
		if m.reflect {
			j = len(segs) - 1 - i
		}
		segs[j] = mapped
	}
	return sectionrecord.LoopRecord{Segments: segs}, true
}

// Segment carries one line, arc or circle. A reflecting map rewinds natural
// lines and arcs and refuses narrowed ranges that require a displacement.
// A whole circle maps its centre without changing its recorded range.
func (m PrismMap) Segment(seg sectionrecord.CurveSegment) (sectionrecord.CurveSegment, bool) {
	natural := func(t0, t1 float64) bool { return (t0 == 0 && t1 == 1) || (t0 == 1 && t1 == 0) }
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		start, end := m.point(s.Start), m.point(s.End)
		if !m.reflect {
			return sectionrecord.LineSeg{Start: start, End: end, TStart: s.TStart, TEnd: s.TEnd}, true
		}
		if !natural(s.TStart, s.TEnd) {
			return nil, false
		}
		return sectionrecord.LineSeg{Start: end, End: start, TStart: s.TStart, TEnd: s.TEnd}, true
	case sectionrecord.ArcSeg:
		c, start, end := m.point(s.Center), m.point(s.Start), m.point(s.End)
		if !m.reflect {
			return sectionrecord.ArcSeg{Center: c, Start: start, End: end, TStart: s.TStart, TEnd: s.TEnd}, true
		}
		if !natural(s.TStart, s.TEnd) {
			return nil, false
		}
		return sectionrecord.ArcSeg{Center: c, Start: end, End: start, TStart: s.TStart, TEnd: s.TEnd}, true
	case sectionrecord.CircleSeg:
		s.Center = m.point(s.Center)
		return s, true
	default:
		return nil, false
	}
}
