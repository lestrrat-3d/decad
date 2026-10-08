package brepgeom

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Map2 is the plane-local map between two faces whose normals lie on one
// reference axis (docs/brep-modify-design.md §5.4): a point of the first face
// is lifted into reference coordinates (Embed.Canon) and read back in the
// second face's frame (Embed.Local). Both steps are signed permutations, so
// the map moves every float coordinate exactly.
type Map2 struct {
	from, to Embed
}

// NewMap2 returns the map from one face's frame into another's. It reports
// false when the two normals lie on different reference axes, where no
// plane-local map exists.
func NewMap2(from, to Embed) (Map2, bool) {
	if from.Axis[2] != to.Axis[2] {
		return Map2{}, false
	}
	return Map2{from: from, to: to}, true
}

// Point maps one plane-local point.
func (m Map2) Point(u, v float64) (float64, float64) {
	l := m.to.Local(m.from.Canon(u, v, 0))
	return l[0], l[1]
}

// Reflects reports whether the map reverses the plane's orientation: the
// determinant of its 2×2 signed-permutation matrix is −1.
func (m Map2) Reflects() bool {
	au, av := m.Point(1, 0)
	bu, bv := m.Point(0, 1)
	return au*bv-av*bu < 0
}

// MapSegment applies a map to one recorded segment: a LineSeg maps its two
// endpoints; an ArcSeg maps its centre and endpoints; a CircleSeg maps its
// centre. Under a reflecting map a counter-clockwise arc becomes a clockwise
// one, so an ArcSeg swaps Start and End and reads its range as 1−t, and a
// CircleSeg flips CCW and reads its range as 1−t; on a natural range (0→1 or
// 1→0) that is the swap general-boolean A4 re-winds a loop by. A whole
// circle's seam moves with the map's rotation, which no pairing reads (a
// closed edge pairs by centre, axis and radius). Any other segment kind is
// an error naming it.
func MapSegment(m Map2, seg sectionrecord.CurveSegment) (sectionrecord.CurveSegment, error) {
	pt := func(p sectionrecord.Point2) sectionrecord.Point2 {
		u, v := m.Point(p.U, p.V)
		return sectionrecord.Point2{U: u, V: v}
	}
	reflects := m.Reflects()
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		return sectionrecord.LineSeg{Start: pt(s.Start), End: pt(s.End), TStart: s.TStart, TEnd: s.TEnd}, nil
	case sectionrecord.ArcSeg:
		out := sectionrecord.ArcSeg{Center: pt(s.Center), Start: pt(s.Start), End: pt(s.End), TStart: s.TStart, TEnd: s.TEnd}
		if reflects {
			out.Start, out.End = out.End, out.Start
			out.TStart, out.TEnd = 1-s.TStart, 1-s.TEnd
		}
		return out, nil
	case sectionrecord.CircleSeg:
		out := s
		out.Center = pt(s.Center)
		if reflects {
			out.CCW = !s.CCW
			out.TStart, out.TEnd = 1-s.TStart, 1-s.TEnd
		}
		return out, nil
	default:
		return nil, fmt.Errorf(`a brep face map moves lines, arcs and circles only, not %T`, seg)
	}
}
