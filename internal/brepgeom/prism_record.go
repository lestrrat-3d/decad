package brepgeom

import (
	"reflect"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// PrismRect is a BRep face rectangle projected onto a prism section plane.
// Its outward normal is one signed coordinate axis in that plane.
type PrismRect struct {
	p, q   sectionrecord.Point2
	nu, nv float64
}

// PrismRectFace contains the record fields used to recognize a rectangle
// across or along a proposed prism axis.
type PrismRectFace struct {
	Region           *momentinput.Profile
	Wall             sectionrecord.CurveSegment
	Z0, Z1           float64
	Z0Delta, Z1Delta float64
	Split0, Split1   bool
	Outward          bool
}

// PlanarPrismRect reads a planar face as four natural-range axis-aligned
// lines spanning the proposed prism's two levels.
func PlanarPrismRect(f PrismRectFace, e, section Embed, axis int, zlo, zhi float64) (PrismRect, bool) {
	if f.Region == nil || f.Z0Delta != 0 || len(f.Region.Holes) != 0 || len(f.Region.Outer.Segments) != 4 {
		return PrismRect{}, false
	}
	corners := make([][3]float64, 0, 4)
	segs := f.Region.Outer.Segments
	for i, seg := range segs {
		from, to, ok := NaturalLine(seg)
		if !ok || (from.U == to.U) == (from.V == to.V) {
			return PrismRect{}, false
		}
		next, _, ok := NaturalLine(segs[(i+1)%len(segs)])
		if !ok || next != to {
			return PrismRect{}, false
		}
		corners = append(corners, e.Canon(from.U, from.V, f.Z0))
	}
	spanAxis := 3 - axis - e.Axis[2]
	lo, hi, ok := prismRectSpan(corners, axis, spanAxis, zlo, zhi)
	if !ok {
		return PrismRect{}, false
	}
	level := e.Sign[2]*f.Z0 + 0
	var normal [3]float64
	normal[e.Axis[2]] = e.Sign[2]
	if !f.Outward {
		normal[e.Axis[2]] = -normal[e.Axis[2]]
	}
	return prismRectIn(section, e.Axis[2], spanAxis, level, level, lo, hi, normal), true
}

// prismRectSpan requires all four distinct combinations of two axis levels
// and two section coordinates.
func prismRectSpan(corners [][3]float64, axis, spanAxis int, zlo, zhi float64) (float64, float64, bool) {
	lo, hi := corners[0][spanAxis], corners[0][spanAxis]
	for _, c := range corners {
		lo, hi = min(lo, c[spanAxis]), max(hi, c[spanAxis])
	}
	if !(lo < hi) {
		return 0, 0, false
	}
	seen := map[[2]float64]struct{}{}
	for _, c := range corners {
		if (c[axis] != zlo && c[axis] != zhi) || (c[spanAxis] != lo && c[spanAxis] != hi) {
			return 0, 0, false
		}
		seen[[2]float64{c[axis], c[spanAxis]}] = struct{}{}
	}
	return lo, hi, len(seen) == 4
}

// SweptPrismRect reads an unsplit straight wall across the proposed axis as
// the same rectangle that PlanarPrismRect reads from a planar face.
func SweptPrismRect(f PrismRectFace, e, section Embed, axis int, zlo, zhi float64) (PrismRect, bool) {
	if f.Z0Delta != 0 || f.Z1Delta != 0 || f.Split0 || f.Split1 {
		return PrismRect{}, false
	}
	from, to, ok := NaturalLine(f.Wall)
	if !ok {
		return PrismRect{}, false
	}
	du, dv := to.U-from.U, to.V-from.V
	if (du == 0) == (dv == 0) {
		return PrismRect{}, false
	}
	a, b := e.Canon(from.U, from.V, 0), e.Canon(to.U, to.V, 0)
	if !((a[axis] == zlo && b[axis] == zhi) || (a[axis] == zhi && b[axis] == zlo)) {
		return PrismRect{}, false
	}
	spanAxis := 3 - axis - e.Axis[2]
	if a[spanAxis] != b[spanAxis] {
		return PrismRect{}, false
	}
	normal := e.Canon(prismSign(dv), prismSign(-du), 0)
	return prismRectIn(section, spanAxis, e.Axis[2], a[spanAxis], a[spanAxis],
		e.Sign[2]*f.Z0+0, e.Sign[2]*f.Z1+0, normal), true
}

func prismRectIn(section Embed, fixedAxis, spanAxis int, fixed0, fixed1, s0, s1 float64,
	normal [3]float64) PrismRect {
	var p, q [3]float64
	p[fixedAxis], p[spanAxis] = fixed0, s0
	q[fixedAxis], q[spanAxis] = fixed1, s1
	lp, lq, ln := section.Local(p), section.Local(q), section.Local(normal)
	return PrismRect{
		p:  sectionrecord.Point2{U: lp[0], V: lp[1]},
		q:  sectionrecord.Point2{U: lq[0], V: lq[1]},
		nu: ln[0], nv: ln[1],
	}
}

func prismSign(x float64) float64 {
	switch {
	case x > 0:
		return 1
	case x < 0:
		return -1
	default:
		return 0
	}
}

// NaturalLine reads a LineSeg over its natural range as its walked ends.
func NaturalLine(seg sectionrecord.CurveSegment) (sectionrecord.Point2, sectionrecord.Point2, bool) {
	l, ok := seg.(sectionrecord.LineSeg)
	switch {
	case !ok:
		return sectionrecord.Point2{}, sectionrecord.Point2{}, false
	case l.TStart == 0 && l.TEnd == 1:
		return l.Start, l.End, true
	case l.TStart == 1 && l.TEnd == 0:
		return l.End, l.Start, true
	default:
		return sectionrecord.Point2{}, sectionrecord.Point2{}, false
	}
}

// PrismWallsClaim checks that every wall and rectangle claims one distinct
// section segment with the same directed walk and outward normal.
func PrismWallsClaim(walls []survey2d.SegmentWalk, rects []PrismRect, section momentinput.Profile,
	work *freeform.FreeformWork) bool {
	var walks []survey2d.SegmentWalk
	for _, loop := range append([]sectionrecord.LoopRecord{section.Outer}, section.Holes...) {
		for _, seg := range loop.Segments {
			walk, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return false
			}
			walks = append(walks, walk)
		}
	}
	if len(walls)+len(rects) != len(walks) {
		return false
	}
	claimed := make([]bool, len(walks))
	take := func(match func(survey2d.SegmentWalk) bool) bool {
		for i, walk := range walks {
			if !claimed[i] && match(walk) {
				claimed[i] = true
				return true
			}
		}
		return false
	}
	for _, wall := range walls {
		key := WalkKeyOf(wall)
		if !take(func(s survey2d.SegmentWalk) bool { return WalkKeyOf(s) == key }) {
			return false
		}
	}
	for _, rect := range rects {
		if !take(rect.matches) {
			return false
		}
	}
	return true
}

func (r PrismRect) matches(s survey2d.SegmentWalk) bool {
	if s.IsCircular() || s.Closed {
		return false
	}
	from, to := sectionrecord.Point2{U: s.StartU, V: s.StartV}, sectionrecord.Point2{U: s.EndU, V: s.EndV}
	sameEnds := (from == r.p && to == r.q) || (from == r.q && to == r.p)
	if !sameEnds {
		return false
	}
	du, dv := to.U-from.U, to.V-from.V
	return prismSign(dv) == r.nu && prismSign(-du) == r.nv
}

// WalkKey is a walk's directed geometry, including circular sense.
type WalkKey struct {
	circular, closed bool
	su, sv, eu, ev   float64
	cu, cv, r        float64
	ccw              bool
}

// WalkKeyOf constructs a comparable key for a directed analytic walk.
func WalkKeyOf(w survey2d.SegmentWalk) WalkKey {
	k := WalkKey{circular: w.IsCircular(), closed: w.Closed}
	if k.circular {
		k.cu, k.cv, k.r, k.ccw = w.CU, w.CV, w.Radius, w.Th1 > w.Th0
	}
	if !k.closed {
		k.su, k.sv, k.eu, k.ev = w.StartU, w.StartV, w.EndU, w.EndV
	}
	return k
}

// SameRegion compares a mapped bottom region with a section and returns the
// section loop index of every matching bottom loop.
func SameRegion(section, bottom momentinput.Profile) ([]int, bool) {
	if len(section.Holes) != len(bottom.Holes) || !LoopsEqual(section.Outer, bottom.Outer) {
		return nil, false
	}
	loops := make([]int, 1+len(bottom.Holes))
	used := make([]bool, len(section.Holes))
	for bi, hole := range bottom.Holes {
		found := false
		for si, want := range section.Holes {
			if !used[si] && LoopsEqual(want, hole) {
				used[si], loops[1+bi], found = true, 1+si, true
				break
			}
		}
		if !found {
			return nil, false
		}
	}
	return loops, true
}

// LoopsEqual compares two loop segment records up to a cyclic start shift.
func LoopsEqual(a, b sectionrecord.LoopRecord) bool {
	n := len(a.Segments)
	if n != len(b.Segments) || n == 0 {
		return false
	}
	for shift := range n {
		same := true
		for i := range n {
			if !reflect.DeepEqual(a.Segments[i], b.Segments[(i+shift)%n]) {
				same = false
				break
			}
		}
		if same {
			return true
		}
	}
	return false
}
