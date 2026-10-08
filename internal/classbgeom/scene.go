package classbgeom

import (
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/sketch"
)

// CarrierSegment tags one scene segment with the surface that supplied it.
type CarrierSegment[C comparable] struct {
	Seg     sectionrecord.CurveSegment
	Carrier C
}

// ToLocal permutes X-local coordinates into the signed axes of a face frame.
func ToLocal(x [3]float64, axis [3]int, sign [3]float64) [3]float64 {
	var out [3]float64
	for i := range out {
		out[i] = sign[i]*x[axis[i]] + 0
	}
	return out
}

// ToX permutes face-local coordinates back into X's axes.
func ToX(l [3]float64, axis [3]int, sign [3]float64) [3]float64 {
	var out [3]float64
	for i, c := range l {
		out[axis[i]] = sign[i]*c + 0
	}
	return out
}

// RectLoop builds a counter-clockwise loop through X-local corners. Each
// resulting edge retains the carrier of the corresponding input edge.
func RectLoop[C comparable](axis [3]int, sign [3]float64, corners [][3]float64, carriers []C) []CarrierSegment[C] {
	pts := make([]sectionrecord.Point2, len(corners))
	for i, c := range corners {
		l := ToLocal(c, axis, sign)
		pts[i] = sectionrecord.Point2{U: l[0], V: l[1]}
	}
	area := 0.0
	for i := range pts {
		j := (i + 1) % len(pts)
		area += pts[i].U*pts[j].V - pts[j].U*pts[i].V
	}
	edgeCarriers := slices.Clone(carriers)
	if area < 0 {
		slices.Reverse(pts)
		// Edge i ran from corner i to i+1; reversed, edge k runs from
		// corner n−1−k to n−2−k, which is old edge n−2−k.
		n := len(pts)
		for k := range n {
			edgeCarriers[k] = carriers[(2*n-2-k)%n]
		}
	}
	segs := make([]CarrierSegment[C], len(pts))
	for i := range pts {
		segs[i] = CarrierSegment[C]{Seg: sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}, Carrier: edgeCarriers[i]}
	}
	return segs
}

// CreateNaturalSegment adds a natural-range segment to a sketch scene.
func CreateNaturalSegment(s *sketch.Sketch, point func(sectionrecord.Point2) *sketch.Point,
	seg sectionrecord.CurveSegment) (sketch.Entity, error) {
	w, err := boundarywalk.WalkOf(seg, nil)
	if err != nil {
		return nil, err
	}
	switch {
	case w.IsLine():
		return s.CreateLine(point(sectionrecord.Point2{U: w.StartU, V: w.StartV}), point(sectionrecord.Point2{U: w.EndU, V: w.EndV})), nil
	case w.Closed:
		return s.CreateCircle(point(sectionrecord.Point2{U: w.CU, V: w.CV}), w.Radius), nil
	default:
		lo, hi := sectionrecord.Point2{U: w.StartU, V: w.StartV}, sectionrecord.Point2{U: w.EndU, V: w.EndV}
		if w.Th1 < w.Th0 {
			lo, hi = hi, lo
		}
		return s.CreateArc(point(sectionrecord.Point2{U: w.CU, V: w.CV}), point(lo), point(hi)), nil
	}
}

// CarriersOf identifies the input carrier of every segment in each result
// loop from the defining geometry that the sketch scene records verbatim.
func CarriersOf[C comparable](loops []sectionrecord.LoopRecord, inputs []CarrierSegment[C]) ([][]C, bool) {
	var out [][]C
	for _, loop := range loops {
		var row []C
		for _, seg := range loop.Segments {
			found := false
			for _, in := range inputs {
				if sameCarrier(seg, in.Seg) {
					row = append(row, in.Carrier)
					found = true
					break
				}
			}
			if !found {
				return nil, false
			}
		}
		out = append(out, row)
	}
	return out, true
}

func sameCarrier(a, b sectionrecord.CurveSegment) bool {
	switch x := a.(type) {
	case sectionrecord.LineSeg:
		y, ok := b.(sectionrecord.LineSeg)
		return ok && ((x.Start == y.Start && x.End == y.End) || (x.Start == y.End && x.End == y.Start))
	case sectionrecord.CircleSeg:
		y, ok := b.(sectionrecord.CircleSeg)
		return ok && x.Center == y.Center && x.Radius == y.Radius
	case sectionrecord.ArcSeg:
		y, ok := b.(sectionrecord.ArcSeg)
		return ok && x.Center == y.Center && ((x.Start == y.Start && x.End == y.End) || (x.Start == y.End && x.End == y.Start))
	}
	return false
}

// BetweenVertices rewrites a region edge between its canonical vertices.
func BetweenVertices(w survey2d.SegmentWalk, axis [3]int, sign [3]float64,
	from, to [3]float64) sectionrecord.CurveSegment {
	pf, pt := ToLocal(from, axis, sign), ToLocal(to, axis, sign)
	start, end := sectionrecord.Point2{U: pf[0], V: pf[1]}, sectionrecord.Point2{U: pt[0], V: pt[1]}
	if w.IsLine() {
		return sectionrecord.LineSeg{Start: start, End: end, TStart: 0, TEnd: 1}
	}
	center := sectionrecord.Point2{U: w.CU, V: w.CV}
	if w.Th1 > w.Th0 {
		return sectionrecord.ArcSeg{Center: center, Start: start, End: end, TStart: 0, TEnd: 1}
	}
	return sectionrecord.ArcSeg{Center: center, Start: end, End: start, TStart: 1, TEnd: 0}
}

// ReframeSegment restates a segment in another signed axis map with the same
// normal axis. The return is false for a segment kind outside this reach.
func ReframeSegment(seg sectionrecord.CurveSegment, fromAxis, toAxis [3]int,
	fromSign, toSign [3]float64) (sectionrecord.CurveSegment, bool) {
	mapPt := func(p sectionrecord.Point2) sectionrecord.Point2 {
		l := ToLocal(ToX([3]float64{p.U, p.V, 0}, fromAxis, fromSign), toAxis, toSign)
		return sectionrecord.Point2{U: l[0], V: l[1]}
	}
	reflected := false
	{
		// A map that swaps handedness in the plane reverses every sense.
		e1 := ToLocal(ToX([3]float64{1, 0, 0}, fromAxis, fromSign), toAxis, toSign)
		e2 := ToLocal(ToX([3]float64{0, 1, 0}, fromAxis, fromSign), toAxis, toSign)
		reflected = e1[0]*e2[1]-e1[1]*e2[0] < 0
	}
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		s.Start, s.End = mapPt(s.Start), mapPt(s.End)
		return s, true
	case sectionrecord.ArcSeg:
		s.Center, s.Start, s.End = mapPt(s.Center), mapPt(s.Start), mapPt(s.End)
		if reflected {
			s.Start, s.End = s.End, s.Start
		}
		return s, true
	case sectionrecord.CircleSeg:
		s.Center = mapPt(s.Center)
		if reflected {
			s.CCW = !s.CCW
			s.TStart, s.TEnd = s.TEnd, s.TStart
		}
		return s, true
	}
	return nil, false
}

// JoinCircleFragments joins consecutive pieces of one circle over both
// parameter ranges, reading a whole circle's range as one turn.
func JoinCircleFragments(a, b sectionrecord.CurveSegment) sectionrecord.CurveSegment {
	ca, okA := a.(sectionrecord.CircleSeg)
	cb, okB := b.(sectionrecord.CircleSeg)
	if !okA || !okB {
		return a
	}
	out := ca
	switch {
	case ca.TEnd == 1 && cb.TStart == 0:
		out.TEnd = 1 + cb.TEnd
	case ca.TEnd == 0 && cb.TStart == 1:
		out.TEnd = cb.TEnd - 1
	default:
		out.TEnd = cb.TEnd
	}
	return out
}
