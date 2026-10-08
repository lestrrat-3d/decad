package classbgeom

import (
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// SplitSegment cuts a face edge at every canonical vertex on its carrier.
// A line uses a vertex on both the face and edge carriers. A circular edge
// uses vertices on its cylinder at the face's level.
func SplitSegment[C comparable, V interface{ Carriers() []C }](w survey2d.SegmentWalk,
	seg sectionrecord.CurveSegment, frameAxis [3]int, frameSign [3]float64, level float64,
	faceCarrier, carrier C, cylinder bool, carrierAxis int, verts map[[3]float64]V,
	miss error) ([]sectionrecord.CurveSegment, error) {
	type cut struct {
		at  [3]float64
		key float64
	}
	var cuts []cut
	if w.IsLine() {
		start := ToX([3]float64{w.StartU, w.StartV, level}, frameAxis, frameSign)
		end := ToX([3]float64{w.EndU, w.EndV, level}, frameAxis, frameSign)
		free := -1
		for k := range 3 {
			if start[k] != end[k] {
				free = k
			}
		}
		if free < 0 {
			return nil, miss
		}
		lo, hi := math.Min(start[free], end[free]), math.Max(start[free], end[free])
		for p, v := range verts {
			if !slices.Contains(v.Carriers(), faceCarrier) || !slices.Contains(v.Carriers(), carrier) {
				continue
			}
			if p[free] > lo && p[free] < hi {
				cuts = append(cuts, cut{at: p, key: math.Abs(p[free] - start[free])})
			}
		}
		slices.SortFunc(cuts, func(a, b cut) int { return cmpFloat(a.key, b.key) })
		points := [][3]float64{start}
		for _, k := range cuts {
			points = append(points, k.at)
		}
		points = append(points, end)
		var out []sectionrecord.CurveSegment
		for i := 0; i+1 < len(points); i++ {
			if points[i] == points[i+1] {
				return nil, miss
			}
			pf, pt := ToLocal(points[i], frameAxis, frameSign), ToLocal(points[i+1], frameAxis, frameSign)
			out = append(out, sectionrecord.LineSeg{
				Start: sectionrecord.Point2{U: pf[0], V: pf[1]}, End: sectionrecord.Point2{U: pt[0], V: pt[1]},
				TStart: 0, TEnd: 1,
			})
		}
		return out, nil
	}
	if !cylinder || frameAxis[2] != carrierAxis {
		return []sectionrecord.CurveSegment{seg}, nil
	}
	// An arc on a cylinder, in a face across the cylinder's axis: cut at the
	// angle of every vertex on the cylinder.
	ccw := w.Th1 > w.Th0
	angle := func(p [3]float64) float64 {
		l := ToLocal(p, frameAxis, frameSign)
		return math.Atan2(l[1]-w.CV, l[0]-w.CU)
	}
	sweep := func(from, to float64) float64 {
		d := to - from
		if !ccw {
			d = -d
		}
		for d < 0 {
			d += 2 * math.Pi
		}
		for d >= 2*math.Pi {
			d -= 2 * math.Pi
		}
		return d
	}
	startPt := ToX([3]float64{w.StartU, w.StartV, level}, frameAxis, frameSign)
	endPt := ToX([3]float64{w.EndU, w.EndV, level}, frameAxis, frameSign)
	a0 := angle(startPt)
	total := sweep(a0, angle(endPt))
	if w.Closed {
		total = 2 * math.Pi
	}
	// Two vertices this close in angle are not ordered by their held
	// points, so the pair misses rather than guess their order.
	const tol = 1e-9
	for p, v := range verts {
		if !slices.Contains(v.Carriers(), carrier) {
			continue
		}
		q := p
		q[carrierAxis] = ToX([3]float64{0, 0, level}, frameAxis, frameSign)[carrierAxis]
		if q == startPt || q == endPt {
			continue
		}
		s := sweep(a0, angle(q))
		if !w.Closed {
			if s >= total+tol {
				continue // outside the arc's span
			}
			if s <= tol || s >= total-tol {
				return nil, miss // too close to an end to order
			}
		}
		cuts = append(cuts, cut{at: q, key: s})
	}
	slices.SortFunc(cuts, func(a, b cut) int { return cmpFloat(a.key, b.key) })
	// Vertices at other levels project to one point: keep it once.
	cuts = slices.CompactFunc(cuts, func(a, b cut) bool { return a.at == b.at })
	for i := 1; i < len(cuts); i++ {
		if cuts[i].key-cuts[i-1].key <= tol {
			return nil, miss
		}
	}
	if len(cuts) == 0 {
		return []sectionrecord.CurveSegment{seg}, nil
	}
	points := [][3]float64{startPt}
	for _, k := range cuts {
		points = append(points, k.at)
	}
	if w.Closed {
		// A whole circle cut at its vertices starts and ends at the first.
		points = points[1:]
		points = append(points, points[0])
	} else {
		points = append(points, endPt)
	}
	var out []sectionrecord.CurveSegment
	for i := 0; i+1 < len(points); i++ {
		pf, pt := ToLocal(points[i], frameAxis, frameSign), ToLocal(points[i+1], frameAxis, frameSign)
		center := sectionrecord.Point2{U: w.CU, V: w.CV}
		start := sectionrecord.Point2{U: pf[0], V: pf[1]}
		end := sectionrecord.Point2{U: pt[0], V: pt[1]}
		if ccw {
			out = append(out, sectionrecord.ArcSeg{Center: center, Start: start, End: end, TStart: 0, TEnd: 1})
		} else {
			out = append(out, sectionrecord.ArcSeg{Center: center, Start: end, End: start, TStart: 1, TEnd: 0})
		}
	}
	return out, nil
}

func cmpFloat(a, b float64) int {
	switch {
	case a < b:
		return -1
	case a > b:
		return 1
	}
	return 0
}
