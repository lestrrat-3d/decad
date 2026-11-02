package sectionrecord

import "math"

// IdenticalAnalyticSegment compares every recorded field of a supported line,
// circle or arc, including the sign bit of zero. Unknown variants refuse.
func IdenticalAnalyticSegment(a, b CurveSegment) bool {
	sameFloat := func(x, y float64) bool { return math.Float64bits(x) == math.Float64bits(y) }
	samePoint := func(x, y Point2) bool { return sameFloat(x.U, y.U) && sameFloat(x.V, y.V) }
	switch x := a.(type) {
	case LineSeg:
		y, ok := b.(LineSeg)
		return ok && samePoint(x.Start, y.Start) && samePoint(x.End, y.End) &&
			sameFloat(x.TStart, y.TStart) && sameFloat(x.TEnd, y.TEnd)
	case CircleSeg:
		y, ok := b.(CircleSeg)
		return ok && samePoint(x.Center, y.Center) &&
			x.Radius.Unit() == y.Radius.Unit() && sameFloat(x.Radius.Mag(), y.Radius.Mag()) &&
			x.CCW == y.CCW && sameFloat(x.TStart, y.TStart) && sameFloat(x.TEnd, y.TEnd)
	case ArcSeg:
		y, ok := b.(ArcSeg)
		return ok && samePoint(x.Center, y.Center) && samePoint(x.Start, y.Start) &&
			samePoint(x.End, y.End) && sameFloat(x.TStart, y.TStart) && sameFloat(x.TEnd, y.TEnd)
	default:
		return false
	}
}
