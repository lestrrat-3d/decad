package loftface

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// A simple source cell is one whole line, one exact fitted Bezier span, or
// one whole short arc in the authenticated Sketch boundary walk. Its box
// encloses the entire true curve, not a sampled polyline.
type SimpleCell struct {
	Segment, Span int
	Start, End    freeform.RatPoint
	Box           [2]Interval
	Bezier        freeform.BezierSpan
	Arc           *sectionrecord.ArcSeg
}

// SimpleRecordDirection reports whether a line or fitted spline record has
// increasing parameters and reversed is false, or decreasing parameters and
// reversed is true. Any other record reports false.
func SimpleRecordDirection(source sectionrecord.CurveSegment, reversed bool) bool {
	switch segment := source.(type) {
	case sectionrecord.LineSeg:
		return (segment.TStart > segment.TEnd) == reversed
	case sectionrecord.FitSplineSeg:
		return (segment.TStart > segment.TEnd) == reversed
	default:
		return false
	}
}

// SimpleBezier builds the simple cell of one exact Bezier span.
func SimpleBezier(segment, spanIndex int,
	span freeform.BezierSpan) (SimpleCell, bool) {
	var cell SimpleCell
	if len(span) < 2 {
		return cell, false
	}
	for _, point := range span {
		if point.U == nil || point.V == nil {
			return cell, false
		}
	}
	for axis := range 2 {
		value := SimpleCoordinate(span[0], axis)
		cell.Box[axis] = Interval{Lo: value, Hi: value}
		for _, point := range span[1:] {
			value = SimpleCoordinate(point, axis)
			if value.Cmp(cell.Box[axis].Lo) < 0 {
				cell.Box[axis].Lo = value
			}
			if value.Cmp(cell.Box[axis].Hi) > 0 {
				cell.Box[axis].Hi = value
			}
		}
	}
	// Strict control monotonicity makes the curve injective, even where a
	// different coordinate bends within its control hull.
	regular := false
	for axis := range 2 {
		increasing, decreasing := true, true
		for i := 1; i < len(span); i++ {
			last := SimpleCoordinate(span[i-1], axis)
			current := SimpleCoordinate(span[i], axis)
			increasing = increasing && current.Cmp(last) > 0
			decreasing = decreasing && current.Cmp(last) < 0
		}
		regular = regular || increasing || decreasing
	}
	if !regular {
		return SimpleCell{}, false
	}
	cell.Segment, cell.Span = segment, spanIndex
	cell.Start, cell.End = span[0], span[len(span)-1]
	cell.Bezier = span
	return cell, true
}

// SimpleCoordinate returns one axis of a rational source point.
func SimpleCoordinate(point freeform.RatPoint, axis int) *big.Rat {
	if axis == 0 {
		return point.U
	}
	return point.V
}

// SimpleSeparated reports whether two simple cells are provably separated.
func SimpleSeparated(a, b SimpleCell) bool {
	for axis := range 2 {
		if a.Box[axis].Hi.Cmp(b.Box[axis].Lo) < 0 ||
			b.Box[axis].Hi.Cmp(a.Box[axis].Lo) < 0 {
			return true
		}
	}
	return false
}
