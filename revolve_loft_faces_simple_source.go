package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// A simple source cell is one whole line, one exact fitted Bezier span, or
// one whole short arc in the authenticated Sketch boundary walk. Its box
// encloses the entire true curve, not a sampled polyline.
type revolveLoftFacesSimpleCell struct {
	Segment, Span int
	Start, End    freeform.RatPoint
	Box           [2]loftFaceInterval
	Bezier        freeform.BezierSpan
	Arc           *arcSeg
}

type revolveLoftFacesSimpleLoop struct {
	cells []revolveLoftFacesSimpleCell
	ccw   bool
}

// Source-wide simplicity is proved from the same live curve records the
// LoftFaces maps consume. Each cell must itself be regular; non-neighboring
// cells have strict coordinate separation, and neighboring cells meet only
// at their common exact endpoint. A unique axis extremum fixes orientation.
// This intentionally refuses geometry whose elementary bounds cannot decide.
func revolveLoftFacesSimpleSourceLoop(ctx context.Context,
	trim *loftFacesTrimRecord) (*revolveLoftFacesSimpleLoop, bool, error) {
	if ctx == nil || trim == nil || trim.source == nil ||
		!pointSourceCurrent(trim.source) ||
		trim.source.source.profile == nil ||
		!trim.source.source.profile.Valid ||
		len(trim.source.source.profile.Holes) != 0 {
		return nil, false, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	fresh, plane, _, err := recordProfile(trim.source.source.sketch,
		trim.source.source.profile)
	if err != nil {
		return nil, false, err
	}
	if !sectionrecord.IdenticalRecord(fresh, trim.source.profile) ||
		!sectionrecord.IdenticalRecord(plane, trim.source.plane) {
		return nil, false, nil
	}
	recorded := trim.source.profile.Outer.Segments
	outer := trim.source.source.profile.Outer
	entities := trim.source.source.sketch.Entities()
	if len(recorded) < 3 || len(recorded) != len(outer) ||
		len(entities) != len(recorded) {
		return nil, false, nil
	}
	// Every current Sketch entity must occur exactly once on this whole
	// boundary. Otherwise a hidden interior contour could change occupancy
	// even if the selected outer walk is exactly simple.
	for i, entity := range trim.source.source.entities {
		found := 0
		for _, current := range entities {
			if current == entity {
				found++
			}
		}
		if found != 1 {
			return nil, false, nil
		}
		for j := range i {
			if trim.source.source.entities[j] == entity {
				return nil, false, nil
			}
		}
	}
	result := &revolveLoftFacesSimpleLoop{}
	for segment, source := range recorded {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		if outer[segment].Partial || outer[segment].Entity !=
			trim.source.source.entities[segment] {
			return nil, false, nil
		}
		switch source := source.(type) {
		case lineSeg, fitSplineSeg:
			if !revolveLoftFacesSimpleRecordDirection(source, outer[segment].Reversed) {
				return nil, false, nil
			}
			spans, ok := revolveLoftFacesStraightOrFitSpans(trim, segment, source)
			if !ok || len(spans) == 0 {
				return nil, false, nil
			}
			for i := range spans {
				spanIndex := i
				if outer[segment].Reversed {
					spanIndex = len(spans) - 1 - i
				}
				span := append(freeform.BezierSpan(nil), spans[spanIndex]...)
				if outer[segment].Reversed {
					for lo, hi := 0, len(span)-1; lo < hi; lo, hi = lo+1, hi-1 {
						span[lo], span[hi] = span[hi], span[lo]
					}
				}
				cell, ok := revolveLoftFacesSimpleBezier(segment, spanIndex, span)
				if !ok {
					return nil, false, nil
				}
				result.cells = append(result.cells, cell)
			}
		case arcSeg:
			cell, ok := revolveLoftFacesSimpleArc(trim, segment,
				source, outer[segment].Reversed)
			if !ok {
				return nil, false, nil
			}
			result.cells = append(result.cells, cell)
		default:
			return nil, false, nil
		}
		if len(result.cells) > 128 {
			return nil, false, nil
		}
	}
	if len(result.cells) < 3 {
		return nil, false, nil
	}
	for i := range result.cells {
		if i%64 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
		}
		j := (i + 1) % len(result.cells)
		if !revolveLoftFacesSimpleNeighbors(result.cells[i], result.cells[j]) {
			return nil, false, nil
		}
		for k := i + 1; k < len(result.cells); k++ {
			if k == j || i == 0 && k == len(result.cells)-1 {
				continue
			}
			if !revolveLoftFacesSimpleSeparated(result.cells[i], result.cells[k]) {
				return nil, false, nil
			}
		}
	}
	result.ccw = revolveLoftFacesSimplePositiveOrientation(result.cells)
	return result, result.ccw, nil
}

func revolveLoftFacesSimpleRecordDirection(source curveSegment, reversed bool) bool {
	switch segment := source.(type) {
	case lineSeg:
		return (segment.TStart > segment.TEnd) == reversed
	case fitSplineSeg:
		return (segment.TStart > segment.TEnd) == reversed
	default:
		return false
	}
}

func revolveLoftFacesSimpleBezier(segment, spanIndex int,
	span freeform.BezierSpan) (revolveLoftFacesSimpleCell, bool) {
	var cell revolveLoftFacesSimpleCell
	if len(span) < 2 {
		return cell, false
	}
	for _, point := range span {
		if point.U == nil || point.V == nil {
			return cell, false
		}
	}
	for axis := range 2 {
		value := revolveLoftFacesSimpleCoordinate(span[0], axis)
		cell.Box[axis] = loftFaceInterval{Lo: value, Hi: value}
		for _, point := range span[1:] {
			value = revolveLoftFacesSimpleCoordinate(point, axis)
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
			last := revolveLoftFacesSimpleCoordinate(span[i-1], axis)
			current := revolveLoftFacesSimpleCoordinate(span[i], axis)
			increasing = increasing && current.Cmp(last) > 0
			decreasing = decreasing && current.Cmp(last) < 0
		}
		regular = regular || increasing || decreasing
	}
	if !regular {
		return revolveLoftFacesSimpleCell{}, false
	}
	cell.Segment, cell.Span = segment, spanIndex
	cell.Start, cell.End = span[0], span[len(span)-1]
	cell.Bezier = span
	return cell, true
}

func revolveLoftFacesSimpleCoordinate(point freeform.RatPoint, axis int) *big.Rat {
	if axis == 0 {
		return point.U
	}
	return point.V
}

func revolveLoftFacesSimpleSeparated(a, b revolveLoftFacesSimpleCell) bool {
	for axis := range 2 {
		if a.Box[axis].Hi.Cmp(b.Box[axis].Lo) < 0 ||
			b.Box[axis].Hi.Cmp(a.Box[axis].Lo) < 0 {
			return true
		}
	}
	return false
}

func revolveLoftFacesSimpleNeighbors(a, b revolveLoftFacesSimpleCell) bool {
	if a.End.U.Cmp(b.Start.U) != 0 || a.End.V.Cmp(b.Start.V) != 0 {
		return false
	}
	for axis := range 2 {
		contact := revolveLoftFacesSimpleCoordinate(a.End, axis)
		for _, side := range [...]int{-1, 1} {
			if revolveLoftFacesSimpleCellSide(a, axis, contact, side) &&
				revolveLoftFacesSimpleCellSide(b, axis, contact, -side) {
				return true
			}
		}
	}
	return false
}

func revolveLoftFacesSimpleCellSide(cell revolveLoftFacesSimpleCell,
	axis int, contact *big.Rat, side int) bool {
	if cell.Arc != nil {
		return revolveLoftFacesSimpleArcSide(*cell.Arc, axis, contact, side)
	}
	if side < 0 {
		return cell.Box[axis].Hi.Cmp(contact) == 0 &&
			cell.Box[axis].Lo.Cmp(contact) < 0
	}
	return cell.Box[axis].Lo.Cmp(contact) == 0 &&
		cell.Box[axis].Hi.Cmp(contact) > 0
}

// A short circular arc that alone passes an axis extremum determines which
// way the simple loop walks. Every other cell remains on the inner side of
// its endpoint chord, so the arc extremum is also the loop's extremum.
func revolveLoftFacesSimplePositiveOrientation(cells []revolveLoftFacesSimpleCell) bool {
	for i, cell := range cells {
		if cell.Arc == nil {
			continue
		}
		for axis := range 2 {
			contact := revolveLoftFacesSimpleCoordinate(cell.Start, axis)
			for _, side := range [...]int{-1, 1} {
				if !revolveLoftFacesSimpleArcChordSide(*cell.Arc, axis, contact, side) {
					continue
				}
				outermost := true
				for j, other := range cells {
					if j == i {
						continue
					}
					if side < 0 {
						outermost = outermost && other.Box[axis].Lo.Cmp(contact) >= 0
					} else {
						outermost = outermost && other.Box[axis].Hi.Cmp(contact) <= 0
					}
				}
				if outermost {
					return cell.Arc.TStart < cell.Arc.TEnd
				}
			}
		}
	}
	return false
}
