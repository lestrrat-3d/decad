package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

type revolveLoftFacesSimpleLoop struct {
	cells []loftface.SimpleCell
	ccw   bool
}

// Source-wide simplicity is proved from the same live curve records the
// LoftFaces maps consume. Each cell must itself be regular; non-neighboring
// cells have strict coordinate separation, and neighboring cells meet only
// at their common exact endpoint. A unique axis extremum fixes orientation.
// This intentionally refuses geometry whose elementary bounds cannot decide.
func revolveLoftFacesSimpleSourceLoop(ctx context.Context,
	trim *loftFacesTrimRecord) (*revolveLoftFacesSimpleLoop, bool, error) {
	if ctx == nil || trim == nil || trim.Source == nil ||
		!pointSourceCurrent(trim.Source) ||
		trim.Source.Source.Profile == nil ||
		!trim.Source.Source.Profile.Valid ||
		len(trim.Source.Source.Profile.Holes) != 0 {
		return nil, false, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	fresh, plane, _, err := sectioncert.RecordProfile(trim.Source.Source.Sketch,
		trim.Source.Source.Profile)
	if err != nil {
		return nil, false, err
	}
	if !sectionrecord.IdenticalRecord(fresh, trim.Source.Profile) ||
		!sectionrecord.IdenticalRecord(plane, trim.Source.Plane) {
		return nil, false, nil
	}
	recorded := trim.Source.Profile.Outer.Segments
	outer := trim.Source.Source.Profile.Outer
	entities := trim.Source.Source.Sketch.Entities()
	if len(recorded) < 3 || len(recorded) != len(outer) ||
		len(entities) != len(recorded) {
		return nil, false, nil
	}
	// Every current Sketch entity must occur exactly once on this whole
	// boundary. Otherwise a hidden interior contour could change occupancy
	// even if the selected outer walk is exactly simple.
	for i, entity := range trim.Source.Source.Entities {
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
			if trim.Source.Source.Entities[j] == entity {
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
			trim.Source.Source.Entities[segment] {
			return nil, false, nil
		}
		switch source := source.(type) {
		case lineSeg, fitSplineSeg:
			if !loftface.SimpleRecordDirection(source, outer[segment].Reversed) {
				return nil, false, nil
			}
			spans, ok := revolveloftfaces.RevolveLoftFacesStraightOrFitSpans(trim, segment, source)
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
				cell, ok := loftface.SimpleBezier(segment, spanIndex, span)
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
			if !loftface.SimpleSeparated(result.cells[i], result.cells[k]) {
				return nil, false, nil
			}
		}
	}
	result.ccw = revolveLoftFacesSimplePositiveOrientation(result.cells)
	return result, result.ccw, nil
}

func revolveLoftFacesSimpleNeighbors(a, b loftface.SimpleCell) bool {
	if a.End.U.Cmp(b.Start.U) != 0 || a.End.V.Cmp(b.Start.V) != 0 {
		return false
	}
	for axis := range 2 {
		contact := loftface.SimpleCoordinate(a.End, axis)
		for _, side := range [...]int{-1, 1} {
			if revolveLoftFacesSimpleCellSide(a, axis, contact, side) &&
				revolveLoftFacesSimpleCellSide(b, axis, contact, -side) {
				return true
			}
		}
	}
	return false
}

func revolveLoftFacesSimpleCellSide(cell loftface.SimpleCell,
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
func revolveLoftFacesSimplePositiveOrientation(cells []loftface.SimpleCell) bool {
	for i, cell := range cells {
		if cell.Arc == nil {
			continue
		}
		for axis := range 2 {
			contact := loftface.SimpleCoordinate(cell.Start, axis)
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
