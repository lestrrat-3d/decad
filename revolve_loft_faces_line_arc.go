package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/sketch"
)

// A line/arc endpoint belongs to one live Sketch point. The Arc's circular
// carrier may miss its pinned End by the already charged loft station bound.
type revolveLoftFacesLineArcVertex struct {
	point                   *sketch.Point
	lineSegment, arcSegment int
	lineSpan                freeform.BezierSpan
	lineAtEnd               bool
	coordinate              [2]*big.Rat
}

func revolveLoftFacesLineArcVertexAt(trim *loftFacesTrimRecord,
	lineIndex, arcIndex int, lineAtWalkEnd bool) (revolveLoftFacesLineArcVertex, bool) {
	var empty revolveLoftFacesLineArcVertex
	if trim == nil || trim.source == nil || trim.source.source == nil {
		return empty, false
	}
	segments, entities := trim.source.profile.Outer.Segments, trim.source.source.entities
	n := len(segments)
	if n < 2 || len(entities) != n || lineIndex < 0 || lineIndex >= n ||
		arcIndex < 0 || arcIndex >= n || lineIndex == arcIndex ||
		lineAtWalkEnd && (lineIndex+1)%n != arcIndex ||
		!lineAtWalkEnd && (arcIndex+1)%n != lineIndex {
		return empty, false
	}
	lineRecord, lineOK := segments[lineIndex].(lineSeg)
	arcRecord, arcOK := segments[arcIndex].(arcSeg)
	line, liveLine := entities[lineIndex].(*sketch.Line)
	arc, liveArc := entities[arcIndex].(*sketch.Arc)
	if !lineOK || !arcOK || !liveLine || !liveArc ||
		!((lineRecord.TStart == 0 && lineRecord.TEnd == 1) ||
			(lineRecord.TStart == 1 && lineRecord.TEnd == 0)) ||
		!((arcRecord.TStart == 0 && arcRecord.TEnd == 1) ||
			(arcRecord.TStart == 1 && arcRecord.TEnd == 0)) ||
		!mappedArcRecordMatches(arcRecord, arc) {
		return empty, false
	}
	linePoint, lineAtNaturalEnd := line.Start, lineRecord.TStart == 1
	arcPoint, arcAtNaturalEnd := arc.Start, arcRecord.TEnd == 1
	if lineAtWalkEnd {
		lineAtNaturalEnd = lineRecord.TEnd == 1
		arcAtNaturalEnd = arcRecord.TStart == 1
	}
	recordedLine, recordedArc := lineRecord.Start, arcRecord.Start
	if lineAtNaturalEnd {
		linePoint = line.End
		recordedLine = lineRecord.End
	}
	if arcAtNaturalEnd {
		arcPoint = arc.End
		recordedArc = arcRecord.End
	}
	if linePoint == nil || linePoint != arcPoint || linePoint.IsStale() ||
		recordedLine != recordedArc || recordedLine.U != linePoint.X() ||
		recordedLine.V != linePoint.Y() {
		return empty, false
	}
	spans, ok := revolveLoftFacesStraightOrFitSpans(trim, lineIndex, lineRecord)
	if !ok || len(spans) != 1 {
		return empty, false
	}
	lineAtEnd := lineAtNaturalEnd
	parameter := lfZero()
	if lineAtEnd {
		parameter = lfOne()
	}
	position := spiralBezierPoint(spans[0], parameter)
	if position[0].Cmp(lfF(recordedLine.U)) != 0 ||
		position[1].Cmp(lfF(recordedLine.V)) != 0 {
		return empty, false
	}
	return revolveLoftFacesLineArcVertex{point: linePoint,
		lineSegment: lineIndex, arcSegment: arcIndex, lineSpan: spans[0],
		lineAtEnd: lineAtEnd, coordinate: position}, true
}

type revolveLoftFacesLineArcJoint struct {
	vertex revolveLoftFacesLineArcVertex
	band   int
	roots  []revolveLoftFacesEndpointRoot
	cells  []revolveLoftFacesJointCell
}

func certifyRevolveLoftFacesLineArcJoint(ctx context.Context, trim *loftFacesTrimRecord,
	section revolveSectionSpan, lineIndex, arcIndex, band int, lineAtWalkEnd bool) (
	revolveLoftFacesLineArcJoint, bool,
) {
	var empty revolveLoftFacesLineArcJoint
	if ctx == nil || ctx.Err() != nil || trim == nil || band < 0 || band+1 >= len(trim.maps) {
		return empty, false
	}
	vertex, ok := revolveLoftFacesLineArcVertexAt(trim, lineIndex, arcIndex, lineAtWalkEnd)
	if !ok {
		return empty, false
	}
	if _, ok = mappedArcRadialTimeBand(ctx, trim, arcIndex, band, section); !ok {
		return empty, false
	}
	roots, ok := revolveLoftFacesEndpointFiniteRoots(trim, section,
		vertex.lineSpan, band, vertex.lineAtEnd)
	if !ok || len(roots) == 0 {
		return empty, false
	}
	cells := make([]revolveLoftFacesJointCell, len(roots))
	for i, root := range roots {
		cells[i], ok = revolveLoftFacesKnotRootCell(trim, section,
			vertex.lineSpan, band, vertex.lineAtEnd, root)
		if !ok {
			return empty, false
		}
	}
	return revolveLoftFacesLineArcJoint{vertex: vertex, band: band,
		roots: roots, cells: cells}, true
}
