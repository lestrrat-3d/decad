package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// A line/fit joint is one live Sketch point shared by adjacent whole entities.
// The source walk may reverse either entity, so source and natural ends differ.
type revolveLoftFacesLineFitVertex struct {
	point                   *sketch.Point
	lineSegment, fitSegment int
	lineSpan, fitSpan       freeform.BezierSpan
	lineAtEnd, fitAtEnd     bool
}

func revolveLoftFacesLineFitVertexOf(trim *loftFacesTrimRecord,
	lineIndex, fitIndex int, fitAtWalkEnd bool) (revolveLoftFacesLineFitVertex, bool) {
	var empty revolveLoftFacesLineFitVertex
	if trim == nil || trim.source == nil || trim.source.source == nil {
		return empty, false
	}
	if !pointSourceCurrent(trim.source) {
		return empty, false
	}
	fresh, plane, _, err := recordProfile(trim.source.source.sketch,
		trim.source.source.profile)
	if err != nil || !sectionrecord.IdenticalRecord(fresh, trim.source.profile) ||
		!sectionrecord.IdenticalRecord(plane, trim.source.plane) {
		return empty, false
	}
	segments := trim.source.profile.Outer.Segments
	entities := trim.source.source.entities
	n := len(segments)
	if n < 2 || len(entities) != n || lineIndex < 0 || lineIndex >= n ||
		fitIndex < 0 || fitIndex >= n ||
		fitAtWalkEnd && (fitIndex+1)%n != lineIndex ||
		!fitAtWalkEnd && (lineIndex+1)%n != fitIndex {
		return empty, false
	}
	lineRecord, lineOK := segments[lineIndex].(lineSeg)
	fitRecord, fitOK := segments[fitIndex].(fitSplineSeg)
	line, liveLine := entities[lineIndex].(*sketch.Line)
	fit, liveFit := entities[fitIndex].(*sketch.FitSpline)
	if !lineOK || !fitOK || !liveLine || !liveFit || line.IsStale() || fit.IsStale() ||
		len(fitRecord.Fit) < 2 || len(fitRecord.Fit) != len(fit.Fit) ||
		!((lineRecord.TStart == 0 && lineRecord.TEnd == 1) ||
			(lineRecord.TStart == 1 && lineRecord.TEnd == 0)) ||
		!((fitRecord.TStart == 0 && fitRecord.TEnd == 1) ||
			(fitRecord.TStart == 1 && fitRecord.TEnd == 0)) {
		return empty, false
	}
	if lineRecord.Start.U != line.Start.X() || lineRecord.Start.V != line.Start.Y() ||
		lineRecord.End.U != line.End.X() || lineRecord.End.V != line.End.Y() {
		return empty, false
	}
	fitAtEnd := fitRecord.TStart == 1
	lineAtEnd := lineRecord.TEnd == 1
	if fitAtWalkEnd {
		fitAtEnd = fitRecord.TEnd == 1
		lineAtEnd = lineRecord.TStart == 1
	}
	fitPoint := fit.Fit[0]
	fitRecorded := fitRecord.Fit[0]
	if fitAtEnd {
		fitPoint = fit.Fit[len(fit.Fit)-1]
		fitRecorded = fitRecord.Fit[len(fitRecord.Fit)-1]
	}
	linePoint := line.Start
	if lineAtEnd {
		linePoint = line.End
	}
	if fitPoint == nil || fitPoint != linePoint || fitPoint.IsStale() ||
		fitRecorded.U != fitPoint.X() || fitRecorded.V != fitPoint.Y() {
		return empty, false
	}
	lineSpans, lineOK := revolveLoftFacesStraightOrFitSpans(trim, lineIndex, lineRecord)
	fitSpans, fitOK := revolveLoftFacesStraightOrFitSpans(trim, fitIndex, fitRecord)
	if !lineOK || !fitOK || len(lineSpans) != 1 || len(fitSpans) == 0 {
		return empty, false
	}
	lineSpan, fitSpan := lineSpans[0], fitSpans[0]
	if fitAtEnd {
		fitSpan = fitSpans[len(fitSpans)-1]
	}
	lineEnd := lfZero()
	if lineAtEnd {
		lineEnd = lfOne()
	}
	fitEnd := lfZero()
	if fitAtEnd {
		fitEnd = lfOne()
	}
	a, b := spiralBezierPoint(lineSpan, lineEnd), spiralBezierPoint(fitSpan, fitEnd)
	if a[0].Cmp(b[0]) != 0 || a[1].Cmp(b[1]) != 0 ||
		a[0].Cmp(lfF(fitPoint.X())) != 0 || a[1].Cmp(lfF(fitPoint.Y())) != 0 {
		return empty, false
	}
	return revolveLoftFacesLineFitVertex{point: fitPoint,
		lineSegment: lineIndex, fitSegment: fitIndex,
		lineSpan: lineSpan, fitSpan: fitSpan,
		lineAtEnd: lineAtEnd, fitAtEnd: fitAtEnd}, true
}

type revolveLoftFacesLineFitJoint struct {
	vertex    revolveLoftFacesLineFitVertex
	band      int
	roots     []revolveLoftFacesEndpointRoot
	pieces    []revolveLoftFacesTimePiece
	lineCells []revolveLoftFacesJointCell
	fitCells  []revolveLoftFacesJointCell
}

func certifyRevolveLoftFacesLineFitJoint(ctx context.Context, trim *loftFacesTrimRecord,
	section revolveSectionSpan, lineIndex, fitIndex, band int,
	fitAtWalkEnd bool) (revolveLoftFacesLineFitJoint, bool) {
	var empty revolveLoftFacesLineFitJoint
	if ctx == nil || ctx.Err() != nil || trim == nil || band < 0 || band+1 >= len(trim.maps) ||
		section.start.U == section.end.U || section.start.V <= 0 || section.end.V <= 0 {
		return empty, false
	}
	vertex, ok := revolveLoftFacesLineFitVertexOf(trim, lineIndex, fitIndex, fitAtWalkEnd)
	if !ok {
		return empty, false
	}
	m, b := revolveLoftFacesMeridian(section)
	lineQ := spiralRootPolynomial(trim.maps[band], trim.maps[band+1], vertex.lineSpan, m, b)
	fitQ := spiralRootPolynomial(trim.maps[band], trim.maps[band+1], vertex.fitSpan, m, b)
	lineEnd, fitEnd := 0, 0
	if vertex.lineAtEnd {
		lineEnd = len(lineQ[0]) - 1
	}
	if vertex.fitAtEnd {
		fitEnd = len(fitQ[0]) - 1
	}
	for row := range lineQ {
		if lineQ[row][lineEnd].Cmp(fitQ[row][fitEnd]) != 0 {
			return empty, false
		}
	}
	roots, ok := revolveLoftFacesEndpointFiniteRoots(trim, section,
		vertex.fitSpan, band, vertex.fitAtEnd)
	if !ok || len(roots) == 0 {
		return empty, false
	}
	q := [3]*big.Rat{fitQ[0][fitEnd], fitQ[1][fitEnd], fitQ[2][fitEnd]}
	pieces, ok := revolveLoftFacesPartitionTime(q, roots)
	if !ok {
		return empty, false
	}
	lineCells := make([]revolveLoftFacesJointCell, len(roots))
	fitCells := make([]revolveLoftFacesJointCell, len(roots))
	for index, root := range roots {
		lineCells[index], ok = revolveLoftFacesKnotRootCell(trim, section,
			vertex.lineSpan, band, vertex.lineAtEnd, root)
		if !ok {
			return empty, false
		}
		fitCells[index], ok = revolveLoftFacesKnotRootCell(trim, section,
			vertex.fitSpan, band, vertex.fitAtEnd, root)
		if !ok {
			return empty, false
		}
	}
	return revolveLoftFacesLineFitJoint{vertex: vertex, band: band,
		roots: roots, pieces: pieces, lineCells: lineCells, fitCells: fitCells}, true
}

func revolveLoftFacesLineFitPartitionCells(ctx context.Context,
	trim *loftFacesTrimRecord, section revolveSectionSpan,
	segment, spanIndex, band int, span freeform.BezierSpan,
	joint revolveLoftFacesLineFitJoint) ([]revolveLoftFacesMappedCell, bool, error) {
	if joint.band != band || segment != joint.vertex.lineSegment && segment != joint.vertex.fitSegment {
		return nil, false, nil
	}
	selected, atEnd := joint.vertex.lineSpan, joint.vertex.lineAtEnd
	jointCells := joint.lineCells
	if segment == joint.vertex.fitSegment {
		selected, atEnd = joint.vertex.fitSpan, joint.vertex.fitAtEnd
		jointCells = joint.fitCells
	}
	spans, ok := revolveLoftFacesStraightOrFitSpans(trim, segment,
		trim.source.profile.Outer.Segments[segment])
	if !ok || spanIndex < 0 || spanIndex >= len(spans) || len(span) != len(selected) ||
		len(spans[spanIndex]) != len(selected) {
		return nil, false, nil
	}
	for index := range span {
		if span[index].U.Cmp(selected[index].U) != 0 ||
			span[index].V.Cmp(selected[index].V) != 0 ||
			spans[spanIndex][index].U.Cmp(selected[index].U) != 0 ||
			spans[spanIndex][index].V.Cmp(selected[index].V) != 0 {
			return nil, false, nil
		}
	}
	cells, ok, err := revolveLoftFacesEndpointPartitionCells(ctx, trim, section,
		segment, spanIndex, band, span, joint.roots, joint.pieces, jointCells, atEnd)
	for index := range cells {
		if cells[index].side == revolveLoftFacesJointContact {
			cells[index].jointKind = revolveLoftFacesLineFitContact
			cells[index].jointPoint = joint.vertex.point
		}
	}
	return cells, ok, err
}
