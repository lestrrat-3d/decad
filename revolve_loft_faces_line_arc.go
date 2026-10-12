package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// A line/arc joint uses one live Sketch point, not coincident rounded endpoints.
// Its arc must independently prove that the whole carrier is radial in the
// mapped band. Only the adjacent line gets a one-sided endpoint cell.
type revolveLoftFacesLineArcVertex struct {
	point                   *sketch.Point
	arc                     *sketch.Arc
	lineSegment, arcSegment int
	lineSpan                freeform.BezierSpan
	lineAtEnd               bool
	coordinate              [2]*big.Rat
}

func revolveLoftFacesLineArcVertexOf(trim *loftFacesTrimRecord,
	lineIndex, arcIndex int, lineAtWalkEnd bool) (revolveLoftFacesLineArcVertex, bool) {
	var empty revolveLoftFacesLineArcVertex
	if trim == nil || trim.source == nil || trim.source.source == nil ||
		!pointSourceCurrent(trim.source) {
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
		arcIndex < 0 || arcIndex >= n ||
		lineAtWalkEnd && (lineIndex+1)%n != arcIndex ||
		!lineAtWalkEnd && (arcIndex+1)%n != lineIndex {
		return empty, false
	}
	lineRecord, lineOK := segments[lineIndex].(lineSeg)
	arcRecord, arcOK := segments[arcIndex].(arcSeg)
	line, liveLine := entities[lineIndex].(*sketch.Line)
	arc, liveArc := entities[arcIndex].(*sketch.Arc)
	if !lineOK || !arcOK || !liveLine || !liveArc || line.IsStale() ||
		!mappedArcRecordMatches(arcRecord, arc) ||
		!((lineRecord.TStart == 0 && lineRecord.TEnd == 1) ||
			(lineRecord.TStart == 1 && lineRecord.TEnd == 0)) ||
		!((arcRecord.TStart == 0 && arcRecord.TEnd == 1) ||
			(arcRecord.TStart == 1 && arcRecord.TEnd == 0)) ||
		lineRecord.Start.U != line.Start.X() || lineRecord.Start.V != line.Start.Y() ||
		lineRecord.End.U != line.End.X() || lineRecord.End.V != line.End.Y() {
		return empty, false
	}
	lineAtEnd := lineRecord.TEnd == 1
	arcAtStart := arcRecord.TStart == 0
	if !lineAtWalkEnd {
		lineAtEnd = lineRecord.TStart == 1
		arcAtStart = arcRecord.TEnd == 0
	}
	point, recordedLine := line.Start, lineRecord.Start
	if lineAtEnd {
		point, recordedLine = line.End, lineRecord.End
	}
	arcPoint, recordedArc := arc.End, arcRecord.End
	if arcAtStart {
		arcPoint, recordedArc = arc.Start, arcRecord.Start
	}
	if point == nil || point != arcPoint || point.IsStale() ||
		recordedLine != recordedArc ||
		recordedLine.U != point.X() || recordedLine.V != point.Y() {
		return empty, false
	}
	spans, ok := revolveLoftFacesStraightOrFitSpans(trim, lineIndex, lineRecord)
	if !ok || len(spans) != 1 {
		return empty, false
	}
	end := lfZero()
	if lineAtEnd {
		end = lfOne()
	}
	coordinate := spiralBezierPoint(spans[0], end)
	if coordinate[0].Cmp(lfF(point.X())) != 0 ||
		coordinate[1].Cmp(lfF(point.Y())) != 0 {
		return empty, false
	}
	return revolveLoftFacesLineArcVertex{
		point: point, arc: arc, lineSegment: lineIndex, arcSegment: arcIndex,
		lineSpan: spans[0], lineAtEnd: lineAtEnd,
		coordinate: [2]*big.Rat{coordinate[0], coordinate[1]},
	}, true
}

type revolveLoftFacesLineArcJoint struct {
	vertex    revolveLoftFacesLineArcVertex
	radial    mappedArcRadialBand
	roots     []revolveLoftFacesEndpointRoot
	pieces    []revolveLoftFacesTimePiece
	lineCells []revolveLoftFacesJointCell
}

func certifyRevolveLoftFacesLineArcJoint(ctx context.Context, trim *loftFacesTrimRecord,
	section revolveSectionSpan, lineIndex, arcIndex, band int,
	lineAtWalkEnd bool) (revolveLoftFacesLineArcJoint, bool) {
	var empty revolveLoftFacesLineArcJoint
	if ctx == nil || ctx.Err() != nil || trim == nil || band < 0 || band+1 >= len(trim.maps) ||
		section.start.U == section.end.U || section.start.V <= 0 || section.end.V <= 0 {
		return empty, false
	}
	vertex, ok := revolveLoftFacesLineArcVertexOf(trim, lineIndex, arcIndex, lineAtWalkEnd)
	if !ok {
		return empty, false
	}
	radial, ok := mappedArcRadialTimeBand(ctx, trim, arcIndex, band, section)
	if !ok || radial.arc != vertex.arc {
		return empty, false
	}
	m, b := revolveLoftFacesMeridian(section)
	lineQ := spiralRootPolynomial(trim.maps[band], trim.maps[band+1], vertex.lineSpan, m, b)
	index := 0
	if vertex.lineAtEnd {
		index = len(lineQ[0]) - 1
	}
	g0 := trim.maps[band].denominator(vertex.coordinate[0], vertex.coordinate[1])
	g1 := trim.maps[band+1].denominator(vertex.coordinate[0], vertex.coordinate[1])
	if g0.Sign() <= 0 || g1.Sign() <= 0 {
		return empty, false
	}
	d := lfMul(g0, g1)
	d2 := lfMul(d, d)
	for row := range lineQ {
		if lfDiv(lineQ[row][index], d2).Cmp(radial.q[row]) != 0 {
			return empty, false
		}
	}
	for capIndex, mapped := range [...]loftFaceMap{trim.maps[band], trim.maps[band+1]} {
		if mapped.at(vertex.coordinate[0], vertex.coordinate[1])[0].Cmp(radial.x[capIndex]) != 0 {
			return empty, false
		}
	}
	roots, ok := revolveLoftFacesEndpointFiniteRoots(trim, section,
		vertex.lineSpan, band, vertex.lineAtEnd)
	if !ok || len(roots) == 0 {
		return empty, false
	}
	arcQ := [3][]*big.Rat{{radial.q[0]}, {radial.q[1]}, {radial.q[2]}}
	arcRoots, ok := revolveLoftFacesEndpointTimeRoots(arcQ, false)
	if !ok || len(roots) != len(arcRoots) {
		return empty, false
	}
	for i := range roots {
		arcRoots[i], ok = revolveLoftFacesEndpointAxialClass(arcRoots[i],
			radial.x[0], radial.x[1], section)
		if !ok || roots[i].time.lo.Cmp(arcRoots[i].time.lo) != 0 ||
			roots[i].time.hi.Cmp(arcRoots[i].time.hi) != 0 ||
			roots[i].finite != arcRoots[i].finite {
			return empty, false
		}
	}
	pieces, ok := revolveLoftFacesPartitionTime(radial.q, roots)
	if !ok {
		return empty, false
	}
	lineCells := make([]revolveLoftFacesJointCell, len(roots))
	for i, root := range roots {
		lineCells[i], ok = revolveLoftFacesKnotRootCell(trim, section,
			vertex.lineSpan, band, vertex.lineAtEnd, root)
		if !ok {
			return empty, false
		}
	}
	return revolveLoftFacesLineArcJoint{vertex: vertex, radial: radial,
		roots: roots, pieces: pieces, lineCells: lineCells}, true
}

func revolveLoftFacesLineArcPartitionCells(ctx context.Context,
	trim *loftFacesTrimRecord, section revolveSectionSpan,
	segment, spanIndex, band int, span freeform.BezierSpan,
	joint revolveLoftFacesLineArcJoint) ([]revolveLoftFacesMappedCell, bool, error) {
	if trim == nil || trim.source == nil ||
		joint.vertex.lineSegment != segment || joint.radial.band != band || spanIndex != 0 {
		return nil, false, nil
	}
	segments := trim.source.profile.Outer.Segments
	if segment < 0 || segment >= len(segments) ||
		joint.vertex.arcSegment < 0 || joint.vertex.arcSegment >= len(segments) {
		return nil, false, nil
	}
	lineAtWalkEnd := (segment+1)%len(segments) == joint.vertex.arcSegment
	current, ok := certifyRevolveLoftFacesLineArcJoint(ctx, trim, section,
		segment, joint.vertex.arcSegment, band, lineAtWalkEnd)
	if !ok || current.vertex.point != joint.vertex.point ||
		current.vertex.arc != joint.vertex.arc ||
		current.vertex.lineAtEnd != joint.vertex.lineAtEnd {
		return nil, false, nil
	}
	spans, ok := revolveLoftFacesStraightOrFitSpans(trim, segment,
		segments[segment])
	if !ok || len(spans) != 1 || !revolveLoftFacesSameSpan(span, current.vertex.lineSpan) ||
		!revolveLoftFacesSameSpan(spans[0], current.vertex.lineSpan) {
		return nil, false, nil
	}
	cells, certified, err := revolveLoftFacesEndpointPartitionCells(ctx, trim, section,
		segment, spanIndex, band, span, current.roots, current.pieces,
		current.lineCells, current.vertex.lineAtEnd)
	for i := range cells {
		if cells[i].side == revolveLoftFacesJointContact {
			cells[i].jointKind = revolveLoftFacesLineArcContact
			cells[i].jointPoint = current.vertex.point
			cells[i].jointArc = current.vertex.arc
		}
	}
	return cells, certified, err
}

func revolveLoftFacesSameSpan(a, b freeform.BezierSpan) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i].U.Cmp(b[i].U) != 0 || a[i].V.Cmp(b[i].V) != 0 {
			return false
		}
	}
	return true
}
