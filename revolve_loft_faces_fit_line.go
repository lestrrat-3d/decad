package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/sketch"
)

// A fit/line joint binds both whole Sketch entities to the same live point.
// The two source spans must give that point identical exact meridian values.
type revolveLoftFacesFitLineVertex struct {
	point                   *sketch.Point
	fitSegment, lineSegment int
	fitSpan, lineSpan       freeform.BezierSpan
	fitAtEnd, lineAtEnd     bool
	coordinate              [2]*big.Rat
}

func revolveLoftFacesFitLineVertexAt(trim *loftFacesTrimRecord,
	fitIndex, lineIndex int, fitAtWalkEnd bool) (revolveLoftFacesFitLineVertex, bool) {
	var empty revolveLoftFacesFitLineVertex
	if trim == nil || trim.Source == nil || trim.Source.Source == nil {
		return empty, false
	}
	segments := trim.Source.Profile.Outer.Segments
	entities := trim.Source.Source.Entities
	n := len(segments)
	if n < 2 || len(entities) != n || fitIndex < 0 || fitIndex >= n ||
		lineIndex < 0 || lineIndex >= n || fitIndex == lineIndex ||
		fitAtWalkEnd && (fitIndex+1)%n != lineIndex ||
		!fitAtWalkEnd && (lineIndex+1)%n != fitIndex {
		return empty, false
	}
	fitRecord, fitOK := segments[fitIndex].(fitSplineSeg)
	lineRecord, lineOK := segments[lineIndex].(lineSeg)
	fitEntity, liveFit := entities[fitIndex].(*sketch.FitSpline)
	lineEntity, liveLine := entities[lineIndex].(*sketch.Line)
	if !fitOK || !lineOK || !liveFit || !liveLine ||
		len(fitRecord.Fit) < 2 || len(fitEntity.Fit) != len(fitRecord.Fit) ||
		!((fitRecord.TStart == 0 && fitRecord.TEnd == 1) ||
			(fitRecord.TStart == 1 && fitRecord.TEnd == 0)) ||
		!((lineRecord.TStart == 0 && lineRecord.TEnd == 1) ||
			(lineRecord.TStart == 1 && lineRecord.TEnd == 0)) {
		return empty, false
	}
	fitAtHigh := fitRecord.TStart == 1
	if fitAtWalkEnd {
		fitAtHigh = fitRecord.TEnd == 1
	}
	fitPointIndex := 0
	if fitAtHigh {
		fitPointIndex = len(fitEntity.Fit) - 1
	}
	fitPoint := fitEntity.Fit[fitPointIndex]
	recordedFit := fitRecord.Fit[fitPointIndex]
	lineAtEnd := !fitAtWalkEnd
	lineAtNaturalEnd := lineRecord.TStart == 1
	linePoint, recordedLine := lineEntity.End, lineRecord.Start
	if lineAtEnd {
		lineAtNaturalEnd = lineRecord.TEnd == 1
		recordedLine = lineRecord.End
	}
	if !lineAtNaturalEnd {
		linePoint = lineEntity.Start
	}
	if fitPoint == nil || fitPoint != linePoint || fitPoint.IsStale() ||
		recordedFit != recordedLine || recordedFit.U != fitPoint.X() ||
		recordedFit.V != fitPoint.Y() {
		return empty, false
	}
	fitSpans, ok := revolveLoftFacesStraightOrFitSpans(trim, fitIndex, fitRecord)
	if !ok || len(fitSpans) == 0 {
		return empty, false
	}
	lineSpans, ok := revolveLoftFacesStraightOrFitSpans(trim, lineIndex, lineRecord)
	if !ok || len(lineSpans) != 1 {
		return empty, false
	}
	fitSpan := fitSpans[0]
	if fitAtHigh {
		fitSpan = fitSpans[len(fitSpans)-1]
	}
	fitParameter, lineParameter := lfZero(), lfZero()
	if fitAtHigh {
		fitParameter = lfOne()
	}
	if lineAtEnd {
		lineParameter = lfOne()
	}
	fitLocation := loftface.BezierPoint(fitSpan, fitParameter)
	lineLocation := loftface.BezierPoint(lineSpans[0], lineParameter)
	if fitLocation[0].Cmp(lfF(recordedFit.U)) != 0 ||
		fitLocation[1].Cmp(lfF(recordedFit.V)) != 0 ||
		fitLocation[0].Cmp(lineLocation[0]) != 0 ||
		fitLocation[1].Cmp(lineLocation[1]) != 0 {
		return empty, false
	}
	return revolveLoftFacesFitLineVertex{point: fitPoint,
		fitSegment: fitIndex, lineSegment: lineIndex,
		fitSpan: fitSpan, lineSpan: lineSpans[0],
		fitAtEnd: fitAtHigh, lineAtEnd: lineAtEnd,
		coordinate: [2]*big.Rat{fitLocation[0], fitLocation[1]}}, true
}

type revolveLoftFacesFitLineJoint struct {
	vertex    revolveLoftFacesFitLineVertex
	band      int
	roots     []loftface.EndpointRoot
	fitCells  []revolveLoftFacesJointCell
	lineCells []revolveLoftFacesJointCell
}

func certifyRevolveLoftFacesFitLineJoint(ctx context.Context, trim *loftFacesTrimRecord,
	section revolveSectionSpan, fitIndex, lineIndex, band int, fitAtWalkEnd bool) (
	revolveLoftFacesFitLineJoint, bool,
) {
	var empty revolveLoftFacesFitLineJoint
	if ctx == nil || ctx.Err() != nil || trim == nil || band < 0 || band+1 >= len(trim.Maps) {
		return empty, false
	}
	vertex, ok := revolveLoftFacesFitLineVertexAt(trim, fitIndex, lineIndex, fitAtWalkEnd)
	if !ok {
		return empty, false
	}
	m, b := revolveLoftFacesMeridian(section)
	memo := revolveLoftFacesMeridianMemoOf(ctx)
	fitQ := memo.QRows(trim.Maps[band], trim.Maps[band+1], vertex.fitSpan, m, b)
	lineQ := memo.QRows(trim.Maps[band], trim.Maps[band+1], vertex.lineSpan, m, b)
	fitEndpoint, lineEndpoint := 0, 0
	if vertex.fitAtEnd {
		fitEndpoint = len(fitQ[0]) - 1
	}
	if vertex.lineAtEnd {
		lineEndpoint = len(lineQ[0]) - 1
	}
	for row := range fitQ {
		if fitQ[row][fitEndpoint].Cmp(lineQ[row][lineEndpoint]) != 0 {
			return empty, false
		}
	}
	roots, ok := revolveLoftFacesEndpointFiniteRootsWith(memo, trim, section,
		vertex.fitSpan, band, vertex.fitAtEnd)
	if !ok || len(roots) == 0 {
		return empty, false
	}
	lineRoots, lineOK := revolveLoftFacesEndpointFiniteRootsWith(memo, trim, section,
		vertex.lineSpan, band, vertex.lineAtEnd)
	if !lineOK || len(lineRoots) != len(roots) {
		return empty, false
	}
	for i, root := range roots {
		if lineRoots[i].Direction != root.Direction ||
			lineRoots[i].Time.Lo.Cmp(root.Time.Lo) != 0 ||
			lineRoots[i].Time.Hi.Cmp(root.Time.Hi) != 0 ||
			lineRoots[i].Finite != root.Finite ||
			lineRoots[i].X.Lo.Cmp(root.X.Lo) != 0 ||
			lineRoots[i].X.Hi.Cmp(root.X.Hi) != 0 {
			return empty, false
		}
	}
	fitCells := make([]revolveLoftFacesJointCell, len(roots))
	lineCells := make([]revolveLoftFacesJointCell, len(roots))
	for i, root := range roots {
		fitCells[i], ok = revolveLoftFacesKnotRootCellWith(memo, trim, section,
			vertex.fitSpan, band, vertex.fitAtEnd, root)
		if !ok {
			return empty, false
		}
		lineCells[i], ok = revolveLoftFacesKnotRootCellWith(memo, trim, section,
			vertex.lineSpan, band, vertex.lineAtEnd, root)
		if !ok {
			return empty, false
		}
	}
	return revolveLoftFacesFitLineJoint{vertex: vertex, band: band,
		roots: roots, fitCells: fitCells, lineCells: lineCells}, true
}

// Every owned time root is paired with a one-sided source cell. This route
// also handles a fit knot or radial arc at the opposite end of the same span.
func revolveLoftFacesFitLinePartitionCells(ctx context.Context,
	trim *loftFacesTrimRecord, section revolveSectionSpan,
	segment, spanIndex, band int, span freeform.BezierSpan,
	knot *revolveLoftFacesFitKnotJoint, knotLeft bool,
	radial *revolveLoftFacesRadialJoint,
	fitLines []*revolveLoftFacesFitLineJoint) ([]revolveLoftFacesMappedCell, bool, error) {
	if len(fitLines) == 0 || len(span) == 0 {
		return nil, false, nil
	}
	var roots []revolveLoftFacesOwnedRoot
	if knot != nil {
		if knot.band != band || knot.vertex.segment != segment ||
			knotLeft && spanIndex != knot.vertex.leftIndex ||
			!knotLeft && spanIndex != knot.vertex.leftIndex+1 {
			return nil, false, nil
		}
		cells := knot.rightCells
		if knotLeft {
			cells = knot.leftCells
		}
		if len(knot.roots) != len(cells) {
			return nil, false, nil
		}
		for i, root := range knot.roots {
			roots = append(roots, revolveLoftFacesOwnedRoot{root: root, cell: cells[i],
				kind: revolveLoftFacesFitKnotContact, point: knot.vertex.point, atEnd: knotLeft})
		}
	}
	if radial != nil {
		if radial.radial.band != band || radial.vertex.fitSegment != segment ||
			len(radial.roots) != len(radial.fitCells) {
			return nil, false, nil
		}
		for i, root := range radial.roots {
			roots = append(roots, revolveLoftFacesOwnedRoot{root: root,
				cell: radial.fitCells[i], kind: revolveLoftFacesRadialArcContact,
				point: radial.vertex.point, atEnd: radial.vertex.fitAtEnd})
		}
	}
	for _, joint := range fitLines {
		if joint == nil || joint.band != band ||
			len(joint.roots) == 0 || len(joint.roots) != len(joint.fitCells) ||
			len(joint.roots) != len(joint.lineCells) {
			return nil, false, nil
		}
		cells, atEnd := joint.fitCells, joint.vertex.fitAtEnd
		if joint.vertex.fitSegment == segment {
			if !revolveLoftFacesSameBezier(span, joint.vertex.fitSpan) {
				return nil, false, nil
			}
		} else if joint.vertex.lineSegment == segment && spanIndex == 0 {
			cells, atEnd = joint.lineCells, joint.vertex.lineAtEnd
			if !revolveLoftFacesSameBezier(span, joint.vertex.lineSpan) {
				return nil, false, nil
			}
		} else {
			return nil, false, nil
		}
		for i, root := range joint.roots {
			roots = append(roots, revolveLoftFacesOwnedRoot{root: root,
				cell: cells[i], kind: revolveLoftFacesFitLineContact,
				point: joint.vertex.point, atEnd: atEnd})
		}
	}
	return revolveLoftFacesPartitionOwnedEndpointCells(ctx, trim, section,
		segment, spanIndex, band, span, roots)
}
