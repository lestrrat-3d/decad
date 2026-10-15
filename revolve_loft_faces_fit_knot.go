package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/sketch"
)

// A fit knot is an interior live Sketch fit point shared by two adjacent
// Bezier spans of one whole fitted entity. This record stays distinct from a
// FitSpline/Arc source-entity junction: a knot does not define an arc seam.
type revolveLoftFacesFitKnot struct {
	point       *sketch.Point
	segment     int
	leftIndex   int
	left, right freeform.BezierSpan
	coordinate  [2]*big.Rat
}

func revolveLoftFacesFitKnotVertex(trim *loftFacesTrimRecord,
	segmentIndex, leftIndex int) (revolveLoftFacesFitKnot, bool) {
	var empty revolveLoftFacesFitKnot
	if trim == nil || trim.Source == nil || trim.Source.source == nil ||
		segmentIndex < 0 || segmentIndex >= len(trim.Source.profile.Outer.Segments) ||
		segmentIndex >= len(trim.Source.source.entities) {
		return empty, false
	}
	record, ok := trim.Source.profile.Outer.Segments[segmentIndex].(fitSplineSeg)
	if !ok || !((record.TStart == 0 && record.TEnd == 1) ||
		(record.TStart == 1 && record.TEnd == 0)) {
		return empty, false
	}
	fit, ok := trim.Source.source.entities[segmentIndex].(*sketch.FitSpline)
	if !ok || len(record.Fit) < 3 || len(record.Fit) != len(fit.Fit) {
		return empty, false
	}
	spans, ok := revolveLoftFacesStraightOrFitSpans(trim, segmentIndex, record)
	if !ok || len(spans) != len(fit.Fit)-1 || leftIndex < 0 || leftIndex+1 >= len(spans) {
		return empty, false
	}
	point := fit.Fit[leftIndex+1]
	if point == nil || point.IsStale() || record.Fit[leftIndex+1].U != point.X() ||
		record.Fit[leftIndex+1].V != point.Y() {
		return empty, false
	}
	left, right := spans[leftIndex], spans[leftIndex+1]
	end, start := left[len(left)-1], right[0]
	if end.U.Cmp(start.U) != 0 || end.V.Cmp(start.V) != 0 ||
		end.U.Cmp(lfF(point.X())) != 0 || end.V.Cmp(lfF(point.Y())) != 0 {
		return empty, false
	}
	return revolveLoftFacesFitKnot{point: point, segment: segmentIndex,
		leftIndex: leftIndex, left: left, right: right,
		coordinate: [2]*big.Rat{end.U, end.V}}, true
}

// This proves a contact-time joint at the shared source knot only. Adjacent
// span side and derivative certificates must still prove how a seam walks
// through the two curves; the knot alone never admits a Boolean result.
type revolveLoftFacesFitKnotJoint struct {
	vertex     revolveLoftFacesFitKnot
	band       int
	roots      []loftface.EndpointRoot
	pieces     []loftface.TimePiece
	leftCells  []revolveLoftFacesJointCell
	rightCells []revolveLoftFacesJointCell
}

func certifyRevolveLoftFacesFitKnotJoint(ctx context.Context, trim *loftFacesTrimRecord,
	section revolveSectionSpan, segmentIndex, leftIndex, band int) (
	revolveLoftFacesFitKnotJoint, bool,
) {
	var empty revolveLoftFacesFitKnotJoint
	if ctx == nil || ctx.Err() != nil || trim == nil || band < 0 || band+1 >= len(trim.Maps) ||
		section.start.U == section.end.U || section.start.V <= 0 || section.end.V <= 0 {
		return empty, false
	}
	vertex, ok := revolveLoftFacesFitKnotVertex(trim, segmentIndex, leftIndex)
	if !ok {
		return empty, false
	}
	m, b := revolveLoftFacesMeridian(section)
	memo := revolveLoftFacesMeridianMemoOf(ctx)
	leftQ := memo.QRows(trim.Maps[band], trim.Maps[band+1], vertex.left, m, b)
	rightQ := memo.QRows(trim.Maps[band], trim.Maps[band+1], vertex.right, m, b)
	leftEnd := len(leftQ[0]) - 1
	for row := range leftQ {
		if leftQ[row][leftEnd].Cmp(rightQ[row][0]) != 0 {
			return empty, false
		}
	}
	roots, ok := revolveLoftFacesEndpointFiniteRootsWith(memo, trim, section, vertex.left, band, true)
	if !ok {
		return empty, false
	}
	q := [3]*big.Rat{leftQ[0][leftEnd], leftQ[1][leftEnd], leftQ[2][leftEnd]}
	pieces, ok := loftface.PartitionTime(q, roots)
	if !ok {
		return empty, false
	}
	leftCells := make([]revolveLoftFacesJointCell, len(roots))
	rightCells := make([]revolveLoftFacesJointCell, len(roots))
	for i, root := range roots {
		leftCells[i], ok = revolveLoftFacesKnotRootCellWith(memo, trim, section,
			vertex.left, band, true, root)
		if !ok {
			return empty, false
		}
		rightCells[i], ok = revolveLoftFacesKnotRootCellWith(memo, trim, section,
			vertex.right, band, false, root)
		if !ok {
			return empty, false
		}
	}
	return revolveLoftFacesFitKnotJoint{vertex: vertex, band: band,
		roots: roots, pieces: pieces,
		leftCells: leftCells, rightCells: rightCells}, true
}
