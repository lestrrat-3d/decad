package decad

import (
	"context"
	"fmt"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/sketch"
)

type revolveLoftFacesJointKind uint8

const (
	revolveLoftFacesFitKnotContact revolveLoftFacesJointKind = iota + 1
	revolveLoftFacesRadialArcContact
	revolveLoftFacesWholeArcContact
	revolveLoftFacesLineFitContact
)

// Each root owns one live source endpoint. The knot and radial-arc records
// remain distinct even when their time brackets occupy one fitted span.
type revolveLoftFacesOwnedRoot struct {
	root  revolveLoftFacesEndpointRoot
	cell  revolveLoftFacesJointCell
	kind  revolveLoftFacesJointKind
	point *sketch.Point
	atEnd bool
}

func revolveLoftFacesMergeEndpointCells(ctx context.Context,
	trim *loftFacesTrimRecord, section revolveSectionSpan,
	segment, spanIndex, band int, span freeform.BezierSpan,
	knot *revolveLoftFacesFitKnotJoint, radial *revolveLoftFacesRadialJoint,
	leftSpan bool) ([]revolveLoftFacesMappedCell, bool, error) {
	if ctx == nil || trim == nil || knot == nil || radial == nil ||
		band < 0 || band+1 >= len(trim.maps) ||
		knot.vertex.segment != segment || knot.band != band ||
		radial.vertex.fitSegment != segment || radial.radial.band != band ||
		knot.vertex.point == nil || radial.vertex.point == nil ||
		knot.vertex.point == radial.vertex.point ||
		len(knot.roots) == 0 || len(knot.roots) != len(knot.leftCells) ||
		len(knot.roots) != len(knot.rightCells) ||
		len(radial.roots) == 0 || len(radial.roots) != len(radial.fitCells) ||
		leftSpan && spanIndex != knot.vertex.leftIndex ||
		!leftSpan && spanIndex != knot.vertex.leftIndex+1 ||
		leftSpan == radial.vertex.fitAtEnd {
		return nil, false, nil
	}
	knotCells := knot.rightCells
	if leftSpan {
		knotCells = knot.leftCells
	}
	roots := make([]revolveLoftFacesOwnedRoot, 0, len(knot.roots)+len(radial.roots))
	for i, root := range knot.roots {
		roots = append(roots, revolveLoftFacesOwnedRoot{
			root: root, cell: knotCells[i], kind: revolveLoftFacesFitKnotContact,
			point: knot.vertex.point, atEnd: leftSpan})
	}
	for i, root := range radial.roots {
		roots = append(roots, revolveLoftFacesOwnedRoot{
			root: root, cell: radial.fitCells[i], kind: revolveLoftFacesRadialArcContact,
			point: radial.vertex.point, atEnd: radial.vertex.fitAtEnd})
	}
	return revolveLoftFacesPartitionOwnedEndpointCells(ctx, trim, section,
		segment, spanIndex, band, span, roots)
}

// A fitted span may meet a line at one source end and a fitted knot at the
// other. Recompute both joint proofs from the live Sketch source, then order
// their exact time brackets before classifying the intervening open gaps.
func revolveLoftFacesMergeLineFitKnotCells(ctx context.Context,
	trim *loftFacesTrimRecord, section revolveSectionSpan,
	segment, spanIndex, band int, span freeform.BezierSpan,
	lineFit *revolveLoftFacesLineFitJoint, knot *revolveLoftFacesFitKnotJoint,
	knotLeft bool) ([]revolveLoftFacesMappedCell, bool, error) {
	if ctx == nil || trim == nil || trim.source == nil || lineFit == nil || knot == nil ||
		band < 0 || band+1 >= len(trim.maps) ||
		lineFit.band != band || knot.band != band ||
		lineFit.vertex.fitSegment != segment || knot.vertex.segment != segment ||
		lineFit.vertex.point == nil || knot.vertex.point == nil ||
		lineFit.vertex.point == knot.vertex.point ||
		lineFit.vertex.fitAtEnd == knotLeft ||
		knotLeft && spanIndex != knot.vertex.leftIndex ||
		!knotLeft && spanIndex != knot.vertex.leftIndex+1 {
		return nil, false, nil
	}
	segments := trim.source.profile.Outer.Segments
	lineIndex := lineFit.vertex.lineSegment
	if lineIndex < 0 || lineIndex >= len(segments) ||
		segment < 0 || segment >= len(segments) {
		return nil, false, nil
	}
	fitAtWalkEnd := (segment+1)%len(segments) == lineIndex
	lineCurrent, ok := certifyRevolveLoftFacesLineFitJoint(ctx, trim, section,
		lineIndex, segment, band, fitAtWalkEnd)
	if !ok || lineCurrent.vertex.point != lineFit.vertex.point ||
		lineCurrent.vertex.fitAtEnd != lineFit.vertex.fitAtEnd {
		return nil, false, nil
	}
	knotCurrent, ok := certifyRevolveLoftFacesFitKnotJoint(ctx, trim, section,
		segment, knot.vertex.leftIndex, band)
	if !ok || knotCurrent.vertex.point != knot.vertex.point {
		return nil, false, nil
	}
	spans, ok := revolveLoftFacesStraightOrFitSpans(trim, segment, segments[segment])
	if !ok || spanIndex < 0 || spanIndex >= len(spans) ||
		len(span) != len(spans[spanIndex]) ||
		len(lineCurrent.vertex.fitSpan) != len(span) {
		return nil, false, nil
	}
	for index := range span {
		if span[index].U.Cmp(spans[spanIndex][index].U) != 0 ||
			span[index].V.Cmp(spans[spanIndex][index].V) != 0 ||
			span[index].U.Cmp(lineCurrent.vertex.fitSpan[index].U) != 0 ||
			span[index].V.Cmp(lineCurrent.vertex.fitSpan[index].V) != 0 {
			return nil, false, nil
		}
	}
	knotCells := knotCurrent.rightCells
	if knotLeft {
		knotCells = knotCurrent.leftCells
	}
	roots := make([]revolveLoftFacesOwnedRoot, 0,
		len(lineCurrent.roots)+len(knotCurrent.roots))
	for index, root := range lineCurrent.roots {
		roots = append(roots, revolveLoftFacesOwnedRoot{
			root: root, cell: lineCurrent.fitCells[index], kind: revolveLoftFacesLineFitContact,
			point: lineCurrent.vertex.point, atEnd: lineCurrent.vertex.fitAtEnd})
	}
	for index, root := range knotCurrent.roots {
		roots = append(roots, revolveLoftFacesOwnedRoot{
			root: root, cell: knotCells[index], kind: revolveLoftFacesFitKnotContact,
			point: knotCurrent.vertex.point, atEnd: knotLeft})
	}
	return revolveLoftFacesPartitionOwnedEndpointCells(ctx, trim, section,
		segment, spanIndex, band, span, roots)
}

// Strict time gaps use the complete fitted source polynomial. Closed root
// brackets use the separate, already certified endpoint slivers. A gap may
// fail its ordinary source proof; this function then refuses the whole span.
func revolveLoftFacesPartitionOwnedEndpointCells(ctx context.Context,
	trim *loftFacesTrimRecord, section revolveSectionSpan,
	segment, spanIndex, band int, span freeform.BezierSpan,
	roots []revolveLoftFacesOwnedRoot) ([]revolveLoftFacesMappedCell, bool, error) {
	if ctx == nil || trim == nil || band < 0 || band+1 >= len(trim.maps) ||
		len(roots) < 2 || len(roots) > 4 || len(span) == 0 {
		return nil, false, nil
	}
	if !revolveLoftFacesOrderOwnedRoots(roots) {
		return nil, false, nil
	}
	m, b := revolveLoftFacesMeridian(section)
	q := spiralRootPolynomial(trim.maps[band], trim.maps[band+1], span, m, b)
	j := spiralRootXOrder(trim.maps[band], trim.maps[band+1], span, q)
	remaining := revolveLoftFacesMaxBandCells
	previous := lfZero()
	var cells []revolveLoftFacesMappedCell
	appendGap := func(hi *big.Rat, loOpen, hiOpen bool) (bool, error) {
		if previous.Cmp(hi) >= 0 {
			return false, nil
		}
		gap, ok, err := revolveLoftFacesOpenTimeSlab(ctx, q, j,
			trim.maps[band], trim.maps[band+1], span, section,
			segment, spanIndex, band, previous, hi,
			loOpen, hiOpen, &remaining)
		if err != nil {
			return false, fmt.Errorf("mapped fitted span %d/%d band %d: %w",
				segment, spanIndex, band, err)
		}
		if !ok {
			return false, nil
		}
		cells = append(cells, gap...)
		return true, nil
	}
	for i, owned := range roots {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		ok, err := appendGap(owned.root.time.lo, i != 0, true)
		if err != nil || !ok {
			return nil, false, err
		}
		cell := owned.cell
		makeCell := func(side int, sLo, sHi *big.Rat) revolveLoftFacesMappedCell {
			return revolveLoftFacesMappedCell{segment: segment, span: spanIndex,
				band: band, side: side, sLo: sLo, sHi: sHi,
				tLo: owned.root.time.lo, tHi: owned.root.time.hi}
		}
		if owned.atEnd && cell.sLo.Sign() > 0 {
			cells = append(cells, makeCell(cell.outsideSide, lfZero(), cell.sLo))
		}
		if !owned.atEnd && cell.sHi.Cmp(lfOne()) < 0 {
			cells = append(cells, makeCell(cell.outsideSide, cell.sHi, lfOne()))
		}
		side := revolveLoftFacesJointContact
		if !cell.finite {
			side = revolveLoftFacesAxialMiss
		}
		contact := makeCell(side, cell.sLo, cell.sHi)
		contact.jointKind, contact.jointPoint = owned.kind, owned.point
		cells = append(cells, contact)
		previous = owned.root.time.hi
	}
	ok, err := appendGap(lfOne(), true, false)
	if err != nil || !ok {
		return nil, false, err
	}
	return cells, true, nil
}

func revolveLoftFacesOrderOwnedRoots(roots []revolveLoftFacesOwnedRoot) bool {
	if len(roots) < 2 || len(roots) > 4 {
		return false
	}
	for _, owned := range roots {
		if owned.root.time.lo == nil {
			return false
		}
	}
	sort.Slice(roots, func(i, j int) bool {
		return roots[i].root.time.lo.Cmp(roots[j].root.time.lo) < 0
	})
	previous := lfZero()
	for _, owned := range roots {
		root, cell := owned.root, owned.cell
		if owned.point == nil || owned.kind != revolveLoftFacesFitKnotContact &&
			owned.kind != revolveLoftFacesRadialArcContact &&
			owned.kind != revolveLoftFacesLineFitContact ||
			root.time.lo == nil || root.time.hi == nil || cell.time.lo == nil ||
			cell.time.hi == nil || cell.sLo == nil || cell.sHi == nil ||
			root.time.lo.Cmp(previous) <= 0 ||
			root.time.hi.Cmp(root.time.lo) < 0 ||
			root.time.hi.Cmp(lfOne()) >= 0 ||
			cell.time.lo.Cmp(root.time.lo) != 0 ||
			cell.time.hi.Cmp(root.time.hi) != 0 ||
			cell.sLo.Sign() < 0 || cell.sHi.Cmp(lfOne()) > 0 ||
			cell.sLo.Cmp(cell.sHi) >= 0 ||
			cell.finite != root.finite ||
			owned.atEnd && cell.sHi.Cmp(lfOne()) != 0 ||
			!owned.atEnd && cell.sLo.Sign() != 0 {
			return false
		}
		previous = root.time.hi
	}
	return true
}
