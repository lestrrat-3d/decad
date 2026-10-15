package decad

import (
	"context"
	"fmt"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/sketch"
)

type revolveLoftFacesJointKind uint8

const (
	revolveLoftFacesFitKnotContact revolveLoftFacesJointKind = iota + 1
	revolveLoftFacesRadialArcContact
	revolveLoftFacesWholeArcContact
	revolveLoftFacesFitLineContact
)

// Each root owns one live source endpoint. Fit-knot, radial-arc and fit-line
// records remain distinct even when their time brackets occupy one span.
type revolveLoftFacesOwnedRoot struct {
	root  loftface.EndpointRoot
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
		band < 0 || band+1 >= len(trim.Maps) ||
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

// Strict time gaps use the complete fitted source polynomial. Closed root
// brackets use the separate, already certified endpoint slivers. A gap may
// fail its ordinary source proof; this function then refuses the whole span.
func revolveLoftFacesPartitionOwnedEndpointCells(ctx context.Context,
	trim *loftFacesTrimRecord, section revolveSectionSpan,
	segment, spanIndex, band int, span freeform.BezierSpan,
	roots []revolveLoftFacesOwnedRoot) ([]revolveLoftFacesMappedCell, bool, error) {
	if ctx == nil || trim == nil || band < 0 || band+1 >= len(trim.Maps) ||
		len(roots) == 0 || len(roots) > 4 || len(span) == 0 {
		return nil, false, nil
	}
	if !revolveLoftFacesOrderOwnedRoots(roots) {
		return nil, false, nil
	}
	m, b := revolveLoftFacesMeridian(section)
	q := revolveLoftFacesMeridianMemoOf(ctx).QRows(trim.Maps[band], trim.Maps[band+1], span, m, b)
	j := loftface.MeridianJRows(trim.Maps[band], trim.Maps[band+1], span, q)
	remaining := revolveLoftFacesMaxBandCells
	previous := lfZero()
	var cells []revolveLoftFacesMappedCell
	appendGap := func(hi *big.Rat, loOpen, hiOpen bool) (bool, error) {
		if previous.Cmp(hi) >= 0 {
			return false, nil
		}
		gap, ok, err := revolveLoftFacesOpenTimeSlab(ctx, q, j,
			trim.Maps[band], trim.Maps[band+1], span, section,
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
		ok, err := appendGap(owned.root.Time.Lo, i != 0, true)
		if err != nil || !ok {
			return nil, false, err
		}
		cell := owned.cell
		makeCell := func(side int, sLo, sHi *big.Rat) revolveLoftFacesMappedCell {
			return revolveLoftFacesMappedCell{segment: segment, span: spanIndex,
				band: band, side: side, sLo: sLo, sHi: sHi,
				tLo: owned.root.Time.Lo, tHi: owned.root.Time.Hi}
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
		previous = owned.root.Time.Hi
	}
	ok, err := appendGap(lfOne(), true, false)
	if err != nil || !ok {
		return nil, false, err
	}
	return cells, true, nil
}

func revolveLoftFacesOrderOwnedRoots(roots []revolveLoftFacesOwnedRoot) bool {
	if len(roots) == 0 || len(roots) > 4 {
		return false
	}
	for _, owned := range roots {
		if owned.root.Time.Lo == nil {
			return false
		}
	}
	sort.Slice(roots, func(i, j int) bool {
		return roots[i].root.Time.Lo.Cmp(roots[j].root.Time.Lo) < 0
	})
	previous := lfZero()
	for _, owned := range roots {
		root, cell := owned.root, owned.cell
		if owned.point == nil || owned.kind != revolveLoftFacesFitKnotContact &&
			owned.kind != revolveLoftFacesRadialArcContact &&
			owned.kind != revolveLoftFacesFitLineContact ||
			root.Time.Lo == nil || root.Time.Hi == nil || cell.time.Lo == nil ||
			cell.time.Hi == nil || cell.sLo == nil || cell.sHi == nil ||
			root.Time.Lo.Cmp(previous) <= 0 ||
			root.Time.Hi.Cmp(root.Time.Lo) < 0 ||
			root.Time.Hi.Cmp(lfOne()) >= 0 ||
			cell.time.Lo.Cmp(root.Time.Lo) != 0 ||
			cell.time.Hi.Cmp(root.Time.Hi) != 0 ||
			cell.sLo.Sign() < 0 || cell.sHi.Cmp(lfOne()) > 0 ||
			cell.sLo.Cmp(cell.sHi) >= 0 ||
			cell.finite != root.Finite ||
			owned.atEnd && cell.sHi.Cmp(lfOne()) != 0 ||
			!owned.atEnd && cell.sLo.Sign() != 0 {
			return false
		}
		previous = root.Time.Hi
	}
	return true
}
