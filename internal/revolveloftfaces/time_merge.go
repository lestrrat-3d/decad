package revolveloftfaces

import (
	"context"
	"fmt"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/sketch"
)

type RevolveLoftFacesJointKind uint8

const (
	RevolveLoftFacesFitKnotContact RevolveLoftFacesJointKind = iota + 1
	RevolveLoftFacesRadialArcContact
	RevolveLoftFacesWholeArcContact
	RevolveLoftFacesFitLineContact
)

// Each root owns one live source endpoint. Fit-knot, radial-arc and fit-line
// records remain distinct even when their time brackets occupy one span.
type RevolveLoftFacesOwnedRoot struct {
	Root  loftface.EndpointRoot
	Cell  RevolveLoftFacesJointCell
	Kind  RevolveLoftFacesJointKind
	Point *sketch.Point
	AtEnd bool
}

func revolveLoftFacesMergeEndpointCells(ctx context.Context,
	trim *loftFacesTrimRecord, section revolveSectionSpan,
	segment, spanIndex, band int, span freeform.BezierSpan,
	knot *revolveLoftFacesFitKnotJoint, radial *revolveLoftFacesRadialJoint,
	leftSpan bool) ([]RevolveLoftFacesMappedCell, bool, error) {
	if ctx == nil || trim == nil || knot == nil || radial == nil ||
		band < 0 || band+1 >= len(trim.Maps) ||
		knot.vertex.segment != segment || knot.band != band ||
		radial.vertex.fitSegment != segment || radial.radial.Band != band ||
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
	roots := make([]RevolveLoftFacesOwnedRoot, 0, len(knot.roots)+len(radial.roots))
	for i, root := range knot.roots {
		roots = append(roots, RevolveLoftFacesOwnedRoot{
			Root: root, Cell: knotCells[i], Kind: RevolveLoftFacesFitKnotContact,
			Point: knot.vertex.point, AtEnd: leftSpan})
	}
	for i, root := range radial.roots {
		roots = append(roots, RevolveLoftFacesOwnedRoot{
			Root: root, Cell: radial.fitCells[i], Kind: RevolveLoftFacesRadialArcContact,
			Point: radial.vertex.point, AtEnd: radial.vertex.fitAtEnd})
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
	roots []RevolveLoftFacesOwnedRoot) ([]RevolveLoftFacesMappedCell, bool, error) {
	if ctx == nil || trim == nil || band < 0 || band+1 >= len(trim.Maps) ||
		len(roots) == 0 || len(roots) > 4 || len(span) == 0 {
		return nil, false, nil
	}
	if !RevolveLoftFacesOrderOwnedRoots(roots) {
		return nil, false, nil
	}
	m, b := RevolveLoftFacesMeridian(section)
	q := RevolveLoftFacesMeridianMemoOf(ctx).QRows(trim.Maps[band], trim.Maps[band+1], span, m, b)
	j := loftface.MeridianJRows(trim.Maps[band], trim.Maps[band+1], span, q)
	remaining := RevolveLoftFacesMaxBandCells
	previous := new(big.Rat)
	var cells []RevolveLoftFacesMappedCell
	appendGap := func(hi *big.Rat, loOpen, hiOpen bool) (bool, error) {
		if previous.Cmp(hi) >= 0 {
			return false, nil
		}
		gap, ok, err := RevolveLoftFacesOpenTimeSlab(ctx, q, j,
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
		ok, err := appendGap(owned.Root.Time.Lo, i != 0, true)
		if err != nil || !ok {
			return nil, false, err
		}
		cell := owned.Cell
		makeCell := func(side int, sLo, sHi *big.Rat) RevolveLoftFacesMappedCell {
			return RevolveLoftFacesMappedCell{Segment: segment, Span: spanIndex,
				Band: band, Side: side, SLo: sLo, SHi: sHi,
				TLo: owned.Root.Time.Lo, THi: owned.Root.Time.Hi}
		}
		if owned.AtEnd && cell.SLo.Sign() > 0 {
			cells = append(cells, makeCell(cell.OutsideSide, new(big.Rat), cell.SLo))
		}
		if !owned.AtEnd && cell.SHi.Cmp(big.NewRat(1, 1)) < 0 {
			cells = append(cells, makeCell(cell.OutsideSide, cell.SHi, big.NewRat(1, 1)))
		}
		side := RevolveLoftFacesJointContact
		if !cell.Finite {
			side = RevolveLoftFacesAxialMiss
		}
		contact := makeCell(side, cell.SLo, cell.SHi)
		contact.JointKind, contact.JointPoint = owned.Kind, owned.Point
		cells = append(cells, contact)
		previous = owned.Root.Time.Hi
	}
	ok, err := appendGap(big.NewRat(1, 1), true, false)
	if err != nil || !ok {
		return nil, false, err
	}
	return cells, true, nil
}

func RevolveLoftFacesOrderOwnedRoots(roots []RevolveLoftFacesOwnedRoot) bool {
	if len(roots) == 0 || len(roots) > 4 {
		return false
	}
	for _, owned := range roots {
		if owned.Root.Time.Lo == nil {
			return false
		}
	}
	sort.Slice(roots, func(i, j int) bool {
		return roots[i].Root.Time.Lo.Cmp(roots[j].Root.Time.Lo) < 0
	})
	previous := new(big.Rat)
	for _, owned := range roots {
		root, cell := owned.Root, owned.Cell
		if owned.Point == nil || owned.Kind != RevolveLoftFacesFitKnotContact &&
			owned.Kind != RevolveLoftFacesRadialArcContact &&
			owned.Kind != RevolveLoftFacesFitLineContact ||
			root.Time.Lo == nil || root.Time.Hi == nil || cell.Time.Lo == nil ||
			cell.Time.Hi == nil || cell.SLo == nil || cell.SHi == nil ||
			root.Time.Lo.Cmp(previous) <= 0 ||
			root.Time.Hi.Cmp(root.Time.Lo) < 0 ||
			root.Time.Hi.Cmp(big.NewRat(1, 1)) >= 0 ||
			cell.Time.Lo.Cmp(root.Time.Lo) != 0 ||
			cell.Time.Hi.Cmp(root.Time.Hi) != 0 ||
			cell.SLo.Sign() < 0 || cell.SHi.Cmp(big.NewRat(1, 1)) > 0 ||
			cell.SLo.Cmp(cell.SHi) >= 0 ||
			cell.Finite != root.Finite ||
			owned.AtEnd && cell.SHi.Cmp(big.NewRat(1, 1)) != 0 ||
			!owned.AtEnd && cell.SLo.Sign() != 0 {
			return false
		}
		previous = root.Time.Hi
	}
	return true
}
