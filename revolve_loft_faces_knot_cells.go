package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
)

// This partitions one authenticated fitted span across the exact time roots
// of one adjacent fit knot. Open time pieces keep ordinary strict/crossing
// cells. Each root bracket has a proved strict outer source range plus a
// monotone one-sided sliver; only a finite sliver records a joint contact.
func revolveLoftFacesKnotPartitionCells(ctx context.Context,
	trim *loftFacesTrimRecord, section revolveSectionSpan,
	segment, spanIndex, band int, span freeform.BezierSpan,
	joint revolveLoftFacesFitKnotJoint, leftSpan bool) (
	[]revolveLoftFacesMappedCell, bool, error,
) {
	if ctx == nil || trim == nil || joint.vertex.segment != segment ||
		joint.band != band ||
		leftSpan && spanIndex != joint.vertex.leftIndex ||
		!leftSpan && spanIndex != joint.vertex.leftIndex+1 {
		return nil, false, nil
	}
	var knotCells []revolveLoftFacesJointCell
	if leftSpan {
		knotCells = joint.leftCells
	} else {
		knotCells = joint.rightCells
	}
	cells, ok, err := revolveLoftFacesEndpointPartitionCells(ctx, trim, section, segment, spanIndex,
		band, span, joint.roots, joint.pieces, knotCells, leftSpan)
	for i := range cells {
		if cells[i].side == revolveLoftFacesJointContact {
			cells[i].jointKind = revolveLoftFacesFitKnotContact
			cells[i].jointPoint = joint.vertex.point
		}
	}
	return cells, ok, err
}

func revolveLoftFacesEndpointPartitionCells(ctx context.Context,
	trim *loftFacesTrimRecord, section revolveSectionSpan,
	segment, spanIndex, band int, span freeform.BezierSpan,
	roots []revolveLoftFacesEndpointRoot, pieces []revolveLoftFacesTimePiece,
	jointCells []revolveLoftFacesJointCell, atEnd bool) (
	[]revolveLoftFacesMappedCell, bool, error,
) {
	if ctx == nil || trim == nil || band < 0 || band+1 >= len(trim.maps) ||
		len(roots) == 0 || len(pieces) != 2*len(roots)+1 ||
		len(jointCells) != len(roots) {
		return nil, false, nil
	}
	m, b := revolveLoftFacesMeridian(section)
	q := spiralRootPolynomial(trim.maps[band], trim.maps[band+1], span, m, b)
	j := spiralRootXOrder(trim.maps[band], trim.maps[band+1], span, q)
	remaining := revolveLoftFacesMaxBandCells
	var cells []revolveLoftFacesMappedCell
	for _, piece := range pieces {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		if piece.rootIndex >= 0 {
			if piece.rootIndex >= len(jointCells) {
				return nil, false, nil
			}
			rootCell := jointCells[piece.rootIndex]
			if rootCell.time.lo == nil || rootCell.time.lo.Cmp(piece.lo) != 0 ||
				rootCell.time.hi.Cmp(piece.hi) != 0 {
				return nil, false, nil
			}
			makeCell := func(side int, sLo, sHi *big.Rat) revolveLoftFacesMappedCell {
				return revolveLoftFacesMappedCell{segment: segment, span: spanIndex,
					band: band, side: side, sLo: sLo, sHi: sHi,
					tLo: piece.lo, tHi: piece.hi}
			}
			if atEnd && rootCell.sLo.Sign() > 0 {
				cells = append(cells, makeCell(rootCell.outsideSide, lfZero(), rootCell.sLo))
			}
			if !atEnd && rootCell.sHi.Cmp(lfOne()) < 0 {
				cells = append(cells, makeCell(rootCell.outsideSide, rootCell.sHi, lfOne()))
			}
			side := revolveLoftFacesJointContact
			if !rootCell.finite {
				side = revolveLoftFacesAxialMiss
			}
			cells = append(cells, makeCell(side, rootCell.sLo, rootCell.sHi))
			continue
		}
		if piece.loOpen && piece.lo.Cmp(piece.hi) == 0 ||
			piece.hiOpen && piece.lo.Cmp(piece.hi) == 0 {
			return nil, false, nil
		}
		openCells, ok, err := revolveLoftFacesTimeSlab(ctx, q, j,
			trim.maps[band], trim.maps[band+1], span, section,
			segment, spanIndex, band, piece.lo, piece.hi, 0, &remaining)
		if err != nil {
			return nil, false, fmt.Errorf("mapped fitted span %d/%d band %d: %w",
				segment, spanIndex, band, err)
		}
		if !ok {
			return nil, false, nil
		}
		cells = append(cells, openCells...)
	}
	return cells, len(cells) != 0, nil
}
