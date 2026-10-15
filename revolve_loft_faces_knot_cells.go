package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
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
	roots []loftface.EndpointRoot, pieces []loftface.TimePiece,
	jointCells []revolveLoftFacesJointCell, atEnd bool) (
	[]revolveLoftFacesMappedCell, bool, error,
) {
	if ctx == nil || trim == nil || band < 0 || band+1 >= len(trim.Maps) ||
		len(roots) == 0 || len(pieces) != 2*len(roots)+1 ||
		len(jointCells) != len(roots) {
		return nil, false, nil
	}
	m, b := revolveLoftFacesMeridian(section)
	q := revolveLoftFacesMeridianMemoOf(ctx).QRows(trim.Maps[band], trim.Maps[band+1], span, m, b)
	j := loftface.MeridianJRows(trim.Maps[band], trim.Maps[band+1], span, q)
	remaining := revolveLoftFacesMaxBandCells
	var cells []revolveLoftFacesMappedCell
	for _, piece := range pieces {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		if piece.RootIndex >= 0 {
			if piece.RootIndex >= len(jointCells) {
				return nil, false, nil
			}
			rootCell := jointCells[piece.RootIndex]
			if rootCell.time.Lo == nil || rootCell.time.Lo.Cmp(piece.Lo) != 0 ||
				rootCell.time.Hi.Cmp(piece.Hi) != 0 {
				return nil, false, nil
			}
			makeCell := func(side int, sLo, sHi *big.Rat) revolveLoftFacesMappedCell {
				return revolveLoftFacesMappedCell{segment: segment, span: spanIndex,
					band: band, side: side, sLo: sLo, sHi: sHi,
					tLo: piece.Lo, tHi: piece.Hi}
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
		if piece.LoOpen && piece.Lo.Cmp(piece.Hi) == 0 ||
			piece.HiOpen && piece.Lo.Cmp(piece.Hi) == 0 {
			return nil, false, nil
		}
		openCells, ok, err := revolveLoftFacesOpenTimeSlab(ctx, q, j,
			trim.Maps[band], trim.Maps[band+1], span, section,
			segment, spanIndex, band, piece.Lo, piece.Hi,
			piece.LoOpen, piece.HiOpen, &remaining)
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
