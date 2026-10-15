package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/sketch"
)

// Every whole live Sketch arc gets one source cell per certified time piece.
// A radial arc has constant axial and cone-implicit values over its entire
// parameter; a time root therefore owns a whole-arc contact, not an angular
// point. Other arcs must prove a strict finite side or axial miss.
func revolveLoftFacesArcSpanCells(ctx context.Context,
	trim *loftFacesTrimRecord, section revolveSectionSpan,
	segment int) ([]revolveLoftFacesMappedCell, bool, error) {
	if ctx == nil || trim == nil || trim.Source == nil || trim.Source.Source == nil ||
		segment < 0 || segment >= len(trim.Source.Profile.Outer.Segments) ||
		segment >= len(trim.Source.Source.Entities) {
		return nil, false, nil
	}
	record, ok := trim.Source.Profile.Outer.Segments[segment].(arcSeg)
	arc, live := trim.Source.Source.Entities[segment].(*sketch.Arc)
	if !ok || !live || !mappedArcRecordMatches(record, arc) {
		return nil, false, nil
	}
	var cells []revolveLoftFacesMappedCell
	for band := 0; band+1 < len(trim.Maps); band++ {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		radial, radialOK := mappedArcRadialTimeBand(ctx, trim, segment, band, section)
		if radialOK {
			q := [3][]*big.Rat{{radial.q[0]}, {radial.q[1]}, {radial.q[2]}}
			roots, rootsOK := loftface.EndpointTimeRoots(q, false)
			if !rootsOK {
				return nil, false, fmt.Errorf("%w: radial source arc %d band %d has undecidable time roots",
					ErrUnsupported, segment, band)
			}
			if len(roots) > 0 {
				for i, root := range roots {
					roots[i], rootsOK = revolveLoftFacesEndpointAxialClass(root,
						radial.x[0], radial.x[1], section)
					if !rootsOK {
						return nil, false, nil
					}
				}
				pieces, partitionOK := loftface.PartitionTime(radial.q, roots)
				if !partitionOK {
					return nil, false, nil
				}
				for _, piece := range pieces {
					cell := revolveLoftFacesMappedCell{segment: segment, span: -1,
						band: band, side: piece.Side, sLo: lfZero(), sHi: lfOne(),
						tLo: piece.Lo, tHi: piece.Hi,
						tLoOpen: piece.LoOpen, tHiOpen: piece.HiOpen}
					if piece.RootIndex >= 0 {
						cell.side = revolveLoftFacesAxialMiss
						if roots[piece.RootIndex].Finite {
							cell.side = revolveLoftFacesJointContact
							cell.jointKind = revolveLoftFacesWholeArcContact
							cell.jointArc = arc
						}
					}
					cells = append(cells, cell)
				}
				continue
			}
		}
		side, sideOK := mappedArcFiniteSideCell(ctx, trim, segment, section,
			band, 0, 1, lfZero(), lfOne())
		if !sideOK {
			return nil, false, fmt.Errorf("%w: mapped source arc %d band %d has undecidable finite side",
				ErrUnsupported, segment, band)
		}
		if side == loftface.ArcMiss {
			side = revolveLoftFacesAxialMiss
		}
		cells = append(cells, revolveLoftFacesMappedCell{segment: segment,
			span: -1, band: band, side: side,
			sLo: lfZero(), sHi: lfOne(), tLo: lfZero(), tHi: lfOne()})
	}
	return cells, len(cells) > 0, nil
}
