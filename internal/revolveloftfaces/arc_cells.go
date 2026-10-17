package revolveloftfaces

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/sketch"
)

// Every whole live Sketch arc gets one source cell per certified time piece.
// A radial arc has constant axial and cone-implicit values over its entire
// parameter; a time root therefore owns a whole-arc contact, not an angular
// point. Other arcs must prove a strict finite side or axial miss.
func RevolveLoftFacesArcSpanCells(ctx context.Context,
	trim *loftFacesTrimRecord, section revolveSectionSpan,
	segment int) ([]RevolveLoftFacesMappedCell, bool, error) {
	if ctx == nil || trim == nil || trim.Source == nil || trim.Source.Source == nil ||
		segment < 0 || segment >= len(trim.Source.Profile.Outer.Segments) ||
		segment >= len(trim.Source.Source.Entities) {
		return nil, false, nil
	}
	record, ok := trim.Source.Profile.Outer.Segments[segment].(arcSeg)
	arc, live := trim.Source.Source.Entities[segment].(*sketch.Arc)
	if !ok || !live || !MappedArcRecordMatches(record, arc) {
		return nil, false, nil
	}
	var cells []RevolveLoftFacesMappedCell
	for band := 0; band+1 < len(trim.Maps); band++ {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		radial, radialOK := MappedArcRadialTimeBand(ctx, trim, segment, band, section)
		if radialOK {
			q := [3][]*big.Rat{{radial.Q[0]}, {radial.Q[1]}, {radial.Q[2]}}
			roots, rootsOK := loftface.EndpointTimeRoots(q, false)
			if !rootsOK {
				return nil, false, fmt.Errorf("%w: radial source arc %d band %d has undecidable time roots",
					decaderr.ErrUnsupported, segment, band)
			}
			if len(roots) > 0 {
				for i, root := range roots {
					roots[i], rootsOK = RevolveLoftFacesEndpointAxialClass(root,
						radial.X[0], radial.X[1], section)
					if !rootsOK {
						return nil, false, nil
					}
				}
				pieces, partitionOK := loftface.PartitionTime(radial.Q, roots)
				if !partitionOK {
					return nil, false, nil
				}
				for _, piece := range pieces {
					cell := RevolveLoftFacesMappedCell{Segment: segment, Span: -1,
						Band: band, Side: piece.Side, SLo: new(big.Rat), SHi: big.NewRat(1, 1),
						TLo: piece.Lo, THi: piece.Hi,
						TLoOpen: piece.LoOpen, THiOpen: piece.HiOpen}
					if piece.RootIndex >= 0 {
						cell.Side = RevolveLoftFacesAxialMiss
						if roots[piece.RootIndex].Finite {
							cell.Side = RevolveLoftFacesJointContact
							cell.JointKind = RevolveLoftFacesWholeArcContact
							cell.JointArc = arc
						}
					}
					cells = append(cells, cell)
				}
				continue
			}
		}
		side, sideOK := mappedArcFiniteSideCell(ctx, trim, segment, section,
			band, 0, 1, new(big.Rat), big.NewRat(1, 1))
		if !sideOK {
			return nil, false, fmt.Errorf("%w: mapped source arc %d band %d has undecidable finite side",
				decaderr.ErrUnsupported, segment, band)
		}
		if side == loftface.ArcMiss {
			side = RevolveLoftFacesAxialMiss
		}
		cells = append(cells, RevolveLoftFacesMappedCell{Segment: segment,
			Span: -1, Band: band, Side: side,
			SLo: new(big.Rat), SHi: big.NewRat(1, 1), TLo: new(big.Rat), THi: big.NewRat(1, 1)})
	}
	return cells, len(cells) > 0, nil
}
