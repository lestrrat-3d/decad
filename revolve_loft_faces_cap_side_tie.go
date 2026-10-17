package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
)

// A cap endpoint and a source-side cell name the same true mapped source
// point. The point itself may be irrational; these are its two certificates.
type revolveLoftFacesCapSideTie struct {
	boundary     revolveLoftFacesToolCapBoundary
	sourceCell   *revolveLoftFacesSourceCell
	verticalCell *revolveLoftFacesVerticalCell
}

type revolveLoftFacesHeelArcSideTies struct {
	negativeZ, positiveZ revolveLoftFacesCapSideTie
}

// The exact cap circle fixes both X and radius. On a nonvertical blank side,
// the mapped meridian polynomial equals the cap radial polynomial at this
// authenticated constant-X band time. A transverse side cell has one source
// root, so both certificates name the same point. A vertical side instead
// shares the cap's exact radial-root record.
func revolveLoftFacesHeelArcSideTiesOf(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	arc *revolveLoftFacesToolCapOccupiedArc) (*revolveLoftFacesHeelArcSideTies, bool, error) {
	return revolveLoftFacesHeelArcSideTiesIn(ctx, proof, nil, arc)
}

// census, when not nil, must be the census that revolveLoftFacesCensus
// returned with ok for this same proof earlier in the caller's call. An arc
// that census derived is compared with its snapshot; any other arc, or any
// arc with a nil census, is rederived from the live source. A nil census is
// derived here, after the arc is checked.
func revolveLoftFacesHeelArcSideTiesIn(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, census *revolveLoftFacesPairCensus,
	arc *revolveLoftFacesToolCapOccupiedArc) (*revolveLoftFacesHeelArcSideTies, bool, error) {
	if ctx == nil || proof == nil || arc == nil || arc.time == nil ||
		arc.circle.span < 0 || arc.circle.span >= len(proof.section.Spans) {
		return nil, false, nil
	}
	current, err := revolveLoftFacesToolCapOccupiedArcCurrentIn(ctx, proof, census, arc)
	if err != nil || !current {
		return nil, false, err
	}
	if census == nil {
		fresh, ok, err := revolveLoftFacesCensus(ctx, proof)
		if err != nil || !ok {
			return nil, false, err
		}
		census = fresh
	} else if census.manifest == nil {
		return nil, false, nil
	}
	var capPair *revolveLoftFacesFacePair
	for index := range census.pairs {
		pair := &census.pairs[index]
		if pair.blankFace == arc.circle.blankFace && pair.trimmedFace == arc.circle.capFace &&
			pair.verdict == revolveLoftFacesPairToolArc &&
			revolveLoftFacesToolCapOccupiedArcEqual(pair.toolArc, arc) {
			if capPair != nil {
				return nil, false, nil
			}
			capPair = pair
		}
	}
	if capPair == nil || capPair.sectionIndex != arc.circle.span {
		return nil, false, nil
	}
	negative, ok := revolveLoftFacesHeelBoundarySideTie(proof, census, arc, arc.negativeZ)
	if !ok {
		return nil, false, nil
	}
	positive, ok := revolveLoftFacesHeelBoundarySideTie(proof, census, arc, arc.positiveZ)
	if !ok || negative.boundary.ref == positive.boundary.ref &&
		negative.boundary.span == positive.boundary.span {
		return nil, false, nil
	}
	return &revolveLoftFacesHeelArcSideTies{
		negativeZ: negative, positiveZ: positive}, true, nil
}

func revolveLoftFacesHeelBoundarySideTie(proof *revolveLoftFacesSourceCertificate,
	census *revolveLoftFacesPairCensus, arc *revolveLoftFacesToolCapOccupiedArc,
	boundary revolveLoftFacesToolCapBoundary) (revolveLoftFacesCapSideTie, bool) {
	var result revolveLoftFacesCapSideTie
	if proof == nil || proof.trim == nil || proof.trim.Source == nil ||
		proof.trim.Source.Source == nil ||
		census == nil || census.manifest == nil || arc == nil || arc.time == nil ||
		arc.circle.span < 0 || arc.circle.span >= len(proof.section.Spans) ||
		boundary.face == nil || boundary.segment < 0 ||
		boundary.segment >= len(proof.trim.Source.Profile.Outer.Segments) {
		return result, false
	}
	section := proof.section.Spans[arc.circle.span]
	vertical := section.Start.U == section.End.U
	if vertical {
		matchedSide := false
		for _, entry := range census.manifest.vertical {
			if entry.sectionIndex != arc.circle.span || entry.side == nil ||
				entry.side.band != arc.band || entry.side.time == nil ||
				entry.side.time.Cmp(arc.time) != 0 || entry.side.x == nil ||
				entry.side.x.Cmp(arc.circle.x) != 0 || entry.side.upper == nil ||
				entry.side.upper.Cmp(arc.circle.radius) != 0 {
				continue
			}
			if matchedSide {
				return result, false
			}
			matchedSide = true
		}
		if !matchedSide {
			return result, false
		}
	} else if !revolveLoftFacesCapMeridianIdentity(proof, arc, boundary) {
		return result, false
	}
	matches := 0
	for _, pair := range census.pairs {
		if pair.blankFace != arc.circle.blankFace || pair.trimmedFace != boundary.face ||
			pair.sectionIndex != arc.circle.span ||
			pair.verdict != revolveLoftFacesPairSourceCandidate {
			continue
		}
		if vertical {
			for _, cell := range pair.verticalCells {
				if cell.segment != boundary.segment || cell.span != boundary.span ||
					cell.band != arc.band || cell.ref != boundary.ref ||
					cell.face != boundary.face ||
					!loftface.IntervalEqual(cell.upper.Root, boundary.parameter) ||
					!revolveLoftFacesCapBoundaryBoxEqual(cell.upper.Box, boundary.box) {
					continue
				}
				matches++
				matchedCell := cell
				result = revolveLoftFacesCapSideTie{boundary: boundary, verticalCell: &matchedCell}
			}
			continue
		}
		for _, cell := range pair.sourceCells {
			mapped := cell.mapped
			if mapped.Segment != boundary.segment || mapped.Span != boundary.span ||
				mapped.Band != arc.band || cell.ref != boundary.ref ||
				cell.face != boundary.face || mapped.Side != loftface.SideCrossed ||
				!revolveLoftFacesTimeInCell(arc.time, mapped) {
				continue
			}
			matches++
			matchedCell := cell
			result = revolveLoftFacesCapSideTie{boundary: boundary, sourceCell: &matchedCell}
		}
	}
	return result, matches == 1
}

func revolveLoftFacesCapMeridianIdentity(proof *revolveLoftFacesSourceCertificate,
	arc *revolveLoftFacesToolCapOccupiedArc,
	boundary revolveLoftFacesToolCapBoundary) bool {
	if proof == nil || proof.trim == nil || proof.trim.Source == nil ||
		proof.trim.Source.Source == nil || arc == nil ||
		arc.time == nil || arc.circle.x == nil || arc.circle.radius == nil ||
		arc.circle.span < 0 || arc.circle.span >= len(proof.section.Spans) ||
		arc.band < 0 || arc.band+1 >= len(proof.trim.Maps) {
		return false
	}
	section := proof.section.Spans[arc.circle.span]
	if section.Start.U == section.End.U ||
		boundary.segment < 0 ||
		boundary.segment >= len(proof.trim.Source.Profile.Outer.Segments) {
		return false
	}
	box, err := revolveloftfaces.LfSourceBox(proof.trim.Source)
	if err != nil {
		return false
	}
	band, time, ok := loftface.ConstantXBand(proof.trim.Maps, box, arc.circle.x)
	if !ok || band != arc.band || time.Cmp(arc.time) != 0 {
		return false
	}
	m, b := revolveloftfaces.RevolveLoftFacesMeridian(section)
	if lfAdd(lfMul(m, arc.circle.x), b).Cmp(arc.circle.radius) != 0 {
		return false
	}
	segment := proof.trim.Source.Profile.Outer.Segments[boundary.segment]
	spans, ok := revolveloftfaces.RevolveLoftFacesStraightOrFitSpans(proof.trim, boundary.segment, segment)
	if !ok || boundary.span < 0 || boundary.span >= len(spans) {
		return false
	}
	first, last := proof.trim.Maps[band], proof.trim.Maps[band+1]
	side := polynomial.BernsteinRowsAt(loftface.MeridianQRows(first, last,
		spans[boundary.span], m, b), time)

	capPolynomial := polynomial.BernsteinRowsAt(loftface.MeridianQRows(first, last,
		spans[boundary.span], lfZero(), arc.circle.radius), time)

	if len(side) != len(capPolynomial) || len(side) < 2 {
		return false
	}
	for index := range side {
		if side[index].Cmp(capPolynomial[index]) != 0 {
			return false
		}
	}
	return true
}

func revolveLoftFacesTimeInCell(time *big.Rat, cell revolveloftfaces.RevolveLoftFacesMappedCell) bool {
	if time == nil || cell.TLo == nil || cell.THi == nil {
		return false
	}
	lo, hi := time.Cmp(cell.TLo), time.Cmp(cell.THi)
	return (lo > 0 || lo == 0 && !cell.TLoOpen) &&
		(hi < 0 || hi == 0 && !cell.THiOpen)
}

func revolveLoftFacesCapBoundaryBoxEqual(a, b [3]loftFaceInterval) bool {
	for axis := range a {
		if !loftface.IntervalEqual(a[axis], b[axis]) {
			return false
		}
	}
	return true
}
