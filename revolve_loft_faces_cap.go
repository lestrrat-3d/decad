package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/r3"
)

// A cap endpoint remains an interval-certified true point. The held point
// projects its box center onto the exact cap circle, then charges the full
// distance back to every point in the box. Wall and cap facets will also
// need their own source-curve and chord bounds before a weld can admit Union.
type revolveLoftFacesCapSeamVertex struct {
	boundary              revolveLoftFacesToolCapBoundary
	point                 r3.Vec
	blankBound, loftBound float64
}

func revolveLoftFacesCapArcSeamVertices(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	arc *revolveLoftFacesToolCapOccupiedArc,
) ([2]revolveLoftFacesCapSeamVertex, bool, error) {
	return revolveLoftFacesCapArcSeamVerticesIn(ctx, proof, nil, arc)
}

// revolveLoftFacesCapArcSeamVerticesIn takes the census under the contract of
// revolveLoftFacesHeelArcSideTiesIn; a nil census rederives the arc.
func revolveLoftFacesCapArcSeamVerticesIn(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, census *revolveLoftFacesPairCensus,
	arc *revolveLoftFacesToolCapOccupiedArc,
) ([2]revolveLoftFacesCapSeamVertex, bool, error) {
	var out [2]revolveLoftFacesCapSeamVertex
	current, err := revolveLoftFacesToolCapOccupiedArcCurrentIn(ctx, proof, census, arc)
	if err != nil || !current {
		return out, false, err
	}
	for i, boundary := range [...]revolveLoftFacesToolCapBoundary{
		arc.negativeZ, arc.positiveZ,
	} {
		vertex, ok := revolveLoftFacesCapBoundarySeamVertex(arc.circle, boundary)
		if !ok {
			return [2]revolveLoftFacesCapSeamVertex{}, false, nil
		}
		out[i] = vertex
	}
	return out, true, nil
}

func revolveLoftFacesCapBoundarySeamVertex(circle revolveLoftFacesToolCapCircle,
	boundary revolveLoftFacesToolCapBoundary,
) (revolveLoftFacesCapSeamVertex, bool) {
	var out revolveLoftFacesCapSeamVertex
	if circle.x == nil || circle.radius == nil || circle.radius.Sign() <= 0 ||
		boundary.parameter.Lo == nil || boundary.parameter.Hi == nil ||
		boundary.parameter.Lo.Cmp(boundary.parameter.Hi) >= 0 {
		return out, false
	}
	var middle [3]*big.Rat
	for axis, interval := range boundary.box {
		if interval.Lo == nil || interval.Hi == nil ||
			interval.Lo.Cmp(interval.Hi) > 0 {
			return out, false
		}
		middle[axis] = lfDiv(lfAdd(interval.Lo, interval.Hi), big.NewRat(2, 1))
	}
	if boundary.box[0].Lo.Cmp(circle.x) > 0 ||
		boundary.box[0].Hi.Cmp(circle.x) < 0 ||
		lfAdd(lfMul(middle[1], middle[1]),
			lfMul(middle[2], middle[2])).Sign() <= 0 {
		return out, false
	}
	point, _, ok := revolveloftfaces.MeridianRayPoint(
		[2]*big.Rat{middle[1], middle[2]}, circle.x, circle.radius)
	if !ok {
		return out, false
	}
	coordinates := [3]float64{point.X, point.Y, point.Z}
	distance := lfZero()
	for axis, held := range coordinates {
		interval := boundary.box[axis]
		lower := lfAbs(lfSub(lfF(held), interval.Lo))
		upper := lfAbs(lfSub(lfF(held), interval.Hi))
		if lower.Cmp(upper) > 0 {
			distance = lfAdd(distance, lower)
		} else {
			distance = lfAdd(distance, upper)
		}
	}
	bound := proofbound.RatFloatUp(distance)
	if proofbound.IsNonFinite(bound) || bound < 0 {
		return out, false
	}
	return revolveLoftFacesCapSeamVertex{
		boundary: boundary, point: point,
		blankBound: bound, loftBound: bound}, true
}

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
					!revolveloftfaces.CapBoundaryBoxEqual(cell.upper.Box, boundary.box) {
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
				!revolveloftfaces.TimeInCell(arc.time, mapped) {
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

// The fresh source census supplies every occupied heel circle. Each circle
// splits its authenticated Revolve meridian before any retained-face cut is
// attempted. This is still a complete blank shell, not a Union result.
func revolveLoftFacesBlankShellWithCapRings(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, sectionIndex, cellIndex int,
) (*revolveLoftFacesBlankSeamShell, bool, error) {
	ctx = revolveLoftFacesWithCallScope(ctx)
	graph, ok, err := revolveLoftFacesContactGraphOf(ctx, proof)
	if err != nil || !ok || graph == nil || graph.manifest == nil {
		return nil, false, err
	}
	if sectionIndex < 0 || sectionIndex >= len(proof.section.Spans) {
		return nil, false, nil
	}
	var selected *revolveLoftFacesGraphRun
	for i := range graph.runs {
		run := &graph.runs[i]
		cell := graph.cells[run.cellID]
		if cell.sectionIndex == sectionIndex && cell.cellIndex == cellIndex &&
			cell.source != nil && faceNode(cell.blankFace) == proof.section.Spans[sectionIndex].Face {
			if selected != nil {
				return nil, false, nil
			}
			selected = run
		}
	}
	if selected == nil {
		return nil, false, nil
	}
	cuts, valid := revolveLoftFacesGraphCapCuts(graph)
	if !valid {
		return nil, false, nil
	}
	shell, built, err := revolveLoftFacesBlankShellAtRunsWithCuts(ctx, proof, graph.manifest,
		[]revolveLoftFacesShellSeamSelector{{sectionIndex: sectionIndex, cellIndex: cellIndex}}, cuts)
	if err != nil || !built {
		return nil, false, err
	}
	if shell == nil || shell.FaceBound >= 0.1 {
		return nil, false, nil
	}
	bound, bindErr := graph.bindBlankShell(ctx, proof, selected, shell)
	if bindErr != nil || !bound {
		return nil, false, bindErr
	}
	return shell, true, nil
}

func revolveLoftFacesGraphCapCuts(graph *revolveLoftFacesContactGraph) (
	[]revolveLoftFacesShellCut, bool,
) {
	if graph == nil {
		return nil, false
	}
	var cuts []revolveLoftFacesShellCut
	for _, event := range graph.arcs {
		if event.pairID < 0 || event.pairID >= len(graph.pairs) {
			return nil, false
		}
		pair := graph.pairs[event.pairID]
		arc := event.arc
		if arc == nil || arc.circle.tool != 1 || arc.circle.sourceMiss ||
			arc.circle.blankFace != pair.blankFace ||
			arc.circle.capFace != pair.trimmedFace ||
			arc.circle.span != pair.sectionIndex ||
			event.endpointIDs[0] == event.endpointIDs[1] ||
			event.endpointCellIDs[0] == event.endpointCellIDs[1] ||
			arc.circle.x == nil || arc.circle.radius == nil ||
			arc.circle.radius.Sign() <= 0 {
			return nil, false
		}
		cuts = append(cuts, revolveLoftFacesShellCut{ShellCut: revolveloftfaces.ShellCut{
			Section: pair.sectionIndex, X: arc.circle.x,
			Radius: arc.circle.radius}, arc: arc})
	}
	if len(cuts) == 0 {
		return nil, false
	}
	return cuts, true
}
