package revolvecontact

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
type CapSeamVertex struct {
	Boundary              ToolCapBoundary
	Point                 r3.Vec
	BlankBound, LoftBound float64
}

func CapArcSeamVertices(ctx context.Context,
	proof *SourceCertificate,
	arc *ToolCapOccupiedArc,
) ([2]CapSeamVertex, bool, error) {
	return CapArcSeamVerticesIn(ctx, proof, nil, arc)
}

// CapArcSeamVerticesIn takes the census under the contract of
// HeelArcSideTiesIn; a nil census rederives the arc.
func CapArcSeamVerticesIn(ctx context.Context,
	proof *SourceCertificate, census *PairCensus,
	arc *ToolCapOccupiedArc,
) ([2]CapSeamVertex, bool, error) {
	var out [2]CapSeamVertex
	current, err := ToolCapOccupiedArcCurrentIn(ctx, proof, census, arc)
	if err != nil || !current {
		return out, false, err
	}
	for i, boundary := range [...]ToolCapBoundary{
		arc.NegativeZ, arc.PositiveZ,
	} {
		vertex, ok := CapBoundarySeamVertex(arc.Circle, boundary)
		if !ok {
			return [2]CapSeamVertex{}, false, nil
		}
		out[i] = vertex
	}
	return out, true, nil
}

func CapBoundarySeamVertex(circle ToolCapCircle,
	boundary ToolCapBoundary,
) (CapSeamVertex, bool) {
	var out CapSeamVertex
	if circle.X == nil || circle.Radius == nil || circle.Radius.Sign() <= 0 ||
		boundary.Parameter.Lo == nil || boundary.Parameter.Hi == nil ||
		boundary.Parameter.Lo.Cmp(boundary.Parameter.Hi) >= 0 {
		return out, false
	}
	var middle [3]*big.Rat
	for axis, interval := range boundary.Box {
		if interval.Lo == nil || interval.Hi == nil ||
			interval.Lo.Cmp(interval.Hi) > 0 {
			return out, false
		}
		middle[axis] = lfDiv(lfAdd(interval.Lo, interval.Hi), big.NewRat(2, 1))
	}
	if boundary.Box[0].Lo.Cmp(circle.X) > 0 ||
		boundary.Box[0].Hi.Cmp(circle.X) < 0 ||
		lfAdd(lfMul(middle[1], middle[1]),
			lfMul(middle[2], middle[2])).Sign() <= 0 {
		return out, false
	}
	point, _, ok := revolveloftfaces.MeridianRayPoint(
		[2]*big.Rat{middle[1], middle[2]}, circle.X, circle.Radius)
	if !ok {
		return out, false
	}
	coordinates := [3]float64{point.X, point.Y, point.Z}
	distance := lfZero()
	for axis, held := range coordinates {
		interval := boundary.Box[axis]
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
	return CapSeamVertex{
		Boundary: boundary, Point: point,
		BlankBound: bound, LoftBound: bound}, true
}

// A cap endpoint and a source-side cell name the same true mapped source
// point. The point itself may be irrational; these are its two certificates.
type CapSideTie struct {
	Boundary     ToolCapBoundary
	SourceCell   *SourceCell
	VerticalCell *VerticalCell
}

type HeelArcSideTies struct {
	NegativeZ, PositiveZ CapSideTie
}

// The exact cap circle fixes both X and radius. On a nonvertical blank side,
// the mapped meridian polynomial equals the cap radial polynomial at this
// authenticated constant-X band time. A transverse side cell has one source
// root, so both certificates name the same point. A vertical side instead
// shares the cap's exact radial-root record.
func HeelArcSideTiesOf(ctx context.Context,
	proof *SourceCertificate,
	arc *ToolCapOccupiedArc) (*HeelArcSideTies, bool, error) {
	return HeelArcSideTiesIn(ctx, proof, nil, arc)
}

// census, when not nil, must be the census that Census
// returned with ok for this same proof earlier in the caller's call. An arc
// that census derived is compared with its snapshot; any other arc, or any
// arc with a nil census, is rederived from the live source. A nil census is
// derived here, after the arc is checked.
func HeelArcSideTiesIn(ctx context.Context,
	proof *SourceCertificate, census *PairCensus,
	arc *ToolCapOccupiedArc) (*HeelArcSideTies, bool, error) {
	if ctx == nil || proof == nil || arc == nil || arc.Time == nil ||
		arc.Circle.Span < 0 || arc.Circle.Span >= len(proof.Section.Spans) {
		return nil, false, nil
	}
	current, err := ToolCapOccupiedArcCurrentIn(ctx, proof, census, arc)
	if err != nil || !current {
		return nil, false, err
	}
	if census == nil {
		fresh, ok, err := Census(ctx, proof)
		if err != nil || !ok {
			return nil, false, err
		}
		census = fresh
	} else if census.Manifest == nil {
		return nil, false, nil
	}
	var capPair *FacePair
	for index := range census.Pairs {
		pair := &census.Pairs[index]
		if pair.BlankFace == arc.Circle.BlankFace && pair.TrimmedFace == arc.Circle.CapFace &&
			pair.Verdict == PairToolArc &&
			ToolCapOccupiedArcEqual(pair.ToolArc, arc) {
			if capPair != nil {
				return nil, false, nil
			}
			capPair = pair
		}
	}
	if capPair == nil || capPair.SectionIndex != arc.Circle.Span {
		return nil, false, nil
	}
	negative, ok := HeelBoundarySideTie(proof, census, arc, arc.NegativeZ)
	if !ok {
		return nil, false, nil
	}
	positive, ok := HeelBoundarySideTie(proof, census, arc, arc.PositiveZ)
	if !ok || negative.Boundary.Ref == positive.Boundary.Ref &&
		negative.Boundary.Span == positive.Boundary.Span {
		return nil, false, nil
	}
	return &HeelArcSideTies{
		NegativeZ: negative, PositiveZ: positive}, true, nil
}

func HeelBoundarySideTie(proof *SourceCertificate,
	census *PairCensus, arc *ToolCapOccupiedArc,
	boundary ToolCapBoundary) (CapSideTie, bool) {
	var result CapSideTie
	if proof == nil || proof.Trim == nil || proof.Trim.Source == nil ||
		proof.Trim.Source.Source == nil ||
		census == nil || census.Manifest == nil || arc == nil || arc.Time == nil ||
		arc.Circle.Span < 0 || arc.Circle.Span >= len(proof.Section.Spans) ||
		boundary.Face == nil || boundary.Segment < 0 ||
		boundary.Segment >= len(proof.Trim.Source.Profile.Outer.Segments) {
		return result, false
	}
	section := proof.Section.Spans[arc.Circle.Span]
	vertical := section.Start.U == section.End.U
	if vertical {
		matchedSide := false
		for _, entry := range census.Manifest.Vertical {
			if entry.SectionIndex != arc.Circle.Span || entry.Side == nil ||
				entry.Side.Band != arc.Band || entry.Side.Time == nil ||
				entry.Side.Time.Cmp(arc.Time) != 0 || entry.Side.X == nil ||
				entry.Side.X.Cmp(arc.Circle.X) != 0 || entry.Side.Upper == nil ||
				entry.Side.Upper.Cmp(arc.Circle.Radius) != 0 {
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
	} else if !CapMeridianIdentity(proof, arc, boundary) {
		return result, false
	}
	matches := 0
	for _, pair := range census.Pairs {
		if pair.BlankFace != arc.Circle.BlankFace || pair.TrimmedFace != boundary.Face ||
			pair.SectionIndex != arc.Circle.Span ||
			pair.Verdict != PairSourceCandidate {
			continue
		}
		if vertical {
			for _, cell := range pair.VerticalCells {
				if cell.Segment != boundary.Segment || cell.Span != boundary.Span ||
					cell.Band != arc.Band || cell.Ref != boundary.Ref ||
					cell.Face != boundary.Face ||
					!loftface.IntervalEqual(cell.Upper.Root, boundary.Parameter) ||
					!revolveloftfaces.CapBoundaryBoxEqual(cell.Upper.Box, boundary.Box) {
					continue
				}
				matches++
				matchedCell := cell
				result = CapSideTie{Boundary: boundary, VerticalCell: &matchedCell}
			}
			continue
		}
		for _, cell := range pair.SourceCells {
			mapped := cell.Mapped
			if mapped.Segment != boundary.Segment || mapped.Span != boundary.Span ||
				mapped.Band != arc.Band || cell.Ref != boundary.Ref ||
				cell.Face != boundary.Face || mapped.Side != loftface.SideCrossed ||
				!revolveloftfaces.TimeInCell(arc.Time, mapped) {
				continue
			}
			matches++
			matchedCell := cell
			result = CapSideTie{Boundary: boundary, SourceCell: &matchedCell}
		}
	}
	return result, matches == 1
}

func CapMeridianIdentity(proof *SourceCertificate,
	arc *ToolCapOccupiedArc,
	boundary ToolCapBoundary) bool {
	if proof == nil || proof.Trim == nil || proof.Trim.Source == nil ||
		proof.Trim.Source.Source == nil || arc == nil ||
		arc.Time == nil || arc.Circle.X == nil || arc.Circle.Radius == nil ||
		arc.Circle.Span < 0 || arc.Circle.Span >= len(proof.Section.Spans) ||
		arc.Band < 0 || arc.Band+1 >= len(proof.Trim.Maps) {
		return false
	}
	section := proof.Section.Spans[arc.Circle.Span]
	if section.Start.U == section.End.U ||
		boundary.Segment < 0 ||
		boundary.Segment >= len(proof.Trim.Source.Profile.Outer.Segments) {
		return false
	}
	box, err := revolveloftfaces.LfSourceBox(proof.Trim.Source)
	if err != nil {
		return false
	}
	band, time, ok := loftface.ConstantXBand(proof.Trim.Maps, box, arc.Circle.X)
	if !ok || band != arc.Band || time.Cmp(arc.Time) != 0 {
		return false
	}
	m, b := revolveloftfaces.RevolveLoftFacesMeridian(section)
	if lfAdd(lfMul(m, arc.Circle.X), b).Cmp(arc.Circle.Radius) != 0 {
		return false
	}
	segment := proof.Trim.Source.Profile.Outer.Segments[boundary.Segment]
	spans, ok := revolveloftfaces.RevolveLoftFacesStraightOrFitSpans(proof.Trim, boundary.Segment, segment)
	if !ok || boundary.Span < 0 || boundary.Span >= len(spans) {
		return false
	}
	first, last := proof.Trim.Maps[band], proof.Trim.Maps[band+1]
	side := polynomial.BernsteinRowsAt(loftface.MeridianQRows(first, last,
		spans[boundary.Span], m, b), time)

	capPolynomial := polynomial.BernsteinRowsAt(loftface.MeridianQRows(first, last,
		spans[boundary.Span], lfZero(), arc.Circle.Radius), time)

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
func BlankShellWithCapRings(ctx context.Context,
	proof *SourceCertificate, sectionIndex, cellIndex int,
) (*BlankSeamShell, bool, error) {
	ctx = WithCallScope(ctx)
	graph, ok, err := ContactGraphOf(ctx, proof)
	if err != nil || !ok || graph == nil || graph.Manifest == nil {
		return nil, false, err
	}
	if sectionIndex < 0 || sectionIndex >= len(proof.Section.Spans) {
		return nil, false, nil
	}
	var selected *GraphRun
	for i := range graph.Runs {
		run := &graph.Runs[i]
		cell := graph.Cells[run.CellID]
		if cell.SectionIndex == sectionIndex && cell.CellIndex == cellIndex &&
			cell.Source != nil && cell.BlankFace == proof.Section.Spans[sectionIndex].Face {
			if selected != nil {
				return nil, false, nil
			}
			selected = run
		}
	}
	if selected == nil {
		return nil, false, nil
	}
	cuts, valid := GraphCapCuts(graph)
	if !valid {
		return nil, false, nil
	}
	shell, built, err := BlankShellAtRunsWithCuts(ctx, proof, graph.Manifest,
		[]ShellSeamSelector{{SectionIndex: sectionIndex, CellIndex: cellIndex}}, cuts)
	if err != nil || !built {
		return nil, false, err
	}
	if shell == nil || shell.FaceBound >= 0.1 {
		return nil, false, nil
	}
	bound, bindErr := graph.BindBlankShell(ctx, proof, selected, shell)
	if bindErr != nil || !bound {
		return nil, false, bindErr
	}
	return shell, true, nil
}

func GraphCapCuts(graph *ContactGraph) (
	[]ShellCut, bool,
) {
	if graph == nil {
		return nil, false
	}
	var cuts []ShellCut
	for _, event := range graph.Arcs {
		if event.PairID < 0 || event.PairID >= len(graph.Pairs) {
			return nil, false
		}
		pair := graph.Pairs[event.PairID]
		arc := event.Arc
		if arc == nil || arc.Circle.Tool != 1 || arc.Circle.SourceMiss ||
			arc.Circle.BlankFace != pair.BlankFace ||
			arc.Circle.CapFace != pair.TrimmedFace ||
			arc.Circle.Span != pair.SectionIndex ||
			event.EndpointIDs[0] == event.EndpointIDs[1] ||
			event.EndpointCellIDs[0] == event.EndpointCellIDs[1] ||
			arc.Circle.X == nil || arc.Circle.Radius == nil ||
			arc.Circle.Radius.Sign() <= 0 {
			return nil, false
		}
		cuts = append(cuts, ShellCut{ShellCut: revolveloftfaces.ShellCut{
			Section: pair.SectionIndex, X: arc.Circle.X,
			Radius: arc.Circle.Radius}, Arc: arc})
	}
	if len(cuts) == 0 {
		return nil, false
	}
	return cuts, true
}
