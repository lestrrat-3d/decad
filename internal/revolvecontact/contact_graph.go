package revolvecontact

import (
	"context"
	"fmt"
	"slices"
	"sync"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// A contact graph assigns source-owned identities before either open patch or
// closed blank-shell construction. Its IDs describe source events, not held
// coordinates and not a completed Boolean seam.
type ContactGraph struct {
	Blank, Tooth *topo.Body
	Manifest     *ContactManifest
	Pairs        []GraphPair
	Cells        []GraphCell
	Arcs         []GraphArc
	Runs         []GraphRun
}

type GraphPair struct {
	BlankFace, TrimmedFace *topo.Face
	SectionIndex           int
	Verdict                PairVerdict
	CellIDs                []int
	ArcID                  int
}

type GraphCell struct {
	SectionIndex, SideIndex, CellIndex int
	BlankFace, TrimmedFace             *topo.Face
	Source                             *SourceCell
	Vertical                           *VerticalCell
	PairID                             int // -1 for a source cell with no retained wall
}

type GraphArc struct {
	PairID, SectionIndex int
	Arc                  *ToolCapOccupiedArc
	EndpointIDs          [2]int
	EndpointCellIDs      [2]int
}

type GraphRun struct {
	CellID     int
	Stations   []int
	StationIDs []int
}

func GraphSameVerticalCell(a, b VerticalCell) bool {
	if a.Segment != b.Segment || a.Span != b.Span || a.Band != b.Band ||
		a.Ref != b.Ref || a.Face != b.Face ||
		!revolveloftfaces.RevolveLoftFacesSameBezier(a.Source, b.Source) ||
		!loftface.LimitEqual(a.Lower, b.Lower) ||
		!loftface.LimitEqual(a.Upper, b.Upper) ||
		len(a.Retained) != len(b.Retained) {
		return false
	}
	for i := range a.Retained {
		if !loftface.IntervalEqual(a.Retained[i].Source, b.Retained[i].Source) ||
			a.Retained[i].LoOpen != b.Retained[i].LoOpen ||
			a.Retained[i].HiOpen != b.Retained[i].HiOpen ||
			!revolveloftfaces.CapBoundaryBoxEqual(a.Retained[i].Box, b.Retained[i].Box) {
			return false
		}
	}
	return true
}

func (g *ContactGraph) SourceCellID(section int, blank *topo.Face,
	cell SourceCell) (int, bool) {
	id := -1
	for i := range g.Cells {
		entry := &g.Cells[i]
		if entry.Source == nil || entry.SectionIndex != section ||
			entry.BlankFace != blank || entry.TrimmedFace != cell.Face ||
			entry.Source.Ref != cell.Ref || entry.Source.Face != cell.Face ||
			!revolveloftfaces.SameMappedCell(entry.Source.Mapped, cell.Mapped) {
			continue
		}
		if id >= 0 {
			return -1, false
		}
		id = i
	}
	return id, id >= 0
}

func (g *ContactGraph) VerticalCellID(section int, blank *topo.Face,
	cell VerticalCell) (int, bool) {
	id := -1
	for i := range g.Cells {
		entry := &g.Cells[i]
		if entry.Vertical == nil || entry.SectionIndex != section ||
			entry.BlankFace != blank || entry.TrimmedFace != cell.Face ||
			!GraphSameVerticalCell(*entry.Vertical, cell) {
			continue
		}
		if id >= 0 {
			return -1, false
		}
		id = i
	}
	return id, id >= 0
}

// The census is the source authority. Every live pair and every cell in its
// manifest receives one graph entry, including cells with no retained face.
func ContactGraphOf(ctx context.Context,
	proof *SourceCertificate) (*ContactGraph, bool, error) {
	census, ok, err := CensusIn(ctx, proof)
	if err != nil || !ok || census == nil || census.Manifest == nil || proof == nil {
		return nil, false, err
	}
	blankFaces, trimmedFaces := topobuild.Faces(proof.Blank), topobuild.Faces(proof.Tooth)
	if len(census.Pairs) != len(blankFaces)*len(trimmedFaces) {
		return nil, false, nil
	}
	g := &ContactGraph{Blank: proof.Blank, Tooth: proof.Tooth,
		Manifest: census.Manifest}
	nextEventID := 0
	for sideIndex, side := range census.Manifest.Nonvertical {
		for cellIndex := range side.Cells {
			cell := side.Cells[cellIndex]
			if side.SectionIndex < 0 || side.SectionIndex >= len(proof.Section.Spans) ||
				side.Face != proof.Section.Spans[side.SectionIndex].Face || cell.Face == nil {
				return nil, false, nil
			}
			g.Cells = append(g.Cells, GraphCell{
				SectionIndex: side.SectionIndex, SideIndex: sideIndex, CellIndex: cellIndex,
				BlankFace: side.Face, TrimmedFace: cell.Face, Source: &side.Cells[cellIndex], PairID: -1})
		}
	}
	for sideIndex, side := range census.Manifest.Vertical {
		if side.Side == nil {
			continue
		}
		for cellIndex := range side.Side.Cells {
			cell := side.Side.Cells[cellIndex]
			if side.SectionIndex < 0 || side.SectionIndex >= len(proof.Section.Spans) ||
				side.Side.Face != proof.Section.Spans[side.SectionIndex].Face ||
				(cell.Face == nil) != (len(cell.Retained) == 0) {
				return nil, false, nil
			}
			g.Cells = append(g.Cells, GraphCell{
				SectionIndex: side.SectionIndex, SideIndex: sideIndex, CellIndex: cellIndex,
				BlankFace: side.Side.Face, TrimmedFace: cell.Face,
				Vertical: &side.Side.Cells[cellIndex], PairID: -1})
		}
	}
	seenPairs := make(map[[2]*topo.Face]bool, len(census.Pairs))
	for _, pair := range census.Pairs {
		key := [2]*topo.Face{pair.BlankFace, pair.TrimmedFace}
		if seenPairs[key] || pair.SectionIndex < 0 ||
			pair.SectionIndex >= len(proof.Section.Spans) ||
			proof.Section.Spans[pair.SectionIndex].Face != pair.BlankFace ||
			!slices.Contains(blankFaces, pair.BlankFace) ||
			!slices.Contains(trimmedFaces, pair.TrimmedFace) {
			return nil, false, nil
		}
		seenPairs[key] = true
		entry := GraphPair{BlankFace: pair.BlankFace,
			TrimmedFace: pair.TrimmedFace, SectionIndex: pair.SectionIndex,
			Verdict: pair.Verdict, ArcID: -1}
		pairID := len(g.Pairs)
		for _, cell := range pair.SourceCells {
			if pair.Verdict == PairMiss &&
				(cell.Mapped.Side == revolveloftfaces.RevolveLoftFacesJointContact ||
					cell.Mapped.Side == loftface.SideCrossed) {
				return nil, false, nil
			}
			id, found := g.SourceCellID(pair.SectionIndex, pair.BlankFace, cell)
			if !found || g.Cells[id].PairID >= 0 {
				return nil, false, nil
			}
			g.Cells[id].PairID = pairID
			entry.CellIDs = append(entry.CellIDs, id)
		}
		for _, cell := range pair.VerticalCells {
			id, found := g.VerticalCellID(pair.SectionIndex, pair.BlankFace, cell)
			if !found || g.Cells[id].PairID >= 0 {
				return nil, false, nil
			}
			g.Cells[id].PairID = pairID
			entry.CellIDs = append(entry.CellIDs, id)
		}
		if pair.Verdict == PairToolArc {
			if pair.ToolArc == nil || len(entry.CellIDs) != 0 {
				return nil, false, nil
			}
			entry.ArcID = len(g.Arcs)
			g.Arcs = append(g.Arcs, GraphArc{
				PairID: pairID, SectionIndex: pair.SectionIndex, Arc: pair.ToolArc,
				EndpointIDs: [2]int{nextEventID, nextEventID + 1}})
			nextEventID += 2
		} else if pair.ToolArc != nil || pair.Verdict == PairRefusal ||
			pair.Verdict == PairSourceCandidate && len(entry.CellIDs) == 0 {
			return nil, false, nil
		}
		g.Pairs = append(g.Pairs, entry)
	}
	if len(seenPairs) != len(blankFaces)*len(trimmedFaces) {
		return nil, false, nil
	}
	for id, cell := range g.Cells {
		if cell.TrimmedFace != nil && cell.PairID < 0 ||
			cell.TrimmedFace == nil && cell.PairID >= 0 {
			return nil, false, nil
		}
		if cell.Source == nil || cell.PairID < 0 ||
			g.Pairs[cell.PairID].Verdict != PairSourceCandidate ||
			cell.Source.Mapped.Side != revolveloftfaces.RevolveLoftFacesJointContact ||
			cell.Source.Mapped.JointKind != revolveloftfaces.RevolveLoftFacesWholeArcContact {
			continue
		}
		stations, complete := revolveloftfaces.CompleteArcRun(proof.Trim, cell.Source.Mapped.Segment)
		if !complete {
			return nil, false, nil
		}
		run := GraphRun{CellID: id, Stations: stations}
		for range stations {
			run.StationIDs = append(run.StationIDs, nextEventID)
			nextEventID++
		}
		g.Runs = append(g.Runs, run)
	}
	for i := range g.Arcs {
		arc := &g.Arcs[i]
		ties, tied, tieErr := HeelArcSideTiesIn(ctx, proof, census, arc.Arc)
		if tieErr != nil || !tied {
			return nil, false, tieErr
		}
		for endpoint, tie := range [...]CapSideTie{ties.NegativeZ, ties.PositiveZ} {
			id, found := -1, false
			if tie.SourceCell != nil {
				id, found = g.SourceCellID(arc.SectionIndex, arc.Arc.Circle.BlankFace, *tie.SourceCell)
			} else if tie.VerticalCell != nil {
				id, found = g.VerticalCellID(arc.SectionIndex, arc.Arc.Circle.BlankFace, *tie.VerticalCell)
			}
			if !found || g.Cells[id].PairID < 0 ||
				g.Cells[id].TrimmedFace != tie.Boundary.Face {
				return nil, false, nil
			}
			arc.EndpointCellIDs[endpoint] = id
		}
		if arc.EndpointCellIDs[0] == arc.EndpointCellIDs[1] {
			return nil, false, nil
		}
	}
	return g, true, nil
}

// checkOpenPatches checks the actual held seam vertices of both open patches
// against the run's stations, distinct station IDs and separate side bounds.
func (g *ContactGraph) checkOpenPatches(run *GraphRun,
	seam []revolveloftfaces.ArcSeamVertex, blank, loft *revolveloftfaces.ArcSeamPatch) bool {
	if g == nil || run == nil || blank == nil || loft == nil ||
		len(run.StationIDs) != len(seam) || len(run.Stations) != len(seam) ||
		len(blank.Seam) != len(seam) || len(loft.Seam) != len(seam) {
		return false
	}
	seen := make(map[int]r3.Vec, len(seam))
	for i, stationID := range run.StationIDs {
		bi, li := blank.Seam[i], loft.Seam[i]
		if seam[i].Station != run.Stations[i] ||
			bi < 0 || bi >= len(blank.Vertices) ||
			li < 0 || li >= len(loft.Vertices) ||
			bi >= len(blank.VertexBounds) || li >= len(loft.VertexBounds) ||
			blank.Vertices[bi] != seam[i].Point || loft.Vertices[li] != seam[i].Point ||
			blank.VertexBounds[bi] < seam[i].BlankBound ||
			loft.VertexBounds[li] < seam[i].LoftBound {
			return false
		}
		if _, duplicate := seen[stationID]; duplicate {
			return false
		}
		seen[stationID] = seam[i].Point
	}
	return true
}

// bindBlankShell connects every graph ID to a produced shell index and to
// the independently built open patches. Equal source IDs must use identical
// held coordinates, while each side keeps its own certified bound.
func (g *ContactGraph) BindBlankShell(ctx context.Context,
	proof *SourceCertificate, run *GraphRun,
	shell *BlankSeamShell) (bool, error) {
	if g == nil || proof == nil || run == nil || shell == nil ||
		run.CellID < 0 || run.CellID >= len(g.Cells) ||
		len(shell.Seam) != len(run.StationIDs) {
		return false, nil
	}
	cell := g.Cells[run.CellID]
	if cell.Source == nil || cell.PairID < 0 ||
		g.Pairs[cell.PairID].Verdict != PairSourceCandidate {
		return false, nil
	}
	mapped := cell.Source.Mapped
	seam, blankPatch, loftPatch, ok, err := CertifiedArcPatches(ctx,
		proof.Blank, proof.Tooth, cell.SectionIndex, mapped.Segment, mapped.Band,
		loftface.Interval{Lo: mapped.TLo, Hi: mapped.THi}, run.Stations)
	if err != nil || !ok ||
		!g.checkOpenPatches(run, seam, &blankPatch, &loftPatch) {
		return false, err
	}
	seenStations, seenVertices := make(map[int]bool), make(map[int]bool)
	for i, stationID := range run.StationIDs {
		index := shell.Seam[i]
		if index < 0 || index >= len(shell.Mesh.Verts) ||
			index >= len(shell.VertexBounds) || seenStations[stationID] ||
			seenVertices[index] ||
			shell.Mesh.Verts[index] != seam[i].Point ||
			blankPatch.Vertices[blankPatch.Seam[i]] != shell.Mesh.Verts[index] ||
			loftPatch.Vertices[loftPatch.Seam[i]] != shell.Mesh.Verts[index] ||
			shell.VertexBounds[index] < seam[i].BlankBound {
			return false, nil
		}
		seenStations[stationID], seenVertices[index] = true, true
	}
	shell.SeamIDs = slices.Clone(run.StationIDs)
	seenArcs, seenEndpoints := make(map[int]bool), make(map[int]bool)
	for _, event := range g.Arcs {
		matched := -1
		for i, ring := range shell.CapRings {
			if ToolCapOccupiedArcEqual(event.Arc, ring.Arc) {
				if matched >= 0 {
					return false, nil
				}
				matched = i
			}
		}
		if matched < 0 || seenArcs[matched] {
			return false, nil
		}
		seenArcs[matched] = true
		ring := &shell.CapRings[matched]
		if ring.Section != event.SectionIndex || ring.Ties == nil ||
			event.EndpointIDs[0] == event.EndpointIDs[1] ||
			event.EndpointCellIDs[0] == event.EndpointCellIDs[1] {
			return false, nil
		}
		for endpoint, cellID := range event.EndpointCellIDs {
			index := ring.Endpoints[endpoint]
			if cellID < 0 || cellID >= len(g.Cells) ||
				index < 0 || index >= len(shell.Mesh.Verts) ||
				index >= len(shell.VertexBounds) ||
				seenEndpoints[index] ||
				shell.Mesh.Verts[index] != ring.EndpointProof[endpoint].Point ||
				shell.VertexBounds[index] < ring.EndpointProof[endpoint].BlankBound ||
				!g.EndpointCellMatches(event, ring, endpoint, cellID) {
				return false, nil
			}
			seenEndpoints[index] = true
		}
		ring.EndpointGraphIDs = event.EndpointIDs
		ring.EndpointCellIDs = event.EndpointCellIDs
	}
	return len(seenArcs) == len(shell.CapRings), nil
}

func (g *ContactGraph) EndpointCellMatches(event GraphArc,
	ring *ShellCapRing, endpoint, cellID int) bool {
	cell := g.Cells[cellID]
	if cell.PairID < 0 || cell.SectionIndex != event.SectionIndex ||
		cell.BlankFace != event.Arc.Circle.BlankFace {
		return false
	}
	tie := ring.Ties.NegativeZ
	boundary := event.Arc.NegativeZ
	if endpoint == 1 {
		tie, boundary = ring.Ties.PositiveZ, event.Arc.PositiveZ
	}
	if !ToolCapBoundaryEqual(tie.Boundary, boundary) ||
		cell.TrimmedFace != boundary.Face {
		return false
	}
	if tie.SourceCell != nil {
		return cell.Source != nil && cell.Source.Ref == tie.SourceCell.Ref &&
			cell.Source.Face == tie.SourceCell.Face &&
			revolveloftfaces.SameMappedCell(cell.Source.Mapped, tie.SourceCell.Mapped)
	}
	return tie.VerticalCell != nil && cell.Vertical != nil &&
		GraphSameVerticalCell(*cell.Vertical, *tie.VerticalCell)
}

// A stored graph is never authority by itself. Before a later use it is
// compared with the graph built from a source certificate of the live bodies.
// Within one top-level call that certificate and graph are built once
// (LiveGraph) and shared by every later check.
func ContactGraphCurrent(ctx context.Context,
	g *ContactGraph) (bool, error) {
	if g == nil {
		return false, nil
	}
	_, fresh, ok, err := LiveGraph(ctx, g.Blank, g.Tooth)
	if err != nil || !ok || !SameShellManifest(g.Manifest, fresh.Manifest) ||
		len(g.Pairs) != len(fresh.Pairs) || len(g.Cells) != len(fresh.Cells) ||
		len(g.Arcs) != len(fresh.Arcs) || len(g.Runs) != len(fresh.Runs) {
		return false, err
	}
	for i, pair := range g.Pairs {
		other := fresh.Pairs[i]
		if pair.BlankFace != other.BlankFace || pair.TrimmedFace != other.TrimmedFace ||
			pair.SectionIndex != other.SectionIndex || pair.Verdict != other.Verdict ||
			pair.ArcID != other.ArcID || !slices.Equal(pair.CellIDs, other.CellIDs) {
			return false, nil
		}
	}
	for i, cell := range g.Cells {
		other := fresh.Cells[i]
		if cell.SectionIndex != other.SectionIndex || cell.SideIndex != other.SideIndex ||
			cell.CellIndex != other.CellIndex || cell.BlankFace != other.BlankFace ||
			cell.TrimmedFace != other.TrimmedFace || cell.PairID != other.PairID ||
			(cell.Source == nil) != (other.Source == nil) ||
			(cell.Vertical == nil) != (other.Vertical == nil) {
			return false, nil
		}
		if cell.Source != nil && (cell.Source.Ref != other.Source.Ref ||
			cell.Source.Face != other.Source.Face ||
			!revolveloftfaces.SameMappedCell(cell.Source.Mapped, other.Source.Mapped)) ||
			cell.Vertical != nil && !GraphSameVerticalCell(*cell.Vertical, *other.Vertical) {
			return false, nil
		}
	}
	for i, arc := range g.Arcs {
		other := fresh.Arcs[i]
		if arc.PairID != other.PairID || arc.SectionIndex != other.SectionIndex ||
			arc.EndpointIDs != other.EndpointIDs || arc.EndpointCellIDs != other.EndpointCellIDs ||
			!ToolCapOccupiedArcEqual(arc.Arc, other.Arc) {
			return false, nil
		}
	}
	for i, run := range g.Runs {
		other := fresh.Runs[i]
		if run.CellID != other.CellID || !slices.Equal(run.Stations, other.Stations) ||
			!slices.Equal(run.StationIDs, other.StationIDs) {
			return false, nil
		}
	}
	return true, nil
}

// A pair verdict is a source answer for one live blank and trimmed face.
// A candidate still needs a shared mesh, topology and volume proof.
type PairVerdict uint8

const (
	PairRefusal PairVerdict = iota
	PairMiss
	PairSourceCandidate
	PairToolArc
)

type FacePair struct {
	BlankFace, TrimmedFace *topo.Face
	SectionIndex           int
	Verdict                PairVerdict
	SourceCells            []SourceCell
	VerticalCells          []VerticalCell
	ToolArc                *ToolCapOccupiedArc
}

type PairCensus struct {
	Manifest *ContactManifest
	Pairs    []FacePair
	// ownArcs holds each occupied arc this census derived, with a private
	// deep copy taken at derivation. The copies are never handed out.
	OwnArcs []CensusArc
}

// CensusArc is an occupied arc this census derived and a
// private deep copy of it taken at derivation. The copy is never handed out.
type CensusArc struct {
	Arc     *ToolCapOccupiedArc
	Derived ToolCapOccupiedArc
}

// The census classifies every actual face pair from live producers. It does
// not admit Union: a source candidate is only a possible contact curve, and
// an occupied tool arc still needs exact common held vertices and bounds.
func Census(ctx context.Context,
	proof *SourceCertificate) (*PairCensus, bool, error) {
	if ctx == nil || proof == nil || proof.Blank == nil || proof.Tooth == nil {
		return nil, false, nil
	}
	ctx = revolveloftfaces.RevolveLoftFacesWithMeridianMemo(ctx)
	manifest, ok, err := AllSideManifest(ctx, proof)
	if err != nil || !ok {
		return nil, false, err
	}
	toolSides, ok := ToolCapSidesOf(proof)
	if !ok {
		return nil, false, nil
	}
	blankFaces, trimmedFaces := topobuild.Faces(proof.Blank), topobuild.Faces(proof.Tooth)
	if len(blankFaces) == 0 || len(trimmedFaces) == 0 ||
		len(blankFaces)*len(trimmedFaces) > 1<<16 {
		return nil, false, nil
	}
	payload, ok := proof.Tooth.Payload.(facetedrecord.Payload)
	if !ok || payload.LoftFacesTrim != proof.Trim ||
		!TrimCurrent(proof.Tooth, proof.Trim) {
		return nil, false, nil
	}
	wallFaces := make(map[*topo.Face]struct{})
	for _, band := range proof.SourceRefs {
		for _, ref := range band {
			index, valid := revolveloftfaces.RefFaceIndex(payload, ref, len(trimmedFaces))
			face, unique := UniqueOriginFace(proof.Tooth, ref)
			if !valid || !unique || (index >= 0) != (face != nil) ||
				index >= 0 && trimmedFaces[index] != face {
				return nil, false, nil
			}
			if face != nil {
				wallFaces[face] = struct{}{}
			}
		}
	}
	_, toeWall := wallFaces[proof.ToeFace]
	_, heelWall := wallFaces[proof.HeelFace]
	if toeWall || heelWall {
		return nil, false, nil
	}
	seenTrimmed := make(map[*topo.Face]struct{}, len(trimmedFaces))
	for _, face := range trimmedFaces {
		_, duplicate := seenTrimmed[face]
		_, wall := wallFaces[face]
		if face == nil || duplicate ||
			face != proof.ToeFace && face != proof.HeelFace && !wall {
			return nil, false, fmt.Errorf("%w: trimmed LoftFaces face has no unique live source role", decaderr.ErrUnsupported)
		}
		seenTrimmed[face] = struct{}{}
	}
	sectionOf := make(map[*topo.Face]int)
	for index, span := range proof.Section.Spans {
		if span.Face == nil {
			continue
		}
		if _, duplicate := sectionOf[span.Face]; duplicate {
			return nil, false, nil
		}
		sectionOf[span.Face] = index
	}
	if len(sectionOf) != len(blankFaces) {
		return nil, false, nil
	}
	seenBlank := make(map[*topo.Face]struct{}, len(blankFaces))
	for _, face := range blankFaces {
		_, duplicate := seenBlank[face]
		if face == nil || duplicate {
			return nil, false, nil
		}
		seenBlank[face] = struct{}{}
		if _, found := sectionOf[face]; !found {
			return nil, false, nil
		}
	}
	nonvertical := make(map[int]NonverticalSide)
	for _, side := range manifest.Nonvertical {
		nonvertical[side.SectionIndex] = side
	}
	vertical := make(map[int]VerticalEntry)
	for _, side := range manifest.Vertical {
		vertical[side.SectionIndex] = side
	}
	circles := make(map[[2]*topo.Face]ToolCapCircle)
	for _, circle := range toolSides.Circles {
		key := [2]*topo.Face{circle.BlankFace, circle.CapFace}
		if _, duplicate := circles[key]; duplicate {
			return nil, false, nil
		}
		circles[key] = circle
	}
	result := &PairCensus{Manifest: manifest,
		Pairs: make([]FacePair, 0, len(blankFaces)*len(trimmedFaces))}
	for _, blankFace := range blankFaces {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		sectionIndex, found := sectionOf[blankFace]
		if !found || sectionIndex >= len(manifest.SideOutcome) {
			return nil, false, nil
		}
		for _, trimmedFace := range trimmedFaces {
			pair := FacePair{BlankFace: blankFace,
				TrimmedFace: trimmedFace, SectionIndex: sectionIndex}
			if trimmedFace == proof.ToeFace || trimmedFace == proof.HeelFace {
				key := [2]*topo.Face{blankFace, trimmedFace}
				circle, possible := circles[key]
				_, separated := toolSides.Separated[key]
				if possible == separated {
					return nil, false, nil
				}
				if separated || circle.SourceMiss {
					pair.Verdict = PairMiss
				} else {
					if circle.Tool != 1 {
						return nil, false, nil
					}
					var arc *ToolCapOccupiedArc
					var certified bool
					var arcErr error
					span := proof.Section.Spans[circle.Span]
					if span.Start.U == span.End.U {
						arc, certified, arcErr = HeelVerticalOccupiedArc(ctx, proof, circle)
					} else {
						arc, certified, arcErr = HeelSlopedOccupiedArc(ctx, proof, circle)
					}
					if arcErr != nil || !certified || arc == nil {
						return nil, false, arcErr
					}
					pair.Verdict, pair.ToolArc = PairToolArc, arc
					result.OwnArcs = append(result.OwnArcs, CensusArc{
						Arc: arc, Derived: CloneToolCapOccupiedArc(arc)})
				}
			} else {
				switch manifest.SideOutcome[sectionIndex] {
				case SideMiss:
					pair.Verdict = PairMiss
				case SideSourceCandidate:
					if side, found := nonvertical[sectionIndex]; found {
						for _, cell := range side.Cells {
							if cell.Face != trimmedFace {
								continue
							}
							pair.SourceCells = append(pair.SourceCells, cell)
							switch cell.Mapped.Side {
							case loftface.SideCrossed, revolveloftfaces.RevolveLoftFacesJointContact:
								pair.Verdict = PairSourceCandidate
							case loftface.SideInside, loftface.SideOutside,
								revolveloftfaces.RevolveLoftFacesAxialMiss:
							default:
								return nil, false, nil
							}
						}
						if len(pair.SourceCells) == 0 {
							return nil, false, nil
						}
					} else if side, found := vertical[sectionIndex]; found && side.Side != nil {
						for _, cell := range side.Side.Cells {
							if cell.Face == trimmedFace && len(cell.Retained) > 0 {
								pair.VerticalCells = append(pair.VerticalCells, cell)
							}
						}
						if len(pair.VerticalCells) > 0 {
							pair.Verdict = PairSourceCandidate
						}
					} else {
						return nil, false, nil
					}
					if pair.Verdict == PairRefusal {
						pair.Verdict = PairMiss
					}
				default:
					return nil, false, nil
				}
			}
			result.Pairs = append(result.Pairs, pair)
		}
	}
	return result, len(result.Pairs) == len(blankFaces)*len(trimmedFaces), nil
}

// CertifiedArcPatches is the source-to-held gate for one
// complete radial Sketch arc. It compares the caller's root bracket and
// station run with the live source certificate and contact graph of the
// bodies, built once per top-level call, before using them. The returned
// pieces remain open and cannot be used as a Boolean result by themselves.
func CertifiedArcPatches(ctx context.Context, blank, tooth *topo.Body,
	sectionIndex, segment, band int, root loftface.Interval, stations []int,
) ([]revolveloftfaces.ArcSeamVertex, revolveloftfaces.ArcSeamPatch,
	revolveloftfaces.ArcSeamPatch, bool, error) {
	var empty revolveloftfaces.ArcSeamPatch
	if ctx == nil || root.Lo == nil || root.Hi == nil ||
		sectionIndex < 0 || segment < 0 || band < 0 {
		return nil, empty, empty, false, nil
	}
	proof, graph, ok, err := LiveGraph(ctx, blank, tooth)
	if err != nil || !ok || sectionIndex >= len(proof.Section.Spans) {
		return nil, empty, empty, false, err
	}
	section := proof.Section.Spans[sectionIndex]
	if section.Face == nil || section.Start.U == section.End.U {
		return nil, empty, empty, false, nil
	}
	var selected *GraphRun
	for i := range graph.Runs {
		run := &graph.Runs[i]
		cell := graph.Cells[run.CellID]
		if cell.SectionIndex != sectionIndex || cell.BlankFace != section.Face ||
			cell.PairID < 0 || cell.Source == nil ||
			graph.Pairs[cell.PairID].Verdict != PairSourceCandidate {
			continue
		}
		mapped := cell.Source.Mapped
		if mapped.Segment != segment || mapped.Band != band ||
			mapped.JointArc == nil || mapped.TLo == nil || mapped.THi == nil ||
			mapped.TLo.Cmp(root.Lo) != 0 || mapped.THi.Cmp(root.Hi) != 0 {
			continue
		}
		if selected != nil {
			return nil, empty, empty, false, nil
		}
		selected = run
	}
	if selected == nil || !slices.Equal(stations, selected.Stations) {
		return nil, empty, empty, false, nil
	}
	seam, ok, err := revolveloftfaces.ArcSeamVerticesUnchecked(ctx, proof.Trim,
		band, selected.Stations, root, section)
	if err != nil || !ok {
		return nil, empty, empty, false, err
	}
	blankPatch, loftPatch, ok, err := revolveloftfaces.ArcPatchesUnchecked(ctx, proof.Trim,
		band, segment, section, seam)
	if err != nil || !ok {
		return nil, empty, empty, false, err
	}
	if !graph.checkOpenPatches(selected, seam, &blankPatch, &loftPatch) {
		return nil, empty, empty, false, nil
	}
	return seam, blankPatch, loftPatch, true, nil
}

type CallScopeKey struct{}

// A call scope holds the records that one top-level consumer call has
// already derived from the live bodies. It lives only in that call's
// context and is never stored anywhere else. Only successes are recorded.
type CallScope struct {
	Mu       sync.Mutex
	Live     map[[2]*topo.Body]ScopedGraph
	Censuses map[*SourceCertificate]*PairCensus
}

type ScopedGraph struct {
	Proof *SourceCertificate
	Graph *ContactGraph
}

// WithCallScope opens a call scope for the outermost
// top-level consumer. A context that already carries one is returned
// unchanged, so nested consumers share the outer scope.
func WithCallScope(ctx context.Context) context.Context {
	if ctx == nil || CallScopeOf(ctx) != nil {
		return ctx
	}
	return context.WithValue(ctx, CallScopeKey{}, &CallScope{
		Live:     make(map[[2]*topo.Body]ScopedGraph),
		Censuses: make(map[*SourceCertificate]*PairCensus),
	})
}

func CallScopeOf(ctx context.Context) *CallScope {
	if ctx == nil {
		return nil
	}
	scope, _ := ctx.Value(CallScopeKey{}).(*CallScope)
	return scope
}

// LiveGraph certifies blank and tooth and builds the contact
// graph of that certificate. Inside a call scope the first success is kept
// and returned to every later check of the same pair. The result is only for
// reading: a caller must never hold or edit a scoped graph, whose cells point
// into its census.
func LiveGraph(ctx context.Context, blank, tooth *topo.Body) (
	*SourceCertificate, *ContactGraph, bool, error) {
	scope := CallScopeOf(ctx)
	key := [2]*topo.Body{blank, tooth}
	if scope != nil {
		scope.Mu.Lock()
		hit, found := scope.Live[key]
		scope.Mu.Unlock()
		if found {
			return hit.Proof, hit.Graph, true, nil
		}
	}
	proof, ok, err := CertifySource(ctx, blank, tooth)
	if err != nil || !ok {
		return nil, nil, false, err
	}
	graph, ok, err := ContactGraphOf(ctx, proof)
	if err != nil || !ok {
		return proof, nil, false, err
	}
	if scope != nil {
		scope.Mu.Lock()
		scope.Live[key] = ScopedGraph{Proof: proof, Graph: graph}
		scope.Mu.Unlock()
	}
	return proof, graph, true, nil
}

// CensusIn derives the census of proof, once per source
// certificate inside a call scope.
func CensusIn(ctx context.Context,
	proof *SourceCertificate) (*PairCensus, bool, error) {
	scope := CallScopeOf(ctx)
	if scope != nil && proof != nil {
		scope.Mu.Lock()
		hit, found := scope.Censuses[proof]
		scope.Mu.Unlock()
		if found {
			return hit, true, nil
		}
	}
	census, ok, err := Census(ctx, proof)
	if err == nil && ok && scope != nil && proof != nil {
		scope.Mu.Lock()
		scope.Censuses[proof] = census
		scope.Mu.Unlock()
	}
	return census, ok, err
}
