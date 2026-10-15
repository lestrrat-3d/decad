package decad

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/r3"
)

// A contact graph assigns source-owned identities before either open patch or
// closed blank-shell construction. Its IDs describe source events, not held
// coordinates and not a completed Boolean seam.
type revolveLoftFacesContactGraph struct {
	blank, tooth *Body
	manifest     *revolveLoftFacesContactManifest
	pairs        []revolveLoftFacesGraphPair
	cells        []revolveLoftFacesGraphCell
	arcs         []revolveLoftFacesGraphArc
	runs         []revolveLoftFacesGraphRun
}

type revolveLoftFacesGraphPair struct {
	blankFace, trimmedFace *Face
	sectionIndex           int
	verdict                revolveLoftFacesPairVerdict
	cellIDs                []int
	arcID                  int
}

type revolveLoftFacesGraphCell struct {
	sectionIndex, sideIndex, cellIndex int
	blankFace, trimmedFace             *Face
	source                             *revolveLoftFacesSourceCell
	vertical                           *revolveLoftFacesVerticalCell
	pairID                             int // -1 for a source cell with no retained wall
}

type revolveLoftFacesGraphArc struct {
	pairID, sectionIndex int
	arc                  *revolveLoftFacesToolCapOccupiedArc
	endpointIDs          [2]int
	endpointCellIDs      [2]int
}

type revolveLoftFacesGraphRun struct {
	cellID     int
	stations   []int
	stationIDs []int
}

func revolveLoftFacesGraphSameVerticalCell(a, b revolveLoftFacesVerticalCell) bool {
	if a.segment != b.segment || a.span != b.span || a.band != b.band ||
		a.ref != b.ref || a.face != b.face ||
		!revolveLoftFacesSameBezier(a.source, b.source) ||
		!revolveLoftFacesSameVerticalLimit(a.lower, b.lower) ||
		!revolveLoftFacesSameVerticalLimit(a.upper, b.upper) ||
		len(a.retained) != len(b.retained) {
		return false
	}
	for i := range a.retained {
		if !revolveLoftFacesSameInterval(a.retained[i].Source, b.retained[i].Source) ||
			a.retained[i].LoOpen != b.retained[i].LoOpen ||
			a.retained[i].HiOpen != b.retained[i].HiOpen ||
			!revolveLoftFacesSameBox(a.retained[i].Box, b.retained[i].Box) {
			return false
		}
	}
	return true
}

func (g *revolveLoftFacesContactGraph) sourceCellID(section int, blank *Face,
	cell revolveLoftFacesSourceCell) (int, bool) {
	id := -1
	for i := range g.cells {
		entry := &g.cells[i]
		if entry.source == nil || entry.sectionIndex != section ||
			entry.blankFace != blank || entry.trimmedFace != cell.face ||
			entry.source.ref != cell.ref || entry.source.face != cell.face ||
			!revolveLoftFacesSameMappedCell(entry.source.mapped, cell.mapped) {
			continue
		}
		if id >= 0 {
			return -1, false
		}
		id = i
	}
	return id, id >= 0
}

func (g *revolveLoftFacesContactGraph) verticalCellID(section int, blank *Face,
	cell revolveLoftFacesVerticalCell) (int, bool) {
	id := -1
	for i := range g.cells {
		entry := &g.cells[i]
		if entry.vertical == nil || entry.sectionIndex != section ||
			entry.blankFace != blank || entry.trimmedFace != cell.face ||
			!revolveLoftFacesGraphSameVerticalCell(*entry.vertical, cell) {
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
func revolveLoftFacesContactGraphOf(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate) (*revolveLoftFacesContactGraph, bool, error) {
	census, ok, err := revolveLoftFacesCensusIn(ctx, proof)
	if err != nil || !ok || census == nil || census.manifest == nil || proof == nil {
		return nil, false, err
	}
	blankFaces, trimmedFaces := proof.blank.Faces(), proof.tooth.Faces()
	if len(census.pairs) != len(blankFaces)*len(trimmedFaces) {
		return nil, false, nil
	}
	g := &revolveLoftFacesContactGraph{blank: proof.blank, tooth: proof.tooth,
		manifest: census.manifest}
	nextEventID := 0
	for sideIndex, side := range census.manifest.nonvertical {
		for cellIndex := range side.cells {
			cell := side.cells[cellIndex]
			if side.sectionIndex < 0 || side.sectionIndex >= len(proof.section.spans) ||
				side.face != proof.section.spans[side.sectionIndex].face || cell.face == nil {
				return nil, false, nil
			}
			g.cells = append(g.cells, revolveLoftFacesGraphCell{
				sectionIndex: side.sectionIndex, sideIndex: sideIndex, cellIndex: cellIndex,
				blankFace: side.face, trimmedFace: cell.face, source: &side.cells[cellIndex], pairID: -1})
		}
	}
	for sideIndex, side := range census.manifest.vertical {
		if side.side == nil {
			continue
		}
		for cellIndex := range side.side.cells {
			cell := side.side.cells[cellIndex]
			if side.sectionIndex < 0 || side.sectionIndex >= len(proof.section.spans) ||
				side.side.face != proof.section.spans[side.sectionIndex].face ||
				(cell.face == nil) != (len(cell.retained) == 0) {
				return nil, false, nil
			}
			g.cells = append(g.cells, revolveLoftFacesGraphCell{
				sectionIndex: side.sectionIndex, sideIndex: sideIndex, cellIndex: cellIndex,
				blankFace: side.side.face, trimmedFace: cell.face,
				vertical: &side.side.cells[cellIndex], pairID: -1})
		}
	}
	seenPairs := make(map[[2]*Face]bool, len(census.pairs))
	for _, pair := range census.pairs {
		key := [2]*Face{pair.blankFace, pair.trimmedFace}
		if seenPairs[key] || pair.sectionIndex < 0 ||
			pair.sectionIndex >= len(proof.section.spans) ||
			proof.section.spans[pair.sectionIndex].face != pair.blankFace ||
			!slices.Contains(blankFaces, pair.blankFace) ||
			!slices.Contains(trimmedFaces, pair.trimmedFace) {
			return nil, false, nil
		}
		seenPairs[key] = true
		entry := revolveLoftFacesGraphPair{blankFace: pair.blankFace,
			trimmedFace: pair.trimmedFace, sectionIndex: pair.sectionIndex,
			verdict: pair.verdict, arcID: -1}
		pairID := len(g.pairs)
		for _, cell := range pair.sourceCells {
			if pair.verdict == revolveLoftFacesPairMiss &&
				(cell.mapped.side == revolveLoftFacesJointContact ||
					cell.mapped.side == loftface.SideCrossed) {
				return nil, false, nil
			}
			id, found := g.sourceCellID(pair.sectionIndex, pair.blankFace, cell)
			if !found || g.cells[id].pairID >= 0 {
				return nil, false, nil
			}
			g.cells[id].pairID = pairID
			entry.cellIDs = append(entry.cellIDs, id)
		}
		for _, cell := range pair.verticalCells {
			id, found := g.verticalCellID(pair.sectionIndex, pair.blankFace, cell)
			if !found || g.cells[id].pairID >= 0 {
				return nil, false, nil
			}
			g.cells[id].pairID = pairID
			entry.cellIDs = append(entry.cellIDs, id)
		}
		if pair.verdict == revolveLoftFacesPairToolArc {
			if pair.toolArc == nil || len(entry.cellIDs) != 0 {
				return nil, false, nil
			}
			entry.arcID = len(g.arcs)
			g.arcs = append(g.arcs, revolveLoftFacesGraphArc{
				pairID: pairID, sectionIndex: pair.sectionIndex, arc: pair.toolArc,
				endpointIDs: [2]int{nextEventID, nextEventID + 1}})
			nextEventID += 2
		} else if pair.toolArc != nil || pair.verdict == revolveLoftFacesPairRefusal ||
			pair.verdict == revolveLoftFacesPairSourceCandidate && len(entry.cellIDs) == 0 {
			return nil, false, nil
		}
		g.pairs = append(g.pairs, entry)
	}
	if len(seenPairs) != len(blankFaces)*len(trimmedFaces) {
		return nil, false, nil
	}
	for id, cell := range g.cells {
		if cell.trimmedFace != nil && cell.pairID < 0 ||
			cell.trimmedFace == nil && cell.pairID >= 0 {
			return nil, false, nil
		}
		if cell.source == nil || cell.pairID < 0 ||
			g.pairs[cell.pairID].verdict != revolveLoftFacesPairSourceCandidate ||
			cell.source.mapped.side != revolveLoftFacesJointContact ||
			cell.source.mapped.jointKind != revolveLoftFacesWholeArcContact {
			continue
		}
		stations, complete := revolveLoftFacesCompleteArcRun(proof.trim, cell.source.mapped.segment)
		if !complete {
			return nil, false, nil
		}
		run := revolveLoftFacesGraphRun{cellID: id, stations: stations}
		for range stations {
			run.stationIDs = append(run.stationIDs, nextEventID)
			nextEventID++
		}
		g.runs = append(g.runs, run)
	}
	for i := range g.arcs {
		arc := &g.arcs[i]
		ties, tied, tieErr := revolveLoftFacesHeelArcSideTiesIn(ctx, proof, census, arc.arc)
		if tieErr != nil || !tied {
			return nil, false, tieErr
		}
		for endpoint, tie := range [...]revolveLoftFacesCapSideTie{ties.negativeZ, ties.positiveZ} {
			id, found := -1, false
			if tie.sourceCell != nil {
				id, found = g.sourceCellID(arc.sectionIndex, arc.arc.circle.blankFace, *tie.sourceCell)
			} else if tie.verticalCell != nil {
				id, found = g.verticalCellID(arc.sectionIndex, arc.arc.circle.blankFace, *tie.verticalCell)
			}
			if !found || g.cells[id].pairID < 0 ||
				g.cells[id].trimmedFace != tie.boundary.face {
				return nil, false, nil
			}
			arc.endpointCellIDs[endpoint] = id
		}
		if arc.endpointCellIDs[0] == arc.endpointCellIDs[1] {
			return nil, false, nil
		}
	}
	return g, true, nil
}

// bindOpenPatches attaches source station IDs only after checking the actual
// held seam vertices and their separate side bounds.
func (g *revolveLoftFacesContactGraph) bindOpenPatches(run *revolveLoftFacesGraphRun,
	seam []revolveLoftFacesSeamVertex, blank, loft *revolveLoftFacesSeamPatch) bool {
	if g == nil || run == nil || blank == nil || loft == nil ||
		len(run.stationIDs) != len(seam) || len(run.stations) != len(seam) ||
		len(blank.seam) != len(seam) || len(loft.seam) != len(seam) {
		return false
	}
	seen := make(map[int]r3.Vec, len(seam))
	for i, stationID := range run.stationIDs {
		bi, li := blank.seam[i], loft.seam[i]
		if seam[i].station != run.stations[i] ||
			bi < 0 || bi >= len(blank.vertices) ||
			li < 0 || li >= len(loft.vertices) ||
			bi >= len(blank.vertexBounds) || li >= len(loft.vertexBounds) ||
			blank.vertices[bi] != seam[i].point || loft.vertices[li] != seam[i].point ||
			blank.vertexBounds[bi] < seam[i].blankBound ||
			loft.vertexBounds[li] < seam[i].loftBound {
			return false
		}
		if _, duplicate := seen[stationID]; duplicate {
			return false
		}
		seen[stationID] = seam[i].point
	}
	blank.stationIDs = slices.Clone(run.stationIDs)
	loft.stationIDs = slices.Clone(run.stationIDs)
	return true
}

// bindBlankShell connects every graph ID to a produced shell index and to
// the independently built open patches. Equal source IDs must use identical
// held coordinates, while each side keeps its own certified bound.
func (g *revolveLoftFacesContactGraph) bindBlankShell(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, run *revolveLoftFacesGraphRun,
	shell *revolveLoftFacesBlankSeamShell) (bool, error) {
	if g == nil || proof == nil || run == nil || shell == nil ||
		run.cellID < 0 || run.cellID >= len(g.cells) ||
		len(shell.seam) != len(run.stationIDs) {
		return false, nil
	}
	cell := g.cells[run.cellID]
	if cell.source == nil || cell.pairID < 0 ||
		g.pairs[cell.pairID].verdict != revolveLoftFacesPairSourceCandidate {
		return false, nil
	}
	mapped := cell.source.mapped
	seam, blankPatch, loftPatch, ok, err := revolveLoftFacesCertifiedArcPatches(ctx,
		proof.blank, proof.tooth, cell.sectionIndex, mapped.segment, mapped.band,
		loftFaceInterval{Lo: mapped.tLo, Hi: mapped.tHi}, run.stations)
	if err != nil || !ok ||
		!g.bindOpenPatches(run, seam, &blankPatch, &loftPatch) {
		return false, err
	}
	seenStations, seenVertices := make(map[int]bool), make(map[int]bool)
	for i, stationID := range run.stationIDs {
		index := shell.seam[i]
		if index < 0 || index >= len(shell.mesh.Verts) ||
			index >= len(shell.vertexBounds) || seenStations[stationID] ||
			seenVertices[index] ||
			shell.mesh.Verts[index] != seam[i].point ||
			blankPatch.vertices[blankPatch.seam[i]] != shell.mesh.Verts[index] ||
			loftPatch.vertices[loftPatch.seam[i]] != shell.mesh.Verts[index] ||
			shell.vertexBounds[index] < seam[i].blankBound {
			return false, nil
		}
		seenStations[stationID], seenVertices[index] = true, true
	}
	shell.seamIDs = slices.Clone(run.stationIDs)
	seenArcs, seenEndpoints := make(map[int]bool), make(map[int]bool)
	for _, event := range g.arcs {
		matched := -1
		for i, ring := range shell.capRings {
			if revolveLoftFacesToolCapOccupiedArcEqual(event.arc, ring.arc) {
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
		ring := &shell.capRings[matched]
		if ring.sectionIndex != event.sectionIndex || ring.ties == nil ||
			event.endpointIDs[0] == event.endpointIDs[1] ||
			event.endpointCellIDs[0] == event.endpointCellIDs[1] {
			return false, nil
		}
		for endpoint, cellID := range event.endpointCellIDs {
			index := ring.endpoints[endpoint]
			if cellID < 0 || cellID >= len(g.cells) ||
				index < 0 || index >= len(shell.mesh.Verts) ||
				index >= len(shell.vertexBounds) ||
				seenEndpoints[index] ||
				shell.mesh.Verts[index] != ring.endpointProof[endpoint].point ||
				shell.vertexBounds[index] < ring.endpointProof[endpoint].blankBound ||
				!g.endpointCellMatches(event, ring, endpoint, cellID) {
				return false, nil
			}
			seenEndpoints[index] = true
		}
		ring.endpointGraphIDs = event.endpointIDs
		ring.endpointCellIDs = event.endpointCellIDs
	}
	return len(seenArcs) == len(shell.capRings), nil
}

func (g *revolveLoftFacesContactGraph) endpointCellMatches(event revolveLoftFacesGraphArc,
	ring *revolveLoftFacesShellCapRing, endpoint, cellID int) bool {
	cell := g.cells[cellID]
	if cell.pairID < 0 || cell.sectionIndex != event.sectionIndex ||
		cell.blankFace != event.arc.circle.blankFace {
		return false
	}
	tie := ring.ties.negativeZ
	boundary := event.arc.negativeZ
	if endpoint == 1 {
		tie, boundary = ring.ties.positiveZ, event.arc.positiveZ
	}
	if !revolveLoftFacesToolCapBoundaryEqual(tie.boundary, boundary) ||
		cell.trimmedFace != boundary.face {
		return false
	}
	if tie.sourceCell != nil {
		return cell.source != nil && cell.source.ref == tie.sourceCell.ref &&
			cell.source.face == tie.sourceCell.face &&
			revolveLoftFacesSameMappedCell(cell.source.mapped, tie.sourceCell.mapped)
	}
	return tie.verticalCell != nil && cell.vertical != nil &&
		revolveLoftFacesGraphSameVerticalCell(*cell.vertical, *tie.verticalCell)
}

// A stored graph is never authority by itself. Before a later use it is
// compared with the graph built from a source certificate of the live bodies.
// Within one top-level call that certificate and graph are built once
// (revolveLoftFacesLiveGraph) and shared by every later check.
func revolveLoftFacesContactGraphCurrent(ctx context.Context,
	g *revolveLoftFacesContactGraph) (bool, error) {
	if g == nil {
		return false, nil
	}
	_, fresh, ok, err := revolveLoftFacesLiveGraph(ctx, g.blank, g.tooth)
	if err != nil || !ok || !revolveLoftFacesSameShellManifest(g.manifest, fresh.manifest) ||
		len(g.pairs) != len(fresh.pairs) || len(g.cells) != len(fresh.cells) ||
		len(g.arcs) != len(fresh.arcs) || len(g.runs) != len(fresh.runs) {
		return false, err
	}
	for i, pair := range g.pairs {
		other := fresh.pairs[i]
		if pair.blankFace != other.blankFace || pair.trimmedFace != other.trimmedFace ||
			pair.sectionIndex != other.sectionIndex || pair.verdict != other.verdict ||
			pair.arcID != other.arcID || !slices.Equal(pair.cellIDs, other.cellIDs) {
			return false, nil
		}
	}
	for i, cell := range g.cells {
		other := fresh.cells[i]
		if cell.sectionIndex != other.sectionIndex || cell.sideIndex != other.sideIndex ||
			cell.cellIndex != other.cellIndex || cell.blankFace != other.blankFace ||
			cell.trimmedFace != other.trimmedFace || cell.pairID != other.pairID ||
			(cell.source == nil) != (other.source == nil) ||
			(cell.vertical == nil) != (other.vertical == nil) {
			return false, nil
		}
		if cell.source != nil && (cell.source.ref != other.source.ref ||
			cell.source.face != other.source.face ||
			!revolveLoftFacesSameMappedCell(cell.source.mapped, other.source.mapped)) ||
			cell.vertical != nil && !revolveLoftFacesGraphSameVerticalCell(*cell.vertical, *other.vertical) {
			return false, nil
		}
	}
	for i, arc := range g.arcs {
		other := fresh.arcs[i]
		if arc.pairID != other.pairID || arc.sectionIndex != other.sectionIndex ||
			arc.endpointIDs != other.endpointIDs || arc.endpointCellIDs != other.endpointCellIDs ||
			!revolveLoftFacesToolCapOccupiedArcEqual(arc.arc, other.arc) {
			return false, nil
		}
	}
	for i, run := range g.runs {
		other := fresh.runs[i]
		if run.cellID != other.cellID || !slices.Equal(run.stations, other.stations) ||
			!slices.Equal(run.stationIDs, other.stationIDs) {
			return false, nil
		}
	}
	return true, nil
}
