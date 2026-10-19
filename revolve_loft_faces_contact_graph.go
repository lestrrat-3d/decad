package decad

import (
	"context"
	"fmt"
	"slices"
	"sync"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/topo"
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
		!revolveloftfaces.RevolveLoftFacesSameBezier(a.source, b.source) ||
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
			if side.sectionIndex < 0 || side.sectionIndex >= len(proof.section.Spans) ||
				faceNode(side.face) != proof.section.Spans[side.sectionIndex].Face || cell.face == nil {
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
			if side.sectionIndex < 0 || side.sectionIndex >= len(proof.section.Spans) ||
				faceNode(side.side.face) != proof.section.Spans[side.sectionIndex].Face ||
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
			pair.sectionIndex >= len(proof.section.Spans) ||
			proof.section.Spans[pair.sectionIndex].Face != faceNode(pair.blankFace) ||
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
				(cell.mapped.Side == revolveloftfaces.RevolveLoftFacesJointContact ||
					cell.mapped.Side == loftface.SideCrossed) {
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
			cell.source.mapped.Side != revolveloftfaces.RevolveLoftFacesJointContact ||
			cell.source.mapped.JointKind != revolveloftfaces.RevolveLoftFacesWholeArcContact {
			continue
		}
		stations, complete := revolveLoftFacesCompleteArcRun(proof.trim, cell.source.mapped.Segment)
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

// checkOpenPatches checks the actual held seam vertices of both open patches
// against the run's stations, distinct station IDs and separate side bounds.
func (g *revolveLoftFacesContactGraph) checkOpenPatches(run *revolveLoftFacesGraphRun,
	seam []revolveloftfaces.ArcSeamVertex, blank, loft *revolveloftfaces.ArcSeamPatch) bool {
	if g == nil || run == nil || blank == nil || loft == nil ||
		len(run.stationIDs) != len(seam) || len(run.stations) != len(seam) ||
		len(blank.Seam) != len(seam) || len(loft.Seam) != len(seam) {
		return false
	}
	seen := make(map[int]r3.Vec, len(seam))
	for i, stationID := range run.stationIDs {
		bi, li := blank.Seam[i], loft.Seam[i]
		if seam[i].Station != run.stations[i] ||
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
		proof.blank, proof.tooth, cell.sectionIndex, mapped.Segment, mapped.Band,
		loftFaceInterval{Lo: mapped.TLo, Hi: mapped.THi}, run.stations)
	if err != nil || !ok ||
		!g.checkOpenPatches(run, seam, &blankPatch, &loftPatch) {
		return false, err
	}
	seenStations, seenVertices := make(map[int]bool), make(map[int]bool)
	for i, stationID := range run.stationIDs {
		index := shell.seam[i]
		if index < 0 || index >= len(shell.mesh.Verts) ||
			index >= len(shell.vertexBounds) || seenStations[stationID] ||
			seenVertices[index] ||
			shell.mesh.Verts[index] != seam[i].Point ||
			blankPatch.Vertices[blankPatch.Seam[i]] != shell.mesh.Verts[index] ||
			loftPatch.Vertices[loftPatch.Seam[i]] != shell.mesh.Verts[index] ||
			shell.vertexBounds[index] < seam[i].BlankBound {
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

// A pair verdict is a source answer for one live blank and trimmed face.
// A candidate still needs a shared mesh, topology and volume proof.
type revolveLoftFacesPairVerdict uint8

const (
	revolveLoftFacesPairRefusal revolveLoftFacesPairVerdict = iota
	revolveLoftFacesPairMiss
	revolveLoftFacesPairSourceCandidate
	revolveLoftFacesPairToolArc
)

type revolveLoftFacesFacePair struct {
	blankFace, trimmedFace *Face
	sectionIndex           int
	verdict                revolveLoftFacesPairVerdict
	sourceCells            []revolveLoftFacesSourceCell
	verticalCells          []revolveLoftFacesVerticalCell
	toolArc                *revolveLoftFacesToolCapOccupiedArc
}

type revolveLoftFacesPairCensus struct {
	manifest *revolveLoftFacesContactManifest
	pairs    []revolveLoftFacesFacePair
	// ownArcs holds each occupied arc this census derived, with a private
	// deep copy taken at derivation. The copies are never handed out.
	ownArcs []revolveLoftFacesCensusArc
}

// revolveLoftFacesCensusArc is an occupied arc this census derived and a
// private deep copy of it taken at derivation. The copy is never handed out.
type revolveLoftFacesCensusArc struct {
	arc     *revolveLoftFacesToolCapOccupiedArc
	derived revolveLoftFacesToolCapOccupiedArc
}

// The census classifies every actual face pair from live producers. It does
// not admit Union: a source candidate is only a possible contact curve, and
// an occupied tool arc still needs exact common held vertices and bounds.
func revolveLoftFacesCensus(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate) (*revolveLoftFacesPairCensus, bool, error) {
	if ctx == nil || proof == nil || proof.blank == nil || proof.tooth == nil {
		return nil, false, nil
	}
	ctx = revolveloftfaces.RevolveLoftFacesWithMeridianMemo(ctx)
	manifest, ok, err := revolveLoftFacesAllSideManifest(ctx, proof)
	if err != nil || !ok {
		return nil, false, err
	}
	toolSides, ok := revolveLoftFacesToolCapSidesOf(proof)
	if !ok {
		return nil, false, nil
	}
	blankFaces, trimmedFaces := proof.blank.Faces(), proof.tooth.Faces()
	if len(blankFaces) == 0 || len(trimmedFaces) == 0 ||
		len(blankFaces)*len(trimmedFaces) > 1<<16 {
		return nil, false, nil
	}
	payload, ok := proof.tooth.n.Payload.(facetedPayload)
	if !ok || payload.LoftFacesTrim != proof.trim ||
		!revolveLoftFacesTrimCurrent(proof.tooth, proof.trim) {
		return nil, false, nil
	}
	wallFaces := make(map[*Face]struct{})
	for _, band := range proof.sourceRefs {
		for _, ref := range band {
			index, valid := revolveLoftFacesRefFaceIndex(payload, ref, len(trimmedFaces))
			face, unique := revolveLoftFacesUniqueOriginFace(proof.tooth, ref)
			if !valid || !unique || (index >= 0) != (face != nil) ||
				index >= 0 && trimmedFaces[index] != face {
				return nil, false, nil
			}
			if face != nil {
				wallFaces[face] = struct{}{}
			}
		}
	}
	_, toeWall := wallFaces[proof.toeFace]
	_, heelWall := wallFaces[proof.heelFace]
	if toeWall || heelWall {
		return nil, false, nil
	}
	seenTrimmed := make(map[*Face]struct{}, len(trimmedFaces))
	for _, face := range trimmedFaces {
		_, duplicate := seenTrimmed[face]
		_, wall := wallFaces[face]
		if face == nil || duplicate ||
			face != proof.toeFace && face != proof.heelFace && !wall {
			return nil, false, fmt.Errorf("%w: trimmed LoftFaces face has no unique live source role", ErrUnsupported)
		}
		seenTrimmed[face] = struct{}{}
	}
	sectionOf := make(map[*topo.Face]int)
	for index, span := range proof.section.Spans {
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
	seenBlank := make(map[*Face]struct{}, len(blankFaces))
	for _, face := range blankFaces {
		_, duplicate := seenBlank[face]
		if face == nil || duplicate {
			return nil, false, nil
		}
		seenBlank[face] = struct{}{}
		if _, found := sectionOf[faceNode(face)]; !found {
			return nil, false, nil
		}
	}
	nonvertical := make(map[int]revolveLoftFacesNonverticalSide)
	for _, side := range manifest.nonvertical {
		nonvertical[side.sectionIndex] = side
	}
	vertical := make(map[int]revolveLoftFacesVerticalEntry)
	for _, side := range manifest.vertical {
		vertical[side.sectionIndex] = side
	}
	circles := make(map[[2]*Face]revolveLoftFacesToolCapCircle)
	for _, circle := range toolSides.circles {
		key := [2]*Face{circle.blankFace, circle.capFace}
		if _, duplicate := circles[key]; duplicate {
			return nil, false, nil
		}
		circles[key] = circle
	}
	result := &revolveLoftFacesPairCensus{manifest: manifest,
		pairs: make([]revolveLoftFacesFacePair, 0, len(blankFaces)*len(trimmedFaces))}
	for _, blankFace := range blankFaces {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		sectionIndex, found := sectionOf[faceNode(blankFace)]
		if !found || sectionIndex >= len(manifest.sideOutcome) {
			return nil, false, nil
		}
		for _, trimmedFace := range trimmedFaces {
			pair := revolveLoftFacesFacePair{blankFace: blankFace,
				trimmedFace: trimmedFace, sectionIndex: sectionIndex}
			if trimmedFace == proof.toeFace || trimmedFace == proof.heelFace {
				key := [2]*Face{blankFace, trimmedFace}
				circle, possible := circles[key]
				_, separated := toolSides.separated[key]
				if possible == separated {
					return nil, false, nil
				}
				if separated || circle.sourceMiss {
					pair.verdict = revolveLoftFacesPairMiss
				} else {
					if circle.tool != 1 {
						return nil, false, nil
					}
					var arc *revolveLoftFacesToolCapOccupiedArc
					var certified bool
					var arcErr error
					span := proof.section.Spans[circle.span]
					if span.Start.U == span.End.U {
						arc, certified, arcErr = revolveLoftFacesHeelVerticalOccupiedArc(ctx, proof, circle)
					} else {
						arc, certified, arcErr = revolveLoftFacesHeelSlopedOccupiedArc(ctx, proof, circle)
					}
					if arcErr != nil || !certified || arc == nil {
						return nil, false, arcErr
					}
					pair.verdict, pair.toolArc = revolveLoftFacesPairToolArc, arc
					result.ownArcs = append(result.ownArcs, revolveLoftFacesCensusArc{
						arc: arc, derived: revolveLoftFacesCloneToolCapOccupiedArc(arc)})
				}
			} else {
				switch manifest.sideOutcome[sectionIndex] {
				case revolveLoftFacesSideMiss:
					pair.verdict = revolveLoftFacesPairMiss
				case revolveLoftFacesSideSourceCandidate:
					if side, found := nonvertical[sectionIndex]; found {
						for _, cell := range side.cells {
							if cell.face != trimmedFace {
								continue
							}
							pair.sourceCells = append(pair.sourceCells, cell)
							switch cell.mapped.Side {
							case loftface.SideCrossed, revolveloftfaces.RevolveLoftFacesJointContact:
								pair.verdict = revolveLoftFacesPairSourceCandidate
							case loftface.SideInside, loftface.SideOutside,
								revolveloftfaces.RevolveLoftFacesAxialMiss:
							default:
								return nil, false, nil
							}
						}
						if len(pair.sourceCells) == 0 {
							return nil, false, nil
						}
					} else if side, found := vertical[sectionIndex]; found && side.side != nil {
						for _, cell := range side.side.cells {
							if cell.face == trimmedFace && len(cell.retained) > 0 {
								pair.verticalCells = append(pair.verticalCells, cell)
							}
						}
						if len(pair.verticalCells) > 0 {
							pair.verdict = revolveLoftFacesPairSourceCandidate
						}
					} else {
						return nil, false, nil
					}
					if pair.verdict == revolveLoftFacesPairRefusal {
						pair.verdict = revolveLoftFacesPairMiss
					}
				default:
					return nil, false, nil
				}
			}
			result.pairs = append(result.pairs, pair)
		}
	}
	return result, len(result.pairs) == len(blankFaces)*len(trimmedFaces), nil
}

// revolveLoftFacesCertifiedArcPatches is the source-to-held gate for one
// complete radial Sketch arc. It compares the caller's root bracket and
// station run with the live source certificate and contact graph of the
// bodies, built once per top-level call, before using them. The returned
// pieces remain open and cannot be used as a Boolean result by themselves.
func revolveLoftFacesCertifiedArcPatches(ctx context.Context, blank, tooth *Body,
	sectionIndex, segment, band int, root loftFaceInterval, stations []int,
) ([]revolveloftfaces.ArcSeamVertex, revolveloftfaces.ArcSeamPatch,
	revolveloftfaces.ArcSeamPatch, bool, error) {
	var empty revolveloftfaces.ArcSeamPatch
	if ctx == nil || root.Lo == nil || root.Hi == nil ||
		sectionIndex < 0 || segment < 0 || band < 0 {
		return nil, empty, empty, false, nil
	}
	proof, graph, ok, err := revolveLoftFacesLiveGraph(ctx, blank, tooth)
	if err != nil || !ok || sectionIndex >= len(proof.section.Spans) {
		return nil, empty, empty, false, err
	}
	section := proof.section.Spans[sectionIndex]
	if section.Face == nil || section.Start.U == section.End.U {
		return nil, empty, empty, false, nil
	}
	var selected *revolveLoftFacesGraphRun
	for i := range graph.runs {
		run := &graph.runs[i]
		cell := graph.cells[run.cellID]
		if cell.sectionIndex != sectionIndex || faceNode(cell.blankFace) != section.Face ||
			cell.pairID < 0 || cell.source == nil ||
			graph.pairs[cell.pairID].verdict != revolveLoftFacesPairSourceCandidate {
			continue
		}
		mapped := cell.source.mapped
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
	if selected == nil || !slices.Equal(stations, selected.stations) {
		return nil, empty, empty, false, nil
	}
	seam, ok, err := revolveloftfaces.ArcSeamVerticesUnchecked(ctx, proof.trim,
		band, selected.stations, root, section)
	if err != nil || !ok {
		return nil, empty, empty, false, err
	}
	blankPatch, loftPatch, ok, err := revolveloftfaces.ArcPatchesUnchecked(ctx, proof.trim,
		band, segment, section, seam)
	if err != nil || !ok {
		return nil, empty, empty, false, err
	}
	if !graph.checkOpenPatches(selected, seam, &blankPatch, &loftPatch) {
		return nil, empty, empty, false, nil
	}
	return seam, blankPatch, loftPatch, true, nil
}

// The station group labels identify source chords. A complete cyclic run
// contains every chord of this Sketch arc and its next shared endpoint.
func revolveLoftFacesCompleteArcRun(trim *loftFacesTrimRecord, segment int) ([]int, bool) {
	if trim == nil || trim.Source == nil || segment < 0 ||
		segment >= len(trim.Source.Profile.Outer.Segments) ||
		len(trim.StationGroup) != len(trim.Source.SourceStation) ||
		len(trim.StationGroup) < 3 {
		return nil, false
	}
	groups := trim.StationGroup
	start, runs := -1, 0
	for index, group := range groups {
		if group == segment && groups[(index+len(groups)-1)%len(groups)] != segment {
			start, runs = index, runs+1
		}
	}
	if runs != 1 {
		return nil, false
	}
	stations := make([]int, 0, len(groups)+1)
	for index := start; groups[index] == segment; index = (index + 1) % len(groups) {
		stations = append(stations, index)
		if len(stations) >= len(groups) {
			return nil, false
		}
	}
	return append(stations, (stations[len(stations)-1]+1)%len(groups)), true
}

type revolveLoftFacesCallScopeKey struct{}

// A call scope holds the records that one top-level consumer call has
// already derived from the live bodies. It lives only in that call's
// context and is never stored anywhere else. Only successes are recorded.
type revolveLoftFacesCallScope struct {
	mu       sync.Mutex
	live     map[[2]*Body]revolveLoftFacesScopedGraph
	censuses map[*revolveLoftFacesSourceCertificate]*revolveLoftFacesPairCensus
}

type revolveLoftFacesScopedGraph struct {
	proof *revolveLoftFacesSourceCertificate
	graph *revolveLoftFacesContactGraph
}

// revolveLoftFacesWithCallScope opens a call scope for the outermost
// top-level consumer. A context that already carries one is returned
// unchanged, so nested consumers share the outer scope.
func revolveLoftFacesWithCallScope(ctx context.Context) context.Context {
	if ctx == nil || revolveLoftFacesCallScopeOf(ctx) != nil {
		return ctx
	}
	return context.WithValue(ctx, revolveLoftFacesCallScopeKey{}, &revolveLoftFacesCallScope{
		live:     make(map[[2]*Body]revolveLoftFacesScopedGraph),
		censuses: make(map[*revolveLoftFacesSourceCertificate]*revolveLoftFacesPairCensus),
	})
}

func revolveLoftFacesCallScopeOf(ctx context.Context) *revolveLoftFacesCallScope {
	if ctx == nil {
		return nil
	}
	scope, _ := ctx.Value(revolveLoftFacesCallScopeKey{}).(*revolveLoftFacesCallScope)
	return scope
}

// revolveLoftFacesLiveGraph certifies blank and tooth and builds the contact
// graph of that certificate. Inside a call scope the first success is kept
// and returned to every later check of the same pair. The result is only for
// reading: a caller must never hold or edit a scoped graph, whose cells point
// into its census.
func revolveLoftFacesLiveGraph(ctx context.Context, blank, tooth *Body) (
	*revolveLoftFacesSourceCertificate, *revolveLoftFacesContactGraph, bool, error) {
	scope := revolveLoftFacesCallScopeOf(ctx)
	key := [2]*Body{blank, tooth}
	if scope != nil {
		scope.mu.Lock()
		hit, found := scope.live[key]
		scope.mu.Unlock()
		if found {
			return hit.proof, hit.graph, true, nil
		}
	}
	proof, ok, err := certifyRevolveLoftFacesSource(ctx, blank, tooth)
	if err != nil || !ok {
		return nil, nil, false, err
	}
	graph, ok, err := revolveLoftFacesContactGraphOf(ctx, proof)
	if err != nil || !ok {
		return proof, nil, false, err
	}
	if scope != nil {
		scope.mu.Lock()
		scope.live[key] = revolveLoftFacesScopedGraph{proof: proof, graph: graph}
		scope.mu.Unlock()
	}
	return proof, graph, true, nil
}

// revolveLoftFacesCensusIn derives the census of proof, once per source
// certificate inside a call scope.
func revolveLoftFacesCensusIn(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate) (*revolveLoftFacesPairCensus, bool, error) {
	scope := revolveLoftFacesCallScopeOf(ctx)
	if scope != nil && proof != nil {
		scope.mu.Lock()
		hit, found := scope.censuses[proof]
		scope.mu.Unlock()
		if found {
			return hit, true, nil
		}
	}
	census, ok, err := revolveLoftFacesCensus(ctx, proof)
	if err == nil && ok && scope != nil && proof != nil {
		scope.mu.Lock()
		scope.censuses[proof] = census
		scope.mu.Unlock()
	}
	return census, ok, err
}
