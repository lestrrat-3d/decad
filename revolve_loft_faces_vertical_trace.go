package decad

import (
	"context"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/sketch"
)

// A vertical trace is ordered by the source's exact parameters. Its endpoints
// are graph-owned cap vertices; its intermediate vertices lie on shell rays.
// The path math lives in revolveloftfaces; this file keeps the steps that
// read the contact graph and its live *Face and *Body records.
type revolveLoftFacesVerticalTrace struct {
	sectionIndex int
	capIDs       [2]int
	cellIDs      []int
	vertices     []int
	vertexRays   []int
	edgeWedges   []int
	edgeCellIDs  []int
	loftBound    float64
}

type revolveLoftFacesVerticalTracePiece struct {
	revolveloftfaces.VerticalTraceSourcePiece
	cellID int
	cell   *revolveLoftFacesVerticalCell
}

// The cap's unique vertical owners choose the ends. Exact mapped endpoints
// then link all retained source pieces without sorting by a gear or face name.
func revolveLoftFacesVerticalTraceChain(graph *revolveLoftFacesContactGraph,
	arc *revolveLoftFacesGraphArc) ([]revolveLoftFacesVerticalTracePiece, bool) {
	if graph == nil || arc == nil || arc.arc == nil ||
		arc.sectionIndex < 0 || arc.endpointCellIDs[0] == arc.endpointCellIDs[1] {
		return nil, false
	}
	var side *revolveLoftFacesVerticalSide
	for _, entry := range graph.manifest.vertical {
		if entry.sectionIndex == arc.sectionIndex && entry.outcome == revolveLoftFacesVerticalContact {
			if side != nil {
				return nil, false
			}
			side = entry.side
		}
	}
	if side == nil || side.yzAffine == nil || side.x == nil ||
		side.x.Cmp(arc.arc.circle.x) != 0 || side.upper == nil ||
		side.upper.Cmp(arc.arc.circle.radius) != 0 {
		return nil, false
	}
	payload, ok := graph.tooth.n.Payload.(facetedPayload)
	if !ok || payload.LoftFacesTrim == nil ||
		payload.LoftFacesTrim.Source == nil ||
		payload.LoftFacesTrim.Source.Source == nil {
		return nil, false
	}
	trim := payload.LoftFacesTrim
	pieces := make([]revolveLoftFacesVerticalTracePiece, 0)
	for id := range graph.cells {
		entry := &graph.cells[id]
		if entry.sectionIndex != arc.sectionIndex || entry.blankFace != arc.arc.circle.blankFace ||
			entry.vertical == nil || entry.pairID < 0 ||
			entry.pairID >= len(graph.pairs) ||
			graph.pairs[entry.pairID].verdict != revolveLoftFacesPairSourceCandidate {
			continue
		}
		cell := entry.vertical
		var y, z []*big.Rat
		var arc *arcSeg
		var radius2 *big.Rat
		var arcBox [2]loftFaceInterval
		if len(cell.source) == 0 {
			if cell.segment < 0 || cell.segment >= len(trim.Source.Profile.Outer.Segments) ||
				cell.segment >= len(trim.Source.Source.Entities) {
				return nil, false
			}
			record, recorded := trim.Source.Profile.Outer.Segments[cell.segment].(arcSeg)
			live, liveArc := trim.Source.Source.Entities[cell.segment].(*sketch.Arc)
			if !recorded || !liveArc || !revolveloftfaces.MappedArcRecordMatches(record, live) ||
				cell.span != 0 || cell.band != side.band {
				return nil, false
			}
			simpleCell, simple := revolveloftfaces.SimpleArc(trim, cell.segment,
				record, record.TStart > record.TEnd)
			if !simple {
				return nil, false
			}
			var valid bool
			radius2, valid = revolveloftfaces.VerticalTraceArcRadius(side.yzAffine, record)
			if !valid {
				return nil, false
			}
			arc = &record
			arcBox = simpleCell.Box
		} else {
			var valid bool
			y, z, valid = revolveloftfaces.VerticalTraceControl(cell.source, side.yzAffine)
			if !valid {
				return nil, false
			}
		}
		for _, piece := range cell.retained {
			if piece.Source.Lo == nil || piece.Source.Hi == nil ||
				piece.Source.Lo.Cmp(piece.Source.Hi) >= 0 ||
				piece.LoOpen || piece.HiOpen {
				return nil, false
			}
			pieces = append(pieces, revolveLoftFacesVerticalTracePiece{
				VerticalTraceSourcePiece: revolveloftfaces.VerticalTraceSourcePiece{
					Piece: piece, Y: y, Z: z, Arc: arc, Affine: side.yzAffine,
					Radius2: radius2, ArcBox: arcBox,
					Band: cell.band, Span: cell.source, UpperRoot: cell.upper.Root,
					X: side.x, Time: side.time, UpperRadius: side.upper,
				},
				cellID: id, cell: cell,
			})
		}
	}
	if len(pieces) < 1 || len(pieces) > 128 {
		return nil, false
	}
	rootEnd := func(p revolveLoftFacesVerticalTracePiece, boundary revolveLoftFacesToolCapBoundary) (
		*big.Rat, *big.Rat, bool) {
		if p.cell.segment != boundary.segment || p.cell.span != boundary.span ||
			p.cell.band != arc.arc.band || p.cell.ref != boundary.ref ||
			p.cell.face != boundary.face ||
			!loftface.IntervalEqual(p.cell.upper.Root, boundary.parameter) ||
			!revolveloftfaces.CapBoundaryBoxEqual(p.cell.upper.Box, boundary.box) {
			return nil, nil, false
		}
		if p.Piece.Source.Lo.Cmp(boundary.parameter.Hi) == 0 {
			return p.Piece.Source.Lo, p.Piece.Source.Hi, true
		}
		if p.Piece.Source.Hi.Cmp(boundary.parameter.Lo) == 0 {
			return p.Piece.Source.Hi, p.Piece.Source.Lo, true
		}
		return nil, nil, false
	}
	start, finish := -1, -1
	for i, piece := range pieces {
		if piece.cellID == arc.endpointCellIDs[0] {
			if from, to, ok := rootEnd(piece, arc.arc.negativeZ); ok {
				if start >= 0 {
					return nil, false
				}
				start = i
				pieces[i].From, pieces[i].To = from, to
			}
		}
		if piece.cellID == arc.endpointCellIDs[1] {
			if from, to, ok := rootEnd(piece, arc.arc.positiveZ); ok {
				if finish >= 0 {
					return nil, false
				}
				finish = i
				// The chain enters the positive cap owner at the opposite end.
				pieces[i].From, pieces[i].To = to, from
			}
		}
	}
	if start < 0 || finish < 0 || start == finish {
		return nil, false
	}
	ordered := make([]revolveLoftFacesVerticalTracePiece, 0, len(pieces))
	used := make([]bool, len(pieces))
	current := start
	for {
		if used[current] {
			return nil, false
		}
		used[current] = true
		piece := pieces[current]
		ordered = append(ordered, piece)
		if current == finish {
			break
		}
		end := revolveloftfaces.VerticalTraceSourcePoint(piece.VerticalTraceSourcePiece, piece.To)
		next := -1
		for i := range pieces {
			if used[i] || i == finish && pieces[i].From == nil {
				continue
			}
			candidate := pieces[i]
			if candidate.cellID == piece.cellID || candidate.cell.band != piece.cell.band {
				continue
			}
			for _, endpoint := range [...]struct{ from, to *big.Rat }{
				{candidate.Piece.Source.Lo, candidate.Piece.Source.Hi},
				{candidate.Piece.Source.Hi, candidate.Piece.Source.Lo},
			} {
				if i == finish && endpoint.from.Cmp(candidate.From) != 0 ||
					!revolveloftfaces.VerticalTraceSamePoint(end,
						revolveloftfaces.VerticalTraceSourcePoint(candidate.VerticalTraceSourcePiece, endpoint.from)) {
					continue
				}
				if next >= 0 {
					return nil, false
				}
				next = i
				pieces[i].From, pieces[i].To = endpoint.from, endpoint.to
			}
		}
		if next < 0 {
			return nil, false
		}
		current = next
		if len(ordered) >= len(pieces) {
			return nil, false
		}
	}
	// Every retained source piece on this planar side must have an owner in
	// this path. A second disconnected component needs a separate trace.
	return ordered, len(ordered) == len(pieces)
}

// This consumer builds the live graph and all-run shell, then adds one
// connected vertical source path to its planar Revolve face. It does not
// classify retained facets or publish a Boolean result.
func revolveLoftFacesBlankShellWithVerticalTrace(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, selectedCellID int,
) (*revolveLoftFacesBlankSeamShell, *revolveLoftFacesContactGraph,
	*revolveLoftFacesVerticalTrace, bool, error) {
	ctx = revolveLoftFacesWithCallScope(ctx)
	shell, graph, built, err := revolveLoftFacesBlankShellWithAllRuns(ctx, proof)
	if err != nil || !built || shell == nil || graph == nil {
		return nil, nil, nil, false, err
	}
	trace, built, err := revolveLoftFacesInsertVerticalTrace(ctx, proof, shell, graph, selectedCellID)
	if err != nil || !built {
		return nil, nil, nil, false, err
	}
	return shell, graph, trace, true, nil
}

func revolveLoftFacesInsertVerticalTrace(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, shell *revolveLoftFacesBlankSeamShell,
	graph *revolveLoftFacesContactGraph, selectedCellID int,
) (*revolveLoftFacesVerticalTrace, bool, error) {
	if ctx == nil || proof == nil || shell == nil || graph == nil ||
		selectedCellID < 0 || selectedCellID >= len(graph.cells) ||
		len(shell.TriangleCells) != len(shell.Mesh.Tris) ||
		len(shell.Rays) != shell.RayCount {
		return nil, false, nil
	}
	current, err := revolveLoftFacesContactGraphCurrent(ctx, graph)
	if err != nil || !current || graph.blank != proof.blank || graph.tooth != proof.tooth {
		return nil, false, err
	}
	selected := graph.cells[selectedCellID]
	if selected.vertical == nil || selected.pairID < 0 ||
		selected.pairID >= len(graph.pairs) ||
		graph.pairs[selected.pairID].verdict != revolveLoftFacesPairSourceCandidate {
		return nil, false, nil
	}
	var event *revolveLoftFacesGraphArc
	var ring *revolveLoftFacesShellCapRing
	for i := range graph.arcs {
		candidate := &graph.arcs[i]
		if candidate.sectionIndex != selected.sectionIndex ||
			candidate.arc == nil ||
			candidate.arc.circle.blankFace != selected.blankFace {
			continue
		}
		if event != nil {
			return nil, false, nil
		}
		event = candidate
	}
	if event == nil || event.sectionIndex < 0 ||
		event.sectionIndex >= len(proof.section.Spans) ||
		proof.section.Spans[event.sectionIndex].Start.U !=
			proof.section.Spans[event.sectionIndex].End.U {
		return nil, false, nil
	}
	for i := range shell.capRings {
		candidate := &shell.capRings[i]
		if candidate.Section == event.sectionIndex &&
			candidate.endpointGraphIDs == event.endpointIDs {
			if ring != nil {
				return nil, false, nil
			}
			ring = candidate
		}
	}
	if ring == nil || ring.endpointCellIDs != event.endpointCellIDs ||
		ring.arc == nil ||
		!revolveLoftFacesToolCapOccupiedArcEqual(ring.arc, event.arc) {
		return nil, false, nil
	}
	pieces, chained := revolveLoftFacesVerticalTraceChain(graph, event)
	if !chained {
		return nil, false, nil
	}
	// The chain already caps this transient projection at 128 ordered pieces.
	sourcePieces := make([]revolveloftfaces.VerticalTraceSourcePiece, len(pieces))
	for i := range pieces {
		sourcePieces[i] = pieces[i].VerticalTraceSourcePiece
	}
	_, monotone := revolveloftfaces.VerticalTracePieceSigns(sourcePieces)
	if !monotone ||
		!revolveloftfaces.VerticalTraceSourceProof(sourcePieces, event.arc.circle.radius) {
		return nil, false, nil
	}
	selectedInChain := false
	for _, piece := range pieces {
		selectedInChain = selectedInChain || piece.cellID == selectedCellID
	}
	if !selectedInChain {
		return nil, false, nil
	}
	if proof.trim == nil || len(ring.Occupied) < 2 {
		return nil, false, nil
	}
	working := *shell
	working.Mesh.Tris = slices.Clone(shell.Mesh.Tris)
	working.Mesh.Src = slices.Clone(shell.Mesh.Src)
	working.Mesh.Groups = slices.Clone(shell.Mesh.Groups)
	working.TriangleCells = slices.Clone(shell.TriangleCells)
	path, built, err := revolveloftfaces.VerticalTracePath(ctx, proof.trim.Maps, working.Rays,
		slices.Clone(shell.Mesh.Verts), slices.Clone(shell.VertexBounds),
		ring.Vertices, ring.Endpoints, ring.arc.circle.radius, sourcePieces)
	if err != nil || !built {
		return nil, false, err
	}
	working.Mesh.Verts, working.VertexBounds = path.Verts, path.VertexBounds
	trace := &revolveLoftFacesVerticalTrace{
		sectionIndex: event.sectionIndex, capIDs: event.endpointIDs,
		vertices: make([]int, len(path.Nodes)), vertexRays: make([]int, len(path.Nodes)),
		cellIDs:    make([]int, len(pieces)),
		edgeWedges: make([]int, len(path.Edges)), edgeCellIDs: make([]int, len(path.Edges)),
		loftBound: path.LoftBound}
	for i, piece := range pieces {
		trace.cellIDs[i] = piece.cellID
	}
	for i, node := range path.Nodes {
		trace.vertices[i] = node.Vertex
		trace.vertexRays[i] = node.Ray
	}
	for i, edge := range path.Edges {
		trace.edgeWedges[i] = edge.Wedge
		trace.edgeCellIDs[i] = pieces[edge.Piece].cellID
	}
	tris, src, cells, built, err := revolveloftfaces.VerticalTraceSplitWedges(ctx,
		&working.Mesh, working.SectionRings, working.RayCount, ring.Vertices,
		event.sectionIndex, path.Nodes, path.Edges, working.TriangleCells)
	if err != nil || !built {
		return nil, false, err
	}
	working.Mesh.Tris, working.Mesh.Src, working.TriangleCells = tris, src, cells
	if len(working.TriangleCells) != len(working.Mesh.Tris) ||
		!working.EulerTwo() {
		return nil, false, err
	}
	audited, err := working.Audit(ctx, 0.1)
	if err != nil || !audited {
		return nil, false, err
	}
	*shell = working
	return trace, true, nil
}
