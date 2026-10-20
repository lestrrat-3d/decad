package revolvecontact

import (
	"context"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// A vertical trace is ordered by the source's exact parameters. Its endpoints
// are graph-owned cap vertices; its intermediate vertices lie on shell rays.
// The path math lives in revolveloftfaces; this file keeps the steps that
// read the contact graph and its live *topo.Face and *topo.Body records.
type VerticalTrace struct {
	SectionIndex int
	CapIDs       [2]int
	CellIDs      []int
	Vertices     []int
	VertexRays   []int
	EdgeWedges   []int
	EdgeCellIDs  []int
	LoftBound    float64
}

type VerticalTracePiece struct {
	revolveloftfaces.VerticalTraceSourcePiece
	CellID int
	Cell   *VerticalCell
}

// The cap's unique vertical owners choose the ends. Exact mapped endpoints
// then link all retained source pieces without sorting by a gear or face name.
func VerticalTraceChain(graph *ContactGraph,
	arc *GraphArc) ([]VerticalTracePiece, bool) {
	if graph == nil || arc == nil || arc.Arc == nil ||
		arc.SectionIndex < 0 || arc.EndpointCellIDs[0] == arc.EndpointCellIDs[1] {
		return nil, false
	}
	var side *VerticalSide
	for _, entry := range graph.Manifest.Vertical {
		if entry.SectionIndex == arc.SectionIndex && entry.Outcome == revolveloftfaces.VerticalContact {
			if side != nil {
				return nil, false
			}
			side = entry.Side
		}
	}
	if side == nil || side.YzAffine == nil || side.X == nil ||
		side.X.Cmp(arc.Arc.Circle.X) != 0 || side.Upper == nil ||
		side.Upper.Cmp(arc.Arc.Circle.Radius) != 0 {
		return nil, false
	}
	payload, ok := graph.Tooth.Payload.(facetedrecord.Payload)
	if !ok || payload.LoftFacesTrim == nil ||
		payload.LoftFacesTrim.Source == nil ||
		payload.LoftFacesTrim.Source.Source == nil {
		return nil, false
	}
	trim := payload.LoftFacesTrim
	pieces := make([]VerticalTracePiece, 0)
	for id := range graph.Cells {
		entry := &graph.Cells[id]
		if entry.SectionIndex != arc.SectionIndex || entry.BlankFace != arc.Arc.Circle.BlankFace ||
			entry.Vertical == nil || entry.PairID < 0 ||
			entry.PairID >= len(graph.Pairs) ||
			graph.Pairs[entry.PairID].Verdict != PairSourceCandidate {
			continue
		}
		cell := entry.Vertical
		var y, z []*big.Rat
		var arc *sectionrecord.ArcSeg
		var radius2 *big.Rat
		var arcBox [2]loftface.Interval
		if len(cell.Source) == 0 {
			if cell.Segment < 0 || cell.Segment >= len(trim.Source.Profile.Outer.Segments) ||
				cell.Segment >= len(trim.Source.Source.Entities) {
				return nil, false
			}
			record, recorded := trim.Source.Profile.Outer.Segments[cell.Segment].(sectionrecord.ArcSeg)
			live, liveArc := trim.Source.Source.Entities[cell.Segment].(*sketch.Arc)
			if !recorded || !liveArc || !revolveloftfaces.MappedArcRecordMatches(record, live) ||
				cell.Span != 0 || cell.Band != side.Band {
				return nil, false
			}
			simpleCell, simple := revolveloftfaces.SimpleArc(trim, cell.Segment,
				record, record.TStart > record.TEnd)
			if !simple {
				return nil, false
			}
			var valid bool
			radius2, valid = revolveloftfaces.VerticalTraceArcRadius(side.YzAffine, record)
			if !valid {
				return nil, false
			}
			arc = &record
			arcBox = simpleCell.Box
		} else {
			var valid bool
			y, z, valid = revolveloftfaces.VerticalTraceControl(cell.Source, side.YzAffine)
			if !valid {
				return nil, false
			}
		}
		for _, piece := range cell.Retained {
			if piece.Source.Lo == nil || piece.Source.Hi == nil ||
				piece.Source.Lo.Cmp(piece.Source.Hi) >= 0 ||
				piece.LoOpen || piece.HiOpen {
				return nil, false
			}
			pieces = append(pieces, VerticalTracePiece{
				VerticalTraceSourcePiece: revolveloftfaces.VerticalTraceSourcePiece{
					Piece: piece, Y: y, Z: z, Arc: arc, Affine: side.YzAffine,
					Radius2: radius2, ArcBox: arcBox,
					Band: cell.Band, Span: cell.Source, UpperRoot: cell.Upper.Root,
					X: side.X, Time: side.Time, UpperRadius: side.Upper,
				},
				CellID: id, Cell: cell,
			})
		}
	}
	if len(pieces) < 1 || len(pieces) > 128 {
		return nil, false
	}
	rootEnd := func(p VerticalTracePiece, boundary ToolCapBoundary) (
		*big.Rat, *big.Rat, bool) {
		if p.Cell.Segment != boundary.Segment || p.Cell.Span != boundary.Span ||
			p.Cell.Band != arc.Arc.Band || p.Cell.Ref != boundary.Ref ||
			p.Cell.Face != boundary.Face ||
			!loftface.IntervalEqual(p.Cell.Upper.Root, boundary.Parameter) ||
			!revolveloftfaces.CapBoundaryBoxEqual(p.Cell.Upper.Box, boundary.Box) {
			return nil, nil, false
		}
		if p.Piece.Source.Lo.Cmp(boundary.Parameter.Hi) == 0 {
			return p.Piece.Source.Lo, p.Piece.Source.Hi, true
		}
		if p.Piece.Source.Hi.Cmp(boundary.Parameter.Lo) == 0 {
			return p.Piece.Source.Hi, p.Piece.Source.Lo, true
		}
		return nil, nil, false
	}
	start, finish := -1, -1
	for i, piece := range pieces {
		if piece.CellID == arc.EndpointCellIDs[0] {
			if from, to, ok := rootEnd(piece, arc.Arc.NegativeZ); ok {
				if start >= 0 {
					return nil, false
				}
				start = i
				pieces[i].From, pieces[i].To = from, to
			}
		}
		if piece.CellID == arc.EndpointCellIDs[1] {
			if from, to, ok := rootEnd(piece, arc.Arc.PositiveZ); ok {
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
	ordered := make([]VerticalTracePiece, 0, len(pieces))
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
			if candidate.CellID == piece.CellID || candidate.Cell.Band != piece.Cell.Band {
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
func BlankShellWithVerticalTrace(ctx context.Context,
	proof *SourceCertificate, selectedCellID int,
) (*BlankSeamShell, *ContactGraph,
	*VerticalTrace, bool, error) {
	ctx = WithCallScope(ctx)
	shell, graph, built, err := BlankShellWithAllRuns(ctx, proof)
	if err != nil || !built || shell == nil || graph == nil {
		return nil, nil, nil, false, err
	}
	trace, built, err := InsertVerticalTrace(ctx, proof, shell, graph, selectedCellID)
	if err != nil || !built {
		return nil, nil, nil, false, err
	}
	return shell, graph, trace, true, nil
}

func InsertVerticalTrace(ctx context.Context,
	proof *SourceCertificate, shell *BlankSeamShell,
	graph *ContactGraph, selectedCellID int,
) (*VerticalTrace, bool, error) {
	if ctx == nil || proof == nil || shell == nil || graph == nil ||
		selectedCellID < 0 || selectedCellID >= len(graph.Cells) ||
		len(shell.TriangleCells) != len(shell.Mesh.Tris) ||
		len(shell.Rays) != shell.RayCount {
		return nil, false, nil
	}
	current, err := ContactGraphCurrent(ctx, graph)
	if err != nil || !current || graph.Blank != proof.Blank || graph.Tooth != proof.Tooth {
		return nil, false, err
	}
	selected := graph.Cells[selectedCellID]
	if selected.Vertical == nil || selected.PairID < 0 ||
		selected.PairID >= len(graph.Pairs) ||
		graph.Pairs[selected.PairID].Verdict != PairSourceCandidate {
		return nil, false, nil
	}
	var event *GraphArc
	var ring *ShellCapRing
	for i := range graph.Arcs {
		candidate := &graph.Arcs[i]
		if candidate.SectionIndex != selected.SectionIndex ||
			candidate.Arc == nil ||
			candidate.Arc.Circle.BlankFace != selected.BlankFace {
			continue
		}
		if event != nil {
			return nil, false, nil
		}
		event = candidate
	}
	if event == nil || event.SectionIndex < 0 ||
		event.SectionIndex >= len(proof.Section.Spans) ||
		proof.Section.Spans[event.SectionIndex].Start.U !=
			proof.Section.Spans[event.SectionIndex].End.U {
		return nil, false, nil
	}
	for i := range shell.CapRings {
		candidate := &shell.CapRings[i]
		if candidate.Section == event.SectionIndex &&
			candidate.EndpointGraphIDs == event.EndpointIDs {
			if ring != nil {
				return nil, false, nil
			}
			ring = candidate
		}
	}
	if ring == nil || ring.EndpointCellIDs != event.EndpointCellIDs ||
		ring.Arc == nil ||
		!ToolCapOccupiedArcEqual(ring.Arc, event.Arc) {
		return nil, false, nil
	}
	pieces, chained := VerticalTraceChain(graph, event)
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
		!revolveloftfaces.VerticalTraceSourceProof(sourcePieces, event.Arc.Circle.Radius) {
		return nil, false, nil
	}
	selectedInChain := false
	for _, piece := range pieces {
		selectedInChain = selectedInChain || piece.CellID == selectedCellID
	}
	if !selectedInChain {
		return nil, false, nil
	}
	if proof.Trim == nil || len(ring.Occupied) < 2 {
		return nil, false, nil
	}
	working := *shell
	working.Mesh.Tris = slices.Clone(shell.Mesh.Tris)
	working.Mesh.Src = slices.Clone(shell.Mesh.Src)
	working.Mesh.Groups = slices.Clone(shell.Mesh.Groups)
	working.TriangleCells = slices.Clone(shell.TriangleCells)
	path, built, err := revolveloftfaces.VerticalTracePath(ctx, proof.Trim.Maps, working.Rays,
		slices.Clone(shell.Mesh.Verts), slices.Clone(shell.VertexBounds),
		ring.Vertices, ring.Endpoints, ring.Arc.Circle.Radius, sourcePieces)
	if err != nil || !built {
		return nil, false, err
	}
	working.Mesh.Verts, working.VertexBounds = path.Verts, path.VertexBounds
	trace := &VerticalTrace{
		SectionIndex: event.SectionIndex, CapIDs: event.EndpointIDs,
		Vertices: make([]int, len(path.Nodes)), VertexRays: make([]int, len(path.Nodes)),
		CellIDs:    make([]int, len(pieces)),
		EdgeWedges: make([]int, len(path.Edges)), EdgeCellIDs: make([]int, len(path.Edges)),
		LoftBound: path.LoftBound}
	for i, piece := range pieces {
		trace.CellIDs[i] = piece.CellID
	}
	for i, node := range path.Nodes {
		trace.Vertices[i] = node.Vertex
		trace.VertexRays[i] = node.Ray
	}
	for i, edge := range path.Edges {
		trace.EdgeWedges[i] = edge.Wedge
		trace.EdgeCellIDs[i] = pieces[edge.Piece].CellID
	}
	tris, src, cells, built, err := revolveloftfaces.VerticalTraceSplitWedges(ctx,
		&working.Mesh, working.SectionRings, working.RayCount, ring.Vertices,
		event.SectionIndex, path.Nodes, path.Edges, working.TriangleCells)
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
