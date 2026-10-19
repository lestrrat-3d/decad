package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/sketch"
)

// A vertical trace is ordered by the source's exact parameters. Its endpoints
// are graph-owned cap vertices; its intermediate vertices lie on shell rays.
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
	cellID         int
	cell           *revolveLoftFacesVerticalCell
	x, time, upper *big.Rat
}

type revolveLoftFacesVerticalTraceHit struct {
	piece  int
	ray    int
	lo, hi *big.Rat
}

func revolveLoftFacesVerticalTraceControl(cell *revolveLoftFacesVerticalCell,
	affine *loftface.VerticalAffine) ([]*big.Rat, []*big.Rat, bool) {
	if cell == nil || affine == nil || len(cell.source) < 2 || len(cell.source) > 4 {
		return nil, nil, false
	}
	y, z := make([]*big.Rat, len(cell.source)), make([]*big.Rat, len(cell.source))
	for i, point := range cell.source {
		if point.U == nil || point.V == nil {
			return nil, nil, false
		}
		y[i] = lfAdd(affine.Offset[0], lfAdd(
			lfMul(affine.Column[0][0], point.U), lfMul(affine.Column[1][0], point.V)))
		z[i] = lfAdd(affine.Offset[1], lfAdd(
			lfMul(affine.Column[0][1], point.U), lfMul(affine.Column[1][1], point.V)))
	}
	return y, z, true
}

func revolveLoftFacesVerticalTraceSamePoint(a, b [2]*big.Rat) bool {
	return a[0].Cmp(b[0]) == 0 && a[1].Cmp(b[1]) == 0
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
			y, z, valid = revolveLoftFacesVerticalTraceControl(cell, side.yzAffine)
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
				},
				cellID: id, cell: cell, x: side.x,
				time: side.time, upper: side.upper,
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
					!revolveLoftFacesVerticalTraceSamePoint(end,
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

// The ray root is enclosed by exact rational source parameters. A rational
// crossing is returned as a zero-width interval; otherwise 80 dyadic steps
// bound its full mapped source box before any held point is chosen.
func revolveLoftFacesVerticalTraceRayHit(piece revolveLoftFacesVerticalTracePiece,
	ray revolveLoftFacesRay) (revolveLoftFacesVerticalTraceHit, bool) {
	if ray.y == nil || ray.z == nil {
		return revolveLoftFacesVerticalTraceHit{}, false
	}
	if piece.Arc != nil {
		start := revolveloftfaces.VerticalTraceSourcePoint(piece.VerticalTraceSourcePiece, lfZero())
		end := revolveloftfaces.VerticalTraceSourcePoint(piece.VerticalTraceSourcePiece, lfOne())
		before := lfSub(lfMul(start[0], ray.z), lfMul(start[1], ray.y))
		after := lfSub(lfMul(ray.y, end[1]), lfMul(ray.z, end[0]))
		return revolveLoftFacesVerticalTraceHit{}, before.Sign() > 0 && after.Sign() > 0
	}
	cross := make([]*big.Rat, len(piece.Y))
	for i := range cross {
		cross[i] = lfSub(lfMul(piece.Y[i], ray.z), lfMul(piece.Z[i], ray.y))
	}
	lo, hi := piece.Piece.Source.Lo, piece.Piece.Source.Hi
	a, b := polynomial.BernsteinSign(cross, lo), polynomial.BernsteinSign(cross, hi)
	if a == 0 {
		return revolveLoftFacesVerticalTraceHit{lo: lo, hi: lo}, true
	}
	if b == 0 {
		return revolveLoftFacesVerticalTraceHit{lo: hi, hi: hi}, true
	}
	if a == b {
		return revolveLoftFacesVerticalTraceHit{}, false
	}
	for range 80 {
		mid := lfDiv(lfAdd(lo, hi), big.NewRat(2, 1))
		sign := polynomial.BernsteinSign(cross, mid)
		if sign == 0 {
			return revolveLoftFacesVerticalTraceHit{lo: mid, hi: mid}, true
		}
		if sign == a {
			lo = mid
		} else {
			hi = mid
		}
	}
	return revolveLoftFacesVerticalTraceHit{lo: lo, hi: hi}, true
}

func revolveLoftFacesVerticalTraceSplitWedges(ctx context.Context,
	shell *revolveLoftFacesBlankSeamShell, ring revolveLoftFacesShellCapRing,
	section int, nodes []revolveLoftFacesTraceNode, path []revolveLoftFacesTraceEdge,
) (bool, error) {
	read := revolveloftfaces.VerticalTraceSplitReaders[
		revolveLoftFacesTraceNode, revolveLoftFacesTraceEdge, revolveLoftFacesShellTriangleCell]{
		NodeVertex: func(node revolveLoftFacesTraceNode) int { return node.vertex },
		NodeRay:    func(node revolveLoftFacesTraceNode) int { return node.ray },
		EdgeEnds: func(edge revolveLoftFacesTraceEdge) (int, int) {
			return edge.start, edge.end
		},
		EdgeWedge: func(edge revolveLoftFacesTraceEdge) int { return edge.wedge },
		CellLabels: func(cell revolveLoftFacesShellTriangleCell) (int, int, int, int) {
			return cell.section, cell.band, cell.ray, cell.half
		},
		NewCell: func(section, band, ray, half int) revolveLoftFacesShellTriangleCell {
			return revolveLoftFacesShellTriangleCell{
				section: section, band: band, ray: ray, half: half,
			}
		},
	}
	tris, src, cells, built, err := revolveloftfaces.VerticalTraceSplitWedges(ctx,
		&shell.mesh, shell.sectionRings, shell.rayCount, ring.vertices,
		section, nodes, path, shell.triangleCells, read)
	if err != nil || !built {
		return false, err
	}
	shell.mesh.Tris, shell.mesh.Src, shell.triangleCells = tris, src, cells
	return true, nil
}
