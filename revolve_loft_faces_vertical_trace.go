package decad

import (
	"context"
	"math"
	"math/big"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/r3"
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
	ray revolveloftfaces.ShellRay) (revolveLoftFacesVerticalTraceHit, bool) {
	if ray.Y == nil || ray.Z == nil {
		return revolveLoftFacesVerticalTraceHit{}, false
	}
	if piece.Arc != nil {
		start := revolveloftfaces.VerticalTraceSourcePoint(piece.VerticalTraceSourcePiece, lfZero())
		end := revolveloftfaces.VerticalTraceSourcePoint(piece.VerticalTraceSourcePiece, lfOne())
		before := lfSub(lfMul(start[0], ray.Z), lfMul(start[1], ray.Y))
		after := lfSub(lfMul(ray.Y, end[1]), lfMul(ray.Z, end[0]))
		return revolveLoftFacesVerticalTraceHit{}, before.Sign() > 0 && after.Sign() > 0
	}
	cross := make([]*big.Rat, len(piece.Y))
	for i := range cross {
		cross[i] = lfSub(lfMul(piece.Y[i], ray.Z), lfMul(piece.Z[i], ray.Y))
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

type revolveLoftFacesTraceNode struct {
	vertex int
	ray    int // -1 for a source junction inside a wedge
	dir    [2]*big.Rat
	hit    revolveLoftFacesVerticalTraceHit
}

type revolveLoftFacesTraceEdge struct {
	start, end int
	piece      int
	wedge      int
	from, to   revolveLoftFacesVerticalTraceHit
}

func revolveLoftFacesTraceCross(a, b [2]*big.Rat) *big.Rat {
	return lfSub(lfMul(a[0], b[1]), lfMul(a[1], b[0]))
}

func revolveLoftFacesTraceWedge(rays []revolveloftfaces.ShellRay, a, b [2]*big.Rat) (int, bool) {
	if a[0] == nil || b[0] == nil || a[0].Sign() <= 0 || b[0].Sign() <= 0 {
		return -1, false
	}
	mid := [2]*big.Rat{lfAdd(a[0], b[0]), lfAdd(a[1], b[1])}
	wedge := -1
	for i := range rays {
		j := (i + 1) % len(rays)
		left := [2]*big.Rat{rays[i].Y, rays[i].Z}
		right := [2]*big.Rat{rays[j].Y, rays[j].Z}
		if revolveLoftFacesTraceCross(left, right).Sign() <= 0 ||
			revolveLoftFacesTraceCross(left, mid).Sign() <= 0 ||
			revolveLoftFacesTraceCross(mid, right).Sign() <= 0 {
			continue
		}
		if revolveLoftFacesTraceCross(left, a).Sign() < 0 ||
			revolveLoftFacesTraceCross(a, right).Sign() < 0 ||
			revolveLoftFacesTraceCross(left, b).Sign() < 0 ||
			revolveLoftFacesTraceCross(b, right).Sign() < 0 {
			continue
		}
		if wedge >= 0 {
			return -1, false
		}
		wedge = i
	}
	return wedge, wedge >= 0
}

func revolveLoftFacesTraceJoin(proof *revolveLoftFacesSourceCertificate,
	piece revolveLoftFacesVerticalTracePiece, parameter *big.Rat,
) (r3.Vec, float64, [2]*big.Rat, bool) {
	point := revolveloftfaces.VerticalTraceSourcePoint(piece.VerticalTraceSourcePiece, parameter)
	if point[0] == nil || point[1] == nil || point[0].Sign() <= 0 ||
		proof == nil || proof.trim == nil {
		return r3.Vec{}, 0, [2]*big.Rat{}, false
	}
	held, bound, ok := revolveloftfaces.HeldPoint(loftFaceRatVec{piece.x, point[0], point[1]})
	return held, bound, point, ok && !proofbound.IsNonFinite(bound)
}

func revolveLoftFacesTraceSourceHits(piece revolveLoftFacesVerticalTracePiece,
	rays []revolveloftfaces.ShellRay, endpointRays [2]int,
) ([]revolveLoftFacesVerticalTraceHit, bool) {
	hits := make([]revolveLoftFacesVerticalTraceHit, 0)
	for ray := range rays {
		if ray == endpointRays[0] || ray == endpointRays[1] {
			continue
		}
		if rays[ray].Y.Sign() <= 0 {
			continue
		}
		hit, found := revolveLoftFacesVerticalTraceRayHit(piece, rays[ray])
		if !found {
			continue
		}
		hit.ray = ray
		if piece.Arc == nil {
			if hit.lo == nil || hit.hi == nil ||
				hit.lo.Cmp(piece.From) <= 0 && hit.hi.Cmp(piece.From) >= 0 ||
				hit.lo.Cmp(piece.To) <= 0 && hit.hi.Cmp(piece.To) >= 0 {
				return nil, false
			}
		} else if hit.lo != nil || hit.hi != nil {
			return nil, false
		}
		hits = append(hits, hit)
	}
	sort.Slice(hits, func(i, j int) bool {
		if piece.Arc == nil {
			if piece.From.Cmp(piece.To) < 0 {
				return hits[i].lo.Cmp(hits[j].lo) < 0
			}
			return hits[i].lo.Cmp(hits[j].lo) > 0
		}
		a := [2]*big.Rat{rays[hits[i].ray].Y, rays[hits[i].ray].Z}
		b := [2]*big.Rat{rays[hits[j].ray].Y, rays[hits[j].ray].Z}
		if piece.From.Cmp(piece.To) < 0 {
			return revolveLoftFacesTraceCross(a, b).Sign() > 0
		}
		return revolveLoftFacesTraceCross(a, b).Sign() < 0
	})
	for i := 1; i < len(hits); i++ {
		if piece.Arc == nil {
			if hits[i-1].hi.Cmp(hits[i].lo) >= 0 && piece.From.Cmp(piece.To) < 0 ||
				hits[i].hi.Cmp(hits[i-1].lo) >= 0 && piece.From.Cmp(piece.To) > 0 {
				return nil, false
			}
		} else {
			a := [2]*big.Rat{rays[hits[i-1].ray].Y, rays[hits[i-1].ray].Z}
			b := [2]*big.Rat{rays[hits[i].ray].Y, rays[hits[i].ray].Z}
			if revolveLoftFacesTraceCross(a, b).Sign() !=
				-piece.From.Cmp(piece.To) {
				return nil, false
			}
		}
	}
	return hits, true
}

func revolveLoftFacesVerticalTraceSegmentBound(proof *revolveLoftFacesSourceCertificate,
	piece revolveLoftFacesVerticalTracePiece, from, to revolveLoftFacesVerticalTraceHit,
	fromDir, toDir [2]*big.Rat, held, endHeld r3.Vec, rays []revolveloftfaces.ShellRay,
) (float64, bool) {
	if piece.Arc != nil {
		if piece.upper == nil || piece.upper.Sign() <= 0 ||
			fromDir[0] == nil || toDir[0] == nil {
			return 0, false
		}
		dot := lfAdd(lfMul(fromDir[0], toDir[0]), lfMul(fromDir[1], toDir[1]))
		cross := lfAbs(revolveLoftFacesTraceCross(fromDir, toDir))
		if dot.Sign() <= 0 || cross.Sign() == 0 || cross.Cmp(dot) >= 0 {
			return 0, false
		}
		endpointDistance := func(hit revolveLoftFacesVerticalTraceHit, held r3.Vec) (float64, bool) {
			if hit.ray >= 0 {
				if hit.ray >= len(rays) {
					return 0, false
				}
				point, _, bound, ok := revolveLoftFacesVerticalTraceHeldPoint(
					proof, piece, hit, rays[hit.ray], piece.upper)
				return bound, ok && point == held
			}
			if hit.lo == nil || hit.hi == nil || hit.lo.Cmp(hit.hi) != 0 {
				return 0, false
			}
			point := revolveloftfaces.VerticalTraceSourcePoint(piece.VerticalTraceSourcePiece, hit.lo)
			if point[0] == nil || point[1] == nil {
				return 0, false
			}
			box := [3]loftface.Interval{{Lo: piece.x, Hi: piece.x},
				{Lo: point[0], Hi: point[0]}, {Lo: point[1], Hi: point[1]}}
			return loftface.VerticalTraceBoxDistance(box, held)
		}
		startDistance, ok := endpointDistance(from, held)
		if !ok {
			return 0, false
		}
		endDistance, ok := endpointDistance(to, endHeld)
		if !ok {
			return 0, false
		}
		// For theta < pi/4, theta <= tan(theta) = cross/dot. The arc
		// sagitta is at most radius * theta^2 / 8.
		q := lfDiv(cross, dot)
		sagitta := proofbound.RatFloatUp(lfDiv(
			lfMul(piece.upper, lfMul(q, q)), big.NewRat(8, 1)))
		bound := proofbound.AbsSumUpper(max(startDistance, endDistance), sagitta)
		return bound, !proofbound.IsNonFinite(bound) && bound >= 0
	}
	if proof == nil || proof.trim == nil || piece.cell.band < 0 ||
		piece.cell.band+1 >= len(proof.trim.Maps) || from.lo == nil || from.hi == nil ||
		to.lo == nil || to.hi == nil {
		return 0, false
	}
	if len(piece.Y) == 2 {
		endpointBox := func(hit revolveLoftFacesVerticalTraceHit) ([3]loftface.Interval, bool) {
			if hit.lo.Cmp(hit.hi) == 0 {
				point := revolveloftfaces.VerticalTraceSourcePoint(piece.VerticalTraceSourcePiece, hit.lo)
				return [3]loftface.Interval{{Lo: piece.x, Hi: piece.x},
						{Lo: point[0], Hi: point[0]}, {Lo: point[1], Hi: point[1]}},
					point[0] != nil && point[1] != nil
			}
			return loftface.JunctionBox(proof.trim.Maps[piece.cell.band],
				proof.trim.Maps[piece.cell.band+1], piece.cell.source,
				hit.lo, hit.hi, piece.time, piece.time)
		}
		start, ok := endpointBox(from)
		if !ok {
			return 0, false
		}
		end, ok := endpointBox(to)
		if !ok {
			return 0, false
		}
		startBound, ok := loftface.VerticalTraceBoxDistance(start, held)
		if !ok {
			return 0, false
		}
		endBound, ok := loftface.VerticalTraceBoxDistance(end, endHeld)
		return max(startBound, endBound), ok
	}
	lo, hi := from.lo, to.hi
	if lo.Cmp(to.lo) > 0 {
		lo = to.lo
	}
	if hi.Cmp(from.hi) < 0 {
		hi = from.hi
	}
	if lo.Cmp(hi) >= 0 {
		return 0, false
	}
	box, ok := loftface.JunctionBox(proof.trim.Maps[piece.cell.band],
		proof.trim.Maps[piece.cell.band+1], piece.cell.source,
		lo, hi, piece.time, piece.time)
	if !ok {
		return 0, false
	}
	return loftface.VerticalTraceBoxDistance(box, held)
}

func revolveLoftFacesVerticalTracePath(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, shell *revolveLoftFacesBlankSeamShell,
	pieces []revolveLoftFacesVerticalTracePiece, ring revolveLoftFacesShellCapRing,
) ([]revolveLoftFacesTraceNode, []revolveLoftFacesTraceEdge, float64, bool, error) {
	if len(pieces) == 0 || len(ring.occupied) < 2 {
		return nil, nil, 0, false, nil
	}
	capRays := [2]int{-1, -1}
	for endpoint := range capRays {
		capRays[endpoint] = -1
		for ray, vertex := range ring.vertices {
			if vertex == ring.endpoints[endpoint] {
				if capRays[endpoint] >= 0 {
					return nil, nil, 0, false, nil
				}
				capRays[endpoint] = ray
			}
		}
		if capRays[endpoint] < 0 {
			return nil, nil, 0, false, nil
		}
	}
	if capRays[0] == capRays[1] {
		return nil, nil, 0, false, nil
	}
	startRay := shell.rays[capRays[0]]
	nodes := []revolveLoftFacesTraceNode{{vertex: ring.endpoints[0], ray: capRays[0],
		dir: [2]*big.Rat{startRay.Y, startRay.Z},
		hit: revolveLoftFacesVerticalTraceHit{piece: 0, ray: capRays[0],
			lo: pieces[0].cell.upper.Root.Lo, hi: pieces[0].cell.upper.Root.Hi}}}
	edges := make([]revolveLoftFacesTraceEdge, 0)
	maxBound := 0.0
	for index, piece := range pieces {
		if err := ctx.Err(); err != nil {
			return nil, nil, 0, false, err
		}
		excluded := [2]int{-1, -1}
		if index == 0 {
			excluded[0] = capRays[0]
		}
		if index+1 == len(pieces) {
			excluded[1] = capRays[1]
		}
		hits, ok := revolveLoftFacesTraceSourceHits(piece, shell.rays, excluded)
		if !ok {
			return nil, nil, 0, false, nil
		}
		var targets []revolveLoftFacesTraceNode
		for _, hit := range hits {
			ray := shell.rays[hit.ray]
			held, blankBound, loftBound, valid := revolveLoftFacesVerticalTraceHeldPoint(
				proof, piece, hit, ray, ring.arc.circle.radius)
			if !valid || loftBound >= 0.1 {
				return nil, nil, 0, false, nil
			}
			vertex := len(shell.mesh.Verts)
			shell.mesh.Verts = append(shell.mesh.Verts, held)
			shell.vertexBounds = append(shell.vertexBounds, blankBound)
			maxBound = max(maxBound, loftBound)
			hit.piece = index
			targets = append(targets, revolveLoftFacesTraceNode{vertex: vertex, ray: hit.ray,
				dir: [2]*big.Rat{ray.Y, ray.Z}, hit: hit})
		}
		if index+1 == len(pieces) {
			ray := shell.rays[capRays[1]]
			targets = append(targets, revolveLoftFacesTraceNode{
				vertex: ring.endpoints[1], ray: capRays[1],
				dir: [2]*big.Rat{ray.Y, ray.Z},
				hit: revolveLoftFacesVerticalTraceHit{piece: index, ray: capRays[1],
					lo: piece.cell.upper.Root.Lo, hi: piece.cell.upper.Root.Hi}})
		} else {
			held, bound, direction, valid := revolveLoftFacesTraceJoin(proof, piece, piece.To)
			if !valid || bound >= 0.1 {
				return nil, nil, 0, false, nil
			}
			vertex := len(shell.mesh.Verts)
			shell.mesh.Verts = append(shell.mesh.Verts, held)
			shell.vertexBounds = append(shell.vertexBounds, bound)
			maxBound = max(maxBound, bound)
			targets = append(targets, revolveLoftFacesTraceNode{vertex: vertex, ray: -1,
				dir: direction, hit: revolveLoftFacesVerticalTraceHit{piece: index, ray: -1,
					lo: piece.To, hi: piece.To}})
		}
		for _, target := range targets {
			prior := nodes[len(nodes)-1]
			wedge, valid := revolveLoftFacesTraceWedge(shell.rays, prior.dir, target.dir)
			if !valid || prior.vertex == target.vertex {
				return nil, nil, 0, false, nil
			}
			bound, valid := revolveLoftFacesVerticalTraceSegmentBound(proof, piece,
				prior.hit, target.hit, prior.dir, target.dir,
				shell.mesh.Verts[prior.vertex], shell.mesh.Verts[target.vertex], shell.rays)
			if !valid || bound >= 0.1 {
				return nil, nil, 0, false, nil
			}
			maxBound = max(maxBound, bound)
			edges = append(edges, revolveLoftFacesTraceEdge{start: len(nodes) - 1,
				end: len(nodes), piece: index, wedge: wedge,
				from: prior.hit, to: target.hit})
			nodes = append(nodes, target)
		}
		if index+1 < len(pieces) {
			nodes[len(nodes)-1].hit = revolveLoftFacesVerticalTraceHit{piece: index + 1,
				ray: -1, lo: pieces[index+1].From, hi: pieces[index+1].From}
		}
	}
	return nodes, edges, maxBound, true, nil
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
		len(shell.triangleCells) != len(shell.mesh.Tris) ||
		len(shell.rays) != shell.rayCount {
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
		if candidate.sectionIndex == event.sectionIndex &&
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
	working := *shell
	working.mesh.Verts = slices.Clone(shell.mesh.Verts)
	working.mesh.Tris = slices.Clone(shell.mesh.Tris)
	working.mesh.Src = slices.Clone(shell.mesh.Src)
	working.mesh.Groups = slices.Clone(shell.mesh.Groups)
	working.vertexBounds = slices.Clone(shell.vertexBounds)
	working.triangleCells = slices.Clone(shell.triangleCells)
	nodes, edges, loftBound, built, err := revolveLoftFacesVerticalTracePath(
		ctx, proof, &working, pieces, *ring)
	if err != nil || !built {
		return nil, false, err
	}
	trace := &revolveLoftFacesVerticalTrace{
		sectionIndex: event.sectionIndex, capIDs: event.endpointIDs,
		vertices: make([]int, len(nodes)), vertexRays: make([]int, len(nodes)),
		cellIDs:    make([]int, len(pieces)),
		edgeWedges: make([]int, len(edges)), edgeCellIDs: make([]int, len(edges)),
		loftBound: loftBound}
	for i, piece := range pieces {
		trace.cellIDs[i] = piece.cellID
	}
	for i, node := range nodes {
		trace.vertices[i] = node.vertex
		trace.vertexRays[i] = node.ray
	}
	for i, edge := range edges {
		trace.edgeWedges[i] = edge.wedge
		trace.edgeCellIDs[i] = pieces[edge.piece].cellID
	}
	built, err = revolveLoftFacesVerticalTraceSplitWedges(ctx, &working, *ring,
		event.sectionIndex, nodes, edges)
	if err != nil || !built || len(working.triangleCells) != len(working.mesh.Tris) ||
		!revolveLoftFacesBlankShellEulerTwo(&working.mesh) {
		return nil, false, err
	}
	maxBound := 0.0
	for _, bound := range working.vertexBounds {
		if proofbound.IsNonFinite(bound) || bound < 0 {
			return nil, false, nil
		}
		maxBound = max(maxBound, bound)
	}
	working.faceBound = proofbound.AbsSumUpper(working.chordBound, maxBound)
	if proofbound.IsNonFinite(working.faceBound) || working.faceBound >= 0.1 {
		return nil, false, nil
	}
	if err := orientJoinedMesh(&working.mesh); err != nil {
		return nil, false, err
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx),
		working.mesh.Verts, working.mesh.Tris, len(working.mesh.Tris), 1<<20); err != nil {
		return nil, false, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, working.mesh.Verts, working.mesh.Tris)
	if err != nil {
		return nil, false, err
	}
	if audit == nil || len(audit.Members) != 1 || audit.CompVol[0].Sign() <= 0 {
		return nil, false, nil
	}
	*shell = working
	return trace, true, nil
}

func revolveLoftFacesVerticalTraceHeldPoint(proof *revolveLoftFacesSourceCertificate,
	piece revolveLoftFacesVerticalTracePiece, hit revolveLoftFacesVerticalTraceHit,
	ray revolveloftfaces.ShellRay, radius *big.Rat) (r3.Vec, float64, float64, bool) {
	var empty r3.Vec
	if radius == nil || radius.Sign() <= 0 ||
		piece.cell.band < 0 || piece.cell.band+1 >= len(proof.trim.Maps) {
		return empty, 0, 0, false
	}
	if piece.Arc != nil {
		if piece.Radius2 == nil || piece.Radius2.Sign() <= 0 ||
			piece.Radius2.Cmp(lfMul(radius, radius)) >= 0 {
			return empty, 0, 0, false
		}
		r2, _ := piece.Radius2.Float64()
		if proofbound.IsNonFinite(r2) || r2 <= 0 {
			return empty, 0, 0, false
		}
		approx := lfF(math.Sqrt(r2))
		if approx.Sign() <= 0 {
			return empty, 0, 0, false
		}
		point, blankBound, valid := revolveLoftFacesMeridianRayPoint(
			[2]*big.Rat{ray.Y, ray.Z}, piece.x, approx)
		if !valid {
			return empty, 0, 0, false
		}
		residual := proofbound.RatFloatUp(lfDiv(
			lfAbs(lfSub(lfMul(approx, approx), piece.Radius2)), approx))
		loftBound := proofbound.AbsSumUpper(blankBound, residual)
		return point, blankBound, loftBound,
			!proofbound.IsNonFinite(loftBound) && loftBound >= 0
	}
	if hit.lo == nil || hit.hi == nil || hit.lo.Cmp(hit.hi) > 0 {
		return empty, 0, 0, false
	}
	box, ok := loftface.JunctionBox(proof.trim.Maps[piece.cell.band],
		proof.trim.Maps[piece.cell.band+1], piece.cell.source,
		hit.lo, hit.hi, piece.time, piece.time)
	if !ok || box[0].Lo.Cmp(piece.x) != 0 || box[0].Hi.Cmp(piece.x) != 0 {
		return empty, 0, 0, false
	}
	midY := lfDiv(lfAdd(box[1].Lo, box[1].Hi), big.NewRat(2, 1))
	midZ := lfDiv(lfAdd(box[2].Lo, box[2].Hi), big.NewRat(2, 1))
	denom := lfAdd(lfMul(ray.Y, ray.Y), lfMul(ray.Z, ray.Z))
	if denom.Sign() <= 0 {
		return empty, 0, 0, false
	}
	scale := lfDiv(lfAdd(lfMul(midY, ray.Y), lfMul(midZ, ray.Z)), denom)
	if scale.Sign() <= 0 {
		return empty, 0, 0, false
	}
	exact := loftFaceRatVec{piece.x, lfMul(scale, ray.Y), lfMul(scale, ray.Z)}
	radial2 := lfAdd(lfMul(exact[1], exact[1]), lfMul(exact[2], exact[2]))
	if radial2.Sign() <= 0 || radial2.Cmp(lfMul(radius, radius)) >= 0 {
		return empty, 0, 0, false
	}
	held, blankBound, ok := revolveloftfaces.HeldPoint(exact)
	if !ok || proofbound.IsNonFinite(blankBound) {
		return empty, 0, 0, false
	}
	loftBound, valid := loftface.VerticalTraceBoxDistance(box, held)
	return held, blankBound, loftBound, valid
}
