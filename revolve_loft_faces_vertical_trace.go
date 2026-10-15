package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
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
	cellID         int
	cell           *revolveLoftFacesVerticalCell
	piece          loftface.VerticalPiece
	x, time, upper *big.Rat
	from           *big.Rat
	to             *big.Rat
	y, z           []*big.Rat
	arc            *arcSeg
	affine         *loftface.VerticalAffine
	radius2        *big.Rat
	arcBox         [2]loftFaceInterval
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

func revolveLoftFacesVerticalTracePoint(piece revolveLoftFacesVerticalTracePiece,
	s *big.Rat) [2]*big.Rat {
	if piece.arc != nil {
		if s.Cmp(lfZero()) == 0 {
			return revolveLoftFacesVerticalTraceMapPoint(piece.affine, piece.arc.Start)
		}
		if s.Cmp(lfOne()) == 0 {
			return revolveLoftFacesVerticalTraceMapPoint(piece.affine, piece.arc.End)
		}
		return [2]*big.Rat{}
	}
	return [2]*big.Rat{polynomial.BernsteinValue(piece.y, s),
		polynomial.BernsteinValue(piece.z, s)}
}

func revolveLoftFacesVerticalTraceMapPoint(affine *loftface.VerticalAffine,
	point planePoint) [2]*big.Rat {
	u, v := lfF(point.U), lfF(point.V)
	return [2]*big.Rat{
		lfAdd(affine.Offset[0], lfAdd(lfMul(affine.Column[0][0], u), lfMul(affine.Column[1][0], v))),
		lfAdd(affine.Offset[1], lfAdd(lfMul(affine.Column[0][1], u), lfMul(affine.Column[1][1], v))),
	}
}

// A centered circular source maps to a centered circle only under an exact
// positive similarity. This is the class whose ray hit and arc length can be
// bounded without treating a sampled arc point as exact.
func revolveLoftFacesVerticalTraceArcRadius(affine *loftface.VerticalAffine,
	arc arcSeg) (*big.Rat, bool) {
	if affine == nil || affine.Det == nil || affine.Det.Sign() <= 0 {
		return nil, false
	}
	center := revolveLoftFacesVerticalTraceMapPoint(affine, arc.Center)
	if center[0].Sign() != 0 || center[1].Sign() != 0 {
		return nil, false
	}
	u, v := affine.Column[0], affine.Column[1]
	u2 := lfAdd(lfMul(u[0], u[0]), lfMul(u[1], u[1]))
	v2 := lfAdd(lfMul(v[0], v[0]), lfMul(v[1], v[1]))
	if u2.Sign() <= 0 || u2.Cmp(v2) != 0 ||
		lfAdd(lfMul(u[0], v[0]), lfMul(u[1], v[1])).Sign() != 0 {
		return nil, false
	}
	start := revolveLoftFacesVerticalTraceMapPoint(affine, arc.Start)
	end := revolveLoftFacesVerticalTraceMapPoint(affine, arc.End)
	r2 := lfAdd(lfMul(start[0], start[0]), lfMul(start[1], start[1]))
	end2 := lfAdd(lfMul(end[0], end[0]), lfMul(end[1], end[1]))
	return r2, r2.Sign() > 0 && r2.Cmp(end2) == 0
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
		payload.LoftFacesTrim.Source.source == nil {
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
			if cell.segment < 0 || cell.segment >= len(trim.Source.profile.Outer.Segments) ||
				cell.segment >= len(trim.Source.source.entities) {
				return nil, false
			}
			record, recorded := trim.Source.profile.Outer.Segments[cell.segment].(arcSeg)
			live, liveArc := trim.Source.source.entities[cell.segment].(*sketch.Arc)
			if !recorded || !liveArc || !mappedArcRecordMatches(record, live) ||
				cell.span != 0 || cell.band != side.band {
				return nil, false
			}
			simpleCell, simple := revolveLoftFacesSimpleArc(trim, cell.segment,
				record, record.TStart > record.TEnd)
			if !simple {
				return nil, false
			}
			var valid bool
			radius2, valid = revolveLoftFacesVerticalTraceArcRadius(side.yzAffine, record)
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
				cellID: id, cell: cell, piece: piece, x: side.x,
				time: side.time, upper: side.upper,
				y: y, z: z, arc: arc, affine: side.yzAffine,
				radius2: radius2, arcBox: arcBox})
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
			!revolveLoftFacesCapBoundaryBoxEqual(p.cell.upper.Box, boundary.box) {
			return nil, nil, false
		}
		if p.piece.Source.Lo.Cmp(boundary.parameter.Hi) == 0 {
			return p.piece.Source.Lo, p.piece.Source.Hi, true
		}
		if p.piece.Source.Hi.Cmp(boundary.parameter.Lo) == 0 {
			return p.piece.Source.Hi, p.piece.Source.Lo, true
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
				pieces[i].from, pieces[i].to = from, to
			}
		}
		if piece.cellID == arc.endpointCellIDs[1] {
			if from, to, ok := rootEnd(piece, arc.arc.positiveZ); ok {
				if finish >= 0 {
					return nil, false
				}
				finish = i
				// The chain enters the positive cap owner at the opposite end.
				pieces[i].from, pieces[i].to = to, from
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
		end := revolveLoftFacesVerticalTracePoint(piece, piece.to)
		next := -1
		for i := range pieces {
			if used[i] || i == finish && pieces[i].from == nil {
				continue
			}
			candidate := pieces[i]
			if candidate.cellID == piece.cellID || candidate.cell.band != piece.cell.band {
				continue
			}
			for _, endpoint := range [...]struct{ from, to *big.Rat }{
				{candidate.piece.Source.Lo, candidate.piece.Source.Hi},
				{candidate.piece.Source.Hi, candidate.piece.Source.Lo},
			} {
				if i == finish && endpoint.from.Cmp(candidate.from) != 0 ||
					!revolveLoftFacesVerticalTraceSamePoint(end,
						revolveLoftFacesVerticalTracePoint(candidate, endpoint.from)) {
					continue
				}
				if next >= 0 {
					return nil, false
				}
				next = i
				pieces[i].from, pieces[i].to = endpoint.from, endpoint.to
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

// Each strict sign makes a ray meet a source piece at most once. Adjacent
// pieces may reverse direction; the insertion retains every separate hit.
func revolveLoftFacesVerticalTracePieceSigns(pieces []revolveLoftFacesVerticalTracePiece) ([]int, bool) {
	signs := make([]int, len(pieces))
	for index, piece := range pieces {
		if piece.arc != nil {
			// A positive similarity about the blank axis preserves the live
			// arc's counter-clockwise sweep. The certified source box keeps
			// the entire short arc in the positive-Y half-plane.
			if piece.arcBox[0].Lo == nil || piece.arcBox[0].Hi == nil ||
				piece.arcBox[1].Lo == nil || piece.arcBox[1].Hi == nil ||
				piece.piece.Source.Lo.Cmp(lfZero()) != 0 ||
				piece.piece.Source.Hi.Cmp(lfOne()) != 0 {
				return nil, false
			}
			minY := (*big.Rat)(nil)
			for _, u := range []*big.Rat{piece.arcBox[0].Lo, piece.arcBox[0].Hi} {
				for _, v := range []*big.Rat{piece.arcBox[1].Lo, piece.arcBox[1].Hi} {
					y := lfAdd(piece.affine.Offset[0], lfAdd(
						lfMul(piece.affine.Column[0][0], u),
						lfMul(piece.affine.Column[1][0], v)))
					if minY == nil || y.Cmp(minY) < 0 {
						minY = y
					}
				}
			}
			if minY == nil || minY.Sign() <= 0 {
				return nil, false
			}
			sign := 1
			if piece.from.Cmp(piece.to) > 0 {
				sign = -1
			}
			signs[index] = sign
			continue
		}
		y := polynomial.BernsteinRestrict(piece.y, piece.piece.Source.Lo, piece.piece.Source.Hi)
		z := polynomial.BernsteinRestrict(piece.z, piece.piece.Source.Lo, piece.piece.Source.Hi)
		for _, control := range y {
			if control.Sign() <= 0 {
				return nil, false
			}
		}
		angular := polynomial.BernsteinSub(
			polynomial.BernsteinMul(y, polynomial.BernsteinDeriv(z)),
			polynomial.BernsteinMul(z, polynomial.BernsteinDeriv(y)))
		sign := angular[0].Sign()
		if piece.from.Cmp(piece.to) > 0 {
			sign = -sign
		}
		if sign == 0 {
			return nil, false
		}
		for _, control := range angular {
			if control.Sign() != sign && piece.from.Cmp(piece.to) < 0 ||
				control.Sign() != -sign && piece.from.Cmp(piece.to) > 0 {
				return nil, false
			}
		}
		signs[index] = sign
	}
	return signs, len(signs) > 0
}

// The ray root is enclosed by exact rational source parameters. A rational
// crossing is returned as a zero-width interval; otherwise 80 dyadic steps
// bound its full mapped source box before any held point is chosen.
func revolveLoftFacesVerticalTraceRayHit(piece revolveLoftFacesVerticalTracePiece,
	ray revolveLoftFacesRay) (revolveLoftFacesVerticalTraceHit, bool) {
	if ray.y == nil || ray.z == nil {
		return revolveLoftFacesVerticalTraceHit{}, false
	}
	if piece.arc != nil {
		start := revolveLoftFacesVerticalTracePoint(piece, lfZero())
		end := revolveLoftFacesVerticalTracePoint(piece, lfOne())
		before := lfSub(lfMul(start[0], ray.z), lfMul(start[1], ray.y))
		after := lfSub(lfMul(ray.y, end[1]), lfMul(ray.z, end[0]))
		return revolveLoftFacesVerticalTraceHit{}, before.Sign() > 0 && after.Sign() > 0
	}
	cross := make([]*big.Rat, len(piece.y))
	for i := range cross {
		cross[i] = lfSub(lfMul(piece.y[i], ray.z), lfMul(piece.z[i], ray.y))
	}
	lo, hi := piece.piece.Source.Lo, piece.piece.Source.Hi
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
