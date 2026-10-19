package revolveloftfaces

import (
	"context"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// VerticalTraceHit encloses one source crossing by exact source parameters.
// Ray is -1 for a source junction inside a wedge.
type VerticalTraceHit struct {
	Piece  int
	Ray    int
	Lo, Hi *big.Rat
}

// VerticalTraceNode is one held path vertex with its exact radial direction.
type VerticalTraceNode struct {
	Vertex int
	Ray    int // -1 for a source junction inside a wedge
	Dir    [2]*big.Rat
	Hit    VerticalTraceHit
}

// VerticalTraceEdge joins two consecutive path nodes inside one ray wedge.
type VerticalTraceEdge struct {
	Start, End int
	Piece      int
	Wedge      int
	From, To   VerticalTraceHit
}

// VerticalTraceHeldPath is a held vertical path. Verts and VertexBounds are
// the caller's shell vertices and bounds with the path's new vertices
// appended; LoftBound is the largest source-to-held displacement bound.
type VerticalTraceHeldPath struct {
	Nodes        []VerticalTraceNode
	Edges        []VerticalTraceEdge
	Verts        []r3.Vec
	VertexBounds []float64
	LoftBound    float64
}

// VerticalTraceControl maps a source span's two to four control points to
// exact world Y and Z through a vertical side's affine map.
func VerticalTraceControl(span freeform.BezierSpan,
	affine *loftface.VerticalAffine) ([]*big.Rat, []*big.Rat, bool) {
	if affine == nil || len(span) < 2 || len(span) > 4 {
		return nil, nil, false
	}
	y, z := make([]*big.Rat, len(span)), make([]*big.Rat, len(span))
	for i, point := range span {
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

// VerticalTraceSamePoint reports exact equality of two mapped Y/Z points.
func VerticalTraceSamePoint(a, b [2]*big.Rat) bool {
	return a[0].Cmp(b[0]) == 0 && a[1].Cmp(b[1]) == 0
}

// The ray root is enclosed by exact rational source parameters. A rational
// crossing is returned as a zero-width interval; otherwise 80 dyadic steps
// bound its full mapped source box before any held point is chosen.
func verticalTraceRayHit(piece VerticalTraceSourcePiece, ray ShellRay) (VerticalTraceHit, bool) {
	if ray.Y == nil || ray.Z == nil {
		return VerticalTraceHit{}, false
	}
	if piece.Arc != nil {
		start := VerticalTraceSourcePoint(piece, lfZero())
		end := VerticalTraceSourcePoint(piece, lfOne())
		before := lfSub(lfMul(start[0], ray.Z), lfMul(start[1], ray.Y))
		after := lfSub(lfMul(ray.Y, end[1]), lfMul(ray.Z, end[0]))
		return VerticalTraceHit{}, before.Sign() > 0 && after.Sign() > 0
	}
	cross := make([]*big.Rat, len(piece.Y))
	for i := range cross {
		cross[i] = lfSub(lfMul(piece.Y[i], ray.Z), lfMul(piece.Z[i], ray.Y))
	}
	lo, hi := piece.Piece.Source.Lo, piece.Piece.Source.Hi
	a, b := polynomial.BernsteinSign(cross, lo), polynomial.BernsteinSign(cross, hi)
	if a == 0 {
		return VerticalTraceHit{Lo: lo, Hi: lo}, true
	}
	if b == 0 {
		return VerticalTraceHit{Lo: hi, Hi: hi}, true
	}
	if a == b {
		return VerticalTraceHit{}, false
	}
	for range 80 {
		mid := lfDiv(lfAdd(lo, hi), big.NewRat(2, 1))
		sign := polynomial.BernsteinSign(cross, mid)
		if sign == 0 {
			return VerticalTraceHit{Lo: mid, Hi: mid}, true
		}
		if sign == a {
			lo = mid
		} else {
			hi = mid
		}
	}
	return VerticalTraceHit{Lo: lo, Hi: hi}, true
}

func verticalTraceCross(a, b [2]*big.Rat) *big.Rat {
	return lfSub(lfMul(a[0], b[1]), lfMul(a[1], b[0]))
}

func verticalTraceWedge(rays []ShellRay, a, b [2]*big.Rat) (int, bool) {
	if a[0] == nil || b[0] == nil || a[0].Sign() <= 0 || b[0].Sign() <= 0 {
		return -1, false
	}
	mid := [2]*big.Rat{lfAdd(a[0], b[0]), lfAdd(a[1], b[1])}
	wedge := -1
	for i := range rays {
		j := (i + 1) % len(rays)
		left := [2]*big.Rat{rays[i].Y, rays[i].Z}
		right := [2]*big.Rat{rays[j].Y, rays[j].Z}
		if verticalTraceCross(left, right).Sign() <= 0 ||
			verticalTraceCross(left, mid).Sign() <= 0 ||
			verticalTraceCross(mid, right).Sign() <= 0 {
			continue
		}
		if verticalTraceCross(left, a).Sign() < 0 ||
			verticalTraceCross(a, right).Sign() < 0 ||
			verticalTraceCross(left, b).Sign() < 0 ||
			verticalTraceCross(b, right).Sign() < 0 {
			continue
		}
		if wedge >= 0 {
			return -1, false
		}
		wedge = i
	}
	return wedge, wedge >= 0
}

// VerticalTraceJoin holds the mapped source point at a junction parameter
// between two path pieces. It returns the held point, its bound and the exact
// radial direction.
func VerticalTraceJoin(piece VerticalTraceSourcePiece, parameter *big.Rat,
) (r3.Vec, float64, [2]*big.Rat, bool) {
	point := VerticalTraceSourcePoint(piece, parameter)
	if point[0] == nil || point[1] == nil || point[0].Sign() <= 0 {
		return r3.Vec{}, 0, [2]*big.Rat{}, false
	}
	held, bound, ok := HeldPoint(loftface.RatVec{piece.X, point[0], point[1]})
	return held, bound, point, ok && !proofbound.IsNonFinite(bound)
}

// VerticalTraceSourceHits finds the piece's crossing with every ray in the
// positive-Y half-plane except the two endpoint rays, ordered along the
// piece from From to To. A line crossing at either end refuses, as do two
// rays claiming one crossing.
func VerticalTraceSourceHits(piece VerticalTraceSourcePiece,
	rays []ShellRay, endpointRays [2]int,
) ([]VerticalTraceHit, bool) {
	hits := make([]VerticalTraceHit, 0)
	for ray := range rays {
		if ray == endpointRays[0] || ray == endpointRays[1] {
			continue
		}
		if rays[ray].Y.Sign() <= 0 {
			continue
		}
		hit, found := verticalTraceRayHit(piece, rays[ray])
		if !found {
			continue
		}
		hit.Ray = ray
		if piece.Arc == nil {
			if hit.Lo == nil || hit.Hi == nil ||
				hit.Lo.Cmp(piece.From) <= 0 && hit.Hi.Cmp(piece.From) >= 0 ||
				hit.Lo.Cmp(piece.To) <= 0 && hit.Hi.Cmp(piece.To) >= 0 {
				return nil, false
			}
		} else if hit.Lo != nil || hit.Hi != nil {
			return nil, false
		}
		hits = append(hits, hit)
	}
	sort.Slice(hits, func(i, j int) bool {
		if piece.Arc == nil {
			if piece.From.Cmp(piece.To) < 0 {
				return hits[i].Lo.Cmp(hits[j].Lo) < 0
			}
			return hits[i].Lo.Cmp(hits[j].Lo) > 0
		}
		a := [2]*big.Rat{rays[hits[i].Ray].Y, rays[hits[i].Ray].Z}
		b := [2]*big.Rat{rays[hits[j].Ray].Y, rays[hits[j].Ray].Z}
		if piece.From.Cmp(piece.To) < 0 {
			return verticalTraceCross(a, b).Sign() > 0
		}
		return verticalTraceCross(a, b).Sign() < 0
	})
	for i := 1; i < len(hits); i++ {
		if piece.Arc == nil {
			if hits[i-1].Hi.Cmp(hits[i].Lo) >= 0 && piece.From.Cmp(piece.To) < 0 ||
				hits[i].Hi.Cmp(hits[i-1].Lo) >= 0 && piece.From.Cmp(piece.To) > 0 {
				return nil, false
			}
		} else {
			a := [2]*big.Rat{rays[hits[i-1].Ray].Y, rays[hits[i-1].Ray].Z}
			b := [2]*big.Rat{rays[hits[i].Ray].Y, rays[hits[i].Ray].Z}
			if verticalTraceCross(a, b).Sign() !=
				-piece.From.Cmp(piece.To) {
				return nil, false
			}
		}
	}
	return hits, true
}

// VerticalTraceSegmentBound bounds the source-to-held displacement of one
// path segment from held to endHeld: by exact endpoint or junction boxes for
// a line piece, and by the endpoint bounds plus the chord sagitta for an arc.
// Maps are the source LoftFaces band maps.
func VerticalTraceSegmentBound(maps []loftface.Map,
	piece VerticalTraceSourcePiece, from, to VerticalTraceHit,
	fromDir, toDir [2]*big.Rat, held, endHeld r3.Vec, rays []ShellRay,
) (float64, bool) {
	if piece.Arc != nil {
		if piece.UpperRadius == nil || piece.UpperRadius.Sign() <= 0 ||
			fromDir[0] == nil || toDir[0] == nil {
			return 0, false
		}
		dot := lfAdd(lfMul(fromDir[0], toDir[0]), lfMul(fromDir[1], toDir[1]))
		cross := lfAbs(verticalTraceCross(fromDir, toDir))
		if dot.Sign() <= 0 || cross.Sign() == 0 || cross.Cmp(dot) >= 0 {
			return 0, false
		}
		endpointDistance := func(hit VerticalTraceHit, held r3.Vec) (float64, bool) {
			if hit.Ray >= 0 {
				if hit.Ray >= len(rays) {
					return 0, false
				}
				point, _, bound, ok := verticalTraceHeldPoint(
					maps, piece, hit, rays[hit.Ray], piece.UpperRadius)
				return bound, ok && point == held
			}
			if hit.Lo == nil || hit.Hi == nil || hit.Lo.Cmp(hit.Hi) != 0 {
				return 0, false
			}
			point := VerticalTraceSourcePoint(piece, hit.Lo)
			if point[0] == nil || point[1] == nil {
				return 0, false
			}
			box := [3]loftface.Interval{{Lo: piece.X, Hi: piece.X},
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
			lfMul(piece.UpperRadius, lfMul(q, q)), big.NewRat(8, 1)))
		bound := proofbound.AbsSumUpper(max(startDistance, endDistance), sagitta)
		return bound, !proofbound.IsNonFinite(bound) && bound >= 0
	}
	if piece.Band < 0 || piece.Band+1 >= len(maps) || from.Lo == nil || from.Hi == nil ||
		to.Lo == nil || to.Hi == nil {
		return 0, false
	}
	if len(piece.Y) == 2 {
		endpointBox := func(hit VerticalTraceHit) ([3]loftface.Interval, bool) {
			if hit.Lo.Cmp(hit.Hi) == 0 {
				point := VerticalTraceSourcePoint(piece, hit.Lo)
				return [3]loftface.Interval{{Lo: piece.X, Hi: piece.X},
						{Lo: point[0], Hi: point[0]}, {Lo: point[1], Hi: point[1]}},
					point[0] != nil && point[1] != nil
			}
			return loftface.JunctionBox(maps[piece.Band], maps[piece.Band+1], piece.Span,
				hit.Lo, hit.Hi, piece.Time, piece.Time)
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
	lo, hi := from.Lo, to.Hi
	if lo.Cmp(to.Lo) > 0 {
		lo = to.Lo
	}
	if hi.Cmp(from.Hi) < 0 {
		hi = from.Hi
	}
	if lo.Cmp(hi) >= 0 {
		return 0, false
	}
	box, ok := loftface.JunctionBox(maps[piece.Band], maps[piece.Band+1], piece.Span,
		lo, hi, piece.Time, piece.Time)
	if !ok {
		return 0, false
	}
	return loftface.VerticalTraceBoxDistance(box, held)
}

// VerticalTracePath holds every ray crossing and junction of the ordered
// pieces between the two cap endpoint vertices of one cap ring. The ring
// lists one shell vertex per ray; radius is the cap circle's radius. Each
// consecutive pair of path vertices must fit one ray wedge, and every held
// vertex and segment bound must stay below 0.1 mm.
func VerticalTracePath(ctx context.Context, maps []loftface.Map, rays []ShellRay,
	verts []r3.Vec, vertexBounds []float64, ring []int, endpoints [2]int, radius *big.Rat,
	pieces []VerticalTraceSourcePiece,
) (VerticalTraceHeldPath, bool, error) {
	var empty VerticalTraceHeldPath
	if len(pieces) == 0 {
		return empty, false, nil
	}
	capRays := [2]int{-1, -1}
	for endpoint := range capRays {
		capRays[endpoint] = -1
		for ray, vertex := range ring {
			if vertex == endpoints[endpoint] {
				if capRays[endpoint] >= 0 {
					return empty, false, nil
				}
				capRays[endpoint] = ray
			}
		}
		if capRays[endpoint] < 0 {
			return empty, false, nil
		}
	}
	if capRays[0] == capRays[1] {
		return empty, false, nil
	}
	startRay := rays[capRays[0]]
	nodes := []VerticalTraceNode{{Vertex: endpoints[0], Ray: capRays[0],
		Dir: [2]*big.Rat{startRay.Y, startRay.Z},
		Hit: VerticalTraceHit{Piece: 0, Ray: capRays[0],
			Lo: pieces[0].UpperRoot.Lo, Hi: pieces[0].UpperRoot.Hi}}}
	edges := make([]VerticalTraceEdge, 0)
	maxBound := 0.0
	for index, piece := range pieces {
		if err := ctx.Err(); err != nil {
			return empty, false, err
		}
		excluded := [2]int{-1, -1}
		if index == 0 {
			excluded[0] = capRays[0]
		}
		if index+1 == len(pieces) {
			excluded[1] = capRays[1]
		}
		hits, ok := VerticalTraceSourceHits(piece, rays, excluded)
		if !ok {
			return empty, false, nil
		}
		var targets []VerticalTraceNode
		for _, hit := range hits {
			ray := rays[hit.Ray]
			held, blankBound, loftBound, valid := verticalTraceHeldPoint(
				maps, piece, hit, ray, radius)
			if !valid || loftBound >= 0.1 {
				return empty, false, nil
			}
			vertex := len(verts)
			verts = append(verts, held)
			vertexBounds = append(vertexBounds, blankBound)
			maxBound = max(maxBound, loftBound)
			hit.Piece = index
			targets = append(targets, VerticalTraceNode{Vertex: vertex, Ray: hit.Ray,
				Dir: [2]*big.Rat{ray.Y, ray.Z}, Hit: hit})
		}
		if index+1 == len(pieces) {
			ray := rays[capRays[1]]
			targets = append(targets, VerticalTraceNode{
				Vertex: endpoints[1], Ray: capRays[1],
				Dir: [2]*big.Rat{ray.Y, ray.Z},
				Hit: VerticalTraceHit{Piece: index, Ray: capRays[1],
					Lo: piece.UpperRoot.Lo, Hi: piece.UpperRoot.Hi}})
		} else {
			held, bound, direction, valid := VerticalTraceJoin(piece, piece.To)
			if !valid || bound >= 0.1 {
				return empty, false, nil
			}
			vertex := len(verts)
			verts = append(verts, held)
			vertexBounds = append(vertexBounds, bound)
			maxBound = max(maxBound, bound)
			targets = append(targets, VerticalTraceNode{Vertex: vertex, Ray: -1,
				Dir: direction, Hit: VerticalTraceHit{Piece: index, Ray: -1,
					Lo: piece.To, Hi: piece.To}})
		}
		for _, target := range targets {
			prior := nodes[len(nodes)-1]
			wedge, valid := verticalTraceWedge(rays, prior.Dir, target.Dir)
			if !valid || prior.Vertex == target.Vertex {
				return empty, false, nil
			}
			bound, valid := VerticalTraceSegmentBound(maps, piece,
				prior.Hit, target.Hit, prior.Dir, target.Dir,
				verts[prior.Vertex], verts[target.Vertex], rays)
			if !valid || bound >= 0.1 {
				return empty, false, nil
			}
			maxBound = max(maxBound, bound)
			edges = append(edges, VerticalTraceEdge{Start: len(nodes) - 1,
				End: len(nodes), Piece: index, Wedge: wedge,
				From: prior.Hit, To: target.Hit})
			nodes = append(nodes, target)
		}
		if index+1 < len(pieces) {
			nodes[len(nodes)-1].Hit = VerticalTraceHit{Piece: index + 1,
				Ray: -1, Lo: pieces[index+1].From, Hi: pieces[index+1].From}
		}
	}
	return VerticalTraceHeldPath{Nodes: nodes, Edges: edges, Verts: verts,
		VertexBounds: vertexBounds, LoftBound: maxBound}, true, nil
}

func verticalTraceHeldPoint(maps []loftface.Map, piece VerticalTraceSourcePiece,
	hit VerticalTraceHit, ray ShellRay, radius *big.Rat) (r3.Vec, float64, float64, bool) {
	var empty r3.Vec
	if radius == nil || radius.Sign() <= 0 ||
		piece.Band < 0 || piece.Band+1 >= len(maps) {
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
		point, blankBound, valid := MeridianRayPoint(
			[2]*big.Rat{ray.Y, ray.Z}, piece.X, approx)
		if !valid {
			return empty, 0, 0, false
		}
		residual := proofbound.RatFloatUp(lfDiv(
			lfAbs(lfSub(lfMul(approx, approx), piece.Radius2)), approx))
		loftBound := proofbound.AbsSumUpper(blankBound, residual)
		return point, blankBound, loftBound,
			!proofbound.IsNonFinite(loftBound) && loftBound >= 0
	}
	if hit.Lo == nil || hit.Hi == nil || hit.Lo.Cmp(hit.Hi) > 0 {
		return empty, 0, 0, false
	}
	box, ok := loftface.JunctionBox(maps[piece.Band], maps[piece.Band+1], piece.Span,
		hit.Lo, hit.Hi, piece.Time, piece.Time)
	if !ok || box[0].Lo.Cmp(piece.X) != 0 || box[0].Hi.Cmp(piece.X) != 0 {
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
	exact := loftface.RatVec{piece.X, lfMul(scale, ray.Y), lfMul(scale, ray.Z)}
	radial2 := lfAdd(lfMul(exact[1], exact[1]), lfMul(exact[2], exact[2]))
	if radial2.Sign() <= 0 || radial2.Cmp(lfMul(radius, radius)) >= 0 {
		return empty, 0, 0, false
	}
	held, blankBound, ok := HeldPoint(exact)
	if !ok || proofbound.IsNonFinite(blankBound) {
		return empty, 0, 0, false
	}
	loftBound, valid := loftface.VerticalTraceBoxDistance(box, held)
	return held, blankBound, loftBound, valid
}
