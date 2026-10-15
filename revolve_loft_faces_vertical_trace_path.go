package decad

import (
	"context"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

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

func revolveLoftFacesTraceWedge(rays []revolveLoftFacesRay, a, b [2]*big.Rat) (int, bool) {
	if a[0] == nil || b[0] == nil || a[0].Sign() <= 0 || b[0].Sign() <= 0 {
		return -1, false
	}
	mid := [2]*big.Rat{lfAdd(a[0], b[0]), lfAdd(a[1], b[1])}
	wedge := -1
	for i := range rays {
		j := (i + 1) % len(rays)
		left := [2]*big.Rat{rays[i].y, rays[i].z}
		right := [2]*big.Rat{rays[j].y, rays[j].z}
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
	point := revolveLoftFacesVerticalTracePoint(piece, parameter)
	if point[0] == nil || point[1] == nil || point[0].Sign() <= 0 ||
		proof == nil || proof.trim == nil {
		return r3.Vec{}, 0, [2]*big.Rat{}, false
	}
	held, bound, ok := revolveLoftFacesHeldPoint(loftFaceRatVec{piece.x, point[0], point[1]})
	return held, bound, point, ok && !proofbound.IsNonFinite(bound)
}

func revolveLoftFacesTraceSourceHits(piece revolveLoftFacesVerticalTracePiece,
	rays []revolveLoftFacesRay, endpointRays [2]int,
) ([]revolveLoftFacesVerticalTraceHit, bool) {
	hits := make([]revolveLoftFacesVerticalTraceHit, 0)
	for ray := range rays {
		if ray == endpointRays[0] || ray == endpointRays[1] {
			continue
		}
		if rays[ray].y.Sign() <= 0 {
			continue
		}
		hit, found := revolveLoftFacesVerticalTraceRayHit(piece, rays[ray])
		if !found {
			continue
		}
		hit.ray = ray
		if piece.arc == nil {
			if hit.lo == nil || hit.hi == nil ||
				hit.lo.Cmp(piece.from) <= 0 && hit.hi.Cmp(piece.from) >= 0 ||
				hit.lo.Cmp(piece.to) <= 0 && hit.hi.Cmp(piece.to) >= 0 {
				return nil, false
			}
		} else if hit.lo != nil || hit.hi != nil {
			return nil, false
		}
		hits = append(hits, hit)
	}
	sort.Slice(hits, func(i, j int) bool {
		if piece.arc == nil {
			if piece.from.Cmp(piece.to) < 0 {
				return hits[i].lo.Cmp(hits[j].lo) < 0
			}
			return hits[i].lo.Cmp(hits[j].lo) > 0
		}
		a := [2]*big.Rat{rays[hits[i].ray].y, rays[hits[i].ray].z}
		b := [2]*big.Rat{rays[hits[j].ray].y, rays[hits[j].ray].z}
		if piece.from.Cmp(piece.to) < 0 {
			return revolveLoftFacesTraceCross(a, b).Sign() > 0
		}
		return revolveLoftFacesTraceCross(a, b).Sign() < 0
	})
	for i := 1; i < len(hits); i++ {
		if piece.arc == nil {
			if hits[i-1].hi.Cmp(hits[i].lo) >= 0 && piece.from.Cmp(piece.to) < 0 ||
				hits[i].hi.Cmp(hits[i-1].lo) >= 0 && piece.from.Cmp(piece.to) > 0 {
				return nil, false
			}
		} else {
			a := [2]*big.Rat{rays[hits[i-1].ray].y, rays[hits[i-1].ray].z}
			b := [2]*big.Rat{rays[hits[i].ray].y, rays[hits[i].ray].z}
			if revolveLoftFacesTraceCross(a, b).Sign() !=
				-piece.from.Cmp(piece.to) {
				return nil, false
			}
		}
	}
	return hits, true
}

func revolveLoftFacesVerticalTraceSegmentBound(proof *revolveLoftFacesSourceCertificate,
	piece revolveLoftFacesVerticalTracePiece, from, to revolveLoftFacesVerticalTraceHit,
	fromDir, toDir [2]*big.Rat, held, endHeld r3.Vec, rays []revolveLoftFacesRay,
) (float64, bool) {
	if piece.arc != nil {
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
			point := revolveLoftFacesVerticalTracePoint(piece, hit.lo)
			if point[0] == nil || point[1] == nil {
				return 0, false
			}
			box := [3]loftface.Interval{{Lo: piece.x, Hi: piece.x},
				{Lo: point[0], Hi: point[0]}, {Lo: point[1], Hi: point[1]}}
			return revolveLoftFacesVerticalTraceBoxDistance(box, held)
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
		piece.cell.band+1 >= len(proof.trim.maps) || from.lo == nil || from.hi == nil ||
		to.lo == nil || to.hi == nil {
		return 0, false
	}
	if len(piece.y) == 2 {
		endpointBox := func(hit revolveLoftFacesVerticalTraceHit) ([3]loftface.Interval, bool) {
			if hit.lo.Cmp(hit.hi) == 0 {
				point := revolveLoftFacesVerticalTracePoint(piece, hit.lo)
				return [3]loftface.Interval{{Lo: piece.x, Hi: piece.x},
						{Lo: point[0], Hi: point[0]}, {Lo: point[1], Hi: point[1]}},
					point[0] != nil && point[1] != nil
			}
			return loftface.JunctionBox(proof.trim.maps[piece.cell.band],
				proof.trim.maps[piece.cell.band+1], piece.cell.source,
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
		startBound, ok := revolveLoftFacesVerticalTraceBoxDistance(start, held)
		if !ok {
			return 0, false
		}
		endBound, ok := revolveLoftFacesVerticalTraceBoxDistance(end, endHeld)
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
	box, ok := loftface.JunctionBox(proof.trim.maps[piece.cell.band],
		proof.trim.maps[piece.cell.band+1], piece.cell.source,
		lo, hi, piece.time, piece.time)
	if !ok {
		return 0, false
	}
	return revolveLoftFacesVerticalTraceBoxDistance(box, held)
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
		dir: [2]*big.Rat{startRay.y, startRay.z},
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
				dir: [2]*big.Rat{ray.y, ray.z}, hit: hit})
		}
		if index+1 == len(pieces) {
			ray := shell.rays[capRays[1]]
			targets = append(targets, revolveLoftFacesTraceNode{
				vertex: ring.endpoints[1], ray: capRays[1],
				dir: [2]*big.Rat{ray.y, ray.z},
				hit: revolveLoftFacesVerticalTraceHit{piece: index, ray: capRays[1],
					lo: piece.cell.upper.Root.Lo, hi: piece.cell.upper.Root.Hi}})
		} else {
			held, bound, direction, valid := revolveLoftFacesTraceJoin(proof, piece, piece.to)
			if !valid || bound >= 0.1 {
				return nil, nil, 0, false, nil
			}
			vertex := len(shell.mesh.Verts)
			shell.mesh.Verts = append(shell.mesh.Verts, held)
			shell.vertexBounds = append(shell.vertexBounds, bound)
			maxBound = max(maxBound, bound)
			targets = append(targets, revolveLoftFacesTraceNode{vertex: vertex, ray: -1,
				dir: direction, hit: revolveLoftFacesVerticalTraceHit{piece: index, ray: -1,
					lo: piece.to, hi: piece.to}})
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
				ray: -1, lo: pieces[index+1].from, hi: pieces[index+1].from}
		}
	}
	return nodes, edges, maxBound, true, nil
}
