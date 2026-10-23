package revolveloftfaces

import (
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/polynomial"
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
