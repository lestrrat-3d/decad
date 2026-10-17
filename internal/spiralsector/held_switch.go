package spiralsector

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// HeldSwitch is a complete, simple source-face switch on the held
// Boolean boundary. The ordered vertices make both sides of an angular chart
// available to the separate exact-source correspondence proof.
type HeldSwitch struct {
	Vertices   []int
	X          loftface.Interval
	box        [3]loftface.Interval
	slope      [2]*big.Rat
	MinBeta    float64
	Increasing bool
}

// HeldHeelSwitch accepts only one angularly ordered cap/root chain.
// A closed output audit remains the caller's prerequisite; this gate checks
// the local face identities and chain topology before any smaller source-
// specific contact allowance may be used.
func HeldHeelSwitch(ctx context.Context, held *meshbool.StitchedMesh,
	blankHeel, toothRoot int) (HeldSwitch, bool, error) {
	return heldSwitchPath(ctx, held, blankHeel, toothRoot, true)
}

// HeldEdge is one undirected held edge, lo < hi.
type HeldEdge struct{ lo, hi int }

// HeldEdgeOwners holds the source groups of an edge's first two facets,
// in facet order, and how many facets use the edge.
type HeldEdgeOwners struct {
	src   [2]int
	count uint8
}

// HeldEdges is the facet-owner table of one held mesh. It depends only
// on the mesh, so a caller may build it once and use it for several switches
// of that unchanged mesh within one call. ok is false when an index is
// invalid or an edge does not have exactly two facets.
func HeldEdges(ctx context.Context,
	held *meshbool.StitchedMesh) (map[HeldEdge]HeldEdgeOwners, bool, error) {
	if ctx == nil || held == nil || len(held.Tris) == 0 || len(held.Tris) > 1<<17 ||
		len(held.Src) != len(held.Tris) || len(held.VertexBound) != len(held.Verts) {
		return nil, false, nil
	}
	owners := make(map[HeldEdge]HeldEdgeOwners, len(held.Tris)*3/2)
	for i, tri := range held.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
		}
		for side := range 3 {
			a, b := tri[side], tri[(side+1)%3]
			if a < 0 || b < 0 || a >= len(held.Verts) || b >= len(held.Verts) || a == b {
				return nil, false, nil
			}
			if a > b {
				a, b = b, a
			}
			key := HeldEdge{a, b}
			owner := owners[key]
			if owner.count >= 2 {
				return nil, false, nil
			}
			owner.src[owner.count] = held.Src[i]
			owner.count++
			owners[key] = owner
		}
	}
	seen := 0
	for _, owner := range owners {
		if seen%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
		}
		seen++
		if owner.count != 2 {
			return nil, false, nil
		}
	}
	return owners, true, nil
}

func heldSwitchPath(ctx context.Context, held *meshbool.StitchedMesh,
	blankHeel, toothRoot int, monotone bool) (HeldSwitch, bool, error) {
	if blankHeel < 0 || toothRoot < 0 || blankHeel == toothRoot {
		return HeldSwitch{}, false, nil
	}
	owners, ok, err := HeldEdges(ctx, held)
	if err != nil || !ok {
		return HeldSwitch{}, false, err
	}
	return HeldSwitchFrom(ctx, held, owners, blankHeel, toothRoot, monotone)
}

// HeldSwitchFrom is heldSwitchPath over a table that
// HeldEdges built from this same, unchanged held mesh.
func HeldSwitchFrom(ctx context.Context, held *meshbool.StitchedMesh,
	owners map[HeldEdge]HeldEdgeOwners,
	blankHeel, toothRoot int, monotone bool) (HeldSwitch, bool, error) {
	if blankHeel < 0 || toothRoot < 0 || blankHeel == toothRoot {
		return HeldSwitch{}, false, nil
	}
	adj := make(map[int][]int)
	seamEdges := 0
	seen := 0
	for key, owner := range owners {
		if seen%256 == 0 {
			if err := ctx.Err(); err != nil {
				return HeldSwitch{}, false, err
			}
		}
		seen++
		pair := owner.src
		if pair[0] != blankHeel || pair[1] != toothRoot {
			if pair[0] != toothRoot || pair[1] != blankHeel {
				continue
			}
		}
		adj[key.lo] = append(adj[key.lo], key.hi)
		adj[key.hi] = append(adj[key.hi], key.lo)
		seamEdges++
	}
	if seamEdges < 1 || seamEdges > 4096 {
		return HeldSwitch{}, false, nil
	}
	starts := make([]int, 0, 2)
	for vertex, neighbors := range adj {
		switch len(neighbors) {
		case 1:
			starts = append(starts, vertex)
		case 2:
		default:
			return HeldSwitch{}, false, nil
		}
	}
	if len(starts) != 2 || len(adj) != seamEdges+1 {
		return HeldSwitch{}, false, nil
	}
	ordered := make([]int, 0, len(adj))
	previous, current := -1, starts[0]
	sign := 0
	minBeta := math.Inf(1)
	var x loftface.Interval
	var box [3]loftface.Interval
	for len(ordered) < len(adj) {
		if len(ordered)%128 == 0 {
			if err := ctx.Err(); err != nil {
				return HeldSwitch{}, false, err
			}
		}
		p := held.Verts[current]
		beta := held.VertexBound[current]
		if !proofbound.FiniteVec(p) || p.Y <= 0 ||
			proofbound.IsNonFinite(beta) || beta < 0 {
			return HeldSwitch{}, false, nil
		}
		if len(ordered) == 0 {
			x = loftface.Interval{Lo: ratF(p.X), Hi: ratF(p.X)}
		} else {
			if ratF(p.X).Cmp(x.Lo) < 0 {
				x.Lo = ratF(p.X)
			}
			if ratF(p.X).Cmp(x.Hi) > 0 {
				x.Hi = ratF(p.X)
			}
		}
		for axis, coordinate := range [...]float64{p.X, p.Y, p.Z} {
			value := ratF(coordinate)
			if box[axis].Lo == nil || value.Cmp(box[axis].Lo) < 0 {
				box[axis].Lo = value
			}
			if box[axis].Hi == nil || value.Cmp(box[axis].Hi) > 0 {
				box[axis].Hi = value
			}
		}
		minBeta = min(minBeta, beta)
		ordered = append(ordered, current)
		next := -1
		for _, neighbor := range adj[current] {
			if neighbor != previous {
				next = neighbor
				break
			}
		}
		if next < 0 {
			break
		}
		q := held.Verts[next]
		if !proofbound.FiniteVec(q) || q.Y <= 0 {
			return HeldSwitch{}, false, nil
		}
		cross := proofarith.SubRat(new(big.Rat),
			proofarith.MulRat(new(big.Rat), ratF(p.Y), ratF(q.Z)),
			proofarith.MulRat(new(big.Rat), ratF(p.Z), ratF(q.Y))).Sign()
		if cross == 0 || monotone && sign != 0 && cross != sign {
			return HeldSwitch{}, false, nil
		}
		sign = cross
		previous, current = current, next
	}
	if len(ordered) != len(adj) || ordered[len(ordered)-1] != starts[1] ||
		proofbound.IsNonFinite(minBeta) || minBeta <= 0 {
		return HeldSwitch{}, false, nil
	}
	first, last := held.Verts[ordered[0]], held.Verts[ordered[len(ordered)-1]]
	return HeldSwitch{Vertices: ordered, X: x, box: box,
		slope: [2]*big.Rat{
			proofarith.QuoRat(new(big.Rat), ratF(first.Z), ratF(first.Y)),
			proofarith.QuoRat(new(big.Rat), ratF(last.Z), ratF(last.Y)),
		},
		MinBeta: minBeta, Increasing: sign > 0}, true, nil
}
