package meshbool

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
)

// This file keeps a rim vertex's large bound next to the rim. A kept piece of
// an operand facet whose corners are a rim vertex and a far operand corner is
// one long triangle, and the facet bound δ(t) of agent-docs/faceted-vertex-bounds-design.md
// §2 is the largest corner bound, so the rim's bound would stand for the whole
// triangle and the next boolean to cross it, however far from the rim, would
// read it. SplitSpread splits such a triangle so that the part away from the
// rim has only corners that carry the operand facet's own bound (§3.7).

const (
	// spreadRatio is how far above the operand facet's own bound a corner must
	// stand to count as a rim corner of its piece.
	spreadRatio = 2.0
	// spreadReach is how many times a rim corner's bound the nearest split
	// point stands from it, at the least.
	spreadReach = 4.0
)

// BudgetFunc charges the work and the records of a bounded replay. A nil
// BudgetFunc charges nothing.
type BudgetFunc func(work, records uint64) error

// SplitSpread returns kept with each piece that has rim corners and operand
// corners split so that the facets away from the rim hold only the operand
// facet's own bound. a and b are the operands kept came from; a piece of the
// second operand is read against a, and one of the first against b. A rim
// corner is one whose bound is above floor, the pair's chord tolerance, and
// above twice the operand facet's bound.
//
// A split point x stands on an edge from a rim corner u to another corner v
// of one piece of operand facet t, so it lies on t. It takes β(x) = δ(t)
// (agent-docs/faceted-vertex-bounds-design.md §3.3) only where the true piece
// of t within δ(t) of x is still part of the result boundary: where x is
// farther than δ(t) + δ_o from the held boundary of the other operand, δ_o the
// largest bound that operand holds. The held and the true boundary of the
// other operand differ by at most δ_o, so the segment from x to that true point
// meets neither. Where no point of the edge passes the test, the piece is kept
// whole, which is the result without this pass. The split adds vertices and
// never lowers a bound a rim vertex or an operand corner carries.
//
// A piece on a planar face is never split: a split point lies on no boundary
// loop of its face, and a retained blend re-triangulates a face from its loops.
// charge, when not nil, is charged the work and records of a bounded replay.
func SplitSpread(ctx context.Context, kept []KeptFacet, a, b *BoolMesh, floor float64, charge BudgetFunc) ([]KeptFacet, error) {
	maxDelta := [2]float64{}
	for i, m := range [2]*BoolMesh{a, b} {
		for _, d := range m.FacetBound {
			maxDelta[i] = max(maxDelta[i], d)
		}
	}
	var out []KeptFacet
	var cut [2]map[spreadEdge]*proof.Xpt
	cut[0], cut[1] = map[spreadEdge]*proof.Xpt{}, map[spreadEdge]*proof.Xpt{}
	var hint int
	onCurvedFace := [2]map[int]bool{{}, {}}
	for i := range kept {
		f := &kept[i]
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		var hi, lo []int
		for k := range 3 {
			if f.Beta[k] > max(spreadRatio*f.Delta, floor) {
				hi = append(hi, k)
			} else {
				lo = append(lo, k)
			}
		}
		if len(hi) == 0 || len(hi) == 3 {
			if out != nil {
				out = append(out, *f)
			}
			continue
		}
		side, own, other := 0, a, b
		if f.Second {
			side, own, other = 1, b, a
		}
		curved, seen := onCurvedFace[side][f.Facet]
		if !seen {
			curved = ownsCurvedFace(own, f.Facet)
			onCurvedFace[side][f.Facet] = curved
		}
		if !curved {
			if out != nil {
				out = append(out, *f)
			}
			continue
		}
		pts := map[[2]int]proof.Xpt{}
		whole := false
		for _, h := range hi {
			for _, l := range lo {
				x, ok, err := spreadPoint(ctx, f, h, l, other, f.Delta+maxDelta[1-side], cut[side], &hint, charge)
				if err != nil {
					return nil, err
				}
				if !ok {
					whole = true
					break
				}
				pts[[2]int{h, l}] = x
			}
			if whole {
				break
			}
		}
		if whole {
			if out != nil {
				out = append(out, *f)
			}
			continue
		}
		if out == nil {
			out = make([]KeptFacet, i, len(kept)+len(kept)/4)
			copy(out, kept[:i])
		}
		if charge != nil {
			if err := charge(3, 3); err != nil {
				return nil, err
			}
		}
		out = appendSplitPieces(out, f, hi, lo, pts)
	}
	if out == nil {
		return kept, nil
	}
	return out, nil
}

// ownsCurvedFace reports whether facet i of m lies on a face that is not
// planar: a facet that shares an edge with another facet of the same source
// face off its own plane. A split point is a vertex of its piece's face that no
// boundary loop of the face reaches. A planar face is re-triangulated from its
// boundary loops alone (internal/retainedblend), so such a vertex is no part of
// its model, and a split never adds one to a planar face.
func ownsCurvedFace(m *BoolMesh, i int) bool {
	tri := m.Tris[i]
	for k := range 3 {
		j, ok := m.TwinFacet(i, k)
		if !ok || m.Src[j] != m.Src[i] {
			continue
		}
		u, v := tri[k], tri[(k+1)%3]
		apex := m.Tris[j][0] + m.Tris[j][1] + m.Tris[j][2] - u - v
		if proof.OrientSignExact(m.Xverts[tri[0]], m.Xverts[tri[1]], m.Xverts[tri[2]], m.Xverts[apex]) != 0 {
			return true
		}
	}
	return false
}

// appendSplitPieces appends the pieces of f cut at pts, which holds the split
// point of every edge from a rim corner in hi to a corner in lo. The pieces
// keep f's orientation.
func appendSplitPieces(out []KeptFacet, f *KeptFacet, hi, lo []int, pts map[[2]int]proof.Xpt) []KeptFacet {
	d := f.Delta
	add := func(v [3]proof.Xpt, beta [3]float64) {
		out = append(out, KeptFacet{V: v, Src: f.Src, Beta: beta, Delta: d, Second: f.Second})
	}
	if len(hi) == 1 {
		u := hi[0]
		p, q := (u+1)%3, (u+2)%3
		x, y := pts[[2]int{u, p}], pts[[2]int{u, q}]
		add([3]proof.Xpt{f.V[u], x, y}, [3]float64{f.Beta[u], d, d})
		add([3]proof.Xpt{x, f.V[p], f.V[q]}, [3]float64{d, f.Beta[p], f.Beta[q]})
		add([3]proof.Xpt{x, f.V[q], y}, [3]float64{d, f.Beta[q], d})
		return out
	}
	w := lo[0]
	u1, u2 := (w+1)%3, (w+2)%3
	x1, x2 := pts[[2]int{u1, w}], pts[[2]int{u2, w}]
	add([3]proof.Xpt{f.V[u1], f.V[u2], x2}, [3]float64{f.Beta[u1], f.Beta[u2], d})
	add([3]proof.Xpt{f.V[u1], x2, x1}, [3]float64{f.Beta[u1], d, d})
	add([3]proof.Xpt{x1, x2, f.V[w]}, [3]float64{d, d, f.Beta[w]})
	return out
}

// spreadEdge names a split edge: its rim corner and other corner by exact key,
// and the bound of the operand facet it lies on, which sets the clearance.
type spreadEdge struct {
	hi, lo string
	delta  float64
}

// spreadPoint finds the split point of the edge from corner hi to corner lo of
// f: the point at 1/2^j of the edge from hi with the largest j that keeps it at
// least spreadReach times hi's bound from hi and farther than clearance from
// other's held boundary. The answer is memoized per edge by the corner keys.
func spreadPoint(ctx context.Context, f *KeptFacet, hi, lo int, other *BoolMesh, clearance float64,
	memo map[spreadEdge]*proof.Xpt, hint *int, charge BudgetFunc) (proof.Xpt, bool, error) {
	u, v := f.V[hi], f.V[lo]
	key := spreadEdge{hi: f.Keys[hi], lo: f.Keys[lo], delta: f.Delta}
	memoized := key.hi != "" && key.lo != ""
	if memoized {
		if p, ok := memo[key]; ok {
			if p == nil {
				return proof.Xpt{}, false, nil
			}
			return *p, true, nil
		}
	}
	reach := spreadReach * f.Beta[hi]
	length := v.Vec().Sub(u.Vec()).Len()
	var found *proof.Xpt
	if length > 2*reach && !math.IsInf(length, 0) {
		for j := int(math.Floor(math.Log2(length / reach))); j >= 1; j-- {
			if charge != nil {
				if err := charge(uint64(len(other.Tris)), 1); err != nil {
					return proof.Xpt{}, false, err
				}
			}
			if err := ctx.Err(); err != nil {
				return proof.Xpt{}, false, err
			}
			x := proof.Xlerp(u, v, big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), uint(j)))
			if depthBeyond(x, other, clearance, hint) {
				found = &x
				break
			}
		}
	}
	if memoized {
		memo[key] = found
	}
	if found == nil {
		return proof.Xpt{}, false, nil
	}
	return *found, true, nil
}
