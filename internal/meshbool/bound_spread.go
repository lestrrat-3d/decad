package meshbool

import (
	"context"
	"math"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
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
	// spreadExactWork is the work one exact triangle-to-triangle distance
	// proof is charged, beyond the facet scan that finds its candidates.
	spreadExactWork = 64
)

// spreadMargins are the multiples of the clearance a split point must stand
// from the other operand's held boundary, tried in turn. A point just past the
// clearance can leave a far piece that reaches within it, so a piece that
// fails is cut again farther from the other operand before it is kept whole.
var spreadMargins = [...]float64{1, 8, 64, 512}

// BudgetFunc charges the work, the records and the KeptFacet slots of a
// bounded replay. facets slots are reserved as backing storage. A nil
// BudgetFunc charges nothing.
type BudgetFunc func(work, records, facets uint64) error

// ReplaySpreadBudget returns the BudgetFunc that charges SplitSpread's work to
// replay, or nil when replay is nil.
func ReplaySpreadBudget(ctx context.Context, replay *proofbound.ReplayBudget) BudgetFunc {
	if replay == nil {
		return nil
	}
	return func(work, records, facets uint64) error {
		if work != 0 {
			if err := replay.SpendWork(ctx, work); err != nil {
				return err
			}
		}
		if records != 0 {
			if err := replay.ReserveRecords(ctx, records); err != nil {
				return err
			}
		}
		if facets != 0 {
			return replay.ReserveArray(ctx, facets, uint64(unsafe.Sizeof(KeptFacet{})))
		}
		return nil
	}
}

// SplitSpread returns kept with each piece that has rim corners and operand
// corners split so that the facets away from the rim hold only the operand
// facet's own bound. a and b are the operands kept came from; a piece of the
// second operand is read against a, and one of the first against b. A rim
// corner is one whose bound is above floor, the pair's chord tolerance, and
// above twice the operand facet's bound.
//
// A split point x stands on an edge from a rim corner u to another corner v
// of one piece of operand facet t, so it lies on t. The piece is cut into a
// rim piece at u and far pieces, those with no rim corner, and a far piece
// carries δ(t) (agent-docs/faceted-vertex-bounds-design.md §3.3, §3.7) for
// every point of it. That holds where each point of the far piece stands
// farther than δ(t) + δ_o from the held boundary of the other operand, δ_o the
// largest bound that operand holds: the held and the true boundary of the
// other operand then differ by at most δ_o, and the segment from the point to
// the true point of t within δ(t) meets neither. The distance to a convex
// other operand dips between two points that both clear it, so the proof is
// over the far piece, not its corners. A split is emitted only when
// farPiecesClear proves, by an exact triangle-to-triangle distance against
// every facet of the other operand near the piece, that every far piece it
// makes clears that distance. A far piece with a corner on the other operand's
// boundary, such as a chain corner, never does. A piece with no such split is
// kept whole, which is the result without this pass. The split adds vertices
// and never lowers a bound a rim vertex or an operand corner carries.
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
	var err error
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
				if out, err = appendKept(out, charge, *f); err != nil {
					return nil, err
				}
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
				if out, err = appendKept(out, charge, *f); err != nil {
					return nil, err
				}
			}
			continue
		}
		// Rounded up, so the sum is never below the true δ(t) + δ_o.
		clearance := proofbound.AbsSumUpper(f.Delta, maxDelta[1-side])
		var pieces [3]KeptFacet
		split := false
		for level, margin := range spreadMargins {
			pts, ok, err := spreadPoints(ctx, f, hi, lo, other, clearance*margin, level, cut[side], &hint, charge)
			if err != nil {
				return nil, err
			}
			if !ok {
				// A deeper level asks for more depth and finds no more points.
				break
			}
			var far [3]bool
			pieces, far = splitPieces(f, hi, lo, pts)
			split, err = farPiecesClear(ctx, pieces, far, other, clearance, charge)
			if err != nil {
				return nil, err
			}
			if split {
				break
			}
		}
		if !split {
			if out != nil {
				if out, err = appendKept(out, charge, *f); err != nil {
					return nil, err
				}
			}
			continue
		}
		if out == nil {
			capacity := max(len(kept)+len(kept)/4, i+3)
			if charge != nil {
				if err := charge(0, 0, uint64(capacity)); err != nil {
					return nil, err
				}
			}
			out = make([]KeptFacet, i, capacity)
			copy(out, kept[:i])
		}
		if charge != nil {
			if err := charge(3, 3, 0); err != nil {
				return nil, err
			}
		}
		if out, err = appendKept(out, charge, pieces[:]...); err != nil {
			return nil, err
		}
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

// splitPieces returns the three pieces of f cut at pts, which holds the split
// point of every edge from a rim corner in hi to a corner in lo, and which of
// them are far pieces, those with no rim corner. The pieces keep f's
// orientation.
func splitPieces(f *KeptFacet, hi, lo []int, pts map[[2]int]proof.Xpt) ([3]KeptFacet, [3]bool) {
	d := f.Delta
	var out [3]KeptFacet
	var far [3]bool
	add := func(n int, isFar bool, v [3]proof.Xpt, beta [3]float64) {
		out[n] = KeptFacet{V: v, Src: f.Src, Beta: beta, Delta: d, Second: f.Second, Facet: f.Facet}
		far[n] = isFar
	}
	if len(hi) == 1 {
		u := hi[0]
		p, q := (u+1)%3, (u+2)%3
		x, y := pts[[2]int{u, p}], pts[[2]int{u, q}]
		add(0, false, [3]proof.Xpt{f.V[u], x, y}, [3]float64{f.Beta[u], d, d})
		add(1, true, [3]proof.Xpt{x, f.V[p], f.V[q]}, [3]float64{d, f.Beta[p], f.Beta[q]})
		add(2, true, [3]proof.Xpt{x, f.V[q], y}, [3]float64{d, f.Beta[q], d})
		return out, far
	}
	w := lo[0]
	u1, u2 := (w+1)%3, (w+2)%3
	x1, x2 := pts[[2]int{u1, w}], pts[[2]int{u2, w}]
	add(0, false, [3]proof.Xpt{f.V[u1], f.V[u2], x2}, [3]float64{f.Beta[u1], f.Beta[u2], d})
	add(1, false, [3]proof.Xpt{f.V[u1], x2, x1}, [3]float64{f.Beta[u1], d, d})
	add(2, true, [3]proof.Xpt{x1, x2, f.V[w]}, [3]float64{d, d, f.Beta[w]})
	return out, far
}

// farPiecesClear reports whether every piece marked far stands farther than
// clearance from every facet of other, proven by DisjointTriDistanceAbove's
// exact distance. A piece's box, grown by clearance and the corners' rounding,
// prunes the facets whose box is out of reach; a facet it keeps must pass the
// exact proof, and one the proof refuses (a facet the piece touches or
// crosses, or one too near) makes the answer false.
func farPiecesClear(ctx context.Context, pieces [3]KeptFacet, far [3]bool, other *BoolMesh, clearance float64,
	charge BudgetFunc) (bool, error) {
	if !(clearance > 0) || math.IsInf(clearance, 0) {
		return false, nil
	}
	for n := range pieces {
		if !far[n] {
			continue
		}
		if charge != nil {
			if err := charge(uint64(len(other.Tris)), 1, 0); err != nil {
				return false, err
			}
		}
		if err := ctx.Err(); err != nil {
			return false, err
		}
		tri := pieces[n].V
		lo, hi := tri[0].Vec(), tri[0].Vec()
		round := 0.0
		for _, c := range tri {
			v := c.Vec()
			lo = r3.Vec{X: min(lo.X, v.X), Y: min(lo.Y, v.Y), Z: min(lo.Z, v.Z)}
			hi = r3.Vec{X: max(hi.X, v.X), Y: max(hi.Y, v.Y), Z: max(hi.Z, v.Z)}
			round = max(round, PointRoundBound(c, v))
		}
		scale := max(math.Abs(lo.X), math.Abs(lo.Y), math.Abs(lo.Z), math.Abs(hi.X), math.Abs(hi.Y), math.Abs(hi.Z))
		reach := clearance*(1+0x1p-30) + round + scale*0x1p-40
		box := [2]r3.Vec{lo, hi}
		fl := [3]r3.Vec{tri[0].Vec(), tri[1].Vec(), tri[2].Vec()}
		for i := range other.Tris {
			if !BoxesWithin(box, other.Boxes[i], reach) {
				continue
			}
			// A float distance at or under the clearance refuses without the exact
			// proof. It only ever refuses, so a pair it misreads (the float distance
			// of a crossing pair is no distance) goes on to the proof, which refuses
			// a crossing pair itself.
			if TriTriDistance(fl, TriCorners(other, i)) <= clearance {
				return false, nil
			}
			// A separating axis (triGapAbove), read in floats with their rounding
			// charged, proves the distance without the exact proof.
			// The float corners of the piece sit within round of its exact ones.
			if triGapAbove(fl, TriCorners(other, i), clearance+round*(1+0x1p-20)) {
				continue
			}
			if charge != nil {
				if err := charge(spreadExactWork, 0, 0); err != nil {
					return false, err
				}
			}
			if !DisjointTriDistanceAbove(tri, XtriCorners(other, i), clearance) {
				return false, nil
			}
		}
	}
	return true, nil
}

// appendKept appends items to out, reserving the storage a growth allocates
// before it allocates it. Growth doubles the capacity, as append does.
func appendKept(out []KeptFacet, charge BudgetFunc, items ...KeptFacet) ([]KeptFacet, error) {
	if len(out)+len(items) > cap(out) {
		capacity := max(2*cap(out), len(out)+len(items))
		if charge != nil {
			if err := charge(0, 0, uint64(capacity)); err != nil {
				return nil, err
			}
		}
		grown := make([]KeptFacet, len(out), capacity)
		copy(grown, out)
		out = grown
	}
	return append(out, items...), nil
}

// triGapAbove reports whether the float triangles ta and tb stand farther than
// slack apart, proven by a gap along one of seventeen axes: the two facet
// normals, the nine edge-edge cross products and the six edge-normal cross
// products. Any direction n separates the triangles by at least the gap of
// their projections over |n|, so the axes need no exactness of their own;
// axisGapExceeds charges each projection's float error as
// TriTriDistanceExceeds does. It never answers true for a pair that crosses or
// comes within slack, and it answers false for a pair no such axis separates.
func triGapAbove(ta, tb [3]r3.Vec, slack float64) bool {
	m, ok := triPairScale(ta, tb)
	if !ok || !(slack >= 0 && slack <= math.MaxFloat64) {
		return false
	}
	thr := slack + (slack+m)*0x1p-26 + 0x1p-199
	na, nb := triNormal(ta), triNormal(tb)
	return axisGapExceeds(na, ta, tb, m, thr) || axisGapExceeds(nb, ta, tb, m, thr) ||
		edgeAxesGapExceeds(ta, tb, na, nb, m, thr)
}

// spreadEdge names a split edge: its rim corner and other corner by exact key,
// the bound of the operand facet it lies on, which sets the clearance, and the
// margin level the point was sought at.
type spreadEdge struct {
	hi, lo string
	delta  float64
	level  int
}

// spreadPoints finds the split point of every edge from a rim corner in hi to
// a corner in lo at depth from other's held boundary, and reports whether every
// edge has one.
func spreadPoints(ctx context.Context, f *KeptFacet, hi, lo []int, other *BoolMesh, depth float64, level int,
	memo map[spreadEdge]*proof.Xpt, hint *int, charge BudgetFunc) (map[[2]int]proof.Xpt, bool, error) {
	pts := map[[2]int]proof.Xpt{}
	for _, h := range hi {
		for _, l := range lo {
			x, ok, err := spreadPoint(ctx, f, h, l, other, depth, level, memo, hint, charge)
			if err != nil || !ok {
				return nil, false, err
			}
			pts[[2]int{h, l}] = x
		}
	}
	return pts, true, nil
}

// spreadPoint finds the split point of the edge from corner hi to corner lo of
// f: the point at 1/2^j of the edge from hi with the largest j that keeps it at
// least spreadReach times hi's bound from hi and deeper than depth from other's
// held boundary. The point is a candidate only: farPiecesClear proves the
// pieces it makes. The answer is memoized per edge and level by the corner keys.
func spreadPoint(ctx context.Context, f *KeptFacet, hi, lo int, other *BoolMesh, depth float64, level int,
	memo map[spreadEdge]*proof.Xpt, hint *int, charge BudgetFunc) (proof.Xpt, bool, error) {
	u, v := f.V[hi], f.V[lo]
	key := spreadEdge{hi: f.Keys[hi], lo: f.Keys[lo], delta: f.Delta, level: level}
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
				if err := charge(uint64(len(other.Tris)), 1, 0); err != nil {
					return proof.Xpt{}, false, err
				}
			}
			if err := ctx.Err(); err != nil {
				return proof.Xpt{}, false, err
			}
			x := proof.Xlerp(u, v, big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), uint(j)))
			if depthBeyond(x, other, depth, hint) {
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
