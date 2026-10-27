package meshbool

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
)

// This file owns how a TriCutter interns its vertices. Both tables below are
// lookup structures only: every hit is confirmed by exact integer comparison,
// and a hash only chooses which vertices to compare. A hash collision costs a
// comparison and can never merge two vertices.

// cutVertIndex interns a vertex by its exact 2D identity: two projections name
// one vertex exactly when their U texts and their V texts agree, which is
// what an index keyed by U.RatString()+"|"+V.RatString() reads.
// proof.RatTextEqual decides that from the numerator and denominator
// integers, so no decimal text is built. Worker-private projections first hash
// correctly rounded floats, whose equality is necessary but never sufficient
// for exact identity. A bucket promotes after eight unequal comparisons to
// canonical integer hashes; close points cannot create an unbounded float chain.
type cutVertIndex struct {
	floatKeys bool
	promoted  map[uint64]struct{}
	exactHead map[[2]uint64]int
	head      map[uint64]int // hash → newest vertex with that hash
	next      []int          // per vertex: the previous vertex with its hash, or -1
}

// lookup returns the vertex of verts whose 2D identity equals p's.
func (x *cutVertIndex) lookup(verts []CutVert, h uint64, p Xp2) (int, bool) {
	if _, ok := x.promoted[h]; ok {
		return x.lookupExact(verts, h, p)
	}
	i, ok := x.head[h]
	first, compared := i, 0
	for ok && i >= 0 {
		if x.floatKeys && sameCutProjection(verts[i].P2, p) || !x.floatKeys &&
			proof.RatTextEqual(verts[i].P2.U, p.U) && proof.RatTextEqual(verts[i].P2.V, p.V) {
			return i, true
		}
		compared++
		if x.floatKeys && compared == 8 {
			if x.promoted == nil {
				x.promoted = make(map[uint64]struct{})
			}
			if x.exactHead == nil {
				x.exactHead = make(map[[2]uint64]int)
			}
			// At most eight entries belong to this bucket. Rewrite only their
			// links; other float buckets have disjoint vertex lists. Save the
			// old link before inserting into the exact-hash collision chain.
			for j := first; j >= 0; {
				oldNext := x.next[j]
				materializeXP2(verts[j].P2)
				key := [2]uint64{h, xp2Hash(verts[j].P2)}
				prev, exists := x.exactHead[key]
				if !exists {
					prev = -1
				}
				x.next[j] = prev
				x.exactHead[key] = j
				j = oldNext
			}
			x.promoted[h] = struct{}{}
			return x.lookupExact(verts, h, p)
		}
		i = x.next[i]
	}
	return 0, false
}

func (x *cutVertIndex) lookupExact(verts []CutVert, h uint64, p Xp2) (int, bool) {
	materializeXP2(p)
	i, ok := x.exactHead[[2]uint64{h, xp2Hash(p)}]
	for ok && i >= 0 {
		if proof.RatTextEqual(verts[i].P2.U, p.U) && proof.RatTextEqual(verts[i].P2.V, p.V) {
			return i, true
		}
		i = x.next[i]
	}
	return 0, false
}

// addForPoint appends the vertex to its float or promoted integer-hash chain.
func (x *cutVertIndex) addForPoint(h uint64, i int, p Xp2) {
	if _, ok := x.promoted[h]; !ok {
		x.add(h, i)
		return
	}
	materializeXP2(p)
	key := [2]uint64{h, xp2Hash(p)}
	prev, ok := x.exactHead[key]
	if !ok {
		prev = -1
	}
	x.next = append(x.next, prev)
	x.exactHead[key] = i
}

// add records vertex i, which must be the next vertex in order.
func (x *cutVertIndex) add(h uint64, i int) {
	if x.head == nil {
		x.head = map[uint64]int{}
	}
	prev, ok := x.head[h]
	if !ok {
		prev = -1
	}
	x.next = append(x.next, prev)
	x.head[h] = i
}

// xp2Hash hashes the integers proof.RatTextEqual compares, so equal
// identities hash equally.
func xp2Hash(p Xp2) uint64 {
	return proof.RatTextHash(proof.RatTextHash(proof.HashSeed, p.U), p.V)
}

// cutRawIndex maps a homogeneous projection (Hu, Hv, Hw) already interned to
// its vertex. Equal integers give equal RatFrac results, so the projection
// they build has the same 2D identity, and cutVertIndex would return the same
// vertex. A hit therefore skips the projection, its two GCDs and the identity
// lookup, and returns that vertex.
type cutRawIndex struct {
	head    map[uint64]int // hash → newest entry with that hash
	entries []cutRawEntry
}

type cutRawEntry struct {
	u, v, w *big.Int
	vert    int
	next    int // the previous entry with this hash, or -1
}

func rawHash(u, v, w *big.Int) uint64 {
	return proof.IntHash(proof.IntHash(proof.IntHash(proof.HashSeed, u), v), w)
}

// lookup returns the vertex recorded for integers equal to (u, v, w).
func (x *cutRawIndex) lookup(h uint64, u, v, w *big.Int) (int, bool) {
	i, ok := x.head[h]
	for ok && i >= 0 {
		e := &x.entries[i]
		if e.u.Cmp(u) == 0 && e.v.Cmp(v) == 0 && e.w.Cmp(w) == 0 {
			return e.vert, true
		}
		i = e.next
	}
	return 0, false
}

// add records that the projection of (u, v, w) is vertex vert. The integers
// belong to an exact point, which is never mutated after construction.
func (x *cutRawIndex) add(h uint64, u, v, w *big.Int, vert int) {
	if x.head == nil {
		x.head = map[uint64]int{}
	}
	prev, ok := x.head[h]
	if !ok {
		prev = -1
	}
	x.head[h] = len(x.entries)
	x.entries = append(x.entries, cutRawEntry{u: u, v: v, w: w, vert: vert, next: prev})
}

// addPoint interns p by its projection on the cutter's axes. It returns what
// AddVert(Proj(p), p, boundary) returns and leaves the same vertex table.
func (tc *TriCutter) addPoint(p proof.Xpt, boundary bool) int {
	u, v := tc.U, tc.V
	if tc.Swap {
		u, v = v, u
	}
	hu, hv := XIntCoordOf(p, u), XIntCoordOf(p, v)
	h := rawHash(hu, hv, p.W)
	if i, ok := tc.raw.lookup(h, hu, hv, p.W); ok {
		if boundary {
			tc.Verts[i].Boundary = true
		}
		return i
	}
	i := tc.AddVert(tc.project(p, u, v), p, boundary)
	tc.raw.add(h, hu, hv, p.W, i)
	return i
}

func sameCutProjection(a, b Xp2) bool {
	if a.deferredRats && b.deferredRats {
		// Both would be canonicalized by RatFrac. A manual non-reduced Rat
		// must instead keep the textual identity checked below.
		return compareXP2Coordinate(a, b, 0) == 0 && compareXP2Coordinate(a, b, 1) == 0
	}
	materializeXP2(a)
	materializeXP2(b)
	return proof.RatTextEqual(a.U, b.U) && proof.RatTextEqual(a.V, b.V)
}

// cutProjectionHash hashes exact identity or its necessarily equal rounding.
// Normalize signed zero because both spellings denote the same rational zero.
func cutProjectionHash(p Xp2, floatKeys bool) uint64 {
	if !floatKeys {
		return xp2Hash(p)
	}
	u, v := p.Fu, p.Fv
	if !p.floatFromExact {
		materializeXP2(p)
		u, _ = p.U.Float64()
		v, _ = p.V.Float64()
	}
	if u == 0 {
		u = 0
	}
	if v == 0 {
		v = 0
	}
	return proof.HashWord(proof.HashWord(proof.HashSeed, math.Float64bits(u)), math.Float64bits(v))
}
