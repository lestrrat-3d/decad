package meshbool

import (
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
// integers, so no decimal text is built.
type cutVertIndex struct {
	head map[uint64]int // hash → newest vertex with that hash
	next []int          // per vertex: the previous vertex with its hash, or -1
}

// lookup returns the vertex of verts whose 2D identity equals p's.
func (x *cutVertIndex) lookup(verts []CutVert, h uint64, p Xp2) (int, bool) {
	i, ok := x.head[h]
	for ok && i >= 0 {
		if proof.RatTextEqual(verts[i].P2.U, p.U) && proof.RatTextEqual(verts[i].P2.V, p.V) {
			return i, true
		}
		i = x.next[i]
	}
	return 0, false
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
