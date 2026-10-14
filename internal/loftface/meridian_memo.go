package loftface

import (
	"encoding/binary"
	"hash/maphash"
	"math/big"
	"sync"

	"github.com/lestrrat-3d/decad/internal/freeform"
)

// MeridianMemo hands back MeridianQRows rows already computed for equal
// inputs within one caller-owned scope. A hit requires every input rational
// to compare equal to the recorded one; the hash only picks the bucket. Each
// call returns rows no other caller holds. A nil memo computes every call.
// A memo belongs to one caller-owned scope, such as one census; never keep
// one across calls.
type MeridianMemo struct {
	mu      sync.Mutex
	seed    maphash.Seed
	entries map[uint64][]meridianMemoEntry
}

type meridianMemoEntry struct {
	first, second Map
	span          freeform.BezierSpan
	m, b          *big.Rat
	rows          [3][]*big.Rat
}

// NewMeridianMemo returns an empty memo.
func NewMeridianMemo() *MeridianMemo {
	return &MeridianMemo{seed: maphash.MakeSeed(), entries: make(map[uint64][]meridianMemoEntry)}
}

// QRows returns MeridianQRows(first, second, span, m, b).
func (c *MeridianMemo) QRows(first, second Map, span freeform.BezierSpan,
	m, b *big.Rat) [3][]*big.Rat {
	if c == nil || !meridianInputsComplete(first, second, span, m, b) {
		return MeridianQRows(first, second, span, m, b)
	}
	key := c.hash(first, second, span, m, b)
	c.mu.Lock()
	for i := range c.entries[key] {
		entry := &c.entries[key][i]
		if entry.same(first, second, span, m, b) {
			rows := copyRows(entry.rows)
			c.mu.Unlock()
			return rows
		}
	}
	c.mu.Unlock()
	rows := MeridianQRows(first, second, span, m, b)
	entry := meridianMemoEntry{first: copyMap(first), second: copyMap(second),
		span: copySpan(span), m: new(big.Rat).Set(m), b: new(big.Rat).Set(b), rows: copyRows(rows)}
	c.mu.Lock()
	c.entries[key] = append(c.entries[key], entry)
	c.mu.Unlock()
	return rows
}

func meridianMapRats(mp Map, visit func(*big.Rat) bool) bool {
	for _, v := range [...]RatVec{mp.Constant, mp.Q0, mp.QU, mp.QV} {
		for _, r := range v {
			if !visit(r) {
				return false
			}
		}
	}
	return visit(mp.G0) && visit(mp.GU) && visit(mp.GV)
}

func meridianInputsComplete(first, second Map, span freeform.BezierSpan, m, b *big.Rat) bool {
	nonNil := func(r *big.Rat) bool { return r != nil }
	if !meridianMapRats(first, nonNil) || !meridianMapRats(second, nonNil) || m == nil || b == nil {
		return false
	}
	for _, p := range span {
		if p.U == nil || p.V == nil {
			return false
		}
	}
	return true
}

func (c *MeridianMemo) hash(first, second Map, span freeform.BezierSpan, m, b *big.Rat) uint64 {
	var h maphash.Hash
	h.SetSeed(c.seed)
	var buf [8]byte
	writeInt := func(x *big.Int) {
		binary.LittleEndian.PutUint64(buf[:], uint64(x.Sign()+1))
		h.Write(buf[:])
		for _, w := range x.Bits() {
			binary.LittleEndian.PutUint64(buf[:], uint64(w))
			h.Write(buf[:])
		}
		h.WriteByte(0xff)
	}
	write := func(r *big.Rat) bool {
		writeInt(new(big.Int).Set(r.Num()))
		writeInt(new(big.Int).Set(r.Denom()))
		return true
	}
	meridianMapRats(first, write)
	meridianMapRats(second, write)
	binary.LittleEndian.PutUint64(buf[:], uint64(len(span)))
	h.Write(buf[:])
	for _, p := range span {
		write(p.U)
		write(p.V)
	}
	write(m)
	write(b)
	return h.Sum64()
}

func (e *meridianMemoEntry) same(first, second Map, span freeform.BezierSpan, m, b *big.Rat) bool {
	if len(span) != len(e.span) || e.m.Cmp(m) != 0 || e.b.Cmp(b) != 0 {
		return false
	}
	for i := range span {
		if e.span[i].U.Cmp(span[i].U) != 0 || e.span[i].V.Cmp(span[i].V) != 0 {
			return false
		}
	}
	return sameMapRats(e.first, first) && sameMapRats(e.second, second)
}

func sameMapRats(a, b Map) bool {
	var left []*big.Rat
	meridianMapRats(a, func(r *big.Rat) bool { left = append(left, r); return true })
	i := 0
	return meridianMapRats(b, func(r *big.Rat) bool {
		ok := left[i].Cmp(r) == 0
		i++
		return ok
	})
}

func copyRows(rows [3][]*big.Rat) [3][]*big.Rat {
	var out [3][]*big.Rat
	for i, row := range rows {
		out[i] = make([]*big.Rat, len(row))
		for j, v := range row {
			out[i][j] = new(big.Rat).Set(v)
		}
	}
	return out
}

func copyRat(r *big.Rat) *big.Rat { return new(big.Rat).Set(r) }

func copyMap(mp Map) Map {
	out := mp
	for i := range out.Constant {
		out.Constant[i] = copyRat(mp.Constant[i])
		out.Q0[i] = copyRat(mp.Q0[i])
		out.QU[i] = copyRat(mp.QU[i])
		out.QV[i] = copyRat(mp.QV[i])
	}
	out.G0, out.GU, out.GV = copyRat(mp.G0), copyRat(mp.GU), copyRat(mp.GV)
	return out
}

func copySpan(span freeform.BezierSpan) freeform.BezierSpan {
	out := make(freeform.BezierSpan, len(span))
	for i, p := range span {
		out[i] = p
		out[i].U, out[i].V = copyRat(p.U), copyRat(p.V)
	}
	return out
}
