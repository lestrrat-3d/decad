// Package edgetable holds a flat open-addressing map keyed by a directed edge.
// It imports nothing from decad, so geometry packages at any level can share it.
package edgetable

import "math"

// edgeEmpty marks a free slot. A packed key is a<<32|b with a and b below
// 2^31, so its top bit is clear and it can never equal edgeEmpty.
const edgeEmpty = ^uint64(0)

// Table is an open-addressing map from a directed edge [2]int to an
// int, with the same lookup results as map[[2]int]int: Set overwrites (last
// write wins), Get reports presence, Delete removes. It exists because the
// Go map's 128-bit key hashing and probing dominates adjacency building on
// meshes of 10^5 triangles.
//
// An edge whose endpoints are not both in [0, 2^31), or whose value does not
// fit int32, lives in a side map instead of the flat arrays, so every input
// behaves exactly as the map would. Each edge is stored in exactly one place.
//
// The table exposes no iteration, so nothing can depend on its layout. A nil
// *Table is an empty table for Get and Len. It is not safe for concurrent
// writes; concurrent reads of a table nobody writes are safe.
type Table struct {
	keys  []uint64
	vals  []int32
	mask  uint64
	shift uint
	flat  int
	// side holds the edges the flat arrays cannot represent.
	side map[[2]int]int
	// tris names the triangle slice the table was built from, so a consumer
	// can tell the table still describes a given slice.
	tris [][3]int
}

// New returns an empty table sized for n edges without growth.
func New(n int) *Table {
	t := &Table{}
	t.alloc(edgeCapacity(n))
	return t
}

// edgeCapacity returns the power-of-two slot count that keeps n entries at
// or below half load, with a floor of 8.
func edgeCapacity(n int) int {
	c := 8
	for c < 2*n && c < 1<<40 {
		c <<= 1
	}
	return c
}

func (t *Table) alloc(c int) {
	t.keys = make([]uint64, c)
	for i := range t.keys {
		t.keys[i] = edgeEmpty
	}
	t.vals = make([]int32, c)
	t.mask = uint64(c - 1)
	t.shift = 64
	for n := c; n > 1; n >>= 1 {
		t.shift--
	}
	t.flat = 0
}

// pack returns the flat key of edge (a, b), or false when the edge needs the
// side map.
func pack(a, b int) (uint64, bool) {
	if uint64(a) >= 1<<31 || uint64(b) >= 1<<31 {
		return 0, false
	}
	return uint64(a)<<32 | uint64(b), true
}

func (t *Table) slot(key uint64) uint64 {
	return (key * 0x9E3779B97F4A7C15) >> t.shift
}

// Len returns the number of distinct edges stored.
func (t *Table) Len() int {
	if t == nil {
		return 0
	}
	return t.flat + len(t.side)
}

// Get returns the value stored for edge (a, b).
func (t *Table) Get(a, b int) (int, bool) {
	if t == nil {
		return 0, false
	}
	key, ok := pack(a, b)
	if !ok {
		v, ok := t.side[[2]int{a, b}]
		return v, ok
	}
	if t.side != nil {
		if v, ok := t.side[[2]int{a, b}]; ok {
			return v, true
		}
	}
	for i := t.slot(key); ; i = (i + 1) & t.mask {
		k := t.keys[i]
		if k == key {
			return int(t.vals[i]), true
		}
		if k == edgeEmpty {
			return 0, false
		}
	}
}

// Set stores v for edge (a, b), replacing any earlier value.
func (t *Table) Set(a, b, v int) {
	key, ok := pack(a, b)
	if !ok || v < math.MinInt32 || v > math.MaxInt32 {
		if ok {
			t.deleteFlat(key)
		}
		if t.side == nil {
			t.side = make(map[[2]int]int)
		}
		t.side[[2]int{a, b}] = v
		return
	}
	if t.side != nil {
		delete(t.side, [2]int{a, b})
	}
	if 2*(t.flat+1) > len(t.keys) {
		t.grow()
	}
	for i := t.slot(key); ; i = (i + 1) & t.mask {
		k := t.keys[i]
		if k == key {
			t.vals[i] = int32(v)
			return
		}
		if k == edgeEmpty {
			t.keys[i] = key
			t.vals[i] = int32(v)
			t.flat++
			return
		}
	}
}

// Clear removes every edge, keeping the allocated capacity.
func (t *Table) Clear() {
	for i := range t.keys {
		t.keys[i] = edgeEmpty
	}
	t.flat = 0
	t.side = nil
}

// Delete removes edge (a, b); it reports whether the edge was present.
func (t *Table) Delete(a, b int) bool {
	key, ok := pack(a, b)
	if !ok {
		_, had := t.side[[2]int{a, b}]
		delete(t.side, [2]int{a, b})
		return had
	}
	if t.side != nil {
		if _, had := t.side[[2]int{a, b}]; had {
			delete(t.side, [2]int{a, b})
			return true
		}
	}
	return t.deleteFlat(key)
}

// deleteFlat removes key from the flat arrays by backward-shift deletion, so
// no tombstones accumulate and probe chains stay unbroken.
func (t *Table) deleteFlat(key uint64) bool {
	i := t.slot(key)
	for {
		k := t.keys[i]
		if k == edgeEmpty {
			return false
		}
		if k == key {
			break
		}
		i = (i + 1) & t.mask
	}
	j := i
	for {
		j = (j + 1) & t.mask
		k := t.keys[j]
		if k == edgeEmpty {
			break
		}
		// Move k into the hole at i unless its home slot lies cyclically in
		// (i, j], where it would become unreachable.
		home := t.slot(k)
		if (j-home)&t.mask < (j-i)&t.mask {
			continue
		}
		t.keys[i], t.vals[i] = k, t.vals[j]
		i = j
	}
	t.keys[i] = edgeEmpty
	t.flat--
	return true
}

func (t *Table) grow() {
	oldKeys, oldVals := t.keys, t.vals
	t.alloc(2 * len(oldKeys))
	for i, k := range oldKeys {
		if k == edgeEmpty {
			continue
		}
		j := t.slot(k)
		for t.keys[j] != edgeEmpty {
			j = (j + 1) & t.mask
		}
		t.keys[j], t.vals[j] = k, oldVals[i]
		t.flat++
	}
}

// SetSource records the triangle slice the table was built from.
func (t *Table) SetSource(tris [][3]int) { t.tris = tris }

// DescribesSource reports whether the table was recorded, by SetSource, over
// exactly this triangle slice (same backing array and length).
func (t *Table) DescribesSource(tris [][3]int) bool {
	if t == nil || len(tris) != len(t.tris) {
		return false
	}
	return len(tris) == 0 || &tris[0] == &t.tris[0]
}
