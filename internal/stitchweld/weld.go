// Package stitchweld groups held vertices and candidate free edges for Stitch.
package stitchweld

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// vertexKey uses float bits so negative and positive zero stay distinct.
type vertexKey struct{ x, y, z uint64 }

func keyOf(v r3.Vec) vertexKey {
	return vertexKey{math.Float64bits(v.X), math.Float64bits(v.Y), math.Float64bits(v.Z)}
}

type curveKey[T comparable] struct {
	key   vertexKey
	token T
}

// Table assigns one class to vertices with bit-identical positions when both
// bounds are zero, or when both carry the same nonzero curve token. A bounded
// vertex without such a token always receives its own class. Vertices, Bounds,
// and Tokens keep the first registered member of each class. A later merge
// never replaces that member's values, including its bound and certificate.
type Table[V comparable, T comparable] struct {
	Classes  map[V]int
	zero     map[vertexKey]int
	byCurve  map[curveKey[T]]int
	Vertices []r3.Vec
	Bounds   []float64
	Tokens   []T
}

func NewTable[V comparable, T comparable]() *Table[V, T] {
	return &Table[V, T]{Classes: map[V]int{}}
}

// ClassOf returns the existing class for v or registers its held values.
// hasCurve states whether token carries a nonzero certificate.
func (t *Table[V, T]) ClassOf(v V, position r3.Vec, bound float64, token T, hasCurve bool) int {
	if idx, ok := t.Classes[v]; ok {
		return idx
	}
	key := keyOf(position)
	zero := bound == 0
	if zero {
		if idx, ok := t.zero[key]; ok {
			t.Classes[v] = idx
			return idx
		}
	}
	var ckey curveKey[T]
	if hasCurve {
		ckey = curveKey[T]{key: key, token: token}
		if idx, ok := t.byCurve[ckey]; ok {
			t.Classes[v] = idx
			return idx
		}
	}
	idx := len(t.Vertices)
	t.Vertices = append(t.Vertices, position)
	t.Bounds = append(t.Bounds, bound)
	t.Tokens = append(t.Tokens, token)
	t.Classes[v] = idx
	if zero {
		if t.zero == nil {
			t.zero = map[vertexKey]int{}
		}
		t.zero[key] = idx
	}
	if hasCurve {
		if t.byCurve == nil {
			t.byCurve = map[curveKey[T]]int{}
		}
		t.byCurve[ckey] = idx
	}
	return idx
}

// Candidate is one free edge admitted for grouping by its endpoint classes.
type Candidate[E comparable] struct {
	Edge       E
	Start, End int
}

type weldKey struct{ a, b int }

func newWeldKey(a, b int) weldKey {
	if a > b {
		a, b = b, a
	}
	return weldKey{a, b}
}

// WeldPairs assigns a group only to a key claimed by exactly two candidates
// whose curve variants the caller confirms join. A key claimed by three or
// more edges leaves every edge free. Candidate order determines group IDs.
func WeldPairs[E comparable](candidates []Candidate[E], joins func(E, E) bool) (map[E]int, int) {
	byKey := map[weldKey][]E{}
	var order []weldKey
	for _, candidate := range candidates {
		key := newWeldKey(candidate.Start, candidate.End)
		if _, ok := byKey[key]; !ok {
			order = append(order, key)
		}
		byKey[key] = append(byKey[key], candidate.Edge)
	}
	group := map[E]int{}
	groups := 0
	for _, key := range order {
		edges := byKey[key]
		if len(edges) != 2 || !joins(edges[0], edges[1]) {
			continue
		}
		id := groups
		groups++
		group[edges[0]] = id
		group[edges[1]] = id
	}
	return group, groups
}
