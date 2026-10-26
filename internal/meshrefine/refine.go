// Package meshrefine splits the long edges of an indexed triangle mesh
// (agent-docs/mesh-import-design.md §11). It works on plain millimetre data and
// touches no body or document.
package meshrefine

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/r3"
)

// ErrTooLarge is returned when the refined mesh would pass the triangle limit,
// or the edges do not shrink within the round limit.
var ErrTooLarge = errors.New("meshrefine: refined mesh too large")

const (
	// MaxTriangles bounds the triangles a refinement may reach.
	MaxTriangles = 5_000_000
	maxRounds    = 40
)

// Result is a refined mesh.
type Result struct {
	Vertices  []r3.Vec
	Triangles [][3]int
	// Rounds counts the passes that split edges, and Splits the edges split.
	Rounds, Splits int
}

type edgeKey [2]int

func keyOf(a, b int) edgeKey {
	if a > b {
		a, b = b, a
	}
	return edgeKey{a, b}
}

// Refine splits every edge longer than maxEdge at its midpoint, once a round,
// and cuts each triangle along its split edges, until no edge is longer. Both
// triangles on a shared edge see the same midpoint vertex, so a closed mesh stays
// closed and its winding is kept. A triangle with one split edge becomes two,
// with two three and with three four, the diagonal of a quad taken on the shorter
// side. The corners' positions never move, so the surface is the same set of
// points and every area and volume reading stands. maxEdge is positive and
// finite.
func Refine(ctx context.Context, vertices []r3.Vec, triangles [][3]int, maxEdge float64) (Result, error) {
	verts := append([]r3.Vec(nil), vertices...)
	tris := append([][3]int(nil), triangles...)
	var out Result
	for round := 0; ; round++ {
		if err := ctx.Err(); err != nil {
			return Result{}, err
		}
		mid := map[edgeKey]int{}
		long := func(a, b int) bool {
			return verts[b].Sub(verts[a]).Len() > maxEdge
		}
		midpoint := func(a, b int) int {
			k := keyOf(a, b)
			if m, ok := mid[k]; ok {
				return m
			}
			m := len(verts)
			verts = append(verts, verts[a].Add(verts[b]).Scale(0.5))
			mid[k] = m
			return m
		}
		next := make([][3]int, 0, len(tris))
		for i, t := range tris {
			if i%65536 == 0 {
				if err := ctx.Err(); err != nil {
					return Result{}, err
				}
			}
			var split [3]bool
			count := 0
			for e := range 3 {
				if long(t[e], t[(e+1)%3]) {
					split[e] = true
					count++
				}
			}
			if count == 0 {
				next = append(next, t)
				continue
			}
			next = appendSplit(next, verts, t, split, midpoint)
		}
		if len(mid) == 0 {
			out.Vertices, out.Triangles = verts, tris
			return out, nil
		}
		if len(next) > MaxTriangles {
			return Result{}, fmt.Errorf("%w: more than %d triangles", ErrTooLarge, MaxTriangles)
		}
		if round+1 >= maxRounds {
			return Result{}, fmt.Errorf("%w: the edges are still long after %d rounds", ErrTooLarge, maxRounds)
		}
		tris = next
		out.Rounds++
		out.Splits += len(mid)
	}
}

// appendSplit cuts triangle t along its split edges (edge e runs from corner e
// to corner e+1) and appends the pieces, wound as t is.
func appendSplit(dst [][3]int, verts []r3.Vec, t [3]int, split [3]bool, midpoint func(a, b int) int) [][3]int {
	// Rotate the corners so that the split edges lead: with one split it is edge 0, with
	// two they are edges 0 and 1, and with three all.
	count := 0
	for _, s := range split {
		if s {
			count++
		}
	}
	rot := 0
	switch count {
	case 1:
		for e := range 3 {
			if split[e] {
				rot = e
			}
		}
	case 2:
		for e := range 3 {
			if split[e] && split[(e+1)%3] {
				rot = e
			}
		}
	}
	a, b, c := t[rot%3], t[(rot+1)%3], t[(rot+2)%3]
	switch count {
	case 1:
		m := midpoint(a, b)
		return append(dst, [3]int{a, m, c}, [3]int{m, b, c})
	case 2:
		mab, mbc := midpoint(a, b), midpoint(b, c)
		dst = append(dst, [3]int{mab, b, mbc})
		// The quad a, mab, mbc, c: cut it along its shorter diagonal.
		if verts[mbc].Sub(verts[a]).Len() <= verts[c].Sub(verts[mab]).Len() {
			return append(dst, [3]int{a, mab, mbc}, [3]int{a, mbc, c})
		}
		return append(dst, [3]int{mab, mbc, c}, [3]int{mab, c, a})
	default:
		mab, mbc, mca := midpoint(a, b), midpoint(b, c), midpoint(c, a)
		return append(dst, [3]int{a, mab, mca}, [3]int{mab, b, mbc}, [3]int{mca, mbc, c}, [3]int{mab, mbc, mca})
	}
}
