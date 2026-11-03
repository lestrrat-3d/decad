package meshbool

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// This file removes needles from the exact stitched mesh before it is rounded
// (mesh_stitch.go). A needle is a facet whose apex lies within a few ulps of
// its longest edge. Its exact points are embedded, but rounding each one to a
// float can carry the apex across that edge, and the float-box search in
// held_embedding.go cannot separate geometry thinner than one float step. When
// the facet across the needle's longest edge is exactly coplanar with it and
// comes from the same source face, flipping that shared edge replaces the two
// facets by two others over the same exact quadrilateral. The exact surface,
// its source faces and its vertices are unchanged, so every later proof reads
// the same surface.

// needleUlps is how many float steps of the facet's largest coordinate the
// apex may sit from its longest edge for the facet to count as a needle. The
// figure only chooses which facets the pass tries to flip: every flip is
// decided by exact predicates and keeps the exact surface.
const needleUlps = 64

// needlePasses bounds the passes over the mesh. A pass that flips nothing
// ends early.
const needlePasses = 4

// FlipNeedles flips, in place, the longest edge of each needle facet of tris
// whose neighbour across that edge has the same source and lies exactly in
// its plane, when the two facets form a strictly convex quadrilateral in that
// plane, the new diagonal is not already an edge, and neither new facet is a
// needle. Each new facet takes the larger of the two old facets' bounds. It
// returns how many edges it flipped.
func FlipNeedles(ctx context.Context, verts []proof.Xpt, tris [][3]int, src []int, bounds []float64) (int, error) {
	held := make([]r3.Vec, len(verts))
	for i, p := range verts {
		held[i] = p.Vec()
	}
	return flipNeedlesHeld(ctx, verts, held, tris, src, bounds)
}

// flipNeedlesHeld is FlipNeedles with held[i] = verts[i].Vec() supplied by
// the caller; it only reads held.
func flipNeedlesHeld(ctx context.Context, verts []proof.Xpt, held []r3.Vec, tris [][3]int, src []int, bounds []float64) (int, error) {
	edges := make(map[[2]int]int, 3*len(tris))
	for t, tri := range tris {
		for k := range 3 {
			edges[[2]int{tri[k], tri[(k+1)%3]}] = t
		}
	}
	flips := 0
	for range needlePasses {
		changed := false
		for t := range tris {
			if t%256 == 0 {
				if err := ctx.Err(); err != nil {
					return flips, err
				}
			}
			k, ok := needleEdge(held, tris[t])
			if !ok {
				continue
			}
			tri := tris[t]
			u, w, b := tri[k], tri[(k+1)%3], tri[(k+2)%3]
			o, ok := edges[[2]int{w, u}]
			if !ok || o == t || src[o] != src[t] {
				continue
			}
			other := tris[o]
			d := -1
			for j := range 3 {
				if other[j] != u && other[j] != w {
					d = other[j]
				}
			}
			if d < 0 || d == b {
				continue
			}
			if _, exists := edges[[2]int{b, d}]; exists {
				continue
			}
			if _, exists := edges[[2]int{d, b}]; exists {
				continue
			}
			left, right := [3]int{u, d, b}, [3]int{d, w, b}
			if _, needle := needleEdge(held, left); needle {
				continue
			}
			if _, needle := needleEdge(held, right); needle {
				continue
			}
			if !flipKeepsSurface(verts, held, u, w, b, d) {
				continue
			}
			for _, e := range [][2]int{{u, w}, {w, b}, {b, u}, {w, u}, {u, d}, {d, w}} {
				delete(edges, e)
			}
			tris[t], tris[o] = left, right
			bound := max(bounds[t], bounds[o])
			bounds[t], bounds[o] = bound, bound
			for k := range 3 {
				edges[[2]int{left[k], left[(k+1)%3]}] = t
				edges[[2]int{right[k], right[(k+1)%3]}] = o
			}
			flips++
			changed = true
		}
		if !changed {
			break
		}
	}
	return flips, nil
}

// needleEdge returns the index k of tri's longest edge (tri[k], tri[k+1]) when
// the opposite vertex lies within needleUlps float steps of that edge's line,
// read on the rounded points.
func needleEdge(held []r3.Vec, tri [3]int) (int, bool) {
	k, longest := 0, -1.0
	for j := range 3 {
		if l := held[tri[(j+1)%3]].Sub(held[tri[j]]).Len(); l > longest {
			k, longest = j, l
		}
	}
	if !(longest > 0) {
		return k, true
	}
	u, w, b := held[tri[k]], held[tri[(k+1)%3]], held[tri[(k+2)%3]]
	height := w.Sub(u).Cross(b.Sub(u)).Len() / longest
	scale := 0.0
	for _, p := range []r3.Vec{u, w, b} {
		scale = max(scale, math.Abs(p.X), math.Abs(p.Y), math.Abs(p.Z))
	}
	ulp := math.Nextafter(scale, math.Inf(1)) - scale
	return k, height <= needleUlps*ulp
}

// flipKeepsSurface decides exactly that the facets (u, w, b) and (w, u, d)
// lie in one plane and form a strictly convex quadrilateral u, d, w, b in it,
// so the facets (u, d, b) and (d, w, b) cover the same region with the same
// orientation.
func flipKeepsSurface(verts []proof.Xpt, held []r3.Vec, u, w, b, d int) bool {
	pu, pw, pb, pd := verts[u], verts[w], verts[b], verts[d]
	if proof.OrientSignExact(pu, pw, pb, pd) != 0 {
		return false
	}
	// A point off the plane reads every in-plane side as an orientation sign.
	// The float normal of the wide facet only proposes it; the exact sign
	// test below rejects one that lies in the plane.
	n := held[w].Sub(held[u]).Cross(held[d].Sub(held[u]))
	l := n.Len()
	if !(l > 0) {
		return false
	}
	scale := held[w].Sub(held[u]).Len() + held[d].Sub(held[u]).Len()
	pe := proof.XptOf(held[u].Add(n.Scale(scale / l)))
	side := proof.OrientSignExact(pu, pw, pb, pe)
	if side == 0 {
		return false
	}
	same := func(a, b2, c proof.Xpt) bool { return proof.OrientSignExact(a, b2, c, pe) == side }
	// The old facets as given, then the new ones: all four must turn the
	// same way about the off-plane point, which in one plane is exactly a
	// strictly convex quadrilateral split either way.
	return same(pw, pu, pd) && same(pu, pd, pb) && same(pd, pw, pb)
}
