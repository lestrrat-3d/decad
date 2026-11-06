package meshbool

import (
	"context"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// This file retries a stitched mesh whose rounding the float-box search left
// folded (mesh_stitch.go). The usual cause is a chain of exact vertices that
// lie within an ulp of one line on a planar face: a cut vertex on a long,
// nearly straight edge of the operand's own triangulation beside a hole's
// chord, say. The exact mesh triangulates the chain with slivers whose apex
// sits less than an ulp off their base, rounding puts all three corners on
// one line, and a sliver whose corners are exact floats or welded points has
// no float-box corner to move to. The facets around the
// fold are flipped toward the Delaunay triangulation of their plane instead,
// which never keeps a sliver whose circumcircle holds a neighbouring vertex.
// Every flip is decided by exact predicates and keeps the exact surface, its
// source faces and its vertices, as the needle flips of mesh_needle.go do;
// the retried rounding is then checked exactly like the first.

// foldFlipRounds bounds how many times a stitch flips around its folds and
// rounds again.
const foldFlipRounds = 3

// foldFlipsPerFacet bounds the flips of one round per facet of the mesh. A
// Delaunay flip sequence terminates on its own; the cap only bounds the work.
const foldFlipsPerFacet = 2

// flipFoldsDelaunay finds the facets the first rounding of xverts folds and
// flips, in place, the shared edges around them that are not locally
// Delaunay, as flipDelaunayFrom states. It returns how many edges it flipped.
func flipFoldsDelaunay(ctx context.Context, verts []proof.Xpt, held []r3.Vec, tris [][3]int, src []int, bounds []float64) (int, error) {
	seeds, err := heldFoldSeeds(ctx, verts, held, tris)
	if err != nil || len(seeds) == 0 {
		return 0, err
	}
	return flipDelaunayFrom(ctx, verts, held, tris, src, bounds, seeds)
}

// heldFoldSeeds rounds the exact mesh as roundStitched does and returns, in
// ascending order, the exact facets whose held facets meet another facet
// improperly or collapse to a line there.
func heldFoldSeeds(ctx context.Context, verts []proof.Xpt, held []r3.Vec, tris [][3]int) ([]int, error) {
	floatIdx := make(map[r3.Vec]int, len(verts))
	remap := make([]int, len(verts))
	var hv []r3.Vec
	for i := range verts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		fi, ok := floatIdx[held[i]]
		if !ok {
			fi = len(hv)
			floatIdx[held[i]] = fi
			hv = append(hv, held[i])
		}
		remap[i] = fi
	}
	h := HeldRounding{
		Verts: hv,
		Exact: make([]proof.Xpt, len(hv)),
		Moved: make([]bool, len(hv)),
	}
	count := make([]int, len(hv))
	for i, p := range verts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		fi := remap[i]
		count[fi]++
		h.Exact[fi] = p
		if CoordDistance(p, hv[fi]).Sign() != 0 || count[fi] > 1 {
			h.Moved[fi] = true
		}
	}
	h.Movable = make([]bool, len(hv))
	var source []int
	for ti, tri := range tris {
		a, b, c := remap[tri[0]], remap[tri[1]], remap[tri[2]]
		if a == b || b == c || c == a {
			continue
		}
		h.Tris = append(h.Tris, [3]int{a, b, c})
		source = append(source, ti)
	}
	folds, err := HeldFolds(ctx, h)
	if err != nil {
		return nil, err
	}
	seen := make(map[int]struct{}, 2*len(folds))
	var seeds []int
	for _, pair := range folds {
		for _, f := range pair {
			if _, ok := seen[f]; ok {
				continue
			}
			seen[f] = struct{}{}
			seeds = append(seeds, source[f])
		}
	}
	slices.Sort(seeds)
	return seeds, nil
}

// flipDelaunayFrom flips, in place, shared edges reached from the facets in
// seeds. An edge (u, w) of facet (u, w, b) whose neighbour (w, u, d) has the
// same source is flipped to (u, d, b) and (d, w, b) when one of the two facets
// is a needle (needleEdge), the new diagonal is not already an edge, and
// delaunayFlip proves the two facets coplanar and strictly convex as a
// quadrilateral with d strictly inside the circle through u, w and b. The four
// outer edges of a flipped quadrilateral are visited next. Each new facet takes the larger of the two old facets' bounds. Every
// flip strictly lowers the triangulation's lifted Delaunay potential in that
// plane, so the visits end; foldFlipsPerFacet also bounds them. It returns how
// many edges it flipped.
func flipDelaunayFrom(ctx context.Context, verts []proof.Xpt, held []r3.Vec, tris [][3]int, src []int, bounds []float64, seeds []int) (int, error) {
	edges := make(map[[2]int]int, 3*len(tris))
	for t, tri := range tris {
		for k := range 3 {
			edges[[2]int{tri[k], tri[(k+1)%3]}] = t
		}
	}
	queued := make(map[[2]int]struct{})
	var queue [][2]int
	push := func(a, b int) {
		key := [2]int{min(a, b), max(a, b)}
		if _, ok := queued[key]; ok {
			return
		}
		queued[key] = struct{}{}
		queue = append(queue, key)
	}
	for _, t := range seeds {
		tri := tris[t]
		for k := range 3 {
			push(tri[k], tri[(k+1)%3])
		}
	}
	limit := foldFlipsPerFacet * len(tris)
	flips := 0
	for visit := 0; len(queue) > 0 && flips < limit; visit++ {
		if visit%256 == 0 {
			if err := ctx.Err(); err != nil {
				return flips, err
			}
		}
		key := queue[0]
		queue = queue[1:]
		delete(queued, key)
		u, w := key[0], key[1]
		t, ok := edges[[2]int{u, w}]
		if !ok {
			u, w = w, u
			t, ok = edges[[2]int{u, w}]
		}
		if !ok {
			continue
		}
		o, ok := edges[[2]int{w, u}]
		if !ok || o == t || src[o] != src[t] {
			continue
		}
		b, d := thirdCorner(tris[t], u, w), thirdCorner(tris[o], u, w)
		if b < 0 || d < 0 || b == d {
			continue
		}
		if _, needle := needleEdge(held, tris[t]); !needle {
			if _, needle := needleEdge(held, tris[o]); !needle {
				continue
			}
		}
		if _, exists := edges[[2]int{b, d}]; exists {
			continue
		}
		if _, exists := edges[[2]int{d, b}]; exists {
			continue
		}
		if !delaunayFlip(verts[u], verts[w], verts[b], verts[d]) {
			continue
		}
		left, right := [3]int{u, d, b}, [3]int{d, w, b}
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
		push(w, b)
		push(b, u)
		push(u, d)
		push(d, w)
	}
	return flips, nil
}

// thirdCorner returns the corner of tri that is neither u nor w, or -1 when
// tri does not hold both.
func thirdCorner(tri [3]int, u, w int) int {
	third, hits := -1, 0
	for _, v := range tri {
		if v == u || v == w {
			hits++
			continue
		}
		third = v
	}
	if hits != 2 {
		return -1
	}
	return third
}

// delaunayFlip decides exactly that the facets (u, w, b) and (w, u, d) lie in
// one plane, form a strictly convex quadrilateral u, d, w, b in it, so the
// facets (u, d, b) and (d, w, b) cover the same region with the same
// orientation, and that d lies strictly inside the circle through u, w and b.
// Both are read in the coordinate projection that keeps the plane invertible.
// The projection is affine, so it keeps the orientations; it maps the circle
// to an ellipse, so the circle test is the Delaunay one of the projected
// points, which only chooses which flips to try.
func delaunayFlip(pu, pw, pb, pd proof.Xpt) bool {
	if proof.OrientSignExact(pu, pw, pb, pd) != 0 {
		return false
	}
	n := proof.Xcross(proof.Xsub(pw, pu), proof.Xsub(pb, pu))
	if zeroXpt(n) {
		return false
	}
	iu, iv := ProjAxes(n)
	// Every coordinate is read relative to d; a translation changes no
	// orientation and no circle membership.
	du, dv := RatCoordOf(pd, iu), RatCoordOf(pd, iv)
	rel := func(p proof.Xpt) [2]*big.Rat {
		return [2]*big.Rat{new(big.Rat).Sub(RatCoordOf(p, iu), du), new(big.Rat).Sub(RatCoordOf(p, iv), dv)}
	}
	u, w, b := rel(pu), rel(pw), rel(pb)
	d := [2]*big.Rat{new(big.Rat), new(big.Rat)}
	side := orient2Rat(u, w, b)
	if side == 0 || orient2Rat(w, u, d) != side || orient2Rat(u, d, b) != side || orient2Rat(d, w, b) != side {
		return false
	}
	lift := func(p [2]*big.Rat) *big.Rat {
		l := new(big.Rat).Mul(p[0], p[0])
		return l.Add(l, new(big.Rat).Mul(p[1], p[1]))
	}
	rows := [3][3]*big.Rat{{u[0], u[1], lift(u)}, {w[0], w[1], lift(w)}, {b[0], b[1], lift(b)}}
	minor := func(i, j int) *big.Rat {
		m := new(big.Rat).Mul(rows[1][i], rows[2][j])
		return m.Sub(m, new(big.Rat).Mul(rows[2][i], rows[1][j]))
	}
	det := new(big.Rat).Mul(rows[0][0], minor(1, 2))
	det.Sub(det, new(big.Rat).Mul(rows[0][1], minor(0, 2)))
	det.Add(det, new(big.Rat).Mul(rows[0][2], minor(0, 1)))
	return det.Sign() == side
}

// orient2Rat is the sign of (b − a) × (c − a) for exact plane points.
func orient2Rat(a, b, c [2]*big.Rat) int {
	l := new(big.Rat).Mul(new(big.Rat).Sub(b[0], a[0]), new(big.Rat).Sub(c[1], a[1]))
	r := new(big.Rat).Mul(new(big.Rat).Sub(b[1], a[1]), new(big.Rat).Sub(c[0], a[0]))
	return l.Cmp(r)
}
