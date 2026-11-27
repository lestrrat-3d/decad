package meshbool

import (
	"context"
	"fmt"
	"math/big"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file keeps the map[[2]int]int versions of the planar-face simplifier
// and the needle flip as they were before they used EdgeTable. They exist only
// so the differential tests can compare the table-based code against them.

func mapRefSimplifyPlanarFaces(ctx context.Context, budget *proofbound.WorkBudget, in PlanarMesh) (PlanarMesh, bool, error) {
	// An open input is left to the closure audit: removing the vertices of a
	// crack could close it with new surface.
	if !stitchEdgesPairedMap(in.Tris, make(map[[2]int]uint8, len(in.Tris)+len(in.Tris)/2)) {
		return in, false, nil
	}
	owner := make(map[[2]int]int, 3*len(in.Tris))
	incident := make([][]int, len(in.Xverts))
	byFace := map[int][]int{}
	var faceOrder []int
	for f, tri := range in.Tris {
		if err := budget.Step(); err != nil {
			return in, false, err
		}
		if f%256 == 0 {
			if err := ctx.Err(); err != nil {
				return in, false, err
			}
		}
		for k := range 3 {
			owner[[2]int{tri[k], tri[(k+1)%3]}] = f
			incident[tri[k]] = append(incident[tri[k]], f)
		}
		s := in.Src[f]
		if _, ok := byFace[s]; !ok {
			faceOrder = append(faceOrder, s)
		}
		byFace[s] = append(byFace[s], f)
	}

	// Step 1: candidate faces.
	normals := map[int]proof.Xpt{}
	regions := map[int][]planarRegion{}
	for _, s := range faceOrder {
		n, rs, ok, err := mapRefPlanarCandidate(ctx, budget, in, owner, byFace[s], s)
		if err != nil {
			return in, false, err
		}
		if ok {
			normals[s], regions[s] = n, rs
		}
	}

	// Steps 2 and 3: the candidate set only shrinks.
	var removable []bool
	for {
		if err := ctx.Err(); err != nil {
			return in, false, err
		}
		if _, err := budget.StepN(uint64(len(in.Xverts))); err != nil {
			return in, false, err
		}
		removable = mapRefRemovableVertices(in, owner, incident, regions)
		dropped := false
		for _, s := range faceOrder {
			rs, ok := regions[s]
			if !ok {
				continue
			}
			if planarCornersAreFloats(in, rs, removable) {
				continue
			}
			delete(regions, s)
			delete(normals, s)
			dropped = true
		}
		if !dropped {
			break
		}
	}

	// Step 4: re-triangulate each region that loses a vertex.
	drop := make([]bool, len(in.Tris))
	var newTris [][3]int
	var newSrc []int
	changed := false
	for _, s := range faceOrder {
		for _, r := range regions[s] {
			loses := false
			for _, f := range r.facets {
				for _, v := range in.Tris[f] {
					if removable[v] {
						loses = true
					}
				}
			}
			if !loses {
				continue
			}
			var corners []int
			for _, v := range r.loop {
				if !removable[v] {
					corners = append(corners, v)
				}
			}
			tris, err := triangulateCorners(budget, in, normals[s], corners)
			if err != nil {
				return in, false, err
			}
			for _, f := range r.facets {
				drop[f] = true
			}
			for _, t := range tris {
				newTris = append(newTris, t)
				newSrc = append(newSrc, s)
			}
			changed = true
		}
	}
	if !changed {
		return in, false, nil
	}

	// Step 5: compact.
	keep := make([]bool, len(in.Xverts))
	var tris [][3]int
	var src []int
	var facetBound []float64
	mark := func(t [3]int) {
		for _, v := range t {
			keep[v] = true
		}
	}
	for f, tri := range in.Tris {
		if drop[f] {
			continue
		}
		tris, src, facetBound = append(tris, tri), append(src, in.Src[f]), append(facetBound, in.FacetBound[f])
		mark(tri)
	}
	for i, t := range newTris {
		tris, src, facetBound = append(tris, t), append(src, newSrc[i]), append(facetBound, 0)
		mark(t)
	}
	remap := make([]int, len(in.Xverts))
	out := PlanarMesh{Tris: tris, Src: src, FacetBound: facetBound}
	for v := range in.Xverts {
		if !keep[v] {
			remap[v] = -1
			continue
		}
		remap[v] = len(out.Xverts)
		out.Xverts = append(out.Xverts, in.Xverts[v])
		out.Approx = append(out.Approx, in.Approx[v])
		out.Xbeta = append(out.Xbeta, in.Xbeta[v])
	}
	for i, t := range out.Tris {
		out.Tris[i] = [3]int{remap[t[0]], remap[t[1]], remap[t[2]]}
	}
	edges := make(map[[2]int]uint8, len(out.Tris)+len(out.Tris)/2)
	if !stitchEdgesPairedMap(out.Tris, edges) {
		return in, false, nil
	}
	return out, true, nil
}

// mapRefPlanarCandidate reads §7.1 step 1 for source face s: exactly planar, every
// vertex claim zero, and every connected region bounded by one loop.
func mapRefPlanarCandidate(ctx context.Context, budget *proofbound.WorkBudget, in PlanarMesh, owner map[[2]int]int, facets []int, s int) (proof.Xpt, []planarRegion, bool, error) {
	var none proof.Xpt
	first := in.Tris[facets[0]]
	a, b, c := in.Xverts[first[0]], in.Xverts[first[1]], in.Xverts[first[2]]
	for _, f := range facets {
		for _, v := range in.Tris[f] {
			if in.Xbeta[v] != 0 {
				return none, nil, false, nil
			}
		}
	}
	// The reference normal is constant for the face, and a vertex is tested
	// once however many facets it touches. n·(v−a) has the sign of
	// det[b−a, c−a, v−a], so a collinear reference (n = 0) reads zero
	// everywhere and is refused after the loop.
	n := proof.Xcross(proof.Xsub(b, a), proof.Xsub(c, a))
	var scratch [3]big.Int
	tested := make(map[int]struct{}, len(facets)+2)
	for _, f := range facets {
		if err := budget.Step(); err != nil {
			return none, nil, false, err
		}
		if err := ctx.Err(); err != nil {
			return none, nil, false, err
		}
		for _, v := range in.Tris[f] {
			if v == first[0] || v == first[1] || v == first[2] {
				continue
			}
			if _, done := tested[v]; done {
				continue
			}
			tested[v] = struct{}{}
			if proof.XdotDiffSignScratch(&scratch, n, in.Xverts[v], a) != 0 {
				return none, nil, false, nil
			}
		}
	}
	if n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0 {
		// A collinear reference facet gives no plane to test against.
		return none, nil, false, nil
	}

	// Connected regions through shared edges, and each region's boundary.
	region := map[int]int{}
	var out []planarRegion
	for _, f := range facets {
		if _, ok := region[f]; ok {
			continue
		}
		id := len(out)
		var r planarRegion
		region[f] = id
		queue := []int{f}
		for len(queue) > 0 {
			g := queue[0]
			queue = queue[1:]
			r.facets = append(r.facets, g)
			t := in.Tris[g]
			for k := range 3 {
				if twin, ok := owner[[2]int{t[(k+1)%3], t[k]}]; ok && in.Src[twin] == s {
					if _, seen := region[twin]; !seen {
						region[twin] = id
						queue = append(queue, twin)
					}
				}
			}
		}
		out = append(out, r)
	}
	for i := range out {
		next := map[int]int{}
		boundary := 0
		for _, f := range out[i].facets {
			t := in.Tris[f]
			for k := range 3 {
				u, w := t[k], t[(k+1)%3]
				if twin, ok := owner[[2]int{w, u}]; ok && in.Src[twin] == s {
					continue
				}
				if _, dup := next[u]; dup {
					return none, nil, false, nil
				}
				next[u] = w
				boundary++
			}
		}
		if boundary < 3 {
			return none, nil, false, nil
		}
		// The walk starts at the least boundary vertex, not at a corner of the
		// first facet, which may lie inside the region.
		start := -1
		for u := range next {
			if start < 0 || u < start {
				start = u
			}
		}
		var loop []int
		for v := start; ; {
			loop = append(loop, v)
			w, ok := next[v]
			if !ok || len(loop) > boundary {
				return none, nil, false, nil
			}
			v = w
			if v == start {
				break
			}
		}
		if len(loop) != boundary {
			return none, nil, false, nil
		}
		out[i].loop = loop
	}
	return n, out, true, nil
}

// mapRefRemovableVertices is §7.1 step 2 over the current candidate set.
func mapRefRemovableVertices(in PlanarMesh, owner map[[2]int]int, incident [][]int, regions map[int][]planarRegion) []bool {
	removable := make([]bool, len(in.Xverts))
	for v, fs := range incident {
		if len(fs) == 0 {
			continue
		}
		s := in.Src[fs[0]]
		if _, ok := regions[s]; !ok {
			continue
		}
		var other = -1
		single := true
		for _, f := range fs {
			if in.Src[f] == s {
				continue
			}
			single = false
			if other >= 0 && in.Src[f] != other {
				other = -2
				break
			}
			other = in.Src[f]
		}
		if single {
			removable[v] = true
			continue
		}
		if other < 0 {
			continue
		}
		if _, ok := regions[other]; !ok {
			continue
		}
		removable[v] = mapRefCollinearSeamVertex(in, owner, fs, v)
	}
	return removable
}

// mapRefCollinearSeamVertex reports whether v, whose facets fs carry exactly two
// source ids, has exactly two boundary edges between them and lies strictly
// between their other ends, collinear with them.
func mapRefCollinearSeamVertex(in PlanarMesh, owner map[[2]int]int, fs []int, v int) bool {
	var ends []int
	for _, f := range fs {
		t := in.Tris[f]
		for k := range 3 {
			if t[k] != v {
				continue
			}
			// The edge leaving v and the edge entering it.
			out, enter := t[(k+1)%3], t[(k+2)%3]
			if twin, ok := owner[[2]int{out, v}]; ok && in.Src[twin] != in.Src[f] {
				ends = append(ends, out)
			}
			if twin, ok := owner[[2]int{v, enter}]; ok && in.Src[twin] != in.Src[f] {
				ends = append(ends, enter)
			}
		}
	}
	// Each seam edge is seen once from each side.
	if len(ends) != 4 {
		return false
	}
	slices.Sort(ends)
	if ends[0] != ends[1] || ends[2] != ends[3] || ends[1] == ends[2] {
		return false
	}
	p, q := in.Xverts[ends[0]], in.Xverts[ends[2]]
	x := in.Xverts[v]
	cross := proof.Xcross(proof.Xsub(p, x), proof.Xsub(q, x))
	if cross.X.Sign() != 0 || cross.Y.Sign() != 0 || cross.Z.Sign() != 0 {
		return false
	}
	return PointOnSegment3D(p, q, x)
}

// planarCornersAreFloats is §7.1 step 3 for the regions of one face: every
// boundary vertex that stays is the exact point of its own float.
func mapRefFlipNeedlesHeld(ctx context.Context, verts []proof.Xpt, held []r3.Vec, tris [][3]int, src []int, bounds []float64) (int, error) {
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

// simplifyGridBox is the closed box [0,n]^3 on integer points, every side an
// n x n grid of squares cut into two facets. Each side is one source face; with
// split, each side's two halves are two source faces, so seam vertices and
// boundary loops arise inside it.
func simplifyGridBox(n int, split bool) PlanarMesh {
	var in PlanarMesh
	index := map[[3]int]int{}
	vert := func(p [3]int) int {
		if v, ok := index[p]; ok {
			return v
		}
		v := len(in.Xverts)
		index[p] = v
		c := r3.Vec{X: float64(p[0]), Y: float64(p[1]), Z: float64(p[2])}
		in.Xverts = append(in.Xverts, proof.XptOf(c))
		in.Approx = append(in.Approx, c)
		in.Xbeta = append(in.Xbeta, 0)
		return v
	}
	for axis := range 3 {
		for side := range 2 {
			point := func(i, j int) int {
				var p [3]int
				p[axis] = side * n
				p[(axis+1)%3], p[(axis+2)%3] = i, j
				return vert(p)
			}
			for i := range n {
				for j := range n {
					a, b, c, d := point(i, j), point(i+1, j), point(i+1, j+1), point(i, j+1)
					tris := [][3]int{{a, b, c}, {a, c, d}}
					if side == 0 {
						tris = [][3]int{{a, c, b}, {a, d, c}}
					}
					s := 2*axis + side
					if split && i >= n/2 {
						s += 6
					}
					for _, tri := range tris {
						in.Tris = append(in.Tris, tri)
						in.Src = append(in.Src, s)
						in.FacetBound = append(in.FacetBound, 0)
					}
				}
			}
		}
	}
	return in
}

// clonePlanarMesh copies every slice of in, so a mutation of the copy cannot
// reach the original. The points are immutable values and are shared.
func clonePlanarMesh(in PlanarMesh) PlanarMesh {
	in.Xverts = slices.Clone(in.Xverts)
	in.Approx = slices.Clone(in.Approx)
	in.Xbeta = slices.Clone(in.Xbeta)
	in.Tris = slices.Clone(in.Tris)
	in.Src = slices.Clone(in.Src)
	in.FacetBound = slices.Clone(in.FacetBound)
	return in
}

// planarMeshDigest renders every field of m, with each exact point as its key,
// so two meshes compare byte for byte.
func planarMeshDigest(m PlanarMesh) string {
	var b strings.Builder
	for _, p := range m.Xverts {
		fmt.Fprintf(&b, "x%v;", p.Key())
	}
	fmt.Fprintf(&b, "\nA%v\nB%v\nT%v\nS%v\nF%v", m.Approx, m.Xbeta, m.Tris, m.Src, m.FacetBound)
	return b.String()
}
