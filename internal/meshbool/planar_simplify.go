package meshbool

import (
	"context"
	"errors"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PlanarMesh is an exact stitched mesh before rounding: its exact vertices,
// their rounded floats, their pre-weld bounds, and per facet the corners, the
// source face and the facet bound.
type PlanarMesh struct {
	Xverts     []proof.Xpt
	Approx     []r3.Vec
	Xbeta      []float64
	Tris       [][3]int
	Src        []int
	FacetBound []float64
}

// planarRegion is one connected region of a candidate face: its facets and its
// one boundary loop, a directed vertex cycle that follows the facets' own
// winding.
type planarRegion struct {
	facets []int
	loop   []int
}

// SimplifyPlanarFaces re-triangulates every candidate planar face of in on its
// corners alone (agent-docs/coplanar-contact-design.md §7). It returns a new,
// compacted mesh and whether anything changed, and never modifies in. A face is
// left alone when it is not exactly planar, holds a vertex claim, has a hole,
// or has a corner that is not a float; the exact surface of every face is
// unchanged either way. A failure of the simplification itself, as opposed to
// the context, leaves the mesh unchanged.
func SimplifyPlanarFaces(ctx context.Context, in PlanarMesh) (PlanarMesh, bool, error) {
	return SimplifyPlanarFacesBudget(ctx, proofbound.NewWorkBudget(ctx), in)
}

// SimplifyPlanarFacesBudget is SimplifyPlanarFaces charging budget one work
// unit for each facet and vertex it visits and for each ear-clipping step. A
// budget error returns as it is.
func SimplifyPlanarFacesBudget(ctx context.Context, budget *proofbound.WorkBudget, in PlanarMesh) (PlanarMesh, bool, error) {
	out, changed, err := simplifyPlanarFaces(ctx, budget, in)
	if err != nil {
		if ctx.Err() != nil || !errors.Is(err, decaderr.ErrBooleanFailed) {
			return in, false, err
		}
		return in, false, nil
	}
	return out, changed, nil
}

func simplifyPlanarFaces(ctx context.Context, budget *proofbound.WorkBudget, in PlanarMesh) (PlanarMesh, bool, error) {
	// An open input is left to the closure audit: removing the vertices of a
	// crack could close it with new surface.
	if !stitchEdgesPaired(in.Tris, make(map[[2]int]uint8, len(in.Tris)+len(in.Tris)/2)) {
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
		n, rs, ok, err := planarCandidate(ctx, budget, in, owner, byFace[s], s)
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
		removable = removableVertices(in, owner, incident, regions)
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
	if !stitchEdgesPaired(out.Tris, edges) {
		return in, false, nil
	}
	return out, true, nil
}

// planarCandidate reads §7.1 step 1 for source face s: exactly planar, every
// vertex claim zero, and every connected region bounded by one loop.
func planarCandidate(ctx context.Context, budget *proofbound.WorkBudget, in PlanarMesh, owner map[[2]int]int, facets []int, s int) (proof.Xpt, []planarRegion, bool, error) {
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
			if proof.OrientSignExact(a, b, c, in.Xverts[v]) != 0 {
				return none, nil, false, nil
			}
		}
	}
	n := proof.Xcross(proof.Xsub(b, a), proof.Xsub(c, a))
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

// removableVertices is §7.1 step 2 over the current candidate set.
func removableVertices(in PlanarMesh, owner map[[2]int]int, incident [][]int, regions map[int][]planarRegion) []bool {
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
		removable[v] = collinearSeamVertex(in, owner, fs, v)
	}
	return removable
}

// collinearSeamVertex reports whether v, whose facets fs carry exactly two
// source ids, has exactly two boundary edges between them and lies strictly
// between their other ends, collinear with them.
func collinearSeamVertex(in PlanarMesh, owner map[[2]int]int, fs []int, v int) bool {
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
func planarCornersAreFloats(in PlanarMesh, rs []planarRegion, removable []bool) bool {
	for _, r := range rs {
		onLoop := make(map[int]struct{}, len(r.loop))
		for _, v := range r.loop {
			onLoop[v] = struct{}{}
			if !removable[v] && !proof.XptDenotes(in.Xverts[v], in.Approx[v]) {
				return false
			}
		}
		// Every vertex of the region is a corner or removed. A vertex that
		// another body also uses fails both, and would be left inside a facet
		// as a T-junction.
		for _, f := range r.facets {
			for _, v := range in.Tris[f] {
				if _, ok := onLoop[v]; !ok && !removable[v] {
					return false
				}
			}
		}
	}
	return true
}

// triangulateCorners triangulates the polygon of corner vertex ids, which
// follows the face's winding, by exact ear clipping in the face's dominant-axis
// projection. The triangles keep the polygon's winding.
func triangulateCorners(budget *proofbound.WorkBudget, in PlanarMesh, n proof.Xpt, corners []int) ([][3]int, error) {
	u, v := ProjAxes(n)
	pts := make([]Xp2, len(corners))
	poly := make([]int, len(corners))
	for i, c := range corners {
		pts[i] = NewXP2FromXpt(in.Xverts[c], u, v)
		poly[i] = i
	}
	sign, err := PolyArea2Sign(budget, pts)
	if err != nil {
		return nil, err
	}
	if sign == 0 {
		return nil, decaderr.ErrBooleanFailed
	}
	if sign < 0 {
		for i := range pts {
			pts[i] = xp2Swapped(pts[i])
		}
	}
	local, err := EarClipX(budget, pts, poly)
	if err != nil {
		return nil, err
	}
	tris := make([][3]int, len(local))
	for i, t := range local {
		tris[i] = [3]int{corners[t[0]], corners[t[1]], corners[t[2]]}
	}
	return tris, nil
}
