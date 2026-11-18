package retainedblend

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file rebuilds the faces a cross-plane chain touches (agent-docs/retained-modify-reach-design.md §15b): each
// plane face's runs of chain edges give way to their setback contours, each cylinder face is cut at the setback
// plane of each arc it borders, and each end plane gives up the corner its end column cuts off.

// facePoly is the region a run or an end removes from a plane face, as exact points in the face's plane, with the
// held vertices that lie on its boundary (a column that meets a held corner) and the sagitta its arc chords may leave
// beyond it.
type facePoly struct {
	pts    []r3.Vec
	keepAt map[int]int
	sag    float64
	rays   []polyRay
}

// polyRay is a boundary line of a removed region that the face's kept boundary continues along beyond the setback: a
// seam, or an end plane's edge. A kept chord along it, beyond from, runs on in line with the region's own edge.
type polyRay struct {
	o, dir r3.Vec
	from   float64
}

// onRay reports whether the chord ab runs along one of the region's rays beyond its setback.
func (cx *crossChain) onRay(pg facePoly, a, b int) bool {
	w := cx.w
	for _, r := range pg.rays {
		ok := true
		for _, v := range []int{a, b} {
			p := w.verts[v]
			ok = ok && distToLine(p, r.o, r.dir) <= w.beta[v]+cx.slack && tau(p, r.o, r.dir) > r.from
		}
		if ok {
			return true
		}
	}
	return false
}

// crossRun is one run of consecutive chain edges on a plane face: the chain points it runs between, the held loop
// vertices it covers, the setback contour that replaces them and the region it removes.
type crossRun struct {
	edges []int
	s, e  int
	path  []int
	dir   int
	extS  []int
	extE  []int
	repl  []int
	poly  facePoly
}

// runsOn groups the chain edges that border face f into maximal runs of consecutive edges.
func (cx *crossChain) runsOn(f *topo.Face) ([][]int, error) {
	n := len(cx.edges)
	on := func(i int) bool { return cx.edges[i].faces[0] == f || cx.edges[i].faces[1] == f }
	start := 0
	if !cx.open {
		start = -1
		for i := range n {
			if on(i) && !on((i-1+n)%n) {
				start = i
				break
			}
		}
		if start < 0 {
			return nil, refuseCross("borders one face with every edge")
		}
	}
	var runs [][]int
	var cur []int
	for k := range n {
		i := (start + k) % n
		if on(i) {
			cur = append(cur, i)
			continue
		}
		if len(cur) > 0 {
			runs = append(runs, cur)
			cur = nil
		}
	}
	if len(cur) > 0 {
		runs = append(runs, cur)
	}
	return runs, nil
}

// onEdge reports whether held vertex v lies on chain edge ei within its bound, and borders only the edge's faces.
func (cx *crossChain) onEdge(ei, v int) bool {
	ce := &cx.edges[ei]
	w := cx.w
	p := w.verts[v]
	b := w.beta[v] + cx.slack
	if !cx.onlyFaces(v, ce.faces[0], ce.faces[1]) {
		return false
	}
	if !ce.arc {
		t := p.Sub(ce.p0).Dot(ce.d)
		return distToLine(p, ce.p0, ce.d) <= b && t >= -b && t <= ce.len+b
	}
	if !ce.rim.onRim(p, w.beta[v]) {
		return false
	}
	s := ce.param(p)
	ang := b/ce.rim.radius + 1e-9
	return s >= -ang && s <= ce.span+ang
}

// runPath walks the face loop from the run's first chain point along its edges to its last, in the one direction in
// which every vertex between lies on the edge being walked.
func (cx *crossChain) runPath(loop []int, r *crossRun) error {
	n := len(loop)
	pos0 := indexIn(loop, cx.pts[r.s].hv)
	found := 0
	for _, dir := range []int{1, -1} {
		path, pos, ok := []int{loop[pos0]}, pos0, true
		for _, ei := range r.edges {
			target := cx.pts[cx.edges[ei].to].hv
			for steps := 0; ; steps++ {
				if steps >= n {
					ok = false
					break
				}
				pos = (pos + dir + n) % n
				v := loop[pos]
				path = append(path, v)
				if v == target {
					break
				}
				if !cx.onEdge(ei, v) {
					ok = false
					break
				}
			}
			if !ok {
				break
			}
		}
		if ok {
			found++
			r.path, r.dir = path, dir
		}
	}
	if found != 1 {
		return refuseCross("has a run that does not follow its face's loop")
	}
	return nil
}

// runExt reads what the setback removes beyond the run's end at chain point k, walking face f's loop from the held
// vertex hv by step away from the run. At a free end it is G's chain along f ∩ G (readEnd). At a junction f is a
// wall, and the seam it shares with the next wall runs from the corner along the base's normal into the material:
// its vertices nearer than the setback go, and the first beyond it stays.
func (cx *crossChain) runExt(loop []int, f *topo.Face, k, hv, step int) ([]int, error) {
	w := cx.w
	n := len(loop)
	pos := indexIn(loop, hv)
	at := func(i int) int { return loop[((pos+step*i)%n+n)%n] }
	pt := cx.pts[k]
	if end := pt.end; end != nil {
		x := slices.Index(end.faces[:], f)
		for i, c := range end.rem[x] {
			if at(i+1) != c {
				return nil, refuseCross("has an end whose face and end plane disagree along their edge")
			}
		}
		if end.held[x] && at(len(end.rem[x])+1) != end.vert[x] {
			return nil, refuseCross("has an end whose face and end plane disagree along their edge")
		}
		return end.rem[x], nil
	}
	var ext []int
	for i := 1; i < n; i++ {
		v := at(i)
		p := w.verts[v]
		if distToLine(p, pt.exact, pt.nP) > w.beta[v]+cx.slack {
			return nil, refuseCross("has a seam that ends within the chamfer's setback")
		}
		dep := -pt.nP.Dot(p.Sub(pt.exact))
		if len(cx.facesOf(v)) == 2 && cx.onlyFaces(v, pt.j.wall1, pt.j.wall2) {
			// A vertex inside the seam is no corner of either wall: it goes or stays by its held place along the
			// seam, as the wall beyond cuts it (clipCylinderFace).
			switch {
			case dep > cx.d+cx.margin:
				return ext, nil
			case dep < cx.d-cx.margin && dep > -cx.margin:
				ext = append(ext, v)
				continue
			}
			return nil, fmt.Errorf("%w: a seam vertex lies at the chamfer's setback", decaderr.ErrUnsupported)
		}
		// A corner of the wall stays when it lies beyond the setback by more than its bound.
		if dep > cx.d+4*(w.beta[v]+cx.margin) {
			return ext, nil
		}
		return nil, refuseCross("has a corner of a wall within the chamfer's setback")
	}
	return nil, refuseCross("has a seam that ends within the chamfer's setback")
}

// heldAt is the held corner of G that the column of free end k meets on face f, or -1.
func (cx *crossChain) heldAt(k int, f *topo.Face) int {
	end := cx.pts[k].end
	if end == nil {
		return -1
	}
	x := slices.Index(end.faces[:], f)
	if x < 0 || !end.held[x] {
		return -1
	}
	return end.vert[x]
}

// sectionAt is the exact point of chain point k's section on face f: a free end's column point, or the row vertex.
func (cx *crossChain) sectionAt(k int, f *topo.Face) (r3.Vec, error) {
	if end := cx.pts[k].end; end != nil {
		if x := slices.Index(end.faces[:], f); x >= 0 {
			return end.q[x], nil
		}
	}
	v, err := cx.onFace(k, f)
	if err != nil {
		return r3.Vec{}, err
	}
	return cx.w.verts[v], nil
}

// runRepl builds the run's setback contour on face f, from its first chain point to its last, and the region it
// removes: the exact run, then the contour back.
func (cx *crossChain) runRepl(f *topo.Face, r *crossRun) error {
	var repl []int
	var old, cut []r3.Vec
	add := func(k int) error {
		v, err := cx.onFace(k, f)
		if err != nil {
			return err
		}
		p, err := cx.sectionAt(k, f)
		if err != nil {
			return err
		}
		repl, old, cut = append(repl, v), append(old, cx.pts[k].exact), append(cut, p)
		return nil
	}
	if err := add(r.s); err != nil {
		return err
	}
	sag := 0.0
	for _, ei := range r.edges {
		ce := &cx.edges[ei]
		if ce.arc && ce.faces[0] == f {
			for _, c := range ce.cols[1 : len(ce.cols)-1] {
				repl, old, cut = append(repl, c.on[0]), append(old, c.at), append(cut, cx.w.verts[c.on[0]])
			}
			step := ce.span / float64(len(ce.cols)-1)
			rho := max(ce.rim.radius, ce.rim.radius+ce.rim.kappa*cx.d)
			sag = max(sag, rho*(1-math.Cos(step/2)))
		}
		if err := add(ce.to); err != nil {
			return err
		}
	}
	poly := facePoly{keepAt: map[int]int{}, sag: sag}
	for _, k := range []int{r.s, r.e} {
		pt := cx.pts[k]
		if pt.end == nil {
			poly.rays = append(poly.rays, polyRay{o: pt.exact, dir: pt.nP.Scale(-1), from: cx.d})
			continue
		}
		x := slices.Index(pt.end.faces[:], f)
		poly.rays = append(poly.rays, polyRay{o: pt.exact, dir: pt.end.dirs[x], from: pt.end.reach[x]})
		if pt.end.held[x] {
			poly.rays = append(poly.rays, pt.end.past[x].onX)
		}
	}
	poly.pts = append(poly.pts, old...)
	for _, p := range slices.Backward(cut) {
		poly.pts = append(poly.pts, p)
	}
	if h := cx.heldAt(r.s, f); h >= 0 {
		poly.keepAt[h] = 2*len(cut) - 1
		repl = repl[1:]
	}
	if h := cx.heldAt(r.e, f); h >= 0 {
		poly.keepAt[h] = len(cut)
		repl = repl[:len(repl)-1]
	}
	r.repl, r.poly = repl, poly
	return nil
}

// rewritePlaneFace replaces each run of chain edges on the plane face f by its setback contour, checks that the rest
// of the face's boundary clears what the runs remove, and triangulates the face again. It returns the face's group.
func (cx *crossChain) rewritePlaneFace(f *topo.Face) (int, error) {
	w := cx.w
	pl, _ := retainedPlane(f)
	fl, err := w.faceLoops(cx.faceIndex[f], &pl)
	if err != nil {
		return 0, err
	}
	groups, err := cx.runsOn(f)
	if err != nil {
		return 0, err
	}
	gone := map[int]bool{}
	runs := make([]crossRun, len(groups))
	for i, edges := range groups {
		r := &runs[i]
		r.edges = edges
		r.s, r.e = cx.edges[edges[0]].from, cx.edges[edges[len(edges)-1]].to
		li, ok := fl.loopWith(cx.pts[r.s].hv, cx.pts[r.e].hv)
		if !ok {
			return 0, refuseCross("has a run whose ends lie on no one loop of its face")
		}
		loop := fl.loops[li]
		if err := cx.runPath(loop, r); err != nil {
			return 0, err
		}
		if r.extS, err = cx.runExt(loop, f, r.s, cx.pts[r.s].hv, -r.dir); err != nil {
			return 0, err
		}
		if r.extE, err = cx.runExt(loop, f, r.e, cx.pts[r.e].hv, r.dir); err != nil {
			return 0, err
		}
		if err := cx.runRepl(f, r); err != nil {
			return 0, err
		}
		for _, vs := range [][]int{r.path, r.extS, r.extE} {
			for _, v := range vs {
				if gone[v] {
					return 0, refuseCross("has two runs that remove one vertex of a face")
				}
				gone[v] = true
			}
		}
	}
	polys := make([]facePoly, len(runs))
	for i := range runs {
		polys[i] = runs[i].poly
	}
	if err := cx.gateChords(pl.n, fl.loops, gone, polys); err != nil {
		return 0, err
	}
	loops := make([][]int, len(fl.loops))
	for k, lp := range fl.loops {
		loops[k] = slices.Clone(lp)
	}
	for _, r := range runs {
		set := map[int]bool{}
		for _, vs := range [][]int{r.path, r.extS, r.extE} {
			for _, v := range vs {
				set[v] = true
			}
		}
		repl := slices.Clone(r.repl)
		if r.dir < 0 {
			slices.Reverse(repl)
		}
		if err := spliceRun(loops, set, repl); err != nil {
			return 0, err
		}
	}
	if err := w.retriangulate(pl.n, loops, fl.group); err != nil {
		return 0, err
	}
	w.removed = append(w.removed, fl.facets...)
	return fl.group, nil
}

// spliceRun replaces the contiguous run set of one of the loops by repl, which runs in the loop's direction. A run
// that is the whole loop gives the loop repl alone.
func spliceRun(loops [][]int, set map[int]bool, repl []int) error {
	li := slices.IndexFunc(loops, func(lp []int) bool { return slices.ContainsFunc(lp, func(v int) bool { return set[v] }) })
	if li < 0 {
		return fmt.Errorf("%w: a face boundary holds no run to replace", decaderr.ErrUnsupported)
	}
	if !slices.ContainsFunc(loops[li], func(v int) bool { return !set[v] }) {
		if len(loops[li]) != len(set) {
			return fmt.Errorf("%w: a face boundary breaks a run into pieces", decaderr.ErrUnsupported)
		}
		loops[li] = repl
		return nil
	}
	outside, _, err := splice(loops[li], set)
	if err != nil {
		return err
	}
	loops[li] = append(outside, repl...)
	return nil
}

// gateChords refuses a kept boundary chord of a plane face with outward normal n that reaches a region polys
// removes (§5.6's base band, for each run): a chord with an end inside it consumes the face, and one that comes
// within its bounds of it is undecided. A chord that leaves a held vertex on a region's boundary must leave it
// outward.
func (cx *crossChain) gateChords(n r3.Vec, loops [][]int, gone map[int]bool, polys []facePoly) error {
	u, ok := perpendicular(n)
	if !ok {
		return fmt.Errorf("%w: a face has no plane frame", decaderr.ErrUnsupported)
	}
	v := n.Cross(u)
	to2 := func(p r3.Vec) pt2 { return pt2{p.Dot(u), p.Dot(v)} }
	for _, lp := range loops {
		for i, a := range lp {
			b := lp[(i+1)%len(lp)]
			if gone[a] || gone[b] {
				continue
			}
			for _, pg := range polys {
				if err := cx.gateChord(a, b, pg, to2); err != nil {
					return err
				}
			}
		}
	}
	return nil
}

func (cx *crossChain) gateChord(a, b int, pg facePoly, to2 func(r3.Vec) pt2) error {
	w := cx.w
	if cx.onRay(pg, a, b) {
		return nil
	}
	ia, aOn := pg.keepAt[a]
	_, bOn := pg.keepAt[b]
	if aOn && bOn {
		return refuseCross("has a removed region whose two held corners a face boundary joins")
	}
	if bOn {
		a, b = b, a
		ia, aOn = pg.keepAt[a]
	}
	pa, pb := w.verts[a], w.verts[b]
	tol := max(w.beta[a], w.beta[b]) + cx.margin + pg.sag
	np := len(pg.pts)
	ps := make([]pt2, np)
	for i, p := range pg.pts {
		ps[i] = to2(p)
	}
	if aOn {
		if err := leavesOutward(ps, ia, to2(pb)); err != nil {
			return err
		}
	} else if inPolygon(ps, to2(pa)) {
		return fmt.Errorf("%w: the blend consumes a face beside the chain", decaderr.ErrDegenerate)
	}
	if inPolygon(ps, to2(pb)) {
		return fmt.Errorf("%w: the blend consumes a face beside the chain", decaderr.ErrDegenerate)
	}
	gap := math.Inf(1)
	for i := range np {
		j := (i + 1) % np
		if aOn && (i == ia || j == ia) {
			continue
		}
		gap = math.Min(gap, segmentDistance(pa, pb, pg.pts[i], pg.pts[j]))
	}
	return clearance(gap, tol, "a face beside the chain")
}

// leavesOutward refuses a direction from polygon vertex i toward q that is not clearly outside the polygon's
// interior angle there, which must be less than a half turn.
func leavesOutward(ps []pt2, i int, q pt2) error {
	n := len(ps)
	area := 0.0
	for k := range n {
		a, b := ps[k], ps[(k+1)%n]
		area += a.u*b.w - b.u*a.w
	}
	s := 1.0
	if area < 0 {
		s = -1
	}
	c, prev, next := ps[i], ps[(i-1+n)%n], ps[(i+1)%n]
	en := pt2{next.u - c.u, next.w - c.w}
	ep := pt2{prev.u - c.u, prev.w - c.w}
	uu := pt2{q.u - c.u, q.w - c.w}
	cross := func(a, b pt2) float64 { return s * (a.u*b.w - a.w*b.u) }
	length := func(a pt2) float64 { return math.Hypot(a.u, a.w) }
	if !(cross(en, ep) > minSine*length(en)*length(ep)) {
		return refuseCross("has a removed region whose corner on a held vertex is not convex")
	}
	if cross(en, uu) < -minSine*length(en)*length(uu) || cross(uu, ep) < -minSine*length(uu)*length(ep) {
		return nil
	}
	return fmt.Errorf("%w: the held bounds cannot show that the blend leaves a face beside the chain",
		decaderr.ErrUnsupported)
}

// inPolygon reports whether p lies inside the polygon ps, by the crossing count.
func inPolygon(ps []pt2, p pt2) bool {
	in := false
	n := len(ps)
	for i := range n {
		a, b := ps[i], ps[(i+1)%n]
		if (a.w > p.w) != (b.w > p.w) {
			x := a.u + (p.w-a.w)*(b.u-a.u)/(b.w-a.w)
			if p.u < x {
				in = !in
			}
		}
	}
	return in
}

// clipCylinderFace cuts the cylinder face f at the setback plane of each chain arc it borders, in turn: the plane
// parallel to the arc's plane face at the setback's depth (§5.5 step 4). The seam at each end of the arc is cut at the
// junction's row vertex on it, which the plane face beyond the seam shares. It returns the face's group and, per arc,
// the vertices its cut made or used.
func (cx *crossChain) clipCylinderFace(f *topo.Face) (int, map[int]map[int]bool, error) {
	w := cx.w
	facets := facetsOfFace(cx.fp.FaceOf, cx.faceIndex[f])
	group, err := w.groupOf(facets)
	if err != nil {
		return 0, nil, err
	}
	tris := make([][3]int, len(facets))
	for i, t := range facets {
		tris[i] = cx.fp.Tris[t]
	}
	cuts := map[int]map[int]bool{}
	for ei := range cx.edges {
		ce := &cx.edges[ei]
		if !ce.arc || ce.faces[1] != f {
			continue
		}
		a := ce.pl[0]
		depth := func(p r3.Vec) float64 { return -a.n.Dot(p.Sub(a.o)) }
		g := map[int]float64{}
		for _, t := range tris {
			for _, v := range t {
				if _, seen := g[v]; seen {
					continue
				}
				p := w.verts[v]
				dep := depth(p)
				b := w.beta[v] + cx.slack
				if math.Abs(dep) <= b {
					if !ce.rim.onRim(p, w.beta[v]) {
						return 0, nil, fmt.Errorf("%w: a wall touches the base plane off the chain", decaderr.ErrUnsupported)
					}
					g[v] = -cx.d
					continue
				}
				gv := dep - cx.d
				if gv > b+w.tau {
					g[v] = gv
					continue
				}
				seam := -1
				for _, k := range []int{ce.from, ce.to} {
					if distToLine(p, cx.pts[k].exact, a.n) <= b {
						seam = k
					}
				}
				if seam >= 0 && dep > 0 {
					j := cx.pts[seam].j
					if j == nil || !cx.onlyFaces(v, j.wall1, j.wall2) {
						return 0, nil, refuseCross("has a seam vertex within the setback that borders another face")
					}
					// A vertex inside the seam goes or stays by its held place along it, as the plane face beyond
					// reads it (runExt); the seam's cut is the junction's row vertex.
					if len(cx.facesOf(v)) == 2 && math.Abs(gv) > cx.margin || gv < -(b+w.tau) {
						g[v] = gv
						continue
					}
				}
				if err := clearance(gv, b+w.tau, "the wall"); err != nil {
					return 0, nil, err
				}
			}
		}
		// The seam is a boundary edge of the face, which one facet holds; an inner edge near the seam's line (the
		// diagonal of a sliver beside it) is not.
		uses := map[[2]int]int{}
		for _, t := range tris {
			for i := range 3 {
				uses[[2]int{min(t[i], t[(i+1)%3]), max(t[i], t[(i+1)%3])}]++
			}
		}
		for _, k := range []int{ce.from, ce.to} {
			pt := cx.pts[k]
			if pt.j == nil {
				return 0, nil, refuseCross("ends on an arc; only a straight end is built")
			}
			var key [2]int
			found := 0
			for _, t := range tris {
				for i := range 3 {
					x, y := t[i], t[(i+1)%3]
					if (g[x] < 0) == (g[y] < 0) {
						continue
					}
					if distToLine(w.verts[x], pt.exact, a.n) > w.beta[x]+cx.slack ||
						distToLine(w.verts[y], pt.exact, a.n) > w.beta[y]+cx.slack {
						continue
					}
					k2 := [2]int{min(x, y), max(x, y)}
					if uses[k2] != 1 {
						continue
					}
					if found == 0 || k2 != key {
						found++
						key = k2
					}
				}
			}
			if found != 1 {
				return 0, nil, fmt.Errorf("%w: a seam between two walls is not cut once", decaderr.ErrUnsupported)
			}
			w.cuts[key] = pt.row[1]
		}
		mine := map[int]bool{}
		cut := func(x, y int, facetBound float64) int {
			k := [2]int{min(x, y), max(x, y)}
			c, ok := w.cuts[k]
			if !ok {
				t := g[x] / (g[x] - g[y])
				p := w.verts[x].Add(w.verts[y].Sub(w.verts[x]).Scale(t))
				c = w.vertex(p, facetBound+cx.slack+w.tau)
				w.noteCut(c, facetBound)
				w.cuts[k] = c
			}
			mine[c] = true
			return c
		}
		tris = clipTris(w, tris, g, cut)
		cuts[ei] = mine
	}
	for _, t := range tris {
		w.emit(t, group)
	}
	w.removed = append(w.removed, facets...)
	return group, cuts, nil
}

// clipTris keeps the positive side of each triangle and cuts its crossing edges.
func clipTris(w *work, tris [][3]int, g map[int]float64, cut func(a, b int, facetBound float64) int) [][3]int {
	var out [][3]int
	for _, t := range tris {
		facetBound := w.facetBound(t)
		var above []int
		for k, v := range t {
			if g[v] > 0 {
				above = append(above, k)
			}
		}
		switch len(above) {
		case 3:
			out = append(out, t)
		case 1:
			x := above[0]
			a, b, c := t[x], t[(x+1)%3], t[(x+2)%3]
			out = append(out, [3]int{a, cut(a, b, facetBound), cut(c, a, facetBound)})
		case 2:
			z := 0
			for k := range 3 {
				if g[t[k]] <= 0 {
					z = k
				}
			}
			x, y, below := t[(z+1)%3], t[(z+2)%3], t[z]
			cyz, czx := cut(y, below, facetBound), cut(below, x, facetBound)
			out = append(out, [3]int{x, y, cyz}, [3]int{x, cyz, czx})
		}
	}
	return out
}
