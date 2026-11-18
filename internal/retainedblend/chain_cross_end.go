package retainedblend

import (
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file reads and rebuilds the free ends of an open cross-plane chain
// (agent-docs/retained-modify-reach-design.md §15b): a straight end edge between planes A and B meets an exact end
// plane G at a convex corner, and its bevel ends on G in the segment between the setback points slid along the edge
// onto G. Where such a point is exactly a corner of G, as where the chain ends on an earlier bevel of the same
// setback, the column takes that held corner.

// edgeBetween is the edge of face g at vertex v whose other face is x, or nil.
func edgeBetween(g, x *topo.Face, v *topo.Vertex) *topo.Edge {
	for _, l := range g.Loops {
		for _, ce := range l.CoEdges {
			e := ce.Edge
			if e == nil || len(e.Faces) != 2 || (e.Start != v && e.End != v) {
				continue
			}
			if (e.Faces[0] == g && e.Faces[1] == x) || (e.Faces[0] == x && e.Faces[1] == g) {
				return e
			}
		}
	}
	return nil
}

// readEnd reads free end k of the chain: the corner v where end edge ei meets its end plane G, other being the exact
// point at the edge's other end. It reads G's chains along A ∩ G and B ∩ G, which of their vertices the setback
// removes, and where each column point lands.
func (cx *crossChain) readEnd(k, ei int, v *topo.Vertex, other r3.Vec) error {
	ce := &cx.edges[ei]
	if ce.arc {
		return refuseCross("ends on an arc; only a straight end is built")
	}
	w := cx.w
	hv, gFace, err := cornerOf(v, cx.fp, cx.faceIndex, cx.faceIndex[ce.faces[0]], cx.faceIndex[ce.faces[1]])
	if err != nil {
		return err
	}
	// G is an exact plane, or one that states a charge: a displacement, or an oblique plane's tilt and extent
	// (retained-edge-blend §6.1, "An oblique end plane"), which widens the slack.
	g, ok := chainPlane(gFace)
	if !ok {
		return refuseCross("ends on a face that is not a retained plane")
	}
	tilt := 0.0
	if _, exactG := retainedPlane(gFace); !exactG {
		tilt = gFace.TagDeparture
	}
	if !(tilt <= maxEndTilt) {
		return refuseCross("ends on a face whose tag turns too far from its plane")
	}
	cx.widen(planeCharge(g))
	exact, ok := intersect3(ce.pl[0], ce.pl[1], g)
	if !ok {
		return refuseCross("meets its end face at a grazing angle")
	}
	if exact.Sub(cx.fp.Verts[hv]).Len() > cx.fp.VertexBound[hv]+cx.slack {
		return refuseCross("ends where its three tags do not meet")
	}
	t, ok := exact.Sub(other).Normalize()
	if !ok || !(g.n.Dot(t) > minSine) {
		return refuseCross("ends on a face that does not face along its end edge")
	}
	// G's two edges at the corner are convex, as the chain edge is: the solid near the corner lies inside all
	// three planes, and the end column lies in G's own face.
	for _, x := range ce.faces {
		if e := edgeBetween(gFace, x, v); e == nil || !e.Convex {
			return refuseCross("ends at a corner that is not convex")
		}
	}
	end := &crossEnd{gFace: gFace, g: g, faces: ce.faces, t: t, tilt: tilt}
	end.q[0] = setbackPoint(exact.Sub(ce.pl[1].n.Scale(cx.d)), t, g)
	end.q[1] = setbackPoint(exact.Sub(ce.pl[0].n.Scale(cx.d)), t, g)
	fl, err := w.faceLoops(cx.faceIndex[gFace], &g)
	if err != nil {
		return err
	}
	li, ok := fl.loopWith(hv)
	if !ok {
		return refuseCross("has an end face that does not hold its corner")
	}
	loop := fl.loops[li]
	n := len(loop)
	at := indexIn(loop, hv)
	var dirs [2]r3.Vec
	for x := range ce.faces {
		d, ok := ce.pl[x].n.Cross(g.n).Normalize()
		if !ok {
			return refuseCross("has an end face parallel to a face of its end edge")
		}
		if tau(end.q[x], exact, d) < 0 {
			d = d.Scale(-1)
		}
		dirs[x] = d
	}
	onLine := func(c, x int) bool { return distToLine(w.verts[c], exact, dirs[x]) <= w.beta[c]+cx.slack }
	lineOf := func(step int) int {
		c := loop[((at+step)%n+n)%n]
		switch {
		case onLine(c, 0) && !onLine(c, 1):
			return 0
		case onLine(c, 1) && !onLine(c, 0):
			return 1
		}
		return -1
	}
	fwd, bwd := lineOf(1), lineOf(-1)
	if fwd < 0 || bwd < 0 || fwd == bwd {
		return refuseCross("has an end face that does not leave its corner along both lines")
	}
	end.towardB = fwd == 1
	end.dirs = dirs
	for x := range 2 {
		step := -1
		if fwd == x {
			step = 1
		}
		reach := tau(end.q[x], exact, dirs[x])
		end.reach[x] = reach
		if !(reach > cx.margin) {
			return refuseCross("has a setback point behind its end corner")
		}
		end.vert[x] = -1
		for s := 1; s < n && end.vert[x] < 0; s++ {
			c := loop[((at+step*s)%n+n)%n]
			if !onLine(c, x) {
				return refuseCross("reaches past an edge of its end face")
			}
			tc := tau(w.verts[c], exact, dirs[x])
			if len(cx.facesOf(c)) == 2 && cx.onlyFaces(c, ce.faces[x], gFace) {
				// A vertex inside the straight edge X ∩ G is no corner of either face: it goes or stays by its held
				// place along the edge, which moves nothing of the solid.
				switch {
				case tc < -cx.margin:
					return refuseCross("has an end face whose edge leaves its corner off the end plane")
				case tc < reach-cx.margin:
					end.rem[x] = append(end.rem[x], c)
				case tc > reach+cx.margin:
					end.vert[x] = w.vertex(end.q[x], cx.bound)
				default:
					return fmt.Errorf("%w: a setback point falls on a vertex of the end face's boundary",
						decaderr.ErrUnsupported)
				}
				continue
			}
			// A corner of G stays when it lies beyond the setback point by more than its bound, and the column takes
			// it when it is exactly the setback point; any other corner the setback reaches refuses.
			if tc > reach+4*(w.beta[c]+cx.margin) {
				end.vert[x] = w.vertex(end.q[x], cx.bound)
				continue
			}
			same, err := cx.coincident(c, x, ce, end)
			if err != nil {
				return err
			}
			if !same {
				return fmt.Errorf("%w: a setback point falls on a vertex of the end face's boundary",
					decaderr.ErrUnsupported)
			}
			end.vert[x], end.held[x] = c, true
		}
		if end.vert[x] < 0 {
			return refuseCross("reaches past an edge of its end face")
		}
	}
	cx.pts[k] = crossPoint{at: v, exact: exact, hv: hv, end: end}
	return nil
}

// ratPlane is a plane's held normal n and its offset h = n · o, read as exact rationals.
type ratPlane struct {
	n ratVec
	h *big.Rat
}

func ratPlaneOf(p plane) ratPlane {
	n := ratVecOf(p.n)
	return ratPlane{n: n, h: n.dot(ratVecOf(p.o))}
}

// meetRat is the exact point common to three planes, false when their normals are dependent.
func meetRat(p, q, s ratPlane) (ratVec, bool) {
	qs, sp, pq := q.n.cross(s.n), s.n.cross(p.n), p.n.cross(q.n)
	det := p.n.dot(qs)
	if det.Sign() == 0 {
		return ratVec{}, false
	}
	x := qs.scale(p.h).add(sp.scale(q.h)).add(pq.scale(s.h))
	return x.scale(new(big.Rat).Inv(det)), true
}

// coincident decides whether held vertex c, met by the column of end edge ce at its face x, is exactly the column
// point: c must be a corner of exactly G, the face x and a third exact plane T, and the point where G, x and T meet
// must equal the setback point slid onto G, by the exact identity on the tags' floats read as rationals. A near miss
// proves nothing and is false. The falsifier holds c to that point.
func (cx *crossChain) coincident(c, x int, ce *crossEdge, end *crossEnd) (bool, error) {
	fs := cx.facesOf(c)
	if len(fs) != 3 || !fs[cx.faceIndex[end.gFace]] || !fs[cx.faceIndex[ce.faces[x]]] {
		return false, nil
	}
	var third *topo.Face
	for fi := range fs {
		if f := cx.faceAt[fi]; f != end.gFace && f != ce.faces[x] {
			third = f
		}
	}
	tp, ok := chainPlane(third)
	if !ok {
		return false, nil
	}
	corner, ok := columnAtCorner(ce.pl, end.g, tp, x, cx.d)
	if !ok {
		return false, nil
	}
	cx.widen(planeCharge(tp))
	if cx.w.verts[c].Sub(corner).Len() > cx.w.beta[c]+cx.slack {
		return false, refuseCross("meets an end face's corner that is not where its tags meet")
	}
	// The tags meet exactly; the true planes lie within their charges of them, so the true corner and the true
	// setback point lie within twice the amplified charges of each other, which the held corner's bound takes.
	if ch := sumUp(planeCharge(end.g), planeCharge(tp)); ch > 0 {
		cx.w.beta[c] = sumUp(cx.w.beta[c], mulUp(2, ch))
	}
	return true, nil
}

// columnAtCorner reports whether the column point on face x of the edge between planes ab, which ends on the plane
// g, is exactly the corner where g, ab[x] and t meet, by the exact identity on the planes' floats read as rationals:
// the corner of ab and g, less d along the other face's normal, slid along the edge onto g. It returns the corner,
// rounded once.
func columnAtCorner(ab [2]plane, g, t plane, x int, d float64) (r3.Vec, bool) {
	a, b, gr := ratPlaneOf(ab[0]), ratPlaneOf(ab[1]), ratPlaneOf(g)
	v, ok := meetRat(a, b, gr)
	if !ok {
		return r3.Vec{}, false
	}
	other := [2]ratPlane{b, a}[x]
	q := v.sub(other.n.scale(ratOf(d)))
	along := a.n.cross(b.n)
	den := gr.n.dot(along)
	if den.Sign() == 0 {
		return r3.Vec{}, false
	}
	q = q.add(along.scale(new(big.Rat).Quo(new(big.Rat).Sub(gr.h, gr.n.dot(q)), den)))
	corner, ok := meetRat(gr, [2]ratPlane{a, b}[x], ratPlaneOf(t))
	if !ok || !q.sub(corner).isZero() {
		return r3.Vec{}, false
	}
	return corner.float(), true
}

// endFaces lists the end planes of the chain's free ends, each once. An end plane must border no chain edge.
func (cx *crossChain) endFaces(touched map[int]bool) ([]*topo.Face, error) {
	var out []*topo.Face
	for _, pt := range cx.pts {
		if pt.end == nil {
			continue
		}
		if touched[cx.faceIndex[pt.end.gFace]] {
			return nil, refuseCross("ends on a face that one of its edges borders")
		}
		if !slices.Contains(out, pt.end.gFace) {
			out = append(out, pt.end.gFace)
		}
	}
	return out, nil
}

// rewriteEndFace gives the end plane f up the corner each end on it cuts off: the corner and the vertices of its
// chains inside the setbacks give way to the end column, and the face is triangulated again. It returns the face's
// group.
func (cx *crossChain) rewriteEndFace(f *topo.Face) (int, error) {
	w := cx.w
	g, _ := chainPlane(f)
	fl, err := w.faceLoops(cx.faceIndex[f], &g)
	if err != nil {
		return 0, err
	}
	gone := map[int]bool{}
	var polys []facePoly
	type cut struct {
		set  map[int]bool
		repl []int
	}
	var cuts []cut
	for _, pt := range cx.pts {
		end := pt.end
		if end == nil || end.gFace != f {
			continue
		}
		set := map[int]bool{pt.hv: true}
		for _, rem := range end.rem {
			for _, v := range rem {
				set[v] = true
			}
		}
		var repl []int
		poly := facePoly{pts: []r3.Vec{pt.exact, end.q[0], end.q[1]}, keepAt: map[int]int{}}
		for x := range 2 {
			poly.rays = append(poly.rays, polyRay{o: pt.exact, dir: end.dirs[x], from: end.reach[x]})
		}
		for x := range 2 {
			if end.held[x] {
				poly.keepAt[end.vert[x]] = x + 1
				continue
			}
			repl = append(repl, end.vert[x])
		}
		if !end.towardB {
			slices.Reverse(repl)
		}
		for v := range set {
			gone[v] = true
		}
		polys = append(polys, poly)
		cuts = append(cuts, cut{set: set, repl: repl})
	}
	if err := cx.gateChords(g.n, fl.loops, gone, polys); err != nil {
		return 0, err
	}
	loops := make([][]int, len(fl.loops))
	for k, lp := range fl.loops {
		loops[k] = slices.Clone(lp)
	}
	for _, c := range cuts {
		if err := spliceRun(loops, c.set, c.repl); err != nil {
			return 0, err
		}
	}
	if err := w.retriangulate(g.n, loops, fl.group); err != nil {
		return 0, err
	}
	w.removed = append(w.removed, fl.facets...)
	return fl.group, nil
}
