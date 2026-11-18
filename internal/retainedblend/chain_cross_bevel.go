package retainedblend

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file emits the bevels of a cross-plane chain (agent-docs/modify-design.md Part M §15b) and their exact
// areas: a line's bevel is the plane through its two end rows, an arc's the cone between its columns on the plane and
// the cylinder's cut, as §5.5 step 2 builds a chain chamfer's.

// emitBevel emits edge ei's bevel as source group gi and returns that group. cuts are the vertices the
// cylinder's cut made or used along an arc.
func (cx *crossChain) emitBevel(env topo.Env, ei, gi int, cuts map[int]bool, spec blendSpec,
	op string) (facetedrecord.Group, error) {
	w := cx.w
	ce := &cx.edges[ei]
	outward := func(p r3.Vec) r3.Vec {
		n1 := ce.pl[1].n
		if ce.arc {
			n1, _ = faceNormalAt(ce.faces[1], p)
		}
		return ce.pl[0].n.Add(n1)
	}
	emit := func(t [3]int) {
		a, b, c := w.verts[t[0]], w.verts[t[1]], w.verts[t[2]]
		if b.Sub(a).Cross(c.Sub(a)).Dot(outward(a.Add(b).Add(c).Scale(1.0/3))) < 0 {
			t[1], t[2] = t[2], t[1]
		}
		w.emit(t, gi)
	}
	if ce.arc {
		if err := cx.emitArcBevel(ce, cuts, emit); err != nil {
			return facetedrecord.Group{}, err
		}
	} else {
		a, b := ce.cols[0].on, ce.cols[1].on
		emit([3]int{a[0], b[0], b[1]})
		emit([3]int{a[0], b[1], a[1]})
	}
	var tag blendTag
	if ce.arc {
		tmp := &work{r: ce.rim, spec: spec, tau: w.tau}
		tag = tmp.coneTag()
	} else {
		// The plane through a row at a junction: a free end's column may meet a held corner.
		col := ce.cols[0]
		if cx.pts[ce.from].end != nil {
			col = ce.cols[len(ce.cols)-1]
		}
		a0, b0 := w.verts[col.on[0]], w.verts[col.on[1]]
		out := outward(a0)
		tag = blendTag{outward: func(r3.Vec) r3.Vec { return out }}
		if chord, ok := b0.Sub(a0).Normalize(); ok {
			if frame, err := r3.NewFrame(a0, ce.d, chord); err == nil {
				tag.surface = surfacegeom.Plane{Frame: frame}
				tag.reversed = frame.N().Dot(out) < 0
			}
		}
	}
	g := facetedrecord.Group{
		Origins: []provenance.Ref{{Producer: env.Ref, Role: fmt.Sprintf("%s(%d)", op, ce.ord)}},
		Source:  facetedrecord.BlendSource(env.Ref, ce.ord),
		Planar:  !ce.arc,
	}
	if tag.surface == nil {
		return g, nil
	}
	return w.tagGroup(g, tag), nil
}

// emitArcBevel joins an arc's columns on the plane, on the exact setback circle, to the cylinder's cut along the
// setback plane, which runs from the row vertex on the seam at one junction to the one at the other, as a chain
// chamfer's strip on one base does (§5.5 step 2). The cut vertices take the band's bound (chargeBand).
func (cx *crossChain) emitArcBevel(ce *crossEdge, cuts map[int]bool, emit func([3]int)) error {
	w := cx.w
	first, last := ce.cols[0].on[1], ce.cols[len(ce.cols)-1].on[1]
	if !cuts[first] || !cuts[last] {
		return fmt.Errorf("%w: a wall's cut does not reach the chain's junction", decaderr.ErrUnsupported)
	}
	var mid []int
	for c := range cuts {
		if c != first && c != last {
			mid = append(mid, c)
		}
	}
	slices.SortFunc(mid, func(a, b int) int {
		pa, pb := ce.param(w.verts[a]), ce.param(w.verts[b])
		switch {
		case pa < pb:
			return -1
		case pa > pb:
			return 1
		}
		return a - b
	})
	ring := append(append([]int{first}, mid...), last)
	bp := make([]float64, len(ring))
	for k, c := range ring {
		bp[k] = ce.param(w.verts[c])
	}
	bp[0], bp[len(bp)-1] = 0, ce.span
	from := len(w.out)
	cols := ce.cols
	for i, bi := 0, 0; i+1 < len(cols) || bi+1 < len(ring); {
		if bi+1 >= len(ring) || (i+1 < len(cols) && cols[i+1].s <= bp[bi+1]) {
			emit([3]int{cols[i].on[0], cols[i+1].on[0], ring[bi]})
			i++
			continue
		}
		emit([3]int{cols[i].on[0], ring[bi+1], ring[bi]})
		bi++
	}
	r := ce.rim
	w.chargeBand(from, bandAxis{o: r.center, n: r.n, ok: true}, max(r.radius, r.radius+r.kappa*cx.d), 0, cx.slack)
	return nil
}

// edgeAreas are the exact area edge ei's chamfer removes from the faces it touches, its end planes' corners
// included, and the exact area of its bevel (§5.7).
func (cx *crossChain) edgeAreas(ei int) (removed, blend float64) {
	ce := &cx.edges[ei]
	d := cx.d
	from, to := cx.pts[ce.from], cx.pts[ce.to]
	if from.end == nil && to.end == nil {
		if !ce.arc {
			return 2 * ce.len * d, ce.len * d * math.Sqrt2
		}
		r, k := ce.rim.radius, ce.rim.kappa
		rho := r + k*d
		return ce.span*math.Abs(r*r-rho*rho)/2 + ce.span*r*d, ce.span * (r + rho) / 2 * d * math.Sqrt2
	}
	// A straight end edge: each face loses the quadrilateral between the edge, the end plane's line and the setback
	// line, the end plane loses the triangle the column cuts off, and the bevel is the quadrilateral of the rows.
	at := func(k, f int) r3.Vec {
		p, _ := cx.sectionAt(k, ce.faces[f])
		return p
	}
	for f := range 2 {
		removed += polyArea(ce.p0, ce.p1, at(ce.to, f), at(ce.from, f))
	}
	blend = polyArea(at(ce.from, 0), at(ce.to, 0), at(ce.to, 1), at(ce.from, 1))
	for _, pt := range []crossPoint{from, to} {
		if pt.end == nil {
			continue
		}
		removed += polyArea(pt.exact, pt.end.q[0], pt.end.q[1])
		// An end plane that states a charge moves the pieces that end on it, and an oblique one's true cap may hold
		// more area than its projection onto the tag (planeAreaCharge, obliqueCapArea).
		c := planeCharge(pt.end.g)
		corner := planePiece(pt.exact, pt.end.q[0], pt.end.q[1])
		pieces := []chargePiece{corner, planePiece(at(ce.from, 0), at(ce.to, 0), at(ce.to, 1), at(ce.from, 1))}
		for f := range 2 {
			pieces = append(pieces, planePiece(ce.p0, ce.p1, at(ce.to, f), at(ce.from, f)))
		}
		if c > 0 || pt.end.tilt > 0 {
			cx.w.tiltArea = sumUp(cx.w.tiltArea, planeAreaCharge(pieces, c), obliqueCapArea(corner, c, pt.end.tilt))
		}
	}
	return removed, blend
}
