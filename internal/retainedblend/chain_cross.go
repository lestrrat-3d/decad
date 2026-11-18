package retainedblend

import (
	"context"
	"fmt"
	"maps"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file chamfers a tangent chain whose edges share no one base plane by rewriting the held mesh
// (agent-docs/retained-agent-docs/modify-design.md Part R §15b). Each junction's row is §5.5's, computed once from its own base;
// every plane face the chain borders gives way to the setback contour of its run of chain edges and is triangulated
// again, every cylinder face is cut at the setback plane of each arc it borders, and each straight end is rebuilt on
// its end plane as §5.5 step 5 does.

// crossEligible reports whether spec is a chamfer the cross-plane rewrite builds: one positive setback on both faces
// and no reference face.
func crossEligible(spec blendSpec) bool {
	return !spec.fillet && spec.asym == nil && spec.dA == spec.dB && spec.dA > 0
}

// crossPoint is one point of the chain in walk order: a junction, or for an open chain a free end.
type crossPoint struct {
	at    *topo.Vertex
	exact r3.Vec
	hv    int
	// j is the junction, nil at a free end; row holds its section's vertex on the base and its vertex on the seam.
	j   *junction
	nP  r3.Vec // the base's outward normal
	row [2]int
	end *crossEnd
}

// crossEnd is a free end of an open chain: the corner where its straight end edge meets the end plane G.
type crossEnd struct {
	gFace *topo.Face
	g     plane
	faces [2]*topo.Face // the end edge's two faces
	t     r3.Vec        // the edge's unit direction toward the corner
	// q are the exact column points on faces[k] ∩ G; vert their vertices, new, or the held corner of G that q
	// is (held); rem G's chain along faces[k] ∩ G that the setback removes, from the corner out.
	q    [2]r3.Vec
	vert [2]int
	held [2]bool
	rem  [2][]int
	// towardB is set when G's loop steps from the corner toward faces[1] ∩ G.
	towardB bool
	// dirs are the unit directions from the corner along faces[k] ∩ G toward G's chains, and reach the distance of
	// q[k] from the corner along them.
	dirs  [2]r3.Vec
	reach [2]float64
	// tilt is an oblique G's stated tag departure, zero for an exact plane.
	tilt float64
	// past holds, for a held corner the column takes on faces[k], the lines from it that the boundary of faces[k] and
	// of G runs on along, the third face's edges with them (coincident).
	past [2]struct{ onX, onG polyRay }
}

// crossEdge is one chain edge in walk order.
type crossEdge struct {
	edge  *topo.Edge
	ord   int
	faces [2]*topo.Face // a line's two planes; an arc's plane, then its cylinder
	pl    [2]plane      // the planes; pl[1] is unset for an arc
	arc   bool
	rim   rim
	from  int // the chain points at the edge's start and end in walk order
	to    int
	p0    r3.Vec
	p1    r3.Vec
	d     r3.Vec // line: unit direction from p0 to p1
	len   float64
	sigma float64 // arc: +1 when the walk turns counter-clockwise about the plane's normal
	phi0  float64
	span  float64
	cols  []crossCol
}

// crossCol is one cross-section of an edge's bevel: the point at on the edge, its parameter s, and the bevel's
// vertices on the edge's two faces. An arc's interior column has none on the cylinder (-1).
type crossCol struct {
	s  float64
	at r3.Vec
	on [2]int
}

// crossChain is one chain's rewrite in progress.
type crossChain struct {
	w         *work
	fp        facetedrecord.Payload
	d         float64
	slack     float64
	margin    float64
	bound     float64
	faceIndex map[*topo.Face]int
	faceAt    map[int]*topo.Face
	faceVerts map[int]map[int]bool
	pts       []crossPoint
	edges     []crossEdge
	open      bool
	// scale is the coordinate scale and charge the plane charge the faces read so far state (widen).
	scale  float64
	charge float64
}

// widen adds a face's plane charge c to the chain's and sets the slack, the margin and the bound of a vertex the
// rewrite makes from it.
func (cx *crossChain) widen(c float64) {
	cx.charge = sumUp(cx.charge, c)
	cx.slack = chargedSlack(cx.scale, cx.charge)
	cx.margin = cx.w.tau + cx.slack
	cx.bound = cx.margin
}

func refuseCross(what string) error {
	return fmt.Errorf("%w: a tangent chain across planes %s", decaderr.ErrUnsupported, what)
}

// rewriteCrossChain chamfers the chain cyc, whose edges share no base plane, on a copy of the held mesh.
func rewriteCrossChain(ctx context.Context, env topo.Env, fp facetedrecord.Payload, cyc chainCycle, ords []int,
	spec blendSpec, op string, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	if chainCharged(cyc.edges) {
		return nil, refuseCross("is not built on faces that state a displacement")
	}
	scale := 1.0
	for _, v := range fp.Verts {
		scale = max(scale, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	// The chain's own faces are exact; an end plane and the third face at a held corner the column meets may state a
	// charge, which widens the slack as each is read (widen). The bevels' surfaces read only the chain's faces, so
	// their tags are published exactly (tagGroup reads w.r.charge, which stays zero).
	w := newWork(fp, rim{scale: scale}, spec)
	w.faceIx = faceIndex
	cx := &crossChain{w: w, fp: fp, d: spec.dA, scale: scale, faceIndex: faceIndex, faceAt: map[int]*topo.Face{},
		faceVerts: w.faceOfVertex(), open: cyc.open}
	cx.widen(0)
	for f, i := range faceIndex {
		cx.faceAt[i] = f
	}
	if err := cx.readChain(cyc, ords); err != nil {
		return nil, err
	}
	if err := cx.buildColumns(spec); err != nil {
		return nil, err
	}

	touched := map[int]bool{}
	var planes []*topo.Face
	var cyls []*topo.Face
	for _, ce := range cx.edges {
		for k, f := range ce.faces {
			fi := cx.faceIndex[f]
			if touched[fi] {
				continue
			}
			touched[fi] = true
			if ce.arc && k == 1 {
				cyls = append(cyls, f)
				continue
			}
			planes = append(planes, f)
		}
	}
	slices.SortFunc(planes, func(a, b *topo.Face) int { return cx.faceIndex[a] - cx.faceIndex[b] })
	slices.SortFunc(cyls, func(a, b *topo.Face) int { return cx.faceIndex[a] - cx.faceIndex[b] })
	var groups []int
	for _, f := range planes {
		g, err := cx.rewritePlaneFace(f)
		if err != nil {
			return nil, err
		}
		groups = append(groups, g)
	}
	ends, err := cx.endFaces(touched)
	if err != nil {
		return nil, err
	}
	for _, f := range ends {
		g, err := cx.rewriteEndFace(f)
		if err != nil {
			return nil, err
		}
		groups = append(groups, g)
	}
	cutsOf := map[int]map[int]bool{}
	for _, f := range cyls {
		g, cuts, err := cx.clipCylinderFace(f)
		if err != nil {
			return nil, err
		}
		groups = append(groups, g)
		maps.Copy(cutsOf, cuts)
	}

	stripFrom := len(w.out)
	all := append([]facetedrecord.Group(nil), fp.Groups...)
	removed, blendArea := 0.0, 0.0
	for ei := range cx.edges {
		g, err := cx.emitBevel(env, ei, len(all), cutsOf[ei], spec, op)
		if err != nil {
			return nil, err
		}
		all = append(all, g)
		r, b := cx.edgeAreas(ei)
		removed += r
		blendArea += b
	}
	for _, g := range groups {
		all[g].HasCentroid = false
		all[g].Centroid = measurement.VecMeasurement{}
	}
	w.removedTrue = removed
	if err := w.checkStripClear(ctx, stripFrom, nil); err != nil {
		return nil, err
	}
	return w.build(ctx, env, all, stripFrom, blendTag{trueArea: blendArea}, faceIndex)
}

// readChain reads the chain's edges, junctions and free ends.
func (cx *crossChain) readChain(cyc chainCycle, ords []int) error {
	n := len(cyc.edges)
	np := n
	if cx.open {
		np = n + 1
	}
	cx.pts = make([]crossPoint, np)
	cx.edges = make([]crossEdge, n)
	for i := range n {
		ce, err := cx.readEdge(cyc.edges[i])
		if err != nil {
			return err
		}
		ce.ord = ords[i]
		if cx.open {
			ce.from, ce.to = i, i+1
		} else {
			ce.from, ce.to = (i-1+n)%n, i
		}
		cx.edges[i] = ce
	}
	for i, l := range cyc.joins {
		k := i
		if cx.open {
			k = i + 1
		}
		if err := cx.readJunction(k, l); err != nil {
			return err
		}
	}
	if cx.open {
		first, last := cyc.freeEnds()
		for _, c := range []struct {
			k, ei int
			v     *topo.Vertex
		}{{0, 0, first}, {n, n - 1, last}} {
			other := cx.pts[1].exact
			if c.k == n {
				other = cx.pts[n-1].exact
			}
			if err := cx.readEnd(c.k, c.ei, c.v, other); err != nil {
				return err
			}
		}
	}
	for i := range cx.edges {
		ce := &cx.edges[i]
		ce.p0, ce.p1 = cx.pts[ce.from].exact, cx.pts[ce.to].exact
		if !ce.arc {
			d, ok := ce.p1.Sub(ce.p0).Normalize()
			if !ok {
				return fmt.Errorf("%w: a chain edge has no length", decaderr.ErrDegenerate)
			}
			ce.d, ce.len = d, ce.p1.Sub(ce.p0).Len()
			continue
		}
		if err := ce.readArc(cx.pts[ce.from].at); err != nil {
			return err
		}
	}
	return nil
}

// readEdge reads one chain edge's faces: a convex line between two exact planes, or a convex arc between an exact
// plane and a cylinder about its normal.
func (cx *crossChain) readEdge(e *topo.Edge) (crossEdge, error) {
	ce := crossEdge{edge: e}
	if !e.Convex {
		return ce, refuseCross("is chamfered only where it is convex")
	}
	switch e.Curve.(type) {
	case surfacegeom.Line3:
		for k, f := range e.Faces {
			p, ok := retainedPlane(f)
			if !ok || p.delta != 0 {
				return ce, refuseCross("has a line whose face is not an exact plane")
			}
			ce.faces[k], ce.pl[k] = f, p
		}
	case surfacegeom.Arc3:
		r, err := readChainRim(e, cx.fp, cx.faceIndex)
		if err != nil {
			return ce, err
		}
		p, ok := retainedPlane(r.planeFace)
		if !r.convex || r.side >= 0 || r.charge != 0 || !ok || p.delta != 0 {
			return ce, refuseCross("has an arc that is not a convex rim of exact tags")
		}
		ce.arc, ce.rim = true, r
		ce.faces = [2]*topo.Face{r.planeFace, r.wallFace}
		ce.pl[0] = p
	default:
		return ce, refuseCross("has an edge that is neither a line nor an arc")
	}
	return ce, nil
}

// readArc reads an arc edge's sense and span along the walk, which starts at the vertex start.
func (ce *crossEdge) readArc(start *topo.Vertex) error {
	curve, _ := ce.edge.Curve.(surfacegeom.Arc3)
	sense := 1.0
	if curve.Axis.Dot(ce.rim.n) < 0 {
		sense = -1
	}
	if ce.edge.Start != start {
		sense = -sense
	}
	for _, p := range []r3.Vec{ce.p0, ce.p1} {
		if !ce.rim.onRim(p, 0) {
			return fmt.Errorf("%w: a chain junction is off its arc", decaderr.ErrUnsupported)
		}
	}
	ce.sigma, ce.phi0 = sense, ce.rim.angle(ce.p0)
	span := sense * (ce.rim.angle(ce.p1) - ce.phi0)
	span -= 2 * math.Pi * math.Floor(span/(2*math.Pi))
	if !(span > minSine) {
		return fmt.Errorf("%w: a chain arc has no span", decaderr.ErrDegenerate)
	}
	if span > math.Pi+1e-9 {
		return fmt.Errorf("%w: a chain arc spans more than a half turn", decaderr.ErrUnsupported)
	}
	ce.span = span
	return nil
}

// param is the position of p along the edge: the distance from its start for a line, the swept angle for an arc.
func (ce *crossEdge) param(p r3.Vec) float64 {
	if !ce.arc {
		return p.Sub(ce.p0).Dot(ce.d)
	}
	d := ce.sigma * (ce.rim.angle(p) - ce.phi0)
	return d - 2*math.Pi*math.Floor((d+math.Pi)/(2*math.Pi))
}

// faceNormalAt is the outward unit normal of an exact plane or cylinder face at p.
func faceNormalAt(f *topo.Face, p r3.Vec) (r3.Vec, bool) {
	if pl, ok := retainedPlane(f); ok {
		return pl.n, true
	}
	cyl, ok := f.Surface.(surfacegeom.Cylinder)
	if !ok {
		return r3.Vec{}, false
	}
	axis, ok := cyl.Axis.Normalize()
	if !ok {
		return r3.Vec{}, false
	}
	q := p.Sub(cyl.Origin)
	n, ok := q.Sub(axis.Scale(q.Dot(axis))).Normalize()
	if !ok {
		return r3.Vec{}, false
	}
	if f.Reversed {
		n = n.Scale(-1)
	}
	return n, true
}

// facesOf is the set of faces whose facets use held vertex v.
func (cx *crossChain) facesOf(v int) map[int]bool { return cx.faceVerts[v] }

// onlyFaces reports whether every face that uses v is one of fs.
func (cx *crossChain) onlyFaces(v int, fs ...*topo.Face) bool {
	for fi := range cx.facesOf(v) {
		if !slices.ContainsFunc(fs, func(f *topo.Face) bool { return f != nil && cx.faceIndex[f] == fi }) {
			return false
		}
	}
	return true
}

// readJunction reads junction l as chain point k: its exact point, its held vertex and its row (§5.5 step 1), the
// setback point on its base and the one on its seam, which both edges that meet there share.
func (cx *crossChain) readJunction(k int, l chainLink) error {
	j := l.j
	hv := heldIndex(cx.fp, l.at)
	if hv < 0 {
		return fmt.Errorf("%w: a chain junction is not a vertex of the held mesh", decaderr.ErrUnsupported)
	}
	if d := cx.fp.Verts[hv].Sub(j.point).Len(); d > cx.fp.VertexBound[hv]+cx.slack {
		return fmt.Errorf("%w: a chain junction is not where its tags meet", decaderr.ErrUnsupported)
	}
	if len(cx.facesOf(hv)) != 3 || !cx.onlyFaces(hv, j.base, j.wall1, j.wall2) {
		return refuseCross("has a junction that is not a corner of its base and its two walls")
	}
	bp, ok := retainedPlane(j.base)
	if !ok || bp.delta != 0 {
		return refuseCross("has a junction whose base is not an exact plane")
	}
	nW, ok := faceNormalAt(j.wall1, j.point)
	if !ok {
		return refuseCross("has a junction whose wall has no normal")
	}
	w := cx.w
	jj := j
	cx.pts[k] = crossPoint{at: l.at, exact: j.point, hv: hv, j: &jj, nP: bp.n, row: [2]int{
		w.vertex(j.point.Sub(nW.Scale(cx.d)), cx.bound),
		w.vertex(j.point.Sub(bp.n.Scale(cx.d)), cx.bound),
	}}
	return nil
}

// onFace is the vertex of chain point k's section on face f, one of the faces of an edge that ends there.
func (cx *crossChain) onFace(k int, f *topo.Face) (int, error) {
	pt := cx.pts[k]
	if pt.end != nil {
		for i, g := range pt.end.faces {
			if g == f {
				return pt.end.vert[i], nil
			}
		}
	} else {
		switch f {
		case pt.j.base:
			return pt.row[0], nil
		case pt.j.wall1, pt.j.wall2:
			return pt.row[1], nil
		}
	}
	return -1, refuseCross("has a point whose section misses a face of its edge")
}

// buildColumns makes each edge's columns: the rows of the chain points at its ends, and for an arc the sections
// between them, spaced as §5.5 step 2 spaces a chain arc's.
func (cx *crossChain) buildColumns(spec blendSpec) error {
	w, d := cx.w, cx.d
	for i := range cx.edges {
		ce := &cx.edges[i]
		end := func(k int) [2]int {
			var on [2]int
			for f := range 2 {
				on[f], _ = cx.onFace(k, ce.faces[f])
			}
			return on
		}
		first := crossCol{s: 0, at: ce.p0, on: end(ce.from)}
		if !ce.arc {
			ce.cols = []crossCol{first, {s: ce.len, at: ce.p1, on: end(ce.to)}}
			continue
		}
		r := ce.rim
		rhoMax := max(r.radius, r.radius+r.kappa*d)
		step := 2 * math.Acos(1-math.Min(azimuthShare(spec, w.tau)/rhoMax, 1))
		na, err := columnCount(ce.span, step, rhoMax, spec.size(), 2)
		if err != nil {
			return err
		}
		ce.cols = []crossCol{first}
		for c := 1; c < na; c++ {
			f := float64(c) / float64(na)
			phi := ce.phi0 + ce.sigma*ce.span*f
			v := r.at(r.radius, phi, 0)
			radial, _ := v.Sub(r.center).Normalize()
			// The bevel's side on the cylinder is the cylinder's own cut at the setback plane (emitBevel), as a chain
			// chamfer's is on one base: an interior column has a vertex on the plane only.
			ce.cols = append(ce.cols, crossCol{s: ce.span * f, at: v, on: [2]int{
				w.vertex(v.Add(radial.Scale(r.kappa*d)), cx.bound), -1,
			}})
		}
		ce.cols = append(ce.cols, crossCol{s: ce.span, at: ce.p1, on: end(ce.to)})
	}
	return nil
}
