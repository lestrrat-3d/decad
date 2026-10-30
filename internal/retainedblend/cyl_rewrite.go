package retainedblend

import (
	"context"
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// section is the cross-section of a plane-cylinder blend, in the plane normal
// to the axis through the first end: the exact points and the numbers the rest
// of the construction reads.
type section struct {
	ta, tb  r3.Vec   // tangent points on the plane and on the cylinder
	centre  r3.Vec   // the fillet ball's centre
	rows    []r3.Vec // the blend's rows, from ta to tb
	phi     float64  // the fillet arc's angle
	thetaT  float64  // the angle of tb about the axis, from the corner into the wall
	setback float64  // the distance of ta from the corner along the plane
}

// solveSection finds the tangent points of the blend of the corner k0 between
// the plane (outward normal nA, running into its face along wA) and the wall
// (a circle of the cylinder's radius about o0, running into its face along
// wB). tau is the chord tolerance of the fillet arc.
func (c cylLine) solveSection(spec blendSpec, k0, o0, wA, wB r3.Vec, convex bool, tau, margin float64) (section, error) {
	var s section
	u1 := k0.Sub(o0).Scale(1 / c.radius)
	if !spec.fillet {
		d := spec.dA
		s.ta = k0.Add(wA.Scale(d))
		s.thetaT = spec.dB / c.radius
		s.tb = o0.Add(u1.Scale(math.Cos(s.thetaT) * c.radius)).Add(wB.Scale(math.Sin(s.thetaT) * c.radius))
		s.rows = []r3.Vec{s.ta, s.tb}
		s.setback = d
		if !(s.thetaT > minSine) || !(s.thetaT < math.Pi-minSine) {
			return s, fmt.Errorf("%w: the chamfer's setback does not fit on the wall", decaderr.ErrDegenerate)
		}
		return s, nil
	}
	r := spec.r
	ball := 1.0
	if convex {
		ball = -1
	}
	kappa := -1.0
	if convex == c.matOutside {
		kappa = 1
	}
	rr := c.radius + kappa*r
	if !(rr > margin) {
		return s, fmt.Errorf("%w: the blend reaches the cylinder's axis: radius %g mm against %g mm", decaderr.ErrDegenerate, r, c.radius)
	}
	ox, oy := o0.Sub(k0).Dot(wA), o0.Sub(k0).Dot(c.a.n)
	cy := ball * r
	disc := rr*rr - (cy-oy)*(cy-oy)
	if !(disc > 0) {
		return s, fmt.Errorf("%w: no ball of that radius touches both faces", decaderr.ErrDegenerate)
	}
	cx := math.Inf(1)
	for _, root := range []float64{ox - math.Sqrt(disc), ox + math.Sqrt(disc)} {
		if root > margin && root < cx {
			cx = root
		}
	}
	if math.IsInf(cx, 1) {
		return s, fmt.Errorf("%w: no ball of that radius touches both faces", decaderr.ErrDegenerate)
	}
	s.ta = k0.Add(wA.Scale(cx))
	s.centre = s.ta.Add(c.a.n.Scale(cy))
	cd := s.centre.Sub(o0)
	s.tb = o0.Add(cd.Scale(c.radius / cd.Len()))
	s.setback = cx
	q := s.tb.Sub(o0)
	s.thetaT = math.Atan2(q.Dot(wB), q.Dot(u1))
	if !(s.thetaT > minSine) || !(s.thetaT < math.Pi-minSine) {
		return s, fmt.Errorf("%w: the blend's tangent point does not lie on the wall's face", decaderr.ErrUnsupported)
	}
	a0 := c.a.n.Scale(-ball)
	a1 := s.tb.Sub(s.centre).Scale(1 / r)
	s.phi = math.Acos(math.Max(-1, math.Min(1, a0.Dot(a1))))
	if !(s.phi > minSine) || !(s.phi < math.Pi-minSine) {
		return s, fmt.Errorf("%w: the blend's arc is degenerate", decaderr.ErrDegenerate)
	}
	step := 2 * math.Acos(1-math.Min(tau/r, 1))
	m := max(2, int(math.Ceil(s.phi/step)))
	sin := math.Sin(s.phi)
	for j := 0; j <= m; j++ {
		alpha := s.phi * float64(j) / float64(m)
		s.rows = append(s.rows, s.centre.Add(a0.Scale(r*math.Sin(s.phi-alpha)/sin)).Add(a1.Scale(r*math.Sin(alpha)/sin)))
	}
	s.rows[0], s.rows[m] = s.ta, s.tb
	return s, nil
}

// cornerArea is the area of the region of the cross-section the blend takes
// from the corner: the plane's line from k to ta, the blend (the fillet arc,
// or the chamfer's chord) from ta to tb, and the wall's circle back to k.
func (c cylLine) cornerArea(spec blendSpec, s section, k0, o0, wA r3.Vec) float64 {
	to2 := func(p r3.Vec) (float64, float64) {
		d := p.Sub(k0)
		return d.Dot(wA), d.Dot(c.a.n)
	}
	segment := func(p, q r3.Vec) float64 {
		x0, y0 := to2(p)
		x1, y1 := to2(q)
		return (x0*y1 - x1*y0) / 2
	}
	arc := func(centre r3.Vec, rho float64, p, q r3.Vec) float64 {
		ax, ay := to2(centre)
		px, py := to2(p)
		qx, qy := to2(q)
		a0 := math.Atan2(py-ay, px-ax)
		sweep := math.Atan2(qy-ay, qx-ax) - a0
		for sweep > math.Pi {
			sweep -= 2 * math.Pi
		}
		for sweep <= -math.Pi {
			sweep += 2 * math.Pi
		}
		a1 := a0 + sweep
		return (rho*rho*sweep + ax*rho*(math.Sin(a1)-math.Sin(a0)) - ay*rho*(math.Cos(a1)-math.Cos(a0))) / 2
	}
	total := segment(k0, s.ta)
	if spec.fillet {
		total += arc(s.centre, spec.r, s.ta, s.tb)
	} else {
		total += segment(s.ta, s.tb)
	}
	total += arc(o0, c.radius, s.tb, k0)
	return math.Abs(total)
}

// rewriteCylLine blends one plane-cylinder line and builds the body over the
// rewritten mesh.
func rewriteCylLine(ctx context.Context, env topo.Env, fp facetedrecord.Payload, c cylLine, spec blendSpec,
	role string, ord int, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	w := newWork(fp, rim{scale: c.scale, charge: c.charge}, spec)
	slack := c.slack()
	margin := w.tau + slack
	k0, k1 := c.ends[0].exact, c.ends[1].exact
	axis := c.axis
	nA := c.a.n

	var faces [4]faceLoops
	for k, f := range []*topo.Face{c.faceA, c.faceB, c.ends[0].face, c.ends[1].face} {
		var err error
		var end *plane
		if k >= 2 {
			end = &c.ends[k-2].g
		}
		if faces[k], err = w.faceLoops(faceIndex[f], end); err != nil {
			return nil, err
		}
	}
	fA, fB, fG := faces[0], faces[1], [2]faceLoops{faces[2], faces[3]}

	// Locate the edge in the plane's loops and in the wall's.
	tL := nA.Cross(axis)
	onEdge := func(v int) bool { return distToLine(w.verts[v], k0, axis) <= w.beta[v]+slack }
	var onLineA, onArcB [2]func(int) bool
	for i := range 2 {
		end := c.ends[i]
		oi := c.axisPoint(end.exact)
		onLineA[i] = func(v int) bool { return distToLine(w.verts[v], end.exact, tL) <= w.beta[v]+slack }
		onArcB[i] = func(v int) bool {
			q := w.verts[v].Sub(oi)
			axial := q.Dot(axis)
			radial := q.Sub(axis.Scale(axial)).Len()
			return math.Hypot(radial-c.radius, axial) <= w.beta[v]+slack
		}
	}
	runA, err := w.locateBy(fA, c.ends[0].v, c.ends[1].v, onEdge, onLineA)
	if err != nil {
		return nil, err
	}
	runB, err := w.locateBy(fB, c.ends[0].v, c.ends[1].v, onEdge, onArcB)
	if err != nil {
		return nil, err
	}
	for i := range 2 {
		dir := tL
		if w.verts[runA.chain[i][len(runA.chain[i])-1]].Sub(c.ends[i].exact).Dot(dir) < 0 {
			dir = dir.Scale(-1)
		}
		runA.dir[i] = dir
	}

	// The ways into the faces, and the corner's kind.
	along := func(run edgeRun) r3.Vec {
		if run.step > 0 {
			return axis
		}
		return axis.Scale(-1)
	}
	o0 := c.axisPoint(k0)
	radialK := k0.Sub(o0).Scale(1 / c.radius)
	nB := radialK
	if c.matOutside {
		nB = radialK.Scale(-1)
	}
	wA, wB := nA.Cross(along(runA)), nB.Cross(along(runB))
	convex := nB.Dot(wA) < 0
	if (nA.Dot(wB) < 0) != convex || c.edgeConvex != convex {
		return nil, fmt.Errorf("%w: the edge's convexity disagrees with its two tags", decaderr.ErrUnsupported)
	}
	var kindA [2]sideKind
	for i := range 2 {
		switch s := runA.dir[i].Dot(wA); {
		case s > minSine:
			kindA[i] = trims
		case s < -minSine:
			kindA[i] = extends
		default:
			return nil, fmt.Errorf("%w: a face leaves an edge end along the edge's own line", decaderr.ErrUnsupported)
		}
		if kindA[i] == extends && convex {
			return nil, fmt.Errorf("%w: a convex edge ends where its plane is reflex", decaderr.ErrUnsupported)
		}
		toward := w.verts[runB.chain[i][0]].Sub(w.verts[c.ends[i].v])
		if toward.Dot(wB) <= minSine*toward.Len() {
			return nil, fmt.Errorf("%w: the wall turns reflex at an edge end", decaderr.ErrUnsupported)
		}
	}

	// The blend's section.
	sec, err := c.solveSection(spec, k0, o0, wA, wB, convex, w.tau, margin)
	if err != nil {
		return nil, err
	}
	length := k1.Sub(k0).Dot(axis)
	if !(length > 4*margin) {
		return nil, fmt.Errorf("%w: the edge is too short for the blend", decaderr.ErrDegenerate)
	}

	// The wall: cut at the generator through tb. The vertices of the wall's
	// facets sit on one side of it or the other by their angle about the axis.
	u1, u2 := radialK, wB
	thetaOf := func(p r3.Vec) float64 {
		q := p.Sub(o0)
		q = q.Sub(axis.Scale(q.Dot(axis)))
		return math.Atan2(q.Dot(u2), q.Dot(u1))
	}
	removedB := map[int]bool{}
	for _, f := range fB.facets {
		for _, v := range fp.Tris[f] {
			if _, seen := removedB[v]; seen {
				continue
			}
			th := thetaOf(w.verts[v])
			if th < 0 {
				th += 2 * math.Pi
			}
			tol := (w.beta[v] + slack + w.tau) / c.radius
			if th > 2*math.Pi-tol {
				th -= 2 * math.Pi
			}
			switch {
			case th < sec.thetaT-tol:
				removedB[v] = true
			case th > sec.thetaT+tol:
				removedB[v] = false
			default:
				return nil, fmt.Errorf("%w: a vertex of the wall lies on the blend's setback line", decaderr.ErrUnsupported)
			}
		}
	}
	nu := u1.Scale(-math.Sin(sec.thetaT)).Add(u2.Scale(math.Cos(sec.thetaT)))
	gT := func(v int) float64 { return nu.Dot(w.verts[v].Sub(o0)) }
	cut := func(a, b int) (int, error) { // a removed, b kept
		key := [2]int{min(a, b), max(a, b)}
		if id, ok := w.cuts[key]; ok {
			return id, nil
		}
		ga, gb := gT(a), gT(b)
		if !(ga < 0 && gb > 0) {
			return 0, fmt.Errorf("%w: a wall facet reaches across half the cylinder", decaderr.ErrUnsupported)
		}
		p := w.verts[a].Add(w.verts[b].Sub(w.verts[a]).Scale(ga / (ga - gb)))
		id := w.vertex(p, max(w.beta[a], w.beta[b])+slack+w.tau)
		w.cuts[key] = id
		return id, nil
	}
	for _, f := range fB.facets {
		t := fp.Tris[f]
		var gone []int
		for k, v := range t {
			if removedB[v] {
				gone = append(gone, k)
			}
		}
		switch len(gone) {
		case 0:
			w.emit(t, fB.group)
		case 3:
		case 1:
			z := gone[0]
			x, y, below := t[(z+1)%3], t[(z+2)%3], t[z]
			cyz, err := cut(below, y)
			if err != nil {
				return nil, err
			}
			czx, err := cut(below, x)
			if err != nil {
				return nil, err
			}
			w.emit([3]int{x, y, cyz}, fB.group)
			w.emit([3]int{x, cyz, czx}, fB.group)
		case 2:
			x := 0
			for k := range 3 {
				if !removedB[t[k]] {
					x = k
				}
			}
			a, b, d := t[x], t[(x+1)%3], t[(x+2)%3]
			cab, err := cut(b, a)
			if err != nil {
				return nil, err
			}
			cda, err := cut(d, a)
			if err != nil {
				return nil, err
			}
			w.emit([3]int{a, cab, cda}, fB.group)
		}
	}
	w.removed = append(w.removed, fB.facets...)

	// The wall's removed part of each chain, and the cut vertices on the chain
	// edges where the setback line crosses them.
	var remB [2][]int
	var pB [2]int
	for i := range 2 {
		seq := append([]int{c.ends[i].v}, runB.chain[i]...)
		if !removedB[seq[0]] {
			return nil, fmt.Errorf("%w: an edge end is not on the removed side of the wall's setback line", decaderr.ErrUnsupported)
		}
		crossed := false
		for k := 1; k < len(seq); k++ {
			if removedB[seq[k-1]] && !removedB[seq[k]] {
				id, ok := w.cuts[[2]int{min(seq[k-1], seq[k]), max(seq[k-1], seq[k])}]
				if !ok {
					return nil, fmt.Errorf("%w: the wall's chain edge at the setback line was not cut", decaderr.ErrUnsupported)
				}
				pB[i], crossed = id, true
				break
			}
			if !removedB[seq[k]] {
				break
			}
			remB[i] = append(remB[i], seq[k])
		}
		if !crossed {
			return nil, fmt.Errorf("%w: the wall's chain ends before the blend's setback line", decaderr.ErrDegenerate)
		}
	}
	exemptB := map[int]bool{}
	for i := range 2 {
		exemptB[c.ends[i].v] = true
		for _, v := range runB.chain[i] {
			exemptB[v] = true
		}
	}
	for _, v := range runB.inner {
		exemptB[v] = true
	}
	for v, gone := range removedB {
		if gone && !exemptB[v] {
			return nil, fmt.Errorf("%w: another part of the wall lies within the blend's reach of the edge", decaderr.ErrUnsupported)
		}
	}

	// The cut vertices, in order along the axis, are the columns of the strip.
	var cols []int
	for _, id := range w.cuts {
		cols = append(cols, id)
	}
	sort.Slice(cols, func(a, b int) bool {
		return w.verts[cols[a]].Sub(k0).Dot(axis) < w.verts[cols[b]].Sub(k0).Dot(axis)
	})
	if len(cols) < 2 || cols[0] != pB[0] || cols[len(cols)-1] != pB[1] {
		return nil, fmt.Errorf("%w: the wall's cut does not run from one end to the other", decaderr.ErrUnsupported)
	}
	sigma := make([]float64, len(cols))
	for k, id := range cols {
		sigma[k] = w.verts[id].Sub(k0).Dot(axis)
	}
	sigma[0], sigma[len(cols)-1] = 0, length
	for k := 1; k < len(cols); k++ {
		if !(sigma[k]-sigma[k-1] > 0) {
			return nil, fmt.Errorf("%w: two cut vertices of the wall share a place on the axis", decaderr.ErrUnsupported)
		}
	}

	// The plane: the setback points and the collinear vertices between them.
	m := len(sec.rows) - 1
	bound := w.tau + slack
	rowIdx := make([][]int, m+1)
	for j := range rowIdx {
		rowIdx[j] = make([]int, len(cols))
	}
	for k := range cols {
		for j := range m {
			rowIdx[j][k] = w.vertex(sec.rows[j].Add(axis.Scale(sigma[k])), bound)
		}
		rowIdx[m][k] = cols[k]
	}
	var remA [2][]int
	for i := range 2 {
		end := c.ends[i]
		p := w.verts[rowIdx[0][0]]
		if i == 1 {
			p = w.verts[rowIdx[0][len(cols)-1]]
		}
		tp := tau(p, end.exact, runA.dir[i])
		chain := runA.chain[i]
		if kindA[i] == extends {
			if tp >= -margin {
				return nil, fmt.Errorf("%w: a setback point lies on the wrong side of an edge end", decaderr.ErrUnsupported)
			}
			continue
		}
		last := tau(w.verts[chain[len(chain)-1]], end.exact, runA.dir[i])
		if tp <= margin {
			return nil, fmt.Errorf("%w: a setback point lies behind an edge end", decaderr.ErrUnsupported)
		}
		if err := clearance(last-tp, w.beta[chain[len(chain)-1]]+margin, "the plane beside the edge"); err != nil {
			return nil, err
		}
		for _, v := range chain {
			tc := tau(w.verts[v], end.exact, runA.dir[i])
			if math.Abs(tc-tp) <= 4*margin {
				return nil, fmt.Errorf("%w: a setback point falls on a vertex of its face's boundary", decaderr.ErrUnsupported)
			}
			if tc < tp {
				remA[i] = append(remA[i], v)
			}
		}
	}

	// The plane keeps clear of the strip.
	{
		exempt := map[int]bool{c.ends[0].v: true, c.ends[1].v: true}
		for _, v := range runA.inner {
			exempt[v] = true
		}
		for i := range 2 {
			for _, v := range runA.chain[i] {
				exempt[v] = true
			}
		}
		zone := []r3.Vec{k0, k1, w.verts[rowIdx[0][len(cols)-1]], w.verts[rowIdx[0][0]]}
		for _, loop := range fA.loops {
			for k, v := range loop {
				q := loop[(k+1)%len(loop)]
				if exempt[v] && exempt[q] {
					continue
				}
				if w.nearZone(w.verts[v], w.verts[q], zone, nA, max(w.beta[v], w.beta[q])+margin) {
					return nil, fmt.Errorf("%w: a face boundary comes within the blend's reach of the edge", decaderr.ErrUnsupported)
				}
			}
		}
	}

	// The end faces.
	var gLoops [2][][]int
	for i := range 2 {
		col := 0
		if i == 1 {
			col = len(cols) - 1
		}
		rows := make([]int, m+1)
		reach := make([]r3.Vec, m+1)
		for j := range rows {
			rows[j] = rowIdx[j][col]
			reach[j] = w.verts[rows[j]]
		}
		kinds := [2]sideKind{kindA[i], trims}
		loops, _, err := w.endFace(c.ends[i], fG[i], runA.chain[i], runB.chain[i], kinds, remA[i], remB[i], rows, reach,
			margin, convex, nil)
		if err != nil {
			return nil, err
		}
		gLoops[i] = loops
	}
	aLoops, err := replaceRun(fA, runA, [2]int{c.ends[0].v, c.ends[1].v}, remA, kindA, rowIdx[0])
	if err != nil {
		return nil, err
	}
	for _, f := range []struct {
		loops [][]int
		n, o  r3.Vec
		fl    faceLoops
	}{{aLoops, nA, k0, fA}, {gLoops[0], c.ends[0].g.n, k0, fG[0]}, {gLoops[1], c.ends[1].g.n, k1, fG[1]}} {
		if err := w.retriangulate(f.n, f.o, f.loops, f.fl.group); err != nil {
			return nil, err
		}
		w.removed = append(w.removed, f.fl.facets...)
	}

	// The strip, and its tag.
	tag := c.tag(spec, sec, convex)
	blendGroup := len(fp.Groups)
	stripFrom := len(w.out)
	for j := range m {
		for k := 0; k+1 < len(cols); k++ {
			a, b, cc, d := rowIdx[j][k], rowIdx[j][k+1], rowIdx[j+1][k+1], rowIdx[j+1][k]
			for _, t := range [][3]int{{a, b, cc}, {a, cc, d}} {
				va, vb, vc := w.verts[t[0]], w.verts[t[1]], w.verts[t[2]]
				centroid := va.Add(vb).Add(vc).Scale(1.0 / 3)
				if vb.Sub(va).Cross(vc.Sub(va)).Dot(tag.outward(centroid)) < 0 {
					t[1], t[2] = t[2], t[1]
				}
				w.emit(t, blendGroup)
			}
		}
	}
	corner := c.cornerArea(spec, sec, k0, o0, wA)
	if !convex {
		corner = -corner
	}
	w.removedTrue = polyArea(k0, k1, w.verts[rowIdx[0][len(cols)-1]], w.verts[rowIdx[0][0]]) +
		c.radius*sec.thetaT*length + 2*corner
	if spec.fillet {
		tag.trueArea = spec.r * sec.phi * length
	} else {
		tag.trueArea = sec.tb.Sub(sec.ta).Len() * length
	}

	groups := append([]facetedrecord.Group(nil), fp.Groups...)
	groups = append(groups, w.tagGroup(facetedrecord.Group{
		Origins: []provenance.Ref{{Producer: env.Ref, Role: role}},
		Source:  facetedrecord.BlendSource(env.Ref, ord),
		Planar:  !spec.fillet,
	}, tag))
	for _, f := range faces {
		groups[f.group].HasCentroid = false
		groups[f.group].Centroid = measurement.VecMeasurement{}
	}
	return w.buildClear(ctx, env, groups, stripFrom, tag, faceIndex)
}

// tag is the exact surface of the blend face.
func (c cylLine) tag(spec blendSpec, sec section, convex bool) blendTag {
	if spec.fillet {
		tag := blendTag{
			surface:  surfacegeom.Cylinder{Origin: sec.centre, Axis: c.axis, Radius: units.Millimeters(spec.r)},
			reversed: !convex,
		}
		tag.outward = func(p r3.Vec) r3.Vec {
			q := p.Sub(sec.centre)
			q = q.Sub(c.axis.Scale(q.Dot(c.axis)))
			if !convex {
				q = q.Scale(-1)
			}
			return q
		}
		return tag
	}
	chord, _ := sec.tb.Sub(sec.ta).Normalize()
	frame, err := r3.NewFrame(sec.ta, c.axis, chord)
	// The chamfer faces away from the material: toward the bisector of the
	// plane's and the wall's outward normals at the corner.
	n := frame.N()
	k0 := c.ends[0].exact
	radial := k0.Sub(c.axisPoint(k0)).Scale(1 / c.radius)
	if c.matOutside {
		radial = radial.Scale(-1)
	}
	away := c.a.n.Add(radial)
	tag := blendTag{surface: surfacegeom.Plane{Frame: frame}}
	if err == nil {
		tag.reversed = n.Dot(away) < 0
	}
	if n.Dot(away) < 0 {
		n = n.Scale(-1)
	}
	tag.outward = func(r3.Vec) r3.Vec { return n }
	return tag
}
