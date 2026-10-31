package retainedblend

import (
	"context"
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
	// A's chain runs along the tag line A ∩ G; its sense comes from the chain's
	// last vertex, and is certain only when that vertex lies farther along the
	// line than its bound.
	for i := range 2 {
		last := runA.chain[i][len(runA.chain[i])-1]
		if runA.dir[i], err = lineSense(tL, w.verts[last], c.ends[i].exact, w.beta[last]+margin); err != nil {
			return nil, err
		}
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
	// At each end the plane trims or, for a concave edge, extends past the
	// corner. The wall's chain leaves the corner along the end circle, whose
	// tangent there is ±wB: along wB the wall ends with the edge, against it the
	// wall extends past the end plane (§5.0). The sense comes from the chain's
	// first vertex, which must lie farther along the tangent than its bound. A
	// wall may extend only beside a convex edge whose plane trims there; the
	// end plane then takes the blend's end cap.
	var kindA, kindB [2]sideKind
	var extB [2]bool
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
		first := runB.chain[i][0]
		if kindB[i], err = wallSense(wB, w.verts[first], c.ends[i].exact, w.beta[first]+margin); err != nil {
			return nil, err
		}
		if kindB[i] == extends && (!convex || kindA[i] == extends) {
			return nil, fmt.Errorf("%w: the wall turns reflex at an edge end", decaderr.ErrUnsupported)
		}
		extB[i] = kindB[i] == extends
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

	// The wall: cut by the removed band (cyl_band.go), bounded by the
	// generator through tb and, at an end where the wall extends past the
	// corner, by the end face's exact plane.
	nu := radialK.Scale(-math.Sin(sec.thetaT)).Add(wB.Scale(math.Cos(sec.thetaT)))
	band := &wallBand{o0: o0, u1: radialK, u2: wB, nu: nu, radius: c.radius, thetaT: sec.thetaT, slack: slack}
	for i := range 2 {
		if !extB[i] {
			continue
		}
		g := c.ends[i].g
		sign := 1.0
		if g.offset(c.ends[1-i].exact) < 0 {
			sign = -1
		}
		on := map[int]bool{c.ends[i].v: true}
		for _, v := range runB.chain[i] {
			on[v] = true
		}
		band.ends[i] = &bandEnd{g: g, sign: sign, on: on}
	}
	cuts, err := w.cutBand(fB, band)
	if err != nil {
		return nil, err
	}
	removedB := cuts.removed
	w.removed = append(w.removed, fB.facets...)

	// The wall's removed part of each chain, and the cut vertices on the chain
	// edges where the setback line crosses them. Where the wall extends past
	// the corner its chain stays whole, and the strip ends at the band's corner
	// in the end plane.
	var remB [2][]int
	var pB [2]int
	for i := range 2 {
		if extB[i] {
			if cuts.corner[i] < 0 {
				return nil, fmt.Errorf("%w: the blend's setback line does not meet an end face on the wall",
					decaderr.ErrUnsupported)
			}
			pB[i] = cuts.corner[i]
			continue
		}
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

	// A fillet's end cap closes in a cusp at tb, and the wall's cut in the end
	// plane lies within its own bound of the circle there. Where the cap the
	// profile's own rows would make is not simple, the rows are spaced again
	// so the last row before tb lies farther inside than that cut can (§5.0,
	// "A wall that extends past the corner"); the last gap's sagitta is
	// charged below. capSimple still decides the rebuilt cap.
	cusp := 0.0
	if spec.fillet && (extB[0] || extB[1]) {
		reach, simple := 0.0, true
		for i := range 2 {
			if !extB[i] {
				continue
			}
			run, err := cuts.endRun(i, c.ends[i].v)
			if err != nil {
				return nil, err
			}
			reach = max(reach, w.beta[cuts.corner[i]])
			shift := axis.Scale(float64(i) * length)
			pts := make([]r3.Vec, 0, len(sec.rows)+len(run)+1)
			for _, p := range sec.rows[:len(sec.rows)-1] {
				pts = append(pts, p.Add(shift))
			}
			pts = append(pts, w.verts[cuts.corner[i]])
			for _, v := range run {
				reach = max(reach, w.beta[v])
				pts = append(pts, w.verts[v])
			}
			pts = append(pts, w.verts[c.ends[i].v])
			simple = simple && capSimplePts(c.ends[i].g.n, pts, len(sec.rows)-1) == nil
		}
		if !simple {
			if cusp, err = c.clearCusp(spec, &sec, o0, convex, proofbound.AbsSumUpper(reach, margin), w.tau); err != nil {
				return nil, err
			}
		}
	}

	// The cut vertices, in order along the axis, are the columns of the strip.
	var cols []int
	for _, id := range w.cuts {
		cols = append(cols, id)
	}
	for i := range 2 {
		if extB[i] {
			cols = append(cols, cuts.corner[i])
		}
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
		kinds := [2]sideKind{kindA[i], kindB[i]}
		// Where the wall extends past the corner, G's turn there is read from
		// the tag directions the chains leave along: A ∩ G, and the end circle's
		// tangent against wB. Its loop runs from the blend's last row along the
		// wall's cut in its plane back to the corner.
		var dirs *[2]r3.Vec
		var run []int
		if extB[i] {
			dirs = &[2]r3.Vec{runA.dir[i], wB.Scale(-1)}
			if run, err = cuts.endRun(i, c.ends[i].v); err != nil {
				return nil, err
			}
			// The cap runs along the rows, back along the wall's run and from
			// V along A ∩ G to P_A; it must be a simple polygon.
			capLoop := append(append(append([]int(nil), rows...), run...), c.ends[i].v)
			if err := w.capSimple(c.ends[i].g.n, capLoop, m); err != nil {
				return nil, err
			}
		}
		loops, _, err := w.endFaceRun(c.ends[i], fG[i], runA.chain[i], runB.chain[i], kinds, remA[i], remB[i], rows,
			run, reach, margin, convex, dirs)
		if err != nil {
			return nil, err
		}
		gLoops[i] = loops
	}
	aLoops, err := replaceRun(fA, runA, [2]int{c.ends[0].v, c.ends[1].v}, remA, kindA, rowIdx[0])
	if err != nil {
		return nil, err
	}
	// An end face that takes the cap is new surface where the cap lies: its
	// rebuilt facets are cleared against the rest of the body as the strip is.
	var taken [][2]int
	for _, f := range []struct {
		loops [][]int
		n, o  r3.Vec
		fl    faceLoops
		takes bool
	}{{aLoops, nA, k0, fA, false}, {gLoops[0], c.ends[0].g.n, k0, fG[0], extB[0]},
		{gLoops[1], c.ends[1].g.n, k1, fG[1], extB[1]}} {
		from := len(w.out)
		if err := w.retriangulate(f.n, f.o, f.loops, f.fl.group); err != nil {
			return nil, err
		}
		if f.takes {
			taken = append(taken, [2]int{from, len(w.out)})
		}
		w.removed = append(w.removed, f.fl.facets...)
	}
	if cusp > 0 {
		w.chargeCusp(cols, []int{rowIdx[m-1][0], rowIdx[m-1][len(cols)-1]}, cusp, spec.r, slack)
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
	w.removedTrue = cylRemovedArea(polyArea(k0, k1, w.verts[rowIdx[0][len(cols)-1]], w.verts[rowIdx[0][0]]),
		c.radius*sec.thetaT*length, c.cornerArea(spec, sec, k0, o0, wA), convex, extB)
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
	if err := w.checkStripClear(ctx, stripFrom, taken); err != nil {
		return nil, err
	}
	return w.build(ctx, env, groups, stripFrom, tag, faceIndex)
}

// clearCusp spaces a fillet's rows again so the last row before tb lies at
// least need from the wall's circle (§5.0, "A wall that extends past the
// corner"). The ball touches the circle at tb, so the arc's point at the tube
// angle ψ short of tb lies inside the circle (outside it, where the material
// lies outside the cylinder) by
//
//	gap(ψ) = κ·(√(D² + r² − 2κ·D·r·cos ψ) − R),  D = R + κ·r,
//
// which grows with ψ over the arc. The last gap spans the ψ where gap(ψ) =
// need, and the rest of the arc keeps the profile's step. It returns the last
// gap's sagitta r·(1 − cos(ψ/2)), which the rows' own bound τ does not cover,
// or 0, rows unchanged, when the last row already lies need from the circle.
// It refuses (SR5) a fillet whose tangent point on the plane lies no farther
// than need from the circle, as a fillet tiny against the wall's facets does.
func (c cylLine) clearCusp(spec blendSpec, s *section, o0 r3.Vec, convex bool, need, tau float64) (float64, error) {
	r, rr := spec.r, c.radius
	kappa := -1.0
	if convex == c.matOutside {
		kappa = 1
	}
	gap := func(p r3.Vec) float64 { return kappa * (p.Sub(o0).Len() - rr) }
	m := len(s.rows) - 1
	if gap(s.rows[m-1]) >= need {
		return 0, nil
	}
	d := rr + kappa*r
	target := rr + kappa*need
	psi := math.Acos(math.Max(-1, math.Min(1, (d*d+r*r-target*target)/(2*kappa*d*r))))
	span := s.phi - psi
	if !(gap(s.ta) > need) || !(span > minSine) {
		return 0, fmt.Errorf("%w: the fillet's arc comes no more than %g mm from the wall's circle, "+
			"and the wall's held facets may lie %g mm from it at the end cap", decaderr.ErrUnsupported, gap(s.ta), need)
	}
	a0 := s.ta.Sub(s.centre).Scale(1 / r)
	a1 := s.tb.Sub(s.centre).Scale(1 / r)
	sin := math.Sin(s.phi)
	step := 2 * math.Acos(1-math.Min(tau/r, 1))
	k := max(1, int(math.Ceil(span/step)))
	rows := make([]r3.Vec, 0, k+2)
	for j := 0; j <= k; j++ {
		alpha := span * float64(j) / float64(k)
		rows = append(rows, s.centre.Add(a0.Scale(r*math.Sin(s.phi-alpha)/sin)).Add(a1.Scale(r*math.Sin(alpha)/sin)))
	}
	rows[0] = s.ta
	s.rows = append(rows, s.tb)
	return filletSag(spec, s.phi-span), nil
}

// chargeCusp states the widened last gap of a fillet's rows (clearCusp),
// whose sagitta sag exceeds the τ the rows' own bounds hold (§6.3, "The
// cusp's last gap"). Each vertex of the wall's cut row cut takes
// cuspCharge(sag, β, r) plus slack, β its own bound; every triangle of the
// gap has such a vertex. Each of the last row's vertices in the end planes,
// end, takes sag plus slack, so both ends of the gap's chord in an end face
// state its sagitta. A bound already larger stays.
func (w *work) chargeCusp(cut, end []int, sag, r, slack float64) {
	for _, v := range cut {
		w.beta[v] = max(w.beta[v], proofbound.AbsSumUpper(cuspCharge(sag, w.beta[v], r), slack))
	}
	for _, v := range end {
		w.beta[v] = max(w.beta[v], proofbound.AbsSumUpper(sag, slack))
	}
}

// cuspCharge bounds, from above, how far the segment from an exact point P of
// a circle of radius r to a point C' within eps of another exact point C of
// it lies from the arc PC, and the arc from the segment, where the chord PC
// has sagitta sag: max over ν in [0, 1] of 4·sag·ν(1 − ν) + ν·eps, which is
// eps when eps ≥ 4·sag and (4·sag + eps)²/(16·sag) otherwise, plus
// sag²/(2r) for an arc point's distance from the chord past its radial
// distance (§6.3, "The cusp's last gap"). Every step rounds up.
func cuspCharge(sag, eps, r float64) float64 {
	if !(sag > 0) {
		return eps
	}
	far := eps
	if eps < 4*sag {
		s := proofbound.AbsSumUpper(4*sag, eps)
		far = proofbound.DivUpper(proofbound.ProductUpper(s, s), 16*sag)
	}
	return proofbound.AbsSumUpper(far, proofbound.DivUpper(proofbound.ProductUpper(sag, sag), 2*r))
}

// lineSense is the sense of the tag line dir along which a face's chain
// leaves the corner exact, read from the chain's last vertex last: the vertex
// must lie farther along the line than bound, else the sense is undecided and
// the edge refuses.
func lineSense(dir, last, exact r3.Vec, bound float64) (r3.Vec, error) {
	if last.Sub(exact).Dot(dir) < 0 {
		dir = dir.Scale(-1)
	}
	if tau(last, exact, dir) <= bound {
		return r3.Vec{}, fmt.Errorf("%w: a face leaves an edge end along too short a run to read its direction",
			decaderr.ErrUnsupported)
	}
	return dir, nil
}

// wallSense reads how the wall meets an edge end from the first vertex of its
// chain along the end circle, whose tangent at the corner exact is ±wB: along
// wB the wall ends with the edge, against it the wall extends past the end
// plane. The vertex must lie farther along the tangent than bound, and than
// minSine of its distance, else the edge refuses.
func wallSense(wB, first, exact r3.Vec, bound float64) (sideKind, error) {
	toward := first.Sub(exact)
	s := toward.Dot(wB)
	sure := max(bound, minSine*toward.Len())
	switch {
	case s > sure:
		return trims, nil
	case -s > sure:
		return extends, nil
	}
	return 0, fmt.Errorf("%w: a face leaves an edge end along too short a run to read its direction",
		decaderr.ErrUnsupported)
}

// cylRemovedArea is the exact area a plane–cylinder blend removes: the
// plane's strip and the wall's band, less the corner region at each end where
// the end face takes it as the cap (the wall extends past the corner) or adds
// the notch's fill (a concave edge), plus it at each end where a convex edge
// takes it out of the end face (the wall ends with the edge).
func cylRemovedArea(strip, band, corner float64, convex bool, extB [2]bool) float64 {
	removed := strip + band
	for i := range 2 {
		if !convex || extB[i] {
			removed -= corner
			continue
		}
		removed += corner
	}
	return removed
}

// capSimple refuses the end cap an end face takes where the wall extends past
// the corner unless the cap is a simple polygon in the face's plane n: the
// loop is the blend's rows from P_A to the band's corner at index tb, the
// wall's cut back to the corner V, and the plane's line from V to P_A. The cap
// ends in a cusp at T_A and, for a fillet, at T_B, where the exact rows meet
// the wall's held facets. A held facet lies inside the wall by up to its bound,
// so near T_B the last rows can lie beyond it and the loop crosses itself. The
// test is exact on the held coordinates and can only refuse: no two edges that
// do not share a vertex touch, no two that do run back along each other, and
// the loop turns at both cusps the way it runs.
func (w *work) capSimple(n r3.Vec, loop []int, tb int) error {
	pts := make([]r3.Vec, len(loop))
	for i, id := range loop {
		pts[i] = w.verts[id]
	}
	return capSimplePts(n, pts, tb)
}

// capSimplePts is capSimple on the loop's points.
func capSimplePts(n r3.Vec, pts []r3.Vec, tb int) error {
	u, ok := perpendicular(n)
	if !ok {
		return fmt.Errorf("%w: an end face has no plane frame", decaderr.ErrUnsupported)
	}
	v := n.Cross(u)
	k := len(pts)
	ps := make([]pt2, k)
	idx := make([]int, k)
	for i, p := range pts {
		q := p.Sub(pts[0])
		ps[i], idx[i] = pt2{q.Dot(u), q.Dot(v)}, i
	}
	folds := fmt.Errorf("%w: the blend's end cap folds where its rows meet the wall's facets", decaderr.ErrUnsupported)
	area := signedArea2(ps, idx)
	if area == 0 {
		return folds
	}
	sense := 1
	if area < 0 {
		sense = -1
	}
	at := func(i int) pt2 { return ps[(i+k)%k] }
	for _, i := range []int{0, tb} {
		if orient(at(i-1), at(i), at(i+1)) != sense {
			return folds
		}
	}
	for i := range k {
		if collinearOverlap(at(i-1), at(i), at(i), at(i+1)) {
			return folds
		}
		for j := i + 2; j < k; j++ {
			if i == 0 && j == k-1 {
				continue
			}
			if segmentsTouch(at(i), at(i+1), at(j), at(j+1)) {
				return folds
			}
		}
	}
	return nil
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
