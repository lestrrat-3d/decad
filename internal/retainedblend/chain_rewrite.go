package retainedblend

import (
	"context"
	"fmt"
	"math"
	"slices"
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

// This file builds a tangent chain (agent-docs/retained-modify-reach-design.md
// §5.5): every edge of one loop of the base plane P, or an open run of them, is
// blended as one chain.
// Section points of a junction are computed once and shared by both strips.

// chainEntry is one chain edge in walk order, which is the direction of the
// base loop.
type chainEntry struct {
	edge   *topo.Edge
	ord    int
	ref    refTag
	wall   *topo.Face
	wallIx int
	arc    bool
	rim    rim
	ja, jb int // junction indices at the start and end
	p0, p1 r3.Vec
	d      r3.Vec // line: direction
	m      [2]r3.Vec
	sigma  float64 // arc: +1 when the walk turns counter-clockwise about the base normal
	phi0   float64
	span   float64 // arc: swept angle
	length float64 // line: length
	cols   []chainCol
	run    []int
	step   blendSpec
	// freeStart and freeEnd are set for an arc that ends the chain on a
	// plane G (chain_arc_end.go).
	freeStart, freeEnd *arcEnd
}

// chainCol is one cross-section of the chain: the point on the chain, the way
// into P, and the vertices of its rows 0 … M−1.
type chainCol struct {
	s    float64
	rows []int
}

func heldIndex(fp facetedrecord.Payload, v *topo.Vertex) int {
	for i, p := range fp.Verts {
		if p == v.Position {
			return i
		}
	}
	return -1
}

// rewriteChain blends the chain on a copy of the held mesh.
func rewriteChain(ctx context.Context, env topo.Env, fp facetedrecord.Payload, cyc chainCycle, ords []int,
	refs []refTag, spec blendSpec, op string, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	n := len(cyc.edges)
	scale := 1.0
	for _, v := range fp.Verts {
		scale = max(scale, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	w := newWork(fp, rim{scale: scale}, spec)
	slack := 64 * ulp * scale
	margin := w.tau + slack

	// The base: the one face every chain edge borders.
	var base *topo.Face
	for _, f := range cyc.edges[0].Faces {
		all := true
		for _, e := range cyc.edges[1:] {
			all = all && (e.Faces[0] == f || e.Faces[1] == f)
		}
		if all {
			if base != nil {
				return nil, fmt.Errorf("%w: the chain's edges share two faces", decaderr.ErrUnsupported)
			}
			base = f
		}
	}
	if base == nil {
		return nil, fmt.Errorf("%w: the chain's edges share no base plane", decaderr.ErrUnsupported)
	}
	bp, ok := retainedPlane(base)
	if !ok {
		return nil, fmt.Errorf("%w: the chain's base is not a retained plane", decaderr.ErrUnsupported)
	}
	nP, oP := bp.n, bp.o

	// The chain's points on the base loop, in the chain's own order: the
	// junctions, and for an open chain the two corners where it ends.
	open := cyc.open
	nj := n
	if open {
		nj = n + 1
	}
	jp := make([]r3.Vec, nj)
	hv := make([]int, nj)
	ats := make([]*topo.Vertex, nj)
	var ecuts [2]*chainEndCut
	edgesW := append([]*topo.Edge(nil), cyc.edges...)
	for i, l := range cyc.joins {
		k := i
		if open {
			k = i + 1
		}
		jp[k], ats[k] = l.j.point, l.at
	}
	if open {
		first, last := cyc.freeEnds()
		ats[0], ats[n] = first, last
		for k, c := range []struct {
			v *topo.Vertex
			e *topo.Edge
		}{{first, cyc.edges[0]}, {last, cyc.edges[n-1]}} {
			ec, err := w.readEndCorner(c.v, c.e, base, bp, fp, faceIndex, slack)
			if err != nil {
				return nil, err
			}
			ecuts[k] = ec
		}
		jp[0], jp[n] = ecuts[0].exact, ecuts[1].exact
	}
	for i := range nj {
		if hv[i] = heldIndex(fp, ats[i]); hv[i] < 0 {
			return nil, fmt.Errorf("%w: a chain junction is not a vertex of the held mesh", decaderr.ErrUnsupported)
		}
		if open && (i == 0 || i == n) {
			continue
		}
		if d := fp.Verts[hv[i]].Sub(jp[i]).Len(); d > fp.VertexBound[hv[i]]+slack {
			return nil, fmt.Errorf("%w: a chain junction is not where its tags meet", decaderr.ErrUnsupported)
		}
	}
	fl, err := w.faceLoops(faceIndex[base])
	if err != nil {
		return nil, err
	}
	li, ok := fl.loopWith(hv...)
	if !ok {
		return nil, fmt.Errorf("%w: the chain's junctions lie on no one loop of the base", decaderr.ErrUnsupported)
	}
	loop := fl.loops[li]
	pos := make([]int, nj)
	for i := range pos {
		pos[i] = indexIn(loop, hv[i])
	}
	entries := make([]chainEntry, n)
	if open {
		// The walk follows the base loop forward: the chain's points must
		// appear in the loop in one cyclic order, and the order reverses when
		// the loop runs the other way.
		fwd, bwd := 0, 0
		for i := 0; i+1 < nj; i++ {
			fwd += (pos[i+1] - pos[i] + len(loop)) % len(loop)
			bwd += (pos[i] - pos[i+1] + len(loop)) % len(loop)
		}
		switch {
		case fwd < len(loop):
		case bwd < len(loop):
			slices.Reverse(jp)
			slices.Reverse(hv)
			slices.Reverse(ats)
			slices.Reverse(pos)
			slices.Reverse(edgesW)
			ecuts[0], ecuts[1] = ecuts[1], ecuts[0]
		default:
			return nil, fmt.Errorf("%w: the chain does not follow its base loop", decaderr.ErrUnsupported)
		}
		for s := range entries {
			entries[s] = chainEntry{edge: edgesW[s], ja: s, jb: s + 1}
		}
	} else {
		descents := 0
		for i := range pos {
			if pos[(i+1)%n] < pos[i] {
				descents++
			}
		}
		switch descents {
		case 1:
			for s := range entries {
				entries[s] = chainEntry{edge: cyc.edges[(s+1)%n], ja: s, jb: (s + 1) % n}
			}
		case n - 1:
			for s := range entries {
				a := n - 1 - s
				entries[s] = chainEntry{edge: cyc.edges[a], ja: a, jb: (a - 1 + n) % n}
			}
		default:
			return nil, fmt.Errorf("%w: the chain does not follow its base loop", decaderr.ErrUnsupported)
		}
	}
	ordOf := map[*topo.Edge]int{}
	refOf := map[*topo.Edge]refTag{}
	for i, e := range cyc.edges {
		ordOf[e], refOf[e] = ords[i], refs[i]
	}
	covered := 0
	topSet := map[int]bool{}
	for _, v := range loop {
		topSet[v] = true
	}
	for t := range entries {
		en := &entries[t]
		en.ord, en.ref = ordOf[en.edge], refOf[en.edge]
		en.p0, en.p1 = jp[en.ja], jp[en.jb]
		for _, f := range en.edge.Faces {
			if f != base {
				en.wall = f
			}
		}
		en.wallIx = faceIndex[en.wall]
		if !en.edge.Convex {
			return nil, fmt.Errorf("%w: a concave tangent chain is not built", decaderr.ErrUnsupported)
		}
		step, err := spec.assign(en.ref, base, en.wall)
		if err != nil {
			return nil, err
		}
		en.step = step
		if t > 0 && (step.planeSetback() != entries[0].step.planeSetback() ||
			step.wallSetback() != entries[0].step.wallSetback()) {
			return nil, fmt.Errorf("%w: a chain takes one setback on the base and one on the walls",
				decaderr.ErrUnsupported)
		}
		if !(step.planeSetback() > 0) || !(step.wallSetback() > 0) {
			return nil, fmt.Errorf("%w: a blend needs a positive size", decaderr.ErrDegenerate)
		}
		// The loop vertices strictly between the two junctions.
		for k := pos[en.ja] + 1; ; k++ {
			v := loop[k%len(loop)]
			if k%len(loop) == pos[en.jb] {
				break
			}
			en.run = append(en.run, v)
		}
		covered += 1 + len(en.run)
		if err := w.readChainEdge(en, fp, faceIndex, nP, ats, slack); err != nil {
			return nil, err
		}
		if open && t == 0 {
			en.freeStart = ecuts[0].arc
		}
		if open && t == n-1 {
			en.freeEnd = ecuts[1].arc
		}
	}
	if !open && covered != len(loop) {
		return nil, fmt.Errorf("%w: the chain is not the whole of its base loop", decaderr.ErrUnsupported)
	}
	tP, tW := entries[0].step.planeSetback(), entries[0].step.wallSetback()

	// The rows of a cross-section.
	fillet := spec.fillet
	rowsM := 1
	if fillet {
		step := 2 * math.Acos(1-math.Min(w.tau/spec.r, 1))
		rowsM = max(2, int(math.Ceil(math.Pi/2/step)))
	}
	if !fillet {
		// A chamfer's section is one segment; an arc end needs rows along it
		// for the chords to follow the section G cuts in the cone.
		for k, ec := range ecuts {
			if ec == nil || ec.arc == nil {
				continue
			}
			en := entries[0]
			if k == 1 {
				en = entries[n-1]
			}
			p, _ := ec.arc.a.profile(en.step)
			rowsM = max(rowsM, ec.arc.chamferEndRows(p, ec.exact, w.tau))
		}
	}
	rowPoint := func(v, m r3.Vec, j int) r3.Vec {
		if !fillet {
			s := float64(j) / float64(rowsM)
			if j == 0 {
				return v.Add(m.Scale(tP))
			}
			return v.Add(m.Scale(tP * (1 - s))).Sub(nP.Scale(tW * s))
		}
		alpha := float64(j) * math.Pi / 2 / float64(rowsM)
		return v.Add(m.Scale(spec.r * (1 - math.Sin(alpha)))).Sub(nP.Scale(spec.r * (1 - math.Cos(alpha))))
	}
	bound := w.tau + slack
	newRows := func(v, m r3.Vec) []int {
		rows := make([]int, rowsM)
		for j := range rows {
			rows[j] = w.vertex(rowPoint(v, m, j), bound)
		}
		return rows
	}
	jrows := make([][]int, nj)
	// An end column slides its rows along the edge to the end face.
	endRows := func(v, m, d r3.Vec, g plane) []int {
		rows := make([]int, rowsM)
		for j := range rows {
			rows[j] = w.vertex(setbackPoint(rowPoint(v, m, j), d, g), bound)
		}
		return rows
	}
	for t := range entries {
		if open && t == 0 && ecuts[0].arc != nil {
			if jrows[0], err = w.arcEndColumn(ecuts[0], entries[0].step, rowsM, bound); err != nil {
				return nil, err
			}
			continue
		}
		if open && t == 0 {
			jrows[0] = endRows(entries[0].p0, entries[0].m[0], entries[0].d, ecuts[0].g)
			continue
		}
		prev := entries[(t-1+n)%n]
		jrows[entries[t].ja] = newRows(entries[t].p0, prev.m[1])
	}
	if open && ecuts[1].arc != nil {
		if jrows[n], err = w.arcEndColumn(ecuts[1], entries[n-1].step, rowsM, bound); err != nil {
			return nil, err
		}
	} else if open {
		last := entries[n-1]
		jrows[n] = endRows(last.p1, last.m[1], last.d, ecuts[1].g)
	}
	for t := range entries {
		en := &entries[t]
		if !en.arc {
			en.cols = []chainCol{{s: 0, rows: jrows[en.ja]}, {s: en.length, rows: jrows[en.jb]}}
			continue
		}
		fan := en.freeStart != nil || en.freeEnd != nil
		if fan {
			if err := en.checkFan(rowsM); err != nil {
				return nil, err
			}
		}
		rhoMax := max(en.rim.radius, en.rim.radius+en.rim.kappa*tP)
		dTheta := 2 * math.Acos(1-math.Min(w.tau/rhoMax, 1))
		na := max(2, int(math.Ceil(en.span/dTheta)))
		for i := 0; i <= na; i++ {
			f := float64(i) / float64(na)
			switch {
			case i == 0:
				en.cols = append(en.cols, chainCol{s: 0, rows: jrows[en.ja]})
			case i == na:
				en.cols = append(en.cols, chainCol{s: en.span, rows: jrows[en.jb]})
			case fan:
				rows := w.fanRows(en, f, rowsM, bound)
				en.cols = append(en.cols, chainCol{s: en.param(w.verts[rows[rowsM-1]]), rows: rows})
			default:
				phi := en.phi0 + en.sigma*en.span*f
				v := en.rim.at(en.rim.radius, phi, 0)
				radial, _ := v.Sub(en.rim.center).Normalize()
				en.cols = append(en.cols, chainCol{s: en.span * f, rows: newRows(v, radial.Scale(-en.sigma))})
			}
		}
	}

	// The base face gives way to the offset contour.
	var ringA []int
	for _, en := range entries {
		for _, c := range en.cols[:len(en.cols)-1] {
			ringA = append(ringA, c.rows[0])
		}
	}
	if open {
		last := entries[n-1].cols
		ringA = append(ringA, last[len(last)-1].rows[0])
	}
	if err := w.checkContour(ringA, nP, oP, open); err != nil {
		return nil, err
	}
	var baseLoop []int
	if open {
		for k := pos[0]; ; k++ {
			baseLoop = append(baseLoop, loop[k%len(loop)])
			if k%len(loop) == pos[n] {
				break
			}
		}
		for _, ec := range ecuts {
			if err := w.readEndCut(ec, nP, tP, tW, margin, slack); err != nil {
				return nil, err
			}
		}
	} else {
		baseLoop = loop
	}
	if err := w.checkBaseClear(fl, li, baseLoop, entries, tP, margin, ecuts[:]); err != nil {
		return nil, err
	}
	loops := make([][]int, 0, len(fl.loops))
	for k, lp := range fl.loops {
		if k != li {
			loops = append(loops, lp)
		}
	}
	if open {
		run := map[int]bool{}
		for _, v := range baseLoop {
			run[v] = true
		}
		for _, ec := range ecuts {
			for _, v := range ec.remP {
				run[v] = true
			}
		}
		outside, _, err := splice(loop, run)
		if err != nil {
			return nil, err
		}
		loops = append(loops, append(outside, ringA...))
	} else {
		loops = append(loops, ringA)
	}
	if err := w.retriangulate(nP, oP, loops, fl.group); err != nil {
		return nil, err
	}
	w.removed = append(w.removed, fl.facets...)

	// The walls lose the band between P and its parallel plane at depth tW.
	faceVerts := w.faceOfVertex()
	wallSet := map[int]bool{}
	for _, en := range entries {
		wallSet[en.wallIx] = true
	}
	if len(wallSet) != n {
		return nil, fmt.Errorf("%w: a chain borders one wall twice", decaderr.ErrUnsupported)
	}
	cutsOf := make([]map[int]bool, n)
	wallGroups := make([]int, n)
	for t := range entries {
		en := entries[t]
		near := map[int]bool{en.wallIx: true, entries[(t+1)%n].wallIx: true, entries[(t-1+n)%n].wallIx: true}
		if open && t == 0 {
			near[ecuts[0].faceIx] = true
		}
		if open && t == n-1 {
			near[ecuts[1].faceIx] = true
		}
		cutsOf[t], wallGroups[t], err = w.clipChainWall(en, nP, oP, tW, topSet, near, faceVerts, slack)
		if err != nil {
			return nil, err
		}
	}
	ringB := make([][]int, n)
	seam := make([]int, nj)
	for t := range entries {
		if open && t == 0 {
			if ecuts[0].arc != nil {
				seam[0], err = w.seamCut(ecuts[0], cutsOf[0])
			} else {
				seam[0], err = w.findEndCut(cutsOf[0], jp[0], nP, slack)
			}
			if err != nil {
				return nil, err
			}
			continue
		}
		var common []int
		for c := range cutsOf[t] {
			if cutsOf[(t-1+n)%n][c] {
				common = append(common, c)
			}
		}
		if len(common) != 1 {
			return nil, fmt.Errorf("%w: a seam between two walls is not cut once", decaderr.ErrUnsupported)
		}
		seam[t] = common[0]
	}
	if open {
		if ecuts[1].arc != nil {
			seam[n], err = w.seamCut(ecuts[1], cutsOf[n-1])
		} else {
			seam[n], err = w.findEndCut(cutsOf[n-1], jp[n], nP, slack)
		}
		if err != nil {
			return nil, err
		}
	}
	for t := range entries {
		en := entries[t]
		first, last := seam[t], seam[(t+1)%n]
		if open {
			last = seam[t+1]
		}
		var mid []int
		for c := range cutsOf[t] {
			if c != first && c != last {
				mid = append(mid, c)
			}
		}
		param := func(c int) float64 { return en.param(w.verts[c]) }
		sort.Slice(mid, func(a, b int) bool { return param(mid[a]) < param(mid[b]) })
		ringB[t] = append(append([]int{first}, mid...), last)
	}

	// The end faces give way to the end columns.
	var removedTrue, blendTrue float64
	if open {
		for i, ec := range ecuts {
			col := 0
			if i == 1 {
				col = n
			}
			seq := append(append([]int(nil), jrows[col]...), seam[col])
			corner, err := w.rebuildEnd(ec, seq, spec.fillet, spec.r, tP, tW)
			if err != nil {
				return nil, err
			}
			removedTrue += corner
		}
	}

	// The strips.
	stripFrom := len(w.out)
	groups := append([]facetedrecord.Group(nil), fp.Groups...)
	for t := range entries {
		en := entries[t]
		tag, planar := w.chainTag(en, nP, tP, tW)
		group := len(fp.Groups) + t
		emit := func(a, b, c int) {
			tri := [3]int{a, b, c}
			pa, pb, pc := w.verts[a], w.verts[b], w.verts[c]
			centroid := pa.Add(pb).Add(pc).Scale(1.0 / 3)
			if pb.Sub(pa).Cross(pc.Sub(pa)).Dot(tag.outward(centroid)) < 0 {
				tri[1], tri[2] = tri[2], tri[1]
			}
			w.emit(tri, group)
		}
		for i := 0; i+1 < len(en.cols); i++ {
			for j := 0; j+1 < rowsM; j++ {
				a, b := en.cols[i].rows, en.cols[i+1].rows
				emit(a[j], b[j], b[j+1])
				emit(a[j], b[j+1], a[j+1])
			}
		}
		last := rowsM - 1
		bi := 0
		bp := make([]float64, len(ringB[t]))
		for k, c := range ringB[t] {
			bp[k] = en.param(w.verts[c])
		}
		bp[0], bp[len(bp)-1] = 0, en.cols[len(en.cols)-1].s
		for i := 0; i+1 < len(en.cols) || bi+1 < len(ringB[t]); {
			advanceA := bi+1 >= len(ringB[t]) ||
				(i+1 < len(en.cols) && en.cols[i+1].s <= bp[bi+1])
			if advanceA {
				emit(en.cols[i].rows[last], en.cols[i+1].rows[last], ringB[t][bi])
				i++
				continue
			}
			emit(en.cols[i].rows[last], ringB[t][bi+1], ringB[t][bi])
			bi++
		}
		pa, wa, ba := en.areas(tP, tW, spec)
		if en.freeStart != nil || en.freeEnd != nil {
			pa, wa, ba = en.areasFree(tP, tW)
		}
		removedTrue += pa + wa
		blendTrue += ba
		groups = append(groups, facetedrecord.Group{
			Origins:  []provenance.Ref{{Producer: env.Ref, Role: fmt.Sprintf("%s(%d)", op, en.ord)}},
			Surface:  tag.surface,
			Retained: true,
			Reversed: tag.reversed,
			Planar:   planar,
		})
	}
	if open {
		for _, ec := range ecuts {
			wallGroups = append(wallGroups, ec.fl.group)
		}
	}
	for _, g := range append([]int{fl.group}, wallGroups...) {
		groups[g].HasCentroid = false
		groups[g].Centroid = measurement.VecMeasurement{}
	}
	w.removedTrue = removedTrue
	return w.build(ctx, env, groups, stripFrom, blendTag{trueArea: blendTrue})
}

// param is the position of p along the entry: the distance from its start for
// a line, the swept angle for an arc.
func (en chainEntry) param(p r3.Vec) float64 {
	if !en.arc {
		return p.Sub(en.p0).Dot(en.d)
	}
	d := en.sigma * (en.rim.angle(p) - en.phi0)
	return d - 2*math.Pi*math.Floor((d+math.Pi)/(2*math.Pi))
}

// readChainEdge fills in the entry's geometry and checks that the base loop's
// run between its junctions lies on the edge.
func (w *work) readChainEdge(en *chainEntry, fp facetedrecord.Payload, faceIndex map[*topo.Face]int,
	nP r3.Vec, ats []*topo.Vertex, slack float64) error {
	switch curve := en.edge.Curve.(type) {
	case surfacegeom.Line3:
		wp, ok := retainedPlane(en.wall)
		if !ok {
			return fmt.Errorf("%w: a chain wall is not a retained plane", decaderr.ErrUnsupported)
		}
		d, ok := en.p1.Sub(en.p0).Normalize()
		if !ok {
			return fmt.Errorf("%w: a chain edge has no length", decaderr.ErrDegenerate)
		}
		en.d, en.length = d, en.p1.Sub(en.p0).Len()
		m := nP.Cross(d)
		if wp.n.Add(m).Len() > 1e-9 {
			return fmt.Errorf("%w: a chain wall does not lean away from the base loop's inside", decaderr.ErrUnsupported)
		}
		en.m = [2]r3.Vec{m, m}
		for _, v := range en.run {
			t := w.verts[v].Sub(en.p0).Dot(d)
			if distToLine(w.verts[v], en.p0, d) > w.beta[v]+slack || t < -w.beta[v]-slack || t > en.length+w.beta[v]+slack {
				return fmt.Errorf("%w: the base loop leaves a chain edge", decaderr.ErrUnsupported)
			}
		}
	case surfacegeom.Arc3:
		r, err := readRim(en.edge, fp, faceIndex)
		if err != nil {
			return err
		}
		if r.planeFace == nil || r.wallFace != en.wall || !r.convex {
			return fmt.Errorf("%w: a chain arc does not border the base and its wall convexly", decaderr.ErrUnsupported)
		}
		if err := r.checkSize(en.step); err != nil {
			return err
		}
		en.arc, en.rim, en.sigma = true, r, -r.kappa
		// The walk's own sense must agree with the arc's recorded sense.
		sense := 1.0
		if curve.Axis.Dot(nP) < 0 {
			sense = -1
		}
		if en.edge.Start != ats[en.ja] {
			sense = -sense
		}
		if sense != en.sigma {
			return fmt.Errorf("%w: a chain arc turns against its base loop", decaderr.ErrUnsupported)
		}
		for _, p := range []r3.Vec{en.p0, en.p1} {
			if !r.onRim(p, 0) {
				return fmt.Errorf("%w: a chain junction is off its arc", decaderr.ErrUnsupported)
			}
		}
		en.phi0 = r.angle(en.p0)
		span := en.sigma * (r.angle(en.p1) - en.phi0)
		span -= 2 * math.Pi * math.Floor(span/(2*math.Pi))
		if !(span > minSine) {
			return fmt.Errorf("%w: a chain arc has no span", decaderr.ErrDegenerate)
		}
		if span > math.Pi+1e-9 {
			return fmt.Errorf("%w: a chain arc spans more than a half turn", decaderr.ErrUnsupported)
		}
		en.span = span
		for k, p := range []r3.Vec{en.p0, en.p1} {
			radial, _ := p.Sub(r.center).Normalize()
			en.m[k] = radial.Scale(-en.sigma)
		}
		for _, v := range en.run {
			if !r.onRim(w.verts[v], w.beta[v]) {
				return fmt.Errorf("%w: the base loop leaves a chain arc", decaderr.ErrUnsupported)
			}
		}
	default:
		return fmt.Errorf("%w: a chain edge is neither a line nor an arc", decaderr.ErrUnsupported)
	}
	return nil
}

// checkContour refuses an offset contour that crosses itself: the setback
// consumes the base face (SR4). An open contour is a path and a closed one a
// polygon.
func (w *work) checkContour(ring []int, nP, oP r3.Vec, open bool) error {
	u, ok := perpendicular(nP)
	if !ok {
		return fmt.Errorf("%w: the base has no frame", decaderr.ErrUnsupported)
	}
	v := nP.Cross(u)
	ps := make([]pt2, len(ring))
	for i, vi := range ring {
		q := w.verts[vi].Sub(oP)
		ps[i] = pt2{q.Dot(u), q.Dot(v)}
	}
	n := len(ps)
	segs := n
	if open {
		segs = n - 1
	}
	for i := range segs {
		for j := i + 2; j < segs; j++ {
			if !open && i == 0 && j == n-1 {
				continue
			}
			if segmentsTouch(ps[i], ps[(i+1)%n], ps[j], ps[(j+1)%n]) {
				return fmt.Errorf("%w: the setback consumes the base face", decaderr.ErrDegenerate)
			}
		}
	}
	return nil
}

// checkBaseClear refuses a chain whose band reaches another part of the base.
// path is the chain's run on the base loop, closed when the chain is. For an
// open chain the rest of its own loop is tested too, except the segments that
// touch the chain or the end faces' lines, which the end checks cover.
func (w *work) checkBaseClear(fl faceLoops, li int, path []int, entries []chainEntry, tP, margin float64,
	ends []*chainEndCut) error {
	closed := ends[0] == nil
	nseg := len(path)
	if !closed {
		nseg--
	}
	// The longest chord of the held outline's arcs bounds their sagitta; a
	// chord of a straight edge has none.
	rMin := math.Inf(1)
	for _, en := range entries {
		if en.arc {
			rMin = math.Min(rMin, en.rim.radius)
		}
	}
	sag := 0.0
	for i := range nseg {
		p, q := path[i], path[(i+1)%len(path)]
		for _, en := range entries {
			if en.arc && en.rim.onRim(w.verts[p], w.beta[p]) && en.rim.onRim(w.verts[q], w.beta[q]) {
				sag = max(sag, w.verts[p].Sub(w.verts[q]).Len())
			}
		}
	}
	allow := 0.0
	if !math.IsInf(rMin, 1) {
		allow = sag * sag / (8 * rMin)
	}
	skip := map[int]bool{}
	if !closed {
		for _, v := range path {
			skip[v] = true
		}
		for _, ec := range ends {
			for _, v := range ec.remP {
				skip[v] = true
			}
			skip[ec.beyondP] = true
		}
	}
	for k, other := range fl.loops {
		for i, a := range other {
			b := other[(i+1)%len(other)]
			if k == li && (closed || skip[a] || skip[b]) {
				continue
			}
			gap := math.Inf(1)
			for j := range nseg {
				c, d := path[j], path[(j+1)%len(path)]
				gap = math.Min(gap, segmentDistance(w.verts[a], w.verts[b], w.verts[c], w.verts[d]))
			}
			if err := clearance(gap-tP, max(w.beta[a], w.beta[b])+margin+allow, "the base face"); err != nil {
				return err
			}
		}
	}
	return nil
}

// faceOfVertex maps each held vertex to the faces that use it.
func (w *work) faceOfVertex() map[int]map[int]bool {
	out := map[int]map[int]bool{}
	for i, t := range w.fp.Tris {
		for _, v := range t {
			if out[v] == nil {
				out[v] = map[int]bool{}
			}
			out[v][w.fp.FaceOf[i]] = true
		}
	}
	return out
}

// clipChainWall cuts the wall at depth tW below the base and returns the
// vertices it cut, by the old mesh edge they lie on.
func (w *work) clipChainWall(en chainEntry, nP, oP r3.Vec, tW float64, topSet, near map[int]bool,
	faceVerts map[int]map[int]bool, slack float64) (map[int]bool, int, error) {
	facets := facetsOfFace(w.fp.FaceOf, en.wallIx)
	group, err := w.groupOf(facets)
	if err != nil {
		return nil, 0, err
	}
	g := map[int]float64{}
	for _, f := range facets {
		for _, v := range w.fp.Tris[f] {
			if _, seen := g[v]; seen {
				continue
			}
			p := w.verts[v]
			depth := -nP.Dot(p.Sub(oP))
			b := w.beta[v] + slack
			if math.Abs(depth) <= b {
				if !topSet[v] {
					return nil, 0, fmt.Errorf("%w: a wall touches the base plane off the chain", decaderr.ErrUnsupported)
				}
				g[v] = -tW
				continue
			}
			gv := depth - tW
			if gv > b+w.tau {
				g[v] = gv
				continue
			}
			onSeam := distToLine(p, en.p0, nP) <= b || distToLine(p, en.p1, nP) <= b
			if onSeam && depth > 0 && gv < -(b+w.tau) {
				for fi := range faceVerts[v] {
					if !near[fi] {
						return nil, 0, fmt.Errorf("%w: a seam vertex inside the band borders another face",
							decaderr.ErrUnsupported)
					}
				}
				g[v] = gv
				continue
			}
			if err := clearance(gv, b+w.tau, "the wall"); err != nil {
				return nil, 0, err
			}
		}
	}
	mine := map[int]bool{}
	cut := func(a, b int) int {
		k := [2]int{min(a, b), max(a, b)}
		c, ok := w.cuts[k]
		if !ok {
			t := g[a] / (g[a] - g[b])
			p := w.verts[a].Add(w.verts[b].Sub(w.verts[a]).Scale(t))
			c = w.vertex(p, max(w.beta[a], w.beta[b])+slack+w.tau)
			w.cuts[k] = c
		}
		mine[c] = true
		return c
	}
	for _, f := range facets {
		t := w.fp.Tris[f]
		var above []int
		for k, v := range t {
			if g[v] > 0 {
				above = append(above, k)
			}
		}
		switch len(above) {
		case 0:
		case 3:
			w.emit(t, group)
		case 1:
			x := above[0]
			a, b, c := t[x], t[(x+1)%3], t[(x+2)%3]
			w.emit([3]int{a, cut(a, b), cut(c, a)}, group)
		case 2:
			z := 0
			for k := range 3 {
				if g[t[k]] <= 0 {
					z = k
				}
			}
			x, y, below := t[(z+1)%3], t[(z+2)%3], t[z]
			cyz, czx := cut(y, below), cut(below, x)
			w.emit([3]int{x, y, cyz}, group)
			w.emit([3]int{x, cyz, czx}, group)
		}
	}
	w.removed = append(w.removed, facets...)
	if len(mine) < 2 {
		return nil, 0, fmt.Errorf("%w: a wall's cut leaves fewer than two vertices", decaderr.ErrUnsupported)
	}
	return mine, group, nil
}

// chainTag is the exact surface of the entry's blend face.
func (w *work) chainTag(en chainEntry, nP r3.Vec, tP, tW float64) (blendTag, bool) {
	if en.arc {
		tmp := &work{r: en.rim, spec: en.step, tau: w.tau}
		return tmp.tag(), false
	}
	if en.step.fillet {
		r := en.step.r
		c0 := en.p0.Add(en.m[0].Scale(r)).Sub(nP.Scale(r))
		return blendTag{
			surface: surfacegeom.Cylinder{Origin: c0, Axis: en.d, Radius: units.Millimeters(r)},
			outward: func(p r3.Vec) r3.Vec {
				q := p.Sub(c0)
				return q.Sub(en.d.Scale(q.Dot(en.d)))
			},
		}, false
	}
	a0 := en.p0.Add(en.m[0].Scale(tP))
	b0 := en.p0.Sub(nP.Scale(tW))
	chord, _ := b0.Sub(a0).Normalize()
	outward := nP.Sub(en.m[0])
	tag := blendTag{outward: func(r3.Vec) r3.Vec { return outward }}
	if frame, err := r3.NewFrame(a0, en.d, chord); err == nil {
		tag.surface = surfacegeom.Plane{Frame: frame}
		tag.reversed = frame.N().Dot(outward) < 0
	}
	return tag, true
}

// areas are the exact areas the entry takes from the base and from its wall,
// and the area of its blend face.
func (en chainEntry) areas(tP, tW float64, spec blendSpec) (base, wall, blend float64) {
	if !en.arc {
		base, wall = en.length*tP, en.length*tW
		if spec.fillet {
			return base, wall, en.length * spec.r * math.Pi / 2
		}
		return base, wall, en.length * math.Hypot(tP, tW)
	}
	R, k := en.rim.radius, en.rim.kappa
	rho := R + k*tP
	base = en.span * math.Abs(R*R-rho*rho) / 2
	wall = en.span * R * tW
	if spec.fillet {
		r := spec.r
		return base, wall, en.span * r * ((R+k*r)*math.Pi/2 - k*r)
	}
	return base, wall, en.span * (R + rho) / 2 * math.Hypot(tP, tW)
}
