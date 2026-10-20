package retainedblend

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// edgeRun is where the straight edge sits in one face's boundary loop: the
// loop, the direction along it from end 0 to end 1, the vertices between the
// two ends, and at each end the collinear vertices leaving along the face's
// meeting with the end face.
type edgeRun struct {
	loop  int
	step  int
	i0    int
	inner []int
	chain [2][]int
	dir   [2]r3.Vec // unit direction of each chain, leaving the corner
}

// locate finds the straight edge in face f's loops.
func (w *work) locate(f faceLoops, l line, pl plane) (edgeRun, error) {
	slack := l.slack()
	v0, v1 := l.ends[0].v, l.ends[1].v
	li, ok := f.loopWith(v0, v1)
	if !ok {
		return edgeRun{}, fmt.Errorf("%w: the edge's ends lie on no one loop of its face", decaderr.ErrUnsupported)
	}
	loop := f.loops[li]
	n := len(loop)
	i0, i1 := indexIn(loop, v0), indexIn(loop, v1)
	onE := func(v int) bool { return distToLine(w.verts[v], l.ends[0].exact, l.d) <= w.beta[v]+slack }
	run := edgeRun{loop: li, i0: i0}
	found := 0
	for _, step := range []int{1, -1} {
		var inner []int
		good := true
		for i := ((i0+step)%n + n) % n; i != i1; i = ((i+step)%n + n) % n {
			if !onE(loop[i]) {
				good = false
				break
			}
			inner = append(inner, loop[i])
		}
		if good {
			run.step, run.inner = step, inner
			found++
		}
	}
	if found != 1 {
		return edgeRun{}, fmt.Errorf("%w: the edge is not one run of its face's boundary", decaderr.ErrUnsupported)
	}
	for i := range 2 {
		end := l.ends[i]
		dir, ok := pl.n.Cross(end.g.n).Normalize()
		if !ok {
			return edgeRun{}, fmt.Errorf("%w: a face is parallel to an end face", decaderr.ErrUnsupported)
		}
		onLine := func(v int) bool { return distToLine(w.verts[v], end.exact, dir) <= w.beta[v]+slack }
		at, step := i0, -run.step
		if i == 1 {
			at, step = i1, run.step
		}
		run.chain[i] = walk(loop, at, step, onLine)
		if len(run.chain[i]) == 0 {
			return edgeRun{}, fmt.Errorf("%w: a face leaves an edge end along no straight run", decaderr.ErrUnsupported)
		}
		last := w.verts[run.chain[i][len(run.chain[i])-1]]
		if last.Sub(end.exact).Dot(dir) < 0 {
			dir = dir.Scale(-1)
		}
		run.dir[i] = dir
	}
	return run, nil
}

// tau is the distance of p from corner exact along dir.
func tau(p, exact, dir r3.Vec) float64 { return p.Sub(exact).Dot(dir) }

// setbackPoint is the meeting of the line through base parallel to d with the
// end plane g.
func setbackPoint(base, d r3.Vec, g plane) r3.Vec {
	return base.Add(d.Scale(-g.offset(base) / d.Dot(g.n)))
}

// rewriteLineEdge blends one straight edge and builds the body over the
// rewritten mesh.
func rewriteLineEdge(ctx context.Context, env topo.Env, fp facetedrecord.Payload, l line, spec blendSpec,
	role string, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	w := newWork(fp, rim{scale: l.scale}, spec)
	slack := l.slack()
	margin := w.tau + slack

	faces := [4]faceLoops{}
	for k, f := range []*topo.Face{l.faceA, l.faceB, l.ends[0].face, l.ends[1].face} {
		var err error
		if faces[k], err = w.faceLoops(faceIndex[f]); err != nil {
			return nil, err
		}
	}
	fA, fB, fG := faces[0], faces[1], [2]faceLoops{faces[2], faces[3]}
	runA, err := w.locate(fA, l, l.a)
	if err != nil {
		return nil, err
	}
	runB, err := w.locate(fB, l, l.b)
	if err != nil {
		return nil, err
	}

	// The directions into the faces, and the corner's kind.
	e0 := l.ends[0].exact
	into := func(run edgeRun, pl plane, loop []int) (r3.Vec, error) {
		across := pl.n.Cross(l.d)
		far := w.verts[run.chain[0][len(run.chain[0])-1]]
		side := far.Sub(e0).Dot(across)
		if math.Abs(side) <= margin {
			return r3.Vec{}, fmt.Errorf("%w: the held bounds cannot tell which side of the edge a face lies", decaderr.ErrUnsupported)
		}
		return across.Scale(math.Copysign(1, side)), nil
	}
	wA, err := into(runA, l.a, fA.loops[runA.loop])
	if err != nil {
		return nil, err
	}
	wB, err := into(runB, l.b, fB.loops[runB.loop])
	if err != nil {
		return nil, err
	}
	convex := l.b.n.Dot(wA) < 0
	if (l.a.n.Dot(wB) < 0) != convex || l.edgeConvex != convex {
		return nil, fmt.Errorf("%w: the edge's convexity disagrees with its two tags", decaderr.ErrUnsupported)
	}
	for i := range 2 {
		for _, c := range []struct {
			run edgeRun
			w   r3.Vec
		}{{runA, wA}, {runB, wB}} {
			far := w.verts[c.run.chain[i][len(c.run.chain[i])-1]]
			if far.Sub(l.ends[i].exact).Dot(c.w) <= margin {
				return nil, fmt.Errorf("%w: a face turns back at an edge end", decaderr.ErrUnsupported)
			}
		}
	}

	// The setback sizes, and the exact rows of the blend.
	tA, tB := spec.planeSetback(), spec.wallSetback()
	if spec.fillet {
		tA = spec.r * math.Tan(l.phi/2)
		tB = tA
	}
	if !(tA > 0) || !(tB > 0) {
		return nil, fmt.Errorf("%w: a blend needs a positive size", decaderr.ErrDegenerate)
	}
	baseA, baseB := e0.Add(wA.Scale(tA)), e0.Add(wB.Scale(tB))
	var tag blendTag
	var rowsAt []r3.Vec
	var center r3.Vec
	if spec.fillet {
		ball := 1.0
		if convex {
			ball = -1
		}
		center = baseA.Add(l.a.n.Scale(ball * spec.r))
		if center.Sub(baseB.Add(l.b.n.Scale(ball*spec.r))).Len() > 16*slack*math.Max(1, spec.r) {
			return nil, fmt.Errorf("%w: the blend's tangent points do not share one ball", decaderr.ErrUnsupported)
		}
		a0, a1 := l.a.n.Scale(-ball), l.b.n.Scale(-ball)
		step := 2 * math.Acos(1-math.Min(w.tau/spec.r, 1))
		m := max(2, int(math.Ceil(l.phi/step)))
		sin := math.Sin(l.phi)
		for j := 0; j <= m; j++ {
			alpha := l.phi * float64(j) / float64(m)
			rowsAt = append(rowsAt, center.Add(a0.Scale(spec.r*math.Sin(l.phi-alpha)/sin)).
				Add(a1.Scale(spec.r*math.Sin(alpha)/sin)))
		}
		rowsAt[0], rowsAt[m] = baseA, baseB
	} else {
		rowsAt = []r3.Vec{baseA, baseB}
	}
	m := len(rowsAt) - 1

	// Per end: the setback points, the chains' removed parts and the gates.
	var rowIdx [2][]int
	var remA, remB [2][]int
	for i := range 2 {
		end := l.ends[i]
		rowIdx[i] = make([]int, m+1)
		for j, p := range rowsAt {
			rowIdx[i][j] = w.vertex(setbackPoint(p, l.d, end.g), w.tau+slack)
		}
		for _, side := range []struct {
			run  edgeRun
			rows int
			rem  *[2][]int
		}{{runA, 0, &remA}, {runB, m, &remB}} {
			p := w.verts[rowIdx[i][side.rows]]
			tp := tau(p, end.exact, side.run.dir[i])
			chain := side.run.chain[i]
			last := tau(w.verts[chain[len(chain)-1]], end.exact, side.run.dir[i])
			if tp <= margin {
				return nil, fmt.Errorf("%w: a setback point lies behind an edge end", decaderr.ErrUnsupported)
			}
			if err := clearance(last-tp, w.beta[chain[len(chain)-1]]+margin, "the face beside the edge"); err != nil {
				return nil, err
			}
			for _, c := range chain {
				tc := tau(w.verts[c], end.exact, side.run.dir[i])
				if math.Abs(tc-tp) <= 4*margin {
					return nil, fmt.Errorf("%w: a setback point falls on a vertex of its face's boundary", decaderr.ErrUnsupported)
				}
				if tc < tp {
					side.rem[i] = append(side.rem[i], c)
				}
			}
		}
	}

	// The faces beside the edge keep clear of the strip.
	for _, side := range []struct {
		f    faceLoops
		run  edgeRun
		w    r3.Vec
		size float64
	}{{fA, runA, wA, tA}, {fB, runB, wB, tB}} {
		exempt := map[int]bool{l.ends[0].v: true, l.ends[1].v: true}
		for _, v := range side.run.inner {
			exempt[v] = true
		}
		for i := range 2 {
			for _, v := range side.run.chain[i] {
				exempt[v] = true
			}
		}
		for _, loop := range side.f.loops {
			for _, v := range loop {
				if exempt[v] {
					continue
				}
				if w.verts[v].Sub(e0).Dot(side.w) < side.size+w.beta[v]+margin {
					return nil, fmt.Errorf("%w: a face boundary comes within the blend's reach of the edge", decaderr.ErrUnsupported)
				}
			}
		}
	}

	// The end faces: each loses the corner, keeps clear of it, and agrees with
	// the faces beside the edge about the chains.
	var gLoops [2][][]int
	for i := range 2 {
		loops, err := w.endFace(l, i, fG[i], [2]edgeRun{runA, runB}, remA[i], remB[i], rowIdx[i], rowsAt, margin)
		if err != nil {
			return nil, err
		}
		gLoops[i] = loops
	}

	// The faces beside the edge swap the edge for its setback edge.
	aLoops, err := w.replaceEdge(fA, runA, l, remA, [2]int{rowIdx[0][0], rowIdx[1][0]})
	if err != nil {
		return nil, err
	}
	bLoops, err := w.replaceEdge(fB, runB, l, remB, [2]int{rowIdx[0][m], rowIdx[1][m]})
	if err != nil {
		return nil, err
	}
	for k, c := range []struct {
		f     faceLoops
		loops [][]int
		pl    plane
		o     r3.Vec
	}{{fA, aLoops, l.a, e0}, {fB, bLoops, l.b, e0},
		{fG[0], gLoops[0], l.ends[0].g, l.ends[0].exact}, {fG[1], gLoops[1], l.ends[1].g, l.ends[1].exact}} {
		_ = k
		if err := w.retriangulate(c.pl.n, c.o, c.loops, c.f.group); err != nil {
			return nil, err
		}
		w.removed = append(w.removed, c.f.facets...)
	}

	tag = w.lineTag(l, convex, center, baseA, baseB)
	// The strip: row after row of two vertices.
	blendGroup := len(fp.Groups)
	stripFrom := len(w.out)
	for j := range m {
		lo0, lo1 := rowIdx[0][j], rowIdx[1][j]
		hi0, hi1 := rowIdx[0][j+1], rowIdx[1][j+1]
		for _, t := range [][3]int{{lo0, lo1, hi1}, {lo0, hi1, hi0}} {
			a, b, c := w.verts[t[0]], w.verts[t[1]], w.verts[t[2]]
			centroid := a.Add(b).Add(c).Scale(1.0 / 3)
			if b.Sub(a).Cross(c.Sub(a)).Dot(tag.outward(centroid)) < 0 {
				t[1], t[2] = t[2], t[1]
			}
			w.emit(t, blendGroup)
		}
	}
	w.removedTrue, tag.trueArea = w.lineAreas(l, spec, convex, center, baseA, baseB, rowIdx, tA, tB)

	groups := append([]facetedrecord.Group(nil), fp.Groups...)
	groups = append(groups, facetedrecord.Group{
		Origins:  []provenance.Ref{{Producer: env.Ref, Role: role}},
		Surface:  tag.surface,
		Retained: true,
		Reversed: tag.reversed,
		Planar:   !spec.fillet,
	})
	for _, f := range faces {
		groups[f.group].HasCentroid = false
		groups[f.group].Centroid = measurement.VecMeasurement{}
	}
	return w.build(ctx, env, groups, stripFrom, tag)
}

// endFace builds end face G_i's new loops: the corner and the removed parts of
// the two chains give way to the end curve of the blend.
func (w *work) endFace(l line, i int, g faceLoops, runs [2]edgeRun, remA, remB, rows []int, rowsAt []r3.Vec,
	margin float64) ([][]int, error) {
	end := l.ends[i]
	li, ok := g.loopWith(end.v)
	if !ok {
		return nil, fmt.Errorf("%w: an end face does not hold the edge's corner", decaderr.ErrUnsupported)
	}
	loop := g.loops[li]
	n := len(loop)
	at := indexIn(loop, end.v)
	prev, next := loop[(at+n-1)%n], loop[(at+1)%n]
	chainA, chainB := runs[0].chain[i], runs[1].chain[i]
	var aFirst bool
	switch {
	case next == chainA[0] && prev == chainB[0]:
		aFirst = false
	case next == chainB[0] && prev == chainA[0]:
		aFirst = true
	default:
		return nil, fmt.Errorf("%w: an end face's chains disagree with the faces beside the edge", decaderr.ErrUnsupported)
	}
	// The corner must turn toward the face's inside, and nothing else of the
	// face may come near it.
	in, out := w.verts[end.v].Sub(w.verts[prev]), w.verts[next].Sub(w.verts[end.v])
	if in.Cross(out).Dot(end.g.n) <= 0 {
		return nil, fmt.Errorf("%w: an end face is not convex at the edge's corner", decaderr.ErrUnsupported)
	}
	reach := 0.0
	for _, p := range rowsAt {
		reach = max(reach, setbackPoint(p, l.d, end.g).Sub(end.exact).Len())
	}
	reach += margin
	skip := map[int]bool{end.v: true}
	for _, v := range chainA {
		skip[v] = true
	}
	for _, v := range chainB {
		skip[v] = true
	}
	for _, lp := range g.loops {
		for k, v := range lp {
			q := lp[(k+1)%len(lp)]
			if skip[v] && skip[q] {
				continue
			}
			if distToSegment(end.exact, w.verts[v], w.verts[q]) < reach+w.beta[v]+w.beta[q] {
				return nil, fmt.Errorf("%w: another part of an end face comes within the blend's reach of the corner", decaderr.ErrUnsupported)
			}
		}
	}
	run := map[int]bool{end.v: true}
	for _, v := range remA {
		run[v] = true
	}
	for _, v := range remB {
		run[v] = true
	}
	outside, inOrder, err := splice(loop, run)
	if err != nil {
		return nil, err
	}
	if len(inOrder) != len(run) {
		return nil, fmt.Errorf("%w: an end face's corner is not one run", decaderr.ErrUnsupported)
	}
	repl := append([]int(nil), rows...)
	if !aFirst {
		for a, b := 0, len(repl)-1; a < b; a, b = a+1, b-1 {
			repl[a], repl[b] = repl[b], repl[a]
		}
	}
	// Each chain's removed part must be on the side the loop says.
	if aFirst && len(remA) > 0 && inOrder[0] != remA[len(remA)-1] {
		return nil, fmt.Errorf("%w: an end face's chain runs the other way", decaderr.ErrUnsupported)
	}
	out2 := make([][]int, 0, len(g.loops))
	for k, lp := range g.loops {
		if k == li {
			out2 = append(out2, append(outside, repl...))
			continue
		}
		out2 = append(out2, lp)
	}
	return out2, nil
}

// replaceEdge swaps the edge's run in face f's loops for the setback edge,
// dropping the removed parts of the two chains.
func (w *work) replaceEdge(f faceLoops, run edgeRun, l line, rem [2][]int, p [2]int) ([][]int, error) {
	set := map[int]bool{l.ends[0].v: true, l.ends[1].v: true}
	for _, v := range run.inner {
		set[v] = true
	}
	for i := range 2 {
		for _, v := range rem[i] {
			set[v] = true
		}
	}
	outside, inOrder, err := splice(f.loops[run.loop], set)
	if err != nil {
		return nil, err
	}
	repl := []int{p[0], p[1]}
	if indexIn(inOrder, l.ends[0].v) > indexIn(inOrder, l.ends[1].v) {
		repl = []int{p[1], p[0]}
	}
	out := make([][]int, 0, len(f.loops))
	for k, lp := range f.loops {
		if k == run.loop {
			out = append(out, append(outside, repl...))
			continue
		}
		out = append(out, lp)
	}
	return out, nil
}

// lineTag is the exact surface of the blend face.
func (w *work) lineTag(l line, convex bool, center, baseA, baseB r3.Vec) blendTag {
	outwardPlane := l.a.n.Add(l.b.n)
	if w.spec.fillet {
		r := w.spec.r
		tag := blendTag{
			surface:  surfacegeom.Cylinder{Origin: center, Axis: l.d, Radius: units.Millimeters(r)},
			reversed: !convex,
		}
		tag.outward = func(p r3.Vec) r3.Vec {
			q := p.Sub(center)
			q = q.Sub(l.d.Scale(q.Dot(l.d)))
			if !convex {
				q = q.Scale(-1)
			}
			return q
		}
		return tag
	}
	chord, _ := baseB.Sub(baseA).Normalize()
	frame, err := r3.NewFrame(baseA, l.d, chord)
	tag := blendTag{surface: surfacegeom.Plane{Frame: frame}}
	if err == nil {
		tag.reversed = frame.N().Dot(outwardPlane) < 0
	}
	tag.outward = func(r3.Vec) r3.Vec { return outwardPlane }
	return tag
}

// polyArea is the area of the planar polygon through ps.
func polyArea(ps ...r3.Vec) float64 {
	sum := r3.Vec{}
	for i, p := range ps {
		sum = sum.Add(p.Cross(ps[(i+1)%len(ps)]))
	}
	return sum.Len() / 2
}

// lineAreas returns the exact area the blend removes from the four faces and
// the exact area of the blend face (design §5.0).
func (w *work) lineAreas(l line, spec blendSpec, convex bool, center, baseA, baseB r3.Vec, rows [2][]int,
	tA, tB float64) (removed, blend float64) {
	m := len(rows[0]) - 1
	pt := func(i, j int) r3.Vec { return w.verts[rows[i][j]] }
	v0, v1 := l.ends[0].exact, l.ends[1].exact
	removed = polyArea(v0, v1, pt(1, 0), pt(0, 0)) + polyArea(v0, v1, pt(1, m), pt(0, m))
	for i := range 2 {
		g := l.ends[i].g
		if spec.fillet {
			corner := spec.r * spec.r * (math.Tan(l.phi/2) - l.phi/2)
			removed += corner / math.Abs(l.d.Dot(g.n))
		} else {
			removed += polyArea(l.ends[i].exact, pt(i, 0), pt(i, m))
		}
	}
	if !spec.fillet {
		chord := baseB.Sub(baseA).Len()
		return removed, chord * (pt(1, 0).Sub(pt(0, 0)).Len() + pt(1, m).Sub(pt(0, m)).Len()) / 2
	}
	g0, g1 := l.ends[0].g, l.ends[1].g
	mu := g0.n.Scale(1 / l.d.Dot(g0.n)).Sub(g1.n.Scale(1 / l.d.Dot(g1.n)))
	length := func(x r3.Vec) float64 {
		return -g1.offset(x)/l.d.Dot(g1.n) + g0.offset(x)/l.d.Dot(g0.n)
	}
	ball := 1.0
	if convex {
		ball = -1
	}
	sum := l.a.n.Add(l.b.n).Scale(-ball)
	r := spec.r
	blend = r*length(center)*l.phi + r*r*math.Tan(l.phi/2)*mu.Dot(sum)
	return removed, blend
}
