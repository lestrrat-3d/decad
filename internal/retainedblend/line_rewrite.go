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
	var dirs [2]r3.Vec
	var onChain [2]func(int) bool
	for i := range 2 {
		end := l.ends[i]
		dir, ok := pl.n.Cross(end.g.n).Normalize()
		if !ok {
			return edgeRun{}, fmt.Errorf("%w: a face is parallel to an end face", decaderr.ErrUnsupported)
		}
		dirs[i] = dir
		onChain[i] = func(v int) bool { return distToLine(w.verts[v], end.exact, dir) <= w.beta[v]+slack }
	}
	onE := func(v int) bool { return distToLine(w.verts[v], l.ends[0].exact, l.d) <= w.beta[v]+slack }
	run, err := w.locateBy(f, l.ends[0].v, l.ends[1].v, onE, onChain)
	if err != nil {
		return edgeRun{}, err
	}
	for i := range 2 {
		last := w.verts[run.chain[i][len(run.chain[i])-1]]
		dir := dirs[i]
		if last.Sub(l.ends[i].exact).Dot(dir) < 0 {
			dir = dir.Scale(-1)
		}
		run.dir[i] = dir
	}
	return run, nil
}

// locateBy finds the edge between the held vertices v0 and v1 in face f's
// loops: the loop, the direction along it, the vertices between the ends that
// satisfy onE, and at each end the vertices leaving the corner that satisfy
// onChain. The chain directions are left for the caller.
func (w *work) locateBy(f faceLoops, v0, v1 int, onE func(int) bool, onChain [2]func(int) bool) (edgeRun, error) {
	li, ok := f.loopWith(v0, v1)
	if !ok {
		return edgeRun{}, fmt.Errorf("%w: the edge's ends lie on no one loop of its face", decaderr.ErrUnsupported)
	}
	loop := f.loops[li]
	n := len(loop)
	i0, i1 := indexIn(loop, v0), indexIn(loop, v1)
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
		at, step := i0, -run.step
		if i == 1 {
			at, step = i1, run.step
		}
		run.chain[i] = walk(loop, at, step, onChain[i])
		if len(run.chain[i]) == 0 {
			return edgeRun{}, fmt.Errorf("%w: a face leaves an edge end along no straight run", decaderr.ErrUnsupported)
		}
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
	role string, ord int, faceIndex map[*topo.Face]int) (*topo.Body, error) {
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

	// A face lies to the left of its own boundary, so the way into it from
	// the edge is the face normal crossed with the way the loop runs along the
	// edge. That holds whether the face is convex or reflex at the ends.
	e0 := l.ends[0].exact
	along := func(run edgeRun) r3.Vec {
		if run.step > 0 {
			return l.d
		}
		return l.d.Scale(-1)
	}
	wA, wB := l.a.n.Cross(along(runA)), l.b.n.Cross(along(runB))
	convex := l.b.n.Dot(wA) < 0
	if (l.a.n.Dot(wB) < 0) != convex || l.edgeConvex != convex {
		return nil, fmt.Errorf("%w: the edge's convexity disagrees with its two tags", decaderr.ErrUnsupported)
	}
	// At each end a face either leaves the edge on its own side (convex at the
	// corner: the blend trims it) or turns back past the edge's line (reflex
	// there: only a concave edge's fill, which extends the end face, can end
	// there).
	var kind [2][2]sideKind
	for i := range 2 {
		for f, c := range []struct {
			run edgeRun
			w   r3.Vec
		}{{runA, wA}, {runB, wB}} {
			s := c.run.dir[i].Dot(c.w)
			switch {
			case s > minSine:
				kind[i][f] = trims
			case s < -minSine:
				kind[i][f] = extends
			default:
				return nil, fmt.Errorf("%w: a face leaves an edge end along the edge's own line", decaderr.ErrUnsupported)
			}
		}
		if (kind[i][0] == extends || kind[i][1] == extends) && convex {
			return nil, fmt.Errorf("%w: a convex edge ends where a face is reflex", decaderr.ErrUnsupported)
		}
		if kind[i][0] == extends && kind[i][1] == extends {
			return nil, fmt.Errorf("%w: both faces are reflex at an edge end", decaderr.ErrUnsupported)
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
		for f, side := range []struct {
			run  edgeRun
			rows int
			rem  *[2][]int
		}{{runA, 0, &remA}, {runB, m, &remB}} {
			p := w.verts[rowIdx[i][side.rows]]
			tp := tau(p, end.exact, side.run.dir[i])
			chain := side.run.chain[i]
			if kind[i][f] == extends {
				// The setback point lies beyond the corner, on the line the
				// chain leaves along, where the face carries on.
				if tp >= -margin {
					return nil, fmt.Errorf("%w: a setback point lies on the wrong side of an edge end", decaderr.ErrUnsupported)
				}
				continue
			}
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

	// The faces beside the edge keep clear of the strip: no boundary segment of
	// a face comes within the held bounds of the region the blend rewrites.
	for _, side := range []struct {
		f   faceLoops
		run edgeRun
		pl  plane
		row int
	}{{fA, runA, l.a, 0}, {fB, runB, l.b, m}} {
		exempt := map[int]bool{l.ends[0].v: true, l.ends[1].v: true}
		for _, v := range side.run.inner {
			exempt[v] = true
		}
		for i := range 2 {
			for _, v := range side.run.chain[i] {
				exempt[v] = true
			}
		}
		zone := []r3.Vec{l.ends[0].exact, l.ends[1].exact,
			w.verts[rowIdx[1][side.row]], w.verts[rowIdx[0][side.row]]}
		for _, loop := range side.f.loops {
			for k, v := range loop {
				q := loop[(k+1)%len(loop)]
				if exempt[v] && exempt[q] {
					continue
				}
				if w.nearZone(w.verts[v], w.verts[q], zone, side.pl.n, max(w.beta[v], w.beta[q])+margin) {
					return nil, fmt.Errorf("%w: a face boundary comes within the blend's reach of the edge", decaderr.ErrUnsupported)
				}
			}
		}
	}

	// The end faces: each loses the corner, keeps clear of it, and agrees with
	// the faces beside the edge about the chains.
	var gLoops [2][][]int
	for i := range 2 {
		loops, err := w.endFace(l.ends[i], fG[i], runA.chain[i], runB.chain[i], kind[i], remA[i], remB[i], rowIdx[i], endPoints(rowsAt, l.d, l.ends[i].g), margin, convex)
		if err != nil {
			return nil, err
		}
		gLoops[i] = loops
	}

	// The faces beside the edge swap the edge for its setback edge.
	aLoops, err := w.replaceEdge(fA, runA, l, remA, kindsOf(kind, 0), [2]int{rowIdx[0][0], rowIdx[1][0]})
	if err != nil {
		return nil, err
	}
	bLoops, err := w.replaceEdge(fB, runB, l, remB, kindsOf(kind, 1), [2]int{rowIdx[0][m], rowIdx[1][m]})
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
	w.removedTrue, tag.trueArea = w.lineAreas(l, spec, convex, center, baseA, baseB, rowIdx)

	groups := append([]facetedrecord.Group(nil), fp.Groups...)
	groups = append(groups, facetedrecord.Group{
		Origins:  []provenance.Ref{{Producer: env.Ref, Role: role}},
		Source:   facetedrecord.BlendSource(env.Ref, ord),
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
func (w *work) endFace(end lineEnd, g faceLoops, chainA, chainB []int, kinds [2]sideKind, remA, remB, rows []int,
	reachPts []r3.Vec, margin float64, convex bool) ([][]int, error) {
	li, ok := g.loopWith(end.v)
	if !ok {
		return nil, fmt.Errorf("%w: an end face does not hold the edge's corner", decaderr.ErrUnsupported)
	}
	loop := g.loops[li]
	n := len(loop)
	at := indexIn(loop, end.v)
	prev, next := loop[(at+n-1)%n], loop[(at+1)%n]
	var aFirst bool
	switch {
	case next == chainA[0] && prev == chainB[0]:
		aFirst = false
	case next == chainB[0] && prev == chainA[0]:
		aFirst = true
	default:
		return nil, fmt.Errorf("%w: an end face's chains disagree with the faces beside the edge", decaderr.ErrUnsupported)
	}
	// A convex edge ends on a face that is convex at the corner (the blend
	// removes the corner from it), a concave edge on one that is reflex there
	// (the blend fills the notch), or, when a face beside the edge is reflex
	// there, on one that is convex (the blend's end cap extends it). Nothing
	// else of the face may come near.
	extended := kinds[0] == extends || kinds[1] == extends
	in, out := w.verts[end.v].Sub(w.verts[prev]), w.verts[next].Sub(w.verts[end.v])
	turn := in.Cross(out).Dot(end.g.n)
	if math.Abs(turn) <= margin*in.Len() || (turn > 0) != (convex || extended) {
		return nil, fmt.Errorf("%w: an end face's corner does not match the edge's convexity", decaderr.ErrUnsupported)
	}
	reach := 0.0
	for _, p := range reachPts {
		reach = max(reach, p.Sub(end.exact).Len())
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
	outside, _, err := splice(loop, run)
	if err != nil {
		return nil, err
	}
	// The corner vertex stays beside the end curve when a face beside the edge
	// extends past it.
	seq := append([]int(nil), rows...)
	switch {
	case kinds[0] == extends:
		seq = append([]int{end.v}, seq...)
	case kinds[1] == extends:
		seq = append(seq, end.v)
	}
	repl := seq
	if !aFirst {
		repl = make([]int, len(seq))
		for k, v := range seq {
			repl[len(seq)-1-k] = v
		}
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
// dropping the removed parts of the two chains. A corner vertex stays at an
// end where the face extends past it.
func (w *work) replaceEdge(f faceLoops, run edgeRun, l line, rem [2][]int, kinds [2]sideKind, p [2]int) ([][]int, error) {
	return replaceRun(f, run, [2]int{l.ends[0].v, l.ends[1].v}, rem, kinds, []int{p[0], p[1]})
}

// replaceRun swaps the edge's run in face f's loops, from v[0] to v[1], for
// the vertices seq (listed from end 0 to end 1), dropping the removed parts of
// the two chains. A corner vertex stays at an end where the face extends past
// it.
func replaceRun(f faceLoops, run edgeRun, v [2]int, rem [2][]int, kinds [2]sideKind, seq []int) ([][]int, error) {
	set := map[int]bool{v[0]: true, v[1]: true}
	for _, x := range run.inner {
		set[x] = true
	}
	for i := range 2 {
		for _, x := range rem[i] {
			set[x] = true
		}
	}
	outside, inOrder, err := splice(f.loops[run.loop], set)
	if err != nil {
		return nil, err
	}
	repl := make([]int, 0, len(seq)+2)
	if kinds[0] == extends {
		repl = append(repl, v[0])
	}
	repl = append(repl, seq...)
	if kinds[1] == extends {
		repl = append(repl, v[1])
	}
	if indexIn(inOrder, v[0]) > indexIn(inOrder, v[1]) {
		for a, b := 0, len(repl)-1; a < b; a, b = a+1, b-1 {
			repl[a], repl[b] = repl[b], repl[a]
		}
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

// sideKind says how a face beside the edge meets an end of the edge.
type sideKind int

const (
	// trims is a face that is convex at the corner: the blend cuts the corner
	// out of it.
	trims sideKind = iota
	// extends is a face that is reflex at the corner: the blend's setback edge
	// runs on past the corner along the line the face's own boundary leaves
	// along.
	extends
)

// endPoints are the points a blend's rows reach on the end face g.
func endPoints(rowsAt []r3.Vec, d r3.Vec, g plane) []r3.Vec {
	out := make([]r3.Vec, len(rowsAt))
	for j, p := range rowsAt {
		out[j] = setbackPoint(p, d, g)
	}
	return out
}

// kindsOf is face f's kind at the two ends.
func kindsOf(kind [2][2]sideKind, f int) [2]sideKind { return [2]sideKind{kind[0][f], kind[1][f]} }

// nearZone reports whether the segment ab comes within margin of the planar
// polygon zone, with normal n, or lies inside it.
func (w *work) nearZone(a, b r3.Vec, zone []r3.Vec, n r3.Vec, margin float64) bool {
	u, ok := perpendicular(n)
	if !ok {
		return true
	}
	v := n.Cross(u)
	project := func(p r3.Vec) pt2 {
		q := p.Sub(zone[0])
		return pt2{q.Dot(u), q.Dot(v)}
	}
	ps := make([]pt2, len(zone))
	loop := make([]int, len(zone))
	for i, p := range zone {
		ps[i] = project(p)
		loop[i] = i
	}
	if insidePolygon(ps, loop, project(a)) || insidePolygon(ps, loop, project(b)) {
		return true
	}
	for i := range zone {
		if segmentDistance(a, b, zone[i], zone[(i+1)%len(zone)]) <= margin {
			return true
		}
	}
	return false
}

// segmentDistance is the distance between the segments ab and cd.
func segmentDistance(a, b, c, d r3.Vec) float64 {
	best := math.Min(math.Min(distToSegment(a, c, d), distToSegment(b, c, d)),
		math.Min(distToSegment(c, a, b), distToSegment(d, a, b)))
	u, v, w := b.Sub(a), d.Sub(c), a.Sub(c)
	uu, uv, vv, uw, vw := u.Dot(u), u.Dot(v), v.Dot(v), u.Dot(w), v.Dot(w)
	den := uu*vv - uv*uv
	if den <= 0 {
		return best
	}
	s := (uv*vw - vv*uw) / den
	t := (uu*vw - uv*uw) / den
	if s < 0 || s > 1 || t < 0 || t > 1 {
		return best
	}
	return math.Min(best, a.Add(u.Scale(s)).Sub(c.Add(v.Scale(t))).Len())
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
func (w *work) lineAreas(l line, spec blendSpec, convex bool, center, baseA, baseB r3.Vec,
	rows [2][]int) (removed, blend float64) {
	m := len(rows[0]) - 1
	pt := func(i, j int) r3.Vec { return w.verts[rows[i][j]] }
	v0, v1 := l.ends[0].exact, l.ends[1].exact
	removed = polyArea(v0, v1, pt(1, 0), pt(0, 0)) + polyArea(v0, v1, pt(1, m), pt(0, m))
	for i := range 2 {
		g := l.ends[i].g
		// A convex edge takes the corner out of its end face; a concave edge
		// adds the notch's fill to it.
		corner := polyArea(l.ends[i].exact, pt(i, 0), pt(i, m))
		if spec.fillet {
			corner = spec.r * spec.r * (math.Tan(l.phi/2) - l.phi/2) / math.Abs(l.d.Dot(g.n))
		}
		if !convex {
			corner = -corner
		}
		removed += corner
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
