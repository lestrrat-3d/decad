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
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file fillets a run of straight convex edges that meet end to end on one base face, each corner of the run a
// corner of three mutually perpendicular faces, as the top edges of a box do. Where two filleted edges meet, a rolling
// ball's fillets of equal radius meet in the plane that bisects the two walls through the corner (the mitre plane), so
// the two cylinders end on that plane, share the vertices of their rows there, and trim no end face. The edge below
// the corner, where the two walls meet, loses its top part and is left with one vertex on it where the strips' feet
// meet. A run that closes on itself is a loop around the base face; an open run ends on end faces that the strip
// trims as a single edge's does (line_rewrite.go).

// mitreEdge is one edge of a run: its tags, which of its two faces is the base, and the joint at each end.
type mitreEdge struct {
	l line
	// ord is the edge's place in the selection.
	ord int
	// baseSide is 0 when l.faceA is the run's base face, else 1.
	baseSide int
	// joint[i] is the joint at end i of the edge, or -1 for an end that stays free.
	joint [2]int
}

// mitreJoint is a corner where two edges of the run meet.
type mitreJoint struct {
	corner r3.Vec
	// plane is the mitre plane through the corner.
	plane plane
	// vertex is the held vertex at the corner.
	vertex int
}

// mitreRun is the edges in the order they meet, and the joints between them.
type mitreRun struct {
	base   *topo.Face
	edges  []mitreEdge
	joints []mitreJoint
	closed bool
}

// planMitre reads the straight edges as one run joined by mitres. It returns false, with no error, when the edges are
// not such a run, and an error when they are but a reading fails.
func planMitre(edges []*topo.Edge, fp facetedrecord.Payload, faceIndex map[*topo.Face]int) (*mitreRun, bool, error) {
	if len(edges) < 2 {
		return nil, false, nil
	}
	lines := make([]line, len(edges))
	for i, e := range edges {
		if !looksLikeLine(e) {
			return nil, false, nil
		}
		l, err := readLine(e, fp, faceIndex)
		if err != nil {
			return nil, true, err
		}
		if !l.edgeConvex {
			return nil, false, nil
		}
		lines[i] = l
	}
	tol := 1e-9 * lines[0].scale
	type endRef struct{ edge, end int }
	partner := make([][2]*endRef, len(lines))
	for i := range lines {
		for ei := range 2 {
			for j := range lines {
				if j == i {
					continue
				}
				for ej := range 2 {
					if lines[i].ends[ei].exact.Sub(lines[j].ends[ej].exact).Len() > tol {
						continue
					}
					if partner[i][ei] != nil {
						return nil, false, nil // three edges at one corner
					}
					partner[i][ei] = &endRef{j, ej}
				}
			}
		}
	}
	// The base: the one face every edge borders.
	var base *topo.Face
	for _, f := range []*topo.Face{lines[0].faceA, lines[0].faceB} {
		all := true
		for _, l := range lines[1:] {
			all = all && (l.faceA == f || l.faceB == f)
		}
		if all {
			if base != nil {
				return nil, false, nil
			}
			base = f
		}
	}
	if base == nil {
		return nil, false, nil
	}
	// Walk the run from a free end, or from anywhere when every end is joined.
	start, enter, closed := -1, 0, true
	for i := range lines {
		for ei := range 2 {
			if partner[i][ei] == nil {
				closed = false
				if start < 0 {
					start, enter = i, ei
				}
			}
		}
	}
	if start < 0 {
		start, enter = 0, 0
	}
	type step struct{ edge, enter int }
	var walk []step
	seen := make([]bool, len(lines))
	for cur := start; !seen[cur]; {
		seen[cur] = true
		walk = append(walk, step{cur, enter})
		next := partner[cur][1-enter]
		if next == nil {
			break
		}
		cur, enter = next.edge, next.end
	}
	if len(walk) != len(lines) {
		return nil, false, nil
	}
	run := &mitreRun{base: base, closed: closed}
	for _, s := range walk {
		me := mitreEdge{l: lines[s.edge], ord: s.edge, joint: [2]int{-1, -1}}
		if me.l.faceA != base {
			me.baseSide = 1
		}
		run.edges = append(run.edges, me)
	}
	n := len(run.edges)
	jointCount := n - 1
	if closed {
		jointCount = n
	}
	for k := range jointCount {
		a, b := &run.edges[k], &run.edges[(k+1)%n]
		ea, eb := 1-walk[k].enter, walk[(k+1)%n].enter
		aWall, bWall := a.wall(), b.wall()
		// Three mutually perpendicular faces meet at the corner.
		baseN := mitreBase(a).n
		if math.Abs(baseN.Dot(aWall.n)) > 1e-9 || math.Abs(baseN.Dot(bWall.n)) > 1e-9 ||
			math.Abs(aWall.n.Dot(bWall.n)) > 1e-9 {
			return nil, false, nil
		}
		if a.l.ends[ea].face != b.wallFace() || b.l.ends[eb].face != a.wallFace() {
			return nil, false, nil
		}
		mn, ok := bWall.n.Sub(aWall.n).Normalize()
		if !ok {
			return nil, false, nil
		}
		corner := a.l.ends[ea].exact
		if math.Abs(a.l.d.Dot(mn)) < minSine || math.Abs(b.l.d.Dot(mn)) < minSine {
			return nil, false, nil
		}
		run.joints = append(run.joints, mitreJoint{corner: corner, plane: plane{n: mn, o: corner}, vertex: a.l.ends[ea].v})
		a.joint[ea], b.joint[eb] = k, k
	}
	return run, true, nil
}

// base is the edge's plane on the run's base face, wall its plane on the face beside the base.
func mitreBase(e *mitreEdge) plane {
	if e.baseSide == 0 {
		return e.l.a
	}
	return e.l.b
}

func (e *mitreEdge) wall() plane {
	if e.baseSide == 0 {
		return e.l.b
	}
	return e.l.a
}

func (e *mitreEdge) wallFace() *topo.Face {
	if e.baseSide == 0 {
		return e.l.faceB
	}
	return e.l.faceA
}

// mitreGeom is one edge's blend: the runs along its two faces, the rows of the strip, and the vertices of each row at
// the two ends.
type mitreGeom struct {
	runs         [2]edgeRun // by l.faceA, l.faceB
	wA, wB       r3.Vec
	baseA, baseB r3.Vec
	center       r3.Vec
	rowsAt       []r3.Vec
	rowIdx       [2][]int
	rem          [2][2][]int // [end][side]: the removed parts of the chains, by l.faceA, l.faceB
}

// rewriteMitreRun fillets the run on a copy of the held mesh and builds the body over it.
func rewriteMitreRun(ctx context.Context, env topo.Env, fp facetedrecord.Payload, run *mitreRun, spec blendSpec,
	op string, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	if !spec.fillet || spec.asym != nil {
		return nil, fmt.Errorf("%w: a mitred run is filleted, not chamfered", decaderr.ErrUnsupported)
	}
	scale := run.edges[0].l.scale
	charge := 0.0
	for _, e := range run.edges {
		charge = max(charge, e.l.charge)
	}
	w := newWork(fp, rim{scale: scale, charge: charge}, spec)
	slack := chargedSlack(scale, charge)
	margin := w.tau + slack
	n := len(run.edges)

	// The faces: the base, each edge's wall, and the end face at each free end.
	loadFace := func(f *topo.Face, g *plane) (faceLoops, error) { return w.faceLoops(faceIndex[f], g) }
	basePlane := mitreBase(&run.edges[0])
	fBase, err := loadFace(run.base, &basePlane)
	if err != nil {
		return nil, err
	}
	walls := make([]faceLoops, n)
	used := map[*topo.Face]bool{run.base: true}
	for k := range run.edges {
		e := &run.edges[k]
		if used[e.wallFace()] {
			return nil, fmt.Errorf("%w: two edges of a run share a wall", decaderr.ErrUnsupported)
		}
		used[e.wallFace()] = true
		wp := e.wall()
		if walls[k], err = loadFace(e.wallFace(), &wp); err != nil {
			return nil, err
		}
	}
	// The end faces of the free ends.
	endFaces := map[*topo.Face]*faceLoops{}
	var endOrder []*topo.Face
	for k := range run.edges {
		for i := range 2 {
			if run.edges[k].joint[i] >= 0 {
				continue
			}
			g := run.edges[k].l.ends[i].face
			if used[g] {
				return nil, fmt.Errorf("%w: a free end of a run meets a face the run already uses", decaderr.ErrUnsupported)
			}
			if _, ok := endFaces[g]; ok {
				// The run's two free ends meet one face, as the ends of three sides of a rectangle do: the second
				// end's chain on the base would run to the first end's corner, which the first has already moved.
				return nil, fmt.Errorf("%w: both free ends of a run meet one face", decaderr.ErrUnsupported)
			}
			gp := run.edges[k].l.ends[i].g
			fl, err := loadFace(g, &gp)
			if err != nil {
				return nil, err
			}
			endFaces[g] = &fl
			endOrder = append(endOrder, g)
		}
	}

	geoms := make([]mitreGeom, n)
	mitreVerts := []int{}
	shareVertex := func(p r3.Vec) int {
		for _, v := range mitreVerts {
			if w.verts[v].Sub(p).Len() <= 16*slack+w.tau*1e-3 {
				return v
			}
		}
		v := w.vertex(p, w.tau+slack)
		mitreVerts = append(mitreVerts, v)
		return v
	}

	// Per edge: the runs, the rows, and the vertices of each row at both ends.
	for k := range run.edges {
		e := &run.edges[k]
		l := e.l
		g := &geoms[k]
		fA, fB := fBase, walls[k]
		if e.baseSide == 1 {
			fA, fB = walls[k], fBase
		}
		var err error
		if g.runs[0], err = w.locate(fA, l, l.a); err != nil {
			return nil, err
		}
		if g.runs[1], err = w.locate(fB, l, l.b); err != nil {
			return nil, err
		}
		e0 := l.ends[0].exact
		along := func(r edgeRun) r3.Vec {
			if r.step > 0 {
				return l.d
			}
			return l.d.Scale(-1)
		}
		g.wA, g.wB = l.a.n.Cross(along(g.runs[0])), l.b.n.Cross(along(g.runs[1]))
		if l.b.n.Dot(g.wA) >= 0 || l.a.n.Dot(g.wB) >= 0 {
			return nil, fmt.Errorf("%w: a mitred edge is not convex", decaderr.ErrUnsupported)
		}
		for i := range 2 {
			for f, c := range []struct {
				run edgeRun
				w   r3.Vec
			}{{g.runs[0], g.wA}, {g.runs[1], g.wB}} {
				if c.run.dir[i].Dot(c.w) <= minSine {
					_ = f
					return nil, fmt.Errorf("%w: a mitred edge ends where a face is reflex", decaderr.ErrUnsupported)
				}
			}
		}
		t := spec.r * math.Tan(l.phi/2)
		g.baseA, g.baseB = e0.Add(g.wA.Scale(t)), e0.Add(g.wB.Scale(t))
		g.center = g.baseA.Add(l.a.n.Scale(-spec.r))
		if g.center.Sub(g.baseB.Add(l.b.n.Scale(-spec.r))).Len() > 16*slack*math.Max(1, spec.r) {
			return nil, fmt.Errorf("%w: the blend's tangent points do not share one ball", decaderr.ErrUnsupported)
		}
		a0, a1 := l.a.n, l.b.n // the ball is inside: -ball = +1 for a convex edge
		step := 2 * math.Acos(1-math.Min(w.tau/spec.r, 1))
		m := max(2, int(math.Ceil(l.phi/step)))
		sin := math.Sin(l.phi)
		for j := 0; j <= m; j++ {
			alpha := l.phi * float64(j) / float64(m)
			g.rowsAt = append(g.rowsAt, g.center.Add(a0.Scale(spec.r*math.Sin(l.phi-alpha)/sin)).
				Add(a1.Scale(spec.r*math.Sin(alpha)/sin)))
		}
		g.rowsAt[0], g.rowsAt[m] = g.baseA, g.baseB
		for i := range 2 {
			g.rowIdx[i] = make([]int, m+1)
			for j, p := range g.rowsAt {
				if jt := e.joint[i]; jt >= 0 {
					g.rowIdx[i][j] = shareVertex(setbackPoint(p, l.d, run.joints[jt].plane))
				} else {
					g.rowIdx[i][j] = w.vertex(setbackPoint(p, l.d, l.ends[i].g), w.tau+slack)
				}
			}
		}
	}

	// The chains behind the setback points: each wall at both ends, the base at free ends.
	for k := range run.edges {
		e := &run.edges[k]
		g := &geoms[k]
		m := len(g.rowsAt) - 1
		for i := range 2 {
			for f := range 2 {
				if f == e.baseSide && e.joint[i] >= 0 {
					continue // the neighbouring edge's run is this chain; the joint replaces both
				}
				row := 0
				if f == 1 {
					row = m
				}
				p := w.verts[g.rowIdx[i][row]]
				tp := tau(p, e.l.ends[i].exact, g.runs[f].dir[i])
				chain := g.runs[f].chain[i]
				if tp <= margin {
					return nil, fmt.Errorf("%w: a setback point lies behind an edge end", decaderr.ErrUnsupported)
				}
				last := tau(w.verts[chain[len(chain)-1]], e.l.ends[i].exact, g.runs[f].dir[i])
				if err := clearance(last-tp, w.beta[chain[len(chain)-1]]+margin, "the face beside the edge"); err != nil {
					return nil, err
				}
				for _, c := range chain {
					tc := tau(w.verts[c], e.l.ends[i].exact, g.runs[f].dir[i])
					if math.Abs(tc-tp) <= 4*margin {
						return nil, fmt.Errorf("%w: a setback point falls on a vertex of its face's boundary", decaderr.ErrUnsupported)
					}
					if tc < tp {
						g.rem[i][f] = append(g.rem[i][f], c)
					}
				}
			}
		}
	}

	// The faces beside the edges keep clear of the strips: no boundary segment of the base face or of a wall comes
	// within the held bounds of the region its strip rewrites.
	runVerts := map[int]bool{}
	for k := range run.edges {
		e := &run.edges[k]
		runVerts[e.l.ends[0].v], runVerts[e.l.ends[1].v] = true, true
		for f := range 2 {
			for _, v := range geoms[k].runs[f].inner {
				runVerts[v] = true
			}
			for i := range 2 {
				for _, v := range geoms[k].runs[f].chain[i] {
					runVerts[v] = true
				}
			}
		}
	}
	for k := range run.edges {
		e := &run.edges[k]
		g := &geoms[k]
		m := len(g.rowsAt) - 1
		for _, side := range []struct {
			f   faceLoops
			pl  plane
			row int
		}{{fBase, mitreBase(e), map[int]int{0: 0, 1: m}[e.baseSide]}, {walls[k], e.wall(), map[int]int{0: m, 1: 0}[e.baseSide]}} {
			zone := []r3.Vec{e.l.ends[0].exact, e.l.ends[1].exact, w.verts[g.rowIdx[1][side.row]], w.verts[g.rowIdx[0][side.row]]}
			for _, loop := range side.f.loops {
				for q, v := range loop {
					nx := loop[(q+1)%len(loop)]
					if runVerts[v] && runVerts[nx] {
						continue
					}
					if w.nearZone(w.verts[v], w.verts[nx], zone, side.pl.n, max(w.beta[v], w.beta[nx])+margin) {
						return nil, fmt.Errorf("%w: a face boundary comes within the blend's reach of the edge", decaderr.ErrUnsupported)
					}
				}
			}
		}
	}

	// The base face: the runs of the edges and their joints give way to the setback polygon.
	var baseLoops [][]int
	{
		loop := fBase.loops
		var at []int // the base row vertex at each joint, then the free ends
		_ = at
		set := map[int]bool{}
		for k := range run.edges {
			e := &run.edges[k]
			set[e.l.ends[0].v], set[e.l.ends[1].v] = true, true
			for _, v := range geoms[k].runs[e.baseSide].inner {
				set[v] = true
			}
			for i := range 2 {
				if e.joint[i] < 0 {
					for _, v := range geoms[k].rem[i][e.baseSide] {
						set[v] = true
					}
				}
			}
		}
		// The base row vertex of joint j, from the edge that has it at end i.
		jointRow := make([]int, len(run.joints))
		for k := range run.edges {
			e := &run.edges[k]
			m := len(geoms[k].rowsAt) - 1
			row := 0
			if e.baseSide == 1 {
				row = m
			}
			for i := range 2 {
				if e.joint[i] >= 0 {
					jointRow[e.joint[i]] = geoms[k].rowIdx[i][row]
				}
			}
		}
		_ = loop
		li, ok := fBase.loopWith(run.joints[0].vertex)
		if !ok && len(run.joints) > 0 {
			return nil, fmt.Errorf("%w: the base face does not hold a joint's corner", decaderr.ErrUnsupported)
		}
		_ = li
		var seq []int
		if run.closed {
			// Every vertex of the base face's loop belongs to the run, which the setback polygon replaces.
			outer := fBase.loops[li]
			for _, v := range outer {
				if !set[v] {
					return nil, fmt.Errorf("%w: the base face holds a vertex the closed run does not cover", decaderr.ErrUnsupported)
				}
			}
			order := make([]int, len(run.joints))
			for j := range order {
				order[j] = j
			}
			sort.Slice(order, func(a, b int) bool {
				return indexIn(outer, run.joints[order[a]].vertex) < indexIn(outer, run.joints[order[b]].vertex)
			})
			for _, j := range order {
				seq = append(seq, jointRow[j])
			}
			for k, lp := range fBase.loops {
				if k == li {
					baseLoops = append(baseLoops, seq)
				} else {
					baseLoops = append(baseLoops, lp)
				}
			}
		} else {
			first, last := &run.edges[0], &run.edges[n-1]
			covered := true
			for _, v := range fBase.loops[li] {
				covered = covered && set[v]
			}
			if covered {
				// The unselected edges of the base face have no vertex but the run's: the setback polygon closes on
				// itself, directly from the last free end to the first.
				fi, li2 := 0, 0
				if first.joint[0] >= 0 {
					fi = 1
				}
				if last.joint[0] >= 0 {
					li2 = 1
				}
				rowOf := func(e *mitreEdge, g *mitreGeom) int {
					if e.baseSide == 1 {
						return len(g.rowsAt) - 1
					}
					return 0
				}
				seq = append(seq, geoms[0].rowIdx[fi][rowOf(first, &geoms[0])])
				for j := range run.joints {
					seq = append(seq, jointRow[j])
				}
				seq = append(seq, geoms[n-1].rowIdx[li2][rowOf(last, &geoms[n-1])])
				outer := fBase.loops[li]
				at := indexIn(outer, first.l.ends[fi].v)
				if outer[(at+1)%len(outer)] != run.joints[first.joint[1-fi]].vertex {
					for a, b := 0, len(seq)-1; a < b; a, b = a+1, b-1 {
						seq[a], seq[b] = seq[b], seq[a]
					}
				}
				for k, lp := range fBase.loops {
					if k == li {
						baseLoops = append(baseLoops, seq)
					} else {
						baseLoops = append(baseLoops, lp)
					}
				}
				goto baseDone
			}
			outside, inOrder, err := splice(fBase.loops[li], set)
			if err != nil {
				return nil, err
			}
			fi := 0
			if first.joint[0] >= 0 {
				fi = 1
			}
			li2 := 0
			if last.joint[0] >= 0 {
				li2 = 1
			}
			mFirst, mLast := len(geoms[0].rowsAt)-1, len(geoms[n-1].rowsAt)-1
			rowOf := func(e *mitreEdge, m int) int {
				if e.baseSide == 1 {
					return m
				}
				return 0
			}
			seq = append(seq, geoms[0].rowIdx[fi][rowOf(first, mFirst)])
			for j := range run.joints {
				seq = append(seq, jointRow[j])
			}
			seq = append(seq, geoms[n-1].rowIdx[li2][rowOf(last, mLast)])
			// Orient the setback polygon the way the removed run lay in the loop.
			if indexIn(inOrder, first.l.ends[fi].v) > indexIn(inOrder, last.l.ends[li2].v) {
				for a, b := 0, len(seq)-1; a < b; a, b = a+1, b-1 {
					seq[a], seq[b] = seq[b], seq[a]
				}
			}
			for k, lp := range fBase.loops {
				if k == li {
					baseLoops = append(baseLoops, append(outside, seq...))
				} else {
					baseLoops = append(baseLoops, lp)
				}
			}
		}
	}

baseDone:
	// The walls swap their edges for the setback edges.
	wallLoops := make([][][]int, n)
	for k := range run.edges {
		e := &run.edges[k]
		g := &geoms[k]
		m := len(g.rowsAt) - 1
		row := m
		if e.baseSide == 1 {
			row = 0
		}
		wf := 1 - e.baseSide
		var rems [2][]int
		rems[0], rems[1] = g.rem[0][wf], g.rem[1][wf]
		kinds := [2]sideKind{trims, trims}
		loops, err := replaceRun(walls[k], g.runs[wf], [2]int{e.l.ends[0].v, e.l.ends[1].v}, rems, kinds,
			[]int{g.rowIdx[0][row], g.rowIdx[1][row]})
		if err != nil {
			return nil, err
		}
		wallLoops[k] = loops
	}

	// The end faces of a free end lose the corner as a single edge's do.
	for k := range run.edges {
		e := &run.edges[k]
		g := &geoms[k]
		for i := range 2 {
			if e.joint[i] >= 0 {
				continue
			}
			fl := endFaces[e.l.ends[i].face]
			loops, err := w.endFace(e.l.ends[i], *fl, g.runs[0].chain[i], g.runs[1].chain[i],
				[2]sideKind{trims, trims}, g.rem[i][0], g.rem[i][1], g.rowIdx[i],
				endPoints(g.rowsAt, e.l.d, e.l.ends[i].g), margin, true)
			if err != nil {
				return nil, err
			}
			fl.loops = loops
		}
	}

	// Re-triangulate every face the run rewrites.
	touched := []faceLoops{fBase}
	if err := w.retriangulate(basePlane.n, basePlane.o, baseLoops, fBase.group); err != nil {
		return nil, err
	}
	w.removed = append(w.removed, fBase.facets...)
	for k := range run.edges {
		wp := run.edges[k].wall()
		if err := w.retriangulate(wp.n, wp.o, wallLoops[k], walls[k].group); err != nil {
			return nil, err
		}
		w.removed = append(w.removed, walls[k].facets...)
		touched = append(touched, walls[k])
	}
	for _, face := range endOrder {
		fl := endFaces[face]
		var gp plane
		var corner r3.Vec
		for k := range run.edges {
			for i := range 2 {
				if run.edges[k].joint[i] < 0 && run.edges[k].l.ends[i].face == face {
					gp, corner = run.edges[k].l.ends[i].g, run.edges[k].l.ends[i].exact
				}
			}
		}
		if err := w.retriangulate(gp.n, corner, fl.loops, fl.group); err != nil {
			return nil, err
		}
		w.removed = append(w.removed, fl.facets...)
		touched = append(touched, *fl)
	}

	// The strips, one group each.
	groups := append([]facetedrecord.Group(nil), fp.Groups...)
	stripFrom := len(w.out)
	var removedTrue, blendTrue float64
	for k := range run.edges {
		e := &run.edges[k]
		g := &geoms[k]
		l := e.l
		m := len(g.rowsAt) - 1
		tag := w.lineTag(l, true, g.center, g.baseA, g.baseB)
		group := len(groups)
		ord := e.ord
		groups = append(groups, w.tagGroup(facetedrecord.Group{
			Origins: []provenance.Ref{{Producer: env.Ref, Role: fmt.Sprintf("%s(%d)", op, ord)}},
			Source:  facetedrecord.BlendSource(env.Ref, ord),
		}, tag))
		for j := range m {
			lo0, lo1 := g.rowIdx[0][j], g.rowIdx[1][j]
			hi0, hi1 := g.rowIdx[0][j+1], g.rowIdx[1][j+1]
			for _, t := range [][3]int{{lo0, lo1, hi1}, {lo0, hi1, hi0}} {
				a, b, c := w.verts[t[0]], w.verts[t[1]], w.verts[t[2]]
				centroid := a.Add(b).Add(c).Scale(1.0 / 3)
				if b.Sub(a).Cross(c.Sub(a)).Dot(tag.outward(centroid)) < 0 {
					t[1], t[2] = t[2], t[1]
				}
				w.emit(t, group)
			}
		}
		// The end planes of a joint's side are the mitre plane for the strip's area.
		la := l
		var mitred [2]bool
		for i := range 2 {
			if jt := e.joint[i]; jt >= 0 {
				la.ends[i].g = run.joints[jt].plane
				mitred[i] = true
			}
		}
		removed, blend := w.lineAreasAt(la, spec, true, g.center, g.baseA, g.baseB, g.rowIdx, mitred)
		removedTrue += removed
		blendTrue += blend
	}
	w.removedTrue = removedTrue
	for _, f := range touched {
		groups[f.group].HasCentroid = false
		groups[f.group].Centroid = measurement.VecMeasurement{}
	}
	return w.buildClear(ctx, env, groups, stripFrom, blendTag{trueArea: blendTrue}, faceIndex)
}
