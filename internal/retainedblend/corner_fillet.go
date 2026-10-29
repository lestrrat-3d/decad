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

// This file fillets straight convex edges of equal radius that meet at corners of three mutually perpendicular
// retained planes (agent-docs/retained-edge-blend-design.md §4.8). Each corner is read on its own. With one selected
// edge the strip ends on the corner's end face as a lone edge's does. With two the strips end on the plane that bisects
// the two walls (§4.7). With three the strips stop a radius short of the corner and a sphere octant of the same radius
// joins them. Every face at a corner is rebuilt from its boundary loops, which swap the corner for the points the
// rule below gives.

// cornerVertex is one held vertex at which selected edges end.
type cornerVertex struct {
	v     int
	exact r3.Vec
	// n[k] is the outward normal of the corner's k-th face; the corner's edge k runs along u(k) = −n[k], between the
	// other two faces.
	n    [3]r3.Vec
	face [3]*topo.Face
	// edge[k] indexes the selected edge along u(k), or -1; end[k] is the end of that edge at this corner.
	edge [3]int
	end  [3]int
}

func possibleSharedVertex(delta r3.Vec, tol float64) bool {
	bound := 2 * tol
	return delta.X <= bound && delta.X >= -bound &&
		delta.Y <= bound && delta.Y >= -bound &&
		delta.Z <= bound && delta.Z >= -bound
}

func (c *cornerVertex) u(k int) r3.Vec { return c.n[k].Scale(-1) }

func (c *cornerVertex) selected() int {
	n := 0
	for _, e := range c.edge {
		if e >= 0 {
			n++
		}
	}
	return n
}

// cornerPlan is the selected edges and the corners they end at.
type cornerPlan struct {
	lines   []line
	corners map[int]*cornerVertex
}

// planCorners reads the straight edges as fillets that meet at perpendicular corners. It returns false, with no error,
// when the edges are not of that form or share no corner.
func planCorners(edges []*topo.Edge, fp facetedrecord.Payload, faceIndex map[*topo.Face]int) (*cornerPlan, bool, error) {
	if len(edges) < 2 {
		return nil, false, nil
	}
	plan := &cornerPlan{corners: map[int]*cornerVertex{}}
	for _, e := range edges {
		if !looksLikeLine(e) {
			return nil, false, nil
		}
		l, err := readLine(e, fp, faceIndex)
		if err != nil {
			return nil, true, err
		}
		if !l.edgeConvex || math.Abs(l.a.n.Dot(l.b.n)) > 1e-9 {
			return nil, false, nil
		}
		plan.lines = append(plan.lines, l)
	}
	for ei, l := range plan.lines {
		for i := range 2 {
			end := l.ends[i]
			g, ok := retainedPlane(end.face)
			if !ok || math.Abs(g.n.Dot(l.a.n)) > 1e-9 || math.Abs(g.n.Dot(l.b.n)) > 1e-9 {
				return nil, false, nil
			}
			c := plan.corners[end.v]
			if c == nil {
				c = &cornerVertex{v: end.v, exact: end.exact, edge: [3]int{-1, -1, -1}}
				c.face = [3]*topo.Face{end.face, l.faceA, l.faceB}
				c.n = [3]r3.Vec{g.n, l.a.n, l.b.n}
				plan.corners[end.v] = c
			}
			k := -1
			for q, f := range c.face {
				if f == end.face {
					k = q
				}
			}
			if k < 0 || c.edge[k] >= 0 {
				return nil, false, nil
			}
			for _, f := range []*topo.Face{l.faceA, l.faceB} {
				if f == end.face || (f != c.face[0] && f != c.face[1] && f != c.face[2]) {
					return nil, false, nil
				}
			}
			c.edge[k], c.end[k] = ei, i
		}
	}
	shared := false
	for _, c := range plan.corners {
		shared = shared || c.selected() > 1
	}
	return plan, shared, nil
}

// cornerGeom is one edge's strip: the rows along it at its start, the vertices of each row at both ends.
type cornerGeom struct {
	rowsAt       []r3.Vec
	rowIdx       [2][]int
	center       r3.Vec
	baseA, baseB r3.Vec
	// dir[i] is the direction from end i into the edge.
	dir [2]r3.Vec
}

// rewriteCorners fillets the planned edges on a copy of the held mesh and builds the body over it.
func rewriteCorners(ctx context.Context, env topo.Env, fp facetedrecord.Payload, plan *cornerPlan, spec blendSpec,
	op string, ords []int, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	if !spec.fillet || spec.asym != nil {
		return nil, fmt.Errorf("%w: corners are filleted, not chamfered", decaderr.ErrUnsupported)
	}
	r := spec.r
	scale := plan.lines[0].scale
	charge := 0.0
	for _, l := range plan.lines {
		charge = max(charge, l.charge)
	}
	w := newWork(fp, rim{scale: scale, charge: charge}, spec)
	// A sphere patch and many strips add their chord deficits to one area reading; an eighth of the chord tolerance
	// keeps the sum inside what Verify accepts for a box with every edge rounded.
	w.tau /= 8
	slack := chargedSlack(scale, charge)
	margin := w.tau + slack
	for _, l := range plan.lines {
		if l.ends[1].exact.Sub(l.ends[0].exact).Len() <= 2*r+8*margin {
			return nil, fmt.Errorf("%w: a selected edge is not longer than two radii", decaderr.ErrUnsupported)
		}
	}

	var shared []int
	shareVertex := func(p r3.Vec) int {
		tol := 16*slack + w.tau*1e-3
		for _, v := range shared {
			delta := w.verts[v].Sub(p)
			if !possibleSharedVertex(delta, tol) {
				continue
			}
			if delta.Len() <= tol {
				return v
			}
		}
		v := w.vertex(p, w.tau+slack)
		shared = append(shared, v)
		return v
	}

	// The end plane of each end of each edge, by the number of edges selected at its corner.
	endPlane := func(ei, i int) (plane, int) {
		l := plan.lines[ei]
		c := plan.corners[l.ends[i].v]
		switch c.selected() {
		case 1:
			return l.ends[i].g, 1
		case 2:
			var k [2]int
			n := 0
			for q := range 3 {
				if c.edge[q] >= 0 {
					k[n] = q
					n++
				}
			}
			mn, ok := c.n[k[0]].Sub(c.n[k[1]]).Normalize()
			if !ok {
				return plane{}, 0
			}
			return plane{n: mn, o: c.exact}, 2
		default:
			dir := l.d
			if i == 1 {
				dir = dir.Scale(-1)
			}
			return plane{n: dir, o: c.exact.Add(dir.Scale(r))}, 3
		}
	}

	geoms := make([]cornerGeom, len(plan.lines))
	for ei, l := range plan.lines {
		g := &geoms[ei]
		e0 := l.ends[0].exact
		g.baseA = e0.Sub(l.b.n.Scale(r))
		g.baseB = e0.Sub(l.a.n.Scale(r))
		g.center = e0.Sub(l.a.n.Scale(r)).Sub(l.b.n.Scale(r))
		g.dir = [2]r3.Vec{l.d, l.d.Scale(-1)}
		m := max(2, int(math.Ceil(math.Pi/2/(2*math.Acos(1-math.Min(w.tau/r, 1))))))
		for j := 0; j <= m; j++ {
			alpha := math.Pi / 2 * float64(j) / float64(m)
			g.rowsAt = append(g.rowsAt, g.center.Add(l.a.n.Scale(r*math.Cos(alpha))).Add(l.b.n.Scale(r*math.Sin(alpha))))
		}
		g.rowsAt[0], g.rowsAt[m] = g.baseA, g.baseB
		for i := range 2 {
			pl, kind := endPlane(ei, i)
			if kind == 0 {
				return nil, fmt.Errorf("%w: two selected edges meet at a degenerate corner", decaderr.ErrUnsupported)
			}
			g.rowIdx[i] = make([]int, m+1)
			for j, p := range g.rowsAt {
				g.rowIdx[i][j] = shareVertex(setbackPoint(p, l.d, pl))
			}
		}
	}
	rowCount := len(geoms[0].rowsAt) - 1

	// The faces at the corners, and the loop of each rebuilt with the corner's points.
	type faceState struct {
		fl    faceLoops
		pl    plane
		loops [][]int
	}
	states := map[*topo.Face]*faceState{}
	var order []*topo.Face
	for _, c := range plan.corners {
		for _, f := range c.face {
			if _, ok := states[f]; ok {
				continue
			}
			pl, ok := retainedPlane(f)
			if !ok {
				return nil, fmt.Errorf("%w: a corner face is not a retained plane", decaderr.ErrUnsupported)
			}
			fl, err := w.faceLoops(faceIndex[f], &pl)
			if err != nil {
				return nil, err
			}
			st := &faceState{fl: fl, pl: pl}
			for _, loop := range fl.loops {
				st.loops = append(st.loops, append([]int(nil), loop...))
			}
			states[f] = st
			order = append(order, f)
		}
	}
	// A corner's replacement for itself in face k.
	replacement := func(c *cornerVertex, k int) []int {
		i, j := (k+1)%3, (k+2)%3
		selI, selJ, selK := c.edge[i] >= 0, c.edge[j] >= 0, c.edge[k] >= 0
		at := func(a, b float64) int {
			return shareVertex(c.exact.Add(c.u(i).Scale(a * r)).Add(c.u(j).Scale(b * r)))
		}
		switch {
		case selI && selJ:
			return []int{at(1, 1)}
		case selI:
			return []int{at(0, 1)}
		case selJ:
			return []int{at(1, 0)}
		case selK:
			g := geoms[c.edge[k]]
			return append([]int(nil), g.rowIdx[c.end[k]]...)
		}
		return nil
	}
	for _, c := range plan.corners {
		for k, f := range c.face {
			st := states[f]
			seq := replacement(c, k)
			if seq == nil {
				continue
			}
			done := false
			for li, loop := range st.loops {
				q := indexIn(loop, c.v)
				if q < 0 {
					continue
				}
				if done {
					return nil, fmt.Errorf("%w: a corner appears twice on one face", decaderr.ErrUnsupported)
				}
				done = true
				// The neighbours are read from the face's own loop, which no other corner has rewritten.
				orig := st.fl.loops[li]
				qo := indexIn(orig, c.v)
				prev, next := orig[(qo+len(orig)-1)%len(orig)], orig[(qo+1)%len(orig)]
				for _, nb := range []int{prev, next} {
					d := w.verts[nb].Sub(c.exact)
					if d.Len() <= r+8*margin {
						return nil, fmt.Errorf("%w: a face edge at a corner is not longer than the radius", decaderr.ErrUnsupported)
					}
					dn, _ := d.Normalize()
					if math.Abs(dn.Dot(c.u((k+1)%3))) < 1-1e-9 && math.Abs(dn.Dot(c.u((k+2)%3))) < 1-1e-9 {
						return nil, fmt.Errorf("%w: a face leaves a corner along no edge", decaderr.ErrUnsupported)
					}
				}
				if len(seq) > 1 {
					// Start with the end nearer the previous loop vertex.
					toPrev := w.verts[prev].Sub(c.exact)
					if w.verts[seq[0]].Sub(c.exact).Dot(toPrev) < w.verts[seq[len(seq)-1]].Sub(c.exact).Dot(toPrev) {
						for a, b := 0, len(seq)-1; a < b; a, b = a+1, b-1 {
							seq[a], seq[b] = seq[b], seq[a]
						}
					}
				}
				rebuilt := append(append(append([]int(nil), loop[:q]...), seq...), loop[q+1:]...)
				st.loops[li] = rebuilt
			}
			if !done {
				return nil, fmt.Errorf("%w: a corner is not on its face's boundary", decaderr.ErrUnsupported)
			}
		}
	}
	// Each rebuilt face must stay a simple polygon of positive area.
	for _, f := range order {
		st := states[f]
		for li, loop := range st.loops {
			if len(loop) == len(st.fl.loops[li]) && sameLoop(loop, st.fl.loops[li]) {
				continue
			}
			if !w.simpleLoop(loop, st.pl.n, li == 0 || signedLoopArea(w, loop, st.pl.n) > 0) {
				return nil, fmt.Errorf("%w: the blend leaves a face with no simple outline", decaderr.ErrUnsupported)
			}
		}
	}

	// Re-triangulate the faces.
	var touched []faceLoops
	for _, f := range order {
		st := states[f]
		if err := w.retriangulate(st.pl.n, st.pl.o, st.loops, st.fl.group); err != nil {
			return nil, err
		}
		w.removed = append(w.removed, st.fl.facets...)
		touched = append(touched, st.fl)
	}

	// The strips, one group each, then a sphere patch at every corner with three selected edges.
	groups := append([]facetedrecord.Group(nil), fp.Groups...)
	stripFrom := len(w.out)
	var removedTrue, blendTrue float64
	for ei, l := range plan.lines {
		g := &geoms[ei]
		tag := w.lineTag(l, true, g.center, g.baseA, g.baseB)
		group := len(groups)
		groups = append(groups, facetedrecord.Group{
			Origins:  []provenance.Ref{{Producer: env.Ref, Role: fmt.Sprintf("%s(%d)", op, ords[ei])}},
			Source:   facetedrecord.BlendSource(env.Ref, ords[ei]),
			Surface:  tag.surface,
			Retained: true,
			Reversed: tag.reversed,
		})
		for j := range rowCount {
			lo0, lo1 := g.rowIdx[0][j], g.rowIdx[1][j]
			hi0, hi1 := g.rowIdx[0][j+1], g.rowIdx[1][j+1]
			for _, t := range [][3]int{{lo0, lo1, hi1}, {lo0, hi1, hi0}} {
				a, b, c := w.verts[t[0]], w.verts[t[1]], w.verts[t[2]]
				if b.Sub(a).Cross(c.Sub(a)).Dot(tag.outward(a.Add(b).Add(c).Scale(1.0/3))) < 0 {
					t[1], t[2] = t[2], t[1]
				}
				w.emit(t, group)
			}
		}
		// The area terms see each end as the plane the strip stops at.
		la := l
		var flat [2]bool
		for i := range 2 {
			pl, kind := endPlane(ei, i)
			la.ends[i].g = pl
			flat[i] = kind > 1
			if kind == 3 {
				la.ends[i].exact = pl.o
			}
		}
		removed, blend := w.lineAreasAt(la, spec, true, g.center, g.baseA, g.baseB, g.rowIdx, flat)
		removedTrue += removed
		blendTrue += blend
	}
	for _, c := range plan.corners {
		if c.selected() != 3 {
			continue
		}
		patch, err := w.spherePatch(c, r, rowCount, shareVertex, len(groups), env, ords, &groups)
		if err != nil {
			return nil, err
		}
		removedTrue += 3 * r * r
		blendTrue += patch
	}
	w.removedTrue = removedTrue
	for _, f := range touched {
		groups[f.group].HasCentroid = false
		groups[f.group].Centroid = measurement.VecMeasurement{}
	}
	return w.build(ctx, env, groups, stripFrom, blendTag{trueArea: blendTrue}, faceIndex)
}

// spherePatch emits the sphere octant of radius r at a corner where three edges are filleted, as its own group, and
// returns its exact area. Its sides are the end rows of the three strips, which it shares vertex for vertex.
func (w *work) spherePatch(c *cornerVertex, r float64, m int, shareVertex func(r3.Vec) int, group int, env topo.Env,
	ords []int, groups *[]facetedrecord.Group) (float64, error) {
	centre := c.exact.Add(c.u(0).Scale(r)).Add(c.u(1).Scale(r)).Add(c.u(2).Scale(r))
	*groups = append(*groups, facetedrecord.Group{
		Origins: []provenance.Ref{{Producer: env.Ref, Role: "cornerFillet"}},
		Source:  facetedrecord.BlendSource(env.Ref, ords[c.edge[0]]),
		Surface: surfacegeom.Sphere{Center: centre, Radius: units.Millimeters(r)},
		// The sphere's outward side is away from its centre, the body's outside at the corner.
		Retained: true,
	})
	id := make([][]int, m+1)
	for a := 0; a <= m; a++ {
		id[a] = make([]int, m-a+1)
		for b := 0; b <= m-a; b++ {
			cc := m - a - b
			dir := c.n[0].Scale(math.Sin(math.Pi / 2 * float64(a) / float64(m))).
				Add(c.n[1].Scale(math.Sin(math.Pi / 2 * float64(b) / float64(m)))).
				Add(c.n[2].Scale(math.Sin(math.Pi / 2 * float64(cc) / float64(m))))
			d, ok := dir.Normalize()
			if !ok {
				return 0, fmt.Errorf("%w: a corner patch has a zero direction", decaderr.ErrUnsupported)
			}
			id[a][b] = shareVertex(centre.Add(d.Scale(r)))
		}
	}
	emit := func(x, y, z int) {
		a, b, cc := w.verts[x], w.verts[y], w.verts[z]
		mid := a.Add(b).Add(cc).Scale(1.0 / 3)
		if b.Sub(a).Cross(cc.Sub(a)).Dot(mid.Sub(centre)) < 0 {
			y, z = z, y
		}
		w.emit([3]int{x, y, z}, group)
	}
	for a := range m {
		for b := range m - a {
			emit(id[a][b], id[a+1][b], id[a][b+1])
			if b < m-a-1 {
				emit(id[a+1][b], id[a+1][b+1], id[a][b+1])
			}
		}
	}
	return math.Pi * r * r / 2, nil
}

func sameLoop(a, b []int) bool {
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

func signedLoopArea(w *work, loop []int, n r3.Vec) float64 {
	sum := r3.Vec{}
	for i, v := range loop {
		sum = sum.Add(w.verts[v].Cross(w.verts[loop[(i+1)%len(loop)]]))
	}
	return sum.Dot(n) / 2
}

// simpleLoop reports whether the loop, projected along n, has no two non-adjacent sides that cross and winds the way
// an outer loop (outer) or a hole winds seen from outside.
func (w *work) simpleLoop(loop []int, n r3.Vec, outer bool) bool {
	area := signedLoopArea(w, loop, n)
	if (outer && area <= 0) || (!outer && area >= 0) {
		return false
	}
	axis := 0
	if math.Abs(n.Y) > math.Abs(n.X) {
		axis = 1
	}
	if math.Abs(n.Z) > math.Abs([3]float64{n.X, n.Y, n.Z}[axis]) {
		axis = 2
	}
	proj := func(v int) [2]float64 {
		p := w.verts[v]
		c := [3]float64{p.X, p.Y, p.Z}
		switch axis {
		case 0:
			return [2]float64{c[1], c[2]}
		case 1:
			return [2]float64{c[2], c[0]}
		}
		return [2]float64{c[0], c[1]}
	}
	cross := func(o, a, b [2]float64) float64 { return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0]) }
	nLoop := len(loop)
	for i := range nLoop {
		a, b := proj(loop[i]), proj(loop[(i+1)%nLoop])
		for j := i + 2; j < nLoop; j++ {
			if (j+1)%nLoop == i {
				continue
			}
			c, d := proj(loop[j]), proj(loop[(j+1)%nLoop])
			if cross(a, b, c)*cross(a, b, d) < 0 && cross(c, d, a)*cross(c, d, b) < 0 {
				return false
			}
		}
	}
	return true
}
