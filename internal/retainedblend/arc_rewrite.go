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

// arcRow is one row of an open-arc blend: its tube position, the circle it
// lies on, and the angles at which that circle meets the two end faces.
type arcRow struct {
	rho, z float64
	theta  [2]float64
}

// colPt is a vertex of a row with its position along the row, in [0, 1].
type colPt struct {
	idx int
	t   float64
}

// arcProfile gives the circle (radius, height) of the blend at parameter s in
// [0, 1], from the plane's setback circle to the wall's.
type arcProfile func(s float64) (rho, z float64)

func (a arcRim) profile(spec blendSpec) (arcProfile, bool) {
	kappa, side := a.kappa, a.side
	if spec.fillet {
		r := spec.r
		rhoC := a.radius + kappa*r
		return func(s float64) (float64, float64) {
			alpha := s * math.Pi / 2
			return rhoC - kappa*r*math.Sin(alpha), side * r * (1 - math.Cos(alpha))
		}, true
	}
	rho1 := a.radius + kappa*spec.dA
	return func(s float64) (float64, float64) {
		return rho1 + s*(a.radius-rho1), s * side * spec.dB
	}, false
}

// rowAngles are the angles at which the circle at parameter s meets each end
// face, each within π of ref.
func (a arcRim) rowAngles(p arcProfile, s float64, ref [2]float64) ([2]float64, error) {
	rho, _ := p(s)
	var out [2]float64
	for i := range 2 {
		roots, ok := a.ringAzimuths(rho, a.ends[i].g, ref[i])
		if !ok {
			return out, fmt.Errorf("%w: a blend circle does not reach an end face", decaderr.ErrUnsupported)
		}
		out[i] = roots[0]
		if math.Abs(roots[1]-ref[i]) < math.Abs(roots[0]-ref[i]) {
			out[i] = roots[1]
		}
	}
	return out, nil
}

// rewriteArc blends one open arc and builds the body over the rewritten mesh.
func rewriteArc(ctx context.Context, env topo.Env, fp facetedrecord.Payload, a arcRim, spec blendSpec,
	role string, ord int, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	if err := a.checkSize(spec); err != nil {
		return nil, err
	}
	w := newWork(fp, a.rim, spec)
	slack := a.slack()
	margin := w.tau + slack
	n := a.n

	var faces [4]faceLoops
	for k, f := range []*topo.Face{a.planeFace, a.wallFace, a.ends[0].face, a.ends[1].face} {
		var err error
		var end *plane
		if k >= 2 {
			end = &a.ends[k-2].g
		}
		if faces[k], err = w.faceLoops(faceIndex[f], end); err != nil {
			return nil, err
		}
	}
	fA, fB, fG := faces[0], faces[1], [2]faceLoops{faces[2], faces[3]}

	// Locate the arc in the plane's loops and in the wall's.
	onArc := func(v int) bool { return a.onRim(w.verts[v], w.beta[v]) }
	var onLineA, onGen [2]func(int) bool
	var tA [2]r3.Vec
	for i := range 2 {
		end := a.ends[i]
		t, ok := n.Cross(end.g.n).Normalize()
		if !ok {
			return nil, fmt.Errorf("%w: an end face is parallel to the plane", decaderr.ErrUnsupported)
		}
		tA[i] = t
		onLineA[i] = func(v int) bool { return distToLine(w.verts[v], end.exact, t) <= w.beta[v]+slack }
		onGen[i] = func(v int) bool { return distToLine(w.verts[v], end.exact, n) <= w.beta[v]+slack }
	}
	runA, err := w.locateBy(fA, a.ends[0].v, a.ends[1].v, onArc, onLineA)
	if err != nil {
		return nil, err
	}
	runB, err := w.locateBy(fB, a.ends[0].v, a.ends[1].v, onArc, onGen)
	if err != nil {
		return nil, err
	}
	for i := range 2 {
		dir := tA[i]
		if w.verts[runA.chain[i][len(runA.chain[i])-1]].Sub(a.ends[i].exact).Dot(dir) < 0 {
			dir = dir.Scale(-1)
		}
		runA.dir[i] = dir
		// The wall's chain must climb the wall, away from the plane.
		toward := w.verts[runB.chain[i][len(runB.chain[i])-1]].Sub(a.ends[i].exact).Dot(n)
		if toward*a.side <= margin {
			return nil, fmt.Errorf("%w: the wall's generator at an arc end leaves the wrong way", decaderr.ErrUnsupported)
		}
	}

	// The profile and its rows.
	profile, isFillet := a.profile(spec)
	ta, tb := spec.planeSetback(), spec.wallSetback()
	rhoA := a.radius + a.kappa*ta
	ref := [2]float64{a.theta0, a.theta0 + a.span}
	rows := []arcRow{}
	var m int
	if isFillet {
		step := 2 * math.Acos(1-math.Min(w.tau/spec.r, 1))
		m = max(2, int(math.Ceil(math.Pi/2/step)))
	} else {
		m = a.chamferRows(profile, w.tau)
	}
	prev := ref
	for j := 0; j <= m; j++ {
		s := float64(j) / float64(m)
		th, err := a.rowAngles(profile, s, prev)
		if err != nil {
			return nil, err
		}
		rho, z := profile(s)
		rows = append(rows, arcRow{rho: rho, z: z, theta: th})
		prev = th
	}
	if !(rows[0].theta[1]-rows[0].theta[0] > minSine) {
		return nil, fmt.Errorf("%w: the plane's setback arc has no length", decaderr.ErrDegenerate)
	}

	// The plane: the setback points, and the clearance of its chains.
	var remA [2][]int
	for i := range 2 {
		end := a.ends[i]
		p := a.atTheta(rows[0].rho, rows[0].theta[i], 0)
		tp := tau(p, end.exact, runA.dir[i])
		chain := runA.chain[i]
		last := tau(w.verts[chain[len(chain)-1]], end.exact, runA.dir[i])
		if tp <= margin {
			return nil, fmt.Errorf("%w: a setback point lies behind an arc end", decaderr.ErrUnsupported)
		}
		if err := clearance(last-tp, w.beta[chain[len(chain)-1]]+margin, "the plane beside the arc"); err != nil {
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

	// The wall is cut at the setback height.
	generator := map[int]bool{}
	for i := range 2 {
		for _, v := range runB.chain[i] {
			generator[v] = true
		}
	}
	cutIDs, remB, pB, err := w.clipArcWall(fB, a, tb, generator, runB, margin)
	if err != nil {
		return nil, err
	}

	// The plane's row, the wall's row, and the rows between them.
	colsA := w.arcRow(a, rows[0], 0, rhoA)
	bound := w.tau + slack
	rowCols := make([][]colPt, m+1)
	rowCols[0] = colsA
	for j := 1; j < m; j++ {
		rowCols[j] = w.interiorRow(a, rows[j], bound)
	}
	rowCols[m] = make([]colPt, len(cutIDs))
	span := rows[m].theta
	for k, id := range cutIDs {
		th := a.theta(w.verts[id], (span[0]+span[1])/2)
		t := (th - span[0]) / (span[1] - span[0])
		rowCols[m][k] = colPt{id, t}
	}
	rowCols[m][0].t, rowCols[m][len(cutIDs)-1].t = 0, 1
	_ = pB

	// The plane's loops: the arc gives way to the setback arc.
	seqA := make([]int, len(colsA))
	for k, c := range colsA {
		seqA[k] = c.idx
	}
	aLoops, err := replaceRun(fA, runA, [2]int{a.ends[0].v, a.ends[1].v}, remA, [2]sideKind{trims, trims}, seqA)
	if err != nil {
		return nil, err
	}
	// The plane keeps clear of the strip.
	{
		exempt := map[int]bool{a.ends[0].v: true, a.ends[1].v: true}
		for _, v := range runA.inner {
			exempt[v] = true
		}
		for i := range 2 {
			for _, v := range runA.chain[i] {
				exempt[v] = true
			}
		}
		for _, loop := range fA.loops {
			for k, v := range loop {
				q := loop[(k+1)%len(loop)]
				if exempt[v] && exempt[q] {
					continue
				}
				lo, hi := w.segmentRadialRange(w.verts[v], w.verts[q])
				if hi > math.Min(a.radius, rhoA)-margin-w.beta[v] && lo < math.Max(a.radius, rhoA)+margin+w.beta[v] &&
					w.nearArcZone(a, w.verts[v], w.verts[q], rhoA, rows[0].theta, margin+w.beta[v]) {
					return nil, fmt.Errorf("%w: a face boundary comes within the blend's reach of the arc", decaderr.ErrUnsupported)
				}
			}
		}
	}

	// The end faces.
	var gLoops [2][][]int
	for i := range 2 {
		endIDs := make([]int, m+1)
		reach := make([]r3.Vec, m+1)
		for j := range endIDs {
			c := rowCols[j][0]
			if i == 1 {
				c = rowCols[j][len(rowCols[j])-1]
			}
			endIDs[j] = c.idx
			reach[j] = w.verts[c.idx]
		}
		loops, err := w.endFace(a.ends[i], fG[i], runA.chain[i], runB.chain[i], [2]sideKind{trims, trims},
			remA[i], remB[i], endIDs, reach, margin, a.convex)
		if err != nil {
			return nil, err
		}
		gLoops[i] = loops
	}
	for _, f := range []struct {
		loops [][]int
		n, o  r3.Vec
		fl    faceLoops
	}{{aLoops, n, a.center, fA}, {gLoops[0], a.ends[0].g.n, a.ends[0].exact, fG[0]},
		{gLoops[1], a.ends[1].g.n, a.ends[1].exact, fG[1]}} {
		if err := w.retriangulate(f.n, f.o, f.loops, f.fl.group); err != nil {
			return nil, err
		}
		w.removed = append(w.removed, f.fl.facets...)
	}

	// The strip, and its tag.
	tag := a.arcTag(spec)
	blendGroup := len(fp.Groups)
	stripFrom := len(w.out)
	for j := range m {
		for _, t := range zipOpen(rowCols[j], rowCols[j+1]) {
			va, vb, vc := w.verts[t[0]], w.verts[t[1]], w.verts[t[2]]
			centroid := va.Add(vb).Add(vc).Scale(1.0 / 3)
			if vb.Sub(va).Cross(vc.Sub(va)).Dot(tag.outward(centroid)) < 0 {
				t[1], t[2] = t[2], t[1]
			}
			w.emit(t, blendGroup)
		}
	}
	w.removedTrue, tag.trueArea = a.arcAreas(profile, rows, tb)

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

// chamferRows is the number of rows a chamfer's cone needs for its chords to
// follow the section the end faces cut in it to within tau.
func (a arcRim) chamferRows(p arcProfile, tau float64) int {
	for m := 2; m < 64; m *= 2 {
		worst := 0.0
		for i := range 2 {
			for j := range m {
				s0, s1, sm := float64(j)/float64(m), float64(j+1)/float64(m), (float64(j)+0.5)/float64(m)
				pt := func(s float64) (r3.Vec, bool) {
					rho, z := p(s)
					roots, ok := a.ringAzimuths(rho, a.ends[i].g, a.theta0+float64(i)*a.span)
					if !ok {
						return r3.Vec{}, false
					}
					return a.atTheta(rho, roots[0], z), true
				}
				p0, ok0 := pt(s0)
				p1, ok1 := pt(s1)
				pm, okm := pt(sm)
				if !ok0 || !ok1 || !okm {
					return m
				}
				worst = max(worst, distToSegment(pm, p0, p1))
			}
		}
		if worst <= tau {
			return m
		}
	}
	return 64
}

// arcRow builds the plane's row: the setback points and the points between
// them along the circle of radius rho.
func (w *work) arcRow(a arcRim, row arcRow, z, rho float64) []colPt {
	bound := w.tau + a.slack()
	t0, t1 := row.theta[0], row.theta[1]
	step := 2 * math.Acos(1-math.Min(w.tau/math.Max(rho, w.tau), 1))
	k := max(1, int(math.Ceil((t1-t0)/step)))
	out := make([]colPt, 0, k+1)
	for i := 0; i <= k; i++ {
		f := float64(i) / float64(k)
		out = append(out, colPt{w.vertex(a.atTheta(rho, t0+(t1-t0)*f, z), bound), f})
	}
	return out
}

// interiorRow builds a row between the plane's and the wall's: its end points
// on the end faces and the points between them.
func (w *work) interiorRow(a arcRim, row arcRow, bound float64) []colPt {
	t0, t1 := row.theta[0], row.theta[1]
	step := 2 * math.Acos(1-math.Min(w.tau/math.Max(row.rho, w.tau), 1))
	k := max(1, int(math.Ceil((t1-t0)/step)))
	out := make([]colPt, 0, k+1)
	for i := 0; i <= k; i++ {
		f := float64(i) / float64(k)
		out = append(out, colPt{w.vertex(a.atTheta(row.rho, t0+(t1-t0)*f, row.z), bound), f})
	}
	return out
}

// zipOpen joins two open rows into a band of triangles by advancing whichever
// row's next vertex lies earlier along its row.
func zipOpen(a, b []colPt) [][3]int {
	var out [][3]int
	i, j := 0, 0
	for i+1 < len(a) || j+1 < len(b) {
		if i+1 < len(a) && (j+1 >= len(b) || a[i+1].t <= b[j+1].t) {
			out = append(out, [3]int{a[i].idx, a[i+1].idx, b[j].idx})
			i++
			continue
		}
		out = append(out, [3]int{a[i].idx, b[j+1].idx, b[j].idx})
		j++
	}
	return out
}

// nearArcZone reports whether segment pq comes within reach of the annular
// sector the plane loses: between the rim and the setback circle, over the
// angles the setback arc spans.
func (w *work) nearArcZone(a arcRim, p, q r3.Vec, rhoA float64, span [2]float64, reach float64) bool {
	lo, hi := math.Min(a.radius, rhoA)-reach, math.Max(a.radius, rhoA)+reach
	const samples = 64
	for i := 0; i <= samples; i++ {
		x := p.Add(q.Sub(p).Scale(float64(i) / samples))
		rho := a.radial(x)
		if rho < lo || rho > hi {
			continue
		}
		th := a.theta(x, (span[0]+span[1])/2)
		if th >= span[0]-reach/math.Max(rho, reach) && th <= span[1]+reach/math.Max(rho, reach) {
			return true
		}
	}
	return false
}

// clipArcWall cuts the wall at the setback height tb below the rim, keeping the
// part beyond it. generator holds the vertices of the wall's lines along the
// end faces, which sit inside the removed zone legitimately. It returns the
// cut vertices in order of angle, the removed part of each end's generator
// chain and the cut vertex on the chain edge that crosses the setback height.
func (w *work) clipArcWall(f faceLoops, a arcRim, tb float64, generator map[int]bool, run edgeRun,
	margin float64) ([]int, [2][]int, [2]int, error) {
	slack := a.slack()
	g := map[int]float64{}
	for _, facet := range f.facets {
		for _, v := range w.fp.Tris[facet] {
			if _, seen := g[v]; seen {
				continue
			}
			p := w.verts[v]
			if a.onRim(p, w.beta[v]) {
				g[v] = -tb
				continue
			}
			gv := a.side*a.height(p) - tb
			if generator[v] && gv < -margin-w.beta[v] {
				g[v] = gv
				continue
			}
			if err := clearance(gv, w.beta[v]+slack+w.tau, "the wall"); err != nil {
				return nil, [2][]int{}, [2]int{}, err
			}
			g[v] = gv
		}
	}
	cut := func(x, y int) int { // x removed, y kept
		key := [2]int{min(x, y), max(x, y)}
		if id, ok := w.cuts[key]; ok {
			return id
		}
		t := g[x] / (g[x] - g[y])
		p := w.verts[x].Add(w.verts[y].Sub(w.verts[x]).Scale(t))
		id := w.vertex(p, max(w.beta[x], w.beta[y])+slack+w.tau)
		w.cuts[key] = id
		return id
	}
	for _, facet := range f.facets {
		t := w.fp.Tris[facet]
		var above []int
		for k, v := range t {
			if g[v] > 0 {
				above = append(above, k)
			}
		}
		switch len(above) {
		case 0:
		case 3:
			w.emit(t, f.group)
		case 1:
			x := above[0]
			p, q, r := t[x], t[(x+1)%3], t[(x+2)%3]
			w.emit([3]int{p, cut(q, p), cut(r, p)}, f.group)
		case 2:
			z := 0
			for k := range 3 {
				if g[t[k]] <= 0 {
					z = k
				}
			}
			x, y, below := t[(z+1)%3], t[(z+2)%3], t[z]
			cyz, czx := cut(below, y), cut(below, x)
			w.emit([3]int{x, y, cyz}, f.group)
			w.emit([3]int{x, cyz, czx}, f.group)
		}
	}
	w.removed = append(w.removed, f.facets...)

	var rem [2][]int
	var pb [2]int
	for i := range 2 {
		seq := append([]int{a.ends[i].v}, run.chain[i]...)
		crossed := false
		for k := 1; k < len(seq); k++ {
			if g[seq[k-1]] <= 0 && g[seq[k]] > 0 {
				id, ok := w.cuts[[2]int{min(seq[k-1], seq[k]), max(seq[k-1], seq[k])}]
				if !ok {
					return nil, rem, pb, fmt.Errorf("%w: the wall's generator edge at the setback height was not cut", decaderr.ErrUnsupported)
				}
				pb[i], crossed = id, true
				break
			}
			if g[seq[k]] > 0 {
				break
			}
			rem[i] = append(rem[i], seq[k])
		}
		if !crossed {
			return nil, rem, pb, fmt.Errorf("%w: the wall's generator ends before the blend's setback height", decaderr.ErrDegenerate)
		}
	}
	ids := make([]int, 0, len(w.cuts))
	for _, id := range w.cuts {
		ids = append(ids, id)
	}
	mid := a.theta0 + a.span/2
	sort.Slice(ids, func(x, y int) bool {
		return a.theta(w.verts[ids[x]], mid) < a.theta(w.verts[ids[y]], mid)
	})
	if len(ids) < 2 || ids[0] != pb[0] || ids[len(ids)-1] != pb[1] {
		return nil, rem, pb, fmt.Errorf("%w: the wall's cut does not run from one end face to the other", decaderr.ErrUnsupported)
	}
	return ids, rem, pb, nil
}

// arcTag is the exact surface of the blend face.
func (a arcRim) arcTag(spec blendSpec) blendTag {
	size := spec.r
	if !spec.fillet {
		size = max(spec.dA, spec.dB)
	}
	w := &work{r: a.rim, spec: spec, tau: math.Ldexp(size, -10)}
	if spec.fillet {
		return w.torusTag()
	}
	return w.coneTag()
}

// arcAreas returns the exact area the blend removes from the four faces
// (signed: a concave blend adds to the end faces) and the exact area of the
// blend face.
func (a arcRim) arcAreas(p arcProfile, rows []arcRow, tb float64) (removed, blend float64) {
	const samples = 4000
	ref0 := [2]float64{a.theta0, a.theta0 + a.span}
	for i := range 2 {
		pts := []r3.Vec{a.ends[i].exact}
		ref := ref0[i]
		for s := 0; s <= samples; s++ {
			rho, z := p(float64(s) / samples)
			roots, ok := a.ringAzimuths(rho, a.ends[i].g, ref)
			if !ok {
				continue
			}
			t := roots[0]
			if math.Abs(roots[1]-ref) < math.Abs(roots[0]-ref) {
				t = roots[1]
			}
			ref = t
			pts = append(pts, a.atTheta(rho, t, z))
		}
		corner := polyArea(pts...)
		if !a.convex {
			corner = -corner
		}
		removed += corner
	}
	// The plane loses the region between the rim and the setback arc, bounded
	// by the two lines along the end faces: Green's theorem over the two arcs
	// and the two lines.
	rhoA := rows[0].rho
	t0, t1 := rows[0].theta[0], rows[0].theta[1]
	xy := func(q r3.Vec) (float64, float64) {
		d := q.Sub(a.center)
		return d.Dot(a.u), d.Dot(a.w)
	}
	line := func(q0, q1 r3.Vec) float64 {
		x0, y0 := xy(q0)
		x1, y1 := xy(q1)
		return (x0*y1 - x1*y0) / 2
	}
	region := a.radius*a.radius*a.sign*a.span/2 +
		line(a.ends[1].exact, a.atTheta(rhoA, t1, 0)) -
		rhoA*rhoA*a.sign*(t1-t0)/2 +
		line(a.atTheta(rhoA, t0, 0), a.ends[0].exact)
	removed += math.Abs(region)
	// The wall loses a band whose sides are generators.
	removed += a.radius * a.span * tb
	prev := 0.0
	for k := 1; k <= samples; k++ {
		s := float64(k) / samples
		sm := (prev + s) / 2
		rho0, z0 := p(prev)
		rho1, z1 := p(s)
		rhoM, _ := p(sm)
		ang, err := a.rowAngles(p, sm, ref0)
		prev = s
		if err != nil {
			continue
		}
		blend += rhoM * (ang[1] - ang[0]) * math.Hypot(rho1-rho0, z1-z0)
	}
	return removed, blend
}
