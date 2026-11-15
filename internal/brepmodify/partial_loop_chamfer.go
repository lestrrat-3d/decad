package brepmodify

import (
	"context"
	"errors"
	"fmt"
	"maps"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/triangulation"
)

// PartialLoopChamfer chamfers part of the boundary loops of a planar cap, the way a CAD chamfer of a few edges of a
// cap does. The selection in each loop must be whole runs: maximal chains of edges that continue one another
// tangentially, each ending at a sharp corner where the next edge is either not selected or starts another run. A run
// whose end meets a selected edge across a sharp corner is mitred with it, the same as at a corner of a whole loop.
//
// Every wall beside a selected edge is a plane normal to the cap or a cylinder with its axis normal to it, so each
// wall descends from the cap and the chamfer removes material. A run is removed by a cutter that holds exactly the
// material at or above its bevel: the region within t of the run, measured inside the cap, above the plane that
// leaves the wall at depth d and the cap at d. The cutter reaches past each end of the run along the run's tangent, so
// the bevel ends on the wall beside the next edge as a chamfer's bevel does. That is exact only where the extension
// lies outside the material, which holds at a convex corner that turns by 90 degrees or more; a run that ends at any
// other corner is refused rather than guessed.
//
// A cutter is a polyhedron held exactly as a mesh solid, as RetainedLoopChamfer's is; its facets sit within
// cutterSagitta of an arc. The result is a Boolean Cut: faceted, with the Cut's own readings.
//
// It takes a straight prism receiver only. It reports false, leaving the other routes in charge, for any receiver or selection it does not recognise,
// including a selection that holds a whole loop. A recognised selection that cannot be cut is refused as
// ErrUnsupported or ErrDegenerate.
func PartialLoopChamfer(ctx context.Context, env topo.Env, hooks Hooks, body *topo.Body, edges []*topo.Edge,
	d, dDelta float64, asym *Asymmetric) (Result, bool, error) {
	if !body.Solid || asym != nil || dDelta != 0 || !(d > 0) || len(edges) == 0 || hooks.MeshSolid == nil ||
		hooks.Boolean == nil {
		return Result{}, false, nil
	}
	// Only a straight prism takes this route: every other receiver has its own exact gates, which a cutter would bypass.
	if _, isPrism := body.Payload.(prismrecord.Payload); !isPrism {
		return Result{}, false, nil
	}
	runs, ok, err := readChamferRuns(body, edges, true)
	if !ok {
		return Result{}, false, nil
	}
	if err != nil {
		return Result{}, true, err
	}
	for _, r := range runs {
		if err := r.fits(d); err != nil {
			return Result{}, true, err
		}
	}
	var last error
	for _, over := range partialOvershoots {
		out, used, err := cutRuns(ctx, env, hooks, body, runs, d, over*d)
		if err == nil {
			return Result{Body: out, Span: used}, true, nil
		}
		last = err
		if ctx.Err() != nil || !isUnsupported(err) {
			break
		}
	}
	return Result{}, true, last
}

// partialSagitta is how far a chord of a partial chamfer's cutter arc may lie inside its circle, in millimetres. It is
// finer than the loop cutter's, since the receiver is held exactly and the Cut's chord tolerance follows the pair's size.
const partialSagitta = 0.0002

// partialOvershoots are how far, in setbacks, a cutter reaches above the cap and outside the wall, tried in turn
// while the Boolean refuses a cutter as unsupported.
var partialOvershoots = []float64{0.5, 0.35, 0.7}

func isUnsupported(err error) bool { return err != nil && errors.Is(err, decaderr.ErrUnsupported) }

// chamferRun is a chain of selected segments of a loop: the loop's segments first through first+count-1, taken
// cyclically.
type chamferRun struct {
	loop         *chamferLoop
	first, count int
}

func (r chamferRun) seg(i int) loopSeg { return r.loop.segs[(r.first+i)%len(r.loop.segs)] }

func (r chamferRun) before() loopSeg {
	n := len(r.loop.segs)
	return r.loop.segs[(r.first+n-1)%n]
}

func (r chamferRun) after() loopSeg { return r.loop.segs[(r.first+r.count)%len(r.loop.segs)] }

func (r chamferRun) fits(d float64) error { return r.loop.fits(d) }

// readChamferRuns reads edges as whole runs of loops of planar faces. ok is false when the selection is not one this
// route recognises. A recognised selection that cannot be chamfered returns its refusal.
func readChamferRuns(body *topo.Body, edges []*topo.Edge, exact bool) (runs []chamferRun, ok bool, err error) {
	selected := make(map[*topo.Edge]bool, len(edges))
	for _, e := range edges {
		selected[e] = true
	}
	// Every selected edge must lie in a loop of one of the prism's two caps.
	type candidate struct {
		face  *topo.Face
		plane surfacegeom.Plane
	}
	var cands []candidate
	caps := CapsOf(body)
	for _, f := range []*topo.Face{caps.Start, caps.End} {
		if f == nil {
			continue
		}
		if plane, isPlane := f.Surface.(surfacegeom.Plane); isPlane && (exact || retainedTag(f)) {
			cands = append(cands, candidate{f, plane})
		}
	}
	count := func(c candidate, left map[*topo.Edge]bool) int {
		n := 0
		for _, loop := range c.face.Loops {
			for _, ce := range loop.CoEdges {
				if left[ce.Edge] {
					n++
				}
			}
		}
		return n
	}
	left := maps.Clone(selected)
	for len(left) > 0 {
		best, bestN := -1, 0
		for i, c := range cands {
			if n := count(c, left); n > bestN {
				best, bestN = i, n
			}
		}
		if best < 0 {
			return nil, false, nil
		}
		c := cands[best]
		cands = slices.Delete(cands, best, best+1)
		for _, loop := range c.face.Loops {
			n := 0
			for _, ce := range loop.CoEdges {
				if left[ce.Edge] {
					n++
				}
			}
			if n == 0 {
				continue
			}
			if n == len(loop.CoEdges) {
				// A whole loop belongs to the loop routes.
				return nil, false, nil
			}
			l, ok := readChamferLoop(c.face, c.plane, loop, exact)
			if !ok || l.circle || len(l.segs) != len(loop.CoEdges) {
				return nil, false, nil
			}
			sel := make([]bool, len(loop.CoEdges))
			for i, ce := range loop.CoEdges {
				sel[i] = left[ce.Edge]
			}
			rs, err := l.runs(sel)
			if err != nil {
				// A selection this route cannot take keeps the refusal the exact routes gave it.
				return nil, false, nil
			}
			runs = append(runs, rs...)
			for _, ce := range loop.CoEdges {
				delete(left, ce.Edge)
			}
		}
	}
	return runs, len(runs) > 0, nil
}

// continues reports whether the walk leaves the end of prev along the tangent it arrived on: the junction is smooth.
func continues(prev, next loopSeg) bool {
	v := next.p0
	tin, tout := tangent(prev, v), tangent(next, v)
	return math.Abs(tin.cross(tout)) < 1e-7 && tin.dot(tout) > 0
}

// runs splits the selected segments sel of the loop into runs, and checks each run's two ends.
func (l *chamferLoop) runs(sel []bool) ([]chamferRun, error) {
	n := len(l.segs)
	// A run starts at a selected segment whose predecessor is not selected, or is selected but met at a sharp corner.
	var out []chamferRun
	for i := range n {
		if !sel[i] {
			continue
		}
		prev := (i + n - 1) % n
		if sel[prev] && continues(l.segs[prev], l.segs[i]) {
			continue
		}
		count := 1
		for sel[(i+count)%n] && continues(l.segs[(i+count-1)%n], l.segs[(i+count)%n]) && count < n {
			count++
		}
		out = append(out, chamferRun{loop: l, first: i, count: count})
	}
	for _, r := range out {
		// A selected neighbour that continues the run is part of it, so one left over means the selection stops
		// short of a smooth chain, which no CAD chamfer builds.
		if next := (r.first + r.count) % n; !sel[next] && continues(l.segs[(next+n-1)%n], l.segs[next]) {
			return nil, fmt.Errorf(`%w: the selection stops inside a tangent chain of the cap loop; select every edge that continues the run`,
				decaderr.ErrUnsupported)
		}
		if prev := (r.first + n - 1) % n; !sel[prev] && continues(l.segs[prev], l.segs[r.first]) {
			return nil, fmt.Errorf(`%w: the selection stops inside a tangent chain of the cap loop; select every edge that continues the run`,
				decaderr.ErrUnsupported)
		}
	}
	if total := func() int {
		c := 0
		for _, r := range out {
			c += r.count
		}
		return c
	}(); total == n {
		return nil, fmt.Errorf(`%w: the selection chains every edge of a cap loop through sharp corners; select the loop whole`,
			decaderr.ErrUnsupported)
	}
	for _, r := range out {
		if err := r.checkEnds(); err != nil {
			return nil, err
		}
	}
	return out, nil
}

// polygon samples the loop as a closed polygon.
func (l *chamferLoop) polygon() []pt2 {
	var out []pt2
	for _, s := range l.segs {
		out = append(out, s.p0)
		if !s.arc {
			continue
		}
		a0 := math.Atan2(s.p0.y-s.c.y, s.p0.x-s.c.x)
		delta := s.sweep()
		if !s.ccw {
			delta = -delta
		}
		const cells = 32
		for k := 1; k < cells; k++ {
			phi := a0 + delta*float64(k)/cells
			out = append(out, pt2{s.c.x + s.r*math.Cos(phi), s.c.y + s.r*math.Sin(phi)})
		}
	}
	return out
}

// contains reports whether p lies inside the polygon, by crossings.
func contains(poly []pt2, p pt2) bool {
	in := false
	for i, a := range poly {
		b := poly[(i+1)%len(poly)]
		if (a.y > p.y) != (b.y > p.y) && p.x < a.x+(p.y-a.y)*(b.x-a.x)/(b.y-a.y) {
			in = !in
		}
	}
	return in
}

// partialExtension is how far a cutter reaches past the end of its run along the tangent, in setbacks.
const partialExtension = 1.0

// checkEnds refuses a run whose end the cutter's tangent extension cannot serve: the next edge must leave at a
// convex corner that turns by 90 degrees or more, so that the extension lies outside the material.
func (r chamferRun) checkEnds() error {
	l := r.loop
	end := func(prev, next loopSeg) error {
		v := next.p0
		tin, tout := tangent(prev, v), tangent(next, v)
		cross := tin.cross(tout)
		if (cross > 0) != l.materialLeft || math.Abs(cross) < 1e-9 || tin.dot(tout) > 1e-9 {
			return fmt.Errorf(`%w: a chamfered run ends at a corner that is not convex and turned by 90 degrees or more; the bevel's end there is not built`,
				decaderr.ErrUnsupported)
		}
		return nil
	}
	if err := end(r.seg(r.count-1), r.after()); err != nil {
		return err
	}
	return end(r.before(), r.seg(0))
}

// checkExtension samples the region each cutter end reaches past its run, to the depth the cutter cuts, and refuses a
// run whose extension holds material of the cap.
func (r chamferRun) checkExtension(d, reach float64) error {
	l := r.loop
	poly := l.polygon()
	probe := func(v, tan, in pt2) error {
		for _, s := range []float64{0.2, 0.4, 0.6, 0.8, 1} {
			for _, u := range []float64{0.05, 0.25, 0.5, 0.75, 1} {
				p := v.add(tan.scale(s * partialExtension * d)).add(in.scale(u * reach))
				if contains(poly, p) == l.outer {
					return fmt.Errorf(`%w: material of the cap lies beside a chamfered run's end, where the bevel would cut it`,
						decaderr.ErrUnsupported)
				}
			}
		}
		return nil
	}
	first, last := r.seg(0), r.seg(r.count-1)
	if err := probe(first.p0, tangent(first, first.p0).scale(-1), l.materialNormal(first, first.p0)); err != nil {
		return err
	}
	return probe(last.p1, tangent(last, last.p1), l.materialNormal(last, last.p1))
}

// polyline is the run's section moved by t into the material, with each end reaching ext past the run along its
// tangent, as the points of a polyline and the arc each span between two points follows (arcAt[i] is the moved radius
// of the span from point i to i+1, or 0 for a line). cells[j] is the number of pieces an arc segment j is cut into.
func (r chamferRun) polyline(t, ext float64, cells []int) ([]pt2, error) {
	l := r.loop
	first, last := r.seg(0), r.seg(r.count-1)
	start := first.p0.add(l.materialNormal(first, first.p0).scale(t))
	end := last.p1.add(l.materialNormal(last, last.p1).scale(t))
	pts := []pt2{start.sub(tangent(first, first.p0).scale(ext))}
	if first.arc {
		pts = append(pts, start)
	}
	for j := range r.count {
		s := r.seg(j)
		var to pt2
		if j == r.count-1 {
			to = end
		} else {
			c, err := l.corner(s, r.seg(j+1), t)
			if err != nil {
				return nil, err
			}
			to = c
		}
		if s.arc {
			mv, err := l.moved(s, t)
			if err != nil {
				return nil, err
			}
			from := pts[len(pts)-1]
			a0 := math.Atan2(from.y-s.c.y, from.x-s.c.x)
			delta := s.sweep()
			if !s.ccw {
				delta = -delta
			}
			for k := 1; k < cells[j]; k++ {
				phi := a0 + delta*float64(k)/float64(cells[j])
				pts = append(pts, pt2{s.c.x + mv.r*math.Cos(phi), s.c.y + mv.r*math.Sin(phi)})
			}
		}
		if j < r.count-1 || last.arc {
			pts = append(pts, to)
		}
	}
	return append(pts, end.add(tangent(last, last.p1).scale(ext))), nil
}

// cutterMesh is the removed region of a run as a closed polyhedron. At height h above the cap the section is the band
// between the run moved outward by margin and the run moved by d + h into the material, with each end extended past
// the run. The bevel is the ruled surface between the top section, over above the cap, and the bottom one, which is
// still outside the wall; the cutter's other faces lie outside the receiver.
func (r chamferRun) cutterMesh(ctx context.Context, d, over float64) (cutterMesh, error) {
	l := r.loop
	margin := over
	top, bottom := over, -(d + margin/2)
	ext := partialExtension * d
	if err := r.checkExtension(d, d+over); err != nil {
		return cutterMesh{}, err
	}
	cells := make([]int, r.count)
	rmin := math.Inf(1)
	for j := range r.count {
		s := r.seg(j)
		cells[j] = 1
		if s.arc {
			step := 2 * math.Acos(1-partialSagitta/(s.r+d+over+margin))
			cells[j] = max(2, int(math.Ceil(s.sweep()/step)))
			rmin = min(rmin, s.r)
		}
	}
	rings := make([][]pt2, 2)
	heights := []float64{top, bottom}
	for k, h := range heights {
		inner, err := r.polyline(d+h, ext, cells)
		if err != nil {
			return cutterMesh{}, err
		}
		outer, err := r.polyline(-margin, ext, cells)
		if err != nil {
			return cutterMesh{}, err
		}
		slices.Reverse(outer)
		rings[k] = append(inner, outer...)
	}
	if ringArea(rings[0]) < 0 {
		for k := range rings {
			slices.Reverse(rings[k])
		}
	}
	m := cutterMesh{}
	if !math.IsInf(rmin, 1) {
		m.bound = partialSagitta
	}
	n := len(rings[0])
	idx := make([][]int, len(rings))
	for k, ring := range rings {
		idx[k] = make([]int, n)
		for j, p := range ring {
			m.verts = append(m.verts, l.to3(p, heights[k]))
			idx[k][j] = len(m.verts) - 1
		}
	}
	for j := range n {
		a, b := idx[0][j], idx[0][(j+1)%n]
		c, e := idx[1][(j+1)%n], idx[1][j]
		m.tris = append(m.tris, [3]int{a, c, b}, [3]int{a, e, c})
	}
	for k := range rings {
		pts := make([]triangulation.Point2, n)
		loop := make([]int, n)
		for j, p := range rings[k] {
			pts[j] = triangulation.Point2{U: p.x, V: p.y}
			loop[j] = j
		}
		tris, err := triangulation.Triangulate(ctx, pts, [][]int{loop})
		if err != nil {
			return cutterMesh{}, fmt.Errorf("%w: the partial chamfer's cutter cap: %v", decaderr.ErrUnsupported, err)
		}
		for _, t := range tris {
			a, b, c := idx[k][t[0]], idx[k][t[1]], idx[k][t[2]]
			if k == 1 {
				b, c = c, b
			}
			m.tris = append(m.tris, [3]int{a, b, c})
		}
	}
	return m, nil
}

// cutRuns cuts every run from body, under producer identities from env.Ref on. It returns the body and how many
// identities it used.
func cutRuns(ctx context.Context, env topo.Env, hooks Hooks, body *topo.Body, runs []chamferRun, d, over float64) (
	*topo.Body, int, error) {
	used := 0
	next := func() topo.Env {
		step := topo.Env{Doc: env.Doc, Ref: env.Ref + provenance.Producer(used), Alloc: env.Alloc}
		used++
		return step
	}
	out := body
	var warnings opdiagnostic.CompositeSummaries
	for _, r := range runs {
		mesh, err := r.cutterMesh(ctx, d, over)
		if err != nil {
			return nil, used, err
		}
		cutter, err := hooks.MeshSolid(ctx, next(), mesh.verts, mesh.tris, mesh.bound)
		if err != nil {
			return nil, used, fmt.Errorf("%w: the partial chamfer's cutter: %v", decaderr.ErrUnsupported, err)
		}
		if out, err = hooks.Boolean(allowCoarse(ctx, out), meshbool.OpCut, next(), out, cutter); err != nil {
			return nil, used, err
		}
		warnings.Add(out.OperationWarnings, 0)
	}
	if out.Volume.Value.Base() >= body.Volume.Value.Base() {
		return nil, used, fmt.Errorf(`%w: the partial chamfer removed no material`, decaderr.ErrUnsupported)
	}
	out.OperationWarnings = warnings.Records()
	return out, used, ctx.Err()
}
