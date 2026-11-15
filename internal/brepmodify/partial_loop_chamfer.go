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
// wall descends from the cap and the chamfer removes material. A chain of selected segments is removed by one cutter
// that holds exactly the material at or above its bevel: the region within t of the chain, measured inside the cap,
// above the plane that leaves the wall at depth d and the cap at d. Segments of a chain that meet at a sharp corner are
// mitred inside the cutter, the same as at a corner of a whole loop. Each end of a chain is read on its own:
//
//   - A convex end turned by 45 degrees or more is served by the cutter reaching past the end along the run's tangent,
//     so the bevel ends on the wall beside the next edge as a chamfer's bevel does. Below 90 degrees the bevel runs out
//     onto that wall, so the next edge must be at least 1.5 reach/sin(turn) long, and an arc run's tangent must stay
//     within partialSagitta of its circle over the run-out.
//   - A concave end turned by up to 90 degrees beside a line stops the bevel on the next face's plane, carried back over
//     the run's band: the cutter loses a local sector about the corner, so the bevel tapers to the corner's vertical
//     edge and the next face grows by a coplanar triangle. There is no square end. The material continues past a
//     concave corner, so a cutter that reaches past the end would carve it. A run that curls back within the sector's
//     reach is refused.
//
// Any other end is refused rather than guessed: a convex end turned by less than 45 degrees, a concave end turned by
// more than 90 degrees, or a concave end beside an arc.
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
	runs, ok := readChamferRuns(body, edges, true)
	if !ok {
		return Result{}, false, nil
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
func readChamferRuns(body *topo.Body, edges []*topo.Edge, exact bool) (runs []chamferRun, ok bool) {
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
			return nil, false
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
				return nil, false
			}
			l, ok := readChamferLoop(c.face, c.plane, loop, exact)
			if !ok || l.circle || len(l.segs) != len(loop.CoEdges) {
				return nil, false
			}
			sel := make([]bool, len(loop.CoEdges))
			for i, ce := range loop.CoEdges {
				sel[i] = left[ce.Edge]
			}
			rs, ok := l.runs(sel)
			if !ok {
				// A selection this route cannot take keeps the refusal the exact routes gave it.
				return nil, false
			}
			runs = append(runs, rs...)
			for _, ce := range loop.CoEdges {
				delete(left, ce.Edge)
			}
		}
	}
	return runs, len(runs) > 0
}

// continues reports whether the walk leaves the end of prev along the tangent it arrived on: the junction is smooth.
func continues(prev, next loopSeg) bool {
	v := next.p0
	tin, tout := tangent(prev, v), tangent(next, v)
	return math.Abs(tin.cross(tout)) < 1e-7 && tin.dot(tout) > 0
}

// runs splits the selected segments sel of the loop into runs, and checks each run's two ends.
func (l *chamferLoop) runs(sel []bool) ([]chamferRun, bool) {
	n := len(l.segs)
	// A run is a maximal chain of selected segments, starting at one whose predecessor is not selected.
	var out []chamferRun
	for i := range n {
		if !sel[i] {
			continue
		}
		prev := (i + n - 1) % n
		// One cutter serves a maximal chain of selected segments, so segments meeting at a sharp corner mitre inside it.
		if sel[prev] {
			continue
		}
		count := 1
		for sel[(i+count)%n] && count < n {
			count++
		}
		out = append(out, chamferRun{loop: l, first: i, count: count})
	}
	for _, r := range out {
		// A selected neighbour that continues the run is part of it, so one left over means the selection stops
		// short of a smooth chain, which no CAD chamfer builds.
		if next := (r.first + r.count) % n; !sel[next] && continues(l.segs[(next+n-1)%n], l.segs[next]) {
			return nil, false
		}
		if prev := (r.first + n - 1) % n; !sel[prev] && continues(l.segs[prev], l.segs[r.first]) {
			return nil, false
		}
	}
	if total := func() int {
		c := 0
		for _, r := range out {
			c += r.count
		}
		return c
	}(); total == n {
		return nil, false
	}
	for _, r := range out {
		if _, ok := r.ends(); !ok {
			return nil, false
		}
	}
	return out, true
}

// runEnd describes one end of a run: the corner at v where the run's last segment (along away, leaving the run) meets
// the unselected or next segment, which leaves v along e2away.
type runEnd struct {
	v, away, e2away, out pt2 // out: the run's wall normal out of the material at v
	concave              bool
	turn                 float64
	e2Len                float64
}

const (
	minConvexTurn  = 45 * math.Pi / 180
	maxConcaveTurn = 90 * math.Pi / 180
)

// ends reads both ends of the run : convex corners turned 45 degrees or more, concave corners turned up to 90
// degrees where the next segment is a line.
func (r chamferRun) ends() ([2]runEnd, bool) {
	l := r.loop
	first, last := r.seg(0), r.seg(r.count-1)
	read := func(prev, next loopSeg, atEnd bool) (runEnd, bool) {
		v := next.p0
		tin, tout := tangent(prev, v), tangent(next, v)
		cross := tin.cross(tout)
		if math.Abs(cross) < 1e-9 && tin.dot(tout) > 0 {
			return runEnd{}, false
		}
		e := runEnd{v: v, turn: math.Atan2(math.Abs(cross), tin.dot(tout))}
		e.concave = (cross > 0) != l.materialLeft
		other := next
		if atEnd {
			e.away, e.e2away, e.out = tin, tout, l.materialNormal(prev, v).scale(-1)
		} else {
			e.away, e.e2away, e.out = tout.scale(-1), tin.scale(-1), l.materialNormal(next, v).scale(-1)
			other = prev
		}
		e.e2Len = other.p0.dist(other.p1)
		if e.concave {
			return e, e.turn <= maxConcaveTurn+1e-9 && !other.arc
		}
		return e, e.turn >= minConvexTurn-1e-12
	}
	a, ok1 := read(r.before(), first, false)
	b, ok2 := read(last, r.after(), true)
	return [2]runEnd{a, b}, ok1 && ok2
}

// clipRadius is the radius of the sector the end clip removes about the corner of a concave end.
func (e runEnd) clipRadius(d, over float64) float64 {
	return 1.5 * ((d+2*over)/math.Sin(math.Min(e.turn, math.Pi/2)) + d + over)
}

// extFor is how far the cutter reaches past a convex end so that its end face lies outside the material.
func (e runEnd) extFor(d, reach float64) float64 {
	if e.concave || e.turn >= math.Pi/2-1e-9 {
		return partialExtension * d
	}
	return max(partialExtension*d, 1.5*reach/math.Tan(e.turn))
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

// checkExtension samples the region each cutter end reaches past its run, to the depth the cutter cuts, and refuses a
// run whose extension holds material of the cap.
func (r chamferRun) checkExtension(d, reach float64) error {
	l := r.loop
	poly := l.polygon()
	ends, _ := r.ends()
	probe := func(e runEnd, seg loopSeg, tan, in pt2) error {
		v := e.v
		if e.concave {
			// The end clip removes what lies past the run's end.
			return nil
		}
		if e.turn < math.Pi/2-1e-9 {
			// The bevel runs out onto the next wall, which must be long enough to hold the run-out, and an arc run's
			// tangent stands in for its cone over that stretch.
			if e.e2Len < 1.5*reach/math.Sin(e.turn) {
				return fmt.Errorf(`%w: a chamfered run ends at a corner whose next edge is shorter than the bevel's run-out`,
					decaderr.ErrUnsupported)
			}
			if run := reach / math.Tan(e.turn); seg.arc && run*run/(2*seg.r) > partialSagitta {
				return fmt.Errorf(`%w: a chamfered arc run ends at a corner too acute for its tangent extension`,
					decaderr.ErrUnsupported)
			}
		}
		ext := e.extFor(d, reach)
		// The material the next segment's wall bounds near v is the bevel's run-out onto that wall.
		e2in := e.e2away.leftNormal()
		if e2in.dot(e.away.scale(-1)) < 0 {
			e2in = e2in.scale(-1)
		}
		for _, s := range []float64{0.2, 0.4, 0.6, 0.8, 1} {
			for _, u := range []float64{0.05, 0.25, 0.5, 0.75, 1} {
				p := v.add(tan.scale(s * ext)).add(in.scale(u * reach))
				if p.sub(v).dot(e2in) > -1e-9 && p.sub(v).dot(e.e2away) < e.e2Len {
					continue
				}
				if contains(poly, p) == l.outer {
					return fmt.Errorf(`%w: material of the cap lies beside a chamfered run's end, where the bevel would cut it`,
						decaderr.ErrUnsupported)
				}
			}
		}
		return nil
	}
	first, last := r.seg(0), r.seg(r.count-1)
	if err := probe(ends[0], first, tangent(first, first.p0).scale(-1), l.materialNormal(first, first.p0)); err != nil {
		return err
	}
	return probe(ends[1], last, tangent(last, last.p1), l.materialNormal(last, last.p1))
}

// polyline is the run's section moved by t into the material, with each end reaching ext past the run along its
// tangent, as the points of a polyline and the arc each span between two points follows (arcAt[i] is the moved radius
// of the span from point i to i+1, or 0 for a line). cells[j] is the number of pieces an arc segment j is cut into.
func (r chamferRun) polyline(t, extStart, extEnd float64, cells []int) ([]pt2, error) {
	l := r.loop
	first, last := r.seg(0), r.seg(r.count-1)
	start := first.p0.add(l.materialNormal(first, first.p0).scale(t))
	end := last.p1.add(l.materialNormal(last, last.p1).scale(t))
	pts := []pt2{start.sub(tangent(first, first.p0).scale(extStart))}
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
			// Sweep the moved arc between its own end points: a mitre moves them off the source radials.
			delta := math.Atan2(to.y-s.c.y, to.x-s.c.x) - a0
			if !s.ccw {
				delta = -delta
			}
			for delta < -1e-9 {
				delta += 2 * math.Pi
			}
			for delta >= 2*math.Pi-1e-9 {
				delta -= 2 * math.Pi
			}
			if delta < 0 {
				delta = 0
			}
			if s.sweep() > math.Pi && delta < math.Pi/2 {
				delta += 2 * math.Pi
			}
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
	return append(pts, end.add(tangent(last, last.p1).scale(extEnd))), nil
}

// cutterMesh is the removed region of a run as a closed polyhedron. At height h above the cap the section is the band
// between the run moved outward by margin and the run moved by d + h into the material, with each end extended past
// the run. The bevel is the ruled surface between the top section, over above the cap, and the bottom one, which is
// still outside the wall; the cutter's other faces lie outside the receiver.
func (r chamferRun) cutterMesh(ctx context.Context, d, over float64) (cutterMesh, error) {
	l := r.loop
	margin := over
	top, bottom := over, -(d + margin/2)
	ends, _ := r.ends()
	extS, extE := ends[0].extFor(d, d+over+margin), ends[1].extFor(d, d+over+margin)
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
		inner, err := r.polyline(d+h, extS, extE, cells)
		if err != nil {
			return cutterMesh{}, err
		}
		outer, err := r.polyline(-margin, extS, extE, cells)
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
		ends, _ := r.ends()
		for i, e := range ends {
			if !e.concave {
				continue
			}
			if err := r.checkClipLocal(e, i == 0, d, over); err != nil {
				return nil, used, err
			}
			clip, err := r.loop.endClipMesh(ctx, e, d, over)
			if err != nil {
				return nil, used, err
			}
			clipBody, err := hooks.MeshSolid(ctx, next(), clip.verts, clip.tris, clip.bound)
			if err != nil {
				return nil, used, fmt.Errorf("%w: the partial chamfer's end clip: %v", decaderr.ErrUnsupported, err)
			}
			if cutter, err = hooks.Boolean(allowCoarse(ctx, cutter), meshbool.OpCut, next(), cutter, clipBody); err != nil {
				return nil, used, fmt.Errorf("%w: clip the partial chamfer's cutter at a concave end: %v",
					decaderr.ErrUnsupported, err)
			}
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

// checkClipLocal refuses a run that curls back into the end clip of the concave end e (the run's start when atStart):
// a stretch of the run farther along it than the clip's reach, and yet within the clip's radius of the corner, would be
// clipped away too.
func (r chamferRun) checkClipLocal(e runEnd, atStart bool, d, over float64) error {
	cells := make([]int, r.count)
	for j := range r.count {
		if s := r.seg(j); s.arc {
			step := 2 * math.Acos(1-partialSagitta/(s.r+d+over))
			cells[j] = max(2, int(math.Ceil(s.sweep()/step)))
		} else {
			cells[j] = 1
		}
	}
	pts, err := r.polyline(0, 0, 0, cells)
	if err != nil {
		return err
	}
	if !atStart {
		slices.Reverse(pts)
	}
	s := e.clipRadius(d, over)
	reach := d + over
	along := 0.0
	for i := 1; i < len(pts); i++ {
		along += pts[i].dist(pts[i-1])
		if along > s+reach && pts[i].dist(e.v) < s {
			return fmt.Errorf(`%w: a chamfered run curls back into the clip at its concave end`, decaderr.ErrUnsupported)
		}
	}
	return nil
}

// endClipMesh is the region a run's cutter loses at a concave end: a sector of a disc about the corner v,
// outside the wedge between the next wall's plane carried back over the run and the run's wall normal at v. What it
// leaves ends the bevel on the next wall's plane, the way a CAD chamfer extends the unselected neighbour.
func (l *chamferLoop) endClipMesh(ctx context.Context, e runEnd, d, over float64) (cutterMesh, error) {
	// The outside boundary bisects the corner's empty wedge, clear of both walls.
	r1, r2 := e.e2away.scale(-1), e.e2away.sub(e.away).unit()
	a1, a2 := math.Atan2(r1.y, r1.x), math.Atan2(r2.y, r2.x)
	aw := math.Atan2(e.away.y, e.away.x)
	mod := func(x float64) float64 {
		for x < 0 {
			x += 2 * math.Pi
		}
		for x >= 2*math.Pi {
			x -= 2 * math.Pi
		}
		return x
	}
	from, sweep := a2, mod(a1-a2)
	if mod(aw-a2) > sweep {
		from, sweep = a1, mod(a2-a1)
	}
	s := e.clipRadius(d, over)
	ring := []pt2{e.v}
	const cells = 24
	for k := 0; k <= cells; k++ {
		phi := from + sweep*float64(k)/cells
		ring = append(ring, e.v.add(pt2{math.Cos(phi), math.Sin(phi)}.scale(s)))
	}
	if ringArea(ring) < 0 {
		slices.Reverse(ring)
	}
	heights := []float64{over + d, -(2*d + over)}
	m := cutterMesh{}
	n := len(ring)
	idx := make([][]int, 2)
	for k, h := range heights {
		idx[k] = make([]int, n)
		for j, p := range ring {
			m.verts = append(m.verts, l.to3(p, h))
			idx[k][j] = len(m.verts) - 1
		}
	}
	for j := range n {
		a, b := idx[0][j], idx[0][(j+1)%n]
		c, f := idx[1][(j+1)%n], idx[1][j]
		m.tris = append(m.tris, [3]int{a, c, b}, [3]int{a, f, c})
	}
	pts := make([]triangulation.Point2, n)
	loop := make([]int, n)
	for j, p := range ring {
		pts[j] = triangulation.Point2{U: p.x, V: p.y}
		loop[j] = j
	}
	tris, err := triangulation.Triangulate(ctx, pts, [][]int{loop})
	if err != nil {
		return cutterMesh{}, fmt.Errorf("%w: the end clip's cap: %v", decaderr.ErrUnsupported, err)
	}
	for k := range 2 {
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
