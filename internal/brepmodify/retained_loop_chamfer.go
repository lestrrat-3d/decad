package brepmodify

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/chaintol"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/retainedblend"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// RetainedLoopChamfer chamfers whole boundary loops of a retained planar face of a mesh Boolean result when the
// per-edge retained routes cannot: a loop whose corners are not tangent, such as a window of two lines and two
// arcs, or a loop mixed with others. Every loop is a closed chain of straight edges and circular arcs, or one
// circle, and every wall beside it is a retained plane normal to the face or a retained cylinder whose axis is
// normal to it. Each wall descends from the face, so the chamfer removes material.
//
// Each loop is cut from the body by a lofted cutter, the way FreeformRimChamfer cuts a free-form rim. At depth s
// below the face the cutter's boundary is the loop moved by d - s into the face's material, a mitre at every
// corner, so the bevel is 45° and meets the wall at depth d. Between two sections the cutter is ruled, so at a
// corner joining a line and an arc the bevel patches meet along a straight ruling where the exact patches meet
// along a conic; the two differ by well under the setback. The result is a Boolean Cut: faceted, with the Cut's
// own readings.
//
// It reports false, leaving the other routes in charge, for any receiver or selection it does not recognise.
// A recognised selection whose setback cannot be cut is refused as ErrUnsupported (the setback reaches the far end
// of a wall) or ErrDegenerate (the setback empties a loop).
func RetainedLoopChamfer(ctx context.Context, env topo.Env, hooks Hooks, body *topo.Body, edges []*topo.Edge,
	d, dDelta float64, asym *Asymmetric) (Result, bool, error) {
	fp, isFaceted := body.Payload.(facetedrecord.Payload)
	if !isFaceted || !body.Solid || asym != nil || dDelta != 0 ||
		!(d > 0) || len(edges) == 0 {
		return Result{}, false, nil
	}
	loops, ok := readChamferLoops(body, edges)
	if !ok {
		return Result{}, false, nil
	}
	// A circle is a loop the retained rim route builds exactly, and a cutter of a large circle needs more chord
	// cells than a loft allows. Circles are chamfered first by that route, and the loops of lines and arcs are cut
	// from its result.
	var rims, rest []*topo.Edge
	var chains []*chamferLoop
	for _, l := range loops {
		if !l.circle {
			chains = append(chains, l)
		}
	}
	if len(chains) == 0 {
		return Result{}, false, nil
	}
	for _, e := range edges {
		if _, ok := e.Curve.(surfacegeom.Circle3); ok {
			rims = append(rims, e)
		} else {
			rest = append(rest, e)
		}
	}
	used := 0
	if len(rims) > 0 {
		node, recognized, err := retainedblend.Chamfer(ctx, env, body, fp, rims, d, dDelta, nil)
		if err != nil || !recognized {
			if err == nil {
				err = fmt.Errorf("%w: the retained rim route does not take the circles beside the loops", decaderr.ErrUnsupported)
			}
			return Result{}, true, err
		}
		used = 1
		body = node
		if edges, ok = carriedEdges(body, rest); !ok {
			return Result{}, true, fmt.Errorf("%w: the rim chamfer moved the edges of the loops beside it",
				decaderr.ErrUnsupported)
		}
		if chains, ok = readChamferLoops(body, edges); !ok {
			return Result{}, true, fmt.Errorf("%w: the rim chamfer left the loops beside it unreadable",
				decaderr.ErrUnsupported)
		}
	} else {
		chains = loops
	}
	for _, l := range chains {
		if err := l.fits(d); err != nil {
			return Result{}, true, err
		}
	}
	if err := ctx.Err(); err != nil {
		return Result{}, true, err
	}
	span := used
	var last error
	for _, attempt := range rimAttempts {
		out, spent, retry, err := cutLoops(ctx, env, hooks, body, chains, d, attempt, span)
		span += spent
		if err == nil {
			return Result{Body: out, Span: span}, true, nil
		}
		last = err
		if !retry || ctx.Err() != nil || !errors.Is(err, decaderr.ErrUnsupported) {
			break
		}
	}
	return Result{}, true, last
}

// carriedEdges finds in body the edges of old, which an earlier chamfer left where they were: the edge with the same
// ends, in either order, and the same kind of curve.
func carriedEdges(body *topo.Body, old []*topo.Edge) ([]*topo.Edge, bool) {
	all := topobuild.Edges(body)
	out := make([]*topo.Edge, 0, len(old))
	at := func(a, b r3.Vec) bool { return a.Sub(b).Len() <= 1e-9 }
	for _, e := range old {
		var found *topo.Edge
		for _, c := range all {
			if c.Start == nil || c.End == nil || fmt.Sprintf("%T", c.Curve) != fmt.Sprintf("%T", e.Curve) {
				continue
			}
			same := at(c.Start.Position, e.Start.Position) && at(c.End.Position, e.End.Position) ||
				at(c.Start.Position, e.End.Position) && at(c.End.Position, e.Start.Position)
			if !same {
				continue
			}
			if found != nil {
				return nil, false
			}
			found = c
		}
		if found == nil {
			return nil, false
		}
		out = append(out, found)
	}
	return out, true
}

// pt2 is a point or vector in a face's plane.
type pt2 struct{ x, y float64 }

func (a pt2) add(b pt2) pt2       { return pt2{a.x + b.x, a.y + b.y} }
func (a pt2) sub(b pt2) pt2       { return pt2{a.x - b.x, a.y - b.y} }
func (a pt2) scale(s float64) pt2 { return pt2{a.x * s, a.y * s} }
func (a pt2) dot(b pt2) float64   { return a.x*b.x + a.y*b.y }
func (a pt2) cross(b pt2) float64 { return a.x*b.y - a.y*b.x }
func (a pt2) len() float64        { return math.Hypot(a.x, a.y) }
func (a pt2) unit() pt2           { return a.scale(1 / a.len()) }
func (a pt2) dist(b pt2) float64  { return a.sub(b).len() }
func (a pt2) leftNormal() pt2     { return pt2{-a.y, a.x} }

// loopSeg is one segment of a loop in walk order: a line from p0 to p1, or a circular arc about c of radius r from p0
// to p1, counter-clockwise when ccw.
type loopSeg struct {
	arc    bool
	p0, p1 pt2
	c      pt2
	r      float64
	ccw    bool
}

// chamferLoop is one selected loop of a planar face, read into the face's own plane.
type chamferLoop struct {
	segs []loopSeg
	// circle is set for a loop that is one full circle, whose only segment holds the centre and radius.
	circle bool
	// outer is set for the face's outer boundary, whose material lies inside the loop.
	outer bool
	// materialLeft is set when the face's material lies to the left of the walk.
	materialLeft bool
	// origin, u, v and n are the face's frame: n points out of the material.
	origin, u, v, n r3.Vec
	// depth is the smallest distance from the face to the far end of the loop's walls.
	depth float64
}

// to2 reads a point of the face's plane.
func (l *chamferLoop) to2(p r3.Vec) pt2 {
	q := p.Sub(l.origin)
	return pt2{q.Dot(l.u), q.Dot(l.v)}
}

// to3 lifts a point of the face's plane to height h above the face along n.
func (l *chamferLoop) to3(p pt2, h float64) r3.Vec {
	return l.origin.Add(l.u.Scale(p.x)).Add(l.v.Scale(p.y)).Add(l.n.Scale(h))
}

// loopEps is how far from exactly normal a wall may lean and still count as normal: the retained tags are exact, so
// only the rounding of their normalisation is allowed.
const loopEps = 1e-12

// readChamferLoops reads edges as whole loops of retained planar faces. It reports false unless every edge lies in a
// whole selected loop of one face, every loop is a chain of lines and arcs or one circle, and every wall is a
// retained plane or cylinder normal to the face on the descending side.
func readChamferLoops(body *topo.Body, edges []*topo.Edge) ([]*chamferLoop, bool) {
	selected := make(map[*topo.Edge]bool, len(edges))
	for _, e := range edges {
		selected[e] = true
	}
	var loops []*chamferLoop
	covered := map[*topo.Edge]bool{}
	for _, f := range topobuild.Faces(body) {
		plane, ok := f.Surface.(surfacegeom.Plane)
		if !ok || !retainedTag(f) {
			continue
		}
		for _, loop := range f.Loops {
			all, some := true, false
			for _, ce := range loop.CoEdges {
				all = all && selected[ce.Edge]
				some = some || selected[ce.Edge]
			}
			// A wall that meets the face holds some selected edges in a loop of its own; that loop is not selected.
			if !some || !all {
				continue
			}
			l, ok := readChamferLoop(f, plane, loop)
			if !ok {
				return nil, false
			}
			for _, ce := range loop.CoEdges {
				if covered[ce.Edge] {
					return nil, false
				}
				covered[ce.Edge] = true
			}
			loops = append(loops, l)
		}
	}
	if len(covered) != len(selected) {
		return nil, false
	}
	return loops, len(loops) > 0
}

// retainedTag reports whether f carries a retained tag read as the surface itself.
func retainedTag(f *topo.Face) bool {
	return f != nil && f.RetainedTag && f.TagDeparture == 0 && f.AxialDelta == 0 && retainedblend.TagIsRecord(f)
}

// readChamferLoop reads loop of the planar face f.
func readChamferLoop(f *topo.Face, plane surfacegeom.Plane, loop *topo.Loop) (*chamferLoop, bool) {
	n := plane.Frame.N()
	if f.Reversed {
		n = n.Scale(-1)
	}
	n, ok := n.Normalize()
	if !ok {
		return nil, false
	}
	u, ok := plane.Frame.U().Sub(n.Scale(plane.Frame.U().Dot(n))).Normalize()
	if !ok {
		return nil, false
	}
	l := &chamferLoop{outer: loop.Outer, origin: plane.Frame.Origin(), u: u, v: n.Cross(u), n: n}
	l.depth = math.Inf(1)
	for _, ce := range loop.CoEdges {
		e := ce.Edge
		if e == nil || !e.Convex || len(e.Faces) != 2 || e.Start == nil || e.End == nil {
			return nil, false
		}
		wall := e.Faces[0]
		if wall == f {
			wall = e.Faces[1]
		}
		depth, ok := l.wallDepth(wall)
		if !ok {
			return nil, false
		}
		l.depth = min(l.depth, depth)
		from, to := e.Start.Position, e.End.Position
		if !ce.Forward {
			from, to = to, from
		}
		switch c := e.Curve.(type) {
		case surfacegeom.Line3:
			l.segs = append(l.segs, loopSeg{p0: l.to2(from), p1: l.to2(to)})
		case surfacegeom.Arc3:
			r, err := c.Radius.In(units.Millimeter)
			axis, ok := c.Axis.Normalize()
			if err != nil || !ok || !(r > 0) || math.Abs(axis.Dot(n)) < 1-loopEps {
				return nil, false
			}
			ccw := axis.Dot(n) > 0
			if !ce.Forward {
				ccw = !ccw
			}
			l.segs = append(l.segs, loopSeg{arc: true, p0: l.to2(from), p1: l.to2(to), c: l.to2(c.Center), r: r, ccw: ccw})
		case surfacegeom.Circle3:
			r, err := c.Radius.In(units.Millimeter)
			axis, ok := c.Axis.Normalize()
			if err != nil || !ok || !(r > 0) || len(loop.CoEdges) != 1 || math.Abs(axis.Dot(n)) < 1-loopEps {
				return nil, false
			}
			l.circle = true
			l.segs = append(l.segs, loopSeg{arc: true, p0: l.to2(from), p1: l.to2(from), c: l.to2(c.Center), r: r,
				ccw: (axis.Dot(n) > 0) == ce.Forward})
		default:
			return nil, false
		}
	}
	if len(l.segs) == 0 || math.IsInf(l.depth, 1) {
		return nil, false
	}
	if !l.circle && !l.closes() {
		return nil, false
	}
	l.materialLeft = l.outer == (l.signedArea() > 0)
	if l.circle {
		// A circle walked counter-clockwise has its inside on the left.
		l.materialLeft = l.outer == l.segs[0].ccw
	}
	return l, true
}

// closes reports whether each segment ends where the next begins.
func (l *chamferLoop) closes() bool {
	for i, s := range l.segs {
		next := l.segs[(i+1)%len(l.segs)]
		if s.p1.dist(next.p0) > 1e-6 {
			return false
		}
	}
	return true
}

// signedArea is the loop's area, positive when it runs counter-clockwise, with each arc taken as its true curve.
func (l *chamferLoop) signedArea() float64 {
	area := 0.0
	for _, s := range l.segs {
		area += s.p0.cross(s.p1) / 2
		if s.arc {
			// The arc's segment between its chord and itself.
			theta := s.sweep()
			sign := 1.0
			if !s.ccw {
				sign = -1
			}
			area += sign * s.r * s.r * (theta - math.Sin(theta)) / 2
		}
	}
	return area
}

// sweep is the unsigned angle an arc turns through.
func (s loopSeg) sweep() float64 {
	a0, a1 := math.Atan2(s.p0.y-s.c.y, s.p0.x-s.c.x), math.Atan2(s.p1.y-s.c.y, s.p1.x-s.c.x)
	theta := a1 - a0
	if !s.ccw {
		theta = -theta
	}
	for theta < 0 {
		theta += 2 * math.Pi
	}
	for theta >= 2*math.Pi {
		theta -= 2 * math.Pi
	}
	return theta
}

// isNormalTo reports whether the vector w is perpendicular to the unit vector n up to rounding.
func isNormalTo(w, n r3.Vec) bool {
	u, ok := w.Normalize()
	return ok && math.Abs(u.Dot(n)) <= loopEps
}

// wallDepth reads the wall beside a loop edge: a retained plane normal to the face or a retained cylinder whose axis
// is normal to it. It returns how far the wall reaches below the face.
func (l *chamferLoop) wallDepth(wall *topo.Face) (float64, bool) {
	if !retainedTag(wall) {
		return 0, false
	}
	switch s := wall.Surface.(type) {
	case surfacegeom.Plane:
		if !isNormalTo(s.Frame.N(), l.n) {
			return 0, false
		}
	case surfacegeom.Cylinder:
		axis, ok := s.Axis.Normalize()
		if !ok || math.Abs(axis.Dot(l.n)) < 1-loopEps {
			return 0, false
		}
	default:
		return 0, false
	}
	lowest := 0.0
	for _, loop := range wall.Loops {
		for _, ce := range loop.CoEdges {
			for _, v := range []*topo.Vertex{ce.Edge.Start, ce.Edge.End} {
				if v == nil {
					return 0, false
				}
				lowest = min(lowest, v.Position.Sub(l.origin).Dot(l.n))
			}
		}
	}
	return -lowest, lowest < 0
}

// fits refuses a setback that reaches the far end of a wall.
func (l *chamferLoop) fits(d float64) error {
	if d >= l.depth {
		return fmt.Errorf(`%w: the chamfer of %.6g mm reaches the far end of a %.6g mm wall`,
			decaderr.ErrUnsupported, d, l.depth)
	}
	return nil
}

// offsetCurve is a segment moved sideways: a line through p along the unit direction dir, or a circle.
type offsetCurve struct {
	arc    bool
	p, dir pt2
	c      pt2
	r      float64
}

// toward reports whether the material of an arc segment lies toward its centre.
func (l *chamferLoop) toward(s loopSeg) bool { return l.materialLeft == s.ccw }

// materialNormal is the unit direction into the face's material at the point p of the segment s.
func (l *chamferLoop) materialNormal(s loopSeg, p pt2) pt2 {
	if s.arc {
		out := p.sub(s.c).unit()
		if l.toward(s) {
			return out.scale(-1)
		}
		return out
	}
	m := s.p1.sub(s.p0).unit().leftNormal()
	if !l.materialLeft {
		m = m.scale(-1)
	}
	return m
}

// tangent is the unit walking direction of s at p.
func tangent(s loopSeg, p pt2) pt2 {
	if !s.arc {
		return s.p1.sub(s.p0).unit()
	}
	t := p.sub(s.c).unit().leftNormal()
	if !s.ccw {
		t = t.scale(-1)
	}
	return t
}

// moved is s moved by t into the face's material.
func (l *chamferLoop) moved(s loopSeg, t float64) (offsetCurve, error) {
	if !s.arc {
		m := l.materialNormal(s, s.p0)
		return offsetCurve{p: s.p0.add(m.scale(t)), dir: s.p1.sub(s.p0).unit()}, nil
	}
	r := s.r + t
	if l.toward(s) {
		r = s.r - t
	}
	if !(r > 0) {
		return offsetCurve{}, fmt.Errorf(`%w: a chamfer of %.6g mm empties a circular wall of radius %.6g mm`,
			decaderr.ErrDegenerate, math.Abs(t), s.r)
	}
	return offsetCurve{arc: true, c: s.c, r: r}, nil
}

// intersect lists where two offset curves meet.
func intersect(a, b offsetCurve) []pt2 {
	switch {
	case !a.arc && !b.arc:
		den := a.dir.cross(b.dir)
		if math.Abs(den) < 1e-12 {
			return nil
		}
		s := b.p.sub(a.p).cross(b.dir) / den
		return []pt2{a.p.add(a.dir.scale(s))}
	case !a.arc && b.arc:
		return lineCircle(a, b)
	case a.arc && !b.arc:
		return lineCircle(b, a)
	}
	d := b.c.sub(a.c)
	dist := d.len()
	if dist < 1e-12 || dist > a.r+b.r || dist < math.Abs(a.r-b.r) {
		// Circles that miss by rounding still touch: take the tangent point.
		if dist < 1e-12 {
			return nil
		}
		slack := 1e-9 * (a.r + b.r)
		if dist-(a.r+b.r) < slack && math.Abs(a.r-b.r)-dist < slack {
			return []pt2{a.c.add(d.scale(a.r / dist))}
		}
		return nil
	}
	x := (dist*dist + a.r*a.r - b.r*b.r) / (2 * dist)
	h := math.Sqrt(math.Max(0, a.r*a.r-x*x))
	dir := d.scale(1 / dist)
	mid := a.c.add(dir.scale(x))
	return []pt2{mid.add(dir.leftNormal().scale(h)), mid.sub(dir.leftNormal().scale(h))}
}

func lineCircle(line, circle offsetCurve) []pt2 {
	f := line.p.sub(circle.c)
	b := f.dot(line.dir)
	disc := b*b - (f.dot(f) - circle.r*circle.r)
	if disc < 0 {
		if -disc > 1e-9*circle.r*circle.r {
			return nil
		}
		disc = 0
	}
	root := math.Sqrt(disc)
	return []pt2{line.p.add(line.dir.scale(-b + root)), line.p.add(line.dir.scale(-b - root))}
}

// corner is where the segments before and after v, each moved by t into the material, meet: the mitre of the offset.
func (l *chamferLoop) corner(prev, next loopSeg, t float64) (pt2, error) {
	v := next.p0
	tin, tout := tangent(prev, v), tangent(next, v)
	mIn := l.materialNormal(prev, v)
	if math.Abs(tin.cross(tout)) < 1e-9 && tin.dot(tout) > 0 {
		// A tangent junction: the offset curves touch where the normals agree.
		return v.add(mIn.scale(t)), nil
	}
	a, err := l.moved(prev, t)
	if err != nil {
		return pt2{}, err
	}
	b, err := l.moved(next, t)
	if err != nil {
		return pt2{}, err
	}
	best, bestDist := pt2{}, math.Inf(1)
	for _, p := range intersect(a, b) {
		if dist := p.dist(v); dist < bestDist {
			best, bestDist = p, dist
		}
	}
	if math.IsInf(bestDist, 1) || bestDist > 8*math.Abs(t)+1e-9 {
		return pt2{}, fmt.Errorf(`%w: a chamfer of %.6g mm leaves a loop corner with no mitre`, decaderr.ErrDegenerate,
			math.Abs(t))
	}
	return best, nil
}

// section is a loop moved by t, ready to sketch.
type section struct {
	// points[i] is where segment i begins; the last segment ends at points[0].
	points []pt2
	// radii[i] is segment i's moved radius.
	radii []float64
}

// sectionAt moves the loop by t into the material.
func (l *chamferLoop) sectionAt(t float64) (section, error) {
	if l.circle {
		c, err := l.moved(l.segs[0], t)
		return section{points: []pt2{l.segs[0].c}, radii: []float64{c.r}}, err
	}
	n := len(l.segs)
	sec := section{points: make([]pt2, n), radii: make([]float64, n)}
	for i, s := range l.segs {
		p, err := l.corner(l.segs[(i+n-1)%n], s, t)
		if err != nil {
			return section{}, err
		}
		sec.points[i] = p
		if s.arc {
			sec.radii[i] = s.r + t
			if l.toward(s) {
				sec.radii[i] = s.r - t
			}
		}
	}
	// A moved loop must keep its sense: a segment that has flipped end for end means the setback ate the segment.
	for i, s := range l.segs {
		a, b := sec.points[i], sec.points[(i+1)%n]
		if s.arc {
			continue
		}
		if b.sub(a).dot(s.p1.sub(s.p0)) <= 0 {
			return section{}, fmt.Errorf(`%w: a chamfer of %.6g mm consumes a straight edge of its loop`,
				decaderr.ErrDegenerate, math.Abs(t))
		}
	}
	return sec, nil
}

// boundary is the circle that bounds every ring cutter: about the middle of the loop's box, scale times the farthest
// point of the loop from that middle.
func (l *chamferLoop) boundary(scale float64) (pt2, float64) {
	lo, hi := pt2{math.Inf(1), math.Inf(1)}, pt2{math.Inf(-1), math.Inf(-1)}
	add := func(p pt2) {
		lo = pt2{math.Min(lo.x, p.x), math.Min(lo.y, p.y)}
		hi = pt2{math.Max(hi.x, p.x), math.Max(hi.y, p.y)}
	}
	for _, s := range l.segs {
		add(s.p0)
		add(s.p1)
		if s.arc {
			add(s.c.add(pt2{s.r, s.r}))
			add(s.c.sub(pt2{s.r, s.r}))
		}
	}
	mid := lo.add(hi).scale(0.5)
	far := 0.0
	for _, s := range l.segs {
		far = math.Max(far, math.Max(s.p0.dist(mid), s.p1.dist(mid)))
		if s.arc {
			far = math.Max(far, s.c.dist(mid)+s.r)
		}
	}
	return mid, scale * far
}

// cutLoops cuts every loop from body with a cutter that reaches factor·d past the face and past the bevel's far end,
// under producer identities from env.Ref+first on. It returns the body, how many identities it used, and whether an
// error came from the Boolean.
func cutLoops(ctx context.Context, env topo.Env, hooks Hooks, body *topo.Body, loops []*chamferLoop, d float64,
	attempt rimAttempt, first int) (*topo.Body, int, bool, error) {
	over := attempt.overshoot * d
	used := 0
	next := func() topo.Env {
		step := topo.Env{Doc: env.Doc, Ref: env.Ref + provenance.Producer(first+used), Alloc: env.Alloc}
		used++
		return step
	}
	out := body
	var warnings opdiagnostic.CompositeSummaries
	reuse := new(loftrecord.PairingReuse)
	for _, l := range loops {
		cutter, err := l.cutter(ctx, next(), d, over, attempt.scale, reuse)
		if err != nil {
			return nil, used, true, err
		}
		if out, err = hooks.Boolean(allowCoarse(ctx, out), meshbool.OpCut, next(), out, cutter); err != nil {
			return nil, used, true, err
		}
		warnings.Add(out.OperationWarnings, 0)
	}
	if out.Volume.Value.Base() >= body.Volume.Value.Base() {
		return nil, used, false, fmt.Errorf(`%w: the loop chamfer removed no material`, decaderr.ErrUnsupported)
	}
	out.OperationWarnings = warnings.Records()
	return out, used, false, ctx.Err()
}

// allowCoarse lets a cut meet the facets body holds at their own bounds. The cutter meets only the face and the walls
// beside the loop, so its rims compose from the bounds those facets already carry; the chain-depth gate would
// otherwise compare them with a chord tolerance that follows the pair's size and refuse a face whose facets reach a
// coarse circle elsewhere (the saddle chamfer allows its walls' bounds the same way).
func allowCoarse(ctx context.Context, body *topo.Body) context.Context {
	fp, ok := body.Payload.(facetedrecord.Payload)
	if !ok {
		return ctx
	}
	return chaintol.With(ctx, max(chaintol.Of(ctx), fp.MeshBound))
}

// cutter lofts the removed region of the loop. At height h above the face the section is the loop moved by d + h into
// the material, from over above the face to over below the bevel's far end. The loop of a face's outer boundary
// bounds material inside it, so its cutter is the ring outside the moved loop; any other loop's cutter is the whole
// region inside the moved loop.
func (l *chamferLoop) cutter(ctx context.Context, env topo.Env, d, over, scale float64,
	reuse *loftrecord.PairingReuse) (*topo.Body, error) {
	top, bottom := over, -(d + over)
	s0, p0, err := l.sketchSection(ctx, top, d+top, scale)
	if err != nil {
		return nil, err
	}
	s1, p1, err := l.sketchSection(ctx, bottom, d+bottom, scale)
	if err != nil {
		return nil, err
	}
	cutter, err := loftrecord.FromProfilesReusing(ctx, env, s0, p0, s1, p1, featureoption.LoftConfig{}, reuse)
	if err != nil {
		return nil, fmt.Errorf("%w: the loop chamfer's cutter: %v", decaderr.ErrUnsupported, err)
	}
	return cutter, nil
}

// sketchSection draws the loop moved by t, at height h above the face, and returns the one profile that is the
// cutter's section there.
func (l *chamferLoop) sketchSection(ctx context.Context, h, t, scale float64) (*sketch.Sketch, *sketch.Profile, error) {
	sec, err := l.sectionAt(t)
	if err != nil {
		return nil, nil, err
	}
	frame, err := r3.NewFrame(l.to3(pt2{}, h), l.u, l.v)
	if err != nil {
		return nil, nil, fmt.Errorf("%w: the loop chamfer has no cutter frame: %v", decaderr.ErrUnsupported, err)
	}
	world := sketch.NewWorld()
	plane, err := world.CreatePlaneFromFrame(frame)
	if err != nil {
		return nil, nil, fmt.Errorf("%w: the loop chamfer has no cutter plane: %v", decaderr.ErrUnsupported, err)
	}
	s, err := world.CreateSketch(plane)
	if err != nil {
		return nil, nil, fmt.Errorf("%w: the loop chamfer has no cutter sketch: %v", decaderr.ErrUnsupported, err)
	}
	joint := func(p pt2) *sketch.Point {
		q := s.CreatePoint(p.x, p.y)
		s.Fix(q)
		return q
	}
	if l.outer {
		mid, radius := l.boundary(scale)
		s.CreateCircle(joint(mid), radius)
	}
	if l.circle {
		s.CreateCircle(joint(sec.points[0]), sec.radii[0])
	} else {
		pts := make([]*sketch.Point, len(sec.points))
		for i, p := range sec.points {
			pts[i] = joint(p)
		}
		for i, seg := range l.segs {
			from, to := pts[i], pts[(i+1)%len(pts)]
			switch {
			case !seg.arc:
				s.CreateLine(from, to)
			case seg.ccw:
				s.CreateArc(joint(seg.c), from, to)
			default:
				s.CreateArc(joint(seg.c), to, from)
			}
		}
	}
	if _, err := s.Solve(ctx); err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, nil, ctxErr
		}
		return nil, nil, fmt.Errorf("%w: the loop chamfer's cutter sketch does not solve: %v", decaderr.ErrUnsupported, err)
	}
	holes := 0
	if l.outer {
		holes = 1
	}
	for _, candidate := range s.Profiles() {
		if candidate.Valid && len(candidate.Holes) == holes {
			return s, candidate, nil
		}
	}
	return nil, nil, fmt.Errorf("%w: the loop chamfer's cutter section is not one valid region", decaderr.ErrUnsupported)
}
