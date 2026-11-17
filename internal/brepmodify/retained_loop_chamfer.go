package brepmodify

import (
	"context"
	"errors"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/chaintol"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/retainedblend"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/triangulation"
)

// RetainedLoopChamfer chamfers whole boundary loops of a retained planar face of a mesh Boolean result when the
// per-edge retained routes cannot: a loop whose corners are not tangent, such as a window of two lines and two
// arcs, or a loop mixed with others. Every loop is a closed chain of straight edges and circular arcs, or one
// circle, and every wall beside it is a retained plane normal to the face or a retained cylinder whose axis is
// normal to it. Each wall descends from the face, so the chamfer removes material.
//
// Each loop is cut from the body by a polyhedral cutter held exactly as a mesh solid, the way FreeformRimChamfer cuts
// a free-form rim with a lofted one. At depth s below the face the cutter's boundary is the loop moved by d - s into
// the face's material, a mitre at every corner, so the bevel is 45° and meets the wall at depth d. The cutter steps
// through cutterLayers sections, so at a corner joining a line and an arc the bevel patches meet along short straight
// rulings where the exact patches meet along a conic; the departure is a few microns for a millimetre setback. A
// lofted two-section cutter is not used: its twisted arc cells carry a facet bound that the Boolean's rim rule
// amplifies into the body's bound. The result is a Boolean Cut: faceted, with the Cut's own readings.
//
// It reports false, leaving the other routes in charge, for any receiver or selection it does not recognise.
// A recognised selection whose setback cannot be cut is refused as ErrUnsupported (the setback reaches the far end
// of a wall) or ErrDegenerate (the setback empties a loop, or crosses it over itself).
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
	// The default cutter runs first, so a chamfer that builds with it keeps its
	// geometry. A receiver held finer than that cutter (small rounds beside a
	// small bore) refuses it at the Cut's chain-depth gate; the cutter is then
	// built within the receiver's own mesh bound.
	for _, fine := range []bool{false, true} {
		for _, attempt := range rimAttempts {
			out, spent, retry, err := cutLoops(ctx, env, hooks, body, chains, d, attempt, span, fine)
			span += spent
			if err == nil {
				return Result{Body: out, Span: span}, true, nil
			}
			if errors.Is(err, errSameCutter) {
				// The fine cutter would repeat the default attempts.
				return Result{}, true, last
			}
			last = err
			if !retry || ctx.Err() != nil || !errors.Is(err, decaderr.ErrUnsupported) {
				return Result{}, true, last
			}
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
	// held marks a chord of an unselected edge's held mesh chain, read for a polygon alone: no cutter meets it.
	held bool
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
	// exact is set for a receiver whose faces carry their own exact surfaces and no retained tags, a prism.
	exact bool
	// face is the face the loop bounds, and edges[i] the edge segs[i] was read from, for the partial route's clearance
	// gate. The chords of one held chain share an edge.
	face  *topo.Face
	edges []*topo.Edge
}

// loopReading tells readChamferLoop which edges of a loop a cutter serves, for a loop of which only some are chamfered.
// A selected edge and the edge beside it must be exact lines or arcs between the face and a wall normal to it; any
// other edge is read for the loop's polygon alone, a held chain's chords for an edge of no exact curve.
type loopReading struct {
	selected map[*topo.Edge]bool
	// held reads the mesh chain of the face's boundary between an edge's end points.
	held *heldFace
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
			l, ok := readChamferLoop(f, plane, loop, false, nil)
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

// readChamferLoop reads loop of the planar face f. A nil reading serves every edge of the loop.
func readChamferLoop(f *topo.Face, plane surfacegeom.Plane, loop *topo.Loop, exact bool,
	reading *loopReading) (*chamferLoop, bool) {
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
	l := &chamferLoop{exact: exact, outer: loop.Outer, origin: plane.Frame.Origin(), u: u, v: n.Cross(u), n: n, face: f}
	l.depth = math.Inf(1)
	for i, ce := range loop.CoEdges {
		e := ce.Edge
		if e == nil || e.Start == nil || e.End == nil {
			return nil, false
		}
		exactEdge := reading == nil || reading.serves(loop, i)
		if exactEdge {
			// A prism's cap edge always meets its wall at a right angle. There e.Convex is only the rim's 2D turn, which
			// a concave circular wall clears, so the exact route does not require it.
			if (!e.Convex && !exact) || len(e.Faces) != 2 {
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
		}
		from, to := e.Start.Position, e.End.Position
		if !ce.Forward {
			from, to = to, from
		}
		before := len(l.segs)
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
			if exactEdge {
				return nil, false
			}
			chain, ok := reading.held.chain(loop, from, to)
			if !ok {
				return nil, false
			}
			for k := 0; k+1 < len(chain); k++ {
				l.segs = append(l.segs, loopSeg{p0: l.to2(chain[k]), p1: l.to2(chain[k+1]), held: true})
			}
		}
		for range len(l.segs) - before {
			l.edges = append(l.edges, e)
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
	if !l.exact && !retainedTag(wall) {
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

// cutLoops cuts every loop from body with a cutter that reaches factor·d past the face and past the bevel's far end,
// under producer identities from env.Ref+first on. It returns the body, how many identities it used, and whether an
// error came from the Boolean.
func cutLoops(ctx context.Context, env topo.Env, hooks Hooks, body *topo.Body, loops []*chamferLoop, d float64,
	attempt rimAttempt, first int, fine bool) (*topo.Body, int, bool, error) {
	over := attempt.overshoot * d
	used := 0
	next := func() topo.Env {
		step := topo.Env{Doc: env.Doc, Ref: env.Ref + provenance.Producer(first+used), Alloc: env.Alloc}
		used++
		return step
	}
	out := body
	var warnings opdiagnostic.CompositeSummaries
	// The Cut admits a tool held as coarsely as the receiver's own mesh
	// (allowCoarse), so a fine cutter is built within that bound.
	var target float64
	if fine {
		target = chaintol.Of(ctx)
		if fp, ok := out.Payload.(facetedrecord.Payload); ok {
			target = max(target, fp.MeshBound)
		}
	}
	// Every loop's cutter is built before any Cut. A fine cutter that equals
	// its loop's default cutter is that cutter, so the loop is cut with it; the
	// fine attempt is skipped only when every loop's is.
	meshes := make([]cutterMesh, len(loops))
	allSame := true
	for i, l := range loops {
		mesh, same, err := l.cutterMesh(ctx, d, over, attempt.scale, target)
		if err != nil {
			return nil, used, true, err
		}
		meshes[i] = mesh
		allSame = allSame && same
	}
	if fine && allSame {
		return nil, used, true, errSameCutter
	}
	for _, mesh := range meshes {
		cutter, err := hooks.MeshSolid(ctx, next(), mesh.verts, mesh.tris, mesh.bound)
		if err != nil {
			return nil, used, true, fmt.Errorf("%w: the loop chamfer's cutter: %v", decaderr.ErrUnsupported, err)
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

const (
	// cutterLayers is how many equal steps the cutter takes from its top section to its bottom one. Between two
	// layers a corner joining a line and an arc is ruled along a straight line where the exact bevel curves; the
	// departure shrinks with the square of the step.
	cutterLayers = 6
	// cutterSagitta is how far a chord of a cutter arc may lie inside its circle, in millimetres. The cutter is a
	// polyhedron, held exactly, and every vertex carries this plus the layer departure as its bound.
	cutterSagitta = 0.002
	// maxCutterLayers caps the layers a cutter takes to meet its receiver's mesh
	// bound.
	maxCutterLayers = 256
)

// cutterMesh is the triangles of a cutter and the bound every vertex carries.
type cutterMesh struct {
	verts []r3.Vec
	tris  [][3]int
	bound float64
}

// cutterRing samples a section as a closed polygon: its corners, and between the corners of an arc evenly spaced
// points. cells[i] is the number of pieces segment i is cut into, equal at every layer so that layers pair vertex for
// vertex.
func (l *chamferLoop) cutterRing(sec section, cells []int) []pt2 {
	n := len(l.segs)
	var ring []pt2
	for i, s := range l.segs {
		a, b := sec.points[i], sec.points[(i+1)%n]
		ring = append(ring, a)
		if !s.arc {
			continue
		}
		a0, a1 := math.Atan2(a.y-s.c.y, a.x-s.c.x), math.Atan2(b.y-s.c.y, b.x-s.c.x)
		delta := a1 - a0
		if s.ccw {
			for delta <= 0 {
				delta += 2 * math.Pi
			}
		} else {
			for delta >= 0 {
				delta -= 2 * math.Pi
			}
		}
		for k := 1; k < cells[i]; k++ {
			phi := a0 + delta*float64(k)/float64(cells[i])
			ring = append(ring, pt2{s.c.x + sec.radii[i]*math.Cos(phi), s.c.y + sec.radii[i]*math.Sin(phi)})
		}
	}
	return ring
}

// cutterMesh builds the removed region of the loop as a closed polyhedron. At height h above the face the section is
// the loop moved by d + h into the material, in equal steps from an overshoot above the face to an overshoot below the
// bevel's far end, each at most over (boundedOvershoot). The outer boundary of a face bounds material inside it, so
// its cutter is the ring between a rectangle scale times the loop's reach and the moved loop; any other loop's cutter
// is the whole region inside the moved loop.
//
// The default cutter (target 0) takes cutterLayers layers and cutterSagitta chords. A fine cutter (target > 0) holds
// both its chord sagitta and its layer departure within target/2. Either way every vertex carries the sagitta plus
// the layer departure the shared closed form proves (chamferRun.layerDepartureWith): the ruling between rings at the
// loop's sharp corners beside an arc, and the twist of those arcs' cells, over the layers that meet the material.
// same reports a fine cutter identical to the default one.
func (l *chamferLoop) cutterMesh(ctx context.Context, d, over, scale, target float64) (cutterMesh, bool, error) {
	fine := target > 0
	sagitta := cutterSagitta
	if fine {
		sagitta = min(sagitta, target/2)
	}
	cells := make([]int, len(l.segs))
	ringLen := 0
	for i, s := range l.segs {
		cells[i] = 1
		if s.arc {
			rmax := s.r + d + over
			step := 2 * math.Acos(1-sagitta/rmax)
			cells[i] = max(2, int(math.Ceil(s.sweep()/step)))
		}
		ringLen += cells[i]
	}
	// The cutter above the face and beyond the bevel's far end removes no
	// material; it only keeps the cutter's caps clear of the face and the walls.
	// Moving the loop on past its narrowest feature (a dart of width 2·d', a
	// convex round of radius r) crosses the moved loop over itself, so each
	// overshoot shrinks until its sections are simple rings.
	topOver, err := l.boundedOvershoot(d, d, over, cells, 1)
	if err != nil {
		return cutterMesh{}, false, err
	}
	bottomOver, err := l.boundedOvershoot(0, d, over, cells, -1)
	if err != nil {
		return cutterMesh{}, false, err
	}
	top, bottom := topOver, -(d + bottomOver)
	layers, departure, err := l.cutterLayersFor(d, top, bottom, sagitta, target)
	if err != nil {
		return cutterMesh{}, false, err
	}
	same := fine && sagitta == cutterSagitta && layers == cutterLayers
	// Each layer contributes two triangles per ring edge, and the caps about
	// two per ring vertex; a cutter past the audit's triangle ceiling is
	// refused before any ring is built.
	if tris := 2*ringLen*layers + 4*ringLen + 16; tris > loftmesh.MaxLoftAuditTriangles {
		return cutterMesh{}, false, fmt.Errorf(`%w: the loop chamfer's cutter needs about %d triangles, more than %d`,
			decaderr.ErrUnsupported, tris, loftmesh.MaxLoftAuditTriangles)
	}
	rings := make([][]pt2, layers+1)
	heights := make([]float64, layers+1)
	for k := range rings {
		heights[k] = top + (bottom-top)*float64(k)/float64(layers)
		sec, err := l.sectionAt(d + heights[k])
		if err != nil {
			return cutterMesh{}, false, err
		}
		rings[k] = l.cutterRing(sec, cells)
		if !ringSimple(rings[k]) {
			return cutterMesh{}, false, fmt.Errorf(`%w: a chamfer of %.6g mm crosses its loop over itself`,
				decaderr.ErrDegenerate, d)
		}
	}
	// Walk every ring counter-clockwise about the face's outward normal.
	ccw := ringArea(rings[0]) > 0
	if !ccw {
		for k := range rings {
			slices.Reverse(rings[k])
		}
	}
	m := cutterMesh{bound: sagitta + departure}
	n := len(rings[0])
	add := func(p pt2, h float64) int {
		m.verts = append(m.verts, l.to3(p, h))
		return len(m.verts) - 1
	}
	idx := make([][]int, len(rings))
	for k, ring := range rings {
		idx[k] = make([]int, n)
		for j, p := range ring {
			idx[k][j] = add(p, heights[k])
		}
	}
	// Side faces of the moved loop between consecutive layers: outward from the cutter's region, which for a filled
	// cutter is away from the loop's inside and for a ring is into it.
	for k := range layers {
		for j := range n {
			a, b := idx[k][j], idx[k][(j+1)%n]
			c, e := idx[k+1][(j+1)%n], idx[k+1][j]
			if l.outer {
				m.tris = append(m.tris, [3]int{a, b, c}, [3]int{a, c, e})
			} else {
				m.tris = append(m.tris, [3]int{a, c, b}, [3]int{a, e, c})
			}
		}
	}
	var boxTop, boxBottom [4]int
	if l.outer {
		box := l.box(scale * (d + over))
		for c, corner := range box {
			boxTop[c], boxBottom[c] = add(corner, heights[0]), add(corner, heights[layers])
		}
		// The rectangle's four walls face outward.
		for c := range box {
			a, b := boxTop[c], boxTop[(c+1)%4]
			cc, e := boxBottom[(c+1)%4], boxBottom[c]
			m.tris = append(m.tris, [3]int{a, cc, b}, [3]int{a, e, cc})
		}
	}
	capTris := func(layer int, up bool) error {
		pts := make([]triangulation.Point2, 0, n+4)
		for _, p := range rings[layer] {
			pts = append(pts, triangulation.Point2{U: p.x, V: p.y})
		}
		loop := make([]int, n)
		for j := range n {
			loop[j] = j
		}
		loops := [][]int{loop}
		box := boxTop
		if layer != 0 {
			box = boxBottom
		}
		if l.outer {
			corners := l.box(scale * (d + over))
			for _, c := range corners {
				pts = append(pts, triangulation.Point2{U: c.x, V: c.y})
			}
			slices.Reverse(loop)
			loops = [][]int{{n, n + 1, n + 2, n + 3}, loop}
		}
		tris, err := triangulation.Triangulate(ctx, pts, loops)
		if err != nil {
			return fmt.Errorf("%w: the loop chamfer's cutter cap: %v", decaderr.ErrUnsupported, err)
		}
		vertex := func(i int) int {
			if i < n {
				return idx[layer][i]
			}
			return box[i-n]
		}
		for _, t := range tris {
			a, b, c := vertex(t[0]), vertex(t[1]), vertex(t[2])
			if !up {
				b, c = c, b
			}
			m.tris = append(m.tris, [3]int{a, b, c})
		}
		return nil
	}
	if err := capTris(0, true); err != nil {
		return cutterMesh{}, false, err
	}
	if err := capTris(layers, false); err != nil {
		return cutterMesh{}, false, err
	}
	return m, same, nil
}

// boundedOvershoot is the overshoot, at most over, of the cutter past the
// section at t0 (the face at t0 = d, sense 1; the bevel's far end at t0 = 0,
// sense -1) whose sections at t0 + sense·k·overshoot/4, k = 1 to 4, are simple
// rings. It halves over until they are. The section at t0 itself must be a
// simple ring: otherwise the chamfer of size d crosses its loop over itself and
// is ErrDegenerate.
func (l *chamferLoop) boundedOvershoot(t0, d, over float64, cells []int, sense float64) (float64, error) {
	if !l.ringValidAt(t0, cells) {
		return 0, fmt.Errorf(`%w: a chamfer of %.6g mm crosses its loop over itself`, decaderr.ErrDegenerate, d)
	}
	for range maxOvershootHalvings {
		simple := true
		for k := 1; k <= 4 && simple; k++ {
			simple = l.ringValidAt(t0+sense*over*float64(k)/4, cells)
		}
		if simple {
			return over, nil
		}
		over /= 2
	}
	return 0, fmt.Errorf(`%w: the loop chamfer's cutter finds no clearance past its loop's narrowest feature`,
		decaderr.ErrUnsupported)
}

// maxOvershootHalvings bounds boundedOvershoot's search: 2^-10 of the cutter's
// first overshoot.
const maxOvershootHalvings = 10

// cutterLayersFor is how many layers a cutter of chord sagitta takes between
// the heights top and bottom, and the layer departure the shared closed form
// proves for them (chamferRun.layerDepartureWith over the whole loop). The
// default cutter (target 0) takes cutterLayers. A fine cutter steps the layers
// up from cutterLayers until the departure is within target/2, up to
// maxCutterLayers.
func (l *chamferLoop) cutterLayersFor(d, top, bottom, sagitta, target float64) (int, float64, error) {
	run := chamferRun{loop: l, count: len(l.segs) + 1}
	// Only the layers that overlap the material band, -d ≤ h ≤ 0, meet the
	// body; the overshoot layers carry no departure that a contact reads.
	departure := func(layers int) (float64, error) {
		heights := make([]float64, layers+1)
		for k := range heights {
			heights[k] = top + (bottom-top)*float64(k)/float64(layers)
		}
		lo, hi := 0, layers
		for lo+1 <= layers && heights[lo+1] >= 0 {
			lo++
		}
		for hi-1 >= 0 && heights[hi-1] <= -d {
			hi--
		}
		return run.layerDepartureWith(d, heights[lo:hi+1], sagitta)
	}
	if target <= 0 {
		dep, err := departure(cutterLayers)
		return cutterLayers, dep, err
	}
	for layers := cutterLayers; layers <= maxCutterLayers; layers++ {
		dep, err := departure(layers)
		if err != nil {
			return 0, 0, err
		}
		if dep <= target/2 {
			return layers, dep, nil
		}
	}
	return 0, 0, fmt.Errorf(`%w: the loop chamfer's cutter needs more than %d layers to hold its loop within %.3g mm`,
		decaderr.ErrUnsupported, maxCutterLayers, target)
}

// errSameCutter is cutLoops's answer for a fine attempt whose every cutter
// would be the default one: the caller has already run it.
var errSameCutter = errors.New("the fine cutter is the default cutter")

// ringValidAt reports whether the loop moved by t is a section whose sampled
// ring is simple.
func (l *chamferLoop) ringValidAt(t float64, cells []int) bool {
	sec, err := l.sectionAt(t)
	if err != nil {
		return false
	}
	return ringSimple(l.cutterRing(sec, cells))
}

// ringSimple reports whether the closed polygon ring meets itself only where
// consecutive edges share their vertex: no two other edges cross or touch. The
// test only refuses: a ring it calls simple is then triangulated and meshed
// under their own checks.
func ringSimple(ring []pt2) bool {
	n := len(ring)
	if n < 3 {
		return false
	}
	for i := range n {
		a, b := ring[i], ring[(i+1)%n]
		for j := i + 1; j < n; j++ {
			if j == i+1 || (i == 0 && j == n-1) {
				continue
			}
			c, e := ring[j], ring[(j+1)%n]
			if segmentsMeet(a, b, c, e) {
				return false
			}
		}
	}
	return true
}

// segmentsMeet reports whether the closed segments ab and ce share a point.
func segmentsMeet(a, b, c, e pt2) bool {
	d1 := b.sub(a).cross(c.sub(a))
	d2 := b.sub(a).cross(e.sub(a))
	d3 := e.sub(c).cross(a.sub(c))
	d4 := e.sub(c).cross(b.sub(c))
	if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)) {
		return true
	}
	onSegment := func(p, q, r pt2) bool {
		return math.Min(p.x, q.x) <= r.x && r.x <= math.Max(p.x, q.x) &&
			math.Min(p.y, q.y) <= r.y && r.y <= math.Max(p.y, q.y)
	}
	return (d1 == 0 && onSegment(a, b, c)) || (d2 == 0 && onSegment(a, b, e)) ||
		(d3 == 0 && onSegment(c, e, a)) || (d4 == 0 && onSegment(c, e, b))
}

// ringArea is the signed area of a closed polygon, positive when it runs counter-clockwise.
func ringArea(ring []pt2) float64 {
	area := 0.0
	for i, p := range ring {
		area += p.cross(ring[(i+1)%len(ring)]) / 2
	}
	return area
}

// box is the counter-clockwise rectangle that surrounds the loop by margin on every side.
func (l *chamferLoop) box(margin float64) [4]pt2 {
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
	lo, hi = lo.sub(pt2{margin, margin}), hi.add(pt2{margin, margin})
	return [4]pt2{lo, {hi.x, lo.y}, hi, {lo.x, hi.y}}
}

// serves reports whether the cutter of the partial route reads the edge at index i of loop as exact: a selected edge,
// and an edge beside one, which the run's end reads.
func (r *loopReading) serves(loop *topo.Loop, i int) bool {
	n := len(loop.CoEdges)
	for _, k := range []int{i, (i + 1) % n, (i + n - 1) % n} {
		if r.selected[loop.CoEdges[k].Edge] {
			return true
		}
	}
	return false
}
