// Package outlineclear measures the least distance between the material
// regions two sets of closed polylines enclose in one plane
// (agent-docs/section-outline-design.md §5). It works on plain millimetre
// data and touches no body or document.
package outlineclear

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/r3"
)

// Result is the polyline clearance of two outline sets.
type Result struct {
	// Distance is the least distance between the two material regions, zero
	// when they overlap or touch.
	Distance float64
	// A and B are the pair of points, one on a loop of each set, that attain
	// Distance. Where the loops cross or one set lies in the other's
	// material they are one point.
	A, B r3.Vec
	// Overlap is true when a loop of one set properly crosses a loop of the
	// other, or a loop of one set lies strictly inside the other's material.
	Overlap bool
}

// leafSize is the most segments a tree leaf holds.
const leafSize = 4

// checkEvery is how many tree visits pass between context checks.
const checkEvery = 1 << 12

// Clearance returns the least distance between the material that the loops of
// a enclose and the material that the loops of b enclose. The loops are
// closed polylines in world coordinates that lie, to rounding, in the plane
// through origin spanned by the unit axes u and v. An outer loop runs
// counter-clockwise seen from u x v and a hole clockwise, so a point is in
// material when the loops' winding numbers around it sum to a positive
// number. Every loop has at least three finite points, and both sets hold at
// least one loop; Clearance does not recheck either.
//
// Distances are measured between the points projected into the plane. Two
// bounding-box trees prune the segment pairs, so the cost is near
// (n+m)log(n+m) for outlines of n and m segments that stay apart.
func Clearance(ctx context.Context, origin, u, v r3.Vec, a, b [][]r3.Vec) (Result, error) {
	sa := project(origin, u, v, a)
	sb := project(origin, u, v, b)
	ta, tb := build(slices.Clone(sa)), build(slices.Clone(sb))
	s := &search{done: ctx.Done(), err: ctx.Err, a: ta, b: tb}

	if found, err := s.crossing(0, 0); err != nil {
		return Result{}, err
	} else if found {
		return Result{A: s.best.pa, B: s.best.pb, Overlap: true}, nil
	}
	if p, ok := inside(sa, sb); ok {
		return Result{A: p, B: p, Overlap: true}, nil
	}
	if p, ok := inside(sb, sa); ok {
		return Result{A: p, B: p, Overlap: true}, nil
	}
	s.best = hit{d2: math.Inf(1)}
	if err := s.nearest(0, 0); err != nil {
		return Result{}, err
	}
	return Result{Distance: math.Sqrt(s.best.d2), A: s.best.pa, B: s.best.pb}, nil
}

// seg is one polyline segment, in plane coordinates and in the world
// coordinates it came from.
type seg struct {
	p, q   [2]float64
	p3, q3 r3.Vec
	loop   int
}

func project(origin, u, v r3.Vec, loops [][]r3.Vec) []seg {
	var out []seg
	for li, loop := range loops {
		for i, p := range loop {
			q := loop[(i+1)%len(loop)]
			out = append(out, seg{
				p:  [2]float64{p.Sub(origin).Dot(u), p.Sub(origin).Dot(v)},
				q:  [2]float64{q.Sub(origin).Dot(u), q.Sub(origin).Dot(v)},
				p3: p, q3: q, loop: li,
			})
		}
	}
	return out
}

// box is minX, minY, maxX, maxY.
type box [4]float64

func (s seg) box() box {
	return box{
		math.Min(s.p[0], s.q[0]), math.Min(s.p[1], s.q[1]),
		math.Max(s.p[0], s.q[0]), math.Max(s.p[1], s.q[1]),
	}
}

func (a box) union(b box) box {
	return box{math.Min(a[0], b[0]), math.Min(a[1], b[1]), math.Max(a[2], b[2]), math.Max(a[3], b[3])}
}

// dist2 is the squared distance between two boxes, zero when they meet.
func (a box) dist2(b box) float64 {
	dx := math.Max(0, math.Max(a[0]-b[2], b[0]-a[2]))
	dy := math.Max(0, math.Max(a[1]-b[3], b[1]-a[3]))
	return dx*dx + dy*dy
}

type node struct {
	box         box
	lo, hi      int // the segments of a leaf: tree.segs[lo:hi]
	left, right int // child nodes, or -1 for a leaf
}

type tree struct {
	nodes []node
	segs  []seg
}

// build orders the segments into a bounding-box tree split at the median of
// the longer side of the boxes' centres.
func build(segs []seg) *tree {
	t := &tree{segs: segs}
	t.node(0, len(segs))
	return t
}

func (t *tree) node(lo, hi int) int {
	b := t.segs[lo].box()
	for _, s := range t.segs[lo+1 : hi] {
		b = b.union(s.box())
	}
	id := len(t.nodes)
	t.nodes = append(t.nodes, node{box: b, lo: lo, hi: hi, left: -1, right: -1})
	if hi-lo <= leafSize {
		return id
	}
	axis := 0
	if b[3]-b[1] > b[2]-b[0] {
		axis = 1
	}
	slices.SortFunc(t.segs[lo:hi], func(x, y seg) int {
		cx := x.p[axis] + x.q[axis]
		cy := y.p[axis] + y.q[axis]
		switch {
		case cx < cy:
			return -1
		case cx > cy:
			return 1
		}
		return 0
	})
	mid := (lo + hi) / 2
	left := t.node(lo, mid)
	right := t.node(mid, hi)
	t.nodes[id].left, t.nodes[id].right = left, right
	return id
}

// hit is the closest pair found so far.
type hit struct {
	d2     float64
	pa, pb r3.Vec
}

type search struct {
	done   <-chan struct{}
	err    func() error
	a, b   *tree
	best   hit
	visits int
}

func (s *search) tick() error {
	s.visits++
	if s.visits%checkEvery != 0 {
		return nil
	}
	select {
	case <-s.done:
		return s.err()
	default:
		return nil
	}
}

// split picks which side of the node pair to open: the one that is not a leaf,
// the larger when neither is.
func (s *search) splitA(na, nb int) bool {
	x, y := s.a.nodes[na], s.b.nodes[nb]
	if x.left < 0 {
		return false
	}
	if y.left < 0 {
		return true
	}
	return x.box[2]-x.box[0]+x.box[3]-x.box[1] >= y.box[2]-y.box[0]+y.box[3]-y.box[1]
}

// crossing looks for a proper crossing of a segment of a with a segment of b,
// in node pairs whose boxes meet. It records the first one in s.best.
func (s *search) crossing(na, nb int) (bool, error) {
	if err := s.tick(); err != nil {
		return false, err
	}
	x, y := s.a.nodes[na], s.b.nodes[nb]
	if x.box.dist2(y.box) > 0 {
		return false, nil
	}
	if x.left < 0 && y.left < 0 {
		for _, sa := range s.a.segs[x.lo:x.hi] {
			for _, sb := range s.b.segs[y.lo:y.hi] {
				if t, u, ok := cross(sa, sb); ok {
					s.best = hit{pa: lerp(sa.p3, sa.q3, t), pb: lerp(sb.p3, sb.q3, u)}
					return true, nil
				}
			}
		}
		return false, nil
	}
	if s.splitA(na, nb) {
		for _, c := range [2]int{x.left, x.right} {
			if found, err := s.crossing(c, nb); found || err != nil {
				return found, err
			}
		}
		return false, nil
	}
	for _, c := range [2]int{y.left, y.right} {
		if found, err := s.crossing(na, c); found || err != nil {
			return found, err
		}
	}
	return false, nil
}

// nearest finds the closest pair of segments below the node pair, pruning
// every pair whose boxes lie no nearer than the best so far.
func (s *search) nearest(na, nb int) error {
	if err := s.tick(); err != nil {
		return err
	}
	x, y := s.a.nodes[na], s.b.nodes[nb]
	if x.box.dist2(y.box) >= s.best.d2 {
		return nil
	}
	if x.left < 0 && y.left < 0 {
		for _, sa := range s.a.segs[x.lo:x.hi] {
			for _, sb := range s.b.segs[y.lo:y.hi] {
				if h := segDist2(sa, sb); h.d2 < s.best.d2 {
					s.best = h
				}
			}
		}
		return nil
	}
	if s.splitA(na, nb) {
		first, second := x.left, x.right
		if s.a.nodes[second].box.dist2(y.box) < s.a.nodes[first].box.dist2(y.box) {
			first, second = second, first
		}
		if err := s.nearest(first, nb); err != nil {
			return err
		}
		return s.nearest(second, nb)
	}
	first, second := y.left, y.right
	if s.b.nodes[second].box.dist2(x.box) < s.b.nodes[first].box.dist2(x.box) {
		first, second = second, first
	}
	if err := s.nearest(na, first); err != nil {
		return err
	}
	return s.nearest(na, second)
}

func lerp(p, q r3.Vec, t float64) r3.Vec { return p.Add(q.Sub(p).Scale(t)) }

func orient(a, b, c [2]float64) float64 {
	return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
}

// cross reports whether the segments properly cross, each strictly on both
// sides of the other's line, and where along each.
func cross(a, b seg) (float64, float64, bool) {
	o1, o2 := orient(b.p, b.q, a.p), orient(b.p, b.q, a.q)
	o3, o4 := orient(a.p, a.q, b.p), orient(a.p, a.q, b.q)
	if o1 == 0 || o2 == 0 || o3 == 0 || o4 == 0 || (o1 > 0) == (o2 > 0) || (o3 > 0) == (o4 > 0) {
		return 0, 0, false
	}
	return o1 / (o1 - o2), o3 / (o3 - o4), true
}

// pointSeg returns the squared distance from c to the segment and the
// parameter along the segment of the nearest point.
func pointSeg(c, p, q [2]float64) (float64, float64) {
	dx, dy := q[0]-p[0], q[1]-p[1]
	t := 0.0
	if l := dx*dx + dy*dy; l > 0 {
		t = math.Min(1, math.Max(0, ((c[0]-p[0])*dx+(c[1]-p[1])*dy)/l))
	}
	ex, ey := p[0]+t*dx-c[0], p[1]+t*dy-c[1]
	return ex*ex + ey*ey, t
}

// segDist2 returns the squared distance between two segments, with the pair
// of points that attains it.
func segDist2(a, b seg) hit {
	if t, u, ok := cross(a, b); ok {
		return hit{pa: lerp(a.p3, a.q3, t), pb: lerp(b.p3, b.q3, u)}
	}
	best := hit{d2: math.Inf(1)}
	// An endpoint of one segment against the nearest point of the other.
	for u, c := range [2][2]float64{a.p, a.q} {
		if d2, t := pointSeg(c, b.p, b.q); d2 < best.d2 {
			best = hit{d2: d2, pa: lerp(a.p3, a.q3, float64(u)), pb: lerp(b.p3, b.q3, t)}
		}
	}
	for u, c := range [2][2]float64{b.p, b.q} {
		if d2, t := pointSeg(c, a.p, a.q); d2 < best.d2 {
			best = hit{d2: d2, pa: lerp(a.p3, a.q3, t), pb: lerp(b.p3, b.q3, float64(u))}
		}
	}
	return best
}

// inside looks for a loop of from that lies in the material of to without
// crossing it: a vertex of the loop off every segment of to, around which to's
// loops wind a positive number of times. It returns that vertex. A loop that
// crosses to was already found, so the vertex stands for the whole loop.
func inside(from, to []seg) (r3.Vec, bool) {
	for i := 0; i < len(from); {
		loop := from[i].loop
		j := i
		for j < len(from) && from[j].loop == loop {
			j++
		}
		for _, s := range from[i:j] {
			if w, on := winding(s.p, to); !on {
				if w > 0 {
					return s.p3, true
				}
				break
			}
		}
		i = j
	}
	return r3.Vec{}, false
}

// winding returns the sum of the winding numbers of the loops of segs around
// c, and whether c lies on one of the segments.
func winding(c [2]float64, segs []seg) (int, bool) {
	w := 0
	for _, s := range segs {
		o := orient(s.p, s.q, c)
		if o == 0 && c[0] >= math.Min(s.p[0], s.q[0]) && c[0] <= math.Max(s.p[0], s.q[0]) &&
			c[1] >= math.Min(s.p[1], s.q[1]) && c[1] <= math.Max(s.p[1], s.q[1]) {
			return 0, true
		}
		switch {
		case s.p[1] <= c[1] && s.q[1] > c[1] && o > 0:
			w++
		case s.p[1] > c[1] && s.q[1] <= c[1] && o < 0:
			w--
		}
	}
	return w, false
}

// Check rejects loops Clearance cannot read: an empty set, a loop of fewer
// than three points or a point that is not finite.
func Check(name string, loops [][]r3.Vec) error {
	if len(loops) == 0 {
		return fmt.Errorf("%s has no loop", name)
	}
	for _, loop := range loops {
		if len(loop) < 3 {
			return fmt.Errorf("%s has a loop of fewer than three points", name)
		}
		for _, p := range loop {
			if math.IsNaN(p.X+p.Y+p.Z) || math.IsInf(p.X+p.Y+p.Z, 0) {
				return fmt.Errorf("%s has a point that is not finite", name)
			}
		}
	}
	return nil
}
