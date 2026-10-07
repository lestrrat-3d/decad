package planar

import (
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/proof"
)

// This file is the exact 2D half of docs/multibody-dynamics-design.md §9.4:
// the plane frame a coplanar face pair is clipped in, Sutherland–Hodgman
// clipping against a convex polygon, and the exact pieces of a segment inside
// a polygon region. Every coordinate is a big.Rat; nothing here rounds.

// Point2 is an exact point of a plane frame.
type Point2 struct {
	X, Y *big.Rat
}

// Point3 is an exact point in space.
type Point3 [3]*big.Rat

// PlaneFrame maps the plane n·x = n·q to the coordinate plane obtained by
// dropping the axis K with the largest |n_K|. The map is an exact rational
// bijection, so Lift undoes Project without rounding. Dropping the largest
// component, rather than a fixed axis, keeps a vertical plane's frame
// nondegenerate.
type PlaneFrame struct {
	n       [3]*big.Rat
	offset  *big.Rat
	I, J, K int
}

// NewPlaneFrame returns the frame of the plane through q with normal n. n
// must be nonzero.
func NewPlaneFrame(n, q proof.DyV3) PlaneFrame {
	k := 0
	for axis := 1; axis < 3; axis++ {
		if proof.DyCmp(proof.DyAbs(n[axis]), proof.DyAbs(n[k])) > 0 {
			k = axis
		}
	}
	f := PlaneFrame{offset: proof.DvDot(n, q).Rat(), K: k, I: (k + 1) % 3, J: (k + 2) % 3}
	for axis := range 3 {
		f.n[axis] = n[axis].Rat()
	}
	return f
}

// Project drops the frame's axis.
func (f PlaneFrame) Project(p Point3) Point2 { return Point2{X: p[f.I], Y: p[f.J]} }

// Lift returns the point of the plane whose kept coordinates are p.
func (f PlaneFrame) Lift(p Point2) Point3 {
	var out Point3
	out[f.I], out[f.J] = p.X, p.Y
	rest := new(big.Rat).Sub(f.offset, new(big.Rat).Mul(f.n[f.I], p.X))
	rest.Sub(rest, new(big.Rat).Mul(f.n[f.J], p.Y))
	out[f.K] = rest.Quo(rest, f.n[f.K])
	return out
}

// ratPoint3 is the exact rational form of a dyadic point.
func ratPoint3(v proof.DyV3) Point3 { return Point3{v[0].Rat(), v[1].Rat(), v[2].Rat()} }

func sub2(a, b Point2) Point2 {
	return Point2{X: new(big.Rat).Sub(a.X, b.X), Y: new(big.Rat).Sub(a.Y, b.Y)}
}

func cross2(a, b Point2) *big.Rat {
	out := new(big.Rat).Mul(a.X, b.Y)
	return out.Sub(out, new(big.Rat).Mul(a.Y, b.X))
}

func dot2(a, b Point2) *big.Rat {
	out := new(big.Rat).Mul(a.X, b.X)
	return out.Add(out, new(big.Rat).Mul(a.Y, b.Y))
}

func equal2(a, b Point2) bool { return a.X.Cmp(b.X) == 0 && a.Y.Cmp(b.Y) == 0 }

// at2 returns p + t·(q − p).
func at2(p, q Point2, t *big.Rat) Point2 {
	d := sub2(q, p)
	return Point2{X: new(big.Rat).Add(p.X, new(big.Rat).Mul(t, d.X)),
		Y: new(big.Rat).Add(p.Y, new(big.Rat).Mul(t, d.Y))}
}

// DoubleArea is twice the exact signed area of a polygon: positive when it
// runs counterclockwise.
func DoubleArea(polygon []Point2) *big.Rat {
	area := new(big.Rat)
	for i, p := range polygon {
		area.Add(area, cross2(p, polygon[(i+1)%len(polygon)]))
	}
	return area
}

// orientCCW returns the polygon running counterclockwise, judged by the sign
// of its exact double area. A zero-area polygon is returned unchanged.
func orientCCW(polygon []Point2) []Point2 {
	if DoubleArea(polygon).Sign() >= 0 {
		return polygon
	}
	out := make([]Point2, len(polygon))
	for i, p := range polygon {
		out[len(polygon)-1-i] = p
	}
	return out
}

// IsConvex reports whether a polygon, either winding, turns one way or runs
// straight at every vertex.
func IsConvex(polygon []Point2) bool {
	polygon = orientCCW(polygon)
	for i, p := range polygon {
		next := polygon[(i+1)%len(polygon)]
		after := polygon[(i+2)%len(polygon)]
		if cross2(sub2(next, p), sub2(after, next)).Sign() < 0 {
			return false
		}
	}
	return true
}

// ClipConvex clips subject against the convex polygon clip with
// Sutherland–Hodgman: each clip edge in turn keeps the closed half-plane to
// its left, and every crossing vertex is the exact intersection of the two
// edge lines. Both loops are first oriented counterclockwise by the sign of
// their exact double area, so either winding gives the same output. Only the
// clip polygon must be convex; the subject may be any simple polygon, and a
// non-convex subject may yield several components joined by zero-width
// edges. Consecutive duplicate vertices are removed. poll is charged once per
// vertex visit; its error is returned unchanged.
func ClipConvex(subject, clip []Point2, poll func() error) ([]Point2, error) {
	subject, clip = orientCCW(subject), orientCCW(clip)
	out := subject
	for i, c0 := range clip {
		if len(out) == 0 {
			break
		}
		edge := sub2(clip[(i+1)%len(clip)], c0)
		in := out
		out = make([]Point2, 0, len(in)+2)
		from := in[len(in)-1]
		fromSide := cross2(edge, sub2(from, c0))
		for _, to := range in {
			if err := poll(); err != nil {
				return nil, err
			}
			toSide := cross2(edge, sub2(to, c0))
			if (fromSide.Sign() >= 0) != (toSide.Sign() >= 0) {
				// side is affine along the segment, so it vanishes at
				// t = side(from) / (side(from) − side(to)).
				t := new(big.Rat).Sub(fromSide, toSide)
				t.Quo(fromSide, t)
				out = append(out, at2(from, to, t))
			}
			if toSide.Sign() >= 0 {
				out = append(out, to)
			}
			from, fromSide = to, toSide
		}
	}
	return dedupeConsecutive(out), nil
}

func dedupeConsecutive(polygon []Point2) []Point2 {
	out := make([]Point2, 0, len(polygon))
	for _, p := range polygon {
		if len(out) == 0 || !equal2(p, out[len(out)-1]) {
			out = append(out, p)
		}
	}
	for len(out) > 1 && equal2(out[0], out[len(out)-1]) {
		out = out[:len(out)-1]
	}
	return out
}

// extremalVertices drops every vertex a clipped polygon passes straight
// through, then every exact repeat, leaving the polygon's corners. A
// zero-width bridge between components reverses direction, so its ends stay.
func extremalVertices(polygon []Point2) []Point2 {
	out := append([]Point2(nil), polygon...)
	for changed := true; changed && len(out) > 2; {
		changed = false
		for i := range out {
			prev, cur, next := out[(i+len(out)-1)%len(out)], out[i], out[(i+1)%len(out)]
			in, onward := sub2(cur, prev), sub2(next, cur)
			if cross2(in, onward).Sign() == 0 && dot2(in, onward).Sign() > 0 {
				out = append(out[:i], out[i+1:]...)
				changed = true
				break
			}
		}
	}
	return uniquePoints(out)
}

func uniquePoints(points []Point2) []Point2 {
	out := make([]Point2, 0, len(points))
	for _, p := range points {
		seen := false
		for _, q := range out {
			if equal2(p, q) {
				seen = true
				break
			}
		}
		if !seen {
			out = append(out, p)
		}
	}
	return out
}

// locate classifies x against the closed region the loops bound (the first
// loop the outer boundary, the rest open holes): 1 strictly inside, 0 on a
// loop, -1 outside. Inside is decided by the parity of a horizontal ray's
// crossings under the half-open rule.
func locate(x Point2, loops [][]Point2) int {
	crossings := 0
	for _, loop := range loops {
		for i, u := range loop {
			w := loop[(i+1)%len(loop)]
			e := sub2(w, u)
			rel := sub2(x, u)
			if cross2(e, rel).Sign() == 0 && dot2(e, rel).Sign() >= 0 && dot2(e, rel).Cmp(dot2(e, e)) <= 0 {
				return 0
			}
			if (u.Y.Cmp(x.Y) > 0) == (w.Y.Cmp(x.Y) > 0) {
				continue
			}
			// The edge's x at height x.Y lies right of x exactly when this
			// sign, normalized by the edge's vertical direction, is positive.
			side := cross2(e, rel)
			if e.Y.Sign() < 0 {
				side.Neg(side)
			}
			if side.Sign() > 0 {
				crossings++
			}
		}
	}
	if crossings%2 == 1 {
		return 1
	}
	return -1
}

// span is one component of a segment inside a closed region: the parameter
// interval [lo, hi] along the segment, a single point when lo == hi, and
// whether some point of it lies strictly inside the region.
type span struct {
	lo, hi   *big.Rat
	interior bool
}

// segmentSpans returns, in parameter order, the components of the segment
// p→q inside the closed region the loops bound. The breakpoints are the
// segment's ends and every exact meeting with a loop edge; between two
// breakpoints the segment is wholly inside or wholly outside, so its
// midpoint decides.
func segmentSpans(p, q Point2, loops [][]Point2, poll func() error) ([]span, error) {
	d := sub2(q, p)
	dd := dot2(d, d)
	if dd.Sign() == 0 {
		if locate(p, loops) < 0 {
			return nil, nil
		}
		return []span{{lo: new(big.Rat), hi: new(big.Rat)}}, nil
	}
	breaks := []*big.Rat{new(big.Rat), big.NewRat(1, 1)}
	add := func(t *big.Rat) {
		if t.Sign() >= 0 && t.Cmp(big.NewRat(1, 1)) <= 0 {
			breaks = append(breaks, t)
		}
	}
	for _, loop := range loops {
		for i, u := range loop {
			if err := poll(); err != nil {
				return nil, err
			}
			w := loop[(i+1)%len(loop)]
			e := sub2(w, u)
			rel := sub2(u, p)
			denom := cross2(d, e)
			if denom.Sign() == 0 {
				if cross2(rel, d).Sign() != 0 {
					continue
				}
				add(new(big.Rat).Quo(dot2(rel, d), dd))
				add(new(big.Rat).Quo(dot2(sub2(w, p), d), dd))
				continue
			}
			s := new(big.Rat).Quo(cross2(rel, d), denom)
			if s.Sign() < 0 || s.Cmp(big.NewRat(1, 1)) > 0 {
				continue
			}
			add(new(big.Rat).Quo(cross2(rel, e), denom))
		}
	}
	breaks = sortedUnique(breaks)
	var spans []span
	open := false
	for i, t := range breaks {
		if err := poll(); err != nil {
			return nil, err
		}
		if locate(at2(p, q, t), loops) < 0 {
			open = false
			continue
		}
		if !open {
			spans = append(spans, span{lo: t, hi: t})
			open = true
		}
		last := &spans[len(spans)-1]
		last.hi = t
		if i+1 == len(breaks) {
			break
		}
		mid := new(big.Rat).Add(t, breaks[i+1])
		mid.Quo(mid, big.NewRat(2, 1))
		switch locate(at2(p, q, mid), loops) {
		case -1:
			open = false
		case 1:
			last.interior = true
		}
	}
	return spans, nil
}

func sortedUnique(values []*big.Rat) []*big.Rat {
	sort.Slice(values, func(i, j int) bool { return values[i].Cmp(values[j]) < 0 })
	out := values[:0]
	for _, v := range values {
		if len(out) == 0 || v.Cmp(out[len(out)-1]) != 0 {
			out = append(out, v)
		}
	}
	return out
}

// segment2 is a closed segment of a plane frame.
type segment2 struct {
	a, b Point2
}

// degenerateContact is the contact set of two coplanar regions whose clip
// has zero area: their intersection then has no interior, so every point of
// it lies on the boundary of one region and inside the other. It returns the
// ends of the maximal segments and the isolated points of that set.
func degenerateContact(convex []Point2, region [][]Point2, poll func() error) ([]Point2, error) {
	var segments []segment2
	var points []Point2
	collect := func(p, q Point2, loops [][]Point2) error {
		spans, err := segmentSpans(p, q, loops, poll)
		if err != nil {
			return err
		}
		for _, s := range spans {
			a, b := at2(p, q, s.lo), at2(p, q, s.hi)
			if s.lo.Cmp(s.hi) == 0 {
				points = append(points, a)
				continue
			}
			segments = append(segments, segment2{a: a, b: b})
		}
		return nil
	}
	convexRegion := [][]Point2{convex}
	for i, p := range convex {
		if err := collect(p, convex[(i+1)%len(convex)], region); err != nil {
			return nil, err
		}
	}
	for _, loop := range region {
		for i, p := range loop {
			if err := collect(p, loop[(i+1)%len(loop)], convexRegion); err != nil {
				return nil, err
			}
		}
	}
	segments = mergeCollinear(segments)
	var out []Point2
	for _, s := range segments {
		out = append(out, s.a, s.b)
	}
	for _, p := range points {
		if !onAnySegment(p, segments) {
			out = append(out, p)
		}
	}
	return uniquePoints(out), nil
}

// mergeCollinear joins every two segments that share a line and meet.
func mergeCollinear(segments []segment2) []segment2 {
	out := append([]segment2(nil), segments...)
	for merged := true; merged; {
		merged = false
		for i := 0; i < len(out) && !merged; i++ {
			for j := i + 1; j < len(out) && !merged; j++ {
				if joined, ok := joinSegments(out[i], out[j]); ok {
					out[i] = joined
					out = append(out[:j], out[j+1:]...)
					merged = true
				}
			}
		}
	}
	return out
}

func joinSegments(s, t segment2) (segment2, bool) {
	d := sub2(s.b, s.a)
	if cross2(d, sub2(t.a, s.a)).Sign() != 0 || cross2(d, sub2(t.b, s.a)).Sign() != 0 {
		return segment2{}, false
	}
	param := func(x Point2) *big.Rat { return dot2(sub2(x, s.a), d) }
	ends := []Point2{s.a, s.b, t.a, t.b}
	sLo, sHi := param(s.a), param(s.b)
	tLo, tHi := param(t.a), param(t.b)
	if tLo.Cmp(tHi) > 0 {
		tLo, tHi = tHi, tLo
	}
	if tLo.Cmp(sHi) > 0 || tHi.Cmp(sLo) < 0 {
		return segment2{}, false
	}
	lo, hi := ends[0], ends[0]
	for _, e := range ends[1:] {
		if param(e).Cmp(param(lo)) < 0 {
			lo = e
		}
		if param(e).Cmp(param(hi)) > 0 {
			hi = e
		}
	}
	return segment2{a: lo, b: hi}, true
}

func onAnySegment(p Point2, segments []segment2) bool {
	for _, s := range segments {
		e := sub2(s.b, s.a)
		rel := sub2(p, s.a)
		if cross2(e, rel).Sign() == 0 && dot2(e, rel).Sign() >= 0 && dot2(e, rel).Cmp(dot2(e, e)) <= 0 {
			return true
		}
	}
	return false
}
