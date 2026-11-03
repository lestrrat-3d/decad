package occupancyleaf

import (
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// convexPieceLimit bounds the corners a section's polygon may have before it
// is split into convex pieces; a larger one is no leaf.
const convexPieceLimit = 64

// xpt is an exact section point.
type xpt [2]*big.Rat

func ratOf(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }

// cross is the exact orientation of a, b, c: positive for a left turn.
func cross(a, b, c xpt) int {
	ux, uy := new(big.Rat).Sub(b[0], a[0]), new(big.Rat).Sub(b[1], a[1])
	vx, vy := new(big.Rat).Sub(c[0], a[0]), new(big.Rat).Sub(c[1], a[1])
	return new(big.Rat).Sub(new(big.Rat).Mul(ux, vy), new(big.Rat).Mul(uy, vx)).Sign()
}

// lineCorners reads a loop of line segments, each walked from its point at
// TStart to its point at TEnd, as the exact corners of its polygon,
// counter-clockwise, with collinear and repeated corners dropped. Each corner
// is the end of a segment; where the next segment starts elsewhere, as the
// rounded trim parameters of an analytic union leave it, the polygon's edge
// departs from the recorded segment by at most that gap, and the largest gap
// is returned as an upper bound on the polygon's departure from the loop.
func lineCorners(loop sectionrecord.LoopRecord) ([]xpt, float64, bool) {
	segs := loop.Segments
	if len(segs) < 3 || len(segs) > convexPieceLimit {
		return nil, 0, false
	}
	at := func(l sectionrecord.LineSeg, t float64) xpt {
		rt := ratOf(t)
		x := new(big.Rat).Add(ratOf(l.Start.U), new(big.Rat).Mul(rt, new(big.Rat).Sub(ratOf(l.End.U), ratOf(l.Start.U))))
		y := new(big.Rat).Add(ratOf(l.Start.V), new(big.Rat).Mul(rt, new(big.Rat).Sub(ratOf(l.End.V), ratOf(l.Start.V))))
		return xpt{x, y}
	}
	starts := make([]xpt, len(segs))
	ends := make([]xpt, len(segs))
	for i, seg := range segs {
		l, ok := seg.(sectionrecord.LineSeg)
		if !ok || l.TStart == l.TEnd || l.TStart < 0 || l.TStart > 1 || l.TEnd < 0 || l.TEnd > 1 {
			return nil, 0, false
		}
		starts[i], ends[i] = at(l, l.TStart), at(l, l.TEnd)
	}
	// The gap at each join, |Δu| + |Δv| ≥ the Euclidean distance.
	gap := new(big.Rat)
	for i := range segs {
		next := starts[(i+1)%len(segs)]
		d := new(big.Rat).Add(new(big.Rat).Abs(new(big.Rat).Sub(next[0], ends[i][0])),
			new(big.Rat).Abs(new(big.Rat).Sub(next[1], ends[i][1])))
		if d.Cmp(gap) > 0 {
			gap = d
		}
	}
	g, _ := gap.Float64()
	if ratOf(g).Cmp(gap) < 0 {
		g = math.Nextafter(g, math.Inf(1))
	}
	pts := dropCollinear(slices.Clone(ends))
	if len(pts) < 3 {
		return nil, 0, false
	}
	area := new(big.Rat)
	for i := range pts {
		a, b := pts[i], pts[(i+1)%len(pts)]
		area.Add(area, new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0])))
	}
	switch area.Sign() {
	case 0:
		return nil, 0, false
	case -1:
		slices.Reverse(pts)
	}
	return pts, g, true
}

// dropCollinear removes every corner collinear with its neighbours, and
// every repeat, until none is left.
func dropCollinear(pts []xpt) []xpt {
	for changed := true; changed && len(pts) >= 3; {
		changed = false
		for i := range pts {
			n := len(pts)
			a, b, c := pts[(i+n-1)%n], pts[i], pts[(i+1)%n]
			if cross(a, b, c) == 0 {
				pts = slices.Delete(pts, i, i+1)
				changed = true
				break
			}
		}
	}
	return pts
}

// convex reports whether the counter-clockwise polygon turns left, or runs
// straight, at every corner.
func convex(pts []xpt) bool {
	n := len(pts)
	for i := range pts {
		if cross(pts[i], pts[(i+1)%n], pts[(i+2)%n]) < 0 {
			return false
		}
	}
	return true
}

// inClosedTriangle reports whether p lies in the closed counter-clockwise
// triangle a, b, c.
func inClosedTriangle(p, a, b, c xpt) bool {
	return cross(a, b, p) >= 0 && cross(b, c, p) >= 0 && cross(c, a, p) >= 0
}

// convexSplit splits a simple counter-clockwise polygon into convex pieces
// whose union is the polygon: ear clipping into triangles, each ear taken
// where no other corner lies in its closed triangle, then adjacent pieces
// merged while their union stays convex (Hertel and Mehlhorn). Each piece
// lists its corners counter-clockwise. A polygon no ear can be clipped from
// is not split.
func convexSplit(pts []xpt) ([][]xpt, bool) {
	if convex(pts) {
		return [][]xpt{pts}, true
	}
	idx := make([]int, len(pts))
	for i := range idx {
		idx[i] = i
	}
	var tris [][]int
	for len(idx) > 3 {
		best, bestQuality := -1, -1.0
		n := len(idx)
		for k := range idx {
			a, b, c := idx[(k+n-1)%n], idx[k], idx[(k+1)%n]
			if cross(pts[a], pts[b], pts[c]) <= 0 {
				continue
			}
			clear := true
			for _, j := range idx {
				if j == a || j == b || j == c {
					continue
				}
				if inClosedTriangle(pts[j], pts[a], pts[b], pts[c]) {
					clear = false
					break
				}
			}
			if !clear {
				continue
			}
			if q := minAngle(pts[a], pts[b], pts[c]); q > bestQuality {
				best, bestQuality = k, q
			}
		}
		if best < 0 {
			return nil, false
		}
		tris = append(tris, []int{idx[(best+n-1)%n], idx[best], idx[(best+1)%n]})
		idx = slices.Delete(idx, best, best+1)
	}
	tris = append(tris, idx)
	pieces := tris
	for merged := true; merged; {
		merged = false
	search:
		for i := range pieces {
			for j := i + 1; j < len(pieces); j++ {
				if u, ok := mergeAlongEdge(pieces[i], pieces[j]); ok {
					corners := make([]xpt, len(u))
					for k, v := range u {
						corners[k] = pts[v]
					}
					if !convex(corners) {
						continue
					}
					pieces[i] = u
					pieces = slices.Delete(pieces, j, j+1)
					merged = true
					break search
				}
			}
		}
	}
	out := make([][]xpt, len(pieces))
	for i, piece := range pieces {
		corners := make([]xpt, len(piece))
		for k, v := range piece {
			corners[k] = pts[v]
		}
		out[i] = dropCollinear(corners)
		if len(out[i]) < 3 {
			return nil, false
		}
	}
	return out, true
}

// mergeAlongEdge joins two counter-clockwise pieces that share one edge, a→b
// in p and b→a in q, into one counter-clockwise loop.
func mergeAlongEdge(p, q []int) ([]int, bool) {
	np, nq := len(p), len(q)
	for i := range p {
		a, b := p[i], p[(i+1)%np]
		for j := range q {
			if q[j] != b || q[(j+1)%nq] != a {
				continue
			}
			// p from b round to a, then q from a round to b, both exclusive of
			// the shared corners' repeats.
			var out []int
			for k := 1; k <= np; k++ {
				out = append(out, p[(i+k)%np])
			}
			for k := 2; k < nq; k++ {
				out = append(out, q[(j+k)%nq])
			}
			return out, true
		}
	}
	return nil, false
}

// minAngle is the float sine of a triangle's sharpest corner, a quality the
// ear choice prefers; it decides no geometry.
func minAngle(a, b, c xpt) float64 {
	f := func(p xpt) (float64, float64) {
		x, _ := p[0].Float64()
		y, _ := p[1].Float64()
		return x, y
	}
	ax, ay := f(a)
	bx, by := f(b)
	cx, cy := f(c)
	sine := func(px, py, qx, qy, rx, ry float64) float64 {
		ux, uy, vx, vy := qx-px, qy-py, rx-px, ry-py
		d := math.Hypot(ux, uy) * math.Hypot(vx, vy)
		if d == 0 {
			return 0
		}
		return math.Abs(ux*vy-uy*vx) / d
	}
	return min(sine(ax, ay, bx, by, cx, cy), sine(bx, by, cx, cy, ax, ay), sine(cx, cy, ax, ay, bx, by))
}

// floatCorners rounds exact corners to floats and returns, beside them, an
// upper bound on how far any rounded corner lies from its exact one.
func floatCorners(pts []xpt) ([][2]float64, float64) {
	out := make([][2]float64, len(pts))
	worst := 0.0
	for i, p := range pts {
		for k := range 2 {
			f, _ := p[k].Float64()
			out[i][k] = f
			d := new(big.Rat).Sub(ratOf(f), p[k])
			d.Abs(d)
			df, _ := d.Float64()
			worst = math.Max(worst, math.Nextafter(df, math.Inf(1)))
		}
	}
	// Each coordinate moves by at most worst, so a corner by at most √2 of it.
	return out, math.Nextafter(1.5*worst, math.Inf(1))
}
